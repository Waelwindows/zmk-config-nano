#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/split/bluetooth/peripheral.h>

#include "ripple.h"

LOG_MODULE_REGISTER(led_ripple, LOG_LEVEL_INF);

#define NUM_LEDS 12
#define MAX_RIPPLES 8
#define FRAME_MS (1000 / CONFIG_LED_RIPPLE_FPS)

#define HUE_STEP 10
#define BRT_STEP 10

/* --- LED strip device --- */
#define STRIP_NODE DT_CHOSEN(zmk_underglow)
static const struct device *strip_dev = DEVICE_DT_GET(STRIP_NODE);

/* --- Pixel buffer --- */
static struct led_rgb pixels[NUM_LEDS];

/* --- Physical coordinate types and LED positions --- */
struct xy { int8_t x; int8_t y; };
#define XY_SKIP INT8_MIN
#define MAX_DIST 16  /* upper bound on key-to-LED distance (coord units) */

/* Physical LED positions (x2 scale: 1 key-width = 2 units).
 * Same for both halves (mirrored physical layout, same coordinate frame).
 * x: 0=inner (split side), 10=outer.  y: 0=top, 11=bottom-inner. */
static const struct xy led_xy[NUM_LEDS] = {
    {10,  0}, {10,  2}, {10,  4}, {10,  6},  /* 0-3: outer edge top->bottom */
    { 8,  7}, { 6,  7}, { 4,  7}, { 2,  7},  /* 4-7: bottom edge outer->inner */
    { 1,  8}, { 0,  9}, {-1, 10}, {-1, 11},  /* 8-11: staggered toward thumbs */
};

/* --- Ripple state --- */
struct ripple {
    int64_t start_time;
    int8_t source_x;
    int8_t source_y;
    uint16_t hue;
    bool active;
};

static struct ripple ripples[MAX_RIPPLES];
static uint16_t hue_offset;

/* --- Held key state --- */
#define MAX_HELD_KEYS 10

struct held_key {
    uint32_t position;
    int8_t source_x;
    int8_t source_y;
    uint16_t hue;
    int64_t press_time;
    int64_t release_time;  /* 0 while held, set on release */
    bool active;
};

static struct held_key held_keys[MAX_HELD_KEYS];

/* --- Runtime parameters --- */
static uint16_t current_hue = CONFIG_LED_RIPPLE_HUE;
static uint8_t current_brt = CONFIG_LED_RIPPLE_BRT;
static bool enabled = true;

/* --- Key physical position mapping ---
 *
 * Maps global key position to physical (x, y) coordinate.
 * {XY_SKIP, XY_SKIP} means "not on this half, skip".
 *
 * The matrix is 5 rows x 12 columns (left cols 0-5, right cols 6-11).
 * Row 4 has 10 positions (6 left + 4 right). Positions are row-major:
 * pos = row * 12 + col  (rows 0-3), row 4 starts at pos 48.
 *
 * Coordinate convention (x2 scale: 1 key-width = 2 units):
 *   x: 0=inner (split side), 10=outer.  y: 0=top, 11=bottom-inner.
 *   Same for both halves after mirroring.
 */

#define S XY_SKIP  /* shorthand for skip entries */

#if IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
/* Central = right half.  cols 6-11 -> x = 0,2,4,6,8,10 (inner->outer) */
static const struct xy key_xy[] = {
    /* Row 0 (y=0): pos 0-11 */
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    { 0, 0}, { 2, 0}, { 4, 0}, { 6, 0}, { 8, 0}, {10, 0},
    /* Row 1 (y=2): pos 12-23 */
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    { 0, 2}, { 2, 2}, { 4, 2}, { 6, 2}, { 8, 2}, {10, 2},
    /* Row 2 (y=4): pos 24-35 */
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    { 0, 4}, { 2, 4}, { 4, 4}, { 6, 4}, { 8, 4}, {10, 4},
    /* Row 3 (y=6): pos 36-47 */
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    { 0, 6}, { 2, 6}, { 4, 6}, { 6, 6}, { 8, 6}, {10, 6},
    /* Row 4 (thumbs): pos 48-57  (6 left skips + 4 right thumb) */
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    {-1, 9}, { 0, 9}, { 1, 8}, { 3, 8},
    /* Encoder: pos 58 */
    { 4, 7},
};
#else
/* Peripheral = left half.  cols 0-5 -> x = 10,8,6,4,2,0 (outer->inner) */
static const struct xy key_xy[] = {
    /* Row 0 (y=0): pos 0-11 */
    {10, 0}, { 8, 0}, { 6, 0}, { 4, 0}, { 2, 0}, { 0, 0},
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    /* Row 1 (y=2): pos 12-23 */
    {10, 2}, { 8, 2}, { 6, 2}, { 4, 2}, { 2, 2}, { 0, 2},
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    /* Row 2 (y=4): pos 24-35 */
    {10, 4}, { 8, 4}, { 6, 4}, { 4, 4}, { 2, 4}, { 0, 4},
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    /* Row 3 (y=6): pos 36-47 */
    {10, 6}, { 8, 6}, { 6, 6}, { 4, 6}, { 2, 6}, { 0, 6},
    {S,S}, {S,S}, {S,S}, {S,S}, {S,S}, {S,S},
    /* Row 4 (thumbs): pos 48-57  (6 left thumb + 4 right skips) */
    { 3, 8}, { 1, 8}, { 0, 9}, {-1, 9}, {-2,10}, {-3,10},
    {S,S}, {S,S}, {S,S}, {S,S},
    /* Encoder: pos 58 */
    { 4, 7},
};
#endif

#undef S

#define POS_COUNT (sizeof(key_xy) / sizeof(key_xy[0]))

/* --- HSL hue wheel: S=100%, L=50% --- */
static const struct led_rgb hue_table[360] = {
    {.r=255, .g=  0, .b=  0},  /* h=0 */
    {.r=255, .g=  4, .b=  0},  /* h=1 */
    {.r=255, .g=  8, .b=  0},  /* h=2 */
    {.r=255, .g= 13, .b=  0},  /* h=3 */
    {.r=255, .g= 17, .b=  0},  /* h=4 */
    {.r=255, .g= 21, .b=  0},  /* h=5 */
    {.r=255, .g= 26, .b=  0},  /* h=6 */
    {.r=255, .g= 30, .b=  0},  /* h=7 */
    {.r=255, .g= 34, .b=  0},  /* h=8 */
    {.r=255, .g= 38, .b=  0},  /* h=9 */
    {.r=255, .g= 42, .b=  0},  /* h=10 */
    {.r=255, .g= 47, .b=  0},  /* h=11 */
    {.r=255, .g= 51, .b=  0},  /* h=12 */
    {.r=255, .g= 55, .b=  0},  /* h=13 */
    {.r=255, .g= 60, .b=  0},  /* h=14 */
    {.r=255, .g= 64, .b=  0},  /* h=15 */
    {.r=255, .g= 68, .b=  0},  /* h=16 */
    {.r=255, .g= 72, .b=  0},  /* h=17 */
    {.r=255, .g= 77, .b=  0},  /* h=18 */
    {.r=255, .g= 81, .b=  0},  /* h=19 */
    {.r=255, .g= 85, .b=  0},  /* h=20 */
    {.r=255, .g= 89, .b=  0},  /* h=21 */
    {.r=255, .g= 94, .b=  0},  /* h=22 */
    {.r=255, .g= 98, .b=  0},  /* h=23 */
    {.r=255, .g=102, .b=  0},  /* h=24 */
    {.r=255, .g=106, .b=  0},  /* h=25 */
    {.r=255, .g=110, .b=  0},  /* h=26 */
    {.r=255, .g=115, .b=  0},  /* h=27 */
    {.r=255, .g=119, .b=  0},  /* h=28 */
    {.r=255, .g=123, .b=  0},  /* h=29 */
    {.r=255, .g=128, .b=  0},  /* h=30 */
    {.r=255, .g=132, .b=  0},  /* h=31 */
    {.r=255, .g=136, .b=  0},  /* h=32 */
    {.r=255, .g=140, .b=  0},  /* h=33 */
    {.r=255, .g=144, .b=  0},  /* h=34 */
    {.r=255, .g=149, .b=  0},  /* h=35 */
    {.r=255, .g=153, .b=  0},  /* h=36 */
    {.r=255, .g=157, .b=  0},  /* h=37 */
    {.r=255, .g=162, .b=  0},  /* h=38 */
    {.r=255, .g=166, .b=  0},  /* h=39 */
    {.r=255, .g=170, .b=  0},  /* h=40 */
    {.r=255, .g=174, .b=  0},  /* h=41 */
    {.r=255, .g=178, .b=  0},  /* h=42 */
    {.r=255, .g=183, .b=  0},  /* h=43 */
    {.r=255, .g=187, .b=  0},  /* h=44 */
    {.r=255, .g=191, .b=  0},  /* h=45 */
    {.r=255, .g=195, .b=  0},  /* h=46 */
    {.r=255, .g=200, .b=  0},  /* h=47 */
    {.r=255, .g=204, .b=  0},  /* h=48 */
    {.r=255, .g=208, .b=  0},  /* h=49 */
    {.r=255, .g=212, .b=  0},  /* h=50 */
    {.r=255, .g=217, .b=  0},  /* h=51 */
    {.r=255, .g=221, .b=  0},  /* h=52 */
    {.r=255, .g=225, .b=  0},  /* h=53 */
    {.r=255, .g=229, .b=  0},  /* h=54 */
    {.r=255, .g=234, .b=  0},  /* h=55 */
    {.r=255, .g=238, .b=  0},  /* h=56 */
    {.r=255, .g=242, .b=  0},  /* h=57 */
    {.r=255, .g=247, .b=  0},  /* h=58 */
    {.r=255, .g=251, .b=  0},  /* h=59 */
    {.r=255, .g=255, .b=  0},  /* h=60 */
    {.r=251, .g=255, .b=  0},  /* h=61 */
    {.r=246, .g=255, .b=  0},  /* h=62 */
    {.r=242, .g=255, .b=  0},  /* h=63 */
    {.r=238, .g=255, .b=  0},  /* h=64 */
    {.r=234, .g=255, .b=  0},  /* h=65 */
    {.r=230, .g=255, .b=  0},  /* h=66 */
    {.r=225, .g=255, .b=  0},  /* h=67 */
    {.r=221, .g=255, .b=  0},  /* h=68 */
    {.r=217, .g=255, .b=  0},  /* h=69 */
    {.r=212, .g=255, .b=  0},  /* h=70 */
    {.r=208, .g=255, .b=  0},  /* h=71 */
    {.r=204, .g=255, .b=  0},  /* h=72 */
    {.r=200, .g=255, .b=  0},  /* h=73 */
    {.r=195, .g=255, .b=  0},  /* h=74 */
    {.r=191, .g=255, .b=  0},  /* h=75 */
    {.r=187, .g=255, .b=  0},  /* h=76 */
    {.r=183, .g=255, .b=  0},  /* h=77 */
    {.r=178, .g=255, .b=  0},  /* h=78 */
    {.r=174, .g=255, .b=  0},  /* h=79 */
    {.r=170, .g=255, .b=  0},  /* h=80 */
    {.r=166, .g=255, .b=  0},  /* h=81 */
    {.r=161, .g=255, .b=  0},  /* h=82 */
    {.r=157, .g=255, .b=  0},  /* h=83 */
    {.r=153, .g=255, .b=  0},  /* h=84 */
    {.r=149, .g=255, .b=  0},  /* h=85 */
    {.r=144, .g=255, .b=  0},  /* h=86 */
    {.r=140, .g=255, .b=  0},  /* h=87 */
    {.r=136, .g=255, .b=  0},  /* h=88 */
    {.r=132, .g=255, .b=  0},  /* h=89 */
    {.r=128, .g=255, .b=  0},  /* h=90 */
    {.r=123, .g=255, .b=  0},  /* h=91 */
    {.r=119, .g=255, .b=  0},  /* h=92 */
    {.r=115, .g=255, .b=  0},  /* h=93 */
    {.r=110, .g=255, .b=  0},  /* h=94 */
    {.r=106, .g=255, .b=  0},  /* h=95 */
    {.r=102, .g=255, .b=  0},  /* h=96 */
    {.r= 98, .g=255, .b=  0},  /* h=97 */
    {.r= 94, .g=255, .b=  0},  /* h=98 */
    {.r= 89, .g=255, .b=  0},  /* h=99 */
    {.r= 85, .g=255, .b=  0},  /* h=100 */
    {.r= 81, .g=255, .b=  0},  /* h=101 */
    {.r= 76, .g=255, .b=  0},  /* h=102 */
    {.r= 72, .g=255, .b=  0},  /* h=103 */
    {.r= 68, .g=255, .b=  0},  /* h=104 */
    {.r= 64, .g=255, .b=  0},  /* h=105 */
    {.r= 59, .g=255, .b=  0},  /* h=106 */
    {.r= 55, .g=255, .b=  0},  /* h=107 */
    {.r= 51, .g=255, .b=  0},  /* h=108 */
    {.r= 47, .g=255, .b=  0},  /* h=109 */
    {.r= 43, .g=255, .b=  0},  /* h=110 */
    {.r= 38, .g=255, .b=  0},  /* h=111 */
    {.r= 34, .g=255, .b=  0},  /* h=112 */
    {.r= 30, .g=255, .b=  0},  /* h=113 */
    {.r= 26, .g=255, .b=  0},  /* h=114 */
    {.r= 21, .g=255, .b=  0},  /* h=115 */
    {.r= 17, .g=255, .b=  0},  /* h=116 */
    {.r= 13, .g=255, .b=  0},  /* h=117 */
    {.r=  8, .g=255, .b=  0},  /* h=118 */
    {.r=  4, .g=255, .b=  0},  /* h=119 */
    {.r=  0, .g=255, .b=  0},  /* h=120 */
    {.r=  0, .g=255, .b=  4},  /* h=121 */
    {.r=  0, .g=255, .b=  9},  /* h=122 */
    {.r=  0, .g=255, .b= 13},  /* h=123 */
    {.r=  0, .g=255, .b= 17},  /* h=124 */
    {.r=  0, .g=255, .b= 21},  /* h=125 */
    {.r=  0, .g=255, .b= 25},  /* h=126 */
    {.r=  0, .g=255, .b= 30},  /* h=127 */
    {.r=  0, .g=255, .b= 34},  /* h=128 */
    {.r=  0, .g=255, .b= 38},  /* h=129 */
    {.r=  0, .g=255, .b= 43},  /* h=130 */
    {.r=  0, .g=255, .b= 47},  /* h=131 */
    {.r=  0, .g=255, .b= 51},  /* h=132 */
    {.r=  0, .g=255, .b= 55},  /* h=133 */
    {.r=  0, .g=255, .b= 60},  /* h=134 */
    {.r=  0, .g=255, .b= 64},  /* h=135 */
    {.r=  0, .g=255, .b= 68},  /* h=136 */
    {.r=  0, .g=255, .b= 72},  /* h=137 */
    {.r=  0, .g=255, .b= 77},  /* h=138 */
    {.r=  0, .g=255, .b= 81},  /* h=139 */
    {.r=  0, .g=255, .b= 85},  /* h=140 */
    {.r=  0, .g=255, .b= 89},  /* h=141 */
    {.r=  0, .g=255, .b= 94},  /* h=142 */
    {.r=  0, .g=255, .b= 98},  /* h=143 */
    {.r=  0, .g=255, .b=102},  /* h=144 */
    {.r=  0, .g=255, .b=106},  /* h=145 */
    {.r=  0, .g=255, .b=111},  /* h=146 */
    {.r=  0, .g=255, .b=115},  /* h=147 */
    {.r=  0, .g=255, .b=119},  /* h=148 */
    {.r=  0, .g=255, .b=123},  /* h=149 */
    {.r=  0, .g=255, .b=128},  /* h=150 */
    {.r=  0, .g=255, .b=132},  /* h=151 */
    {.r=  0, .g=255, .b=136},  /* h=152 */
    {.r=  0, .g=255, .b=140},  /* h=153 */
    {.r=  0, .g=255, .b=144},  /* h=154 */
    {.r=  0, .g=255, .b=149},  /* h=155 */
    {.r=  0, .g=255, .b=153},  /* h=156 */
    {.r=  0, .g=255, .b=157},  /* h=157 */
    {.r=  0, .g=255, .b=162},  /* h=158 */
    {.r=  0, .g=255, .b=166},  /* h=159 */
    {.r=  0, .g=255, .b=170},  /* h=160 */
    {.r=  0, .g=255, .b=174},  /* h=161 */
    {.r=  0, .g=255, .b=179},  /* h=162 */
    {.r=  0, .g=255, .b=183},  /* h=163 */
    {.r=  0, .g=255, .b=187},  /* h=164 */
    {.r=  0, .g=255, .b=191},  /* h=165 */
    {.r=  0, .g=255, .b=196},  /* h=166 */
    {.r=  0, .g=255, .b=200},  /* h=167 */
    {.r=  0, .g=255, .b=204},  /* h=168 */
    {.r=  0, .g=255, .b=208},  /* h=169 */
    {.r=  0, .g=255, .b=212},  /* h=170 */
    {.r=  0, .g=255, .b=217},  /* h=171 */
    {.r=  0, .g=255, .b=221},  /* h=172 */
    {.r=  0, .g=255, .b=225},  /* h=173 */
    {.r=  0, .g=255, .b=230},  /* h=174 */
    {.r=  0, .g=255, .b=234},  /* h=175 */
    {.r=  0, .g=255, .b=238},  /* h=176 */
    {.r=  0, .g=255, .b=242},  /* h=177 */
    {.r=  0, .g=255, .b=247},  /* h=178 */
    {.r=  0, .g=255, .b=251},  /* h=179 */
    {.r=  0, .g=255, .b=255},  /* h=180 */
    {.r=  0, .g=251, .b=255},  /* h=181 */
    {.r=  0, .g=246, .b=255},  /* h=182 */
    {.r=  0, .g=242, .b=255},  /* h=183 */
    {.r=  0, .g=238, .b=255},  /* h=184 */
    {.r=  0, .g=234, .b=255},  /* h=185 */
    {.r=  0, .g=229, .b=255},  /* h=186 */
    {.r=  0, .g=225, .b=255},  /* h=187 */
    {.r=  0, .g=221, .b=255},  /* h=188 */
    {.r=  0, .g=217, .b=255},  /* h=189 */
    {.r=  0, .g=212, .b=255},  /* h=190 */
    {.r=  0, .g=208, .b=255},  /* h=191 */
    {.r=  0, .g=204, .b=255},  /* h=192 */
    {.r=  0, .g=200, .b=255},  /* h=193 */
    {.r=  0, .g=195, .b=255},  /* h=194 */
    {.r=  0, .g=191, .b=255},  /* h=195 */
    {.r=  0, .g=187, .b=255},  /* h=196 */
    {.r=  0, .g=183, .b=255},  /* h=197 */
    {.r=  0, .g=178, .b=255},  /* h=198 */
    {.r=  0, .g=174, .b=255},  /* h=199 */
    {.r=  0, .g=170, .b=255},  /* h=200 */
    {.r=  0, .g=166, .b=255},  /* h=201 */
    {.r=  0, .g=161, .b=255},  /* h=202 */
    {.r=  0, .g=157, .b=255},  /* h=203 */
    {.r=  0, .g=153, .b=255},  /* h=204 */
    {.r=  0, .g=149, .b=255},  /* h=205 */
    {.r=  0, .g=144, .b=255},  /* h=206 */
    {.r=  0, .g=140, .b=255},  /* h=207 */
    {.r=  0, .g=136, .b=255},  /* h=208 */
    {.r=  0, .g=132, .b=255},  /* h=209 */
    {.r=  0, .g=127, .b=255},  /* h=210 */
    {.r=  0, .g=123, .b=255},  /* h=211 */
    {.r=  0, .g=119, .b=255},  /* h=212 */
    {.r=  0, .g=115, .b=255},  /* h=213 */
    {.r=  0, .g=110, .b=255},  /* h=214 */
    {.r=  0, .g=106, .b=255},  /* h=215 */
    {.r=  0, .g=102, .b=255},  /* h=216 */
    {.r=  0, .g= 98, .b=255},  /* h=217 */
    {.r=  0, .g= 94, .b=255},  /* h=218 */
    {.r=  0, .g= 89, .b=255},  /* h=219 */
    {.r=  0, .g= 85, .b=255},  /* h=220 */
    {.r=  0, .g= 81, .b=255},  /* h=221 */
    {.r=  0, .g= 76, .b=255},  /* h=222 */
    {.r=  0, .g= 72, .b=255},  /* h=223 */
    {.r=  0, .g= 68, .b=255},  /* h=224 */
    {.r=  0, .g= 64, .b=255},  /* h=225 */
    {.r=  0, .g= 59, .b=255},  /* h=226 */
    {.r=  0, .g= 55, .b=255},  /* h=227 */
    {.r=  0, .g= 51, .b=255},  /* h=228 */
    {.r=  0, .g= 47, .b=255},  /* h=229 */
    {.r=  0, .g= 43, .b=255},  /* h=230 */
    {.r=  0, .g= 38, .b=255},  /* h=231 */
    {.r=  0, .g= 34, .b=255},  /* h=232 */
    {.r=  0, .g= 30, .b=255},  /* h=233 */
    {.r=  0, .g= 25, .b=255},  /* h=234 */
    {.r=  0, .g= 21, .b=255},  /* h=235 */
    {.r=  0, .g= 17, .b=255},  /* h=236 */
    {.r=  0, .g= 13, .b=255},  /* h=237 */
    {.r=  0, .g=  8, .b=255},  /* h=238 */
    {.r=  0, .g=  4, .b=255},  /* h=239 */
    {.r=  0, .g=  0, .b=255},  /* h=240 */
    {.r=  4, .g=  0, .b=255},  /* h=241 */
    {.r=  8, .g=  0, .b=255},  /* h=242 */
    {.r= 13, .g=  0, .b=255},  /* h=243 */
    {.r= 17, .g=  0, .b=255},  /* h=244 */
    {.r= 21, .g=  0, .b=255},  /* h=245 */
    {.r= 25, .g=  0, .b=255},  /* h=246 */
    {.r= 30, .g=  0, .b=255},  /* h=247 */
    {.r= 34, .g=  0, .b=255},  /* h=248 */
    {.r= 38, .g=  0, .b=255},  /* h=249 */
    {.r= 42, .g=  0, .b=255},  /* h=250 */
    {.r= 47, .g=  0, .b=255},  /* h=251 */
    {.r= 51, .g=  0, .b=255},  /* h=252 */
    {.r= 55, .g=  0, .b=255},  /* h=253 */
    {.r= 60, .g=  0, .b=255},  /* h=254 */
    {.r= 64, .g=  0, .b=255},  /* h=255 */
    {.r= 68, .g=  0, .b=255},  /* h=256 */
    {.r= 72, .g=  0, .b=255},  /* h=257 */
    {.r= 77, .g=  0, .b=255},  /* h=258 */
    {.r= 81, .g=  0, .b=255},  /* h=259 */
    {.r= 85, .g=  0, .b=255},  /* h=260 */
    {.r= 89, .g=  0, .b=255},  /* h=261 */
    {.r= 94, .g=  0, .b=255},  /* h=262 */
    {.r= 98, .g=  0, .b=255},  /* h=263 */
    {.r=102, .g=  0, .b=255},  /* h=264 */
    {.r=106, .g=  0, .b=255},  /* h=265 */
    {.r=110, .g=  0, .b=255},  /* h=266 */
    {.r=115, .g=  0, .b=255},  /* h=267 */
    {.r=119, .g=  0, .b=255},  /* h=268 */
    {.r=123, .g=  0, .b=255},  /* h=269 */
    {.r=127, .g=  0, .b=255},  /* h=270 */
    {.r=132, .g=  0, .b=255},  /* h=271 */
    {.r=136, .g=  0, .b=255},  /* h=272 */
    {.r=140, .g=  0, .b=255},  /* h=273 */
    {.r=144, .g=  0, .b=255},  /* h=274 */
    {.r=149, .g=  0, .b=255},  /* h=275 */
    {.r=153, .g=  0, .b=255},  /* h=276 */
    {.r=157, .g=  0, .b=255},  /* h=277 */
    {.r=162, .g=  0, .b=255},  /* h=278 */
    {.r=166, .g=  0, .b=255},  /* h=279 */
    {.r=170, .g=  0, .b=255},  /* h=280 */
    {.r=174, .g=  0, .b=255},  /* h=281 */
    {.r=179, .g=  0, .b=255},  /* h=282 */
    {.r=183, .g=  0, .b=255},  /* h=283 */
    {.r=187, .g=  0, .b=255},  /* h=284 */
    {.r=191, .g=  0, .b=255},  /* h=285 */
    {.r=195, .g=  0, .b=255},  /* h=286 */
    {.r=200, .g=  0, .b=255},  /* h=287 */
    {.r=204, .g=  0, .b=255},  /* h=288 */
    {.r=208, .g=  0, .b=255},  /* h=289 */
    {.r=212, .g=  0, .b=255},  /* h=290 */
    {.r=217, .g=  0, .b=255},  /* h=291 */
    {.r=221, .g=  0, .b=255},  /* h=292 */
    {.r=225, .g=  0, .b=255},  /* h=293 */
    {.r=229, .g=  0, .b=255},  /* h=294 */
    {.r=234, .g=  0, .b=255},  /* h=295 */
    {.r=238, .g=  0, .b=255},  /* h=296 */
    {.r=242, .g=  0, .b=255},  /* h=297 */
    {.r=246, .g=  0, .b=255},  /* h=298 */
    {.r=251, .g=  0, .b=255},  /* h=299 */
    {.r=255, .g=  0, .b=255},  /* h=300 */
    {.r=255, .g=  0, .b=251},  /* h=301 */
    {.r=255, .g=  0, .b=246},  /* h=302 */
    {.r=255, .g=  0, .b=242},  /* h=303 */
    {.r=255, .g=  0, .b=238},  /* h=304 */
    {.r=255, .g=  0, .b=234},  /* h=305 */
    {.r=255, .g=  0, .b=230},  /* h=306 */
    {.r=255, .g=  0, .b=225},  /* h=307 */
    {.r=255, .g=  0, .b=221},  /* h=308 */
    {.r=255, .g=  0, .b=217},  /* h=309 */
    {.r=255, .g=  0, .b=212},  /* h=310 */
    {.r=255, .g=  0, .b=208},  /* h=311 */
    {.r=255, .g=  0, .b=204},  /* h=312 */
    {.r=255, .g=  0, .b=200},  /* h=313 */
    {.r=255, .g=  0, .b=195},  /* h=314 */
    {.r=255, .g=  0, .b=191},  /* h=315 */
    {.r=255, .g=  0, .b=187},  /* h=316 */
    {.r=255, .g=  0, .b=183},  /* h=317 */
    {.r=255, .g=  0, .b=178},  /* h=318 */
    {.r=255, .g=  0, .b=174},  /* h=319 */
    {.r=255, .g=  0, .b=170},  /* h=320 */
    {.r=255, .g=  0, .b=166},  /* h=321 */
    {.r=255, .g=  0, .b=161},  /* h=322 */
    {.r=255, .g=  0, .b=157},  /* h=323 */
    {.r=255, .g=  0, .b=153},  /* h=324 */
    {.r=255, .g=  0, .b=149},  /* h=325 */
    {.r=255, .g=  0, .b=144},  /* h=326 */
    {.r=255, .g=  0, .b=140},  /* h=327 */
    {.r=255, .g=  0, .b=136},  /* h=328 */
    {.r=255, .g=  0, .b=132},  /* h=329 */
    {.r=255, .g=  0, .b=128},  /* h=330 */
    {.r=255, .g=  0, .b=123},  /* h=331 */
    {.r=255, .g=  0, .b=119},  /* h=332 */
    {.r=255, .g=  0, .b=115},  /* h=333 */
    {.r=255, .g=  0, .b=110},  /* h=334 */
    {.r=255, .g=  0, .b=106},  /* h=335 */
    {.r=255, .g=  0, .b=102},  /* h=336 */
    {.r=255, .g=  0, .b= 98},  /* h=337 */
    {.r=255, .g=  0, .b= 93},  /* h=338 */
    {.r=255, .g=  0, .b= 89},  /* h=339 */
    {.r=255, .g=  0, .b= 85},  /* h=340 */
    {.r=255, .g=  0, .b= 81},  /* h=341 */
    {.r=255, .g=  0, .b= 76},  /* h=342 */
    {.r=255, .g=  0, .b= 72},  /* h=343 */
    {.r=255, .g=  0, .b= 68},  /* h=344 */
    {.r=255, .g=  0, .b= 64},  /* h=345 */
    {.r=255, .g=  0, .b= 59},  /* h=346 */
    {.r=255, .g=  0, .b= 55},  /* h=347 */
    {.r=255, .g=  0, .b= 51},  /* h=348 */
    {.r=255, .g=  0, .b= 47},  /* h=349 */
    {.r=255, .g=  0, .b= 43},  /* h=350 */
    {.r=255, .g=  0, .b= 38},  /* h=351 */
    {.r=255, .g=  0, .b= 34},  /* h=352 */
    {.r=255, .g=  0, .b= 30},  /* h=353 */
    {.r=255, .g=  0, .b= 26},  /* h=354 */
    {.r=255, .g=  0, .b= 21},  /* h=355 */
    {.r=255, .g=  0, .b= 17},  /* h=356 */
    {.r=255, .g=  0, .b= 13},  /* h=357 */
    {.r=255, .g=  0, .b=  8},  /* h=358 */
    {.r=255, .g=  0, .b=  4},  /* h=359 */
};

static struct led_rgb hue_to_rgb(uint16_t hue, uint8_t brt) {
    struct led_rgb c = hue_table[hue % 360];
    return (struct led_rgb){
        .r = (uint8_t)((uint16_t)c.r * brt / 255),
        .g = (uint8_t)((uint16_t)c.g * brt / 255),
        .b = (uint8_t)((uint16_t)c.b * brt / 255),
    };
}

/* --- Octagonal distance approximation (within ~4% of Euclidean) --- */
static int approx_dist(int x1, int y1, int x2, int y2) {
    int dx = x1 - x2;
    int dy = y1 - y2;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    if (dx < dy) { int t = dx; dx = dy; dy = t; }
    return dx + (dy * 3 + 4) / 8;
}

/* --- Add a new ripple with a specific hue --- */
static void add_ripple_at(int8_t x, int8_t y, uint16_t hue) {
    if (!enabled) {
        return;
    }

    /* Find a free slot, or overwrite the oldest */
    int oldest = 0;
    int64_t oldest_time = INT64_MAX;

    for (int i = 0; i < MAX_RIPPLES; i++) {
        if (!ripples[i].active) {
            ripples[i].active = true;
            ripples[i].source_x = x;
            ripples[i].source_y = y;
            ripples[i].hue = hue;
            ripples[i].start_time = k_uptime_get();
            return;
        }
        if (ripples[i].start_time < oldest_time) {
            oldest_time = ripples[i].start_time;
            oldest = i;
        }
    }

    /* All slots full -- recycle oldest */
    ripples[oldest].active = true;
    ripples[oldest].source_x = x;
    ripples[oldest].source_y = y;
    ripples[oldest].hue = hue;
    ripples[oldest].start_time = k_uptime_get();
}

/* --- Held key tracking --- */
static void add_held_key(uint32_t position, int8_t x, int8_t y, uint16_t hue) {
    int oldest = 0;
    int64_t oldest_time = INT64_MAX;

    for (int i = 0; i < MAX_HELD_KEYS; i++) {
        if (!held_keys[i].active) {
            held_keys[i].active = true;
            held_keys[i].position = position;
            held_keys[i].source_x = x;
            held_keys[i].source_y = y;
            held_keys[i].hue = hue;
            held_keys[i].press_time = k_uptime_get();
            held_keys[i].release_time = 0;
            return;
        }
        if (held_keys[i].press_time < oldest_time) {
            oldest_time = held_keys[i].press_time;
            oldest = i;
        }
    }

    /* All slots full -- recycle oldest */
    held_keys[oldest].active = true;
    held_keys[oldest].position = position;
    held_keys[oldest].source_x = x;
    held_keys[oldest].source_y = y;
    held_keys[oldest].hue = hue;
    held_keys[oldest].press_time = k_uptime_get();
    held_keys[oldest].release_time = 0;
}

static void release_held_key(uint32_t position) {
    for (int i = 0; i < MAX_HELD_KEYS; i++) {
        if (held_keys[i].active && held_keys[i].release_time == 0 &&
            held_keys[i].position == position) {
            held_keys[i].release_time = k_uptime_get();
            return;
        }
    }
}

/* --- Animation tick --- */
static void ripple_tick(struct k_work *work);
static void ripple_timer_handler(struct k_timer *timer);

K_WORK_DEFINE(ripple_work, ripple_tick);
K_TIMER_DEFINE(ripple_timer, ripple_timer_handler, NULL);

static void ripple_timer_handler(struct k_timer *timer) {
    k_work_submit(&ripple_work);
}

static void ripple_tick(struct k_work *work) {
    if (!enabled) {
        /* Turn off all LEDs */
        memset(pixels, 0, sizeof(pixels));
        led_strip_update_rgb(strip_dev, pixels, NUM_LEDS);
        return;
    }

    int64_t now = k_uptime_get();
    uint8_t base_brt = (uint8_t)((uint16_t)CONFIG_LED_RIPPLE_BASE_BRT * 255 / 100);
    uint8_t max_brt = (uint8_t)((uint16_t)current_brt * 255 / 100);

    for (int i = 0; i < NUM_LEDS; i++) {
        int best_fade = 0;
        uint16_t best_hue = current_hue;

        for (int r = 0; r < MAX_RIPPLES; r++) {
            if (!ripples[r].active) {
                continue;
            }

            int dist = approx_dist(led_xy[i].x, led_xy[i].y,
                                    ripples[r].source_x, ripples[r].source_y);

            int64_t elapsed = now - ripples[r].start_time;
            int wave_radius = (int)(elapsed / CONFIG_LED_RIPPLE_SPEED);

            if (dist <= wave_radius) {
                int64_t arrival_time = (int64_t)dist * CONFIG_LED_RIPPLE_SPEED;
                int64_t time_since_arrival = elapsed - arrival_time;

                if (time_since_arrival < CONFIG_LED_RIPPLE_DECAY_MS) {
                    int fade = max_brt - (int)(time_since_arrival * max_brt /
                                               CONFIG_LED_RIPPLE_DECAY_MS);
                    if (fade < 0) {
                        fade = 0;
                    }
                    if (fade > best_fade) {
                        best_fade = fade;
                        best_hue = ripples[r].hue;
                    }
                }
            }
        }

        /* --- Held key layers (LIFO stack with crossfade) --- */

        /* Base layer: newest sustaining (not yet released) key */
        struct led_rgb base_held_rgb = {.r = 0, .g = 0, .b = 0};
        int64_t base_held_time = -1;
        bool has_base_held = false;

        for (int h = 0; h < MAX_HELD_KEYS; h++) {
            if (!held_keys[h].active || held_keys[h].release_time != 0) {
                continue;
            }
            int dist = approx_dist(led_xy[i].x, led_xy[i].y,
                                    held_keys[h].source_x, held_keys[h].source_y);
            int64_t elapsed = now - held_keys[h].press_time;
            int wr = (int)(elapsed / CONFIG_LED_RIPPLE_SPEED);
            if (dist > wr) {
                continue;
            }
            if (held_keys[h].press_time > base_held_time) {
                base_held_time = held_keys[h].press_time;
                base_held_rgb = hue_to_rgb(held_keys[h].hue, max_brt);
                has_base_held = true;
            }
        }

        /* Overlay: newest releasing key that was on top of the base */
        struct led_rgb overlay_rgb = {.r = 0, .g = 0, .b = 0};
        int overlay_alpha = 0;
        int64_t overlay_pt = -1;

        for (int h = 0; h < MAX_HELD_KEYS; h++) {
            if (!held_keys[h].active || held_keys[h].release_time == 0) {
                continue;
            }
            if (held_keys[h].press_time <= base_held_time) {
                continue;
            }
            if (held_keys[h].press_time <= overlay_pt) {
                continue;
            }

            int dist = approx_dist(led_xy[i].x, led_xy[i].y,
                                    held_keys[h].source_x,
                                    held_keys[h].source_y);
            int64_t since_rel = now - held_keys[h].release_time;
            int wr = (int)(since_rel / CONFIG_LED_RIPPLE_SPEED);

            int alpha;
            if (dist > wr) {
                /* Release wave hasn't reached — still fully visible */
                alpha = 255;
            } else {
                int64_t arrival = (int64_t)dist * CONFIG_LED_RIPPLE_SPEED;
                int64_t since_arr = since_rel - arrival;
                if (since_arr >= CONFIG_LED_RIPPLE_DECAY_MS) {
                    alpha = 0;
                } else {
                    alpha = 255 - (int)(since_arr * 255 /
                                        CONFIG_LED_RIPPLE_DECAY_MS);
                    if (alpha < 0) { alpha = 0; }
                }
            }

            if (alpha > 0) {
                overlay_pt = held_keys[h].press_time;
                overlay_alpha = alpha;
                overlay_rgb = hue_to_rgb(held_keys[h].hue, max_brt);
            }
        }

        /* Composite held layers */
        struct led_rgb held_rgb = {.r = 0, .g = 0, .b = 0};
        bool has_held = false;

        if (overlay_alpha > 0) {
            uint8_t oa = (uint8_t)overlay_alpha;
            uint8_t inv = 255 - oa;
            held_rgb.r = (uint8_t)((uint16_t)overlay_rgb.r * oa / 255 +
                                   (uint16_t)base_held_rgb.r * inv / 255);
            held_rgb.g = (uint8_t)((uint16_t)overlay_rgb.g * oa / 255 +
                                   (uint16_t)base_held_rgb.g * inv / 255);
            held_rgb.b = (uint8_t)((uint16_t)overlay_rgb.b * oa / 255 +
                                   (uint16_t)base_held_rgb.b * inv / 255);
            has_held = true;
        } else if (has_base_held) {
            held_rgb = base_held_rgb;
            has_held = true;
        }

        /* --- Final pixel: blend ripple over held over base --- */
        if (best_fade > 0 && has_held) {
            /* Ripple fading on top of held — crossfade */
            struct led_rgb rip_rgb = hue_to_rgb(best_hue, max_brt);
            uint8_t ra = (max_brt > 0)
                ? (uint8_t)((uint16_t)best_fade * 255 / max_brt)
                : 0;
            uint8_t inv = 255 - ra;
            pixels[i].r = (uint8_t)((uint16_t)rip_rgb.r * ra / 255 +
                                    (uint16_t)held_rgb.r * inv / 255);
            pixels[i].g = (uint8_t)((uint16_t)rip_rgb.g * ra / 255 +
                                    (uint16_t)held_rgb.g * inv / 255);
            pixels[i].b = (uint8_t)((uint16_t)rip_rgb.b * ra / 255 +
                                    (uint16_t)held_rgb.b * inv / 255);
        } else if (has_held) {
            pixels[i] = held_rgb;
        } else {
            uint16_t brightness = base_brt + (uint16_t)best_fade;
            if (brightness > 255) { brightness = 255; }
            pixels[i] = hue_to_rgb(best_hue, (uint8_t)brightness);
        }
    }

    /* Expire dead ripples */
    int64_t max_lifetime = (int64_t)MAX_DIST * CONFIG_LED_RIPPLE_SPEED +
                           CONFIG_LED_RIPPLE_DECAY_MS;
    for (int r = 0; r < MAX_RIPPLES; r++) {
        if (!ripples[r].active) {
            continue;
        }
        if (now - ripples[r].start_time > max_lifetime) {
            ripples[r].active = false;
        }
    }

    /* Expire fully-faded releasing held keys */
    for (int h = 0; h < MAX_HELD_KEYS; h++) {
        if (!held_keys[h].active || held_keys[h].release_time == 0) {
            continue;
        }
        if (now - held_keys[h].release_time > max_lifetime) {
            held_keys[h].active = false;
        }
    }

    led_strip_update_rgb(strip_dev, pixels, NUM_LEDS);
}

/* --- Key event listener --- */
static int on_position_state_changed(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    uint32_t position = ev->position;

    if (position >= POS_COUNT) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    struct xy pos = key_xy[position];
    if (pos.x == XY_SKIP) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->state) {
        /* Key press: ripple + sustain */
        uint16_t hue = (current_hue + hue_offset) % 360;
        hue_offset = (hue_offset + CONFIG_LED_RIPPLE_HUE_STEP) % 360;
        add_ripple_at(pos.x, pos.y, hue);
        add_held_key(position, pos.x, pos.y, hue);
    } else {
        /* Key release: transition to next held layer via crossfade */
        release_held_key(position);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(led_ripple, on_position_state_changed);
ZMK_SUBSCRIPTION(led_ripple, zmk_position_state_changed);

/* --- Runtime control functions --- */
void ripple_toggle(void) {
    enabled = !enabled;
    if (!enabled) {
        /* Clear all ripples, held keys, and LEDs */
        for (int i = 0; i < MAX_RIPPLES; i++) {
            ripples[i].active = false;
        }
        for (int i = 0; i < MAX_HELD_KEYS; i++) {
            held_keys[i].active = false;
        }
        memset(pixels, 0, sizeof(pixels));
        led_strip_update_rgb(strip_dev, pixels, NUM_LEDS);
    }
    LOG_INF("Ripple %s", enabled ? "enabled" : "disabled");
}

void ripple_hue_inc(void) {
    current_hue = (current_hue + HUE_STEP) % 360;
    LOG_INF("Hue: %d", current_hue);
}

void ripple_hue_dec(void) {
    current_hue = (current_hue + 360 - HUE_STEP) % 360;
    LOG_INF("Hue: %d", current_hue);
}

void ripple_brt_inc(void) {
    if (current_brt <= 90) {
        current_brt += BRT_STEP;
    } else {
        current_brt = 100;
    }
    LOG_INF("Brightness: %d%%", current_brt);
}

void ripple_brt_dec(void) {
    if (current_brt >= BRT_STEP) {
        current_brt -= BRT_STEP;
    } else {
        current_brt = 0;
    }
    LOG_INF("Brightness: %d%%", current_brt);
}

/* --- Initialization --- */
static int ripple_init(void) {
    if (!device_is_ready(strip_dev)) {
        LOG_ERR("LED strip device not ready");
        return -ENODEV;
    }

    for (int i = 0; i < MAX_RIPPLES; i++) {
        ripples[i].active = false;
    }
    for (int i = 0; i < MAX_HELD_KEYS; i++) {
        held_keys[i].active = false;
    }

    memset(pixels, 0, sizeof(pixels));

    LOG_INF("LED ripple initialized (hue=%d, brt=%d)", current_hue, current_brt);

    k_timer_start(&ripple_timer, K_MSEC(FRAME_MS), K_MSEC(FRAME_MS));

    return 0;
}

SYS_INIT(ripple_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
