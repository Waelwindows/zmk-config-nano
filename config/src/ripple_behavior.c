#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>

#include <zmk/behavior.h>
#include <zmk/keymap.h>

#include "ripple.h"

LOG_MODULE_DECLARE(led_ripple, LOG_LEVEL_INF);

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case RIPPLE_TOG:
        ripple_toggle();
        break;
    case RIPPLE_HUI:
        ripple_hue_inc();
        break;
    case RIPPLE_HUD:
        ripple_hue_dec();
        break;
    case RIPPLE_BRI:
        ripple_brt_inc();
        break;
    case RIPPLE_BRD:
        ripple_brt_dec();
        break;
    default:
        LOG_WRN("Unknown ripple command: %d", binding->param1);
        return -ENOTSUP;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api ripple_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

#define RIPPLE_INST(n)                                                         \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, NULL, NULL, POST_KERNEL,            \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,               \
                            &ripple_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RIPPLE_INST)
