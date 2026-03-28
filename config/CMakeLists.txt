zephyr_include_directories(include)

if(CONFIG_LED_RIPPLE)
  zephyr_library_named(led_ripple)
  zephyr_library_sources(
    src/ripple.c
    src/ripple_behavior.c
  )
  zephyr_library_include_directories(src include)
endif()
