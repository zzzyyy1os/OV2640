idf_component_register(SRCS "st7735s.c" "st7735s_font_ascii.c" "st7735s_font_gb16.c" "st7735s_font_gb32.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_spi esp_driver_gpio)
