idf_component_register(SRCS "ov2640.c" "ov2640_regs.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_i2c esp_driver_gpio)
