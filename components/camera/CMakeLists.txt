idf_component_register(SRCS "camera.c"
                    INCLUDE_DIRS "include"
                    REQUIRES ov2640 esp_driver_cam esp_driver_gpio esp_driver_i2c esp_mm esp_hw_support)

# camera.c provides __wrap_esp_cache_msync (internal-SRAM DVP frame buffers
# are rejected by esp_cache_msync's cache-region range check on the S3);
# --wrap redirects every esp_cache_msync call in the final link to it.
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=esp_cache_msync")
