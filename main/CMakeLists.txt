idf_component_register(SRCS "main.c" "jc8012_lcd_init.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_lcd esp_driver_ppa esp_driver_jpeg esp_driver_gpio esp_hw_support esp_timer)
