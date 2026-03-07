idf_component_register(SRCS "map_speed_to_pulsewidth.c" "calculate_rpm.c" "main.c"
                       PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio esp_driver_gptimer
                       INCLUDE_DIRS ".")
