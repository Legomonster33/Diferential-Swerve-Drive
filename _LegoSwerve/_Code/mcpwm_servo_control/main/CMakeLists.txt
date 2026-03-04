idf_component_register(SRCS "mcpwm_servo_control_example_main.c"
                       PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio esp_timer
                       INCLUDE_DIRS ".")
