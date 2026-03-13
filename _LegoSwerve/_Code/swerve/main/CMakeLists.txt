idf_component_register(
    SRCS "main.c" "motor_config.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio esp_driver_gptimer
    REQUIRES functions data_types
)