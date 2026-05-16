idf_component_register(
    SRCS "Motor_config.c" "Wheel_config.c" "main.c" "motor_config.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_mcpwm esp_driver_gpio esp_driver_gptimer esp_driver_i2c esp_driver_spi nvs_flash
    REQUIRES functions data_types Initialization_functions lcd_functions 
)