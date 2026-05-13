file(GLOB SRCS "i2c_slave_init.c" "init_capture_timer.c" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES data_types main functions
    PRIV_REQUIRES esp_common esp_driver_mcpwm esp_driver_gpio esp_driver_gptimer esp_driver_i2c
)