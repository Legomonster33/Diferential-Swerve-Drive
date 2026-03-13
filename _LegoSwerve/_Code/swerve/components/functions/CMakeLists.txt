file(GLOB SRCS "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES data_types
    PRIV_REQUIRES esp_common esp_driver_mcpwm esp_driver_gpio esp_driver_gptimer
)