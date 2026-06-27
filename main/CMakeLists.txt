idf_component_register(
    SRCS "tusb_mouse_ethernet.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio Ethernet
)
