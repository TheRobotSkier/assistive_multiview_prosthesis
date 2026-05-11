idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer nvs_flash
    PRIV_REQUIRES gy91
)
