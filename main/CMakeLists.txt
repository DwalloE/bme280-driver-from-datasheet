idf_component_register(
    SRCS "main.c" "bme280.c"
    INCLUDE_DIRS "."
    # bme280.c itself needs nothing from ESP-IDF - the same file is
    # compiled by test/Makefile with plain cc. Only main.c's bus binding
    # pulls in the i2c_master driver.
    PRIV_REQUIRES esp_driver_i2c log
)
