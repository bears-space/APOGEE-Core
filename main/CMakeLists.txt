idf_component_register(
    SRCS
        "main.c"
        "sensor/LSM6DSV_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES vigilant_engine lsm6dsv telemetry fusion esp_timer
)
