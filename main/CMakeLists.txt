idf_component_register(
    SRCS "imu_low_g.c" "thermometer.c" "barometer.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES vigilant_engine wuerth_sensors_sdk_esp32 telemetry fusion
)
