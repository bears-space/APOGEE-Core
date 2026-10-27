idf_component_register(
<<<<<<< HEAD
    SRCS
        "main.c"
=======
    SRCS "imu_low_g.c" "thermometer.c" "barometer.c" "main.c"
>>>>>>> origin/niklas/wuerth-fixes
    INCLUDE_DIRS "."
    REQUIRES vigilant_engine wuerth_sensors_sdk_esp32
)
