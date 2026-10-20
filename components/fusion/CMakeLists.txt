idf_component_register(
    SRCS "src/fusion.cpp" "src/APOGEE_EKF.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp esp_timer telemetry
)
