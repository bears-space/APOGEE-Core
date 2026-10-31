idf_component_register(
    SRCS "src/fusion.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp-dsp esp_timer telemetry
)
