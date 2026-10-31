idf_component_register(SRCS "telemetry.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos esp_common
                       PRIV_REQUIRES log)
