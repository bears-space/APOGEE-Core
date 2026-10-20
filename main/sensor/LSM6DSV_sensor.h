#pragma once

#include "esp_err.h"
#include "telemetry.h"

/* Initialize the IMU, register channel 1, and start publishing SI measurements.
 * The caller must initialize the pipeline and drain the returned channel. */
esp_err_t lsm6dsv_sensor_init(sensor_channel_t** out_channel);
