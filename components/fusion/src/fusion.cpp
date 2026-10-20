#include "fusion.h"

#include "APOGEE_EKF.cpp"
#include "ekf_imu13states.h"
#include "esp_dsp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "telemetry.h"

static ApogeeEKF apogee_ekf;

static const char* TAG = "fusion";

void fusionInit(void) { apogee_ekf.Init(); }

void ekf_step(float ax, float ay, float az, float dt) {
    float u[3] = {0.0f, 0.0f, 0.0f};

    // 1. Predict
    apogee_ekf.Process(u, dt);

    // 2. Measurement matrix
    dspm::Mat H(3, 3);
    H *= 0;

    H(0, 0) = 1.0f;
    H(1, 1) = 1.0f;
    H(2, 2) = 1.0f;

    // 3. Sensor measurement
    float measured[3] = {ax, ay, az};

    // 4. What the EKF currently expects to measure
    float expected[3] = {apogee_ekf.X(0, 0), apogee_ekf.X(1, 0),
                         apogee_ekf.X(2, 0)};

    float R[3] = {0.5f, 0.5f, 0.5f};

    // 5. Correct
    apogee_ekf.Update(H, measured, expected, R);
}

void fusionProcess(void* pvParameters) {
    fusionInit();

    sensor_channel_t* imu_channel = (sensor_channel_t*)pvParameters;
    if (imu_channel == NULL || imu_channel->config == NULL) {
        ESP_LOGE(TAG, "IMU consumer task received an invalid channel");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "[IMU Consumer] IMU channel found with ID: %d",
             imu_channel->config->id);

    uint64_t last_timestamp = 0;
    sensor_measurement_t measurement;
    while (xQueueReceive(imu_channel->queue, &measurement, portMAX_DELAY) ==
           pdPASS) {
        if (measurement.type != MEASUREMENT_IMU ||
            !(measurement.flags & MEASUREMENT_VALID) ||
            (measurement.flags & (MEASUREMENT_ERROR | MEASUREMENT_SATURATED))) {
            continue;
        }
        if (last_timestamp == 0) {
            last_timestamp = measurement.timestamp_us;
            continue;
        }
        if (measurement.timestamp_us <= last_timestamp) {
            continue;
        }
        const float dt = (measurement.timestamp_us - last_timestamp) / 1e6f;
        last_timestamp = measurement.timestamp_us;
        ekf_step(measurement.data.imu.acceleration[0],
                 measurement.data.imu.acceleration[1],
                 measurement.data.imu.acceleration[2], dt);
    }
    vTaskDelete(NULL);
}
