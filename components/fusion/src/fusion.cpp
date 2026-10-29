#include "fusion.h"

#include "SIMPLE_EKF.h"
#include "esp_log.h"
#include "telemetry.h"

static const char* TAG = "fusion";

void ekf_step(SimpleEKF ekf, float p, float dt) {
    // The control inputs are null in this model
    float u[3] = {0.0f, 0.0f, 0.0f};

    // Observation matrix
    dspm::Mat H(1, 3);
    H *= 0;

    H(0, 2) = 1.0f;

    // Nose vector covariance
    float R = ekf.config_.r_p / ekf.config_.Dt;

    // Prediction step
    ekf.Process(u, dt);

    float predicted[3] = {ekf.X(0, 0), ekf.X(1, 0), ekf.X(2, 0)};
    float measured[3] = {0.0f, 0.0f, p};

    // Correction step
    ekf.Update(H, measured, predicted, &R);
}

void fusionProcess(void* pvParameters) {
    sensor_channel_t* imu_channel = (sensor_channel_t*)pvParameters;
    if (imu_channel == NULL || imu_channel->config == NULL ||
        imu_channel->queue == NULL) {
        ESP_LOGE(TAG, "IMU consumer task received an invalid channel");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "[IMU Consumer] IMU channel found with ID: %d",
             imu_channel->config->id);

    SimpleEKF simple_ekf;
    simple_ekf.Init();

    uint64_t last_timestamp = 0;

    while (1) {
        sensor_measurement_t measurement;
        QueueHandle_t queue = imu_channel->queue;

        if (xQueueReceive(queue, &measurement, portMAX_DELAY) == pdPASS) {
            // log the measurement data for debugging
            ESP_LOGI(TAG,
                     "[IMU Consumer] Received measurement: Accel: [%.2f, %.2f, "
                     "%.2f], Gyro: [%.2f, %.2f, %.2f], Temp: %.2f",
                     measurement.data.imu_low_g.xAcc,
                     measurement.data.imu_low_g.yAcc,
                     measurement.data.imu_low_g.zAcc,
                     measurement.data.imu_low_g.xRate,
                     measurement.data.imu_low_g.yRate,
                     measurement.data.imu_low_g.zRate,
                     measurement.data.imu_low_g.temperature);
        }
        float dt = (measurement.timestamp_us - last_timestamp) / 1e6f;
        // FIX: Add barometer measurement
        ekf_step(simple_ekf, measurement.baro.p, dt);

        // delay task
        vTaskDelay(pdMS_TO_TICKS(1000));  // Adjust the delay as needed
    }
}

esp_err_t fusionInit(void) {
    ESP_LOGI(TAG, "Initializing fusion module...");

    sensor_channel_id_t channel =
        2;  // example for channel of the low g imu sensor
    sensor_channel_t* out_channel = NULL;
    sensor_channel_registry_t* registry = get_sensor_registry();
    esp_err_t err = sensor_registry_find(registry, channel, &out_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to find IMU channel: %s", esp_err_to_name(err));
        return err;
    }
    if (out_channel == NULL || out_channel->queue == NULL ||
        out_channel->config == NULL ||
        out_channel->config->measurement_type != MEASUREMENT_IMU_LOW_G) {
        ESP_LOGE(TAG, "Invalid low-G IMU channel");
        return ESP_ERR_INVALID_STATE;
    }

    BaseType_t result =
        xTaskCreate(fusionProcess, TAG, 2048, out_channel, 5, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create fusion task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
