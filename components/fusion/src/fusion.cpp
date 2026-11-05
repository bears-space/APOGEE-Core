#include "fusion.h"

#include "SIMPLE_EKF.h"
#include "esp_log.h"
#include "telemetry.h"

static const float EKF_freq = 200.0f;  // Hz
static const char* TAG = "fusion";

void ekf_predict(SimpleEKF& ekf, float dt) {
    // Prediction step
    ekf.Process(nullptr, dt);
}
void ekf_correct(SimpleEKF& ekf, float dd_z, float dt) {
    // Observation matrix
    dspm::Mat H(1, 4);
    H *= 0;
    H(0, 2) = 1.0f;

    // Measurement noise vector covariance
    float R = ekf.config_.r_z / dt;

    float predicted = ekf.X(2, 0);
    float measured = dd_z;

    // Correction step
    ekf.Update(H, &measured, &predicted, &R);
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
    ESP_LOGI(TAG, "[IMU Consumer] EKF initiated.");

    uint64_t last_timestamp = 0;

    while (1) {
        sensor_measurement_t measurement;
        QueueHandle_t queue = imu_channel->queue;

        if (xQueueReceive(queue, &measurement, portMAX_DELAY) == pdPASS) {
            // log the measurement data for debugging
            float dt;
            if (last_timestamp == 0) {
                dt = 1.0f / EKF_freq;
            } else {
                dt = (measurement.timestamp_us - last_timestamp) / 1e6f;
            }
            // prevent dt from being to small or big
            if (dt <= 0.0f || dt > 1.0f) {
                dt = 1.0f / EKF_freq;
            }
            last_timestamp = measurement.timestamp_us;
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
            ekf_predict(simple_ekf, dt);
            float z_acc = (measurement.data.imu_low_g.zAcc / 1000.0f - 1.0f) *
                          9.80665f;  // Convert from mg to m/s^2
            ekf_correct(simple_ekf, z_acc, dt);
            ESP_LOGI(
                TAG,
                "[IMU Consumer] EKF state: altitude = %.2f m, vertical speed "
                "= %.2f m/s, acceleration = %.2f m/s^2, pressure = %.2f hPa.",
                simple_ekf.X(0, 0), simple_ekf.X(1, 0), simple_ekf.X(2, 0),
                simple_ekf.X(3, 0) / 100.0f);
        }
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
        xTaskCreate(fusionProcess, TAG, 8192, out_channel, 5, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create fusion task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
