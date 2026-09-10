#include <stdio.h>

#include "bno055.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "telemetry.h"
#include "vigilant.h"

static const char* TAG = "app_main";

esp_err_t err;

esp_err_t telem_init() {
    ESP_LOGI(TAG, "Initializing telemetry pipeline");
    err = pipeline_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize telemetry pipeline: %s",
            esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Telemetry pipeline initialized successfully");
    return ESP_OK;
}

void IMU_Gather_Task(void* pvParameters) {
    sensor_channel_t* imu_channel = (sensor_channel_t*)pvParameters;
    if (imu_channel == NULL || imu_channel->config == NULL) {
        ESP_LOGE(TAG, "IMU task received an invalid channel");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "[IMU] IMU channel found with ID: %d",
             imu_channel->config->id);
    while (1) {
        ESP_LOGI(TAG, "[IMU] IMU Task started\n");

        i2c_master_bus_handle_t bus = NULL;
        ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_NUM_0, &bus));
        i2c_master_dev_handle_t sensor = NULL;
        ESP_ERROR_CHECK(bno055_init(bus, CONFIG_BNO055_I2C_ADDRESS, &sensor));

        const TickType_t period =
            (CONFIG_BNO055_SAMPLE_PERIOD_MS * configTICK_RATE_HZ + 999) / 1000;
        TickType_t last_wake = xTaskGetTickCount();

        while (true) {
            bno055_raw_data_t raw;
            esp_err_t err = bno055_read_raw(sensor, &raw);
            if (err == ESP_OK) {
                sensor_measurement_t measurement = {
                    .sensor_id = 1,
                    .sequence = 0,  // telem should set this
                    .timestamp_us = esp_timer_get_time(),
                    .data.imu =
                        {
                            .acceleration = {raw.accel.x, raw.accel.y,
                                             raw.accel.z},
                            .angular_velocity = {0.0f, 0.0f, 0.0f},
                        },  // Populate with actual sensor data if needed
                    .flags = MEASUREMENT_VALID,
                };

                // push the measurement to the telemetry pipeline
                err = pipeline_submit_measurement(imu_channel, &measurement,
                                                  pdMS_TO_TICKS(100));
                if (err != ESP_OK) {
                    if (err == ESP_ERR_TIMEOUT) {
                        ESP_LOGW(
                            TAG, "Measurement dropped, total: %lu",
                            (unsigned long)imu_channel->dropped_measurements);
                    } else {
                        ESP_LOGE(TAG, "Failed to submit measurement: %s",
                                 esp_err_to_name(err));
                    }
                }

                // CSV: ax,ay,az,mx,my,mz,gx,gy,gz (signed register values).
                /*printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", raw.accel.x,
                   raw.accel.y, raw.accel.z, raw.mag.x, raw.mag.y, raw.mag.z,
                   raw.gyro.x, raw.gyro.y, raw.gyro.z);
                */
            } else {
                ESP_LOGE(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
            }
            if (xTaskDelayUntil(&last_wake, period) == pdFALSE) {
                // Skip missed deadlines after slow I2C/console operations
                // instead of emitting a burst of catch-up reads.
                last_wake = xTaskGetTickCount();
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
}

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    // Initialize the telemetry pipeline after all other components are set up
    err = telem_init();
    if (err != ESP_OK) {
        return;
    }

    // The registry retains this pointer after app_main() returns.
    static const sensor_channel_config_t IMU_channel_config = {
        .id = 1,
        .name = "IMU",
        .measurement_type = MEASUREMENT_IMU,
        .nominal_period_us = CONFIG_BNO055_SAMPLE_PERIOD_MS * 1000,
        .notification_bit = 1 << 0,  // Use bit 0 for IMU notifications
    };

    // Register the IMU channel with the telemetry pipeline
    err = pipeline_create_channel(&IMU_channel_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register IMU channel: %s",
                 esp_err_to_name(err));
        return;
    }

    sensor_channel_t* imu_channel = NULL;
    err = sensor_registry_find(get_sensor_registry(), IMU_channel_config.id,
                               &imu_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to find IMU channel: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "IMU channel registered successfully with ID: %d",
             imu_channel->config->id);

    // Start the IMU gather task with the registered channel.
    if (xTaskCreate(IMU_Gather_Task, "IMU_Gather", 2048, imu_channel, 1,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU gather task");
    }
}
