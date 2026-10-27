#include "thermometer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "telemetry.h"

static const char* TAG = "thermometer";

int count = 0;

float thermometer_humidity = 0.0;
float thermometer_temperature = 0.0;

static WE_sensorInterface_t hids;

void thermometer_init(void) {
    // TODO optimize initialization routine
    HIDS_Get_Default_Interface(&hids);
    hids.interfaceType = WE_i2c;
    hids.handle = i2c_get_bus_handle();

    // Wait for sensor bootup
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&hids)) {
    }
    ESP_LOGI(TAG, "Initialization successful");
}

bool thermometer_read(float* temperature, float* humidity) {
    int32_t hids_temperatureRaw = 0;
    int32_t hids_humidityRaw = 0;
    hids_measureCmd_t measureCmd = HIDS_MEASURE_HPM;
    if (WE_SUCCESS == HIDS_Sensor_Measure_Raw(&hids, measureCmd,
                                              &hids_temperatureRaw,
                                              &hids_humidityRaw)) {
        *temperature = (float)hids_temperatureRaw / 1000.0;
        *humidity = (float)hids_humidityRaw / 1000.0;
        return true;
    } else {
        return false;
    }
}


static void thermometer_task(void* arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    sensor_channel_t* channel = arg;

    // Check if the channel and its config are valid
    if (channel == NULL || channel->config == NULL ||
        channel->config->name == NULL) {
        ESP_LOGE(TAG, "Invalid task argument");
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = ESP_OK;

    while (true) { // the main loop for gathering and submitting sensor data
        ESP_LOGI(TAG, "%d: Thermometer sensor task running, channel: %s", count++,
                 channel->config->name);

        bool success = thermometer_read(&thermometer_temperature, &thermometer_humidity);

        // write to queue to simulate a measurement being sent
        sensor_measurement_t measurement = {
            .timestamp_us = (uint64_t)esp_timer_get_time(),
            .data = {
                .thermometer = {
                    .temperature_c = thermometer_temperature,
                    .humidity_percent = thermometer_humidity,
                },
            },
            .flags = MEASUREMENT_VALID,
        };

        if (err != ESP_OK || !success) {
            measurement.flags = MEASUREMENT_ERROR;

            ESP_LOGE(TAG, "Thermometer read failed: %s", esp_err_to_name(err));
        }

        err = pipeline_submit_measurement(channel, &measurement, 0);

        if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Measurement dropped, total: %lu",
                     (unsigned long)channel->dropped_measurements);
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Submission failed: %s", esp_err_to_name(err));
        }

        xTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(1000)  // THIS MUST BE COMPLIANT WITH
                                // CONFIG_FREERTOS_HZ AND nominal_period_us
        );
    }
}

esp_err_t thermometer_sensor_init(void) {
    thermometer_init();

    // create a config for the example sensor channel we want to create
    static const sensor_channel_config_t example_config = {
        .id = 1,  // channel id, must be unique for each channel
        .name = "Example Thermometer Sensor",
        .measurement_type = MEASUREMENT_THERMOMETER,
        .nominal_period_us = 100000,
    };

    esp_err_t err = pipeline_create_channel(&example_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create sensor channel: %s",
                 esp_err_to_name(err));
        return err;
    }

    // now we can find the channel we just created and start a task for it
    sensor_channel_t* channel = NULL;

    err = sensor_registry_find(get_sensor_registry(), 1, &channel);

    if (err != ESP_OK || channel == NULL) {
        ESP_LOGE(TAG, "Failed to find channel: %s", esp_err_to_name(err));

        return err != ESP_OK ? err : ESP_ERR_NOT_FOUND;
    }

    // start a FreeRTOS task for the example sensor channel
    BaseType_t result = xTaskCreate(thermometer_task, TAG, 2048, channel, 5, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create example task");
    }

    return ESP_OK;
}
