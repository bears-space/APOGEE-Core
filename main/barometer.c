#include "barometer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "telemetry.h"

static const char* TAG = "barometer";

static WE_sensorInterface_t pads;

bool barometer_init(void) {
    // TODO optimize initialization routine
    // Initialize sensor interface (i2c with PADS address, burst mode
    // deactivated)
    PADS_getDefaultInterface(&pads);
    pads.interfaceType = WE_i2c;
    pads.handle = i2c_get_bus_handle();

    // Wait for boot
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&pads)) {
    }

    WE_Delay(5);

    // First communication test
    uint8_t deviceIdValue = 0;
    if (WE_SUCCESS != PADS_getDeviceID(&pads, &deviceIdValue)) {
        ESP_LOGE(TAG, "Failed to get device ID");
        return false;
    }
    if (deviceIdValue != PADS_DEVICE_ID_VALUE) {
        ESP_LOGE(TAG, "Unexpected device ID: %d (instead of %d)", deviceIdValue,
                 PADS_DEVICE_ID_VALUE);
        return false;
    }

    // Perform soft reset of the sensor
    PADS_softReset(&pads, PADS_enable);
    PADS_state_t swReset;
    do {
        PADS_getSoftResetState(&pads, &swReset);
    } while (swReset);

    ESP_LOGI(TAG, "Initialization successful");
    return true;
}

bool barometer_read_pressure(float* pressure) {
    float pads_pressureFloat = 0;
    if (WE_SUCCESS == PADS_getPressure_float(&pads, &pads_pressureFloat)) {
        *pressure = pads_pressureFloat;
        return true;
    }
    return false;
}

bool barometer_read_temperature(float* temperature) {
    float pads_temperatureFloat = 0;
    if (WE_SUCCESS ==
        PADS_getTemperature_float(&pads, &pads_temperatureFloat)) {
        *temperature = pads_temperatureFloat;
        return true;
    }
    return false;
}

static void barometer_task(void* arg) {
    if (!barometer_init()) {
        ESP_LOGE(TAG, "Failed to initialize barometer sensor");
        vTaskDelete(NULL);
        return;
    }

    float barometer_pressure = 0.0;
    float barometer_temperature = 0.0;

    int count = 0;

    TickType_t last_wake_time = xTaskGetTickCount();
    sensor_channel_t* channel = arg;

    // Check if the channel and its config are valid
    if (channel == NULL || channel->config == NULL ||
        channel->config->name == NULL) {
        ESP_LOGE(TAG, "Invalid task argument");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "%d: Barometer sensor task running, channel: %s", count++,
        channel->config->name);
    while (true) { // the main loop for gathering and submitting sensor data

        bool success = barometer_read_pressure(&barometer_pressure) &&
                       barometer_read_temperature(&barometer_temperature);

        // write to queue to simulate a measurement being sent
        sensor_measurement_t measurement = {
            .timestamp_us = (uint64_t)esp_timer_get_time(),
            .data = {
                .barometer = {
                    .pressure_pa = barometer_pressure * 1000.0, // convert kPa to Pa
                    .temperature_c = barometer_temperature,
                },
            },
            .flags = MEASUREMENT_VALID,
        };

        if (!success) {
            measurement.flags = MEASUREMENT_ERROR;

            ESP_LOGE(TAG, "Barometer read failed");
        }

        esp_err_t err = pipeline_submit_measurement(channel, &measurement, 0);

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

esp_err_t barometer_sensor_init(void)
{    
    // create a config for the example sensor channel we want to create
    static const sensor_channel_config_t example_config = {
        .id = 3,  // channel id, must be unique for each channel
        .name = "Barometer Sensor",
        .measurement_type = MEASUREMENT_BAROMETER,
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

    err = sensor_registry_find(get_sensor_registry(), 3, &channel);

    if (err != ESP_OK || channel == NULL) {
        ESP_LOGE(TAG, "Failed to find channel: %s", esp_err_to_name(err));

        return err != ESP_OK ? err : ESP_ERR_NOT_FOUND;
    }

    // start a FreeRTOS task for the example sensor channel
    BaseType_t result = xTaskCreate(barometer_task, TAG, 2048, channel, 5, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create barometer task");
    }

    return ESP_OK;
}

