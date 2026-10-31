#include "imu_low_g.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "telemetry.h"

static const char* TAG = "imu_low_g";

static WE_sensorInterface_t isds;

bool imu_low_g_init(void) {
    // TODO optimize initialization routine
    // Initialize sensor interface (i2c with ISDS address, burst mode activated)
    ISDS_getDefaultInterface(&isds);
    isds.interfaceType = WE_i2c;
    isds.options.i2c.burstMode = 1;
    isds.options.i2c.address =
        ISDS_ADDRESS_I2C_0;  // is on 6A instead of 6B, maybe change back when
                             // new PCB arrives
    isds.handle = i2c_get_bus_handle();

    // Wait for sensor bootup
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&isds)) {
    }

    WE_Delay(5);

    // First communication test
    uint8_t deviceIdValue = 0;
    if (WE_SUCCESS != ISDS_getDeviceID(&isds, &deviceIdValue)) {
        ESP_LOGE(TAG, "Failed to get device ID");
        return false;
    }
    if (deviceIdValue != ISDS_DEVICE_ID_VALUE) {
        ESP_LOGE(TAG, "Unexpected device ID: %d (instead of %d)", deviceIdValue,
                 ISDS_DEVICE_ID_VALUE);
        return false;
    }

    // Perform soft reset of the sensor
    ISDS_softReset(&isds, ISDS_enable);
    ISDS_state_t swReset;
    do {
        ISDS_getSoftResetState(&isds, &swReset);
    } while (swReset);

    // Perform reboot (retrieve trimming parameters from nonvolatile memory)
    ISDS_reboot(&isds, ISDS_enable);
    WE_Delay(15);

    // Enable block data update
    ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

    // Sampling rate (104 Hz)
    ISDS_setAccOutputDataRate(&isds, ISDS_accOdr104Hz);
    ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr104Hz);

    // Accelerometer 16g range
    ISDS_setAccFullScale(&isds, ISDS_accFullScaleSixteenG);

    // Gyroscope 2000 dps range
    ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);

    // Disable the disabling of high performance mode
    ISDS_disableAccHighPerformanceMode(&isds, ISDS_disable);
    ISDS_disableGyroHighPerformanceMode(&isds, ISDS_disable);

    ESP_LOGI(TAG, "Initialization successful");

    return true;
}

bool imu_low_g_read(float* xAcc, float* yAcc, float* zAcc, float* xRate,
                    float* yRate, float* zRate, float* temperature) {
    ISDS_state_t dataReady;
    float isds_xAcc = 0;
    float isds_yAcc = 0;
    float isds_zAcc = 0;
    float isds_xRate = 0;
    float isds_yRate = 0;
    float isds_zRate = 0;
    float isds_temperature = 0;

    // Wait until the acceleration values are ready to read
    do {
        ISDS_isAccelerationDataReady(&isds, &dataReady);
    } while (dataReady == ISDS_disable);

    // Read acceleration values
    if (ISDS_getAccelerations_float(&isds, &isds_xAcc, &isds_yAcc,
                                    &isds_zAcc) != WE_SUCCESS) {
        return false;
    }

    // Wait until the gyroscope values are ready to read
    do {
        ISDS_isGyroscopeDataReady(&isds, &dataReady);
    } while (dataReady == ISDS_disable);

    // Read gyroscope values
    if (ISDS_getAngularRates_float(&isds, &isds_xRate, &isds_yRate,
                                   &isds_zRate) != WE_SUCCESS) {
        return false;
    }

    // Wait until the temperature value is ready to read
    do {
        ISDS_isTemperatureDataReady(&isds, &dataReady);
    } while (dataReady == ISDS_disable);
    if (ISDS_getTemperature_float(&isds, &isds_temperature) != WE_SUCCESS) {
        return false;
    }

    *xAcc = isds_xAcc;
    *yAcc = isds_yAcc;
    *zAcc = isds_zAcc;
    *xRate = isds_xRate;
    *yRate = isds_yRate;
    *zRate = isds_zRate;
    *temperature = isds_temperature;
    return true;
}

static void imu_low_g_task(void* arg) {
    if (!imu_low_g_init()) {
        ESP_LOGE(TAG, "Failed to initialize IMU");
        vTaskDelete(NULL);
        return;
    }

    int count = 0;

    float imu_low_g_xAcc = 0;
    float imu_low_g_yAcc = 0;
    float imu_low_g_zAcc = 0;
    float imu_low_g_xRate = 0;
    float imu_low_g_yRate = 0;
    float imu_low_g_zRate = 0;
    float imu_low_g_temperature = 0;

    TickType_t last_wake_time = xTaskGetTickCount();
    sensor_channel_t* channel = arg;

    // Check if the channel and its config are valid
    if (channel == NULL || channel->config == NULL ||
        channel->config->name == NULL) {
        ESP_LOGE(TAG, "Invalid task argument");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "%d: LOW-G IMU sensor task running, channel: %s", count++,
        channel->config->name);
    while (true) { // the main loop for gathering and submitting sensor data

        bool success = imu_low_g_read(&imu_low_g_xAcc, &imu_low_g_yAcc, &imu_low_g_zAcc,
                       &imu_low_g_xRate, &imu_low_g_yRate, &imu_low_g_zRate,
                       &imu_low_g_temperature);

        // write to queue to simulate a measurement being sent
        sensor_measurement_t measurement = {
            .timestamp_us = (uint64_t)esp_timer_get_time(),
            .data = {
                .imu_low_g = {
                    .xAcc = imu_low_g_xAcc,
                    .yAcc = imu_low_g_yAcc,
                    .zAcc = imu_low_g_zAcc,
                    .xRate = imu_low_g_xRate,
                    .yRate = imu_low_g_yRate,
                    .zRate = imu_low_g_zRate,
                    .temperature = imu_low_g_temperature,
                },
            },
            .flags = MEASUREMENT_VALID,
        };

        if (!success) {
            measurement.flags = MEASUREMENT_ERROR;

            ESP_LOGE(TAG, "IMU read failed");
        }

        if((uxQueueSpacesAvailable(channel->queue) == 0)) {
            // remove the oldest measurement from the queue to make space for the new one
            sensor_measurement_t dropped_measurement;
            if (xQueueReceive(channel->queue, &dropped_measurement, 0) == pdPASS) {
                channel->dropped_measurements++;
                ESP_LOGW(TAG, "Dropped oldest measurement, total dropped: %lu",
                         (unsigned long)channel->dropped_measurements);
            }
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
            pdMS_TO_TICKS(channel->config->nominal_period_us / 1000)
        );
    }
}

esp_err_t imu_low_g_sensor_init(void) {
    // create a config for the example sensor channel we want to create
    static const sensor_channel_config_t example_config = {
        .id = 2,  // channel id, must be unique for each channel
        .name = "Example IMU Low-G Sensor",
        .measurement_type = MEASUREMENT_IMU_LOW_G,
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

    err = sensor_registry_find(get_sensor_registry(), example_config.id, &channel);

    if (err != ESP_OK || channel == NULL) {
        ESP_LOGE(TAG, "Failed to find channel: %s", esp_err_to_name(err));

        return err != ESP_OK ? err : ESP_ERR_NOT_FOUND;
    }

    // start a FreeRTOS task for the example sensor channel
    BaseType_t result = xTaskCreate(imu_low_g_task, TAG, 2048, channel, 5, NULL);

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create IMU task");
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
