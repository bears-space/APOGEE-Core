#include "LSM6DSV_sensor.h"

#include <limits.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "lsm6dsv.h"

static const char* TAG = "LSM6DSV_sensor";
static float_t (*accel_to_mg)(int16_t);
static float_t (*gyro_to_mdps)(int16_t);
static TaskHandle_t sensor_task;
static esp_timer_handle_t sample_timer;
static sensor_channel_config_t channel_config = {
    .id = 1,
    .name = "LSM6DSV320X IMU",
    .measurement_type = MEASUREMENT_IMU,
};

/* Read back the hardware configuration so telemetry follows menuconfig ranges
 * and ODR, rather than assuming the defaults. */
static esp_err_t configure_sampling(void) {
    const stmdev_ctx_t* ctx = ve_lsm6dsv_get_ctx();
    lsm6dsv320x_xl_full_scale_t accel_fs;
    lsm6dsv320x_gy_full_scale_t gyro_fs;
    lsm6dsv320x_data_rate_t accel_odr, gyro_odr;
    if (lsm6dsv320x_xl_full_scale_get(ctx, &accel_fs) != 0 ||
        lsm6dsv320x_gy_full_scale_get(ctx, &gyro_fs) != 0 ||
        lsm6dsv320x_xl_data_rate_get(ctx, &accel_odr) != 0 ||
        lsm6dsv320x_gy_data_rate_get(ctx, &gyro_odr) != 0) {
        return ESP_FAIL;
    }
    switch (accel_fs) {
        case LSM6DSV320X_2g:
            accel_to_mg = lsm6dsv320x_from_fs2_to_mg;
            break;
        case LSM6DSV320X_4g:
            accel_to_mg = lsm6dsv320x_from_fs4_to_mg;
            break;
        case LSM6DSV320X_8g:
            accel_to_mg = lsm6dsv320x_from_fs8_to_mg;
            break;
        case LSM6DSV320X_16g:
            accel_to_mg = lsm6dsv320x_from_fs16_to_mg;
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
    switch (gyro_fs) {
        case LSM6DSV320X_250dps:
            gyro_to_mdps = lsm6dsv320x_from_fs250_to_mdps;
            break;
        case LSM6DSV320X_500dps:
            gyro_to_mdps = lsm6dsv320x_from_fs500_to_mdps;
            break;
        case LSM6DSV320X_1000dps:
            gyro_to_mdps = lsm6dsv320x_from_fs1000_to_mdps;
            break;
        case LSM6DSV320X_2000dps:
            gyro_to_mdps = lsm6dsv320x_from_fs2000_to_mdps;
            break;
        case LSM6DSV320X_4000dps:
            gyro_to_mdps = lsm6dsv320x_from_fs4000_to_mdps;
            break;
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
    /* Standard ODRs exposed by this component's Kconfig are powers of two
     * starting at 15 Hz. Reject disabled/mismatched channels explicitly. */
    if (accel_odr != gyro_odr || gyro_odr < LSM6DSV320X_ODR_AT_15Hz ||
        gyro_odr > LSM6DSV320X_ODR_AT_7680Hz) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    const uint32_t hz = 15U << (gyro_odr - LSM6DSV320X_ODR_AT_15Hz);
    channel_config.nominal_period_us = (1000000U + hz / 2U) / hz;
    return ESP_OK;
}

static esp_err_t read_measurement(sensor_measurement_t* measurement) {
    bool accel_ready = false, gyro_ready = false;
    esp_err_t err =
        ve_lsm6dsv_data_ready(NULL, &accel_ready, &gyro_ready, NULL);
    if (err != ESP_OK) {
        return err;
    }
    if (!accel_ready || !gyro_ready) {
        return ESP_ERR_NOT_FOUND;
    }

    int16_t accel[3], gyro[3];
    /* This is the host read time, not a hardware sample timestamp. */
    const uint64_t timestamp_us = (uint64_t)esp_timer_get_time();
    err = ve_lsm6dsv_read_accel_raw(accel);
    if (err != ESP_OK) {
        return err;
    }
    err = ve_lsm6dsv_read_gyro_raw(gyro);
    if (err != ESP_OK) {
        return err;
    }

    *measurement = (sensor_measurement_t){
        .timestamp_us = timestamp_us,
        .flags = MEASUREMENT_VALID,
    };
    for (int axis = 0; axis < 3; ++axis) {
        measurement->data.imu.acceleration[axis] =
            accel_to_mg(accel[axis]) * 0.00980665f;
        measurement->data.imu.angular_velocity[axis] =
            gyro_to_mdps(gyro[axis]) * 0.00001745329252f;
        if (accel[axis] == INT16_MIN || accel[axis] == INT16_MAX ||
            gyro[axis] == INT16_MIN || gyro[axis] == INT16_MAX) {
            measurement->flags |= MEASUREMENT_SATURATED;
        }
    }
    return ESP_OK;
}

/* ESP_TIMER_TASK dispatch: no I2C or logging in the timer callback. */
static void sample_tick(void* arg) { xTaskNotifyGive(sensor_task); }

static void lsm6dsv_task(void* arg) {
    sensor_channel_t* channel = arg;
    int64_t last_report_us = 0;
    while (true) {
        /* Coalesce overdue wakeups: never manufacture catch-up samples. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        sensor_measurement_t measurement;
        esp_err_t err = read_measurement(&measurement);
        if (err == ESP_ERR_NOT_FOUND) {
            continue;
        }
        if (err == ESP_OK) {
            err = pipeline_submit_measurement(channel, &measurement, 0);
        }
        const int64_t now = esp_timer_get_time();
        if (err != ESP_OK && now - last_report_us >= 1000000) {
            ESP_LOGW(TAG, "Acquisition/submission failed: %s; queue drops=%lu",
                     esp_err_to_name(err),
                     (unsigned long)channel->dropped_measurements);
            last_report_us = now;
        }
    }
}

esp_err_t lsm6dsv_sensor_init(sensor_channel_t** out_channel) {
    bool channel_created = false;
    if (out_channel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_channel = NULL;
    if (sensor_task != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = ve_lsm6dsv_init(NULL);
    if (err != ESP_OK) {
        return err;
    }
    err = configure_sampling();
    if (err != ESP_OK) {
        goto fail;
    }
    err = pipeline_create_channel(&channel_config);
    if (err != ESP_OK) {
        goto fail;
    }
    channel_created = true;
    sensor_channel_t* channel = NULL;
    err = sensor_registry_find(get_sensor_registry(), channel_config.id,
                               &channel);
    if (err != ESP_OK) {
        goto fail;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = sample_tick,
        .name = "imu_sample",
        .skip_unhandled_events = true,
    };
    err = esp_timer_create(&timer_args, &sample_timer);
    if (err != ESP_OK) {
        goto fail;
    }
    if (xTaskCreate(lsm6dsv_task, "LSM6DSV", 4096, channel, 5, &sensor_task) !=
        pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    /* Poll twice per ODR interval to reduce readiness phase delays. The timer
     * has microsecond resolution, independent of CONFIG_FREERTOS_HZ. */
    err = esp_timer_start_periodic(sample_timer,
                                   channel_config.nominal_period_us / 2);
    if (err != ESP_OK) {
        vTaskDelete(sensor_task);
        sensor_task = NULL;
        goto fail;
    }
    *out_channel = channel;
    ESP_LOGI(TAG,
             "Publishing acceleration (m/s^2) and angular velocity (rad/s), "
             "nominal period %lu us",
             (unsigned long)channel_config.nominal_period_us);
    return ESP_OK;

fail:
    if (sample_timer != NULL) {
        esp_timer_delete(sample_timer);
        sample_timer = NULL;
    }
    if (channel_created) {
        pipeline_destroy_channel(channel_config.id);
    }
    ve_lsm6dsv_deinit();
    return err;
}
