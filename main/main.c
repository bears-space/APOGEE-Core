#include <unistd.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"
#include "vigilant.h"
#include "lsm6dsv.h"

// static const char *TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    esp_err_t imu_err = ve_lsm6dsv_init(NULL);
    ESP_LOGI("LSM6DSV", "ve_lsm6dsv_init: %s", esp_err_to_name(imu_err));
    if (imu_err != ESP_OK) {
        return;
    }

    int16_t gyro_data[3] = {0};
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        imu_err = ve_lsm6dsv_read_gyro_raw(gyro_data);
        if (imu_err != ESP_OK) {
            ESP_LOGE("LSM6DSV", "gyro read failed: %s",
                     esp_err_to_name(imu_err));
        } else {
            ESP_LOGI("LSM6DSV", "Gyro data: x=%d, y=%d, z=%d", gyro_data[0],
                     gyro_data[1], gyro_data[2]);
        }
    }
}
