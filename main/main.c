#include <stdio.h>

#include "bno055.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "vigilant.h"
static const char* TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(I2C_NUM_0, &bus));
    i2c_master_dev_handle_t sensor = NULL;
    ESP_ERROR_CHECK(bno055_init(bus, CONFIG_BNO055_I2C_ADDRESS, &sensor));

    while (true) {
        bno055_raw_data_t raw;
        esp_err_t err = bno055_read_raw(sensor, &raw);
        if (err == ESP_OK) {
            // CSV: ax,ay,az,mx,my,mz,gx,gy,gz (signed register values).
            printf("%d,%d,%d,%d,%d,%d,%d,%d,%d\n", raw.accel.x, raw.accel.y,
                   raw.accel.z, raw.mag.x, raw.mag.y, raw.mag.z, raw.gyro.x,
                   raw.gyro.y, raw.gyro.z);
        } else {
            ESP_LOGE(TAG, "BNO055 read failed: %s", esp_err_to_name(err));
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_BNO055_SAMPLE_PERIOD_MS) + 1);
    }
}
