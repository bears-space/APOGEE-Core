#include "vigilant.h"

#include <unistd.h>

#include "barometer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "imu_low_g.h"
#include "platform.h"
#include "thermometer.h"

#include "telemetry.h"
#include "fusion.h"

static const char* TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));


    ESP_LOGI(TAG, "Initializing telemetry pipeline...");
    ESP_ERROR_CHECK(pipeline_init());
    
    ESP_LOGI(TAG, "Initializing sensors...");
    ESP_ERROR_CHECK(thermometer_sensor_init());
    ESP_ERROR_CHECK(imu_low_g_sensor_init());
    ESP_ERROR_CHECK(barometer_sensor_init());

    ESP_LOGI(TAG, "Initializing fusion task...");
    ESP_ERROR_CHECK(fusionInit());
}
