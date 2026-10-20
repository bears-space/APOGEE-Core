#include "esp_err.h"
#include "esp_log.h"
#include "fusion.h"
#include "sensor/LSM6DSV_sensor.h"
#include "telemetry.h"
#include "vigilant.h"

void app_main(void) {
    VigilantConfig config = {.unique_component_name = "Vigilant ESP Test",
                             .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(config));
    ESP_ERROR_CHECK(pipeline_init());

    sensor_channel_t* imu_channel = NULL;
    ESP_ERROR_CHECK(lsm6dsv_sensor_init(&imu_channel));
    if (xTaskCreate(fusionProcess, "Fusion_Process", 8192, imu_channel, 4,
                    NULL) != pdPASS) {
        ESP_LOGE("app_main", "Failed to create IMU consumer");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
