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

static const char* TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));


    pipeline_init();

    thermometer_sensor_init();

    /*
    // Init sensors
    ESP_LOGI(TAG, "Initializing sensors...");
    // TODO fail gracefully if any sensor breaks / fails to initialize
    // barometer_init(); // TODO reenable when we have board with working PADS
    imu_low_g_init();
    

    // Sensor values
    float barometer_pressure = 0.0;
    float barometer_temperature = 0.0;

    float imu_low_g_xAcc = 0;
    float imu_low_g_yAcc = 0;
    float imu_low_g_zAcc = 0;
    float imu_low_g_xRate = 0;
    float imu_low_g_yRate = 0;
    float imu_low_g_zRate = 0;
    float imu_low_g_temperature = 0;

    // Main loop
    ESP_LOGI(TAG, "Entering main loop...");
    while (true) {
        // Update sensor values
        // barometer_read_pressure(&barometer_pressure);
        // barometer_read_temperature(&barometer_temperature);
        
        imu_low_g_read(&imu_low_g_xAcc, &imu_low_g_yAcc, &imu_low_g_zAcc,
                       &imu_low_g_xRate, &imu_low_g_yRate, &imu_low_g_zRate,
                       &imu_low_g_temperature);

        // Print sensor values to debug UART
        printf("Barometer pressure %f, temperature %f\n", barometer_pressure,
               barometer_temperature);
        printf("Thermometer temperature %f, humidity %f\n",
               thermometer_temperature, thermometer_humidity);
        printf(
            "LOW-G IMU xAcc %f, yAcc %f, zAcc %f, xRate %f, yRate %f, zRate "
            "%f, temp %f\n",
            imu_low_g_xAcc, imu_low_g_yAcc, imu_low_g_zAcc, imu_low_g_xRate,
            imu_low_g_yRate, imu_low_g_zRate, imu_low_g_temperature);
    }
    */
}
