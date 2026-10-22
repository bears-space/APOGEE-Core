#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "vigilant.h"

#include "platform.h"
#include "WSEN_HIDS_2525020210002.h"
#include "WSEN_PADS_2511020213301.h"

static WE_sensorInterface_t hids;

// static const char *TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    // Init thermometer
    HIDS_Get_Default_Interface(&hids);
    hids.interfaceType = WE_i2c;
    hids.handle = i2c_get_bus_handle();

    // Wait for sensor bootup
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&hids)) {

    }
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    // Test sensor readout routine
    int32_t temperatureRaw = 0;
    int32_t humidityRaw = 0;
    hids_measureCmd_t measureCmd = HIDS_MEASURE_HPM;
    while (true) {
        if (WE_SUCCESS == HIDS_Sensor_Measure_Raw(&hids, measureCmd, &temperatureRaw, &humidityRaw))
        {
            printf("temp is %f °C, humidity is %f %%\n", (float) temperatureRaw / 1000.0, (float) humidityRaw / 1000.0);
        }
    }
}
