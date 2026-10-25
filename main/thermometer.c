#include "thermometer.h"

static const char* TAG = "thermometer";

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
