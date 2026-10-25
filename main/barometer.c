#include "barometer.h"

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
