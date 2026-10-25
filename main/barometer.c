#include "barometer.h"

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
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    WE_Delay(5);

    // First communication test
    uint8_t deviceIdValue = 0;
    if (WE_SUCCESS == PADS_getDeviceID(&pads, &deviceIdValue)) {
        if (deviceIdValue ==
            PADS_DEVICE_ID_VALUE)  // who am i ? - i am WSEN-PADS!
        {
            debugPrintln("**** PADS_DEVICE_ID_VALUE: OK ****");
        } else {
            debugPrintln("**** PADS_DEVICE_ID_VALUE: NOT OK ****");
            return false;
        }
    } else {
        debugPrintln("**** PADS_getDeviceID(): NOT OK ****");
        return false;
    }

    // Perform soft reset of the sensor
    PADS_softReset(&pads, PADS_enable);
    PADS_state_t swReset;
    do {
        PADS_getSoftResetState(&pads, &swReset);
    } while (swReset);
    debugPrintln("**** PADS reset complete ****");

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
