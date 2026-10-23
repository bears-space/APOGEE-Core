#include <unistd.h>

#include "WSEN_HIDS_2525020210002.h"
#include "WSEN_ISDS_2536030320001.h"
#include "WSEN_PADS_2511020213301.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "platform.h"
#include "vigilant.h"

static WE_sensorInterface_t hids;  // thermometer
static WE_sensorInterface_t isds;  // low-g IMU
static WE_sensorInterface_t pads;  // barometer

// static const char *TAG = "app_main";

void hids_init(void) {
    // TODO optimize initialization routine
    HIDS_Get_Default_Interface(&hids);
    hids.interfaceType = WE_i2c;
    hids.handle = i2c_get_bus_handle();

    // Wait for sensor bootup
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&hids)) {
    }
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");
}

bool isds_init(void) {
    // TODO optimize initialization routine
    // Initialize sensor interface (i2c with ISDS address, burst mode activated)
    ISDS_getDefaultInterface(&isds);
    isds.interfaceType = WE_i2c;
    isds.options.i2c.burstMode = 1;
    isds.options.i2c.address =
        ISDS_ADDRESS_I2C_0;  // is on 6A instead of 6B, maybe change back when
                             // new PCB arrives
    isds.handle = i2c_get_bus_handle();

    // Wait for sensor bootup
    while (WE_SUCCESS != WE_isSensorInterfaceReady(&isds)) {
    }
    debugPrintln("**** WE_isSensorInterfaceReady(): OK ****");

    WE_Delay(5);

    // First communication test
    uint8_t deviceIdValue = 0;
    if (WE_SUCCESS == ISDS_getDeviceID(&isds, &deviceIdValue)) {
        printf("ISDS_DEVICE_ID_VALUE should be %d\n", ISDS_DEVICE_ID_VALUE);
        printf("deviceIdValue is actually %d\n", deviceIdValue);
        if (deviceIdValue ==
            ISDS_DEVICE_ID_VALUE)  // who am i ? - i am WSEN-ISDS!
        {
            debugPrintln("**** ISDS_DEVICE_ID_VALUE: OK ****");
        } else {
            debugPrintln("**** ISDS_DEVICE_ID_VALUE: NOT OK ****");
            return false;
        }
    } else {
        debugPrintln("**** ISDS_getDeviceID(): NOT OK ****");
        return false;
    }

    // Perform soft reset of the sensor
    ISDS_softReset(&isds, ISDS_enable);
    ISDS_state_t swReset;
    do {
        ISDS_getSoftResetState(&isds, &swReset);
    } while (swReset);
    debugPrintln("**** ISDS reset complete ****");

    // Perform reboot (retrieve trimming parameters from nonvolatile memory)
    ISDS_reboot(&isds, ISDS_enable);
    WE_Delay(15);
    debugPrintln("**** ISDS reboot complete ****");

    // Enable block data update
    ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

    // Sampling rate (104 Hz)
    ISDS_setAccOutputDataRate(&isds, ISDS_accOdr104Hz);
    ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr104Hz);

    // Accelerometer 16g range
    ISDS_setAccFullScale(&isds, ISDS_accFullScaleSixteenG);

    // Gyroscope 2000 dps range
    ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);

    // Disable the disabling of high performance mode
    ISDS_disableAccHighPerformanceMode(&isds, ISDS_disable);
    ISDS_disableGyroHighPerformanceMode(&isds, ISDS_disable);

    return true;
}

bool pads_init(void) {
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

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    // Init thermometer
    // TODO fail gracefully if any sensor breaks / fails to initialize
    hids_init();
    isds_init();
    // pads_init(); // TODO reenable when we have a board with a non-fried PADS
    // sensor

    // Test sensor readout routine
    // TODO replace test code with proper readout logic
    while (true) {
        // Test thermometer
        int32_t hids_temperatureRaw = 0;
        int32_t hids_humidityRaw = 0;
        hids_measureCmd_t measureCmd = HIDS_MEASURE_HPM;
        if (WE_SUCCESS == HIDS_Sensor_Measure_Raw(&hids, measureCmd,
                                                  &hids_temperatureRaw,
                                                  &hids_humidityRaw)) {
            printf("Thermometer temp is %f °C, humidity is %f %%\n",
                   (float)hids_temperatureRaw / 1000.0,
                   (float)hids_humidityRaw / 1000.0);
        }

        // Test barometer
        // TODO reenable when we have a board with a non-fried PADS sensor
        /*float pads_pressureFloat = 0;
        if (WE_SUCCESS == PADS_getPressure_float(&pads, &pads_pressureFloat))
        {
            printf("Barometer pressure is %f kPa\n", pads_pressureFloat);
        }
        else
        {
            debugPrintln("**** PADS_getPressure_float(): NOT OK ****");
        }

        float pads_temperatureFloat = 0;
        if (WE_SUCCESS == PADS_getTemperature_float(&pads,
        &pads_temperatureFloat))
        {
            printf("Barometer temperature is %f °C\n", pads_temperatureFloat);
        }
        else
        {
            debugPrintln("**** PADS_getTemperature_float(): NOT OK ****");
        }*/

        // Test low-g IMU
        ISDS_state_t dataReady;
        float xAcc = 0;
        float yAcc = 0;
        float zAcc = 0;
        float xRate = 0;
        float yRate = 0;
        float zRate = 0;
        float isds_temperature = 0;

        // Wait until the acceleration values are ready to read
        do {
            ISDS_isAccelerationDataReady(&isds, &dataReady);
        } while (dataReady == ISDS_disable);

        // Read acceleration values
        if (ISDS_getAccelerations_float(&isds, &xAcc, &yAcc, &zAcc) !=
            WE_SUCCESS) {
            debugPrintln("**** ISDS_getAccelerations_float(): NOT OK ****");
            xAcc = 0;
            yAcc = 0;
            zAcc = 0;
        }

        // Wait until the gyroscope values are ready to read
        do {
            ISDS_isGyroscopeDataReady(&isds, &dataReady);
        } while (dataReady == ISDS_disable);

        // Read gyroscope values
        if (ISDS_getAngularRates_float(&isds, &xRate, &yRate, &zRate) !=
            WE_SUCCESS) {
            debugPrintln("**** ISDS_getAngularRates_float(): NOT OK ****");
            xRate = 0;
            yRate = 0;
            zRate = 0;
        }

        // Wait until the temperature value is ready to read
        do {
            ISDS_isTemperatureDataReady(&isds, &dataReady);
        } while (dataReady == ISDS_disable);
        if (ISDS_getTemperature_float(&isds, &isds_temperature) != WE_SUCCESS) {
            debugPrintln("**** ISDS_getTemperature_float(): NOT OK ****");
            isds_temperature = 0;
        }

        /* Print values to debug UART */
        printf(
            "LOW-G IMU xAcc %f, yAcc %f, zAcc %f, xRate %f, yRate %f, zRate "
            "%f, temp %f\n",
            xAcc, yAcc, zAcc, xRate, yRate, zRate, isds_temperature);
    }
}
