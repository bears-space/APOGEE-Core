#include "imu_low_g.h"

static WE_sensorInterface_t isds;

bool imu_low_g_init(void) {
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

bool imu_low_g_read(float* xAcc, float* yAcc, float* zAcc, float* xRate,
                    float* yRate, float* zRate, float* temperature) {
    ISDS_state_t dataReady;
    float isds_xAcc = 0;
    float isds_yAcc = 0;
    float isds_zAcc = 0;
    float isds_xRate = 0;
    float isds_yRate = 0;
    float isds_zRate = 0;
    float isds_temperature = 0;

    // Wait until the acceleration values are ready to read
    do {
        ISDS_isAccelerationDataReady(&isds, &dataReady);
    } while (dataReady == ISDS_disable);

    // Read acceleration values
    if (ISDS_getAccelerations_float(&isds, &isds_xAcc, &isds_yAcc,
                                    &isds_zAcc) != WE_SUCCESS) {
        return false;
    }

    // Wait until the gyroscope values are ready to read
    do {
        ISDS_isGyroscopeDataReady(&isds, &dataReady);
    } while (dataReady == ISDS_disable);

    // Read gyroscope values
    if (ISDS_getAngularRates_float(&isds, &isds_xRate, &isds_yRate,
                                   &isds_zRate) != WE_SUCCESS) {
        return false;
    }

    // Wait until the temperature value is ready to read
    do {
        ISDS_isTemperatureDataReady(&isds, &dataReady);
    } while (dataReady == ISDS_disable);
    if (ISDS_getTemperature_float(&isds, &isds_temperature) != WE_SUCCESS) {
        return false;
    }

    *xAcc = isds_xAcc;
    *yAcc = isds_yAcc;
    *zAcc = isds_zAcc;
    *xRate = isds_xRate;
    *yRate = isds_yRate;
    *zRate = isds_zRate;
    *temperature = isds_temperature;
    return true;
}
