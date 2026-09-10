#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BNO055_I2C_ADDRESS_LOW 0x28
#define BNO055_I2C_ADDRESS_HIGH 0x29

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} bno055_raw_vector_t;

typedef struct {
    bno055_raw_vector_t accel;
    bno055_raw_vector_t mag;
    bno055_raw_vector_t gyro;
} bno055_raw_data_t;

/** Attach to an existing synchronous I2C bus and initialize in non-fusion AMG
 * mode. Blocks during startup. On success, caller owns *device and must remove
 * it with i2c_master_bus_rm_device() before deleting the bus. On failure,
 * *device is NULL. Serialize initialization/removal against reads.
 */
esp_err_t bno055_init(i2c_master_bus_handle_t bus, uint8_t address,
                      i2c_master_dev_handle_t* device);

/** Read signed register values without scaling, filtering, or fusion. Output is
 * unchanged on I2C failure. Device must have been initialized by bno055_init().
 */
esp_err_t bno055_read_raw(i2c_master_dev_handle_t device,
                          bno055_raw_data_t* data);

#ifdef __cplusplus
}
#endif
