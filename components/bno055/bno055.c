#include "bno055.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BNO055_REG_CHIP_ID 0x00
#define BNO055_REG_PAGE_ID 0x07
#define BNO055_REG_ACC_DATA 0x08
#define BNO055_REG_MAG_CONFIG 0x09  // Page 1.
#define BNO055_REG_UNIT_SEL 0x3B
#define BNO055_REG_OPR_MODE 0x3D
#define BNO055_REG_PWR_MODE 0x3E
#define BNO055_REG_SYS_TRIGGER 0x3F
#define BNO055_CHIP_ID 0xA0
#define BNO055_MODE_CONFIG 0x00
#define BNO055_MODE_AMG 0x07
#define BNO055_TIMEOUT_MS 100

static void delay_ms(uint32_t ms) {
    // Round up and allow one extra tick so minimum hardware delays are met.
    vTaskDelay(pdMS_TO_TICKS(ms) + 1);
}

static esp_err_t write_reg(i2c_master_dev_handle_t device, uint8_t reg,
                           uint8_t value) {
    const uint8_t bytes[] = {reg, value};
    return i2c_master_transmit(device, bytes, sizeof(bytes), BNO055_TIMEOUT_MS);
}

static esp_err_t configure(i2c_master_dev_handle_t device) {
    // Allow power-on startup before accessing the register map.
    delay_ms(700);
    esp_err_t err = write_reg(device, BNO055_REG_PAGE_ID, 0);
    if (err != ESP_OK) return err;

    const uint8_t reg = BNO055_REG_CHIP_ID;
    uint8_t id = 0;
    err =
        i2c_master_transmit_receive(device, &reg, 1, &id, 1, BNO055_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    if (id != BNO055_CHIP_ID) return ESP_ERR_INVALID_RESPONSE;

    err = write_reg(device, BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG);
    if (err != ESP_OK) return err;
    delay_ms(25);

    // Reset restores sensor ranges, axis mapping, units, and offsets even when
    // only the ESP has restarted and the BNO055 remained powered.
    err = write_reg(device, BNO055_REG_SYS_TRIGGER, 0x20);
    if (err != ESP_OK) return err;
    delay_ms(700);

    err =
        i2c_master_transmit_receive(device, &reg, 1, &id, 1, BNO055_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    if (id != BNO055_CHIP_ID) return ESP_ERR_INVALID_RESPONSE;

    err = write_reg(device, BNO055_REG_PWR_MODE, 0);  // Normal power.
    if (err != ESP_OK) return err;
    delay_ms(10);
    err = write_reg(device, BNO055_REG_UNIT_SEL, 0);
    if (err != ESP_OK) return err;
    err = write_reg(device, BNO055_REG_PAGE_ID, 1);
    if (err != ESP_OK) return err;
    // Normal power, regular preset, 30 Hz magnetometer output.
    err = write_reg(device, BNO055_REG_MAG_CONFIG, 0x0F);
    if (err != ESP_OK) return err;
    err = write_reg(device, BNO055_REG_PAGE_ID, 0);
    if (err != ESP_OK) return err;
    err = write_reg(device, BNO055_REG_OPR_MODE, BNO055_MODE_AMG);
    if (err != ESP_OK) return err;
    // Allow the sensors to start and produce their first samples.
    delay_ms(100);
    return ESP_OK;
}

esp_err_t bno055_init(i2c_master_bus_handle_t bus, uint8_t address,
                      i2c_master_dev_handle_t* device) {
    if (!device) return ESP_ERR_INVALID_ARG;
    *device = NULL;
    if (!bus || (address != BNO055_I2C_ADDRESS_LOW &&
                 address != BNO055_I2C_ADDRESS_HIGH)) {
        return ESP_ERR_INVALID_ARG;
    }

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
        .scl_wait_us = 20000,  // BNO055 uses clock stretching.
    };
    i2c_master_dev_handle_t handle = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &config, &handle);
    if (err != ESP_OK) return err;
    err = configure(handle);
    if (err != ESP_OK) {
        i2c_master_bus_rm_device(handle);
        return err;
    }
    *device = handle;
    return ESP_OK;
}

static int16_t decode_i16(const uint8_t* bytes) {
    uint16_t value = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return (int16_t)(value < 0x8000 ? (int32_t)value : (int32_t)value - 65536);
}

static bno055_raw_vector_t decode_vector(const uint8_t* bytes) {
    return (bno055_raw_vector_t){
        .x = decode_i16(bytes),
        .y = decode_i16(bytes + 2),
        .z = decode_i16(bytes + 4),
    };
}

esp_err_t bno055_read_raw(i2c_master_dev_handle_t device,
                          bno055_raw_data_t* data) {
    if (!device || !data) return ESP_ERR_INVALID_ARG;
    const uint8_t reg = BNO055_REG_ACC_DATA;
    uint8_t bytes[18];
    esp_err_t err = i2c_master_transmit_receive(
        device, &reg, 1, bytes, sizeof(bytes), BNO055_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    *data = (bno055_raw_data_t){
        .accel = decode_vector(bytes),
        .mag = decode_vector(bytes + 6),
        .gyro = decode_vector(bytes + 12),
    };
    return ESP_OK;
}
