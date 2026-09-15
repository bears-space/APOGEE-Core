#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "lsm6dsv320x_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t i2c_addr;                              /* 7-bit I2C address */
    lsm6dsv320x_gy_full_scale_t gy_full_scale;
    lsm6dsv320x_xl_full_scale_t xl_full_scale;
    lsm6dsv320x_hg_xl_full_scale_t hg_xl_full_scale;
    lsm6dsv320x_data_rate_t xl_odr;
    lsm6dsv320x_xl_mode_t xl_mode;
    lsm6dsv320x_data_rate_t gy_odr;
    lsm6dsv320x_gy_mode_t gy_mode;
    lsm6dsv320x_hg_xl_data_rate_t hg_xl_odr;
    bool hg_xl_reg_out_en;
    bool bdu;
    bool auto_increment;
} ve_lsm6dsv_config_t;

esp_err_t ve_lsm6dsv_init(const ve_lsm6dsv_config_t* config);
esp_err_t ve_lsm6dsv_deinit(void);
bool ve_lsm6dsv_is_initialized(void);
const stmdev_ctx_t* ve_lsm6dsv_get_ctx(void);

esp_err_t ve_lsm6dsv_read_accel_raw(int16_t* out);
esp_err_t ve_lsm6dsv_read_hg_accel_raw(int16_t* out);
esp_err_t ve_lsm6dsv_read_gyro_raw(int16_t* out);
esp_err_t ve_lsm6dsv_read_temp_raw(int16_t* out);
esp_err_t ve_lsm6dsv_data_ready(bool* drdy_hgxl, bool* drdy_xl, bool* drdy_gy,
                                bool* drdy_temp);

#ifdef __cplusplus
}
#endif
