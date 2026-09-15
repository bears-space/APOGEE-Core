#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vigilant.h"

#include "lsm6dsv.h"
#include "lsm6dsv_kconfig_map.h"
#include "lsm6dsv320x_reg.h"

static VigilantI2CDevice s_dev = {
    .address = VE_LSM6DSV_I2C_ADDR,
    .whoami_reg = LSM6DSV320X_WHO_AM_I,
    .expected_whoami = LSM6DSV320X_ID,
    .handle = NULL,
};

static stmdev_ctx_t s_ctx = {0};
static bool s_initialized = false;

static int32_t platform_read(void* handle, uint8_t reg, uint8_t* buf,
                             uint16_t len)
{
    if (len == 0) {
        return 0;
    }
    return vigilant_i2c_read_regs((VigilantI2CDevice*)handle, reg, buf, len) ==
                   ESP_OK
               ? 0
               : -1;
}

static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* buf,
                              uint16_t len)
{
    if (len == 0) {
        return 0;
    }
    return vigilant_i2c_write_regs((VigilantI2CDevice*)handle, reg, buf, len) ==
                   ESP_OK
               ? 0
               : -1;
}

static void platform_delay(uint32_t millisec)
{
    vTaskDelay(pdMS_TO_TICKS(millisec));
}

static esp_err_t ve_lsm6dsv_dev_add(void)
{
    if (s_dev.handle != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return vigilant_i2c_add_device(&s_dev);
}

static esp_err_t ve_lsm6dsv_dev_remove(void)
{
    esp_err_t err = ESP_OK;
    if (s_dev.handle != NULL) {
        err = vigilant_i2c_remove_device(&s_dev);
    }
    return err;
}

esp_err_t ve_lsm6dsv_init(const ve_lsm6dsv_config_t* config)
{
    ve_lsm6dsv_config_t cfg;
    esp_err_t err;
    int32_t ret;

    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        cfg = VE_LSM6DSV_CONFIG_DEFAULTS;
    } else {
        cfg = *config;
    }

    s_dev.address = cfg.i2c_addr;

    err = ve_lsm6dsv_dev_add();
    if (err != ESP_OK) {
        return err;
    }

    s_ctx.read_reg = platform_read;
    s_ctx.write_reg = platform_write;
    s_ctx.mdelay = platform_delay;
    s_ctx.handle = &s_dev;
    s_ctx.priv_data = NULL;

    ret = lsm6dsv320x_sw_reset(&s_ctx);
    if (ret != 0) {
        goto fail;
    }

    err = vigilant_i2c_whoami_check(&s_dev);
    if (err != ESP_OK) {
        goto fail;
    }

    ret = lsm6dsv320x_block_data_update_set(&s_ctx, cfg.bdu ? 1 : 0);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_auto_increment_set(&s_ctx, cfg.auto_increment ? 1 : 0);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_gy_full_scale_set(&s_ctx, cfg.gy_full_scale);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_xl_full_scale_set(&s_ctx, cfg.xl_full_scale);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_hg_xl_full_scale_set(&s_ctx, cfg.hg_xl_full_scale);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_xl_setup(&s_ctx, cfg.xl_odr, cfg.xl_mode);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_gy_setup(&s_ctx, cfg.gy_odr, cfg.gy_mode);
    if (ret != 0) {
        goto fail;
    }

    ret = lsm6dsv320x_hg_xl_data_rate_set(&s_ctx, cfg.hg_xl_odr,
                                          cfg.hg_xl_reg_out_en ? 1 : 0);
    if (ret != 0) {
        goto fail;
    }

    s_initialized = true;
    return ESP_OK;

fail:
    ve_lsm6dsv_dev_remove();
    memset(&s_ctx, 0, sizeof(s_ctx));
    return ESP_FAIL;
}

esp_err_t ve_lsm6dsv_deinit(void)
{
    esp_err_t err;

    if (!s_initialized) {
        return ESP_OK;
    }

    err = ve_lsm6dsv_dev_remove();
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_initialized = false;
    return err;
}

bool ve_lsm6dsv_is_initialized(void)
{
    return s_initialized;
}

const stmdev_ctx_t* ve_lsm6dsv_get_ctx(void)
{
    return s_initialized ? &s_ctx : NULL;
}

static esp_err_t ve_lsm6dsv_read_raw(int32_t (*getter)(const stmdev_ctx_t*,
                                                       int16_t*),
                                     int16_t* out)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return getter(&s_ctx, out) == 0 ? ESP_OK : ESP_FAIL;
}

esp_err_t ve_lsm6dsv_read_accel_raw(int16_t* out)
{
    return ve_lsm6dsv_read_raw(lsm6dsv320x_acceleration_raw_get, out);
}

esp_err_t ve_lsm6dsv_read_hg_accel_raw(int16_t* out)
{
    return ve_lsm6dsv_read_raw(lsm6dsv320x_hg_acceleration_raw_get, out);
}

esp_err_t ve_lsm6dsv_read_gyro_raw(int16_t* out)
{
    return ve_lsm6dsv_read_raw(lsm6dsv320x_angular_rate_raw_get, out);
}

esp_err_t ve_lsm6dsv_read_temp_raw(int16_t* out)
{
    return ve_lsm6dsv_read_raw(lsm6dsv320x_temperature_raw_get, out);
}

esp_err_t ve_lsm6dsv_data_ready(bool* drdy_hgxl, bool* drdy_xl, bool* drdy_gy,
                                bool* drdy_temp)
{
    lsm6dsv320x_data_ready_t drdy = {0};
    int32_t ret;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = lsm6dsv320x_flag_data_ready_get(&s_ctx, &drdy);
    if (ret != 0) {
        return ESP_FAIL;
    }

    if (drdy_hgxl != NULL) {
        *drdy_hgxl = drdy.drdy_hgxl != 0;
    }
    if (drdy_xl != NULL) {
        *drdy_xl = drdy.drdy_xl != 0;
    }
    if (drdy_gy != NULL) {
        *drdy_gy = drdy.drdy_gy != 0;
    }
    if (drdy_temp != NULL) {
        *drdy_temp = drdy.drdy_temp != 0;
    }
    return ESP_OK;
}
