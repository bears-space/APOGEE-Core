#pragma once

#include "lsm6dsv.h"

/* SA0 address select. Defaults to SA0=0 -> 0x6A. */
#if defined(CONFIG_VE_LSM6DSV_SA0_LOW)
#define VE_LSM6DSV_I2C_ADDR 0x6A
#elif defined(CONFIG_VE_LSM6DSV_SA0_HIGH)
#define VE_LSM6DSV_I2C_ADDR 0x6B
#else
#define VE_LSM6DSV_I2C_ADDR 0x6A
#endif

/* Gyro full scale. */
#if defined(CONFIG_VE_LSM6DSV_GY_FS_250DPS)
#define VE_LSM6DSV_GY_FS LSM6DSV320X_250dps
#elif defined(CONFIG_VE_LSM6DSV_GY_FS_500DPS)
#define VE_LSM6DSV_GY_FS LSM6DSV320X_500dps
#elif defined(CONFIG_VE_LSM6DSV_GY_FS_1000DPS)
#define VE_LSM6DSV_GY_FS LSM6DSV320X_1000dps
#elif defined(CONFIG_VE_LSM6DSV_GY_FS_2000DPS)
#define VE_LSM6DSV_GY_FS LSM6DSV320X_2000dps
#elif defined(CONFIG_VE_LSM6DSV_GY_FS_4000DPS)
#define VE_LSM6DSV_GY_FS LSM6DSV320X_4000dps
#else
#define VE_LSM6DSV_GY_FS LSM6DSV320X_2000dps
#endif

/* Low-G accelerometer full scale. */
#if defined(CONFIG_VE_LSM6DSV_XL_FS_2G)
#define VE_LSM6DSV_XL_FS LSM6DSV320X_2g
#elif defined(CONFIG_VE_LSM6DSV_XL_FS_4G)
#define VE_LSM6DSV_XL_FS LSM6DSV320X_4g
#elif defined(CONFIG_VE_LSM6DSV_XL_FS_8G)
#define VE_LSM6DSV_XL_FS LSM6DSV320X_8g
#elif defined(CONFIG_VE_LSM6DSV_XL_FS_16G)
#define VE_LSM6DSV_XL_FS LSM6DSV320X_16g
#else
#define VE_LSM6DSV_XL_FS LSM6DSV320X_4g
#endif

/* High-G accelerometer full scale. */
#if defined(CONFIG_VE_LSM6DSV_HG_FS_32G)
#define VE_LSM6DSV_HG_FS LSM6DSV320X_32g
#elif defined(CONFIG_VE_LSM6DSV_HG_FS_64G)
#define VE_LSM6DSV_HG_FS LSM6DSV320X_64g
#elif defined(CONFIG_VE_LSM6DSV_HG_FS_128G)
#define VE_LSM6DSV_HG_FS LSM6DSV320X_128g
#elif defined(CONFIG_VE_LSM6DSV_HG_FS_256G)
#define VE_LSM6DSV_HG_FS LSM6DSV320X_256g
#elif defined(CONFIG_VE_LSM6DSV_HG_FS_320G)
#define VE_LSM6DSV_HG_FS LSM6DSV320X_320g
#else
#define VE_LSM6DSV_HG_FS LSM6DSV320X_320g
#endif

/* Shared low-G / gyro output data rate. */
#if defined(CONFIG_VE_LSM6DSV_ODR_OFF)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_OFF
#elif defined(CONFIG_VE_LSM6DSV_ODR_15HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_15Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_30HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_30Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_60HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_60Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_120HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_120Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_240HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_240Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_480HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_480Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_960HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_960Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_1920HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_1920Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_3840HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_3840Hz
#elif defined(CONFIG_VE_LSM6DSV_ODR_7680HZ)
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_7680Hz
#else
#define VE_LSM6DSV_ODR LSM6DSV320X_ODR_AT_120Hz
#endif

/* Low-G accelerometer operating mode. */
#if defined(CONFIG_VE_LSM6DSV_XL_MODE_HIGH_PERFORMANCE)
#define VE_LSM6DSV_XL_MODE LSM6DSV320X_XL_HIGH_PERFORMANCE_MD
#elif defined(CONFIG_VE_LSM6DSV_XL_MODE_NORMAL)
#define VE_LSM6DSV_XL_MODE LSM6DSV320X_XL_NORMAL_MD
#else
#define VE_LSM6DSV_XL_MODE LSM6DSV320X_XL_HIGH_PERFORMANCE_MD
#endif

/* Gyroscope operating mode. */
#if defined(CONFIG_VE_LSM6DSV_GY_MODE_HIGH_PERFORMANCE)
#define VE_LSM6DSV_GY_MODE LSM6DSV320X_GY_HIGH_PERFORMANCE_MD
#elif defined(CONFIG_VE_LSM6DSV_GY_MODE_LOW_POWER)
#define VE_LSM6DSV_GY_MODE LSM6DSV320X_GY_LOW_POWER_MD
#elif defined(CONFIG_VE_LSM6DSV_GY_MODE_SLEEP)
#define VE_LSM6DSV_GY_MODE LSM6DSV320X_GY_SLEEP_MD
#else
#define VE_LSM6DSV_GY_MODE LSM6DSV320X_GY_HIGH_PERFORMANCE_MD
#endif

/* High-G accelerometer output data rate. */
#if defined(CONFIG_VE_LSM6DSV_HG_ODR_OFF)
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_OFF
#elif defined(CONFIG_VE_LSM6DSV_HG_ODR_480HZ)
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_480Hz
#elif defined(CONFIG_VE_LSM6DSV_HG_ODR_960HZ)
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_960Hz
#elif defined(CONFIG_VE_LSM6DSV_HG_ODR_1920HZ)
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_1920Hz
#elif defined(CONFIG_VE_LSM6DSV_HG_ODR_3840HZ)
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_3840Hz
#elif defined(CONFIG_VE_LSM6DSV_HG_ODR_7680HZ)
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_7680Hz
#else
#define VE_LSM6DSV_HG_ODR LSM6DSV320X_HG_XL_ODR_AT_1920Hz
#endif

/* High-G channel data registers enable. */
#if defined(CONFIG_VE_LSM6DSV_HG_REG_OUT)
#define VE_LSM6DSV_HG_REG_OUT_EN true
#else
#define VE_LSM6DSV_HG_REG_OUT_EN false
#endif

/* Block data update. */
#if defined(CONFIG_VE_LSM6DSV_BDU)
#define VE_LSM6DSV_BDU_EN true
#else
#define VE_LSM6DSV_BDU_EN false
#endif

/* Register address auto-increment. */
#if defined(CONFIG_VE_LSM6DSV_IF_INC)
#define VE_LSM6DSV_IF_INC_EN true
#else
#define VE_LSM6DSV_IF_INC_EN false
#endif

#define VE_LSM6DSV_CONFIG_DEFAULTS                                        \
    (ve_lsm6dsv_config_t){                                                \
        .i2c_addr = VE_LSM6DSV_I2C_ADDR,                                  \
        .gy_full_scale = VE_LSM6DSV_GY_FS,                                \
        .xl_full_scale = VE_LSM6DSV_XL_FS,                                \
        .hg_xl_full_scale = VE_LSM6DSV_HG_FS,                             \
        .xl_odr = VE_LSM6DSV_ODR,                                         \
        .xl_mode = VE_LSM6DSV_XL_MODE,                                    \
        .gy_odr = VE_LSM6DSV_ODR,                                         \
        .gy_mode = VE_LSM6DSV_GY_MODE,                                    \
        .hg_xl_odr = VE_LSM6DSV_HG_ODR,                                   \
        .hg_xl_reg_out_en = VE_LSM6DSV_HG_REG_OUT_EN,                     \
        .bdu = VE_LSM6DSV_BDU_EN,                                         \
        .auto_increment = VE_LSM6DSV_IF_INC_EN,                           \
    }
