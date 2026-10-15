/*
 * SPDX-FileCopyrightText: 2026 STARSTREAK
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * C port of the SparkFun MMC5983MA Magnetometer Arduino Library
 * (MIT License, written by Ricardo Ramos @ SparkFun Electronics, Feb 2022):
 * https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library
 *
 * I2C communication goes through the Vigilant Engine I2C interface
 * (vigilant_i2c_read_regs / vigilant_i2c_write_regs).
 */

#include "mmc5983ma.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mmc5983ma_reg.h"
#include "vigilant.h"

static const char* TAG = "mmc5983ma";

ESP_EVENT_DEFINE_BASE(MMC5983MA_EVENT);

struct mmc5983ma {
    VigilantI2CDevice i2c_device;
    mmc5983ma_config_t config;

    // INT_CTRL_0..3 are write-only on the MMC5983MA (they read back garbage),
    // so all bit manipulation is done in shadow memory which is atomically
    // written to the register.
    struct {
        uint8_t internal_control0;
        uint8_t internal_control1;
        uint8_t internal_control2;
        uint8_t internal_control3;
    } shadow;

    esp_event_loop_handle_t event_loop;
    TaskHandle_t task_handle;
};

static uint8_t* get_shadow_register(mmc5983ma_handle_t handle, uint8_t reg) {
    switch (reg) {
        case INT_CTRL_0_REG:
            return &handle->shadow.internal_control0;
        case INT_CTRL_1_REG:
            return &handle->shadow.internal_control1;
        case INT_CTRL_2_REG:
            return &handle->shadow.internal_control2;
        case INT_CTRL_3_REG:
            return &handle->shadow.internal_control3;
        default:
            return NULL;
    }
}

// Sets bit(s) on the shadow register and then on the register (if do_write).
static esp_err_t set_shadow_bit(mmc5983ma_handle_t handle, uint8_t reg,
                                uint8_t mask, bool do_write) {
    uint8_t* shadow = get_shadow_register(handle, reg);
    if (!shadow) {
        return ESP_ERR_INVALID_ARG;
    }

    *shadow |= mask;
    if (!do_write) {
        return ESP_OK;
    }
    return vigilant_i2c_write_regs(&handle->i2c_device, reg, shadow, 1);
}

// Clears bit(s) on the shadow register and then on the register (if do_write).
static esp_err_t clear_shadow_bit(mmc5983ma_handle_t handle, uint8_t reg,
                                  uint8_t mask, bool do_write) {
    uint8_t* shadow = get_shadow_register(handle, reg);
    if (!shadow) {
        return ESP_ERR_INVALID_ARG;
    }

    *shadow &= ~mask;
    if (!do_write) {
        return ESP_OK;
    }
    return vigilant_i2c_write_regs(&handle->i2c_device, reg, shadow, 1);
}

// Checks if a specific bit is set on a shadow register.
static bool is_shadow_bit_set(mmc5983ma_handle_t handle, uint8_t reg,
                              uint8_t mask) {
    uint8_t* shadow = get_shadow_register(handle, reg);
    if (!shadow) {
        return false;
    }
    return (*shadow & mask) != 0;
}

static bool mmc_is_bit_set(mmc5983ma_handle_t handle, uint8_t reg,
                           uint8_t mask) {
    uint8_t value = 0;
    if (vigilant_i2c_read_regs(&handle->i2c_device, reg, &value, 1) != ESP_OK) {
        return false;
    }
    return (value & mask) != 0;
}

// Returns a timeout for mmc5983ma_read_raw based on BW1/0.
static uint16_t get_timeout(mmc5983ma_handle_t handle) {
    uint16_t bandwidth = mmc5983ma_get_filter_bandwidth(handle);
    if (bandwidth == 0) {
        bandwidth = 100;
    }

    // 4 * the measurement time (defined by BW1/0), plus 1 ms just in case.
    uint16_t timeout = 800 / bandwidth;  // 8/4/2/1 ms
    timeout *= 4;                        // 32/16/8/4 ms
    timeout += 1;
    return timeout;
}

// Reads and assembles the X, Y and Z 18-bit field strengths.
static esp_err_t read_fields_xyz(mmc5983ma_handle_t handle, uint32_t* x,
                                 uint32_t* y, uint32_t* z) {
    uint8_t register_values[7] = {0};

    esp_err_t err =
        vigilant_i2c_read_regs(&handle->i2c_device, X_OUT_0_REG,
                               register_values, sizeof(register_values));
    if (err != ESP_OK) {
        return err;
    }

    *x = register_values[0];                              // Xout[17:10]
    *x = (*x << 8) | register_values[1];                  // Xout[9:2]
    *x = (*x << 2) | (register_values[6] >> 6);           // Xout[1:0]
    *y = register_values[2];                              // Yout[17:10]
    *y = (*y << 8) | register_values[3];                  // Yout[9:2]
    *y = (*y << 2) | ((register_values[6] >> 4) & 0x03);  // Yout[1:0]
    *z = register_values[4];                              // Zout[17:10]
    *z = (*z << 8) | register_values[5];                  // Zout[9:2]
    *z = (*z << 2) | ((register_values[6] >> 2) & 0x03);  // Zout[1:0]

    return ESP_OK;
}

// Continuous measurement task: reads at the configured frequency and posts
// MMC5983MA_EVENT_MEASUREMENT events on the driver's own event loop.
static void mmc5983ma_task_entry(void* arg) {
    mmc5983ma_handle_t dev = (mmc5983ma_handle_t)arg;

    uint16_t frequency = mmc5983ma_get_continuous_frequency(dev);
    uint32_t period_ms = frequency > 0 ? (1000u / frequency) : 1000u;
    if (period_ms == 0) {
        period_ms = 1;
    }

    while (1) {
        mmc5983ma_measurement_t measurement = {0};
        esp_err_t err = mmc5983ma_read_raw(dev, &measurement.x, &measurement.y,
                                           &measurement.z);
        if (err == ESP_OK) {
            esp_err_t post_err = esp_event_post_to(
                dev->event_loop, MMC5983MA_EVENT, MMC5983MA_EVENT_MEASUREMENT,
                &measurement, sizeof(measurement), 0);
            if (post_err != ESP_OK) {
                ESP_LOGW(TAG, "Failed to post measurement event: %s",
                         esp_err_to_name(post_err));
            }
        } else {
            ESP_LOGW(TAG, "Measurement failed: %s", esp_err_to_name(err));
        }

        esp_event_loop_run(dev->event_loop, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(period_ms));
    }

    vTaskDelete(NULL);
}

mmc5983ma_handle_t mmc5983ma_init(const mmc5983ma_config_t* config) {
    mmc5983ma_handle_t dev = calloc(1, sizeof(*dev));
    if (!dev) {
        ESP_LOGE(TAG, "Failed to allocate MMC5983MA driver");
        return NULL;
    }

    if (config) {
        dev->config = *config;
    } else {
        dev->config = (mmc5983ma_config_t)MMC5983MA_CONFIG_DEFAULT();
    }

    dev->i2c_device = (VigilantI2CDevice){
        .address = MMC5983MA_I2C_ADDRESS,
        .whoami_reg = PROD_ID_REG,
        .expected_whoami = PROD_ID,
        .handle = NULL,
    };

    esp_err_t err = vigilant_i2c_add_device(&dev->i2c_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        goto err_add_device;
    }

    if (!mmc5983ma_is_connected(dev)) {
        ESP_LOGE(TAG, "MMC5983MA not detected at address 0x%02X",
                 MMC5983MA_I2C_ADDRESS);
        goto err_not_connected;
    }

    err = mmc5983ma_set_filter_bandwidth(dev, dev->config.filter_bandwidth_hz);
    if (err != ESP_OK) {
        goto err_configure;
    }

    err = mmc5983ma_set_continuous_frequency(
        dev, dev->config.continuous_frequency_hz);
    if (err != ESP_OK) {
        goto err_configure;
    }

    if (dev->config.automatic_set_reset) {
        err = mmc5983ma_enable_automatic_set_reset(dev);
        if (err != ESP_OK) {
            goto err_configure;
        }
    }

    if (dev->config.continuous_mode) {
        err = mmc5983ma_enable_continuous_mode(dev);
        if (err != ESP_OK) {
            goto err_configure;
        }

        esp_event_loop_args_t loop_args = {
            .queue_size = dev->config.event_queue_size,
            .task_name = NULL,  // no dedicated loop task; driven in the task
        };
        err = esp_event_loop_create(&loop_args, &dev->event_loop);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create event loop: %s",
                     esp_err_to_name(err));
            goto err_configure;
        }

        BaseType_t task_err = xTaskCreate(
            mmc5983ma_task_entry, "mmc5983ma", dev->config.task_stack_size, dev,
            dev->config.task_priority, &dev->task_handle);
        if (task_err != pdPASS) {
            ESP_LOGE(TAG, "Failed to create measurement task");
            goto err_task_create;
        }
    }

    ESP_LOGI(TAG, "MMC5983MA initialized (BW=%u Hz, continuous=%s)",
             dev->config.filter_bandwidth_hz,
             dev->config.continuous_mode ? "on" : "off");
    return dev;

err_task_create:
    esp_event_loop_delete(dev->event_loop);
    dev->event_loop = NULL;
err_configure:
err_not_connected:
    vigilant_i2c_remove_device(&dev->i2c_device);
err_add_device:
    free(dev);
    return NULL;
}

esp_err_t mmc5983ma_deinit(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (handle->task_handle) {
        vTaskDelete(handle->task_handle);
        handle->task_handle = NULL;
    }
    if (handle->event_loop) {
        esp_event_loop_delete(handle->event_loop);
        handle->event_loop = NULL;
    }

    esp_err_t err = vigilant_i2c_remove_device(&handle->i2c_device);
    free(handle);
    return err;
}

bool mmc5983ma_is_connected(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }

    uint8_t id = 0;
    esp_err_t err =
        vigilant_i2c_read_regs(&handle->i2c_device, PROD_ID_REG, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read product ID: %s", esp_err_to_name(err));
        return false;
    }
    return id == PROD_ID;
}

esp_err_t mmc5983ma_soft_reset(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the SW_RST bit to perform a software reset through the shadow
    // register (read-modify-write would also set reserved and BW_0 bits).
    esp_err_t err = set_shadow_bit(handle, INT_CTRL_1_REG, SW_RST, true);

    // A soft reset clears all registers.
    clear_shadow_bit(handle, INT_CTRL_0_REG, 0xFF, false);
    clear_shadow_bit(handle, INT_CTRL_1_REG, 0xFF, false);
    clear_shadow_bit(handle, INT_CTRL_2_REG, 0xFF, false);
    clear_shadow_bit(handle, INT_CTRL_3_REG, 0xFF, false);

    // The reset time is 10 ms but we wait 15 ms just in case.
    vTaskDelay(pdMS_TO_TICKS(15));

    return err;
}

esp_err_t mmc5983ma_set_filter_bandwidth(mmc5983ma_handle_t handle,
                                         uint16_t bandwidth) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    switch (bandwidth) {
        case 800:  // BW[1:0] = 11
            err = set_shadow_bit(handle, INT_CTRL_1_REG, BW0, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_1_REG, BW1, true);
            break;

        case 400:  // BW[1:0] = 10
            err = clear_shadow_bit(handle, INT_CTRL_1_REG, BW0, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_1_REG, BW1, true);
            break;

        case 200:  // BW[1:0] = 01
            err = set_shadow_bit(handle, INT_CTRL_1_REG, BW0, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_1_REG, BW1, true);
            break;

        case 100:  // BW[1:0] = 00
            err = clear_shadow_bit(handle, INT_CTRL_1_REG, BW0, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_1_REG, BW1, true);
            break;

        default:
            ESP_LOGE(TAG, "Invalid filter bandwidth: %u", bandwidth);
            err = ESP_ERR_INVALID_ARG;
            break;
    }

    return err;
}

uint16_t mmc5983ma_get_filter_bandwidth(mmc5983ma_handle_t handle) {
    if (!handle) {
        return 0;
    }

    bool bw0 = is_shadow_bit_set(handle, INT_CTRL_1_REG, BW0);
    bool bw1 = is_shadow_bit_set(handle, INT_CTRL_1_REG, BW1);

    uint8_t value = (bw1 ? 2 : 0) + (bw0 ? 1 : 0);
    switch (value) {
        case 1:
            return 200;
        case 2:
            return 400;
        case 3:
            return 800;
        default:
            return 100;
    }
}

esp_err_t mmc5983ma_enable_continuous_mode(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_2_REG, CMM_EN, true);
}

esp_err_t mmc5983ma_disable_continuous_mode(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return clear_shadow_bit(handle, INT_CTRL_2_REG, CMM_EN, true);
}

bool mmc5983ma_is_continuous_mode_enabled(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    return is_shadow_bit_set(handle, INT_CTRL_2_REG, CMM_EN);
}

esp_err_t mmc5983ma_set_continuous_frequency(mmc5983ma_handle_t handle,
                                             uint16_t frequency) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    switch (frequency) {
        case 1:  // CM_FREQ[2:0] = 001
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 10:  // CM_FREQ[2:0] = 010
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 20:  // CM_FREQ[2:0] = 011
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 50:  // CM_FREQ[2:0] = 100
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 100:  // CM_FREQ[2:0] = 101
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 200:  // CM_FREQ[2:0] = 110
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 1000:  // CM_FREQ[2:0] = 111
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        case 0:  // CM_FREQ[2:0] = 000 (off)
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, CM_FREQ_0, true);
            break;

        default:
            ESP_LOGE(TAG, "Invalid continuous frequency: %u", frequency);
            err = ESP_ERR_INVALID_ARG;
            break;
    }

    return err;
}

uint16_t mmc5983ma_get_continuous_frequency(mmc5983ma_handle_t handle) {
    if (!handle) {
        return 0;
    }

    // INT_CTRL_2_REG cannot be read, so evaluate the shadow memory.
    uint8_t value = handle->shadow.internal_control2 & 0x07;
    switch (value) {
        case 0x01:
            return 1;
        case 0x02:
            return 10;
        case 0x03:
            return 20;
        case 0x04:
            return 50;
        case 0x05:
            return 100;
        case 0x06:
            return 200;
        case 0x07:
            return 1000;
        default:
            return 0;
    }
}

esp_err_t mmc5983ma_enable_periodic_set(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_2_REG, EN_PRD_SET, true);
}

esp_err_t mmc5983ma_disable_periodic_set(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return clear_shadow_bit(handle, INT_CTRL_2_REG, EN_PRD_SET, true);
}

bool mmc5983ma_is_periodic_set_enabled(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    return is_shadow_bit_set(handle, INT_CTRL_2_REG, EN_PRD_SET);
}

esp_err_t mmc5983ma_set_periodic_set_samples(mmc5983ma_handle_t handle,
                                             uint16_t number_of_samples) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_OK;
    switch (number_of_samples) {
        case 25:  // PRD_SET[2:0] = 001
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 75:  // PRD_SET[2:0] = 010
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 100:  // PRD_SET[2:0] = 011
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 250:  // PRD_SET[2:0] = 100
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 500:  // PRD_SET[2:0] = 101
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 1000:  // PRD_SET[2:0] = 110
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 2000:  // PRD_SET[2:0] = 111
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = set_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        case 1:  // PRD_SET[2:0] = 000
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_2, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_1, false);
            if (err != ESP_OK) {
                break;
            }
            err = clear_shadow_bit(handle, INT_CTRL_2_REG, PRD_SET_0, true);
            break;

        default:
            ESP_LOGE(TAG, "Invalid periodic set samples: %u",
                     number_of_samples);
            err = ESP_ERR_INVALID_ARG;
            break;
    }

    return err;
}

uint16_t mmc5983ma_get_periodic_set_samples(mmc5983ma_handle_t handle) {
    if (!handle) {
        return 0;
    }

    // INT_CTRL_2_REG cannot be read, so evaluate the shadow memory.
    uint8_t value = handle->shadow.internal_control2 & 0x70;
    switch (value) {
        case 0x10:
            return 25;
        case 0x20:
            return 75;
        case 0x30:
            return 100;
        case 0x40:
            return 250;
        case 0x50:
            return 500;
        case 0x60:
            return 1000;
        case 0x70:
            return 2000;
        default:
            return 1;
    }
}

esp_err_t mmc5983ma_perform_set_operation(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the SET bit to perform a set operation through the shadow register.
    esp_err_t err = set_shadow_bit(handle, INT_CTRL_0_REG, SET_OPERATION, true);

    clear_shadow_bit(handle, INT_CTRL_0_REG, SET_OPERATION, false);

    // Wait for the set operation to complete (500 ns).
    vTaskDelay(pdMS_TO_TICKS(1));

    return err;
}

esp_err_t mmc5983ma_perform_reset_operation(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the RESET bit to perform a reset operation through the shadow
    // register.
    esp_err_t err =
        set_shadow_bit(handle, INT_CTRL_0_REG, RESET_OPERATION, true);

    clear_shadow_bit(handle, INT_CTRL_0_REG, RESET_OPERATION, false);

    // Wait for the reset operation to complete (500 ns).
    vTaskDelay(pdMS_TO_TICKS(1));

    return err;
}

esp_err_t mmc5983ma_enable_automatic_set_reset(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_0_REG, AUTO_SR_EN, true);
}

esp_err_t mmc5983ma_disable_automatic_set_reset(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return clear_shadow_bit(handle, INT_CTRL_0_REG, AUTO_SR_EN, true);
}

bool mmc5983ma_is_automatic_set_reset_enabled(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    return is_shadow_bit_set(handle, INT_CTRL_0_REG, AUTO_SR_EN);
}

esp_err_t mmc5983ma_enable_x_channel(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    // The inhibit bit must be cleared so the X channel is enabled.
    return clear_shadow_bit(handle, INT_CTRL_1_REG, X_INHIBIT, true);
}

esp_err_t mmc5983ma_disable_x_channel(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_1_REG, X_INHIBIT, true);
}

bool mmc5983ma_is_x_channel_enabled(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    // Note: this returns true when the X channel is inhibited. Strictly, it
    // should be called is_x_channel_inhibited.
    return is_shadow_bit_set(handle, INT_CTRL_1_REG, X_INHIBIT);
}

esp_err_t mmc5983ma_enable_yz_channels(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    // The inhibit bits must be cleared so the Y and Z channels are enabled.
    return clear_shadow_bit(handle, INT_CTRL_1_REG, YZ_INHIBIT, true);
}

esp_err_t mmc5983ma_disable_yz_channels(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_1_REG, YZ_INHIBIT, true);
}

bool mmc5983ma_are_yz_channels_enabled(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    // Note: this returns true when the Y and Z channels are inhibited.
    // Strictly, it should be called are_yz_channels_inhibited.
    return is_shadow_bit_set(handle, INT_CTRL_1_REG, YZ_INHIBIT);
}

esp_err_t mmc5983ma_enable_interrupt(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_0_REG, INT_MEAS_DONE_EN, true);
}

esp_err_t mmc5983ma_disable_interrupt(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return clear_shadow_bit(handle, INT_CTRL_0_REG, INT_MEAS_DONE_EN, true);
}

bool mmc5983ma_is_interrupt_enabled(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    return is_shadow_bit_set(handle, INT_CTRL_0_REG, INT_MEAS_DONE_EN);
}

esp_err_t mmc5983ma_apply_extra_current_pos_to_neg(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_3_REG, ST_ENP, true);
}

esp_err_t mmc5983ma_remove_extra_current_pos_to_neg(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return clear_shadow_bit(handle, INT_CTRL_3_REG, ST_ENP, true);
}

bool mmc5983ma_is_extra_current_applied_pos_to_neg(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    return is_shadow_bit_set(handle, INT_CTRL_3_REG, ST_ENP);
}

esp_err_t mmc5983ma_apply_extra_current_neg_to_pos(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return set_shadow_bit(handle, INT_CTRL_3_REG, ST_ENM, true);
}

esp_err_t mmc5983ma_remove_extra_current_neg_to_pos(mmc5983ma_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    return clear_shadow_bit(handle, INT_CTRL_3_REG, ST_ENM, true);
}

bool mmc5983ma_is_extra_current_applied_neg_to_pos(mmc5983ma_handle_t handle) {
    if (!handle) {
        return false;
    }
    return is_shadow_bit_set(handle, INT_CTRL_3_REG, ST_ENM);
}

esp_err_t mmc5983ma_read_raw(mmc5983ma_handle_t handle, uint32_t* x,
                             uint32_t* y, uint32_t* z) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the TM_M bit to start the measurement through the shadow register.
    // Read-modify-write would also set the Auto_SR_en bit, which always reads
    // back as 1.
    esp_err_t err = set_shadow_bit(handle, INT_CTRL_0_REG, TM_M, true);
    if (err != ESP_OK) {
        clear_shadow_bit(handle, INT_CTRL_0_REG, TM_M, false);
        return err;
    }

    // Wait until the measurement is completed or times out. A timeout is rare
    // but possible, so we cannot wait forever.
    uint16_t timeout = get_timeout(handle);
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    } while (!mmc_is_bit_set(handle, STATUS_REG, MEAS_M_DONE) && timeout > 0);

    clear_shadow_bit(handle, INT_CTRL_0_REG, TM_M, false);

    // Read the fields even if a timeout occurred - old data vs no data.
    err = read_fields_xyz(handle, x, y, z);
    if (err != ESP_OK) {
        return err;
    }

    return timeout > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t mmc5983ma_get_temperature(mmc5983ma_handle_t handle,
                                    int* temperature_c) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!temperature_c) {
        return ESP_ERR_INVALID_ARG;
    }

    // Set the TM_T bit to start the temperature conversion through the shadow
    // register (see mmc5983ma_read_raw for details).
    esp_err_t err = set_shadow_bit(handle, INT_CTRL_0_REG, TM_T, true);
    if (err != ESP_OK) {
        clear_shadow_bit(handle, INT_CTRL_0_REG, TM_T, false);
        return err;
    }

    // Wait until the measurement is completed, timeout after 5 ms.
    uint8_t timeout = 5;
    do {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    } while (!mmc_is_bit_set(handle, STATUS_REG, MEAS_T_DONE) && timeout > 0);

    clear_shadow_bit(handle, INT_CTRL_0_REG, TM_T, false);

    // Get the raw temperature value even if a timeout occurred - old data vs
    // no data.
    uint8_t result = 0;
    err = vigilant_i2c_read_regs(&handle->i2c_device, T_OUT_REG, &result, 1);
    if (err != ESP_OK) {
        return err;
    }

    // Convert it using the equation provided in the datasheet.
    float temperature = -75.0f + ((float)result * (200.0f / 255.0f));

    // Return the integer part of the temperature.
    *temperature_c = (int)temperature;
    return ESP_OK;
}

esp_err_t mmc5983ma_clear_meas_done_interrupt(mmc5983ma_handle_t handle,
                                              uint8_t meas_mask) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // Ensure only the Meas_T_Done and Meas_M_Done interrupts can be cleared.
    meas_mask &= (MMC5983MA_MEAS_T_DONE | MMC5983MA_MEAS_M_DONE);

    // Writing 1 into these bits will clear the corresponding interrupt.
    // Read-modify-write is OK here since STATUS_REG is readable.
    uint8_t value = 0;
    esp_err_t err =
        vigilant_i2c_read_regs(&handle->i2c_device, STATUS_REG, &value, 1);
    if (err != ESP_OK) {
        return err;
    }
    value |= meas_mask;
    return vigilant_i2c_write_regs(&handle->i2c_device, STATUS_REG, &value, 1);
}

esp_err_t mmc5983ma_add_handler(mmc5983ma_handle_t handle,
                                esp_event_handler_t event_handler,
                                void* handler_args) {
    if (!handle || !handle->event_loop) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_event_handler_register_with(handle->event_loop, MMC5983MA_EVENT,
                                           ESP_EVENT_ANY_ID, event_handler,
                                           handler_args);
}

esp_err_t mmc5983ma_remove_handler(mmc5983ma_handle_t handle,
                                   esp_event_handler_t event_handler) {
    if (!handle || !handle->event_loop) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_event_handler_unregister_with(
        handle->event_loop, MMC5983MA_EVENT, ESP_EVENT_ANY_ID, event_handler);
}
