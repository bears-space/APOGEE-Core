/*
 * SPDX-FileCopyrightText: 2026 STARSTREAK
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * C port of the SparkFun MMC5983MA Magnetometer Arduino Library
 * (MIT License, written by Ricardo Ramos @ SparkFun Electronics, Feb 2022):
 * https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_event.h"

#ifdef __cplusplus
extern "C" {
#endif

/** I2C address of the MMC5983MA (fixed, no address pin). */
#define MMC5983MA_I2C_ADDRESS 0x30

/**
 * @brief Declare the MMC5983MA event base.
 *
 * Events are posted on the driver's own event loop. Register a handler with
 * mmc5983ma_add_handler() to receive them.
 */
ESP_EVENT_DECLARE_BASE(MMC5983MA_EVENT);

/**
 * @brief MMC5983MA event IDs.
 */
typedef enum {
    /** A new measurement is available. event_data: mmc5983ma_measurement_t* */
    MMC5983MA_EVENT_MEASUREMENT = 0,
} mmc5983ma_event_id_t;

/** Bit mask for the magnetic measurement done interrupt (clear function). */
#define MMC5983MA_MEAS_M_DONE (1 << 0)
/** Bit mask for the temperature measurement done interrupt (clear function). */
#define MMC5983MA_MEAS_T_DONE (1 << 1)

/**
 * @brief Raw 18-bit X, Y and Z field strength measurement.
 */
typedef struct {
    uint32_t x; /*!< X axis output, 18-bit (0..262143) */
    uint32_t y; /*!< Y axis output, 18-bit (0..262143) */
    uint32_t z; /*!< Z axis output, 18-bit (0..262143) */
} mmc5983ma_measurement_t;

/**
 * @brief MMC5983MA handle
 */
typedef struct mmc5983ma mmc5983ma_t;
typedef mmc5983ma_t* mmc5983ma_handle_t;

/**
 * @brief Default configuration for the MMC5983MA driver
 *
 * I2C bus: uses the Vigilant Engine I2C interface (address 0x30, product ID
 * check on register 0x2F).
 */
#define MMC5983MA_CONFIG_DEFAULT() \
    {.filter_bandwidth_hz = 100,   \
     .continuous_mode = false,     \
     .continuous_frequency_hz = 0, \
     .automatic_set_reset = false, \
     .event_queue_size = 16,       \
     .task_stack_size = 3072,      \
     .task_priority = 5}

/**
 * @brief Configuration of the MMC5983MA driver
 */
typedef struct {
    /** Decimation filter bandwidth: 100, 200, 400 or 800 Hz. */
    uint16_t filter_bandwidth_hz;
    /** Start the continuous measurement task when the driver is initialized. */
    bool continuous_mode;
    /** Continuous mode frequency: 1, 10, 20, 50, 100, 200, 1000 or 0 (off). */
    uint16_t continuous_frequency_hz;
    /** Enable automatic SET/RESET to clear residual magnetization. */
    bool automatic_set_reset;
    /** Size of the driver event loop queue. */
    uint32_t event_queue_size;
    /** Stack size of the continuous measurement task. */
    uint32_t task_stack_size;
    /** Priority of the continuous measurement task. */
    uint32_t task_priority;
} mmc5983ma_config_t;

/**
 * @brief Init the MMC5983MA driver
 *
 * Adds the device to the Vigilant Engine I2C bus and verifies the product ID.
 * When continuous mode is enabled, a task is started that periodically reads
 * measurements and posts MMC5983MA_EVENT_MEASUREMENT events on the driver's
 * own event loop.
 *
 * @param config Driver configuration, or NULL for the defaults
 * @return mmc5983ma_handle_t handle of the MMC5983MA driver, or NULL on error
 */
mmc5983ma_handle_t mmc5983ma_init(const mmc5983ma_config_t* config);

/**
 * @brief Deinit the MMC5983MA driver
 *
 * Stops the continuous measurement task, deletes the event loop, removes the
 * device from the I2C bus and frees the handle.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_deinit(mmc5983ma_handle_t handle);

/**
 * @brief Check if the MMC5983MA is connected and its product ID matches
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if connected and product ID matches
 */
bool mmc5983ma_is_connected(mmc5983ma_handle_t handle);

/**
 * @brief Soft reset the device
 *
 * Sets the SW_RST bit and waits for the reset time (15 ms). Clears all
 * shadow register contents.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_soft_reset(mmc5983ma_handle_t handle);

/**
 * @brief Set the decimation filter bandwidth
 *
 * Allowed values are 800, 400, 200 and 100 Hz. Defaults to 100 on invalid
 * values.
 *
 * @param handle handle of the MMC5983MA driver
 * @param bandwidth filter bandwidth in Hz
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_set_filter_bandwidth(mmc5983ma_handle_t handle,
                                         uint16_t bandwidth);

/**
 * @brief Get the current decimation filter bandwidth
 *
 * @param handle handle of the MMC5983MA driver
 * @return filter bandwidth in Hz (100, 200, 400 or 800), 0 on invalid handle
 */
uint16_t mmc5983ma_get_filter_bandwidth(mmc5983ma_handle_t handle);

/**
 * @brief Enable continuous mode
 *
 * Note: the driver only runs its own measurement task when continuous mode is
 * enabled at initialization (config.continuous_mode). When enabling it at
 * runtime, read the measurements with mmc5983ma_read_raw().
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_enable_continuous_mode(mmc5983ma_handle_t handle);

/**
 * @brief Disable continuous mode
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_disable_continuous_mode(mmc5983ma_handle_t handle);

/**
 * @brief Check if continuous mode is enabled
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if continuous mode is enabled
 */
bool mmc5983ma_is_continuous_mode_enabled(mmc5983ma_handle_t handle);

/**
 * @brief Set the continuous mode frequency
 *
 * Allowed values are 1000, 200, 100, 50, 20, 10, 1 and 0 (off). Defaults to 0
 * (off) on invalid values.
 *
 * @param handle handle of the MMC5983MA driver
 * @param frequency continuous mode frequency in Hz
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_set_continuous_frequency(mmc5983ma_handle_t handle,
                                             uint16_t frequency);

/**
 * @brief Get the continuous mode frequency
 *
 * @param handle handle of the MMC5983MA driver
 * @return continuous mode frequency in Hz, 0 on invalid handle
 */
uint16_t mmc5983ma_get_continuous_frequency(mmc5983ma_handle_t handle);

/**
 * @brief Enable periodic set
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_enable_periodic_set(mmc5983ma_handle_t handle);

/**
 * @brief Disable periodic set
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_disable_periodic_set(mmc5983ma_handle_t handle);

/**
 * @brief Check if periodic set is enabled
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if periodic set is enabled
 */
bool mmc5983ma_is_periodic_set_enabled(mmc5983ma_handle_t handle);

/**
 * @brief Set how often the chip performs an automatic set operation
 *
 * Allowed values are 1, 25, 75, 100, 250, 500, 1000 and 2000 samples. Defaults
 * to 1 on invalid values.
 *
 * @param handle handle of the MMC5983MA driver
 * @param number_of_samples samples between automatic set operations
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_set_periodic_set_samples(mmc5983ma_handle_t handle,
                                             uint16_t number_of_samples);

/**
 * @brief Get how many samples the chip performs between automatic set
 * operations
 *
 * @param handle handle of the MMC5983MA driver
 * @return number of samples between automatic set operations
 */
uint16_t mmc5983ma_get_periodic_set_samples(mmc5983ma_handle_t handle);

/**
 * @brief Perform a SET operation
 *
 * This can be used to clear residual magnetization before a measurement.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_perform_set_operation(mmc5983ma_handle_t handle);

/**
 * @brief Perform a RESET operation
 *
 * This can be used to clear residual magnetization before a measurement.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_perform_reset_operation(mmc5983ma_handle_t handle);

/**
 * @brief Enable automatic SET/RESET
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_enable_automatic_set_reset(mmc5983ma_handle_t handle);

/**
 * @brief Disable automatic SET/RESET
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_disable_automatic_set_reset(mmc5983ma_handle_t handle);

/**
 * @brief Check if automatic SET/RESET is enabled
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if automatic SET/RESET is enabled
 */
bool mmc5983ma_is_automatic_set_reset_enabled(mmc5983ma_handle_t handle);

/**
 * @brief Enable the X channel output
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_enable_x_channel(mmc5983ma_handle_t handle);

/**
 * @brief Disable the X channel output
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_disable_x_channel(mmc5983ma_handle_t handle);

/**
 * @brief Check if the X channel output is enabled
 *
 * Note: this returns true when the X channel is inhibited.
 *
 * @param handle handle of the MMC5983MA driver
 * @return true when the X channel is inhibited
 */
bool mmc5983ma_is_x_channel_enabled(mmc5983ma_handle_t handle);

/**
 * @brief Enable the Y and Z channel outputs
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_enable_yz_channels(mmc5983ma_handle_t handle);

/**
 * @brief Disable the Y and Z channel outputs
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_disable_yz_channels(mmc5983ma_handle_t handle);

/**
 * @brief Check if the Y and Z channel outputs are enabled
 *
 * Note: this returns true when the Y and Z channels are inhibited.
 *
 * @param handle handle of the MMC5983MA driver
 * @return true when the Y and Z channels are inhibited
 */
bool mmc5983ma_are_yz_channels_enabled(mmc5983ma_handle_t handle);

/**
 * @brief Enable the interrupt generation after measurement is completed
 *
 * Must be re-enabled after each measurement.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_enable_interrupt(mmc5983ma_handle_t handle);

/**
 * @brief Disable the interrupt generation
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_disable_interrupt(mmc5983ma_handle_t handle);

/**
 * @brief Check if the interrupt generation is enabled
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if the interrupt generation is enabled
 */
bool mmc5983ma_is_interrupt_enabled(mmc5983ma_handle_t handle);

/**
 * @brief Apply extra current from the positive to the negative side of the
 * coil
 *
 * This feature can be used to check whether the sensor has been saturated.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_apply_extra_current_pos_to_neg(mmc5983ma_handle_t handle);

/**
 * @brief Remove the extra current from the positive to the negative side of
 * the coil
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_remove_extra_current_pos_to_neg(mmc5983ma_handle_t handle);

/**
 * @brief Check if the extra current is applied from the positive to the
 * negative side of the coil
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if the extra current is applied
 */
bool mmc5983ma_is_extra_current_applied_pos_to_neg(mmc5983ma_handle_t handle);

/**
 * @brief Apply extra current from the negative to the positive side of the
 * coil
 *
 * This feature can be used to check whether the sensor has been saturated.
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_apply_extra_current_neg_to_pos(mmc5983ma_handle_t handle);

/**
 * @brief Remove the extra current from the negative to the positive side of
 * the coil
 *
 * @param handle handle of the MMC5983MA driver
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_remove_extra_current_neg_to_pos(mmc5983ma_handle_t handle);

/**
 * @brief Check if the extra current is applied from the negative to the
 * positive side of the coil
 *
 * @param handle handle of the MMC5983MA driver
 * @return true if the extra current is applied
 */
bool mmc5983ma_is_extra_current_applied_neg_to_pos(mmc5983ma_handle_t handle);

/**
 * @brief Read the X, Y and Z field strengths in a single measurement
 *
 * Triggers a measurement (TM_M bit), waits for completion (or the timeout
 * derived from the filter bandwidth) and burst-reads the 7 output registers.
 * On timeout the (stale) field strengths are still written to the output
 * parameters.
 *
 * @param handle handle of the MMC5983MA driver
 * @param x X axis output, 18-bit
 * @param y Y axis output, 18-bit
 * @param z Z axis output, 18-bit
 * @return esp_err_t ESP_OK on success, ESP_ERR_TIMEOUT on measurement timeout
 */
esp_err_t mmc5983ma_read_raw(mmc5983ma_handle_t handle, uint32_t* x,
                             uint32_t* y, uint32_t* z);

/**
 * @brief Return the die temperature
 *
 * Triggers a temperature measurement (TM_T bit), waits for completion (or a
 * 5 ms timeout) and converts the raw value using the datasheet equation.
 *
 * @param handle handle of the MMC5983MA driver
 * @param temperature_c output temperature in degrees Celsius (-75..125)
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_get_temperature(mmc5983ma_handle_t handle,
                                    int* temperature_c);

/**
 * @brief Clear the Meas_T_Done and/or Meas_M_Done interrupts
 *
 * Use MMC5983MA_MEAS_T_DONE | MMC5983MA_MEAS_M_DONE to clear both.
 *
 * @param handle handle of the MMC5983MA driver
 * @param meas_mask mask of the interrupts to clear
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error
 */
esp_err_t mmc5983ma_clear_meas_done_interrupt(mmc5983ma_handle_t handle,
                                              uint8_t meas_mask);

/**
 * @brief Add a user handler for MMC5983MA driver events
 *
 * @param handle handle of the MMC5983MA driver
 * @param event_handler user event handler
 * @param handler_args handler specific arguments
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE when the driver
 * event loop does not exist (continuous mode disabled)
 */
esp_err_t mmc5983ma_add_handler(mmc5983ma_handle_t handle,
                                esp_event_handler_t event_handler,
                                void* handler_args);

/**
 * @brief Remove a user handler for MMC5983MA driver events
 *
 * @param handle handle of the MMC5983MA driver
 * @param event_handler user event handler
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_STATE when the driver
 * event loop does not exist (continuous mode disabled)
 */
esp_err_t mmc5983ma_remove_handler(mmc5983ma_handle_t handle,
                                   esp_event_handler_t event_handler);

#ifdef __cplusplus
}
#endif
