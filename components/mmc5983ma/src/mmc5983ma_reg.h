/*
 * SPDX-FileCopyrightText: 2026 STARSTREAK
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register and bit definitions for the MMC5983MA. Ported from the SparkFun
 * MMC5983MA Magnetometer Arduino Library constants header.
 */

#pragma once

#include <stdint.h>

/* Register addresses */
#define X_OUT_0_REG 0x00
#define X_OUT_1_REG 0x01
#define Y_OUT_0_REG 0x02
#define Y_OUT_1_REG 0x03
#define Z_OUT_0_REG 0x04
#define Z_OUT_1_REG 0x05
#define XYZ_OUT_2_REG 0x06
#define T_OUT_REG 0x07
#define STATUS_REG 0x08
#define INT_CTRL_0_REG 0x09
#define INT_CTRL_1_REG 0x0A
#define INT_CTRL_2_REG 0x0B
#define INT_CTRL_3_REG 0x0C
#define PROD_ID_REG 0x2F

/* Constants */
#define PROD_ID 0x30

/* STATUS_REG bits */
#define MEAS_M_DONE (1 << 0)
#define MEAS_T_DONE (1 << 1)
#define OTP_READ_DONE (1 << 4)

/* INT_CTRL_0_REG bits */
#define TM_M (1 << 0)
#define TM_T (1 << 1)
#define INT_MEAS_DONE_EN (1 << 2)
#define SET_OPERATION (1 << 3)
#define RESET_OPERATION (1 << 4)
#define AUTO_SR_EN (1 << 5)
#define OTP_READ (1 << 6)

/* INT_CTRL_1_REG bits */
#define BW0 (1 << 0)
#define BW1 (1 << 1)
#define X_INHIBIT (1 << 2)
#define YZ_INHIBIT (3 << 3)
#define SW_RST (1 << 7)

/* INT_CTRL_2_REG bits */
#define CM_FREQ_0 (1 << 0)
#define CM_FREQ_1 (1 << 1)
#define CM_FREQ_2 (1 << 2)
#define CMM_EN (1 << 3)
#define PRD_SET_0 (1 << 4)
#define PRD_SET_1 (1 << 5)
#define PRD_SET_2 (1 << 6)
#define EN_PRD_SET (1 << 7)

/* INT_CTRL_3_REG bits */
#define ST_ENP (1 << 1)
#define ST_ENM (1 << 2)
