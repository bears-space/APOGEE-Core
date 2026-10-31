#pragma once

#include <unistd.h>

#include "WSEN_ISDS_2536030320001.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "platform.h"
#include "vigilant.h"

bool imu_low_g_init(void);
bool imu_low_g_read(float* xAcc, float* yAcc, float* zAcc, float* xRate,
                    float* yRate, float* zRate, float* temperature);
