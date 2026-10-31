#pragma once

#include <unistd.h>

#include "WSEN_ISDS_2536030320001.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "platform.h"
#include "vigilant.h"


esp_err_t imu_low_g_sensor_init(void);