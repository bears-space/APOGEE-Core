#pragma once

#include <unistd.h>

#include "WSEN_HIDS_2525020210002.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "platform.h"
#include "vigilant.h"

esp_err_t thermometer_sensor_init(void);