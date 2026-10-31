#pragma once

#include <unistd.h>

#include "WSEN_PADS_2511020213301.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c.h"
#include "platform.h"
#include "vigilant.h"

bool barometer_init(void);
bool barometer_read_pressure(float* pressure);
bool barometer_read_temperature(float* temperature);
