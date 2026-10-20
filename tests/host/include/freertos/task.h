#pragma once
#include "FreeRTOS.h"
typedef void* TaskHandle_t;
typedef enum { eSetBits } eNotifyAction;
BaseType_t xTaskCreate(void (*)(void*), const char*, unsigned, void*, unsigned,
                       TaskHandle_t*);
void vTaskDelete(TaskHandle_t);
void xTaskNotifyGive(TaskHandle_t);
uint32_t ulTaskNotifyTake(BaseType_t, TickType_t);
BaseType_t xTaskNotify(TaskHandle_t, uint32_t, eNotifyAction);
