#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
typedef unsigned TickType_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define portMAX_DELAY UINT32_MAX
#define pvPortMalloc malloc
#define vPortFree free
