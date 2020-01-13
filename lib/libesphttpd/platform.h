#ifndef PLATFORM_H
#define PLATFORM_H

#include <esp.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

//#include "esp_timer.h"
typedef struct RtosConnType RtosConnType;
typedef RtosConnType* ConnTypePtr;

// freertos v8 api
typedef TimerHandle_t HttpdPlatTimerHandle;

#define ICACHE_FLASH_ATTR

#endif
