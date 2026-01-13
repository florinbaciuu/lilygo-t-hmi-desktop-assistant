#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

//code aici 


bool s_lvgl_port_init_locking_mutex(void);
bool s_lvgl_lock(uint32_t timeout_ms);
bool ss_lvgl_lock(TickType_t timeout);
void s_lvgl_unlock(void);


#ifdef __cplusplus
}
#endif