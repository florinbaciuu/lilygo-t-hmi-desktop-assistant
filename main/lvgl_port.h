#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

//code aici 


bool s_lvgl_port_init_locking_mutex(void);
bool s_lvgl_lock(uint32_t timeout_ms);
void s_lvgl_unlock(void);




#ifdef __cplusplus
}
#endif