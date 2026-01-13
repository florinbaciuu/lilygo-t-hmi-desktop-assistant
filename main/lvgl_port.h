#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include <lv_conf.h>
#include "CONFIG.h"

#ifdef __cplusplus
extern "C" {
#endif

// code aici

bool s_lvgl_port_init_locking_mutex(void);
bool s_lvgl_lock(uint32_t timeout_ms);
bool ss_lvgl_lock(TickType_t timeout);  // functie mai buna decat cea de mai sus
void s_lvgl_unlock(void);

/**********************
 *   LVGL FUNCTIONS
 **********************/
void lv_disp_flush(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
void lv_touchpad_read(lv_indev_t* indev_drv, lv_indev_data_t* data);
void lv_touchpad_read_v2(lv_indev_t* indev_drv, lv_indev_data_t* data);


#if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK
uint32_t lv_get_rtos_tick_count_callback(void);
#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK */

void lvgl_set_buffers_config();

// ===========================================  //


void lvgl_port_config(void);
void lvgl_port_init(void);

#ifdef __cplusplus
}
#endif