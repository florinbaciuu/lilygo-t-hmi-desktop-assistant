#include "lvgl_port.h"

// lbgl semafor principal (draw)
SemaphoreHandle_t s_lvgl_mutex;
// static SemaphoreHandle_t s_lvgl_mutex = NULL;

bool s_lvgl_port_init_locking_mutex(void) {
    s_lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    return (s_lvgl_mutex != NULL);
}

bool s_lvgl_lock(uint32_t timeout_ms) {
    if (!s_lvgl_mutex)
        return false;
    return xSemaphoreTakeRecursive(s_lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool ss_lvgl_lock(TickType_t timeout_ms)
{
    if (!s_lvgl_mutex) {
        return false;
    }
    return (xSemaphoreTakeRecursive(s_lvgl_mutex, timeout_ms) == pdTRUE);
}

void s_lvgl_unlock(void) {
    if (s_lvgl_mutex)
        xSemaphoreGiveRecursive(s_lvgl_mutex);
}

// -------------------------------