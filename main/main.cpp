

//---------

/*********************
 *      INCLUDES
 *********************/
extern "C" {
#include "CONFIG.h"
#include "board_pins.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_rom_sys.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lvgl.h"
#include <lv_conf.h>

#include "lvgl_port.h"
#include "display_port.h"
#include "touch_port.h"

// my include
#include "filesystem-os.h"
#include "ui.h"
}
/**********************
 *   GLOBAL VARIABLES
 **********************/


//---------

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
//---------
void gpio_extra_set_init(uint32_t mode) {
    // Setăm ambii pini ca output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PWR_EN_PIN) | (1ULL << PWR_ON_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) PWR_EN_PIN, mode);
    gpio_set_level((gpio_num_t) PWR_ON_PIN, mode);  // nu e nevoie de el daca alimentam usb
}
//---------
void power_latch_init() {
    gpio_config_t io_conf = {.pin_bit_mask = 1ULL << PWR_EN_PIN,
        .mode                              = GPIO_MODE_OUTPUT,
        .pull_up_en                        = GPIO_PULLUP_DISABLE,
        .pull_down_en                      = GPIO_PULLDOWN_DISABLE,
        .intr_type                         = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) PWR_EN_PIN, 1);  // ⚡ ține placa aprinsă
}
//---------
void gfx_set_backlight(uint32_t mode) {
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << BOARD_TFT_BL,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t) BOARD_TFT_BL, mode);
}
//---------
/**********************
 *   DISPLAY FUNCTIONS
 **********************/


/**********************
 *   LVGL VARIABLES
 **********************/


/**********************
 *   LVGL FUNCTIONS
 **********************/
//---------
//---------

//---------
//--------------------------------------

/*
███████ ██████  ███████ ███████ ██████ ████████  ██████  ███████ 
██      ██   ██ ██      ██      ██   ██   ██    ██    ██ ██      
█████   ██████  █████   █████   ██████    ██    ██    ██ ███████ 
██      ██   ██ ██      ██      ██   ██   ██    ██    ██      ██ 
██      ██   ██ ███████ ███████ ██   ██   ██     ██████  ███████ 
*/

/*********************
 *  rtos variables
 *********************/
TaskHandle_t xHandle_lv_main_task;
TaskHandle_t xHandle_lv_main_tick_task;
TaskHandle_t xHandle_chechButton0State;

// -------------------------------

// -------------------------------

/************************************************** */

#if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER

static void lv_tick_timer_callback_func(void* arg) {
    lv_tick_inc((TICK_INCREMENTATION));  // 5 ms ticklv_timer_handler();
}

static void lv_tick_start_timer(void) {
    const esp_timer_create_args_t timer_args = {
        .callback              = &lv_tick_timer_callback_func,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                  = "lv_tick",
        .skip_unhandled_events = true};
    esp_timer_handle_t tick_timer;

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LV_TIMER_INCREMENT));  // 1000 us = 1 ms
}

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_main_task(void* parameter) {
    xHandle_lv_main_task   = xTaskGetCurrentTaskHandle();
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();
    while (true) {
        if (s_lvgl_lock(portMAX_DELAY)) {
            lv_timer_handler();
            // xTaskGenericNotifyFromISR(xHandle_lv_main_task, tskDEFAULT_INDEX_TO_NOTIFY, 0x01, eSetBits);
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY));
    }
}

#elif LV_TICK_SOURCE == LV_TICK_SOURCE_TASK

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_main_tick_task(void* parameter) {
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();
    while (true) {
        lv_tick_inc(TICK_INCREMENTATION);                 // Incrementeaza tick-urile LVGL
        xTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY));  // Delay precis mult mai rapid asa
    }
}

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_main_task(void* parameter) {
    xHandle_lv_main_task   = xTaskGetCurrentTaskHandle();
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();  // Inițializare corectă
    while (true) {
        if (s_lvgl_lock(portMAX_DELAY)) {  // <— ADD
            lv_timer_handler();
            s_lvgl_unlock();  // <— ADD
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY));
    }
}

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_ui_task(void* arg) {
    // s_lvgl_lock(portMAX_DELAY);
    ss_lvgl_lock(portMAX_DELAY);
    create_tabs_ui();
    s_lvgl_unlock();
    vTaskDelete(NULL);  // moare dupa creare
}

#elif LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK
/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_main_task(void* parameter) {
    xHandle_lv_main_task   = xTaskGetCurrentTaskHandle();
    static TickType_t tick = 0;
    tick                   = xTaskGetTickCount();  // Inițializare corectă
    while (true) {
        if (s_lvgl_lock(portMAX_DELAY)) {  // <— ADD
            lv_timer_handler();
            s_lvgl_unlock();  // <— ADD
        }
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(LV_DELAY));
    }
}

#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER */

/********************************************** */
/*                   TASK                       */
/********************************************** */
static void IRAM_ATTR chechButton0State_isr_handler(void* arg) {
    // NOTĂ: NU face log sau delay aici
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR((TaskHandle_t) arg, 0x01, eSetBits, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}
//---------
void chechButton0State(void* parameter) {
    (void) parameter;
    xHandle_chechButton0State = xTaskGetCurrentTaskHandle();  // Încoronarea oficială
    uint32_t      notificationValue;
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << 0,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,  // activăm pull-up intern
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,  // întrerupere pe orice schimbare de stare
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);  // doar o dată în tot proiectul
    gpio_isr_handler_add((gpio_num_t) 0, chechButton0State_isr_handler, (void*) xHandle_chechButton0State);
    while (true) {
        xTaskNotifyWait(0x00, 0xFFFFFFFF, &notificationValue, portMAX_DELAY);
        if (notificationValue & 0x01) {
            ESP_LOGW("BUTTON", "Button ACTIVAT pe GPIO0");
            //// Acțiune custom (PUNE COD AICI)
        }
        vTaskDelay(200);
    }
}
/****************************/

//--------------------------------------

/*
███    ███  █████  ██ ███    ██ 
████  ████ ██   ██ ██ ████   ██ 
██ ████ ██ ███████ ██ ██ ██  ██ 
██  ██  ██ ██   ██ ██ ██  ██ ██ 
██      ██ ██   ██ ██ ██   ████ 
  * This is the main entry point of the application.
  * It initializes the hardware, sets up the display, and starts the LVGL tasks.
  * The application will run indefinitely until the device is powered off or reset.
*/
extern "C" void app_main(void) {
    power_latch_init();  // Inițializare latch pentru alimentare
    gfx_set_backlight(1);
    esp_log_level_set("*", ESP_LOG_INFO);

    // NVS FIRST, ALWAYS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    lv_init();

#if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK
    // Next function comment because create problems with lvgl timers and esp32 timers
    lv_tick_set_cb(lv_get_rtos_tick_count_callback);
#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_CALLBACK */

    disp = lv_display_create(
        (int32_t) LCD_WIDTH,
        (int32_t) LCD_HEIGHT);

    
    lvgl_set_buffers_config(); // configure buffers based on CONFIG settings

    lv_display_set_resolution(disp, LCD_WIDTH, LCD_HEIGHT);           // Seteaza rezolutia software
    lv_display_set_physical_resolution(disp, LCD_WIDTH, LCD_HEIGHT);  // Actualizeaza rezolutia reala
    lv_display_set_rotation(disp, (lv_display_rotation_t) 0);         // Seteaza rotatia lvgl
    lv_display_set_render_mode(disp,
        (lv_display_render_mode_t) RENDER_MODE);  // Seteaza (lv_display_render_mode_t)
    lv_display_set_antialiasing(disp, true);      // Antialiasing DA sau NU
    ESP_LOGI("LVGL", "LVGL display settings done");

    lv_display_set_flush_cb(disp, lv_disp_flush);  // Set the flush callback which will be called to
                                                   // copy the rendered image to the display.
    ESP_LOGI("LVGL", "LVGL display flush callback set");

    lv_indev_t* indev = lv_indev_create();           /*Initialize the (dummy) input device driver*/
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER); /*Touchpad should have POINTER type*/
    ////lv_indev_set_read_cb(indev, lv_touchpad_read);    // old version
    lv_indev_set_read_cb(indev, lv_touchpad_read_v2);
    ESP_LOGI("LVGL", "LVGL Setup done");

    s_lvgl_port_init_locking_mutex();


    display_bus_config();
    display_io_i80_config();
    display_panel_config();

    spi_bus_config();
    touch_io_config();
    touch_panel_config();

    vTaskDelay(500);

    init_filesystem_sys();
    // initialize_filesystem_sdmmc() ;

    // =================================================================================================
    esp_rom_delay_us(100);
    xTaskCreatePinnedToCore(lv_main_task,  // Functia task-ului
        (const char*) "LVGL Main Task",    // Numele task-ului
        (uint32_t) (8192),                 // Dimensiunea stack-ului
        (NULL),                            // Parametri (daca exista)
        (UBaseType_t) 5,                   // Prioritatea task-ului // 3
        &xHandle_lv_main_task,             // Handle-ul task-ului
        ((1))                              // Nucleul pe care ruleaza task-ul
    );

#if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER
    lv_tick_start_timer();
#elif LV_TICK_SOURCE == LV_TICK_SOURCE_TASK
    esp_rom_delay_us(100);
    xTaskCreatePinnedToCore(lv_main_tick_task,  // Functia care ruleaza task-ul
        (const char*) "LVGL Tick Task",         // Numele task-ului
        (uint32_t) (3072),                      // Dimensiunea stack-ului
        (NULL),                                 // Parametri
        (UBaseType_t) 3,                        // Prioritatea task-ului // 1
        &xHandle_lv_main_tick_task,             // Handle-ul task-ului
        ((1))                                   // Nucleul pe care ruleaza (ESP32 e dual-core)
    );
#endif /* #if LV_TICK_SOURCE == LV_TICK_SOURCE_TIMER */
    esp_rom_delay_us(100);
    xTaskCreatePinnedToCore(chechButton0State,  // Functia care ruleaza task-ul
        (const char*) "v_check_0_pin_state",    // Numele task-ului
        (uint32_t) (4096),                      // Dimensiunea stack-ului
        (NULL),                                 // Parametri
        (UBaseType_t) 3,                        // Prioritatea task-ului // 6
        &xHandle_chechButton0State,             // Handle-ul task-ului
        ((1))                                   // Nucleul pe care ruleaza (ESP32 e dual-core)
    );
    esp_rom_delay_us(100);

    xTaskCreatePinnedToCore(lv_ui_task,  // Functia care ruleaza task-ul
        (const char*) "ui_task",         // Numele task-ului
        (uint32_t) (4096),               // Dimensiunea stack-ului
        (NULL),                          // Parametri
        (UBaseType_t) 4,                 // Prioritatea task-ului // 6
        NULL,                            // Handle-ul task-ului
        ((1))                            // Nucleul pe care ruleaza (ESP32 e dual-core)
    );

    esp_rom_delay_us(100);
}
