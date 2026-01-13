

//---------

/*********************
 *      INCLUDES
 *********************/
extern "C" {
#include "CONFIG.h"
#include "board_pins.h"
#include "board_config.h"
#include "esp_rom_sys.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "lvgl_port.h"
#include "display_port.h"
#include "touch_port.h"
#include "ui.h"

// my include
#include "filesystem-os.h"

}
/**********************
 *   GLOBAL VARIABLES
 **********************/


//---------



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

TaskHandle_t xHandle_chechButton0State;

// -------------------------------

// -------------------------------

/************************************************** */



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

/********************************************** */
/*                   TASK                       */
/********************************************** */
void lv_ui_task(void* arg) {
    // s_lvgl_lock(portMAX_DELAY);
    s_lvgl_lock(portMAX_DELAY);
    create_tabs_ui();
    s_lvgl_unlock();
    vTaskDelete(NULL);  // moare dupa creare
}

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

    s_lvgl_port_init();

    display_bus_config();
    display_io_i80_config();
    display_panel_config();

    spi_bus_config();
    touch_io_config();
    touch_panel_config();

    vTaskDelay(500);

    init_filesystem_sys();
    // initialize_filesystem_sdmmc() ;

    create_and_start_lvgl_tasks(); // freetos tasks for lvgl

    xTaskCreatePinnedToCore(lv_ui_task,  // Functia care ruleaza task-ul
        (const char*) "ui_task",         // Numele task-ului
        (uint32_t) (4096),               // Dimensiunea stack-ului
        (NULL),                          // Parametri
        (UBaseType_t) 4,                 // Prioritatea task-ului // 6
        NULL,                            // Handle-ul task-ului
        ((1))                            // Nucleul pe care ruleaza (ESP32 e dual-core)
    );

    xTaskCreatePinnedToCore(chechButton0State,  // Functia care ruleaza task-ul
        (const char*) "v_check_0_pin_state",    // Numele task-ului
        (uint32_t) (4096),                      // Dimensiunea stack-ului
        (NULL),                                 // Parametri
        (UBaseType_t) 3,                        // Prioritatea task-ului // 6
        &xHandle_chechButton0State,             // Handle-ul task-ului
        ((1))                                   // Nucleul pe care ruleaza (ESP32 e dual-core)
    );
    esp_rom_delay_us(100);
}
