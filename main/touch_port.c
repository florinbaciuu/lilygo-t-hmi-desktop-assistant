

#include "touch_port.h"
#include "board_pins.h"

#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"

#include "esp_log.h"

esp_lcd_panel_io_handle_t touch_io_handle = NULL;
esp_lcd_touch_handle_t    touch_handle    = NULL;

/**********************
 *   TOUCH DRIVER VARIABLES
 **********************/
int16_t  touch_map_x1 = 3857;
int16_t  touch_map_x2 = 239;
int16_t  touch_map_y1 = 213;
int16_t  touch_map_y2 = 3693;
uint16_t x            = 0;
uint16_t y            = 0;
uint8_t  num_points   = 0;

/**********************
 *   TOUCH DRIVER FUNCTIONS
 **********************/
int touch_map_value(int val, int in_min, int in_max, int out_min, int out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//---------
void touch_get_calibrated_point(int16_t xraw, int16_t yraw, int16_t* x_out, int16_t* y_out) {
    *x_out = touch_map_value(xraw, touch_map_x1, touch_map_x2, 0, LCD_WIDTH - 1);
    *y_out = touch_map_value(yraw, touch_map_y1, touch_map_y2, 0, LCD_HEIGHT - 1);
    if (*x_out < 0) {
        *x_out = 0;
    }
    if (*x_out >= LCD_WIDTH) {
        *x_out = LCD_WIDTH - 1;
    }
    if (*y_out < 0) {
        *y_out = 0;
    }
    if (*y_out >= LCD_HEIGHT) {
        *y_out = LCD_HEIGHT - 1;
    }
}
//---------
bool touch_read(uint16_t* x_out, uint16_t* y_out) {
    uint16_t x_raw = 0, y_raw = 0;
    uint8_t  point_count = 0;
    esp_lcd_touch_read_data(touch_handle);
    bool touched = esp_lcd_touch_get_coordinates(touch_handle, &x_raw, &y_raw, NULL, &point_count, 1);
    if (touched && point_count > 0) {
        int16_t x_cal, y_cal;
        touch_get_calibrated_point(x_raw, y_raw, &x_cal, &y_cal);
        if (x_out) {
            *x_out = x_cal;
        }
        if (y_out) {
            *y_out = y_cal;
        }
        return true;
    }
    return false;
}
//---------
bool touch_panel_is_touched(void) {
    uint16_t x_raw = 0, y_raw = 0;
    uint8_t  point_count = 0;
    esp_lcd_touch_read_data(touch_handle);
    bool is_touched = esp_lcd_touch_get_coordinates(touch_handle, &x_raw, &y_raw, NULL, &point_count, 1);
    return is_touched && point_count > 0;
}
//---------

// ==================================================
void spi_bus_config() {
    //  Configurare SPI Touch IO
    spi_bus_config_t buscfg = {.mosi_io_num = (int) PIN_NUM_MOSI,
        .miso_io_num                        = (int) PIN_NUM_MISO,
        .sclk_io_num                        = (int) PIN_NUM_CLK,
        .quadwp_io_num                      = (int) (-1),
        .quadhd_io_num                      = (int) (-1),
        .data4_io_num                       = (int) (-1),
        .data5_io_num                       = (int) (-1),
        .data6_io_num                       = (int) (-1),
        .data7_io_num                       = (int) (-1),
        .data_io_default_level              = false,
        .max_transfer_sz                    = (int) 4096,
        .flags                              = (int) 0,
        .isr_cpu_id                         = (esp_intr_cpu_affinity_t) 0,
        .intr_flags                         = (int) 0};
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI("LVGL", "SPI bus initialized");
}
// ==================================================
void touch_io_config() {
    // Configurare IO pentru touch
    esp_lcd_panel_io_spi_config_t touch_io_config = {.cs_gpio_num = (gpio_num_t) PIN_NUM_CS,
        .dc_gpio_num                                              = GPIO_NUM_NC,
        .spi_mode                                                 = (int) 0,
        .pclk_hz                                                  = ESP_LCD_TOUCH_SPI_CLOCK_HZ,
        .trans_queue_depth                                        = 3,
        .on_color_trans_done                                      = NULL,
        .user_ctx                                                 = NULL,
        .lcd_cmd_bits                                             = 8,
        .lcd_param_bits                                           = 8,
        .cs_ena_pretrans                                          = 0,
        .cs_ena_posttrans                                         = 0,
        .flags                                                    = {.dc_high_on_cmd = 0,
                                                               .dc_low_on_data       = 0,
                                                               .dc_low_on_param      = 0,
                                                               .octal_mode           = 0,
                                                               .quad_mode            = 0,
                                                               .sio_mode             = 0,
                                                               .lsb_first            = 0,
                                                               .cs_high_active       = 0}};
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &touch_io_config, &touch_io_handle));
    ESP_LOGI("LVGL", "Touch panel IO created");
}

// ==================================================

void touch_panel_config() {
    // Configurare driver touch
    esp_lcd_touch_config_t touch_config = {.x_max = 4095,
        .y_max                                    = 4095,
        .rst_gpio_num                             = (gpio_num_t) -1,
        .int_gpio_num                             = (gpio_num_t) PIN_NUM_IRQ,
        .levels                                   = {.reset = 0, .interrupt = 0},
        .flags                                    = {.swap_xy = true, .mirror_x = false, .mirror_y = false},
        .process_coordinates                      = NULL,
        .interrupt_callback                       = NULL,
        .user_data                                = NULL,
        .driver_data                              = NULL};
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(touch_io_handle, &touch_config, &touch_handle));
    ESP_LOGI("LVGL", "Touch panel created");
}

// ==================================================