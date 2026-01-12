#pragma once

#include <stdint.h>
#include <stdbool.h>



int touch_map_value(int val, int in_min, int in_max, int out_min, int out_max);
void touch_get_calibrated_point(int16_t xraw, int16_t yraw, int16_t* x_out, int16_t* y_out);
bool touch_read(uint16_t* x_out, uint16_t* y_out);
bool touch_panel_is_touched(void);

void spi_bus_config();

void touch_io_config();
void touch_panel_config();
