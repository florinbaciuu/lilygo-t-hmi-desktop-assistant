#pragma once

/*********************
 *    LVGL CONFIGS
 *********************/

#define TICK_INCREMENTATION 5  // in ms(milliseconds) must be equal incrementation with delay
#define LV_TIMER_INCREMENT  TICK_INCREMENTATION * 1000
#define LV_DELAY            TICK_INCREMENTATION
#define LV_NO_OOP() do {} while(0)

//-----------------------------------------------------------
#define LV_TICK_SOURCE_TIMER    0
#define LV_TICK_SOURCE_TASK     1
#define LV_TICK_SOURCE_CALLBACK 2
#ifndef LV_TICK_SOURCE
#    define LV_TICK_SOURCE (LV_TICK_SOURCE_TASK)
#endif /* #ifndef LV_TICK_SOURCE */
//---------
//---------

/*Where flush_ready must to go :
 in display_flush or in io_trans_done_cb*/

// #define flush_ready_in_disp_flush // nu e asa bun
#define flush_ready_in_io_trans_done  // mult mai bine asa deoarece se da flush ready in momentul cand display face trans_done
//---------

/* BUFFER MODE */
#define BUFFER_20LINES     1
#define BUFFER_40LINES     2
#define BUFFER_60LINES     3  // merge
#define BUFFER_DEVIDED4    4
#define BUFFER_FULL        5            // merge super ok
#define BUFFER_MODE        BUFFER_FULL  // selecteaza modul de buffer , defaut este BUFFER_FULL
#define DOUBLE_BUFFER_MODE true
//---------
/* BUFFER MEMORY TYPE AND DMA */
#define BUFFER_INTERNAL 1
#define BUFFER_SPIRAM   2
#define BUFFER_MEM      BUFFER_SPIRAM
#if (BUFFER_MEM == BUFFER_INTERNAL)
#    define DMA_ON (true)
#endif /* #if (BUFFER_MEM == BUFFER_INTERNAL) */
//---------

//-----------------------------------------------------------
/* RENDER MODE */
// Aici e mod mai special pt ca se transmite direct functiei....
#define RENDER_MODE_PARTIAL (LV_DISPLAY_RENDER_MODE_PARTIAL)
#define RENDER_MODE_FULL    (LV_DISPLAY_RENDER_MODE_FULL)
#define RENDER_MODE_DIRECT  (LV_DISPLAY_RENDER_MODE_DIRECT)

#define RENDER_MODE (RENDER_MODE_PARTIAL)
//--------------------- --------------------------------------

/*********************
 *   END LVGL CONFIGS
 *********************/