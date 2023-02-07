/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <stdlib.h>
#include <sys/cdefs.h>
#include <sys/param.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_intr_alloc.h"
#include "esp_heap_caps.h"
#include "esp_pm.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_ops.h"
#include "esp_rom_gpio.h"
#include "soc/soc_caps.h"
#include "soc/rtc.h" // for querying XTAL clock

#if SOC_LCDCAM_SUPPORTED
#include "hal/dma_types.h"
#include "hal/gpio_hal.h"
#include "esp_private/gdma.h"
#include "hal/gdma_ll.h"
#include "driver/gpio.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif
#include "qmsd_lcd_common.h"
#include "soc/lcd_periph.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"
#include "soc/gdma_struct.h"
#include <rom/cache.h>

#include "qmsd_lcd_rgb_panel.h"

static const char *TAG = "lcd_panel.rgb";

typedef struct qmsd_rgb_panel_t qmsd_rgb_panel_t;

#define RGB_DMA_BUFFER_SIZE 3968

// This function is located in ROM (also see esp_rom/${target}/ld/${target}.rom.ld)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

static esp_err_t rgb_panel_del(esp_lcd_panel_t *panel);
static esp_err_t rgb_panel_reset(esp_lcd_panel_t *panel);
static esp_err_t rgb_panel_init(esp_lcd_panel_t *panel);
static esp_err_t rgb_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t rgb_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t rgb_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t rgb_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t rgb_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t rgb_panel_disp_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t lcd_rgb_panel_select_clock_src(qmsd_rgb_panel_t *panel, lcd_clock_source_t clk_src);
static esp_err_t lcd_rgb_panel_create_trans_link(qmsd_rgb_panel_t *panel);
static esp_err_t lcd_rgb_panel_configure_gpio(qmsd_rgb_panel_t *panel, const qmsd_lcd_rgb_panel_config_t *panel_config);
static IRAM_ATTR void lcd_default_isr_handler(void *args);
static IRAM_ATTR void lcd_rgb_panel_start_transmission(qmsd_rgb_panel_t *rgb_panel);

struct qmsd_rgb_panel_t {
    esp_lcd_panel_t base;  // Base class of generic lcd panel
    int panel_id;          // LCD panel ID
    lcd_hal_context_t hal; // Hal layer object
    size_t data_width;     // Number of data lines (e.g. for RGB565, the data width is 16)
    int disp_gpio_num;     // Display control GPIO, which is used to perform action like "disp_off"
    intr_handle_t intr;    // LCD peripheral interrupt handle
    esp_pm_lock_handle_t pm_lock; // Power management lock
    size_t num_dma_nodes;  // Number of DMA descriptors that used to carry the frame buffer
    size_t fb_size;          // Size of frame buffer
    int data_gpio_nums[SOC_LCD_RGB_DATA_WIDTH]; // GPIOs used for data lines, we keep these GPIOs for action like "invert_color"
    uint32_t src_clk_hz;   // Peripheral source clock resolution
    int bounce_buffer_size_bytes;   //If not-zero, the driver uses a bounce buffer in internal memory to DMA from. It's in bytes here.
    uint8_t *bounce_buffer[2];      //Pointer to the bounce buffers
    int bounce_pos_px;                 // Position in whatever source material is used for the bounce buffer, in pixels
    esp_lcd_rgb_timing_t timings;   // RGB timing parameters (e.g. pclk, sync pulse, porch width)
    gdma_channel_handle_t dma_chan; // DMA channel handle
    qmsd_rgb_panel_vsync_cb_t on_vsync; // VSYNC event callback
    int new_frame_id;               // ID for new frame, we use ID to identify whether the frame content has been updated
    int cur_frame_id;               // ID for current transferring frame
    SemaphoreHandle_t done_sem;     // Binary semaphore, indicating if the new frame has been flushed to LCD
    int x_gap;                      // Extra gap in x coordinate, it's used when calculate the flush window
    int y_gap;                      // Extra gap in y coordinate, it's used when calculate the flush window
    portMUX_TYPE spinlock;          // to protect panel specific resource from concurrent access (e.g. between task and ISR)
    int lcd_clk_flags;
    struct {
        unsigned int disp_en_level: 1; // The level which can turn on the screen by `disp_gpio_num`
        unsigned int stream_mode: 1;   // If set, the LCD transfers data continuously, otherwise, it stops refreshing the LCD when transaction done
        unsigned int new_frame: 1;     // Whether the frame we're going to flush is a new one
        unsigned int fb_in_psram: 1;   // Whether the frame buffer is in PSRAM
        unsigned int bb_do_cache_invalidate: 1; //If we do cache invalidate in bb psram fb mode
        uint32_t need_update_pclk: 1;    // Whether to update the PCLK before start a new transaction
    } flags;
    uint8_t* buf1;
    uint8_t* buf2;
    uint8_t *fb;           // Frame buffer
    dma_descriptor_t dma_restart_node; //DMA descriptor used to restart the transfer
    dma_descriptor_t* dma_nodes; // DMA descriptor pool of size `num_dma_nodes`
    dma_descriptor_t* dma_nodes_1; // DMA descriptor pool of size `num_dma_nodes`
    dma_descriptor_t* dma_nodes_2; // DMA descriptor pool of size `num_dma_nodes`
};

esp_err_t qmsd_lcd_new_rgb_panel(const qmsd_lcd_rgb_panel_config_t *rgb_panel_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    qmsd_rgb_panel_t *rgb_panel = NULL;
    ESP_GOTO_ON_FALSE(rgb_panel_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid parameter");
    ESP_GOTO_ON_FALSE(rgb_panel_config->data_width == 16, ESP_ERR_NOT_SUPPORTED, err, TAG,
                      "unsupported data width %d", rgb_panel_config->data_width);
    // calculate the number of DMA descriptors
    size_t fb_size = rgb_panel_config->timings.h_res * rgb_panel_config->timings.v_res * rgb_panel_config->data_width / 8;
    size_t num_dma_nodes = 0;

    // DMA descriptors must be placed in internal SRAM (requested by DMA)
    rgb_panel = heap_caps_calloc(1, sizeof(qmsd_rgb_panel_t), MALLOC_CAP_DMA);
    ESP_GOTO_ON_FALSE(rgb_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for rgb panel");

    if (rgb_panel_config->bounce_buffer_size_px == 0) {
        num_dma_nodes = fb_size / RGB_DMA_BUFFER_SIZE;
        if (fb_size > num_dma_nodes * RGB_DMA_BUFFER_SIZE) {
            num_dma_nodes++;
        }
        rgb_panel->dma_nodes_1 = heap_caps_calloc(1, num_dma_nodes * sizeof(dma_descriptor_t), MALLOC_CAP_DMA);
        rgb_panel->dma_nodes = rgb_panel->dma_nodes_1;
        ESP_GOTO_ON_FALSE(rgb_panel->dma_nodes_1, ESP_ERR_NO_MEM, err, TAG, "no mem for rgb dma nodes 0");
        if (rgb_panel_config->flags.double_buffer) {
            rgb_panel->dma_nodes_2 = heap_caps_calloc(1, num_dma_nodes * sizeof(dma_descriptor_t), MALLOC_CAP_DMA);
            ESP_GOTO_ON_FALSE(rgb_panel->dma_nodes_2, ESP_ERR_NO_MEM, err, TAG, "no mem for rgb dma nodes 1");
        }
    } else {
        int bounce_bytes = 0;

        //The FB needs to be an integer multiple of the size of a bounce buffer (so
        //we end on the end of the second bounce buffer). Adjust the size of the
        //bounce buffers if it is not.
        int pixel_data_bytes = rgb_panel_config->data_width / 8; //size of one pixel, in bytes
        int no_pixels = rgb_panel_config->timings.h_res * rgb_panel_config->timings.v_res;
        bounce_bytes = rgb_panel_config->bounce_buffer_size_px * pixel_data_bytes;
        if (no_pixels % (rgb_panel_config->bounce_buffer_size_px * pixel_data_bytes)) {
            //Search for some value that does work. Yes, this is a stupidly simple algo, but it only
            //needs to run on startup.
            for (int a = rgb_panel_config->bounce_buffer_size_px; a > 0; a--) {
                if ((no_pixels % (a * pixel_data_bytes)) == 0) {
                    bounce_bytes = a * pixel_data_bytes;
                    ESP_LOGW(TAG, "Frame buffer is not an integer multiple of bounce buffers.");
                    ESP_LOGW(TAG, "Adjusted bounce buffer size from %d to %d pixels to fix this.",
                             rgb_panel_config->bounce_buffer_size_px, bounce_bytes / pixel_data_bytes);
                    break;
                }
            }
        }

        // DMA descriptors need to fit both bounce buffers
        num_dma_nodes = (bounce_bytes + DMA_DESCRIPTOR_BUFFER_MAX_SIZE - 1) / DMA_DESCRIPTOR_BUFFER_MAX_SIZE;
        num_dma_nodes = num_dma_nodes * 2; //as we have two bounce buffers
        rgb_panel->dma_nodes = heap_caps_calloc(1, num_dma_nodes * sizeof(dma_descriptor_t), MALLOC_CAP_DMA);
        ESP_GOTO_ON_FALSE(rgb_panel->dma_nodes, ESP_ERR_NO_MEM, err, TAG, "no mem for rgb dma nodes");
        rgb_panel->bounce_buffer_size_bytes = bounce_bytes;
    }
    rgb_panel->num_dma_nodes = num_dma_nodes;
    rgb_panel->panel_id = -1;
    // register to platform
    int panel_id = lcd_com_register_device(LCD_COM_DEVICE_TYPE_RGB, rgb_panel);
    ESP_GOTO_ON_FALSE(panel_id >= 0, ESP_ERR_NOT_FOUND, err, TAG, "no free rgb panel slot");
    rgb_panel->panel_id = panel_id;
    // enable APB to access LCD registers
    periph_module_enable(lcd_periph_signals.panels[panel_id].module);
    // alloc frame buffer
    bool alloc_from_psram = false;
    // fb_in_psram is only an option, if there's no PSRAM on board, we still alloc from SRAM
    if (rgb_panel_config->flags.fb_in_psram) {
#if CONFIG_SPIRAM_USE_MALLOC || CONFIG_SPIRAM_USE_CAPS_ALLOC
        if (esp_psram_is_initialized()) {
            alloc_from_psram = true;
        }
#endif
    }

    uint32_t malloc_caps;
    if (alloc_from_psram) {
        malloc_caps = MALLOC_CAP_SPIRAM;
    } else {
        malloc_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA;
    }
    rgb_panel->buf1 = heap_caps_aligned_calloc(64, 1, fb_size, malloc_caps);
    ESP_GOTO_ON_FALSE(rgb_panel->buf1, ESP_ERR_NO_MEM, err, TAG, "no mem for frame buffer");
    if (rgb_panel_config->flags.double_buffer) {
        rgb_panel->buf2 = heap_caps_aligned_calloc(64, 1, fb_size, malloc_caps);
        ESP_GOTO_ON_FALSE(rgb_panel->buf2, ESP_ERR_NO_MEM, err, TAG, "no mem for frame buffer");
    }
    ESP_GOTO_ON_FALSE(rgb_panel->buf1, ESP_ERR_NO_MEM, err, TAG, "no mem for frame buffer");
    rgb_panel->fb_size = fb_size;
    rgb_panel->flags.fb_in_psram = alloc_from_psram;
    if (rgb_panel_config->flags.double_buffer) {
        rgb_panel->fb = rgb_panel->buf2;
    } else {
        rgb_panel->fb = rgb_panel->buf1;
    }

    if (rgb_panel->bounce_buffer_size_bytes) {
        //We need to allocate bounce buffers.
        rgb_panel->bounce_buffer[0] = heap_caps_aligned_calloc(4, 1,
                                      rgb_panel->bounce_buffer_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        ESP_GOTO_ON_FALSE(rgb_panel->bounce_buffer[0], ESP_ERR_NO_MEM, err, TAG, "no mem for rgb bounce 0");
        rgb_panel->bounce_buffer[1] = heap_caps_aligned_calloc(4, 1,
                                      rgb_panel->bounce_buffer_size_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        ESP_GOTO_ON_FALSE(rgb_panel->bounce_buffer[1], ESP_ERR_NO_MEM, err, TAG, "no mem for rgb bounce 1");
    }

    // semaphore indicates new frame trans done
    rgb_panel->done_sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(rgb_panel->done_sem, ESP_ERR_NO_MEM, err, TAG, "create done sem failed");
    xSemaphoreGive(rgb_panel->done_sem); // initialize the semaphore count to 1
    // initialize HAL layer, so we can call LL APIs later
    lcd_hal_init(&rgb_panel->hal, panel_id);
    // enable clock gating
    lcd_ll_enable_clock(rgb_panel->hal.dev, true);
    // set peripheral clock resolution
    ret = lcd_rgb_panel_select_clock_src(rgb_panel, rgb_panel_config->clk_src);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "select periph clock failed");
    // set minimal PCLK divider
    // A limitation in the hardware, if the LCD_PCLK == LCD_CLK, then the PCLK polarity can't be adjustable
    if (!(rgb_panel_config->timings.flags.pclk_active_neg || rgb_panel_config->timings.flags.pclk_idle_high)) {
        rgb_panel->lcd_clk_flags |= LCD_HAL_PCLK_FLAG_ALLOW_EQUAL_SYSCLK;
    }
    // install interrupt service, (LCD peripheral shares the interrupt source with Camera by different mask)
    int isr_flags = ESP_INTR_FLAG_SHARED;
    ret = esp_intr_alloc_intrstatus(lcd_periph_signals.panels[panel_id].irq_id, isr_flags,
                                    (uint32_t)lcd_ll_get_interrupt_status_reg(rgb_panel->hal.dev),
                                    LCD_LL_EVENT_VSYNC_END, lcd_default_isr_handler, rgb_panel, &rgb_panel->intr);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "install interrupt failed");
    lcd_ll_enable_interrupt(rgb_panel->hal.dev, LCD_LL_EVENT_VSYNC_END, false); // disable all interrupts
    lcd_ll_clear_interrupt_status(rgb_panel->hal.dev, UINT32_MAX); // clear pending interrupt
    // install DMA service
    rgb_panel->flags.stream_mode = !rgb_panel_config->flags.relax_on_idle;
    ret = lcd_rgb_panel_create_trans_link(rgb_panel);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "install DMA failed");
    // configure GPIO
    ret = lcd_rgb_panel_configure_gpio(rgb_panel, rgb_panel_config);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "configure GPIO failed");
    // fill other rgb panel runtime parameters
    memcpy(rgb_panel->data_gpio_nums, rgb_panel_config->data_gpio_nums, SOC_LCD_RGB_DATA_WIDTH);
    rgb_panel->timings = rgb_panel_config->timings;
    rgb_panel->data_width = rgb_panel_config->data_width;
    rgb_panel->disp_gpio_num = rgb_panel_config->disp_gpio_num;
    rgb_panel->flags.disp_en_level = !rgb_panel_config->flags.disp_active_low;
    // fill function table
    rgb_panel->base.del = rgb_panel_del;
    rgb_panel->base.reset = rgb_panel_reset;
    rgb_panel->base.init = rgb_panel_init;
    rgb_panel->base.draw_bitmap = rgb_panel_draw_bitmap;
    //rgb_panel->base.disp_off = rgb_panel_disp_off;
    rgb_panel->base.invert_color = rgb_panel_invert_color;
    rgb_panel->base.mirror = rgb_panel_mirror;
    rgb_panel->base.swap_xy = rgb_panel_swap_xy;
    rgb_panel->base.set_gap = rgb_panel_set_gap;
    rgb_panel->spinlock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    // return base class
    *ret_panel = &(rgb_panel->base);
    ESP_LOGD(TAG, "new rgb panel(%d) @%p, fb_size=%zu", rgb_panel->panel_id, rgb_panel, rgb_panel->fb_size);
    return ESP_OK;

err:
    if (rgb_panel) {
        if (rgb_panel->panel_id >= 0) {
            periph_module_disable(lcd_periph_signals.panels[rgb_panel->panel_id].module);
            lcd_com_remove_device(LCD_COM_DEVICE_TYPE_RGB, rgb_panel->panel_id);
        }
        if (rgb_panel->buf1) {
            free(rgb_panel->buf1);
        }
        if (rgb_panel->buf2) {
            free(rgb_panel->buf2);
        }
        if (rgb_panel->dma_nodes_1) {
            free(rgb_panel->dma_nodes_1);
        }
        if (rgb_panel->dma_nodes_2) {
            free(rgb_panel->dma_nodes_2);
        }
        if (rgb_panel->done_sem) {
            vSemaphoreDelete(rgb_panel->done_sem);
        }
        if (rgb_panel->dma_chan) {
            gdma_disconnect(rgb_panel->dma_chan);
            gdma_del_channel(rgb_panel->dma_chan);
        }
        if (rgb_panel->intr) {
            esp_intr_free(rgb_panel->intr);
        }
        if (rgb_panel->pm_lock) {
            esp_pm_lock_release(rgb_panel->pm_lock);
            esp_pm_lock_delete(rgb_panel->pm_lock);
        }
        if (rgb_panel->bounce_buffer[0]) {
            free(rgb_panel->bounce_buffer[0]);
        }
        if (rgb_panel->bounce_buffer[1]) {
            free(rgb_panel->bounce_buffer[1]);
        }
        free(rgb_panel);
    }
    return ret;
}

void qmsd_lcd_rgb_panel_get_buffer(esp_lcd_panel_t *panel, uint8_t** buf1, uint8_t** buf2) {
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    if (buf1) {
        *buf1 = rgb_panel->buf1;
    }
    if (buf2) {
        *buf2 = rgb_panel->buf2;
    }
}

void qmsd_lcd_rgb_panel_start(esp_lcd_panel_t *panel) {
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);

    if (rgb_panel->flags.fb_in_psram) {
        // CPU writes data to PSRAM through DCache, data in PSRAM might not get updated, so write back
        Cache_WriteBack_Addr((uint32_t)rgb_panel->buf1, rgb_panel->fb_size);
    }

    // start LCD engine
    lcd_ll_start(rgb_panel->hal.dev);
}

void qmsd_lcd_panel_cache_flush(esp_lcd_panel_t *panel) {
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    Cache_WriteBack_Addr((uint32_t)rgb_panel->buf1, rgb_panel->fb_size);
}

void qmsd_lcd_panel_wait_flush(esp_lcd_panel_t *panel) {
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    xSemaphoreTake(rgb_panel->done_sem, 0); // wait for last flush done
    xSemaphoreTake(rgb_panel->done_sem, portMAX_DELAY); // wait for last flush done
}

void qmsd_lcd_buffer_select(esp_lcd_panel_t *panel, uint8_t* buf) {
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    if (!rgb_panel->on_vsync)
        xSemaphoreTake(rgb_panel->done_sem, portMAX_DELAY); // wait for last flush done

    if (rgb_panel->bounce_buffer_size_bytes != 0) {
        if (buf == rgb_panel->buf2) {
            rgb_panel->fb = rgb_panel->buf2;
        } else {
            rgb_panel->fb = rgb_panel->buf1;
        }
    } else {
        if (buf == rgb_panel->buf2) {
            rgb_panel->dma_nodes = rgb_panel->dma_nodes_2;
        } else {
            rgb_panel->dma_nodes = rgb_panel->dma_nodes_1;
        }
    }
}

static esp_err_t rgb_panel_del(esp_lcd_panel_t *panel)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    xSemaphoreTake(rgb_panel->done_sem, portMAX_DELAY); // wait for last flush done
    int panel_id = rgb_panel->panel_id;
    gdma_disconnect(rgb_panel->dma_chan);
    gdma_del_channel(rgb_panel->dma_chan);
    esp_intr_free(rgb_panel->intr);
    periph_module_disable(lcd_periph_signals.panels[panel_id].module);
    lcd_com_remove_device(LCD_COM_DEVICE_TYPE_RGB, rgb_panel->panel_id);
    vSemaphoreDelete(rgb_panel->done_sem);
    free(rgb_panel->buf1);
    if (rgb_panel->buf2) {
        free(rgb_panel->buf2);
    }
    if (rgb_panel->bounce_buffer[0]) {
        free(rgb_panel->bounce_buffer[0]);
    }
    if (rgb_panel->bounce_buffer[1]) {
        free(rgb_panel->bounce_buffer[1]);
    }
    if (rgb_panel->pm_lock) {
        esp_pm_lock_release(rgb_panel->pm_lock);
        esp_pm_lock_delete(rgb_panel->pm_lock);
    }
    free(rgb_panel);
    ESP_LOGD(TAG, "del rgb panel(%d)", panel_id);
    return ESP_OK;
}

static esp_err_t rgb_panel_reset(esp_lcd_panel_t *panel)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    lcd_ll_fifo_reset(rgb_panel->hal.dev);
    lcd_ll_reset(rgb_panel->hal.dev);
    return ESP_OK;
}

static esp_err_t rgb_panel_init(esp_lcd_panel_t *panel)
{
    esp_err_t ret = ESP_OK;
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    rgb_panel->timings.pclk_hz = lcd_hal_cal_pclk_freq(&rgb_panel->hal, rgb_panel->src_clk_hz, rgb_panel->timings.pclk_hz, rgb_panel->lcd_clk_flags);
    lcd_ll_set_clock_idle_level(rgb_panel->hal.dev, rgb_panel->timings.flags.pclk_idle_high);
    lcd_ll_set_pixel_clock_edge(rgb_panel->hal.dev, rgb_panel->timings.flags.pclk_active_neg);
    // enable RGB mode and set data width
    lcd_ll_enable_rgb_mode(rgb_panel->hal.dev, true);
    lcd_ll_set_data_width(rgb_panel->hal.dev, rgb_panel->data_width);
    lcd_ll_set_phase_cycles(rgb_panel->hal.dev, 0, 0, 1); // enable data phase only
    // number of data cycles is controlled by DMA buffer size
    lcd_ll_enable_output_always_on(rgb_panel->hal.dev, true);
    // configure HSYNC, VSYNC, DE signal idle state level
    lcd_ll_set_idle_level(rgb_panel->hal.dev, !rgb_panel->timings.flags.hsync_idle_low,
                          !rgb_panel->timings.flags.vsync_idle_low, rgb_panel->timings.flags.de_idle_high);
    // configure blank region timing
    lcd_ll_set_blank_cycles(rgb_panel->hal.dev, 1, 1); // RGB panel always has a front and back blank (porch region)
    lcd_ll_set_horizontal_timing(rgb_panel->hal.dev, rgb_panel->timings.hsync_pulse_width,
                                 rgb_panel->timings.hsync_back_porch, rgb_panel->timings.h_res,
                                 rgb_panel->timings.hsync_front_porch);
    lcd_ll_set_vertical_timing(rgb_panel->hal.dev, rgb_panel->timings.vsync_pulse_width,
                               rgb_panel->timings.vsync_back_porch, rgb_panel->timings.v_res,
                               rgb_panel->timings.vsync_front_porch);
    // output hsync even in porch region
    lcd_ll_enable_output_hsync_in_porch_region(rgb_panel->hal.dev, true);
    // generate the hsync at the very begining of line
    lcd_ll_set_hsync_position(rgb_panel->hal.dev, 0);
    // restart flush by hardware has some limitation, instead, the driver will restart the flush in the VSYNC end interrupt by software
    lcd_ll_enable_auto_next_frame(rgb_panel->hal.dev, rgb_panel->flags.stream_mode);
    // trigger interrupt on the end of frame
    lcd_ll_enable_interrupt(rgb_panel->hal.dev, LCD_LL_EVENT_VSYNC_END, true);
    esp_intr_enable(rgb_panel->intr);
    if (rgb_panel->flags.stream_mode) {
        lcd_rgb_panel_start_transmission(rgb_panel);
    }

    ESP_LOGI(TAG, "rgb panel(%d) start, pclk=%uHz", rgb_panel->panel_id, rgb_panel->timings.pclk_hz);
    return ret;
}

static esp_err_t rgb_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    return ESP_FAIL;
}

static esp_err_t rgb_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    int panel_id = rgb_panel->panel_id;
    // inverting the data line by GPIO matrix
    for (int i = 0; i < rgb_panel->data_width; i++) {
        esp_rom_gpio_connect_out_signal(rgb_panel->data_gpio_nums[i], lcd_periph_signals.panels[panel_id].data_sigs[i],
                                        invert_color_data, false);
    }
    return ESP_OK;
}

static esp_err_t rgb_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t rgb_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t rgb_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    rgb_panel->x_gap = x_gap;
    rgb_panel->x_gap = y_gap;
    return ESP_OK;
}

static esp_err_t rgb_panel_disp_off(esp_lcd_panel_t *panel, bool off)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    if (rgb_panel->disp_gpio_num < 0) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (off) { // turn off screen
        gpio_set_level(rgb_panel->disp_gpio_num, !rgb_panel->flags.disp_en_level);
    } else { // turn on screen
        gpio_set_level(rgb_panel->disp_gpio_num, rgb_panel->flags.disp_en_level);
    }
    return ESP_OK;
}

static esp_err_t lcd_rgb_panel_configure_gpio(qmsd_rgb_panel_t *panel, const qmsd_lcd_rgb_panel_config_t *panel_config)
{
    int panel_id = panel->panel_id;
    // check validation of GPIO number
    bool valid_gpio = (panel_config->hsync_gpio_num >= 0) && (panel_config->vsync_gpio_num >= 0) &&
                      (panel_config->pclk_gpio_num >= 0);
    for (size_t i = 0; i < panel_config->data_width; i++) {
        valid_gpio = valid_gpio && (panel_config->data_gpio_nums[i] >= 0);
    }
    if (!valid_gpio) {
        return ESP_ERR_INVALID_ARG;
    }
    // connect peripheral signals via GPIO matrix
    for (size_t i = 0; i < panel_config->data_width; i++) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[panel_config->data_gpio_nums[i]], PIN_FUNC_GPIO);
        gpio_set_direction(panel_config->data_gpio_nums[i], GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(panel_config->data_gpio_nums[i],
                                        lcd_periph_signals.panels[panel_id].data_sigs[i], false, false);
    }
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[panel_config->hsync_gpio_num], PIN_FUNC_GPIO);
    gpio_set_direction(panel_config->hsync_gpio_num, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(panel_config->hsync_gpio_num,
                                    lcd_periph_signals.panels[panel_id].hsync_sig, false, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[panel_config->vsync_gpio_num], PIN_FUNC_GPIO);
    gpio_set_direction(panel_config->vsync_gpio_num, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(panel_config->vsync_gpio_num,
                                    lcd_periph_signals.panels[panel_id].vsync_sig, false, false);
    gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[panel_config->pclk_gpio_num], PIN_FUNC_GPIO);
    gpio_set_direction(panel_config->pclk_gpio_num, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(panel_config->pclk_gpio_num,
                                    lcd_periph_signals.panels[panel_id].pclk_sig, false, false);
    // DE signal might not be necessary for some RGB LCD
    if (panel_config->de_gpio_num >= 0) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[panel_config->de_gpio_num], PIN_FUNC_GPIO);
        gpio_set_direction(panel_config->de_gpio_num, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(panel_config->de_gpio_num,
                                        lcd_periph_signals.panels[panel_id].de_sig, false, false);
    }
    // disp enable GPIO is optional
    if (panel_config->disp_gpio_num >= 0) {
        gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[panel_config->disp_gpio_num], PIN_FUNC_GPIO);
        gpio_set_direction(panel_config->disp_gpio_num, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(panel_config->disp_gpio_num, SIG_GPIO_OUT_IDX, false, false);
    }
    return ESP_OK;
}

static esp_err_t lcd_rgb_panel_select_clock_src(qmsd_rgb_panel_t *panel, lcd_clock_source_t clk_src)
{
    esp_err_t ret = ESP_OK;
    switch (clk_src) {
    case LCD_CLK_SRC_PLL240M:
        panel->src_clk_hz = 240000000;
        break;
    case LCD_CLK_SRC_PLL160M:
        panel->src_clk_hz = 160000000;
        break;
    case LCD_CLK_SRC_XTAL:
        panel->src_clk_hz = esp_clk_xtal_freq();
        break;
    default:
        ESP_RETURN_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, TAG, "unsupported clock source: %d", clk_src);
        break;
    }
    lcd_ll_select_clk_src(panel->hal.dev, clk_src);

    if (clk_src == LCD_CLK_SRC_PLL240M || clk_src == LCD_CLK_SRC_PLL160M) {
#if CONFIG_PM_ENABLE
        ret = esp_pm_lock_create(ESP_PM_APB_FREQ_MAX, 0, "rgb_panel", &panel->pm_lock);
        ESP_RETURN_ON_ERROR(ret, TAG, "create ESP_PM_APB_FREQ_MAX lock failed");
        // hold the lock during the whole lifecycle of RGB panel
        esp_pm_lock_acquire(panel->pm_lock);
        ESP_LOGD(TAG, "installed ESP_PM_APB_FREQ_MAX lock and hold the lock during the whole panel lifecycle");
#endif
    }
    return ret;
}


static void rgb_com_mount_dma_data(dma_descriptor_t *desc_head, const void *buffer, size_t len) {
    size_t prepared_length = 0;
    uint8_t *data = (uint8_t *)buffer;
    dma_descriptor_t *desc = desc_head;
    while (len > RGB_DMA_BUFFER_SIZE) {
        desc->dw0.suc_eof = 0; // not the end of the transaction
        desc->dw0.size = RGB_DMA_BUFFER_SIZE;
        desc->dw0.length = RGB_DMA_BUFFER_SIZE;
        desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc->buffer = &data[prepared_length];
        desc = desc->next; // move to next descriptor
        prepared_length += RGB_DMA_BUFFER_SIZE;
        len -= RGB_DMA_BUFFER_SIZE;
    }
    if (len) {
        desc->dw0.suc_eof = 1; // end of the transaction
        desc->dw0.size = len;
        desc->dw0.length = len;
        desc->dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_DMA;
        desc->buffer = &data[prepared_length];
        desc = desc->next; // move to next descriptor
        prepared_length += len;
    }
}

static IRAM_ATTR bool lcd_rgb_panel_fill_bounce_buffer(qmsd_rgb_panel_t *panel, uint8_t *buffer)
{
    int bytes_per_pixel = panel->data_width / 8;

    //We do have a framebuffer; copy from there.
    memcpy(buffer, &panel->fb[panel->bounce_pos_px * bytes_per_pixel], panel->bounce_buffer_size_bytes);
    if (panel->flags.bb_do_cache_invalidate) {
        //We don't need the bytes we copied from psram anymore.
        //Make sure that if anything happened to have changed (because the line already was in cache) we write
        //the data back.
        Cache_WriteBack_Addr((uint32_t)&panel->fb[panel->bounce_pos_px * bytes_per_pixel], panel->bounce_buffer_size_bytes);
        //Invalidate the data. Note: possible race: perhaps something on the other core can squeeze a write
        //between this and the writeback, in which case that data gets discarded.
        Cache_Invalidate_Addr((uint32_t)&panel->fb[panel->bounce_pos_px * bytes_per_pixel], panel->bounce_buffer_size_bytes);
    }
    panel->bounce_pos_px += panel->bounce_buffer_size_bytes / bytes_per_pixel;
    //If the bounce pos is larger than the framebuffer size, wrap around so the next isr starts pre-loading
    //the next frame.
    if (panel->bounce_pos_px >= panel->fb_size / bytes_per_pixel) {
        panel->bounce_pos_px = 0;
    }

    //Preload the next bit of buffer into psram.
    Cache_Start_DCache_Preload((uint32_t)&panel->fb[panel->bounce_pos_px * bytes_per_pixel],
                                panel->bounce_buffer_size_bytes, 0);

    return false;
}

static IRAM_ATTR bool lcd_rgb_panel_eof_handler(gdma_channel_handle_t dma_chan, gdma_event_data_t *event_data, void *user_data)
{
    qmsd_rgb_panel_t *panel = (qmsd_rgb_panel_t *)user_data;
    dma_descriptor_t *desc = (dma_descriptor_t *)event_data->tx_eof_desc_addr;
    //Figure out which bounce buffer to write to.
    //Note: what we receive is the *last* descriptor of this bounce buffer.
    int bb = (desc == &panel->dma_nodes[panel->num_dma_nodes - 1]) ? 1 : 0;
    return lcd_rgb_panel_fill_bounce_buffer(panel, panel->bounce_buffer[bb]);
}

//If we restart GDMA, this many pixels will already have been transfered to the
//LCD peripheral. Looks like that has 16 pixels of FIFO plus one holding register.
#define LCD_FIFO_SIZE_PX 17

static esp_err_t lcd_rgb_panel_create_trans_link(qmsd_rgb_panel_t *panel)
{
    esp_err_t ret = ESP_OK;
    if (panel->bounce_buffer_size_bytes == 0) {
        // chain DMA descriptors
        for (int i = 0; i < panel->num_dma_nodes; i++) {
            panel->dma_nodes_1[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
            panel->dma_nodes_1[i].next = &panel->dma_nodes_1[i + 1];
        }
        // fix the last DMA descriptor according to whether the LCD works in stream mode
        if (panel->flags.stream_mode) {
            panel->dma_nodes_1[panel->num_dma_nodes - 1].next = &panel->dma_nodes_1[0]; // chain into a circle
        } else {
            panel->dma_nodes_1[panel->num_dma_nodes - 1].next = NULL; // one-off DMA chain
        }

        if (panel->dma_nodes_2) {
            for (int i = 0; i < panel->num_dma_nodes; i++) {
                panel->dma_nodes_2[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
                panel->dma_nodes_2[i].next = &panel->dma_nodes_2[i + 1];
            }
            // fix the last DMA descriptor according to whether the LCD works in stream mode
            if (panel->flags.stream_mode) {
                panel->dma_nodes_2[panel->num_dma_nodes - 1].next = &panel->dma_nodes_2[0]; // chain into a circle
            } else {
                panel->dma_nodes_2[panel->num_dma_nodes - 1].next = NULL; // one-off DMA chain
            }
        }
        // mount the frame buffer to the DMA descriptors
        rgb_com_mount_dma_data(panel->dma_nodes_1, panel->buf1, panel->fb_size);
        if (panel->buf2) {
            rgb_com_mount_dma_data(panel->dma_nodes_2, panel->buf2, panel->fb_size);
        }
    } else {
        //Create DMA descriptors for bounce buffers:
        // chain DMA descriptors
        for (int i = 0; i < panel->num_dma_nodes; i++) {
            panel->dma_nodes[i].dw0.owner = DMA_DESCRIPTOR_BUFFER_OWNER_CPU;
            panel->dma_nodes[i].next = &panel->dma_nodes[i + 1];
        }
        //loop end back to start
        panel->dma_nodes[panel->num_dma_nodes - 1].next = &panel->dma_nodes[0];
        //set eof on end of 1st and 2nd bounce buffer
        panel->dma_nodes[(panel->num_dma_nodes / 2) - 1].dw0.suc_eof = 1;
        panel->dma_nodes[panel->num_dma_nodes - 1].dw0.suc_eof = 1;
        // mount the bounce buffers to the DMA descriptors
        lcd_com_mount_dma_data(&panel->dma_nodes[0], panel->bounce_buffer[0], panel->bounce_buffer_size_bytes);
        lcd_com_mount_dma_data(&panel->dma_nodes[(panel->num_dma_nodes / 2)], panel->bounce_buffer[1], panel->bounce_buffer_size_bytes);
    }

    // alloc DMA channel and connect to LCD peripheral
    gdma_channel_alloc_config_t dma_chan_config = {
        .direction = GDMA_CHANNEL_DIRECTION_TX,
    };
    ret = gdma_new_channel(&dma_chan_config, &panel->dma_chan);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "alloc DMA channel failed");
    gdma_connect(panel->dma_chan, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_LCD, 0));

    int channel_id;
    gdma_get_channel_id(panel->dma_chan, &channel_id);
    gdma_ll_tx_set_priority(&GDMA, channel_id, 1);

    gdma_transfer_ability_t ability = {
        .psram_trans_align = 64,
        .sram_trans_align = 4,
    };
    gdma_set_transfer_ability(panel->dma_chan, &ability);

    if (panel->bounce_buffer_size_bytes != 0) {
        // register callback to re-fill bounce buffers once they're fully sent
        gdma_tx_event_callbacks_t cbs = {0};
        cbs.on_trans_eof = lcd_rgb_panel_eof_handler;
        gdma_register_tx_event_callbacks(panel->dma_chan, &cbs, panel);
        // the start of DMA should be prior to the start of LCD engine
        gdma_start(panel->dma_chan, (intptr_t)panel->dma_nodes);
    } else {
        // the start of DMA should be prior to the start of LCD engine
        gdma_start(panel->dma_chan, (intptr_t)panel->dma_nodes_1);
    }

err:
    return ret;
}

static IRAM_ATTR void lcd_rgb_panel_restart_transmission(qmsd_rgb_panel_t *panel)
{
    if (panel->bounce_buffer_size_bytes != 0) {
        //Catch de-synced framebuffer and reset if needed.
        if (panel->bounce_pos_px > panel->bounce_buffer_size_bytes) {
            panel->bounce_pos_px = 0;
        }
        //Pre-fill bounce buffer 0, if the EOF ISR didn't do that already
        if (panel->bounce_pos_px < panel->bounce_buffer_size_bytes / 2) {
            lcd_rgb_panel_fill_bounce_buffer(panel, panel->bounce_buffer[0]);
        }
    }

    gdma_reset(panel->dma_chan);
    gdma_start(panel->dma_chan, (intptr_t)panel->dma_nodes);
    //On restart, the data sent to the LCD peripheral needs to start 17 pixels after the FB start (the length of the
    //lcd fifo) so we use a special DMA node to restart the DMA transaction.
    memcpy(&panel->dma_restart_node, &panel->dma_nodes[0], sizeof(panel->dma_restart_node));
    int restart_skip_bytes = LCD_FIFO_SIZE_PX*sizeof(uint16_t);
    uint8_t *p=(uint8_t*)panel->dma_restart_node.buffer;
    panel->dma_restart_node.buffer=&p[restart_skip_bytes];
    panel->dma_restart_node.dw0.length-=restart_skip_bytes;
    panel->dma_restart_node.dw0.size-=restart_skip_bytes;
    gdma_start(panel->dma_chan, (intptr_t)&panel->dma_restart_node);

    if (panel->bounce_buffer_size_bytes != 0) {
        //Fill 2nd bounce buffer while 1st is being sent out, if needed.
        if (panel->bounce_pos_px < panel->bounce_buffer_size_bytes) {
            lcd_rgb_panel_fill_bounce_buffer(panel, panel->bounce_buffer[0]);
        }
    }
}

static IRAM_ATTR void lcd_rgb_panel_start_transmission(qmsd_rgb_panel_t *rgb_panel)
{
    // reset FIFO of DMA and LCD, incase there remains old frame data
    gdma_reset(rgb_panel->dma_chan);
    lcd_ll_stop(rgb_panel->hal.dev);
    lcd_ll_fifo_reset(rgb_panel->hal.dev);

    //pre-fill bounce buffers if needed
    if (rgb_panel->bounce_buffer_size_bytes != 0) {
        rgb_panel->bounce_pos_px = 0;
        lcd_rgb_panel_fill_bounce_buffer(rgb_panel, rgb_panel->bounce_buffer[0]);
        lcd_rgb_panel_fill_bounce_buffer(rgb_panel, rgb_panel->bounce_buffer[1]);
    }

    gdma_start(rgb_panel->dma_chan, (intptr_t)rgb_panel->dma_nodes);
    // delay 1us is sufficient for DMA to pass data to LCD FIFO
    // in fact, this is only needed when LCD pixel clock is set too high
    esp_rom_delay_us(1);
    // start LCD engine
    lcd_ll_start(rgb_panel->hal.dev);
}

esp_err_t qsmd_lcd_rgb_panel_refresh(esp_lcd_panel_t *panel)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    lcd_rgb_panel_start_transmission(rgb_panel);
    return ESP_OK;
}

esp_err_t qmsd_lcd_rgb_panel_register_vsync(esp_lcd_panel_t *panel, qmsd_rgb_panel_vsync_cb_t on_vsync)
{
    qmsd_rgb_panel_t *rgb_panel = __containerof(panel, qmsd_rgb_panel_t, base);
    rgb_panel->on_vsync = on_vsync;
    return ESP_OK;
}

IRAM_ATTR static void lcd_rgb_panel_try_update_pclk(qmsd_rgb_panel_t *rgb_panel)
{
    portENTER_CRITICAL_ISR(&rgb_panel->spinlock);
    if (unlikely(rgb_panel->flags.need_update_pclk)) {
        rgb_panel->flags.need_update_pclk = false;
        rgb_panel->timings.pclk_hz = lcd_hal_cal_pclk_freq(&rgb_panel->hal, rgb_panel->src_clk_hz, rgb_panel->timings.pclk_hz, rgb_panel->lcd_clk_flags);
    }
    portEXIT_CRITICAL_ISR(&rgb_panel->spinlock);
}

IRAM_ATTR static void lcd_default_isr_handler(void *args)
{
    qmsd_rgb_panel_t *panel = (qmsd_rgb_panel_t *)args;
    bool need_yield = false;
    BaseType_t high_task_woken = pdFALSE;

    uint32_t intr_status = lcd_ll_get_interrupt_status(panel->hal.dev);
    lcd_ll_clear_interrupt_status(panel->hal.dev, intr_status);
    if (intr_status & LCD_LL_EVENT_VSYNC_END) {
        // check whether to update the PCLK frequency, it should be safe to update the PCLK frequency in the VSYNC interrupt
        lcd_rgb_panel_try_update_pclk(panel);
        if (panel->on_vsync) {
            if (panel->on_vsync()) {
                need_yield = true;
            }
        } else {
            if (panel->bounce_buffer_size_bytes != 0) {
                // In some cases, there can be a GDMA bandwidth issue that de-syncs the LCD and GDMA
                // states, as in: the GDMA provides a pixel earlier or later than what the LCD peripheral
                // wants to output. To compensate for that, we reset the DMA in the VBlank because we
                // know the LCD will be requesting pixel (0,0) next.
                // Note that this fix has its own problems, as in: if this interrupt is late enough,
                // the display will shift as the LCD controller already read out the first data bytes,
                // and resetting DMA will re-send those. This is less of a bad thing, as it leads to a
                // single-frame 'glitch' in the image rather than an image that's skewed for the rest of
                // the runtime of the program. It's also not super-likely as this interrupt has the
                // entirety of the VBlank time to reset DMA. ToDo: see if we can detect if the interrupt
                // is late and not reset the transmission in that case.
                lcd_rgb_panel_restart_transmission(panel);
            } else {
                gdma_reset(panel->dma_chan);
                lcd_ll_stop(panel->hal.dev);
                lcd_ll_fifo_reset(panel->hal.dev);
                gdma_start(panel->dma_chan, (intptr_t)panel->dma_nodes);
                // delay 1us is sufficient for DMA to pass data to LCD FIFO
                // in fact, this is only needed when LCD pixel clock is set too high
                esp_rom_delay_us(1);
                lcd_ll_start(panel->hal.dev);
            }

            xSemaphoreGiveFromISR(panel->done_sem, &high_task_woken);
            if (high_task_woken == pdTRUE) {
                need_yield = true;
            }
        }
    }

    if (need_yield) {
        portYIELD_FROM_ISR();
    }
}

#endif // SOC_LCDCAM_SUPPORTED
