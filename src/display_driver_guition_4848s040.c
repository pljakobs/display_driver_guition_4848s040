#include "display_driver_guition_4848s040/display_driver_guition_4848s040.h"

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_st7701.h"
#include "esp_lcd_panel_io_additions.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "guition_4848s040";

/* ---- Board-fixed GPIO assignments ---------------------------------------- */
#define GUITION_GPIO_SPI_CS    39
#define GUITION_GPIO_SPI_CLK   48
#define GUITION_GPIO_SPI_MOSI  47

#define GUITION_GPIO_DE        18
#define GUITION_GPIO_HSYNC     16
#define GUITION_GPIO_VSYNC     17
#define GUITION_GPIO_PCLK      21

/* RGB data pins for 16-bit RGB565 bus:
 * DATA[0-4]  = B[0-4],  DATA[5-10] = G[0-5],  DATA[11-15] = R[0-4]
 * Verified against BOARD_JINGCAI_ESP32_4848S040C_I_Y_3.h */
#define GUITION_RGB_DATA_PINS  \
     4,  5,  6,  7, 15,        \
     8, 20,  3, 46,  9, 10,    \
    11, 12, 13, 14,  0

/* Board-specific ST7701S vendor initialisation sequence.
 * Taken verbatim from ESP32_Display_Panel BOARD_JINGCAI_ESP32_4848S040C_I_Y_3.h.
 * Ends with SLPOUT (0x11) + 120 ms; DISPON (0x29) is sent separately via
 * esp_lcd_panel_disp_on_off() after esp_lcd_panel_init(). */
static const st7701_lcd_init_cmd_t guition_4848s040_init_cmds[] = {
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t[]){0x3B, 0x00}, 2, 0},
    {0xC1, (uint8_t[]){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t[]){0x31, 0x05}, 2, 0},
    {0xCD, (uint8_t[]){0x00}, 1, 0},
    {0xB0, (uint8_t[]){0x00, 0x11, 0x18, 0x0E, 0x11, 0x06, 0x07, 0x08, 0x07, 0x22, 0x04, 0x12, 0x0F, 0xAA, 0x31, 0x18}, 16, 0},
    {0xB1, (uint8_t[]){0x00, 0x11, 0x19, 0x0E, 0x12, 0x07, 0x08, 0x08, 0x08, 0x22, 0x04, 0x11, 0x11, 0xA9, 0x32, 0x18}, 16, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t[]){0x60}, 1, 0},
    {0xB1, (uint8_t[]){0x32}, 1, 0},
    {0xB2, (uint8_t[]){0x07}, 1, 0},
    {0xB3, (uint8_t[]){0x80}, 1, 0},
    {0xB5, (uint8_t[]){0x49}, 1, 0},
    {0xB7, (uint8_t[]){0x85}, 1, 0},
    {0xB8, (uint8_t[]){0x21}, 1, 0},
    {0xC1, (uint8_t[]){0x78}, 1, 0},
    {0xC2, (uint8_t[]){0x78}, 1, 0},
    {0xE0, (uint8_t[]){0x00, 0x1B, 0x02}, 3, 0},
    {0xE1, (uint8_t[]){0x08, 0xA0, 0x00, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x44, 0x44}, 11, 0},
    {0xE2, (uint8_t[]){0x11, 0x11, 0x44, 0x44, 0xED, 0xA0, 0x00, 0x00, 0xEC, 0xA0, 0x00, 0x00}, 12, 0},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0xE4, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t[]){0x0A, 0xE9, 0xD8, 0xA0, 0x0C, 0xEB, 0xD8, 0xA0, 0x0E, 0xED, 0xD8, 0xA0, 0x10, 0xEF, 0xD8, 0xA0}, 16, 0},
    {0xE6, (uint8_t[]){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0xE7, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t[]){0x09, 0xE8, 0xD8, 0xA0, 0x0B, 0xEA, 0xD8, 0xA0, 0x0D, 0xEC, 0xD8, 0xA0, 0x0F, 0xEE, 0xD8, 0xA0}, 16, 0},
    {0xEB, (uint8_t[]){0x02, 0x00, 0xE4, 0xE4, 0x88, 0x00, 0x40}, 7, 0},
    {0xEC, (uint8_t[]){0x3C, 0x00}, 2, 0},
    {0xED, (uint8_t[]){0xAB, 0x89, 0x76, 0x54, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x45, 0x67, 0x98, 0xBA}, 16, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xE5, (uint8_t[]){0xE4}, 1, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},
    {0x11, NULL, 0, 120},  /* sleep out + 120 ms delay */
};
#define GUITION_INIT_CMDS_SIZE (sizeof(guition_4848s040_init_cmds) / sizeof(guition_4848s040_init_cmds[0]))

/**
 * Board-specific runtime state for GUITION-4848S040.
 */
typedef struct display_driver_guition_4848s040 {
    display_driver_guition_4848s040_config_t config;
    bool initialized;
    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_io_handle_t io;
    SemaphoreHandle_t flush_done;
} display_driver_guition_4848s040_t;

/* Forward declarations */
static esp_err_t guition_4848s040_init(display_driver_handle_t driver,
                                        const display_driver_config_t *config);
static esp_err_t guition_4848s040_deinit(display_driver_handle_t driver);
static bool      guition_4848s040_is_ready(display_driver_handle_t driver);
static esp_err_t guition_4848s040_flush(display_driver_handle_t driver,
                                         uint16_t x1, uint16_t y1,
                                         uint16_t x2, uint16_t y2,
                                         const void *pixel_data,
                                         size_t pixel_data_size);
static esp_err_t guition_4848s040_wait_flush_complete(display_driver_handle_t driver);

static const display_driver_vtable_t guition_4848s040_vtable = {
    .init                 = guition_4848s040_init,
    .deinit               = guition_4848s040_deinit,
    .is_ready             = guition_4848s040_is_ready,
    .flush                = guition_4848s040_flush,
    .wait_flush_complete  = guition_4848s040_wait_flush_complete,
};

/* =========================================================================
 * Vtable implementations
 * ========================================================================= */

static esp_err_t guition_4848s040_init(display_driver_handle_t driver,
                                        const display_driver_config_t *config)
{
    display_driver_guition_4848s040_t *self = (display_driver_guition_4848s040_t *)driver;

    if (self == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->width == 0 || config->height == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (self->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    self->config.base.width  = config->width;
    self->config.base.height = config->height;

    esp_err_t ret;

    /* -- PSRAM pre-flight check ------------------------------------------- */
    /* The RGB framebuffer (480x480x2 = 460 800 B) must be allocated from PSRAM
     * inside esp_lcd_new_rgb_panel().  In ESP-IDF ≤ v5.2.1 the error-cleanup
     * path in esp_lcd_new_rgb_panel() crashes with a LoadProhibited exception
     * when that allocation fails because hal.dev is still NULL at that point
     * (see lcd_rgb_panel_destory()).  Catch a likely failure here, before
     * touching the LCD peripheral, so the user gets a meaningful error. */
    {
        size_t fb_bytes = (size_t)config->width * config->height * 2u;
        size_t free_psram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        if (free_psram < fb_bytes) {
            ESP_LOGE(TAG, "insufficient PSRAM: need %zu B for framebuffer, "
                     "largest free block is %zu B", fb_bytes, free_psram);
            return ESP_ERR_NO_MEM;
        }
    }

    /* -- 3-wire SPI panel IO for ST7701S vendor init ----------------------- */
    spi_line_config_t spi_lines = {
        .cs_io_type   = IO_TYPE_GPIO,
        .cs_gpio_num  = GUITION_GPIO_SPI_CS,
        .scl_io_type  = IO_TYPE_GPIO,
        .scl_gpio_num = GUITION_GPIO_SPI_CLK,
        .sda_io_type  = IO_TYPE_GPIO,
        .sda_gpio_num = GUITION_GPIO_SPI_MOSI,
        .io_expander  = NULL,
    };
    esp_lcd_panel_io_3wire_spi_config_t io_cfg =
        ST7701_PANEL_IO_3WIRE_SPI_CONFIG(spi_lines, 0);
    ESP_LOGI(TAG, "Creating 3-wire SPI IO (CS=%d CLK=%d MOSI=%d)...",
             GUITION_GPIO_SPI_CS, GUITION_GPIO_SPI_CLK, GUITION_GPIO_SPI_MOSI);
    ret = esp_lcd_new_panel_io_3wire_spi(&io_cfg, &self->io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "3-wire SPI IO create failed: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "3-wire SPI IO created");

    /* -- RGB panel config -------------------------------------------------- */
    /* Library uses 26 MHz for this panel; timings are tuned for that clock */
    uint32_t pclk    = self->config.pclk_frequency_hz
                           ? self->config.pclk_frequency_hz
                           : 26000000U;
    uint8_t frame_buffer_count = self->config.frame_buffer_count
                           ? self->config.frame_buffer_count
                           : 1;
    if (frame_buffer_count > 2) {
        frame_buffer_count = 2;
    }
    size_t dma_bufs  = self->config.dma_buffer_size
                           ? self->config.dma_buffer_size
                           : 10;

    esp_lcd_rgb_panel_config_t rgb_cfg = {
        .clk_src               = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align     = 64,
        .data_width            = 16,
        .bits_per_pixel        = 16,
        .num_fbs               = frame_buffer_count,
        .bounce_buffer_size_px = dma_bufs * config->width,
        .de_gpio_num           = GUITION_GPIO_DE,
        .pclk_gpio_num         = GUITION_GPIO_PCLK,
        .vsync_gpio_num        = GUITION_GPIO_VSYNC,
        .hsync_gpio_num        = GUITION_GPIO_HSYNC,
        .disp_gpio_num         = -1,
        .data_gpio_nums        = {GUITION_RGB_DATA_PINS},
        .timings               = ST7701_480_480_PANEL_60HZ_RGB_TIMING(),
        .flags = {
            .fb_in_psram = 1,  /* 460 KB framebuffer must live in PSRAM */
        },
    };
    rgb_cfg.timings.pclk_hz = pclk;
    rgb_cfg.timings.flags.pclk_active_neg = self->config.pclk_inverted ? 1 : 0;

    /* -- ST7701S vendor config --------------------------------------------- */
    st7701_vendor_config_t vendor_cfg = {
        .rgb_config     = &rgb_cfg,
        .init_cmds      = guition_4848s040_init_cmds,
        .init_cmds_size = GUITION_INIT_CMDS_SIZE,
        .flags = {
            .mirror_by_cmd       = 0,
            .enable_io_multiplex = 0,
        },
    };
    const esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        /* Match ESP32_Display_Panel reference for this board:
         * RGB bus transfers RGB565 on 16 data lines, while the panel device
         * itself is configured as RGB666 (18 bpp). */
        .bits_per_pixel = 18,
        .vendor_config  = &vendor_cfg,
    };

    ESP_LOGI(TAG, "Creating ST7701 panel (pclk=%"PRIu32" Hz, fb_in_psram=1, num_fbs=%u)...",
             pclk, (unsigned)frame_buffer_count);
    ret = esp_lcd_new_panel_st7701(self->io, &panel_dev_cfg, &self->panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST7701S panel create failed: %d", ret);
        goto fail_io;
    }
    ESP_LOGI(TAG, "ST7701 panel created, resetting...");

    ret = esp_lcd_panel_reset(self->panel);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "panel reset failed: %d", ret); goto fail_panel; }
    ESP_LOGI(TAG, "panel reset OK, initing...");

    ret = esp_lcd_panel_init(self->panel);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "panel init failed: %d", ret); goto fail_panel; }
    ESP_LOGI(TAG, "panel init OK, enabling display...");

    /* Send DISPON (0x29) — the board-specific init sequence ends after SLPOUT
     * without DISPON, so we must turn the display on explicitly. */
    ret = esp_lcd_panel_disp_on_off(self->panel, true);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "panel disp_on failed: %d", ret); goto fail_panel; }
    ESP_LOGI(TAG, "panel display on");

    self->flush_done = xSemaphoreCreateBinary();
    if (self->flush_done == NULL) {
        ret = ESP_ERR_NO_MEM;
        goto fail_panel;
    }
    xSemaphoreGive(self->flush_done);  /* Start signalled */

    self->initialized = true;
    ESP_LOGI(TAG, "Panel ready (%"PRIu32"x%"PRIu32", pclk %"PRIu32" Hz)",
             (uint32_t)config->width, (uint32_t)config->height, pclk);
    return ESP_OK;

fail_panel:
    esp_lcd_panel_del(self->panel);
    self->panel = NULL;
fail_io:
    esp_lcd_panel_io_del(self->io);
    self->io = NULL;
    return ret;
}

static esp_err_t guition_4848s040_deinit(display_driver_handle_t driver)
{
    display_driver_guition_4848s040_t *self = (display_driver_guition_4848s040_t *)driver;

    if (self == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!self->initialized) {
        return ESP_OK;
    }

    if (self->panel) {
        esp_lcd_panel_del(self->panel);
        self->panel = NULL;
    }
    if (self->io) {
        esp_lcd_panel_io_del(self->io);
        self->io = NULL;
    }
    if (self->flush_done) {
        vSemaphoreDelete(self->flush_done);
        self->flush_done = NULL;
    }

    self->initialized = false;
    return ESP_OK;
}

static bool guition_4848s040_is_ready(display_driver_handle_t driver)
{
    const display_driver_guition_4848s040_t *self =
        (const display_driver_guition_4848s040_t *)driver;
    return self != NULL && self->initialized;
}

static esp_err_t guition_4848s040_flush(display_driver_handle_t driver,
                                         uint16_t x1, uint16_t y1,
                                         uint16_t x2, uint16_t y2,
                                         const void *pixel_data,
                                         size_t pixel_data_size)
{
    display_driver_guition_4848s040_t *self = (display_driver_guition_4848s040_t *)driver;

    if (self == NULL || pixel_data == NULL || pixel_data_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!self->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x2 < x1 || y2 < y1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (x2 >= self->config.base.width || y2 >= self->config.base.height) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Take the semaphore so wait_flush_complete blocks until draw_bitmap returns */
    xSemaphoreTake(self->flush_done, portMAX_DELAY);

    esp_err_t ret = esp_lcd_panel_draw_bitmap(
        self->panel, x1, y1, x2 + 1, y2 + 1, pixel_data);

    /* For bounce-buffer RGB panels, draw_bitmap is synchronous; release now.
     * When using interrupt-driven DMA, release from the trans_done callback. */
    xSemaphoreGive(self->flush_done);

    return ret;
}

static esp_err_t guition_4848s040_wait_flush_complete(display_driver_handle_t driver)
{
    display_driver_guition_4848s040_t *self = (display_driver_guition_4848s040_t *)driver;

    if (self == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!self->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Block until flush is done (timeout 1 s) */
    if (xSemaphoreTake(self->flush_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    xSemaphoreGive(self->flush_done);
    return ESP_OK;
}

/* =========================================================================
 * Public factory functions
 * ========================================================================= */

esp_err_t display_driver_guition_4848s040_create(
    const display_driver_guition_4848s040_config_t *config,
    display_driver_t **out_driver)
{
    if (config == NULL || out_driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (config->base.width == 0 || config->base.height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    display_driver_t *wrapper = calloc(1, sizeof(*wrapper));
    if (wrapper == NULL) {
        return ESP_ERR_NO_MEM;
    }

    display_driver_guition_4848s040_t *impl = calloc(1, sizeof(*impl));
    if (impl == NULL) {
        free(wrapper);
        return ESP_ERR_NO_MEM;
    }

    memcpy(&impl->config, config, sizeof(impl->config));
    impl->initialized = false;

    wrapper->vtable = &guition_4848s040_vtable;
    wrapper->impl   = (display_driver_handle_t)impl;

    *out_driver = wrapper;
    return ESP_OK;
}

esp_err_t display_driver_guition_4848s040_destroy(display_driver_t *driver)
{
    if (driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    display_driver_guition_4848s040_t *impl =
        (display_driver_guition_4848s040_t *)driver->impl;

    if (impl != NULL && impl->initialized) {
        guition_4848s040_deinit(impl);
    }

    free(impl);
    free(driver);
    return ESP_OK;
}

const display_driver_guition_4848s040_config_t *
display_driver_guition_4848s040_get_config(const display_driver_t *driver)
{
    if (driver == NULL) {
        return NULL;
    }
    const display_driver_guition_4848s040_t *impl =
        (const display_driver_guition_4848s040_t *)driver->impl;
    return impl ? &impl->config : NULL;
}

esp_err_t display_driver_guition_4848s040_get_framebuffer(
    const display_driver_t *driver,
    void **fb_ptr)
{
    if (driver == NULL || fb_ptr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    const display_driver_guition_4848s040_t *impl =
        (const display_driver_guition_4848s040_t *)driver->impl;
    if (impl == NULL || !impl->initialized || impl->panel == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_lcd_rgb_panel_get_frame_buffer(impl->panel, 1, fb_ptr);
}
