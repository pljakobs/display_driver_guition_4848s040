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

/* RGB data pins: R0-R4, G0-G5, B0-B4 (16-bit RGB565) */
#define GUITION_RGB_DATA_PINS  \
    11, 12, 13, 14,  0,        \
     8, 20,  3, 46,  9, 10,    \
     4,  5,  6,  7, 15

/* Extra vendor init command required by GUITION panel (0xCD = pixel format) */
static const st7701_lcd_init_cmd_t guition_extra_init[] = {
    {0xCD, (uint8_t[]){0x00}, 1, 0},
};

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
    ret = esp_lcd_new_panel_io_3wire_spi(&io_cfg, &self->io);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "3-wire SPI IO create failed: %d", ret);
        return ret;
    }

    /* -- RGB panel config -------------------------------------------------- */
    uint32_t pclk    = self->config.pclk_frequency_hz
                           ? self->config.pclk_frequency_hz
                           : 12000000U;
    size_t dma_bufs  = self->config.dma_buffer_size
                           ? self->config.dma_buffer_size
                           : 10;

    esp_lcd_rgb_panel_config_t rgb_cfg = {
        .clk_src               = LCD_CLK_SRC_DEFAULT,
        .psram_trans_align     = 64,
        .data_width            = 16,
        .bits_per_pixel        = 16,
        .num_fbs               = 1,
        .bounce_buffer_size_px = dma_bufs * config->width,
        .de_gpio_num           = GUITION_GPIO_DE,
        .pclk_gpio_num         = GUITION_GPIO_PCLK,
        .vsync_gpio_num        = GUITION_GPIO_VSYNC,
        .hsync_gpio_num        = GUITION_GPIO_HSYNC,
        .disp_gpio_num         = -1,
        .data_gpio_nums        = {GUITION_RGB_DATA_PINS},
        .timings               = ST7701_480_480_PANEL_60HZ_RGB_TIMING(),
        .flags = {
            .pclk_active_neg = self->config.pclk_inverted ? 1 : 0,
        },
    };
    rgb_cfg.timings.pclk_hz = pclk;

    /* -- ST7701S vendor config --------------------------------------------- */
    st7701_vendor_config_t vendor_cfg = {
        .rgb_config     = &rgb_cfg,
        .init_cmds      = guition_extra_init,
        .init_cmds_size = sizeof(guition_extra_init) / sizeof(guition_extra_init[0]),
        .flags = {
            .mirror_by_cmd       = 1,
            .enable_io_multiplex = 0,
        },
    };
    const esp_lcd_panel_dev_config_t panel_dev_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config  = &vendor_cfg,
    };

    ret = esp_lcd_new_panel_st7701(self->io, &panel_dev_cfg, &self->panel);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ST7701S panel create failed: %d", ret);
        goto fail_io;
    }

    ret = esp_lcd_panel_reset(self->panel);
    if (ret != ESP_OK) { goto fail_panel; }

    ret = esp_lcd_panel_init(self->panel);
    if (ret != ESP_OK) { goto fail_panel; }

    /* Mirror to match LVGL default 180° rotation for this panel */
    ret = esp_lcd_panel_mirror(self->panel, true, true);
    if (ret != ESP_OK) { goto fail_panel; }

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
