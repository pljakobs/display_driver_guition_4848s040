#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_adapter/display_driver_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Board-specific configuration for GUITION-4848S040.
 * Extends the base display_driver_config_t with board-specific fields.
 */
typedef struct {
    display_driver_config_t base;  // Must be first (width, height)

    // Hardware pin mappings (from ESPHome model)
    // DE, HSYNC, VSYNC, PCLK pins are statically defined; GPIO assignments
    // can be overridden via build-time config if needed.

    // DMA and timing parameters
    uint32_t pclk_frequency_hz;  // Default: 12MHz
    bool pclk_inverted;          // Default: false
    uint32_t dma_buffer_size;    // Bytes; default: computed from resolution
} display_driver_guition_4848s040_config_t;

typedef struct display_driver_guition_4848s040 display_driver_guition_4848s040_t;

/**
 * Create a new display driver instance implementing the display_driver_t interface.
 *
 * Returns an allocated display_driver_t with vtable and impl set.
 * Caller must call display_driver_guition_4848s040_destroy() to release.
 */
esp_err_t display_driver_guition_4848s040_create(
    const display_driver_guition_4848s040_config_t *config,
    display_driver_t **out_driver);

/**
 * Destroy a display driver instance and release all resources.
 * Calls deinit internally if not already called.
 */
esp_err_t display_driver_guition_4848s040_destroy(display_driver_t *driver);

/**
 * Get the board-specific runtime config from a driver instance.
 * Used internally and for diagnostics.
 */
const display_driver_guition_4848s040_config_t *display_driver_guition_4848s040_get_config(
    const display_driver_t *driver);

#ifdef __cplusplus
}
#endif
