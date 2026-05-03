## Sming component: GUITION ESP32-S3-4848S040 display driver
## Implements the display_driver_t interface for the ST7701S RGB panel.

COMPONENT_SRCDIRS  := src
COMPONENT_INCDIRS  := include

COMPONENT_DEPENDS  := esp_lcd_adapter esp_lcd_st7701 esp_lcd_panel_io_additions

## Only meaningful on ESP32 targets (ESP-IDF is required)
COMPONENT_SOC      := esp32*

## Add IDF esp_lcd and esp_psram components to the SDK build and link step.
## esp_psram is required for linking when CONFIG_SPIRAM=y.
SDK_COMPONENTS     += esp_lcd esp_psram
EXTRA_LIBS         += esp_lcd esp_psram

## ESP-IDF headers not in Sming's default SDK_INCDIRS.
## esp_lcd_panel_io.h (IDF v5) pulls in driver/i2c_types.h and hal/i2c_types.h.
COMPONENT_CFLAGS   += \
    -I$(IDF_PATH)/components/esp_lcd/include \
    -I$(IDF_PATH)/components/esp_lcd/interface \
    -I$(IDF_PATH)/components/driver/i2c/include \
    -I$(IDF_PATH)/components/hal/include
