# display_driver_guition_4848s040

Board-specific display driver component for the Guition ESP32-S3-4848S040 panel.

## Scope

- Provide a small, stable C API for panel lifecycle and frame transfer hooks.
- Keep board/panel-specific details inside this component.
- Remain independent from product UI logic.

## Status

Initial scaffold with API contract and a stub implementation.

## Build

This repository is structured as an ESP-IDF component.

From an ESP-IDF project that consumes this component:

1. Add this repository as a component dependency.
2. Build with `idf.py build`.

## Next Steps

- Wire up actual `esp_lcd` panel bring-up for GUITION-4848S040.
- Add DMA-capable flush path and synchronization.
- Add integration tests against a minimal LVGL runtime setup.
