# U8g2 ESP32 HAL
Hardware Abstraction Layer (HAL) component for the U8g2 library for the ESP32 (ESP-IDF framework)

## Features
 - Supports hardware I2C communication;
 - Uses the [espressif/i2c_bus](https://components.espressif.com/components/espressif/i2c_bus) library to allow thread-safe I2C communication;
 - Supports the menu home/prev/next/select button for MUI.

**Note:** SPI communication is not supported yet.

## Example
See the [hello-world example](examples/hello-world) for a basic example on how to use this component