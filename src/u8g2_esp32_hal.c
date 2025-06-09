//
// Created by nltimv on 7-6-25.
//

#include <string.h>
#include <u8g2_esp32_hal.h>

#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"

#define BITMASK_PIN_ADD(x) (x != GPIO_NUM_NC ? 1ULL << (x) : 0)

static const char *TAG = "u8g2_esp32_hal";

static u8g2_hal_config_t *u8g2_hal_config = NULL;
static i2c_bus_handle_t i2c_bus = NULL;
static i2c_bus_handle_t i2c_dev = NULL;

static uint8_t data_buf[32];
static uint8_t data_buf_idx;

esp_err_t u8g2_hal_init(const u8g2_hal_config_t *config) {
    u8g2_hal_config = malloc(sizeof(u8g2_hal_config_t));
    memcpy(u8g2_hal_config, config, sizeof(u8g2_hal_config_t));
    return ESP_OK;
}

uint8_t u8g2_esp32_hal_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    ESP_LOGD(TAG, "i2c_cb: Received a msg: %d, arg_int: %d, arg_ptr: %p", msg,
           arg_int, arg_ptr);
    uint8_t *data_ptr;
    switch (msg) {
        case U8X8_MSG_BYTE_INIT:
            if (u8g2_hal_config == NULL) {
                ESP_LOGE(TAG, "u8g2_esp32_hal is not initialized!");
                return 0;
            }
            const i2c_config_t i2c_config = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = u8g2_hal_config->i2c.sda,
                .scl_io_num = u8g2_hal_config->i2c.scl,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
                .master.clk_speed = u8g2_hal_config->i2c.clk_speed
            };
            i2c_bus = i2c_bus_create(u8g2_hal_config->i2c.port, &i2c_config);
            if (i2c_bus == NULL) {
                ESP_LOGE(TAG, "i2c_bus_create failed");
                return 0;
            }
            break;
        case U8X8_MSG_BYTE_START_TRANSFER:
            if (i2c_bus == NULL) {
                ESP_LOGE(TAG, "i2c_bus not initialized!");
                return 0;
            }
            const uint8_t i2c_address = u8x8_GetI2CAddress(u8x8) >> 1;
            ESP_LOGD(TAG, "Start I2C transfer to %02X.", i2c_address);
            i2c_dev = i2c_bus_device_create(i2c_bus, i2c_address, 0);
            if (i2c_dev == NULL) {
                ESP_LOGE(TAG, "i2c_bus_device_create failed");
                return 0;
            }
            break;
        case U8X8_MSG_BYTE_SEND:
            data_ptr = (uint8_t*)arg_ptr;
            ESP_LOG_BUFFER_HEXDUMP(TAG, data_ptr, arg_int, ESP_LOG_VERBOSE);
            while (arg_int > 0) {
                data_buf[data_buf_idx++] = *data_ptr;
                data_ptr++;
                arg_int--;
            }
            break;
        case U8X8_MSG_BYTE_END_TRANSFER: {
            if (i2c_dev == NULL) {
                ESP_LOGE(TAG, "i2c_bus_device not initialized!");
                return 0;
            }
            ESP_LOGD(TAG, "End I2C transfer to %02X.", u8x8_GetI2CAddress(u8x8) >> 1);
            while (ESP_ERROR_CHECK_WITHOUT_ABORT(
                i2c_bus_write_bytes(i2c_dev, NULL_I2C_MEM_ADDR, data_buf_idx, data_buf)) != ESP_OK) {
                vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            i2c_bus_device_delete(&i2c_dev);
            i2c_dev = NULL;
            data_buf_idx = 0;
        }
        break;
        case U8X8_MSG_BYTE_SET_DC:
            break;
        default:
            return 0;
    }
    return 1;
}

uint8_t u8g2_esp32_hal_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
    switch (msg) {
        case U8X8_MSG_GPIO_AND_DELAY_INIT:
            if (u8g2_hal_config == NULL) {
                ESP_LOGE(TAG, "u8g2_esp32_hal is not initialized!");
                return 0;
            }
            // Initialize RST output pin
            unsigned long long bitmask;
            bitmask = BITMASK_PIN_ADD(u8g2_hal_config->gpio.rst);
            if (bitmask > 0) {
                const gpio_config_t gpio_conf_rst = {
                    .pin_bit_mask = bitmask,
                    .mode = GPIO_MODE_OUTPUT,
                    .pull_up_en = GPIO_PULLUP_DISABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE
                };
                gpio_config(&gpio_conf_rst);
            }

            //Initialize I2C output pins
            bitmask = BITMASK_PIN_ADD(u8g2_hal_config->i2c.scl) | BITMASK_PIN_ADD(u8g2_hal_config->i2c.sda);
            if (bitmask > 0) {
                const gpio_config_t gpio_conf_i2c = {
                    .pin_bit_mask = bitmask,
                    .mode = GPIO_MODE_OUTPUT,
                    .pull_up_en = GPIO_PULLUP_ENABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE
                };
                gpio_config(&gpio_conf_i2c);
            }

            //Initialize menu input buttons
            bitmask = BITMASK_PIN_ADD(u8g2_hal_config->gpio.btn_menu_home)
                    | BITMASK_PIN_ADD(u8g2_hal_config->gpio.btn_menu_prev)
                    | BITMASK_PIN_ADD(u8g2_hal_config->gpio.btn_menu_next)
                    | BITMASK_PIN_ADD(u8g2_hal_config->gpio.btn_menu_select);
            if (bitmask > 0) {
                const gpio_config_t gpio_config_menu = {
                    .pin_bit_mask = bitmask,
                    .mode = GPIO_MODE_INPUT,
                    .pull_up_en = GPIO_PULLUP_DISABLE,
                    .pull_down_en = GPIO_PULLDOWN_DISABLE,
                    .intr_type = GPIO_INTR_DISABLE
                };
                gpio_config(&gpio_config_menu);
            }
            break;
        // GPIO handlers
        case U8X8_MSG_GPIO_RESET:
            if (u8g2_hal_config->gpio.rst != GPIO_NUM_NC) {
                gpio_set_level(u8g2_hal_config->gpio.rst, arg_int);
            }
            break;
        case U8X8_MSG_GPIO_I2C_CLOCK:
            if (u8g2_hal_config->i2c.scl != GPIO_NUM_NC) {
                gpio_set_level(u8g2_hal_config->i2c.scl, arg_int);
            }
            break;
        case U8X8_MSG_GPIO_I2C_DATA:
            if (u8g2_hal_config->i2c.sda != GPIO_NUM_NC) {
                gpio_set_level(u8g2_hal_config->i2c.sda, arg_int);
            }
            break;
        case U8X8_MSG_GPIO_MENU_HOME:
            if (u8g2_hal_config->gpio.btn_menu_home != GPIO_NUM_NC) {
                u8x8_SetGPIOResult(u8x8, gpio_get_level(u8g2_hal_config->gpio.btn_menu_home));
            }
            break;
        case U8X8_MSG_GPIO_MENU_PREV:
            if (u8g2_hal_config->gpio.btn_menu_prev != GPIO_NUM_NC) {
                u8x8_SetGPIOResult(u8x8, gpio_get_level(u8g2_hal_config->gpio.btn_menu_prev));
            }
            break;
        case U8X8_MSG_GPIO_MENU_NEXT:
            if (u8g2_hal_config->gpio.btn_menu_next != GPIO_NUM_NC) {
                u8x8_SetGPIOResult(u8x8, gpio_get_level(u8g2_hal_config->gpio.btn_menu_next));
            }
            break;
        case U8X8_MSG_GPIO_MENU_SELECT:
            if (u8g2_hal_config->gpio.btn_menu_select != GPIO_NUM_NC) {
                u8x8_SetGPIOResult(u8x8, gpio_get_level(u8g2_hal_config->gpio.btn_menu_select));
            }
        // Delay handlers
        case U8X8_MSG_DELAY_MILLI:
            vTaskDelay(arg_int / portTICK_PERIOD_MS);
            break;
        default:
            return 0;
    }
    return 1;
}
