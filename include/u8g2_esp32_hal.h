//
// Created by nltimv on 7-6-25.
//

#ifndef U8G2_ESP32_HAL_NEW_H
#define U8G2_ESP32_HAL_NEW_H

#include <i2c_bus.h>
#include <soc/gpio_num.h>
#include <u8g2.h>

typedef struct {
    struct {
        gpio_num_t scl;
        gpio_num_t sda;
        i2c_port_t port;
        uint32_t clk_flags;
        uint32_t clk_speed;
    } i2c;
    struct {
        gpio_num_t rst;
        gpio_num_t btn_menu_select;
        gpio_num_t btn_menu_next;
        gpio_num_t btn_menu_prev;
        gpio_num_t btn_menu_home;
    } gpio;

} u8g2_hal_config_t;

#define U8G2_HAL_CONFIG_DEFAULT {       \
    .i2c = {                            \
        .scl = GPIO_NUM_NC,             \
        .sda = GPIO_NUM_NC,             \
        .port = I2C_NUM_0,              \
        .clk_flags = 0,                 \
        .clk_speed = 400000             \
    },                                  \
    .gpio = {                           \
        .rst = GPIO_NUM_NC,             \
        .btn_menu_select = GPIO_NUM_NC, \
        .btn_menu_next = GPIO_NUM_NC,   \
        .btn_menu_prev = GPIO_NUM_NC,   \
        .btn_menu_home = GPIO_NUM_NC    \
    }                                   \
}                                       \


esp_err_t u8g2_hal_init(const u8g2_hal_config_t *config);
uint8_t u8g2_esp32_hal_i2c_byte_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_esp32_hal_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

#endif //U8G2_ESP32_HAL_NEW_H
