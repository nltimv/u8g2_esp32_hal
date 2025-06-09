#include <esp_log.h>
#include <u8g2_esp32_hal.h>
#include <u8g2.h>

// Set the GPIO pins that your display is connected to
#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

// Optionally, override the I2C address and contrast here
// #define U8G2_I2C_ADDR
// #define U8G2_CONTRAST 0

// Set the constructor specific to your display here
#define U8G2_CTR u8g2_Setup_st7567_i2c_jlx12864_f

static const char *TAG = "hello-world";

static u8g2_t u8g2;

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing display...");
    u8g2_hal_config_t u8g2_hal_config = U8G2_HAL_CONFIG_DEFAULT;
    u8g2_hal_config.i2c.scl = PIN_I2C_SCL;
    u8g2_hal_config.i2c.sda = PIN_I2C_SDA;
    u8g2_hal_init(&u8g2_hal_config);

    U8G2_CTR(
      &u8g2,
      U8G2_R2,
      u8g2_esp32_hal_i2c_byte_cb,
      u8g2_esp32_hal_gpio_and_delay_cb);

#if U8G2_I2C_ADDR > 0
    u8g2_SetI2CAddress(&u8g2, U8G2_I2C_ADDR << 1);
#endif
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
#if U8G2_CONTRAST > 0
    u8g2_SetContrast(&u8g2, U8G2_CONTRAST);
#endif
    ESP_LOGI(TAG, "Display initialized");

    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_logisoso16_tf);
    u8g2_DrawUTF8(&u8g2, 0, 20, "Hello World!");
    u8g2_SendBuffer(&u8g2);

    ESP_LOGI(TAG, "Display rendered");
}
