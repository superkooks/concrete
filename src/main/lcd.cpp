#include <lcd.hpp>
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "string.h"

#define PIN_DB7 GPIO_NUM_2
#define PIN_DB6 GPIO_NUM_25
#define PIN_DB5 GPIO_NUM_26
#define PIN_DB4 GPIO_NUM_27
#define PIN_RS GPIO_NUM_33
#define PIN_E GPIO_NUM_32
#define PIN_DIM GPIO_NUM_4

void lcd_init() {
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_DB7));
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_DB6));
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_DB5));
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_DB4));
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_RS));
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_E));
    ESP_ERROR_CHECK(gpio_reset_pin(PIN_DIM));

    ESP_ERROR_CHECK(gpio_set_direction(PIN_DB7, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_DB6, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_DB5, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_DB4, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_RS, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_E, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(PIN_DIM, GPIO_MODE_OUTPUT));

    ESP_ERROR_CHECK(gpio_set_level(PIN_DB7, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB6, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB5, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB4, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_RS, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_E, 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DIM, 0));

    // Turn on backlight
    ESP_ERROR_CHECK(gpio_set_level(PIN_DIM, 1));
    ets_delay_us(500000);

    // Initialize controller???
    ESP_LOGI("TEST", "initing lcd");
    lcd_write_bits(0, 0b0011);
    ets_delay_us(100000); // ets_delay_us(4500);
    lcd_write_bits(0, 0b0011);
    ets_delay_us(100000); // ets_delay_us(4500);
    lcd_write_bits(0, 0b0011);

    // Third time's the charm. Set to 4-bit interface
    ets_delay_us(100000); // ets_delay_us(150);
    lcd_write_bits(0, 0b0010);

    // Set the function to 4-bit, 2 lines
    lcd_write_bits(0, 0b0010);
    lcd_write_bits(0, 0b1000);

    // Turn the display on, with a blinking cursor
    lcd_write_bits(0, 0b0000);
    lcd_write_bits(0, 0b1100);

    vTaskDelay(2);

    // Clear the display
    lcd_write_bits(0, 0b0000);
    lcd_write_bits(0, 0b0001);

    // Set the entry mode to increment (with no shift)
    lcd_write_bits(0, 0b0000);
    lcd_write_bits(0, 0b0110);

    // Go home
    lcd_write_bits(0, 0b0000);
    lcd_write_bits(0, 0b0010);

    ets_delay_us(20000);
    lcd_write_text("->");

    ESP_LOGI("TEST", "done");
}

void lcd_write_bits(bool rs, uint8_t b) {
    ESP_ERROR_CHECK(gpio_set_level(PIN_RS, rs ? 1 : 0));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB4, (b) & 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB5, (b >> 1) & 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB6, (b >> 2) & 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_DB7, (b >> 3) & 1));
    ESP_ERROR_CHECK(gpio_set_level(PIN_E, 1));
    ets_delay_us(20);
    ESP_ERROR_CHECK(gpio_set_level(PIN_E, 0));
    ets_delay_us(20);
}

void lcd_write_text(std::string text) {
    for (int i = 0; i < text.length(); i++) {
        lcd_write_bits(1, text[i] >> 4);
        lcd_write_bits(1, text[i] & 0xf);
    }
}
