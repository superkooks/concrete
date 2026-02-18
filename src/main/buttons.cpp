#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "buttons.hpp"
#include "esp_timer.h"
#include <vector>
#include <memory>
#include "rom/ets_sys.h"

spi_device_handle_t gpex_handle;
std::vector<Debounced> debounces;

void init_buttons(void (*callback)(void *)) {
    // Setup debouncers
    for (int i = 0; i < 16; i++) {
        debounces.push_back(Debounced(50, callback));
    }

    // Keypad GPIO Expander Buttons
    spi_device_interface_config_t expander_config = {
        .command_bits = 8,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = 32 * 100 * 1000, // 3.2MHz
        .spics_io_num = 18,
        .queue_size = 1,
    };

    
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &expander_config, &gpex_handle));

    // Set interrupts
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
        .cmd = 0b01000000,
        .addr = 0x04, // Bank A
        .length = 8,
        .tx_data = {0xff, 0, 0, 0},
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(gpex_handle, &trans));

    trans = {
        .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
        .cmd = 0b01000000,
        .addr = 0x05, // Bank B
        .length = 8,
        .tx_data = {0xff, 0, 0, 0},
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(gpex_handle, &trans));

    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_34, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT));
}

void poll_buttons() {
    // // Read from gpio expander
    // spi_transaction_t trans = {
    //     .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
    //     .cmd = 0b01000001,
    //     .addr = 0x12, // Bank A
    //     .length = 8,
    //     .tx_data = {0, 0, 0, 0},
    // };
    // ESP_ERROR_CHECK(spi_device_polling_transmit(gpex_handle, &trans));

    // for (int i = 0; i < 8; i++) {
    //     // TODO Remove
    //     if ((trans.rx_data[0] & (1 << i)) == 0) {
    //         ESP_LOGI("TEST", "triggered! BANK A %d", i);
    //         while (1) {

    //         }
    //     }
        
    //     debounces[i].update((trans.rx_data[0] & (1 << i)) == 0, &i);
    // }

    // trans = {
    //     .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
    //     .cmd = 0b01000001,
    //     .addr = 0x13, // Bank B
    //     .length = 8,
    //     .tx_data = {0, 0, 0, 0},
    // };
    // ESP_ERROR_CHECK(spi_device_polling_transmit(gpex_handle, &trans));

    // for (int i = 8; i < 16; i++) {
    //     // TODO Remove
    //     if ((trans.rx_data[0] & (1 << (i - 8))) == 0) {
    //         ESP_LOGI("TEST", "triggered! BANK B %d", i-8);
    //         while (1) {

    //         }
    //     }

    //     debounces[i].update((trans.rx_data[0] & (1 << (i - 8))) == 0, &i);
    // }

    if (gpio_get_level(GPIO_NUM_34) == 0) {
        ESP_LOGI("TEST", "triggered! BANK A");

        spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
            .cmd = 0b01000001,
            .addr = 0x12, // Bank A
            .length = 8,
            .tx_data = {0, 0, 0, 0},
        };
        ESP_ERROR_CHECK(spi_device_polling_transmit(gpex_handle, &trans));

        for (int i = 0; i < 8; i++) {
            if ((trans.rx_data[0] & (1 << i)) == 0) {
                ESP_LOGI("TEST", "read! BANK A %d", i);
            }
            
            // debounces[i].update((trans.rx_data[0] & (1 << i)) == 0, &i);
        }
    }

    if (gpio_get_level(GPIO_NUM_35) == 0) {
        ESP_LOGI("TEST", "triggered! BANK B");

        spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
            .cmd = 0b01000001,
            .addr = 0x13, // Bank B
            .length = 8,
            .tx_data = {0, 0, 0, 0},
        };
        ESP_ERROR_CHECK(spi_device_polling_transmit(gpex_handle, &trans));

        for (int i = 8; i < 16; i++) {
            if ((trans.rx_data[0] & (1 << (i - 8))) == 0) {
                ESP_LOGI("TEST", "read! BANK B %d", i-8);
            }

            // debounces[i].update((trans.rx_data[0] & (1 << (i - 8))) == 0, &i);
        }
    }
}

Debounced::Debounced(int delay_ms, void (*callback)(void *)) {
    this->delay = delay_ms * 1000;
    this->callback = callback;
    this->last_call = 0;
    this->cur_value = false;
}

void Debounced::update(bool next_value, void *arg) {
    if (next_value != cur_value && next_value) {
        uint32_t t = esp_timer_get_time();
        if (t > last_call + delay) {
            last_call = t;
            callback(arg);
        }
    }

    cur_value = next_value;
}
