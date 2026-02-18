#include "adc.h"

spi_device_handle_t adc1 = NULL;
spi_device_handle_t adc2 = NULL;

// SPI Bus (host 2) must be initialized before calling this function
void init_adcs() {
    spi_device_interface_config_t device_config = {
        .mode = 0,
        .clock_speed_hz = 32 * 100 * 1000, // 3.2MHz
        .spics_io_num = 4,
        .queue_size = 1,
    };
    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &device_config, &adc1);
    ESP_ERROR_CHECK(ret);

    device_config.spics_io_num = 5;
    ret = spi_bus_add_device(SPI2_HOST, &device_config, &adc2);
    ESP_ERROR_CHECK(ret);
}

void read_adcs(uint8_t buf[16]) {
    read_adc(adc1, buf);
    read_adc(adc2, buf + 8);
}

void read_adc(spi_device_handle_t adc, uint8_t buf[8]) {
    for (int i = 0; i < 9; i++) {
        spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
            .tx_data = {(i & 7) << 3, 0, 0, 0},
            .length = 16,
        };

        esp_err_t ret = spi_device_polling_transmit(adc, &trans);
        ESP_ERROR_CHECK(ret);

        uint8_t val = trans.rx_data[0] << 4;
        val |= trans.rx_data[1] >> 4;
        if (i > 0) {
            buf[i - 1] = val;
        }
    }
}
