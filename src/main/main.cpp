#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "driver/i2s_std.h"
#include <math.h>
#include <format>
#include "buttons.hpp"
#include "lcd.hpp"

const char *TAG = "test";

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                    mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

esp_err_t get_handler(httpd_req_t *req) {
    /* Send a simple response */
    const char resp[] = "URI GET Response";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

#define LED_NUMBER 14
#define PIXEL_SIZE 12 // each colour takes 4 bytes
#define ZERO_BUFFER 96

typedef struct {
  uint8_t green;
  uint8_t red;
  uint8_t blue;
} ws2812_pixel_t;

i2s_chan_handle_t ws2182_i2s_hand;

static uint8_t out_buffer[LED_NUMBER * PIXEL_SIZE] = {0};
static uint8_t off_buffer[ZERO_BUFFER] = {0};

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

void ws2812_init() {
    i2s_chan_config_t chan_conf = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_conf, &ws2182_i2s_hand, NULL));

    i2s_std_config_t conf = {
        .clk_cfg = {
            .sample_rate_hz = 69444,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
            .ws_pol = false,
            .bit_shift = false
        },
        .gpio_cfg = {
            .mclk = GPIO_NUM_NC,
            .bclk = GPIO_NUM_NC,
            .ws = GPIO_NUM_NC,
            .dout = GPIO_NUM_22,
            .din = GPIO_NUM_NC,
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(ws2182_i2s_hand, &conf));

    ESP_ERROR_CHECK(i2s_channel_enable(ws2182_i2s_hand));
}

void ws2812_update(ws2812_pixel_t *pixels) {
    size_t bytes_written = 0;

    for (uint16_t i = 0; i < LED_NUMBER; i++) {
        int loc = i * PIXEL_SIZE;

        out_buffer[loc] = bitpatterns[pixels[i].red >> 6 & 0x03];
        out_buffer[loc + 1] = bitpatterns[pixels[i].red >> 4 & 0x03];
        out_buffer[loc + 2] = bitpatterns[pixels[i].red >> 2 & 0x03];
        out_buffer[loc + 3] = bitpatterns[pixels[i].red & 0x03];

        out_buffer[loc + 4] = bitpatterns[pixels[i].green >> 6 & 0x03];
        out_buffer[loc + 5] = bitpatterns[pixels[i].green >> 4 & 0x03];
        out_buffer[loc + 6] = bitpatterns[pixels[i].green >> 2 & 0x03];
        out_buffer[loc + 7] = bitpatterns[pixels[i].green & 0x03];

        out_buffer[loc + 8] = bitpatterns[pixels[i].blue >> 6 & 0x03];
        out_buffer[loc + 9] = bitpatterns[pixels[i].blue >> 4 & 0x03];
        out_buffer[loc + 10] = bitpatterns[pixels[i].blue >> 2 & 0x03];
        out_buffer[loc + 11] = bitpatterns[pixels[i].blue & 0x03];
    }

    ESP_ERROR_CHECK(i2s_channel_write(ws2182_i2s_hand, out_buffer, PIXEL_SIZE * LED_NUMBER, &bytes_written, portMAX_DELAY));
    ESP_ERROR_CHECK(i2s_channel_write(ws2182_i2s_hand, off_buffer, ZERO_BUFFER, &bytes_written, portMAX_DELAY));
    vTaskDelay(pdMS_TO_TICKS(10));
}

i2s_chan_handle_t amplifier_hand;
extern const uint8_t chime_pcm_start[] asm("_binary_chime_pcm_start");
extern const uint8_t chime_pcm_end[] asm("_binary_chime_pcm_end");

void handle_button_press(void *arg) {
    int bnum = *(int*)arg;
    ESP_LOGI("IDK", "button press %d", bnum);

    lcd_write_text(std::format("{}", bnum));

    // ESP_ERROR_CHECK(i2s_channel_enable(amplifier_hand));

    // size_t bytes_written;
    // ESP_ERROR_CHECK(i2s_channel_write(amplifier_hand, desert_pcm_start, desert_pcm_end - desert_pcm_start, &bytes_written, portMAX_DELAY));

    // ESP_ERROR_CHECK(i2s_channel_disable(amplifier_hand));

    
}

extern "C" void app_main(void) {
    printf("Hello world!\n");

    gpio_reset_pin(GPIO_NUM_5);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_5, 1);
    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 1);

    // gpio_reset_pin(15);
    // gpio_set_direction(15, GPIO_MODE_OUTPUT);

    // for (int i = 5; i >= 0; i--) {
    //     gpio_set_level(15, true);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     printf("Hello world! %d\n", i);
    //     gpio_set_level(15, false);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    spi_bus_config_t spi_config = {
        .mosi_io_num = 33,
        .miso_io_num = 34,
        .sclk_io_num = 32,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &spi_config, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // // WS2182
    // ws2812_init();
    // ws2812_pixel_t pixels[LED_NUMBER];
    // for (int i = 0; i < LED_NUMBER; i++) {
    //     pixels[i] = {0, 2, 0};
    // }

    // ws2812_update(pixels);

    // RS485
    // uart_config_t uart_config = {
    //     .baud_rate = 9600,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    // };
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 12, 13, -1, -1));

    // const int uart_buffer_size = (1024 * 2);
    // QueueHandle_t uart_queue;
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

    // while (1) {
    //     const char *t = "Hello world from Concrete v1.0!";
    //     uart_write_bytes(UART_NUM_2, t, strlen(t));
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }

    // Ethernet
    // // Enable the ethernet clock
    // gpio_reset_pin(16);
    // gpio_set_direction(16, GPIO_MODE_OUTPUT);
    // gpio_set_level(16, true);

    // // Setup ethernet mac and phy
    // eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    // eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    // esp32_emac_config.smi_gpio.mdc_num = 18;
    // esp32_emac_config.smi_gpio.mdio_num = 17;
    // esp32_emac_config.interface = EMAC_DATA_INTERFACE_RMII;
    // esp32_emac_config.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    // esp32_emac_config.clock_config.rmii.clock_gpio = EMAC_CLK_IN_GPIO;
    // esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    // eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    // phy_config.phy_addr = 0;
    // phy_config.reset_gpio_num = 2;
    // esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

    // esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy); // apply default driver configuration
    // esp_eth_handle_t eth_handle = NULL; // after the driver is installed, we will get the handle of the driver
    // esp_eth_driver_install(&config, &eth_handle); // install driver


    // esp_event_loop_create_default(); // create a default event loop that runs in the background
    // esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL);
    

    // esp_netif_init(); // Initialize TCP/IP network interface (should be called only once in application)
    // esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH(); // apply default network interface configuration for Ethernet
    // esp_netif_t *eth_netif = esp_netif_new(&cfg); // create network interface for Ethernet driver

    // esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)); // attach Ethernet driver to TCP/IP stack
    // esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL); // register user defined IP event handlers
    // esp_eth_start(eth_handle); // start Ethernet driver state machine

    // // HTTP server
    // httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    // httpd_handle_t server = NULL;
    // ret = httpd_start(&server, &http_config);
    // ESP_ERROR_CHECK(ret);

    // httpd_uri_t uri_get = {
    //     .uri = "/",
    //     .method = HTTP_GET,
    //     .handler = get_handler,
    //     .user_ctx = NULL
    // };
    // httpd_register_uri_handler(server, &uri_get);

    // GPIO Expander SPI
    spi_device_interface_config_t expander_config = {
        .command_bits = 8,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = 32 * 100 * 1000, // 3.2MHz
        .spics_io_num = 23,
        .queue_size = 1,
    };

    spi_device_handle_t gpex_handle;
    ret = spi_bus_add_device(SPI2_HOST, &expander_config, &gpex_handle);
    ESP_ERROR_CHECK(ret);

    // Set pin directions
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
        .cmd = 0b01000000,
        .addr = 0x00,
        .length = 8,
        .tx_data = {0, 0, 0, 0},
    };

    ret = spi_device_polling_transmit(gpex_handle, &trans);
    ESP_ERROR_CHECK(ret);

    trans = (struct spi_transaction_t) {
        .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
        .cmd = 0b01000000,
        .addr = 0x12,
        .length = 8,
        .tx_data = {1 << 2, 0, 0, 0},
    };

    ret = spi_device_polling_transmit(gpex_handle, &trans);
    ESP_ERROR_CHECK(ret);

    // Read back data
    trans = {
        .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
        .cmd = 0b01000001,
        .addr = 0x12,
        .length = 8,
        .tx_data = {0, 0, 0, 0},
    };

    ret = spi_device_polling_transmit(gpex_handle, &trans);
    ESP_ERROR_CHECK(ret);
    printf("%d\n", trans.rx_data[0]);
    vTaskDelay(15000 / portTICK_PERIOD_MS);

    // Turn GPIOA4 on and off
    // while (1) {
    //     trans = (struct spi_transaction_t) {
    //         .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
    //         .cmd = 0b01000000,
    //         .addr = 0x12,
    //         .tx_data = {1 << 6, 0, 0, 0},
    //         .length = 8,
    //     };

    //     ret = spi_device_polling_transmit(gpex_handle, &trans);
    //     ESP_ERROR_CHECK(ret);

    //     vTaskDelay(3000 / portTICK_PERIOD_MS);

    //     trans = (struct spi_transaction_t) {
    //         .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
    //         .cmd = 0b01000000,
    //         .addr = 0x12,
    //         .tx_data = {0, 0, 0, 0},
    //         .length = 8,
    //     };

    //     ret = spi_device_polling_transmit(gpex_handle, &trans);
    //     ESP_ERROR_CHECK(ret);

    //     vTaskDelay(3000 / portTICK_PERIOD_MS);
    // }

    // ADC SPI
    spi_device_interface_config_t device_config = {
        .mode = 0,
        .clock_speed_hz = 32 * 100 * 1000, // 3.2MHz
        .spics_io_num = 4,
        .queue_size = 1,
    };
    // spi_device_interface_config_t device_config2 = {
    //     .mode = 0,
    //     .clock_speed_hz = 32 * 100 * 1000, // 3.2MHz
    //     .spics_io_num = 5,
    //     .queue_size = 1,
    // };

    spi_device_handle_t adc1_handle;
    // spi_device_handle_t adc2_handle;
    ret = spi_bus_add_device(SPI2_HOST, &device_config, &adc1_handle);
    ESP_ERROR_CHECK(ret);
    // ret = spi_bus_add_device(SPI2_HOST, &device_config2, &adc2_handle);
    // ESP_ERROR_CHECK(ret);

    while (1) {
        for (int i = 0; i < 8; i++) {
            spi_transaction_t trans = {
                .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
                .length = 32,
                .tx_data = {(uint8_t)(i << 3), 0, 0, 0},
            };

            ret = spi_device_polling_transmit(adc1_handle, &trans);
            ESP_ERROR_CHECK(ret);

            uint32_t val = trans.rx_data[2] << 8;
            val |= trans.rx_data[3];
            printf("%ld ", val);

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        // for (int i = 0; i < 9; i++) {
        //     spi_transaction_t trans = {
        //         .flags = SPI_TRANS_USE_RXDATA|SPI_TRANS_USE_TXDATA,
        //         .length = 16,
        //         .tx_data = {(uint8_t)((i & 7) << 3), 0, 0, 0},
        //     };

        //     ret = spi_device_polling_transmit(adc2_handle, &trans);
        //     ESP_ERROR_CHECK(ret);

        //     uint16_t val = trans.rx_data[0] << 8;
        //     val |= trans.rx_data[1];
        //     if (i != 0) {
        //         printf("%d ", val);
        //     }
        // }
        printf("\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    // I2S
    // i2s_chan_config_t chan_conf = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    // ESP_ERROR_CHECK(i2s_new_channel(&chan_conf, &amplifier_hand, NULL));

    // i2s_std_config_t conf = {
    //     .clk_cfg = {
    //         .sample_rate_hz = 16000,
    //         .clk_src = I2S_CLK_SRC_DEFAULT,
    //         .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    //     },
    //     .slot_cfg = {
    //         .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
    //         .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
    //         .slot_mode = I2S_SLOT_MODE_MONO,
    //         .slot_mask = I2S_STD_SLOT_BOTH,
    //         .ws_pol = false,
    //         .bit_shift = true
    //     },
    //     .gpio_cfg = {
    //         .mclk = GPIO_NUM_NC,
    //         .bclk = GPIO_NUM_5,
    //         .ws = GPIO_NUM_19,
    //         .dout = GPIO_NUM_21,
    //         .din = GPIO_NUM_NC,
    //     }
    // };
    // ESP_ERROR_CHECK(i2s_channel_init_std_mode(amplifier_hand, &conf));

    // ESP_ERROR_CHECK(i2s_channel_enable(amplifier_hand));

    // size_t bytes_written;
    // ESP_ERROR_CHECK(i2s_channel_write(amplifier_hand, chime_pcm_start, chime_pcm_end - chime_pcm_start, &bytes_written, portMAX_DELAY));

    // ESP_ERROR_CHECK(i2s_channel_disable(amplifier_hand));

    // /* Print chip information */
    // esp_chip_info_t chip_info;
    // uint32_t flash_size;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
    //        CONFIG_IDF_TARGET,
    //        chip_info.cores,
    //        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
    //        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
    //        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
    //        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    // unsigned major_rev = chip_info.revision / 100;
    // unsigned minor_rev = chip_info.revision % 100;
    // printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    // if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    //     printf("Get flash size failed");
    //     return;
    // }

    // printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
    //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // gpio_reset_pin(15);
    // gpio_set_direction(15, GPIO_MODE_OUTPUT);

    // for (int i = 5; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i*2);
    //     gpio_set_level(15, true);
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    //     gpio_set_level(15, false);
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();

    // init_buttons(handle_button_press);
    // lcd_init();

    // int b = 0;
    // while (1) {
    //     poll_buttons();
    //     vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
}
