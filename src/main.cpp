#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
extern "C"
{
#include "ws2812.h"
}
#include "lora/radiohal.hpp"
#include "lora/llcc68.hpp"
#include "lora/esp_idf.hpp"

using namespace radiohal;

esp_idf::SPIIoDelegate init_dev()
{
    esp_idf::SpiConfig bus = {
        .spi_host = SPI2_HOST,
        .spi_mosi = GPIO_NUM_4,
        .spi_miso = GPIO_NUM_5,
        .spi_sclk = GPIO_NUM_6,
    };
    esp_idf::SPIIOConfig dev1 = {
        .spi_host = bus.spi_host,
        .spi_nss = GPIO_NUM_7,
        .gpio_busy = GPIO_NUM_NC,
        .gpio_dio1 = GPIO_NUM_3,
        .gpio_reset = GPIO_NUM_9,
    };
    // esp_idf::SPIIOConfig dev2 = {
    //     .spi_host = bus.spi_host,
    //     .spi_nss = 0,
    //     .gpio_busy = -1,
    //     .gpio_dio1 = 1,
    //     .gpio_reset = 9,
    // };

    ESP_ERROR_CHECK(esp_idf::init_spi_bus(bus));

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    return esp_idf::SPIIoDelegate(dev1);
}

void llcc68_rx_thread(void *arg)
{
    Radio *driver = (Radio *)arg;

    uint8_t buffer[100];
    while (true)
    {
        printf("RX ITER\r\n");
        radiohal::length_type length = sizeof(buffer);
        if (driver->recv(&length, buffer, 1000) == radiohal::OK)
        {
            printf("RECV: %s\r\n", buffer);
        }
    }
}

void llcc68_tx_thread(void *arg)
{
    Radio *driver = (Radio *)arg;

    uint8_t buffer[] = "HELLO";
    while (true)
    {
        printf("TX ITER\r\n");
        assert(driver->send(sizeof(buffer), buffer) == OK);
        // Sending a packet takes roughly 30ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ws2812_thread(void *arg)
{
    static DMA_ATTR uint8_t my_ws2812_buffer[WS2812_DMA_SIZE(10)];

    ws2812_driver driver;
    ws2812_config config = {
        .ws2812_pin = 8,
        .spi_host = SPI2_HOST};
    ws2812_init(&driver, &config);

    for (uint8_t i = 0;; i++)
    {
        ws2812_color colors[3] = {
            {.red = 10, .green = 0, .blue = 0},
            {.red = 0, .green = 10, .blue = 0},
            {.red = 0, .green = 0, .blue = 10}};
        ws2812_encode(my_ws2812_buffer, &colors[i % 3], 1);

        ws2812_transmit(&driver, my_ws2812_buffer, 1);

        printf("Meow <3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    // Important: don't lose them when our thread exists
    auto delegate = init_dev();
    llcc68::LLCC68Radio radio(&delegate);
    assert(radio.reset_init() == radiohal::OK);
    assert(radio.configurePhy(PhyConfig{}) == radiohal::OK);

    // TaskHandle_t led_thread_h;
    // xTaskCreate(ws2812_thread, "ws2812_thread", 8000, NULL, tskIDLE_PRIORITY, &led_thread_h);

    llcc68_tx_thread(&radio);
    // TaskHandle_t tx_thread_h;
    // xTaskCreate(llcc68_tx_thread, "llcc68_tx", 8000, &radio, tskIDLE_PRIORITY, &tx_thread_h);
}