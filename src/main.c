#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ws2812.h"
#include "llcc68.h"
#include "lora/llc68_idf.h"

void llcc68_tx_thread(void *arg) {
    llc68_module driver = {0};
    llc68_config config = {
        .gpio_dio1 = 4,
        .spi_host = SPI2_HOST,
        .spi_miso = 5,
        .spi_mosi = 6,
        .spi_sclk = 7,
        .spi_nss = 9,
        .gpio_busy = 10,
    };
    llc68_init(&driver, &config);

    while (true) {
        uint8_t buffer[] = "HELLO";
        llc68_send(&driver, buffer, sizeof buffer);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ws2812_thread(void *arg)
{
    static DMA_ATTR uint8_t my_ws2812_buffer[WS2812_DMA_SIZE(10)];

    ws2812_driver driver;
    ws2812_config config = {.spi_host = SPI2_HOST, .ws2812_pin = 8};
    ws2812_init(&driver, &config);

    uint8_t i = 0;
    while (1)
    {
        ws2812_color colors[3] = {
            {.red = 10, .green = 0, .blue = 0},
            {.red = 0, .green = 10, .blue = 0},
            {.red = 0, .green = 0, .blue = 10}};
        ws2812_encode(my_ws2812_buffer, &colors[i % 3], 1);

        ws2812_transmit(&driver, my_ws2812_buffer, 1);

        printf("Meow <3\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        i++;
    }
}

void app_main()
{
    TaskHandle_t tx_thread_h;
    xTaskCreate(llcc68_tx_thread, "llcc68_tx", 8000, NULL, tskIDLE_PRIORITY, &tx_thread_h);
    // xTaskCreate(ws2812_thread, "ws2812", 8000, NULL, 16, ws2812_thread);
}