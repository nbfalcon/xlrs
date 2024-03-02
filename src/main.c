#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "ws2812.h"

DMA_ATTR uint8_t my_ws2812_buffer[WS2812_DMA_SIZE(10)];

void app_main()
{
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