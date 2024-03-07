#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "ws2812.h"
#include "llcc68.h"
#include "lora/llc68_idf.h"

void init_devs(llc68_module *driver1, llc68_module *driver2)
{
    llc68_spi_bus_config bus = {
        .spi_host = SPI2_HOST,
        .spi_mosi = 4,
        .spi_miso = 5,
        .spi_sclk = 6,
    };
    llc68_config dev1 = {
        .config = &bus,
        .spi_nss = 7,
        .gpio_dio1 = 3,
        .gpio_busy = -1,
        .gpio_reset = -1,
    };
    llc68_config dev2 = {
        .config = &bus,
        .spi_nss = 0,
        .gpio_dio1 = 1,
        .gpio_busy = -1,
        .gpio_reset = -1,
    };
    int gpio_reset = 9;

    llc68_init_spi_bus(&bus);

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    llc68_do_reset_pulse(gpio_reset);

    llc68_init(driver1, &dev1);
    // llc68_init(driver2, &dev2);
}

void llcc68_rx_thread(void *arg)
{
    llc68_module *driver = (llc68_module *)arg;

    uint8_t buffer[256];
    while (true)
    {
        printf("RX ITER\r\n");
        if (llc68_recv(driver, buffer, sizeof buffer, 1000))
        {
            printf("RECV: %s\r\n", buffer);
        }
    }
}

void llcc68_tx_thread(void *arg)
{
    llc68_module *driver = (llc68_module *)arg;

    uint8_t buffer[] = "HELLO";
    while (true)
    {
        printf("TX ITER\r\n");
        llc68_send(driver, buffer, sizeof buffer);
        // Sending a packet takes roughly 30ms
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
    // Important: don't lose them when our thread exists
    static llc68_module driver1 = {0}, driver2 = {0};
    init_devs(&driver1, &driver2);

    TaskHandle_t tx_thread_h, rx_thread_h;
    xTaskCreate(llcc68_tx_thread, "llcc68_tx", 8000, &driver1, tskIDLE_PRIORITY, &tx_thread_h);
    // xTaskCreate(llcc68_rx_thread, "llcc68_rx", 8000, &driver2, tskIDLE_PRIORITY, &rx_thread_h);
    // xTaskCreate(ws2812_thread, "ws2812", 8000, NULL, 16, &tx_thread_h);
}