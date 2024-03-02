#pragma once

#include <stdint.h>
#include <stddef.h>
#include <driver/spi_master.h>

typedef struct ws2812_color {
    uint8_t red, green, blue;
} ws2812_color;

#define WS2812_DMA_SIZE(n_leds_capacity) ((n_leds_capacity) * 9)

typedef struct ws2812_driver {
    spi_device_handle_t spi_handle;
} ws2812_driver;

typedef struct ws2812_config {
    int ws2812_pin;
    int spi_host;
} ws2812_config;

void ws2812_init(struct ws2812_driver *driver, struct ws2812_config *config);
void ws2812_encode(uint8_t *dma_buffer, struct ws2812_color *color_strip, size_t count);
void ws2812_transmit(struct ws2812_driver *driver, uint8_t *dma_buffer, size_t count);