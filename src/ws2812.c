#include "ws2812.h"
#include <assert.h>
#include <string.h>
#include <driver/spi_master.h>

inline static void encode_u8_ws2812(uint8_t b, uint8_t out[3])
{
    uint32_t r = 0;
    //
    for (int bit = 0; bit < 8; bit++)
    {
        if (b & (1 << bit))
        {
            r |= 0b011 << (bit * 3);
        }
        else
        {
            r |= 0b001 << (bit * 3);
        }
    }

    out[0] = (r >> 0) & 0xFF;
    out[1] = (r >> 8) & 0xFF;
    out[2] = (r >> 16) & 0xFF;
}

void ws2812_init(ws2812_driver *driver, ws2812_config *config)
{
    spi_bus_config_t bus = {
        .miso_io_num = -1,
        .mosi_io_num = config->ws2812_pin,
        .sclk_io_num = -1,
        .max_transfer_sz = 0, // Use default for DMA (4092Bytes)
    };

    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_speed_hz = 2500000,
        .spics_io_num = -1,
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        // RESET
        .dummy_bits = 125,
        .flags = SPI_DEVICE_BIT_LSBFIRST,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(config->spi_host, &bus, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(config->spi_host, &dev, &driver->spi_handle));
}

void ws2812_encode(uint8_t *dma_buffer, ws2812_color *color_strip, size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        ws2812_color color = color_strip[i];

        encode_u8_ws2812(color.red, &dma_buffer[i * 9] + 0);
        encode_u8_ws2812(color.green, &dma_buffer[i * 9] + 3);
        encode_u8_ws2812(color.blue, &dma_buffer[i * 9] + 6);
    }
}

void ws2812_transmit(ws2812_driver *driver, uint8_t *dma_buffer, size_t count)
{
    size_t len = WS2812_DMA_SIZE(count);

    spi_transaction_t tx = {
        .length = len * 8,
        .tx_buffer = dma_buffer,
    };

    ESP_ERROR_CHECK(spi_device_transmit(driver->spi_handle, &tx));
}
