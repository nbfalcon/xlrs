#pragma once

#include "radiohal.hpp"
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/task.h>

namespace radiohal::esp_idf
{
    struct SpiConfig
    {
        spi_host_device_t spi_host;

        gpio_num_t spi_mosi;
        gpio_num_t spi_miso;
        gpio_num_t spi_sclk;
    };

    struct SPIIOConfig
    {
        spi_host_device_t spi_host;
        gpio_num_t spi_nss;

        // FIXME: currently unused
        gpio_num_t gpio_busy;
        gpio_num_t gpio_dio1;

        // Can be set to -1 if the user wants to do reset themselves (llc68_reset_pulse)
        gpio_num_t gpio_reset;
    };

    esp_err_t init_spi_bus(const SpiConfig &config);
    esp_err_t do_reset_pulse(gpio_num_t gpio_reset);

    class SPIIoDelegate : public radiohal::IODelegate
    {
        spi_device_handle_t spi_handle;
        // We don't have <atomic>, so use the next best thing, volatile. This is fine, since all ESP32 should be able to atomically write words.
        volatile TaskHandle_t blocked_task = NULL;
        gpio_num_t gpio_busy, gpio_reset;

        static void IRAM_ATTR dio1_interrupt_handler(void *arg);

    public:
        SPIIoDelegate(const SPIIOConfig &config);

        Result reset() override;
        Result transact(uint16_t command_size, const uint8_t command[],
                        uint16_t data_size, const uint8_t data[],
                        uint16_t rx_size, uint8_t rx[]) override;
      
        void prepareInterrupt() override;
        bool waitForInterrupt(timeout timeout_ms) override;
    };
}