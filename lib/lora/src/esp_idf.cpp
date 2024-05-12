#include "lora/esp_idf.hpp"
extern "C"
{
#include "llcc68_hal.h"
#include "llcc68.h"
}

#define LORA_IC(x)                 \
    if ((hal_err = (x)) != ESP_OK) \
    {                              \
        LORA_LOG_ERROR(x, hal_err);     \
        return ERROR;              \
    }
#define ESPERR(x)                  \
    if ((hal_err = (x)) != ESP_OK) \
    {                              \
        LORA_LOG_ERROR(x, hal_err);     \
        return hal_err;            \
    }
#define ERRS                     \
    [[maybe_unused]] Result err; \
    esp_err_t hal_err;

using namespace radiohal;
using namespace radiohal::esp_idf;

static Result do_spi_tx(spi_device_handle_t spi_handle,
                        uint16_t command_length, const uint8_t command[],
                        uint16_t data_length, const uint8_t data[],
                        uint8_t *rx_data_result)
{
    ERRS;

    bool have_data = data_length != 0;
    spi_transaction_t cmd_tx{}, data_tx{};

    LORA_IC(spi_device_acquire_bus(spi_handle, portMAX_DELAY));
    if (command_length == 1)
    {
        // We still want a status for one-byte commands, so we actually need 2
        cmd_tx.tx_data[0] = command[0];
        cmd_tx.length = 2 * 8;
        cmd_tx.rxlength = 2 * 8; // RFU + STATUS
        cmd_tx.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA | (have_data ? SPI_TRANS_CS_KEEP_ACTIVE : 0);

        LORA_IC(spi_device_queue_trans(spi_handle, &cmd_tx, portMAX_DELAY));
    }
    else
    {
        cmd_tx.tx_buffer = command;
        cmd_tx.length = command_length * 8;
        cmd_tx.rxlength = 2 * 8; // RFU + STATUS
        cmd_tx.flags = SPI_TRANS_USE_RXDATA | (have_data ? SPI_TRANS_CS_KEEP_ACTIVE : 0);

        LORA_IC(spi_device_queue_trans(spi_handle, &cmd_tx, portMAX_DELAY));
    }

    if (have_data)
    {
        data_tx.tx_buffer = data;
        data_tx.length = data_length * 8;
        if (rx_data_result != NULL)
        {
            data_tx.rx_buffer = rx_data_result;
            data_tx.rxlength = data_length * 8;
        }

        LORA_IC(spi_device_queue_trans(spi_handle, &data_tx, portMAX_DELAY));
    }

    spi_transaction_t *cmd_tx_result = NULL;
    LORA_IC(spi_device_get_trans_result(spi_handle, &cmd_tx_result, portMAX_DELAY));
    if (have_data)
    {
        spi_transaction_t *wait_for_tx2 = NULL;
        LORA_IC(spi_device_get_trans_result(spi_handle, &wait_for_tx2, portMAX_DELAY));
    }
    spi_device_release_bus(spi_handle);

    uint8_t status_byte = cmd_tx_result->rx_data[1];
    uint8_t cmd_status = (status_byte >> 1) & 0b111;
    if (cmd_status == 0x3)
    {
        // "0x3" is a timeout, but we need it to be "OK", so that a single failed recv does not break everything.
        // "Timeout" is a sticky status, and does not change until the next non-timeout RQ.
        return OK;
    }
    else if (cmd_status == 0x4 || cmd_status == 0x5)
    {
        return ERROR;
    }
    else
    {
        return OK;
    }
}

namespace radiohal::esp_idf
{
    esp_err_t init_spi_bus(const SpiConfig &config)
    {
        ERRS;

        spi_bus_config_t bus{};
        bus.mosi_io_num = config.spi_mosi;
        bus.miso_io_num = config.spi_miso;
        bus.sclk_io_num = config.spi_sclk;
        bus.max_transfer_sz = 0; // Use default for DMA (4092Bytes)

        ESPERR(spi_bus_initialize(config.spi_host, &bus, SPI_DMA_CH_AUTO));
        return ESP_OK;
    }

    esp_err_t do_reset_pulse(gpio_num_t gpio_reset)
    {
        ERRS;

        // Doing this multiple times won't hurt
        ESPERR(gpio_set_direction(gpio_reset, GPIO_MODE_OUTPUT));
        ESPERR(gpio_set_level(gpio_reset, 1));
        esp_rom_delay_us(200);
        ESPERR(gpio_set_level(gpio_reset, 0));
        esp_rom_delay_us(100);
        ESPERR(gpio_set_level(gpio_reset, 1));
        vTaskDelay(pdMS_TO_TICKS(2000));
        return ESP_OK;
    }

    void IRAM_ATTR SPIIoDelegate::dio1_interrupt_handler(void *arg)
    {
        SPIIoDelegate *driver = (SPIIoDelegate *)arg;

        // Assume the pin may glitch around during RESET, so we discard those IRQs
        if (driver->blocked_task != NULL)
        {
            xTaskNotifyGiveIndexed(driver->blocked_task, 0);
        }
    }

    SPIIoDelegate::SPIIoDelegate(const SPIIOConfig &config)
    {
        // The bus is already initialized by this pint
        spi_device_interface_config_t dev{};
        dev.command_bits = 0;
        dev.address_bits = 0;
        dev.dummy_bits = 0;
        dev.mode = 0;
        dev.clock_source = SPI_CLK_SRC_DEFAULT;
        dev.cs_ena_pretrans = 16;
        dev.clock_speed_hz = 1000000; // 1MHz, it doesn't really matter
        dev.spics_io_num = config.spi_nss;
        dev.queue_size = 2; // We can have request+response (see do_spi_tx)

        // NOTE: I don't think handling "resource allocation failure" like this is too hacky.
        ESP_ERROR_CHECK(spi_bus_add_device(config.spi_host, &dev, &spi_handle));

        ESP_ERROR_CHECK(gpio_set_direction(config.gpio_dio1, GPIO_MODE_INPUT));
        ESP_ERROR_CHECK(gpio_set_intr_type(config.gpio_dio1, GPIO_INTR_POSEDGE));
        ESP_ERROR_CHECK(gpio_isr_handler_add(config.gpio_dio1, &dio1_interrupt_handler, this));
    }

    Result SPIIoDelegate::reset()
    {
        if (gpio_reset != -1)
        {
            do_reset_pulse(gpio_reset);
        }

        return OK;
    }

    Result SPIIoDelegate::transact(uint16_t command_size, const uint8_t command[], uint16_t data_size, const uint8_t data[], uint16_t rx_size, uint8_t rx[])
    {
        return do_spi_tx(spi_handle, command_size, command, rx ? rx_size : data_size, data, rx);
    }

    void SPIIoDelegate::prepareInterrupt()
    {
        blocked_task = xTaskGetCurrentTaskHandle();
    }

    bool SPIIoDelegate::waitForInterrupt(timeout timeout_ms)
    {
        // We assume a well-behaved device that does not generate spurious interrupts, so the next one that comes must be right
        // We can't have both an RX+TX interrupt
        uint32_t result = ulTaskNotifyTakeIndexed(0, pdTRUE, timeout_ms);
        if (result == 0)
        {
            printf("No IRQ. Something is wrong...");
            return false;
        }
        return true;
    }
}