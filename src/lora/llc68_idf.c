#include "llcc68_hal.h"
#include "llcc68.h"
#include "llc68_idf.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>

static bool wait_for_busy(llc68_module *driver, uint32_t timeout_ms);

static uint8_t TX_NOPS[256] = {};

llcc68_hal_status_t llcc68_hal_write(const void *context, const uint8_t *command, uint16_t command_length, const uint8_t *data, const uint16_t data_length)
{
    llc68_module *driver = (llc68_module *)context;

    llcc68_hal_status_t result = LLCC68_STATUS_OK;

    // DMA writes 4-bytes at a time
    uint32_t dma_rx = 0, dma_tx = 0;

    if (data_length == 0)
    {
        if (command_length == 1)
        {
            ((uint8_t *)dma_tx)[0] = command[0];
            ((uint8_t *)dma_tx)[1] = 0;
            command = (uint8_t *)dma_tx;
            command_length = 2;
        }

        spi_transaction_t tx_cmd = {
            .length = command_length * 8,
            .tx_buffer = command,
            // We get an RFU followed by a STATUS byte
            .rx_buffer = &dma_rx,
            .rxlength = 2 * 8,
        };
        ESP_ERROR_CHECK(spi_device_transmit(driver->spi_handle, &tx_cmd));

        uint8_t status_byte = ((uint8_t *)dma_rx)[1];
        uint8_t command_status = (status_byte >> 1) & 0b111;
        if (command_status == 0x3 || command_status == 0x4 || command_status == 0x5) {
            result = LLCC68_HAL_STATUS_ERROR;
        }
    }
    else
    {
        ESP_ERROR_CHECK(spi_device_acquire_bus(driver->spi_handle, portMAX_DELAY));

        spi_transaction_t tx_cmd = {
            .length = command_length * 8,
            .tx_buffer = command,
            .flags = SPI_TRANS_CS_KEEP_ACTIVE,
        };
        ESP_ERROR_CHECK(spi_device_transmit(driver->spi_handle, &tx_cmd));
        spi_transaction_t tx_addr = {
            .length = data_length * 8,
            .tx_buffer = data,
        };
        ESP_ERROR_CHECK(spi_device_transmit(driver->spi_handle, &tx_addr));

        spi_device_release_bus(driver->spi_handle);
    }
    return LLCC68_STATUS_OK;
}

llcc68_hal_status_t llcc68_hal_read(const void *context, const uint8_t *command, const uint16_t command_length, uint8_t *data, const uint16_t data_length)
{
    llc68_module *driver = (llc68_module *)context;

    ESP_ERROR_CHECK(spi_device_acquire_bus(driver->spi_handle, portMAX_DELAY));

    spi_transaction_t tx_cmd = {
        .length = command_length * 8,
        .tx_buffer = command,
        .flags = SPI_TRANS_CS_KEEP_ACTIVE,
    };
    ESP_ERROR_CHECK(spi_device_transmit(driver->spi_handle, &tx_cmd));

    spi_transaction_t read_bytes = {
        .length = data_length * 8,
        .tx_buffer = TX_NOPS,
        .rxlength = data_length * 8,
        .rx_buffer = data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(driver->spi_handle, &read_bytes));

    spi_device_release_bus(driver->spi_handle);

    return LLCC68_STATUS_OK;
}

// We dont actually need these functions, since they are called only trough direct wrappers
llcc68_hal_status_t llcc68_hal_reset(const void *context)
{
    return LLCC68_STATUS_UNSUPPORTED_FEATURE;
}

llcc68_hal_status_t llcc68_hal_wakeup(const void *context)
{
    return LLCC68_STATUS_UNSUPPORTED_FEATURE;
}

#define ECK(x)                          \
    do                                  \
    {                                   \
        llcc68_status_t result = (x);   \
        if (result != LLCC68_STATUS_OK) \
        {                               \
            printf(#x " failed!\r\n");      \
            return;                     \
        }                               \
    } while (0)

IRAM_ATTR static void dio1_interrupt_handler(void *context)
{
    llc68_module *driver = (llc68_module *)context;

    TaskHandle_t blockee = driver->task_blocked_on_dio;
    if (blockee != NULL)
    {
        BaseType_t highPrioWoken = pdFALSE;
        vTaskNotifyGiveIndexedFromISR(blockee, 0, &highPrioWoken);
    }
}

IRAM_ATTR static void busy_interrupt_handler(void *context)
{
    llc68_module *driver = (llc68_module *)context;

    TaskHandle_t blockee = driver->task_blocked_on_dio;
    if (blockee != NULL)
    {
        BaseType_t highPrioWoken = pdFALSE;
        vTaskNotifyGiveIndexedFromISR(blockee, 1, &highPrioWoken);
    }
}

static bool wait_for_irq(llc68_module *driver, uint32_t timeout_ms)
{
    bool result = ulTaskNotifyTakeIndexed(0, pdFALSE, pdMS_TO_TICKS(timeout_ms)) == 1;
    if (!result)
    {
        printf("ulTaskNotify timeout\r\n");
    }
    else
    {
        printf("TX/RX done\r\n");
    }
    return result;
}

static bool wait_for_busy(llc68_module *driver, uint32_t timeout_ms)
{
    for (int i = 0; i < 10000; i++)
    {
        if (!gpio_get_level(10))
            break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

void llc68_init(llc68_module *driver, const llc68_config *config)
{
    // FIXME: should we change this? Probably in each call to llc68_send; then again, we don't support
    // multithreading anyway
    driver->task_blocked_on_dio = xTaskGetCurrentTaskHandle();

    spi_bus_config_t bus = {
        .miso_io_num = config->spi_miso,
        .mosi_io_num = config->spi_mosi,
        .sclk_io_num = config->spi_sclk,
        .max_transfer_sz = 0, // Use default for DMA (4092Bytes)
    };
    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .spics_io_num = config->spi_nss,
        .clock_speed_hz = 1000000, // 1MHz, it doesn't really matter
        .queue_size = 1,
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(config->spi_host, &bus, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(config->spi_host, &dev, &driver->spi_handle));

    // No one needs efficiency xD
    ECK(llcc68_set_reg_mode(driver, LLCC68_REG_MODE_LDO));

    ECK(llcc68_set_dio_irq_params(driver, LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE,
                                  LLCC68_IRQ_ALL, 0, 0));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));
    ESP_ERROR_CHECK(gpio_set_direction(config->gpio_dio1, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(config->gpio_dio1, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(config->gpio_dio1, &dio1_interrupt_handler, driver));

    ESP_ERROR_CHECK(gpio_set_direction(config->gpio_busy, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(config->gpio_busy, GPIO_INTR_NEGEDGE));

        // See page 94 of https://www.mouser.com/pdfDocs/DS_LLCC68_V10-2.pdf
    // 1. Set STDBY_RC
    ECK(llcc68_set_standby(driver, LLCC68_STANDBY_CFG_RC));
    ECK(llcc68_set_pkt_type(driver, LLCC68_PKT_TYPE_LORA));

    ECK(llcc68_set_rf_freq(driver, 433050000));

    // "+22dbm" configuration. DO NOT USE in JAPAN.
    // These values are from page 71.
    llcc68_pa_cfg_params_t pa_cfg = {
        .pa_duty_cycle = 0x04,
        .hp_max = 0x07,
        .device_sel = 0x00,
        .pa_lut = 0x01,
    };
    ECK(llcc68_set_pa_cfg(driver, &pa_cfg));
    // FIXME: ramp time?! I guess the higher the better, since we get less of a "high frequency" load
    ECK(llcc68_set_tx_params(driver, 22, LLCC68_RAMP_800_US));

    llcc68_mod_params_lora_t mod_params = {
        .sf = LLCC68_LORA_SF5,
        .bw = LLCC68_GFSK_BW_467000,
        .ldro = 0,
        .cr = LLCC68_LORA_CR_4_8,
    };
    llcc68_set_lora_mod_params(driver, &mod_params);

    // 10. Skip; we already did this in _init

    ECK(llcc68_set_lora_sync_word(driver, 0x34));

    vTaskDelay(pdMS_TO_TICKS(2000));
}

void llc68_send(llc68_module *driver, uint8_t buffer[], size_t size)
{
    ECK(llcc68_set_buffer_base_address(driver, 0, 0));
    ECK(llcc68_write_buffer(driver, 0, buffer, size));

    // 9. Define the frame format
    llcc68_pkt_params_lora_t pkt_params = {
        .crc_is_on = 1,
        .header_type = LLCC68_LORA_PKT_EXPLICIT,
        .invert_iq_is_on = false,
        .pld_len_in_bytes = size,
        .preamble_len_in_symb = 16,
    };
    ECK(llcc68_set_lora_pkt_params(driver, &pkt_params));

    // 12. Transmit
    // FIXME: the timeotus are off
    llcc68_set_tx(driver, 500);
    // 14. Clear IRQ status
    // ECK(llcc68_clear_irq_status(driver, LLCC68_IRQ_TX_DONE));
}

void llc68_recv(llc68_module *driver, uint8_t buffer[], size_t size, uint32_t timeout_in_ms)
{
    ECK(llcc68_cfg_rx_boosted(driver, true));
    ECK(llcc68_set_rx(driver, timeout_in_ms));
    ECK(llcc68_read_buffer(driver, 0, buffer, size));
}
