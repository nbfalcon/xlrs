#include "llcc68_hal.h"
#include "llcc68.h"
#include "llc68_idf.h"
#include <driver/gpio.h>
#include <driver/spi_master.h>

llcc68_hal_status_t do_spi_tx(llc68_module *driver,
                              uint16_t command_length, const uint8_t *command, const uint16_t data_length, const uint8_t *data,
                              uint8_t *rx_data_result)
{
    bool have_data = data_length != 0;
    spi_transaction_t cmd_tx = {0}, data_tx = {0};

    ESP_ERROR_CHECK(spi_device_acquire_bus(driver->spi_handle, portMAX_DELAY));
    if (command_length == 1)
    {
        // We still want a status for one-byte commands, so we actually need 2
        cmd_tx.tx_data[0] = command[0];
        cmd_tx.length = 2 * 8;
        cmd_tx.rxlength = 2 * 8; // RFU + STATUS
        cmd_tx.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA | (have_data ? SPI_TRANS_CS_KEEP_ACTIVE : 0);

        ESP_ERROR_CHECK(spi_device_queue_trans(driver->spi_handle, &cmd_tx, portMAX_DELAY));
    }
    else
    {
        cmd_tx.tx_buffer = command;
        cmd_tx.length = command_length * 8;
        cmd_tx.rxlength = 2 * 8; // RFU + STATUS
        cmd_tx.flags = SPI_TRANS_USE_RXDATA | (have_data ? SPI_TRANS_CS_KEEP_ACTIVE : 0);

        ESP_ERROR_CHECK(spi_device_queue_trans(driver->spi_handle, &cmd_tx, portMAX_DELAY));
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

        ESP_ERROR_CHECK(spi_device_queue_trans(driver->spi_handle, &data_tx, portMAX_DELAY));
    }

    spi_transaction_t *cmd_tx_result = NULL;
    ESP_ERROR_CHECK(spi_device_get_trans_result(driver->spi_handle, &cmd_tx_result, portMAX_DELAY));
    if (have_data)
    {
        spi_transaction_t *wait_for_tx2 = NULL;
        ESP_ERROR_CHECK(spi_device_get_trans_result(driver->spi_handle, &wait_for_tx2, portMAX_DELAY));
    }
    spi_device_release_bus(driver->spi_handle);

    uint8_t status_byte = cmd_tx_result->rx_data[1];
    uint8_t cmd_status = (status_byte >> 1) & 0b111;
    if (cmd_status == 0x3 || cmd_status == 0x4 || cmd_status == 0x5)
    {
        return LLCC68_HAL_STATUS_ERROR;
    }
    else
    {
        return LLCC68_HAL_STATUS_OK;
    }
}

llcc68_hal_status_t llcc68_hal_write(const void *context, const uint8_t *command, uint16_t command_length, const uint8_t *data, const uint16_t data_length)
{
    llc68_module *driver = (llc68_module *)context;

    return do_spi_tx(driver, command_length, command, data_length, data, NULL);
}

llcc68_hal_status_t llcc68_hal_read(const void *context, const uint8_t *command, const uint16_t command_length, uint8_t *data, const uint16_t data_length)
{
    llc68_module *driver = (llc68_module *)context;

    return do_spi_tx(driver, command_length, command, data_length, NULL, data);
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
            printf(#x " failed!\r\n");  \
            return;                     \
        }                               \
    } while (0)
#define ECK2(x)                         \
    do                                  \
    {                                   \
        llcc68_status_t result = (x);   \
        if (result != LLCC68_STATUS_OK) \
        {                               \
            printf(#x " failed!\r\n");  \
            return false;               \
        }                               \
    } while (0)

static void IRAM_ATTR dio1_interrupt_handler(void *arg)
{
    llc68_module *driver = (llc68_module *)arg;

    // Assume the pin may glitch around during RESET, so we discard those IRQs
    if (driver->blocked_task != NULL)
    {
        xTaskNotifyGiveIndexed(driver->blocked_task, 0);
    }
}

static void init_wait_for_interrupt(llc68_module *driver)
{
    driver->blocked_task = xTaskGetCurrentTaskHandle();
}

static bool wait_for_interrupt(llc68_module *driver, uint16_t irq_mask, uint16_t timeout_ms)
{
    // We assume a well-behaved device that does not generate spurious interrupts, so the next one that comes must be right
    // We can't have both an RX+TX interrupt
    uint32_t result = ulTaskNotifyTakeIndexed(0, pdTRUE, timeout_ms);
    if (result == 0)
    {
        printf("No IRQ. Something is wrong...");
        // Timeout
        return false;
    }
    llcc68_irq_mask_t irq;
    llcc68_get_and_clear_irq_status(driver, &irq);
    return irq & irq_mask; // Did we get RX_DONE/TX_DONE or just TIMEOUT?
}

void llc68_init_spi_bus(llc68_spi_bus_config *config)
{
    spi_bus_config_t bus = {
        .miso_io_num = config->spi_miso,
        .mosi_io_num = config->spi_mosi,
        .sclk_io_num = config->spi_sclk,
        .max_transfer_sz = 0, // Use default for DMA (4092Bytes)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(config->spi_host, &bus, SPI_DMA_CH_AUTO));
}

void llc68_do_reset_pulse(int gpio_reset)
{
    // Doing this multiple times won't hurt
    ESP_ERROR_CHECK(gpio_set_direction(gpio_reset, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(gpio_reset, 1));
    esp_rom_delay_us(200);
    ESP_ERROR_CHECK(gpio_set_level(gpio_reset, 0));
    esp_rom_delay_us(100);
    ESP_ERROR_CHECK(gpio_set_level(gpio_reset, 1));
}

void llc68_init(llc68_module *driver, const llc68_config *config)
{
    driver->blocked_task = NULL;

    if (config->gpio_reset != -1)
    {
        llc68_do_reset_pulse(config->gpio_reset);
    }

    // The bus is already initialized by this pint
    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .spics_io_num = config->spi_nss,
        .clock_speed_hz = 1000000, // 1MHz, it doesn't really matter
        .queue_size = 2,           // We can have request+response (see do_spi_tx)
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .cs_ena_pretrans = 16,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(config->config->spi_host, &dev, &driver->spi_handle));

    // ESP_ERROR_CHECK(gpio_set_direction(config->gpio_busy, GPIO_MODE_INPUT));
    // ESP_ERROR_CHECK(gpio_set_intr_type(config->gpio_busy, GPIO_INTR_NEGEDGE));

    ESP_ERROR_CHECK(gpio_set_direction(config->gpio_dio1, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_intr_type(config->gpio_dio1, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(config->gpio_dio1, &dio1_interrupt_handler, driver));

    vTaskDelay(pdMS_TO_TICKS(100));

    ECK(llcc68_set_reg_mode(driver, LLCC68_REG_MODE_DCDC));
}

void llc68_send(llc68_module *driver, uint8_t buffer[], size_t size)
{
    // See page 94 of https://www.mouser.com/pdfDocs/DS_LLCC68_V10-2.pdf
    // 1. Set STDBY_RC

    llcc68_chip_status_t status;
    // Dont error check; the whole point is that we get the status
    llcc68_get_status(driver, &status);
    if (status.cmd_status == 0x2)
    {
        WORD_ALIGNED_ATTR uint8_t buffer[256];
        llcc68_read_buffer(driver, 0, buffer, 255);
        printf("%2x%2x%2x%2x; ", buffer[0], buffer[1], buffer[2], buffer[3]);
        static uint8_t zeros[256];
        llcc68_write_buffer(driver, 0, zeros, 255);
    }
    llcc68_irq_mask_t irq;
    llcc68_get_irq_status(driver, &irq);
    printf("%2x:%2x | %x:%2x ", status.chip_mode, status.cmd_status, irq, irq & LLCC68_IRQ_TX_DONE);
    llcc68_clear_irq_status(driver, LLCC68_IRQ_ALL);

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

    ECK(llcc68_set_buffer_base_address(driver, 0, 0));
    ECK(llcc68_write_buffer(driver, 0, buffer, size));

    llcc68_mod_params_lora_t mod_params = {
        .sf = LLCC68_LORA_SF7,
        .bw = LLCC68_LORA_BW_125,
        .ldro = 0,
        .cr = LLCC68_LORA_CR_4_8,
    };
    ECK(llcc68_set_lora_mod_params(driver, &mod_params));

    // 9. Define the frame format
    llcc68_pkt_params_lora_t pkt_params = {
        .crc_is_on = 1,
        .header_type = LLCC68_LORA_PKT_EXPLICIT,
        .invert_iq_is_on = false,
        .pld_len_in_bytes = size,
        .preamble_len_in_symb = 16,
    };
    ECK(llcc68_set_lora_pkt_params(driver, &pkt_params));

    ECK(llcc68_set_dio_irq_params(driver, LLCC68_IRQ_ALL, LLCC68_IRQ_ALL, 0, 0));

    ECK(llcc68_set_lora_sync_word(driver, 0x34));

    init_wait_for_interrupt(driver);

    // 12. Transmit
    ECK(llcc68_set_tx(driver, 1000));

    // 13. Wait for interrupt and 14. Clear interrupt status;
    bool success = wait_for_interrupt(driver, LLCC68_IRQ_TX_DONE, 1000);
    if (success)
    {
        printf("Success!! \r\n");
    }
}

bool llc68_recv(llc68_module *driver, uint8_t buffer[], size_t size, uint32_t timeout_in_ms)
{
    ECK2(llcc68_set_standby(driver, LLCC68_STANDBY_CFG_RC));
    ECK2(llcc68_set_pkt_type(driver, LLCC68_PKT_TYPE_LORA));
    ECK2(llcc68_set_rf_freq(driver, 433000000));
    ECK2(llcc68_set_buffer_base_address(driver, 0, 0));

    llcc68_mod_params_lora_t mod_params = {
        .sf = LLCC68_LORA_SF7,
        .bw = LLCC68_LORA_BW_125,
        .ldro = 0,
        .cr = LLCC68_LORA_CR_4_8,
    };
    ECK2(llcc68_set_lora_mod_params(driver, &mod_params));

    llcc68_pkt_params_lora_t pkt_params = {
        .crc_is_on = 1,
        .header_type = LLCC68_LORA_PKT_EXPLICIT,
        .invert_iq_is_on = false,
        .pld_len_in_bytes = size,
        .preamble_len_in_symb = 16,
    };
    ECK2(llcc68_set_lora_pkt_params(driver, &pkt_params));

    ECK2(llcc68_set_dio_irq_params(driver, LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_RX_DONE, LLCC68_IRQ_TIMEOUT | LLCC68_IRQ_RX_DONE, 0, 0));
    ECK2(llcc68_set_lora_sync_word(driver, 0x34));
    ECK2(llcc68_cfg_rx_boosted(driver, true));
    ECK2(llcc68_set_rx(driver, 1000));

    // FIXME: handle CRCERR (CRC checksum was off); we must not get that
    bool success = wait_for_interrupt(driver, LLCC68_IRQ_RX_DONE, 1000);

    if (success)
    {
        ECK2(llcc68_read_buffer(driver, 0, buffer, size));
    }

    return success;
}
