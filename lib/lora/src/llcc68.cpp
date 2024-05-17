#include "lora/llcc68.hpp"

extern "C"
{
#include "llcc68.h"
#include "llcc68_hal.h"
#include "llcc68_regs.h"
}
#include "lora/llcc68.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LORA_EH(x)                           \
    if ((hal_err = (x)) != LLCC68_STATUS_OK) \
    {                                        \
        LORA_LOG_ERROR(x, hal_err)           \
        return ERROR;                        \
    }
#define ERRS                     \
    [[maybe_unused]] Result err; \
    llcc68_status_t hal_err;

namespace radiohal::llcc68
{
    Result LLCC68Radio::halConfigureLora(const LoraConfig &config, length_type size)
    {
        ERRS;

        llcc68_mod_params_lora_t mod_params = {
            .sf = (llcc68_lora_sf_t)config.sf,
            .bw = (llcc68_lora_bw_t)config.bw,
            .cr = (llcc68_lora_cr_t)config.cr,
            .ldro = 0,
        };
        LORA_EH(llcc68_set_lora_mod_params(io, &mod_params));

        // 9. Define the frame format
        // FIXME: some more configurability
        llcc68_pkt_params_lora_t pkt_params = {
            .preamble_len_in_symb = 8,
            .header_type = config.implicitMode ? LLCC68_LORA_PKT_IMPLICIT : LLCC68_LORA_PKT_EXPLICIT,
            .pld_len_in_bytes = size,
            .crc_is_on = 1,
            .invert_iq_is_on = false,
        };
        LORA_EH(llcc68_set_lora_pkt_params(io, &pkt_params));
        LORA_EH(llcc68_set_lora_sync_word(io, 0x34));

        return OK;
    }

    Result LLCC68Radio::waitForInterrupt(uint16_t irq_mask, timeout timeout_ms)
    {
        // FIXME: clear interrupt at start of recv() and send()
        io->waitForInterrupt(timeout_ms + 100);

        llcc68_irq_mask_t irq;
        llcc68_get_and_clear_irq_status(io, &irq);
        if (irq & LLCC68_IRQ_TIMEOUT)
        {
            return TIMEOUT;
        }
        if (!(irq & irq_mask))
        {
            // Did we get RX_DONE/TX_DONE or just TIMEOUT?
            return ERROR;
        }

        return OK;
    }

    Result LLCC68Radio::reset_init()
    {
        ERRS;

        LORA_EC(io->reset());
        LORA_EH(llcc68_set_reg_mode(io, LLCC68_REG_MODE_LDO));
        const int which_irqs = LLCC68_IRQ_TX_DONE | LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT;
        LORA_EH(llcc68_set_dio_irq_params(io, which_irqs, which_irqs, 0, which_irqs));

        return OK;
    }

    Result LLCC68Radio::configurePhy(const PhyConfig &config)
    {
        ERRS;

        LORA_EH(llcc68_set_pkt_type(io, LLCC68_PKT_TYPE_LORA));
        LORA_EH(llcc68_set_rf_freq(io, config.freqHZ));

        // "+22dbm" configuration. DO NOT USE in JAPAN.
        // These values are from page 71.
        llcc68_pa_cfg_params_t pa_cfg = {
            .pa_duty_cycle = 0x04,
            .hp_max = 0x07,
            .device_sel = 0x00,
            .pa_lut = 0x01,
        };
        LORA_EH(llcc68_set_pa_cfg(io, &pa_cfg));
        // FIXME: ramp time?! I guess the higher the better, since we get less of a "high frequency" load
        LORA_EH(llcc68_set_tx_params(io, 22, LLCC68_RAMP_800_US));

        return OK;
    }

    Result LLCC68Radio::setLora(const LoraConfig &config)
    {
        this->configLora = config;
        return OK;
    }

    Result LLCC68Radio::send(length_type data_size, uint8_t data[])
    {
        ERRS;

        LORA_EC(halConfigureLora(configLora, data_size));

        LORA_EH(llcc68_set_buffer_base_address(io, 0, 0));
        LORA_EH(llcc68_write_buffer(io, 0, data, data_size));

        // FIXME: calculate the actual transmission duration
        io->prepareInterrupt();
        LORA_EH(llcc68_set_tx(io, 1000));
        LORA_EC(waitForInterrupt(LLCC68_IRQ_TX_DONE, 1000));

        return OK;
    }

    Result LLCC68Radio::recv(length_type *data_size, uint8_t data[], timeout timeout_ms)
    {
        ERRS;

        LORA_EH(llcc68_set_standby(io, LLCC68_STANDBY_CFG_RC));

        LORA_EH(llcc68_set_pkt_type(io, LLCC68_PKT_TYPE_LORA));
        LORA_EH(llcc68_set_rf_freq(io, 433050000));
        LORA_EH(llcc68_set_buffer_base_address(io, 0, 0));

        LORA_EC(halConfigureLora(configLora, *data_size));

        LORA_EH(llcc68_cfg_rx_boosted(io, true));
        io->prepareInterrupt();
        LORA_EH(llcc68_set_rx(io, timeout_ms));
        LORA_EC(waitForInterrupt(LLCC68_IRQ_RX_DONE | LLCC68_IRQ_TIMEOUT, timeout_ms));

        llcc68_rx_buffer_status_t rx_status;
        LORA_EH(llcc68_get_rx_buffer_status(io, &rx_status));

        *data_size = rx_status.pld_len_in_bytes;
        LORA_EH(llcc68_read_buffer(io, rx_status.buffer_start_pointer, data, rx_status.pld_len_in_bytes));

        return OK;
    }
}
// HAL impl here
extern "C"
{
#define DRV_TO_HAL(x) return ((x) == radiohal::OK) ? LLCC68_HAL_STATUS_OK : LLCC68_HAL_STATUS_ERROR

    llcc68_hal_status_t llcc68_hal_write(const void *context, const uint8_t *command, uint16_t command_length, const uint8_t *data, const uint16_t data_length)
    {
        radiohal::IODelegate *driver = (radiohal::IODelegate *)context;
        DRV_TO_HAL(driver->transact(command_length, command, data_length, data, 0, NULL) != radiohal::OK);
    }

    llcc68_hal_status_t llcc68_hal_read(const void *context, const uint8_t *command, const uint16_t command_length, uint8_t *data, const uint16_t data_length)
    {
        radiohal::IODelegate *driver = (radiohal::IODelegate *)context;
        DRV_TO_HAL(driver->transact(command_length, command, 0, NULL, data_length, data) != radiohal::OK);
    }

    // We dont actually need these functions, since they are called only trough direct wrappers
    llcc68_hal_status_t llcc68_hal_reset(const void *context)
    {
        radiohal::IODelegate *driver = (radiohal::IODelegate *)context;
        DRV_TO_HAL(driver->reset());
    }

    llcc68_hal_status_t llcc68_hal_wakeup(const void *context)
    {
        return LLCC68_HAL_STATUS_ERROR;
    }
}