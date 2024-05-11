#pragma once

#include "radiohal.hpp"
#include <stdint.h>

namespace radiohal::llcc68
{
    class LLCC68Radio : public Radio
    {
        IODelegate *io;
        LoraConfig configLora{};

        Result halConfigureLora(const LoraConfig &config, length_type size);
        Result waitForInterrupt(uint16_t irq_mask, timeout timeout_ms);

    public:
        LLCC68Radio(IODelegate *io)
            : io(io) {}

        Result reset_init() override;
        Result configurePhy(const PhyConfig &config) override;

        Result setLora(const LoraConfig &config) override;

        Result send(length_type data_size, uint8_t data[]) override;
        Result recv(length_type *data_size, uint8_t data[], timeout timeout_ms) override;
    };
}