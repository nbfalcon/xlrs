#pragma once

#include <stdint.h>

#ifdef LORA_RELEASE
#define LORA_LOG_ERROR(x, code)
#else
#include <stdio.h>
#define LORA_LOG_ERROR(x, code) printf("%s:%d: %s failed with code: %d\r\n", __FILE__, __LINE__, #x, code);
#endif
#define LORA_EC(x)         \
    if ((err = (x)) != OK) \
    {                      \
        LORA_LOG_ERROR(x, err)  \
        return err;        \
    }

namespace radiohal
{
    typedef uint8_t length_type;
    typedef int16_t slength_type;
    typedef uint32_t timeout;
    typedef uint32_t frequency;

    typedef enum : uint8_t
    {
        OK,
        ERROR,
        // Your LoRa modem does not support the required operation or parameters
        UNSUPPORTED,
        TIMEOUT,
    } Result;

    struct LoraConfig
    {
        // NOTE: these numbers are for LLCC68. Maybe this is different for other modems.
        enum CodingRate
        {
            CR_4_5 = 0x01,
            CR_4_6 = 0x02,
            CR_4_7 = 0x03,
            CR_4_8 = 0x04,
        } cr = CR_4_5;
        enum SpreadingFactor
        {
            SF5 = 0x05,
            SF6 = 0x06,
            SF7 = 0x07,
            SF8 = 0x08,
            SF9 = 0x09,
            SF10 = 0x0A,
            SF11 = 0x0B,
        } sf = SF5;
        enum Bandwidth
        {
            BW_500 = 6,
            BW_250 = 5,
            BW_125 = 4,
            BW_062 = 3,
            BW_041 = 10,
            BW_031 = 2,
            BW_020 = 9,
            BW_015 = 1,
            BW_010 = 8,
            BW_007 = 0,
        } bw = BW_500;

        uint8_t syncWord = 0x34;

        bool implicitMode = false;
        // Packet length (in explicit mode)
        length_type explictLength = 1;
    };

    struct PhyConfig
    {
        enum RadioType
        {
            LORA
        } type = LORA;
        frequency freqHZ = 433050000;
    };

    class IODelegate
    {
    public:
        virtual Result reset() = 0;
        // Perform an SPI transaction. Send command, followed by data and read rx_size bytes during the data phase.
        // data and rx may be NULL (in which case the first phase is skipped).
        // Return the LoRa request value (the first byte of the data phase is the result).
        virtual Result transact(uint16_t command_size, const uint8_t command[],
                                uint16_t data_size, const uint8_t data[],
                                uint16_t rx_size, uint8_t rx[]) = 0;
        // Cannot fail; prepares the current thread for a later waitForInterrupt (the interrupt will only be scheduled between these two calls)
        virtual void prepareInterrupt() = 0;
        virtual bool waitForInterrupt(timeout timeout_ms) = 0;
    };

    class Radio
    {
    public:
        virtual Result reset_init() = 0;
        virtual Result configurePhy(const PhyConfig &config) = 0;

        virtual Result setLora(const LoraConfig &config) = 0;

        virtual Result send(length_type data_size, uint8_t data[]) = 0;
        virtual Result recv(length_type *data_size, uint8_t data[], timeout timeout_ms) = 0;
    };
}