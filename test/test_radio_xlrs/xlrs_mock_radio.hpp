#pragma once

#include "XLRS.hpp"
#include "blocking_queue.hpp"
#include <vector>
#include <stdint.h>
#include <stddef.h>

class MockRadioDelegate : public radio::xlrs::RadioDelegate
{
public:
    MockRadioDelegate(bool side) : isSideA(side) {}

    void sendPacket(const uint8_t buf[], size_t length) override
    {
        std::vector<uint8_t> data(buf, buf + length);
        if (isSideA)
            queueSideB.push(std::move(data));
        else
            queueSideA.push(std::move(data));
    }

    size_t receivePacket(uint8_t buf[], size_t max_length, uint32_t timeout) override
    {
        BlockingQueue<std::vector<uint8_t>>& receiveQueue = isSideA ? queueSideA : queueSideB;
        std::vector<uint8_t> receivedData = receiveQueue.pop();
        size_t copyLength = std::min(max_length, receivedData.size());
        std::copy(receivedData.begin(), receivedData.begin() + copyLength, buf);
        return copyLength;
    }

private:
    bool isSideA;
    static BlockingQueue<std::vector<uint8_t>> queueSideA;
    static BlockingQueue<std::vector<uint8_t>> queueSideB;
};
