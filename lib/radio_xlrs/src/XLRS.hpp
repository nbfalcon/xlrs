#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

namespace radio::xlrs
{
    class RadioDelegate
    {
    public:
        // FIXME: report failure?
        virtual void sendPacket(const uint8_t buf[], size_t length) = 0;
        virtual size_t receivePacket(uint8_t buf[], size_t max_length, uint32_t timeout) = 0;
    };

    typedef uint8_t AES128Key[16];

    enum State
    {
        PAIRING,
        CONNECTING,
        CONNECTED
    };

    class XLRSConnection;
    class XLRSSignalsDelegate
    {
    public:
        virtual void connectionStateChanged(XLRSConnection &conn) = 0;
    };

    class XLRSConnection
    {
        AES128Key pairingKey;
        enum State connectionState;

        AES128Key sessionKey;
        uint32_t next_nonce = 0;

        RadioDelegate *radio = nullptr;
        XLRSSignalsDelegate *signals = nullptr;
        
    public:
        XLRSConnection(RadioDelegate *radio)
            : radio(radio) {}

        void init(const AES128Key pairingKey)
        {
            next_nonce = 0;
            memcpy(this->pairingKey, pairingKey, sizeof(this->pairingKey));
        }

#ifdef XLRS_TARGET_MOCK
        void setSK(const AES128Key sessionKey)
        {
            memcpy(this->sessionKey, sessionKey, sizeof(this->sessionKey));
        }
#endif

        // Request to connect to a receiver; it must be running the "respondConnect" protocol
        bool requestConnect();
        bool respondConnect();

        void sendL2Message(const uint8_t buf[], size_t length);
        ssize_t receiveL2Message(uint8_t buf[], size_t length);
    };
}