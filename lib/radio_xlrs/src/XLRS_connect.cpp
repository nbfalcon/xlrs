#include "XLRS.hpp"

#include <string.h>
#include <mbedtls/aes.h>
#include <mbedtls/constant_time.h>

#ifdef XLRS_TARGET_ESP32
#include <esp_timer.h>
#include <esp_random.h>
#elif XLRS_TARGET_MOCK
#include <time.h>
static inline uint64_t esp_timer_get_time()
{
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    long long elapsed_us = (time.tv_sec) * (1000LL * 1000LL) +
                           (time.tv_nsec) / 1000LL;
    return elapsed_us;
}

static inline void esp_fill_random(void *out, size_t length)
{
    // Chosen with a fair dice roll
    memset(out, 4, length);
}

// For *some* reason, only this symbol is unavailabe in our hacked-together native platformio build system
inline int mbedtls_ct_memcmp(const void *a, const void *b, size_t length)
{
    return memcmp(a, b, length);
}
#endif

#define ECK_MBEDTLS(x) assert((x) == 0)

namespace radio::xlrs
{
    // Sent from TX -> RX to request a connection
    struct SConnectionHello1
    {
        AES128Key nonce1_sk;

        uint8_t tag[8] = "H1 XLRS";
        uint64_t reserved1 = 0;
    };

    // Sent from RX -> TX as an ACK to SConnectionHello1 (proves to TX that the RX knows the pairingKey)
    struct SConnectionResponse
    {
        AES128Key nonce1_sk;

        uint8_t tag[8] = "R1 XLRS";
        uint64_t reserved1 = 0;

        AES128Key nonce2;
    };

    // Sent from TX -> RX as an ACK to SConnectionResponse (proves to the RX that the TX knows the pairingKey)
    struct SConnectionHello2
    {
        AES128Key nonce2;
    };

    // Sent from RX -> TX to tell it that Hello2 was received (so that it stops retransmitting)
    struct SConnectionACK2
    {
        AES128Key nonce1_xor_2;
    };

    static uint32_t get_millis()
    {
        return (uint32_t)(esp_timer_get_time() / 1000);
    }

    bool XLRSConnection::requestConnect()
    {
        connectionState = CONNECTING;
        if (signals)
        {
            signals->connectionStateChanged(*this);
        }

        AES128Key nonce1_sk;
        esp_fill_random(&nonce1_sk, sizeof(nonce1_sk));

        mbedtls_aes_context ctx_enc;
        mbedtls_aes_init(&ctx_enc);
        mbedtls_aes_setkey_enc(&ctx_enc, pairingKey, 128);
        mbedtls_aes_context ctx_dec;
        mbedtls_aes_init(&ctx_dec);
        mbedtls_aes_setkey_dec(&ctx_dec, pairingKey, 128);

        // Phase 1: sent connection hello1
        SConnectionHello1 hello1{};
        memcpy(hello1.nonce1_sk, nonce1_sk, sizeof(nonce1_sk));
        unsigned char hello1_iv[16] = {0}; // This is fine, since the first block is wlays the nonce and as such random
        unsigned char hello1_enc[sizeof(SConnectionHello1)];
        mbedtls_aes_crypt_cbc(&ctx_enc, MBEDTLS_AES_ENCRYPT, sizeof(hello1), hello1_iv, (const uint8_t *)&hello1, hello1_enc);
        radio->sendPacket(hello1_enc, sizeof(hello1_enc));

        // Phase 1/2: receive response
        SConnectionResponse response{};
        unsigned char response_enc[sizeof(SConnectionResponse)];
        if (radio->receivePacket(response_enc, sizeof(response_enc), 100) != sizeof(SConnectionResponse))
        {
            return false; // FIXME: loop
        }
        unsigned char response_iv[16] = {0};
        mbedtls_aes_crypt_cbc(&ctx_dec, MBEDTLS_AES_DECRYPT, sizeof(response), response_iv, response_enc, (uint8_t *)&response);
        if (mbedtls_ct_memcmp(response.nonce1_sk, nonce1_sk, sizeof(nonce1_sk)) != 0)
        {
            return false;
        }

        // Phase 2: send hello2
        SConnectionHello2 hello2{};
        memcpy(hello2.nonce2, response.nonce2, sizeof(response.nonce2));
        unsigned char hello2_enc[sizeof(SConnectionHello2)];
        static_assert(sizeof(hello2) == 16);
        mbedtls_aes_crypt_ecb(&ctx_enc, MBEDTLS_AES_ENCRYPT, (const unsigned char *)&hello2, hello2_enc);
        radio->sendPacket(hello2_enc, sizeof(hello2_enc));

        // Phase 2/1: receive hello2 ack
        AES128Key nonce1_xor_2;
        for (int i = 0; i < sizeof(AES128Key); i++)
        {
            nonce1_xor_2[i] = nonce1_sk[i] ^ response.nonce2[i];
        }
        unsigned char ack2_enc[sizeof(SConnectionACK2)];
        if (radio->receivePacket(ack2_enc, sizeof(ack2_enc), 100) != sizeof(ack2_enc))
        {
            return false;
        }
        SConnectionACK2 ack2;
        mbedtls_aes_crypt_ecb(&ctx_dec, MBEDTLS_AES_DECRYPT, (const unsigned char *)ack2_enc, (unsigned char *)&ack2);
        if (mbedtls_ct_memcmp(ack2.nonce1_xor_2, nonce1_xor_2, sizeof(nonce1_xor_2)) != 0)
        {
            return false;
        }
        // And were done. We have an ACK, which means both the receiver and transmitter consider eachother connected.

        connectionState = CONNECTED;
        if (signals)
        {
            signals->connectionStateChanged(*this);
        }
        memcpy(sessionKey, nonce1_sk, sizeof(nonce1_sk));
        return true;
    }

    bool XLRSConnection::respondConnect()
    {
        connectionState = CONNECTING;
        if (signals)
        {
            signals->connectionStateChanged(*this);
        }

        AES128Key nonce2;
        esp_fill_random(&nonce2, sizeof(nonce2));

        mbedtls_aes_context ctx_enc;
        mbedtls_aes_init(&ctx_enc);
        mbedtls_aes_setkey_enc(&ctx_enc, pairingKey, 128);
        mbedtls_aes_context ctx_dec;
        mbedtls_aes_init(&ctx_dec);
        mbedtls_aes_setkey_dec(&ctx_dec, pairingKey, 128);

        // Phase 1: receive hello1
        unsigned char hello1_enc[sizeof(SConnectionHello1)];
        if (radio->receivePacket(hello1_enc, sizeof(hello1_enc), 100) != sizeof(hello1_enc))
        {
            return false;
        }
        SConnectionHello1 hello1{};
        unsigned char hello1_iv[16] = {0};
        mbedtls_aes_crypt_cbc(&ctx_dec, MBEDTLS_AES_DECRYPT, sizeof(hello1), hello1_iv, hello1_enc, (unsigned char *)&hello1);
        if (mbedtls_ct_memcmp(hello1.tag, "H1 XLRS", 8) != 0)
        {
            return false;
        }

        // Phase 1/2: send response
        SConnectionResponse response1{};
        memcpy(response1.nonce1_sk, hello1.nonce1_sk, sizeof(response1.nonce1_sk));
        memcpy(response1.nonce2, nonce2, sizeof(response1.nonce2));
        unsigned char response1_enc[sizeof(SConnectionResponse)];
        unsigned char response1_iv[16] = {0};
        mbedtls_aes_crypt_cbc(&ctx_enc, MBEDTLS_AES_ENCRYPT, sizeof(response1), response1_iv, (const unsigned char *)&response1, response1_enc);
        radio->sendPacket(response1_enc, sizeof(response1_enc));

        // Phase 2: receive Hello2
        SConnectionHello2 hello2{};
        unsigned char hello2_enc[sizeof(SConnectionHello2)];
        if (radio->receivePacket(hello2_enc, sizeof(hello2_enc), 100) != sizeof(hello2_enc))
        {
            return false;
        }
        static_assert(sizeof(hello2) == 16);
        mbedtls_aes_crypt_ecb(&ctx_dec, MBEDTLS_AES_DECRYPT, hello2_enc, (unsigned char *)&hello2);
        if (mbedtls_ct_memcmp(hello2.nonce2, nonce2, sizeof(nonce2)) != 0)
        {
            return false;
        }

        // Phase 2/1: send ACK2
        AES128Key nonce1_xor_2;
        for (int i = 0; i < sizeof(nonce1_xor_2); i++)
        {
            nonce1_xor_2[i] = hello1.nonce1_sk[i] ^ nonce2[i];
        }
        SConnectionACK2 ack2{};
        memcpy(ack2.nonce1_xor_2, nonce1_xor_2, sizeof(nonce1_xor_2));
        static_assert(sizeof(ack2) == 16);
        unsigned char ack2_enc[sizeof(SConnectionACK2)];
        mbedtls_aes_crypt_ecb(&ctx_enc, MBEDTLS_AES_ENCRYPT, (const unsigned char *)&ack2, ack2_enc);
        radio->sendPacket(ack2_enc, sizeof(ack2_enc));

        connectionState = CONNECTED;
        if (signals)
        {
            signals->connectionStateChanged(*this);
        }
        memcpy(sessionKey, hello1.nonce1_sk, sizeof(hello1.nonce1_sk));
        return true;
    }
}