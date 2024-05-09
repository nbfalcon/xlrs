#include "XLRS.hpp"
#include <assert.h>
#include <mbedtls/aes.h>
#include <mbedtls/cmac.h>
#include <mbedtls/constant_time.h>
#include "XLRS_mbedtls_util.hpp"

namespace radio::xlrs
{
    struct L2Header
    {
        uint32_t mic;
        // Make MAC computation more convenient by placing the nonce second (since then we can easily MAC(nonce, ciphertext))
        // without weird *_update wrangling
        uint32_t nonce;
    };

    void XLRSConnection::sendL2Message(const uint8_t buf[], size_t length)
    {
        assert(length <= 200 && length > 0);

        uint32_t nonce = next_nonce;
        next_nonce += (length + 15) / 16;

        uint8_t msgbuf[256];

        // Encrypt
        MbedtlsAesContext ctx_enc;
        mbedtls_aes_setkey_enc(&ctx_enc, sessionKey, 128);

        size_t offset = 0;
        unsigned char stream_block[16];
        unsigned char nonce_bytes[16] = {0};
        memcpy(nonce_bytes, &nonce, sizeof(nonce));
        uint8_t *ciphertext = msgbuf + sizeof(L2Header);
        mbedtls_aes_crypt_ctr(&ctx_enc, length, &offset, nonce_bytes, stream_block, buf, ciphertext);

        memcpy(ciphertext - sizeof(nonce), &nonce, sizeof(nonce));
        const mbedtls_cipher_info_t *cipher_aes128 = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
        uint8_t cmac_buf[16];
        mbedtls_cipher_cmac(cipher_aes128, sessionKey, 128, ciphertext - sizeof(nonce), length + sizeof(nonce), cmac_buf);

        // Prepend header
        struct L2Header header;
        header.nonce = nonce;
        memcpy(&header.mic, cmac_buf + 12, sizeof(header.mic));
        memcpy(msgbuf, &header, sizeof(header));

        // Send
        radio->sendPacket(msgbuf, length + sizeof(L2Header));
    }

    // FIXME: think trough how to handle failure
    ssize_t XLRSConnection::receiveL2Message(uint8_t buf[], size_t length)
    {
        const size_t rx_length = radio->receivePacket(buf, length, 100);
        if (rx_length < sizeof(L2Header) + 1)
        {
            return -1;
        }
        const size_t ciphertext_length = rx_length - sizeof(L2Header);
        const size_t ciphertext_with_nonce_length = ciphertext_length + sizeof(L2Header::nonce);

        L2Header header;
        memcpy(&header, buf, sizeof(header));
        if (header.nonce < next_nonce)
        {
            return -2;
        }
        next_nonce += (ciphertext_length + 15) / 16;

        uint8_t *ciphertext = buf + sizeof(L2Header);
        const mbedtls_cipher_info_t *cipher_aes128 = mbedtls_cipher_info_from_type(MBEDTLS_CIPHER_AES_128_ECB);
        uint8_t cmac_buf[16];
        mbedtls_cipher_cmac(cipher_aes128, sessionKey, 128,
                            ciphertext - sizeof(L2Header::nonce), ciphertext_with_nonce_length, cmac_buf);

        if (mbedtls_ct_memcmp(cmac_buf + 12, &header.mic, sizeof(header.mic)) != 0)
        {
            return -3;
        }

        MbedtlsAesContext ctx_enc;
        mbedtls_aes_setkey_enc(&ctx_enc, sessionKey, 128);

        size_t offset = 0;
        unsigned char stream_block[16];
        unsigned char nonce_bytes[16] = {0};
        mbedtls_aes_crypt_ctr(&ctx_enc, ciphertext_length, &offset, nonce_bytes, stream_block, ciphertext, buf);

        return ciphertext_length;
    }
}
