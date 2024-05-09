#define SEND_MESSAGE_CBC(val_name)                                                                                                                        \
    unsigned char val_name##_enc[sizeof(val_name)];                                                                                                       \
    unsigned char val_name##_iv[16];                                                                                                                      \
    ECK_MBEDTLS(mbedtls_aes_crypt_cbc(&ctx_enc, MBEDTLS_AES_ENCRYPT, sizeof(val_name), val_name##_iv, (const unsigned char *)&val_name, val_name##_enc)); \
    radio->sendPacket(val_name##_enc, sizeof(val_name));
#define RECV_MESSAGE_CBC(ty, val_name)                                  \
    ty val_name;                                                        \
    unsigned char val_name##_enc[sizeof(val_name)];                     \
    if (radio->receivePacket(val_name##_enc, sizeof(ty)) != sizeof(ty)) \
    {                                                                   \
        return false;                                                   \
    }                                                                   \
    unsigned char val_name##_iv[16];                                    \
    ECK_MBEDTLS(mbedtls_aes_crypt_cbc(&ctx_dec, MBEDTLS_AES_DECRYPT, sizeof(ty), val_name##_iv, val_name##_enc, (unsigned char *)&val_name));
