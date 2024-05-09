#pragma once

#include <mbedtls/aes.h>
#include <mbedtls/cmac.h>

struct MbedtlsAesContext : public mbedtls_aes_context
{
    MbedtlsAesContext()
    {
        mbedtls_aes_init(this);
    }

    ~MbedtlsAesContext()
    {
        mbedtls_aes_free(this);
    }
};

struct MbedtlsCipherContext : public mbedtls_cipher_context_t
{
    MbedtlsCipherContext()
    {
        mbedtls_cipher_init(this);
    }

    ~MbedtlsCipherContext()
    {
        mbedtls_cipher_free(this);
    }
};
