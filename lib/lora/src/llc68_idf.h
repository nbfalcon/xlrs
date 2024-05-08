#pragma once

#include <driver/spi_master.h>
#include <freertos/task.h>

// BEGIN: low-level bus customization

typedef struct {
    int spi_host;
    
    int spi_mosi;
    int spi_miso;
    int spi_sclk;
} llc68_spi_bus_config;

void llc68_init_spi_bus(llc68_spi_bus_config *config);
void llc68_do_reset_pulse(int gpio_reset);

// BEGIN: main api

typedef struct {
    llc68_spi_bus_config *config;
    int spi_nss;

    // FIXME: currently unused
    int gpio_busy;
    int gpio_dio1;
    // Can be set to -1 if the user wants to do reset themselves (llc68_reset_pulse)
    int gpio_reset;
} llc68_config;

typedef struct {
    spi_device_handle_t spi_handle;

    _Atomic TaskHandle_t blocked_task;
    int gpio_busy;
} llc68_module;

// ISR service must already be installed
void llc68_init(llc68_module *driver, const llc68_config *config);
void llc68_send(llc68_module *driver, uint8_t buffer[], size_t size);
bool llc68_recv(llc68_module *driver, uint8_t buffer[], size_t size, uint32_t timeout_in_ms);