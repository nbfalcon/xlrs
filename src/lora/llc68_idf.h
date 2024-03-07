#include <driver/spi_master.h>
#include <freertos/task.h>

typedef struct {
    spi_device_handle_t spi_handle;

    TaskHandle_t blocked_task;
    int gpio_busy;
} llc68_module;

typedef struct {
    int spi_host;
    int spi_mosi;
    int spi_miso;
    int spi_sclk;
    int spi_nss;

    int gpio_busy;
    int gpio_dio1;
    int gpio_reset;
} llc68_config;

void llc68_init(llc68_module *driver, const llc68_config *config);
void llc68_send(llc68_module *driver, uint8_t buffer[], size_t size);
void llc68_recv(llc68_module *driver, uint8_t buffer[], size_t size, uint32_t timeout_in_ms);