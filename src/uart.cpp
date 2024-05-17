#include "uart.hpp"
#include <driver/uart.h>

[[noreturn]] void uart_thread() {
    const uart_port_t uart = UART_NUM_1;
    const uart_config_t config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart, &config));

    while (true) {
        char buf[256];
        int length = uart_read_bytes(uart, buf, sizeof(buf), portMAX_DELAY);
        if (length < 0) {
            printf("ERROR: uart_read_bytes()\r\n");
        }

        uart_write_bytes(uart, buf, length);
    }
}