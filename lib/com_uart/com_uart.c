#include "com_uart.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define COM_UART_NUM    UART_NUM_0
#define BUF_SIZE        1024
#define READ_TIMEOUT_MS 100

void com_uart_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_param_config(COM_UART_NUM, &uart_cfg);
    uart_driver_install(COM_UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_flush(COM_UART_NUM);
}

void com_send_str(const char *str)
{
    if (str == NULL) return;
    uart_write_bytes(COM_UART_NUM, str, strlen(str));
}

void com_send_printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    com_send_str(buf);
}

int com_read_line(char *buf, int max_len)
{
    if (buf == NULL || max_len <= 1) return 0;

    int idx = 0;
    uint8_t ch;

    while (idx < max_len - 1)
    {
        int len = uart_read_bytes(COM_UART_NUM, &ch, 1, pdMS_TO_TICKS(READ_TIMEOUT_MS));
        if (len == 0) break;
        if (ch == '\n' || ch == '\r') break;
        buf[idx++] = ch;
    }

    buf[idx] = '\0';
    return idx;
}