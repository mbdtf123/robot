#ifndef COM_UART_H
#define COM_UART_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void com_uart_init(void);
void com_send_str(const char *str);
void com_send_printf(const char *fmt, ...);
int com_read_line(char *buf, int max_len);

#ifdef __cplusplus
}
#endif

#endif