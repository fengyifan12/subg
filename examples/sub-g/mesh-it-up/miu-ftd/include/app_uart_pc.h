#ifndef APP_UART_PC_H
#define APP_UART_PC_H

/* FTD UART map:
 * UART0: PC JSON bridge through app_uart0_dev.
 * UART1: log_info output and OpenThread CLI through uart1_dev.
 * uartstdio/CONFIG_UART_STDIO_* is not used by the FTD PC bridge.
 */
void app_uart_pc_init(void);
void app_uart_pc_send(const char *json_str);
void app_uart_pc_json_recv(void);

void app_uart_pc_process_line(const char *line);

#endif /* APP_UART_PC_H */
