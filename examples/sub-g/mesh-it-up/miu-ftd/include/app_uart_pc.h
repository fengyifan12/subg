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

/**
 * 向 PC 发送 DEV_ONLINE 消息，通知设备上线（online=1）或离线（online=0）。
 * seq 由内部计数器自动递增，调用方无需管理。
 */
void app_uart_pc_send_dev_online(const char *dev_type_str,
                                  const char *dev_name,
                                  const char *dev_id,
                                  int         online);

#endif /* APP_UART_PC_H */
