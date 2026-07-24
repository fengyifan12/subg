#ifndef APP_UART_H
#define APP_UART_H

#include <stdint.h>

typedef enum {
    UART_DATA_VALID = 0,
    UART_DATA_VALID_CRC_OK,
    UART_DATA_INVALID,
    UART_DATA_CS_ERROR,
} uart_handler_data_sts_t;

void app_uart_init(void);
void app_uart_log_init(void);
void app_uart0_enable(void);
void app_uart0_disable(void);
int app_uart0_data_send(const uint8_t *p_data, uint16_t data_len);

/**
 * 从 UART0 环形缓冲中读取最多 p_data_len 字节到 p_data。
 * 返回实际读取字节数，0=无数据，-1=参数错误。
 * FTD PC 桥接模块（app_uart_pc.c）通过此接口读取来自 PC 的数据。
 */
int app_uart0_rx_read(uint8_t *p_data, uint32_t p_data_len);

#endif /* APP_UART_H */
