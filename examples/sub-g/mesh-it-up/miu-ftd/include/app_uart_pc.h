/*
 * app_uart_pc.h — Leader UART0 ↔ PC JSON 桥接
 *
 * 数据流：
 *
 *   PC → UART0 RX → 按行缓冲（\n 分隔）→ 解析 CONTROL JSON
 *                                       → 查设备表得到目标 IP
 *                                       → 通过 UDP 转发给目标子设备
 *
 *   子设备 REPORT/REGISTER（UDP）→ app_udp_comm_json_process()
 *                                → app_uart_pc_send()
 *                                → UART0 TX → PC
 *
 * UART0（uartstdio）：连接 PC，波特率 115200，JSON + '\n' 行格式。
 * UART1（uart1_dev） ：保留用于 log_info() 调试输出，不受影响。
 */

#ifndef APP_UART_PC_H
#define APP_UART_PC_H

/**
 * 初始化 PC 桥接模块（清空行缓冲）。
 * 需在 app_uart_init() 之后调用。
 */
void app_uart_pc_init(void);

/**
 * 将一条 JSON 字符串追加 '\n' 后通过 UART0 发送给 PC。
 * 在 OT 任务上下文或 FreeRTOS 任务中均可调用。
 */
void app_uart_pc_send(const char *json_str);

/**
 * 从 UART0 环形缓冲中读取字节，按行解析 JSON CONTROL 命令。
 * 由 app_uart.c 的 UART0 事件处理路径调用（FTD 专属）。
 */
void app_uart_pc_json_recv(void);

/**
 * 直接处理一行完整的 JSON 字符串（不含 '\n'）。
 * 由 cli_uart_ftd.cpp 在 ProcessCommand 中检测到 '{' 开头时调用，
 * 使 CLI 与 PC JSON 桥接可以在 UART0 上共存。
 */
void app_uart_pc_process_line(const char *line);

#endif /* APP_UART_PC_H */
