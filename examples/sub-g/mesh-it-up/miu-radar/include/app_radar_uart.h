/*
 * app_radar_uart.h — 雷达传感器 UART 协议驱动
 *
 * 使用独立 UART1（TX/RX 引脚由 Kconfig 配置，默认 TX=28/RX=29），
 * 与 UART0 stdio 完全分离，miu-common 零修改：
 *   TX: hosal_uart_send(&s_radar_uart1, ...)
 *   RX: 中断驱动，ISR 填入私有环形缓冲，OT 任务消费
 *
 * 帧格式（LD2410 兼容）：
 *   Header  (4B): FD FC FB FA
 *   DataLen (2B, LE): Cmd(2) + Payload 的字节数
 *   Cmd     (2B, LE): 见 RADAR_CMD_* / RADAR_ACK_*
 *   Payload (可变)
 *   Tail    (4B): 04 03 02 01
 *
 * ACK 命令字 = 请求命令字 | 0x0100：
 *   0x0007 SET_PARAM  → ACK 0x0107, Payload: status(2B)
 *   0x0008 READ_PARAM → ACK 0x0108, Payload: status(2B) + value(4B LE)
 *
 * 示例 — 读最大距离 (ID=0x0001)：
 *   TX: FD FC FB FA  04 00  08 00  01 00  04 03 02 01
 *   RX: FD FC FB FA  08 00  08 01  00 00  64 00 00 00  04 03 02 01
 *
 * 示例 — 设置最大距离 10m (value=100)：
 *   TX: FD FC FB FA  08 00  07 00  01 00  64 00 00 00  04 03 02 01
 *   RX: FD FC FB FA  04 00  07 01  00 00  04 03 02 01
 */

#ifndef APP_RADAR_UART_H
#define APP_RADAR_UART_H

#include <stdint.h>

/* -----------------------------------------------------------------------
 * Frame constants
 * ----------------------------------------------------------------------- */
#define RADAR_CMD_SET_PARAM     0x0007U  /* 配置传感器参数 (5.2.7) */
#define RADAR_CMD_READ_PARAM    0x0008U  /* 读取传感器参数 (5.2.6) */

#define RADAR_ACK_SET_PARAM     0x0107U  /* SET_PARAM 的 ACK */
#define RADAR_ACK_READ_PARAM    0x0108U  /* READ_PARAM 的 ACK */

#define RADAR_ACK_STATUS_OK     0x0000U
#define RADAR_ACK_STATUS_FAIL   0x0001U

/* -----------------------------------------------------------------------
 * Built-in parameter IDs
 * ----------------------------------------------------------------------- */
#define RADAR_PARAM_MAX_DISTANCE     0x0001U  /* 最大探测距离：单位 0.1m，范围 7~100 */
#define RADAR_PARAM_DISAPPEAR_DELAY  0x0004U  /* 目标消失延迟：单位秒，0~65535 */
#define RADAR_PARAM_POWER_ALARM      0x0005U  /* 电源干扰报警（只读）：0/1/2 */

/* -----------------------------------------------------------------------
 * Callback types (called in OT task context)
 * ----------------------------------------------------------------------- */

/**
 * Called after a SET_PARAM ACK is received (or timed out).
 * @param param_id  Parameter that was set
 * @param status    0 = OK, non-zero = error (0xFFFF = timeout)
 */
typedef void (*radar_set_ack_cb_t)(uint16_t param_id, uint16_t status);

/**
 * Called after a READ_PARAM ACK is received (or timed out).
 * @param param_id  Parameter that was read
 * @param status    0 = OK, non-zero = error (0xFFFF = timeout)
 * @param value     Parameter value (valid only when status == OK)
 */
typedef void (*radar_get_ack_cb_t)(uint16_t param_id, uint16_t status, uint32_t value);

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

/**
 * Initialize the radar UART driver.
 * Must be called once before any other radar_uart function.
 * Registers callbacks and resets the RX state machine.
 */
void app_radar_uart_init(radar_set_ack_cb_t set_cb, radar_get_ack_cb_t get_cb);

/**
 * Send a SET_PARAM command (0x0007) to the radar sensor via UART1.
 * The registered set_cb will be called when the ACK arrives or times out.
 */
void app_radar_uart_set_param(uint16_t param_id, uint32_t value);

/**
 * Send a READ_PARAM command (0x0008) to the radar sensor via UART1.
 * The registered get_cb will be called when the ACK arrives or times out.
 */
void app_radar_uart_get_param(uint16_t param_id);


#endif /* APP_RADAR_UART_H */
