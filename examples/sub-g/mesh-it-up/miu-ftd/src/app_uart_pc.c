/*
 * app_uart_pc.c — Leader UART0 ↔ PC JSON 桥接实现
 */

#include "app_uart_pc.h"
#include "app_device_table.h"
#include "app_udp.h"
#include "app_uart.h"
#include "miu_json.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "hosal_uart.h"
#include "uart_stdio.h"
#include "miu_port.h"
#include <openthread/ip6.h>

/* uartstdio 是 UART0，由 uart_stdio.c 在 BSP 阶段初始化 */
extern hosal_uart_dev_t uartstdio;

/* -----------------------------------------------------------------------
 * 行缓冲（每条 JSON 消息一行，以 '\n' 为分隔符）
 * ----------------------------------------------------------------------- */
#define PC_LINE_BUF_SIZE   512

static char s_line_buf[PC_LINE_BUF_SIZE];
static int  s_line_len = 0;

/* -----------------------------------------------------------------------
 * 内部：处理完整的一行 JSON（来自 PC 的 CONTROL 命令）
 * ----------------------------------------------------------------------- */
static void on_pc_json_line(const char *line)
{
    char     msg_type[16]            = {0};
    char     dev_name[MIU_DEV_NAME_MAX] = {0};
    otIp6Address dst_ip;

    if (miu_json_get_str(line, "type", msg_type, sizeof(msg_type)) < 0) {
        log_info("[uart_pc] rx: missing 'type'");
        return;
    }

    if (strcmp(msg_type, "CONTROL") != 0) {
        log_info("[uart_pc] rx: unsupported type '%s'", msg_type);
        return;
    }

    if (miu_json_get_str(line, "dev_name", dev_name, sizeof(dev_name)) < 0) {
        log_info("[uart_pc] rx: missing 'dev_name'");
        return;
    }

    if (!app_device_table_get_ip(dev_name, &dst_ip)) {
        log_info("[uart_pc] rx: unknown device '%s'", dev_name);
        return;
    }

    /* 原样转发给目标子设备（Leader 不修改 CONTROL 内容） */
    uint16_t len = (uint16_t)strlen(line);
    uint8_t *buf = pvPortMalloc(len);
    if (!buf) {
        log_info("[uart_pc] rx: alloc fail");
        return;
    }
    memcpy(buf, line, len);

    if (app_udpSend(dst_ip, buf, len, false) != 0) {
        char ip_str[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_ip, ip_str, sizeof(ip_str));
        log_info("[uart_pc] rx: UDP send fail → %s (%s)", dev_name, ip_str);
    } else {
        char ip_str[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_ip, ip_str, sizeof(ip_str));
        log_info("[uart_pc] rx: CONTROL → %s (%s)", dev_name, ip_str);
    }
    vPortFree(buf);
}

/* -----------------------------------------------------------------------
 * 公开接口
 * ----------------------------------------------------------------------- */
void app_uart_pc_init(void)
{
    s_line_len = 0;
    memset(s_line_buf, 0, sizeof(s_line_buf));
    log_info("[uart_pc] PC bridge ready on UART0 (115200)");
}

void app_uart_pc_send(const char *json_str)
{
    if (!json_str) return;
    uint16_t len = (uint16_t)strlen(json_str);
    hosal_uart_send(&uartstdio, (const uint8_t *)json_str, len);
    hosal_uart_send(&uartstdio, (const uint8_t *)"\n", 1);
}

/*
 * 由 app_uart.c 的 UART0 事件回调触发（FTD 编译路径）。
 * 每次调用尽量排空环形缓冲，遇 '\n' 则处理当前积累行。
 */
void app_uart_pc_json_recv(void)
{
    uint8_t byte;
    while (app_uart0_rx_read(&byte, 1) == 1) {
        if (byte == '\n' || byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                on_pc_json_line(s_line_buf);
                s_line_len = 0;
            }
        } else if (s_line_len < PC_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)byte;
        } else {
            /* 行过长，丢弃并重置 */
            log_info("[uart_pc] rx: line too long, discard");
            s_line_len = 0;
        }
    }
}
