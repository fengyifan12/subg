/*
 * Leader UART0 <-> PC JSON bridge.
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
#include "miu_port.h"
#include <openthread/ip6.h>

#define PC_LINE_BUF_SIZE 512

static char s_line_buf[PC_LINE_BUF_SIZE];
static int  s_line_len = 0;

static void on_pc_json_line(const char *line)
{
    char msg_type[16] = {0};
    char dev_name[MIU_DEV_NAME_MAX] = {0};
    otIp6Address dst_ip;

    if (miu_json_get_str(line, "type", msg_type, sizeof(msg_type)) < 0) {
        log_info("[uart_pc] rx: missing 'type'");
        return;
    }

    /* ---- QUERY_TABLE：PC 请求重播设备表 ---- */
    if (strcmp(msg_type, "QUERY_TABLE") == 0) {
        log_info("[uart_pc] rx: QUERY_TABLE, replaying device table to PC");
        app_device_table_iter_register_json(app_uart_pc_send);
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

    uint16_t len = (uint16_t) strlen(line);
    uint8_t *buf = pvPortMalloc(len);
    if (!buf) {
        log_info("[uart_pc] rx: alloc fail");
        return;
    }
    memcpy(buf, line, len);

    if (app_udpSend(dst_ip, buf, len, false) != 0) {
        char ip_str[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_ip, ip_str, sizeof(ip_str));
        log_info("[uart_pc] rx: UDP send fail -> %s (%s)", dev_name, ip_str);
    } else {
        char ip_str[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_ip, ip_str, sizeof(ip_str));
        log_info("[uart_pc] rx: CONTROL -> %s (%s)", dev_name, ip_str);
    }
    vPortFree(buf);
}

static uint32_t s_leader_seq = 0;

void app_uart_pc_init(void)
{
    s_line_len   = 0;
    s_leader_seq = 0;
    memset(s_line_buf, 0, sizeof(s_line_buf));
    log_info("[uart_pc] PC bridge ready on UART0 (115200)");
}

void app_uart_pc_process_line(const char *line)
{
    if (line && line[0] != '\0') {
        on_pc_json_line(line);
    }
}

void app_uart_pc_send(const char *json_str)
{
    if (!json_str) return;
    uint16_t len = (uint16_t)strlen(json_str);
    app_uart0_data_send((const uint8_t *) json_str, len);
    app_uart0_data_send((const uint8_t *) "\n", 1);
}

void app_uart_pc_send_dev_online(const char *dev_type_str,
                                  const char *dev_name,
                                  const char *dev_id,
                                  int         online)
{
    if (!dev_type_str || !dev_name || !dev_id) return;

    char buf[192];
    snprintf(buf, sizeof(buf),
             "{\"ver\":1,\"type\":\"DEV_ONLINE\","
             "\"dev_type\":\"%s\","
             "\"dev_name\":\"%s\","
             "\"dev_id\":\"%s\","
             "\"seq\":%u,"
             "\"data\":{\"online\":%d}}",
             dev_type_str, dev_name, dev_id,
             (unsigned)++s_leader_seq,
             online ? 1 : 0);

    log_info("[uart_pc] DEV_ONLINE %s -> %s (online=%d)", dev_name, dev_type_str, online);
    app_uart_pc_send(buf);
}

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
            s_line_buf[s_line_len++] = (char) byte;
        } else {
            log_info("[uart_pc] rx: line too long, discard");
            s_line_len = 0;
        }
    }
}
