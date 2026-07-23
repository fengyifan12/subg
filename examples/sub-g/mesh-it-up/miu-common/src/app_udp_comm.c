/*
 * app_udp_comm.c — 私有 UDP 应用层协议分发器
 *
 * 收到 UDP 数据后由 app_udpReceived_task() 调用：
 *   - 头部匹配 0x8888xxxx → 按 header 分发到各功能处理函数
 *   - 头部不匹配          → 当作远程 CLI 命令注入 otCliInputLine()
 *
 * FTD/MTD 差异通过 CONFIG_MIU_DEVICE_TYPE_FTD / CONFIG_MIU_DEVICE_TYPE_MTD
 * 编译开关区分，无需维护两份文件。
 */

#include <string.h>
#include <stdio.h>
#include <miu_port.h>
#include <openthread/cli.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>
#include "FreeRTOS.h"
#include "task.h"
#include "log.h"
#include "app_protocol.h"
#include "app_ctrl.h"
#include "app_udp.h"
#include "miu_json.h"
#if CONFIG_MIU_DEVICE_TYPE_FTD
#include "app_device_table.h"
#include "app_uart_pc.h"
#endif

/* -----------------------------------------------------------------------
 * 内部工具
 * ----------------------------------------------------------------------- */

bool app_udp_comm_header_check(uint8_t *head, uint8_t lens)
{
    if (lens < 4) {
        return false;
    }
    uint32_t header = *(uint32_t *)head;
    return ((header & UDP_APP_HEADER_MASK) == UDP_APP_HEADER_MAGIC);
}

/* -----------------------------------------------------------------------
 * 雷达上报接收处理（FTD Leader 接收 MTD 上报的雷达事件）
 * ----------------------------------------------------------------------- */
#if CONFIG_MIU_DEVICE_TYPE_FTD
void app_udp_comm_radar_report_proc(uint8_t *data, uint16_t lens,
                                     otIp6Address src_addr)
{
    char ip_str[OT_IP6_ADDRESS_STRING_SIZE];

    if (lens < sizeof(radar_report_t)) {
        return;
    }

    radar_report_t *pkt = (radar_report_t *)data;
    otIp6AddressToString(&src_addr, ip_str, sizeof(ip_str));

    if (pkt->status == 1) {
        log_info("[radar] %s seq=%u: person detected", ip_str, pkt->seq);
        /* TODO: 添加有人时的业务处理 */
    } else {
        log_info("[radar] %s seq=%u: no person", ip_str, pkt->seq);
        /* TODO: 添加无人时的业务处理 */
    }
}
#endif /* CONFIG_MIU_DEVICE_TYPE_FTD */

/* -----------------------------------------------------------------------
 * 主分发函数
 * ----------------------------------------------------------------------- */
void app_udp_comm_process(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    uint32_t header = *(uint32_t *)data;

    switch (header) {
    /* ---- 路径追踪（FTD + MTD 均处理） ---- */
    case PATH_REQUEST_HEADER:
        app_udp_comm_path_req_proc(data, lens);
        break;
    case PATH_RESPOND_HEADER:
        app_udp_comm_path_resp_proc(data, lens);
        break;

#if CFG_USE_CENTRAK_CONFIG

    /* ---- 网管：FTD 专属接收 ---- */
#if CONFIG_MIU_DEVICE_TYPE_FTD
    case RADAR_REPORT_HEADER:
        app_udp_comm_radar_report_proc(data, lens, src_addr);
        break;
    case NET_MGM_NODE_ASK_HEADER:
        /* Router 收到 Leader 的轮询请求，回复本机子节点表 */
        app_udp_comm_net_mgm_node_ack_proc(data, lens, src_addr);
        break;
    case NET_MGM_NODE_REPLY_HEADER:
        /* Leader 收到 Router 的子节点表回复 */
        app_udp_comm_net_mgm_node_reply_proc(data, lens, src_addr);
        break;
    case NET_MGM_NODE_CHALLENGE_HEADER:
        /* Leader 收到新节点的入网申请 */
        app_udp_comm_net_mgm_node_challenge_proc(data, lens, src_addr);
        break;
    case CTRL_NODE_ACK_HEADER:
        /* Leader 收到节点的控制命令回执 */
        app_udp_comm_ctrl_ack_proc(data, lens, src_addr);
        break;
#endif /* CONFIG_MIU_DEVICE_TYPE_FTD */

    /* ---- 网管：MTD 专属接收 ---- */
#if CONFIG_MIU_DEVICE_TYPE_MTD
    case NET_MGM_NODE_ACCEPTED_HEADER:
        /* MTD 收到 Leader 的入网许可 */
        app_udp_comm_net_mgm_node_accepted_proc(data, lens, src_addr);
        break;
#endif /* CONFIG_MIU_DEVICE_TYPE_MTD */

    /* ---- 网管：FTD + MTD 均处理 ---- */
    case NET_MGM_NODE_RESET_HEADER:
        app_udp_comm_net_mgm_node_reset_proc(data, lens, src_addr);
        break;

    /* ---- 控制命令：所有节点均作为接收方 ---- */
    case CTRL_NODE_DETACH_HEADER:
        app_udp_comm_ctrl_detach_proc(data, lens, src_addr);
        break;
    case CTRL_NODE_WHITELIST_HEADER:
        app_udp_comm_ctrl_whitelist_proc(data, lens, src_addr);
        break;
    case CTRL_NODE_CHILD_TIMEOUT_HEADER:
        app_udp_comm_ctrl_child_timeout_proc(data, lens, src_addr);
        break;
    case CTRL_NODE_REATTACH_HEADER:
        app_udp_comm_ctrl_reattach_proc(data, lens, src_addr);
        break;

#endif /* CFG_USE_CENTRAK_CONFIG */

    default:
        break;
    }
}

/* -----------------------------------------------------------------------
 * JSON 私有协议处理
 * ----------------------------------------------------------------------- */

#if CONFIG_MIU_DEVICE_TYPE_FTD
/*
 * Leader 侧：接收子设备发来的 REGISTER / REPORT / ACK。
 *
 * REGISTER → 写入设备表 + 回 ACK + 转发到 PC
 * REPORT   → 更新设备表状态缓存 + 转发到 PC
 * ACK      → 仅打印日志（Leader 发出的 CONTROL 的回执）
 */
void app_udp_comm_json_process(uint8_t *data, uint16_t lens,
                                otIp6Address src_addr)
{
    char *json = (char *)data;

    char msg_type[16]               = {0};
    char dev_type_str[16]           = {0};
    char dev_name[MIU_DEV_NAME_MAX] = {0};
    int  rloc16_val                 = 0;
    int  seq                        = 0;

    if (miu_json_get_str(json, "type",     msg_type,     sizeof(msg_type))     < 0 ||
        miu_json_get_str(json, "dev_type", dev_type_str, sizeof(dev_type_str)) < 0 ||
        miu_json_get_str(json, "dev_name", dev_name,     sizeof(dev_name))     < 0) {
        log_info("[json] malformed packet, drop");
        return;
    }
    miu_json_get_int(json, "rloc16", &rloc16_val);
    miu_json_get_int(json, "seq",    &seq);

    miu_dev_type_t dev_type = app_device_type_from_str(dev_type_str);

    /* ---- REGISTER ---- */
    if (strcmp(msg_type, "REGISTER") == 0) {
        char fw_ver[MIU_FW_VER_MAX] = {0};
        char hw_ver[MIU_HW_VER_MAX] = {0};
        miu_json_get_str(json, "fw_ver", fw_ver, sizeof(fw_ver));
        miu_json_get_str(json, "hw_ver", hw_ver, sizeof(hw_ver));

        app_device_table_add(dev_name, dev_type, &src_addr,
                             (uint16_t)rloc16_val, fw_ver, hw_ver);

        /* 转发到 PC */
        app_uart_pc_send(json);

        /* 回 ACK 给子设备 */
        char ack_buf[256];
        snprintf(ack_buf, sizeof(ack_buf),
                 "{\"ver\":1,\"type\":\"ACK\",\"dev_type\":\"%s\","
                 "\"dev_name\":\"%s\",\"seq\":%d,"
                 "\"data\":{\"seq_ack\":%d,\"code\":0,\"msg\":\"ok\"}}",
                 dev_type_str, dev_name, seq + 1, seq);
        uint16_t ack_len = (uint16_t)strlen(ack_buf);
        uint8_t *buf = pvPortMalloc(ack_len);
        if (buf) {
            memcpy(buf, ack_buf, ack_len);
            app_udpSend(src_addr, buf, ack_len, false);
            vPortFree(buf);
        }

    /* ---- REPORT ---- */
    } else if (strcmp(msg_type, "REPORT") == 0) {
        miu_device_info_t *entry = app_device_table_find(dev_name);
        if (!entry) {
            /* 收到 REPORT 但尚未 REGISTER：尝试自动补录（IP/rloc 已知）*/
            entry = app_device_table_add(dev_name, dev_type, &src_addr,
                                         (uint16_t)rloc16_val, "", "");
        }
        if (entry) {
            taskENTER_CRITICAL();
            entry->last_seen_ms =
                (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            int v = 0;
            switch (dev_type) {
            case MIU_DEV_TYPE_SOCKET:
                miu_json_get_int(json, "state",    &v);
                entry->data.socket.state = (uint8_t)v;
                break;
            case MIU_DEV_TYPE_LIGHT:
                miu_json_get_int(json, "lux",      &v);
                entry->data.light.lux = (uint32_t)v;
                break;
            case MIU_DEV_TYPE_RADAR:
                miu_json_get_int(json, "presence", &v);
                entry->data.radar.presence = (uint8_t)v;
                break;
            case MIU_DEV_TYPE_RGBCW:
                miu_json_get_int(json, "r", &v); entry->data.rgbcw.r = (uint8_t)v;
                miu_json_get_int(json, "g", &v); entry->data.rgbcw.g = (uint8_t)v;
                miu_json_get_int(json, "b", &v); entry->data.rgbcw.b = (uint8_t)v;
                miu_json_get_int(json, "c", &v); entry->data.rgbcw.c = (uint8_t)v;
                miu_json_get_int(json, "w", &v); entry->data.rgbcw.w = (uint8_t)v;
                break;
            default:
                break;
            }
            taskEXIT_CRITICAL();
        }
        /* 转发到 PC */
        app_uart_pc_send(json);

    /* ---- ACK（子设备响应 Leader 下发的 CONTROL） ---- */
    } else if (strcmp(msg_type, "ACK") == 0) {
        int code = 0;
        miu_json_get_int(json, "code", &code);
        log_info("[json] ACK from %s  code=%d", dev_name, code);

    } else {
        log_info("[json] unknown type '%s' from %s", msg_type, dev_name);
    }
}

#else /* MTD 侧：各子产品固件自行实现，此处提供弱符号占位 */

__attribute__((weak))
void app_udp_comm_json_process(uint8_t *data, uint16_t lens,
                                otIp6Address src_addr)
{
    (void)data; (void)lens; (void)src_addr;
    /* 子产品固件覆盖此函数以处理 CONTROL/ACK */
}
#endif /* CONFIG_MIU_DEVICE_TYPE_FTD */

/* -----------------------------------------------------------------------
 * 初始化：由 otrInitUser() 调用
 * ----------------------------------------------------------------------- */
void app_udp_comm_init(void)
{
    path_init();
#if CFG_USE_CENTRAK_CONFIG
    net_mgm_init();   /* 内部会调 otThreadSetEnabled，替代原 otrInitUser 末尾的调用 */
    app_ctrl_init();
#endif
}
