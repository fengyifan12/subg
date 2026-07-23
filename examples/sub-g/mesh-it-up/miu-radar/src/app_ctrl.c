/*
 * app_ctrl.c (MTD) — 控制命令接收侧
 *
 * MTD 节点只处理 Leader 下发的控制命令，不发送控制命令。
 *
 * 支持命令：
 *   CTRL_NODE_DETACH        (0x88880090) 停止 Thread 协议栈
 *   CTRL_NODE_WHITELIST     (0x88880091) 设置 MAC Allowlist 并重新 attach
 *   CTRL_NODE_CHILD_TIMEOUT (0x88880092) 更新 child timeout
 *   CTRL_NODE_REATTACH      (0x88880094) 触发重新选 parent
 *
 * 移植改动：
 *   - otGetInstance()            → otrGetInstance()
 *   - sw_timer_* / sw_timer_*   → TimerHandle_t / xTimer*
 *   - timer 回调内的 OT API      → ot_app_task_post()
 *   - info()                     → log_info()
 *   - mem_malloc / mem_free      → pvPortMalloc / vPortFree
 *   - app_udp_send()             → app_udpSend(..., false)
 */

#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <miu_port.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include "log.h"
#include "app_protocol.h"
#include "app_ctrl.h"
#include "app_udp.h"

#if CFG_USE_CENTRAK_CONFIG

#define CTRL_EXEC_DELAY_MS  1000

/* -----------------------------------------------------------------------
 * 状态
 * ----------------------------------------------------------------------- */
static TimerHandle_t s_detach_timer        = NULL;
static TimerHandle_t s_whitelist_timer     = NULL;
static TimerHandle_t s_reattach_exec_timer = NULL;
static uint8_t       s_whitelist_extaddr[OT_EXT_ADDRESS_SIZE];

/* -----------------------------------------------------------------------
 * 解析 / 序列化
 * ----------------------------------------------------------------------- */
static int ctrl_parse(uint32_t header, uint8_t *payload, uint16_t len, void *out)
{
    uint8_t *p = payload;

    if (header == CTRL_NODE_DETACH_HEADER) {
        ctrl_node_detach_t *d = (ctrl_node_detach_t *)out;
        memcpy(&d->header, p, 4); p += 4;
        d->seq = *p++;
    } else if (header == CTRL_NODE_WHITELIST_HEADER) {
        ctrl_node_whitelist_t *w = (ctrl_node_whitelist_t *)out;
        memcpy(&w->header, p, 4); p += 4;
        w->seq = *p++;
        memcpy(w->router_extaddr, p, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
    } else if (header == CTRL_NODE_CHILD_TIMEOUT_HEADER) {
        ctrl_node_child_timeout_t *t = (ctrl_node_child_timeout_t *)out;
        memcpy(&t->header, p, 4); p += 4;
        t->seq = *p++;
        memcpy(&t->timeout_secs, p, 4); p += 4;
    } else if (header == CTRL_NODE_ACK_HEADER) {
        ctrl_node_ack_t *a = (ctrl_node_ack_t *)out;
        memcpy(&a->header, p, 4); p += 4;
        a->seq    = *p++;
        a->cmd    = *p++;
        a->status = *p++;
    } else if (header == CTRL_NODE_REATTACH_HEADER) {
        ctrl_node_reattach_t *r = (ctrl_node_reattach_t *)out;
        memcpy(&r->header, p, 4); p += 4;
        r->seq = *p++;
    }

    if ((uint16_t)(p - payload) != len) {
        log_info("[ctrl] parse fail hdr=%08x (%u/%u)",
                 (unsigned)header, (unsigned)(p - payload), len);
        return 1;
    }
    return 0;
}

static void ctrl_ack_send(otIp6Address dst, uint8_t seq, uint8_t cmd, uint8_t status)
{
    ctrl_node_ack_t pkt = {
        .header = CTRL_NODE_ACK_HEADER,
        .seq    = seq,
        .cmd    = cmd,
        .status = status
    };
    uint8_t buf[7]; /* 4(header) + 1(seq) + 1(cmd) + 1(status) */
    uint8_t *p = buf;
    memcpy(p, &pkt.header, 4); p += 4;
    *p++ = pkt.seq;
    *p++ = pkt.cmd;
    *p++ = pkt.status;
    uint16_t len = (uint16_t)(p - buf);
    if (app_udpSend(dst, buf, len, false)) {
        log_info("[ctrl] ack send fail");
    }
}

/* -----------------------------------------------------------------------
 * 延迟执行 tasks
 * ----------------------------------------------------------------------- */
static void do_detach_task(void *arg)
{
    if (otThreadSetEnabled(otrGetInstance(), false) != OT_ERROR_NONE) {
        log_info("[ctrl] detach: otThreadSetEnabled(false) fail");
    } else {
        log_info("[ctrl] detach: Thread stack stopped");
    }
}

static void detach_timer_cb(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    s_detach_timer = NULL;
    ot_app_task_post(do_detach_task, NULL);
}

static void do_whitelist_task(void *arg)
{
    otInstance *inst = otrGetInstance();
    bool is_clear = true;
    for (int i = 0; i < OT_EXT_ADDRESS_SIZE; i++) {
        if (s_whitelist_extaddr[i] != 0xFF) { is_clear = false; break; }
    }

    if (is_clear) {
        otLinkFilterClearAddresses(inst);
        otLinkFilterSetAddressMode(inst, OT_MAC_FILTER_ADDRESS_MODE_DISABLED);
        log_info("[ctrl] whitelist cleared");
    } else {
        otExtAddress ext;
        memcpy(ext.m8, s_whitelist_extaddr, OT_EXT_ADDRESS_SIZE);
        otLinkFilterClearAddresses(inst);
        otLinkFilterAddAddress(inst, &ext);
        otLinkFilterSetAddressMode(inst, OT_MAC_FILTER_ADDRESS_MODE_ALLOWLIST);
        log_info("[ctrl] whitelist set router=%02X%02X%02X%02X%02X%02X%02X%02X, re-attaching",
                 s_whitelist_extaddr[0], s_whitelist_extaddr[1],
                 s_whitelist_extaddr[2], s_whitelist_extaddr[3],
                 s_whitelist_extaddr[4], s_whitelist_extaddr[5],
                 s_whitelist_extaddr[6], s_whitelist_extaddr[7]);
        if (otThreadBecomeDetached(inst) != OT_ERROR_NONE) {
            log_info("[ctrl] whitelist: BecomeDetached fail");
        }
    }
}

static void whitelist_timer_cb(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    s_whitelist_timer = NULL;
    ot_app_task_post(do_whitelist_task, NULL);
}

static void do_reattach_task(void *arg)
{
    if (otThreadBecomeDetached(otrGetInstance()) != OT_ERROR_NONE) {
        log_info("[ctrl] reattach: BecomeDetached fail");
    } else {
        log_info("[ctrl] reattach: detached, will re-select parent");
    }
}

static void reattach_timer_cb(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    s_reattach_exec_timer = NULL;
    ot_app_task_post(do_reattach_task, NULL);
}

/* -----------------------------------------------------------------------
 * 接收处理函数（被 app_udp_comm_process 调用，在 OT 任务上下文）
 * ----------------------------------------------------------------------- */
void app_udp_comm_ctrl_detach_proc(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    ctrl_node_detach_t pkt;
    if (ctrl_parse(CTRL_NODE_DETACH_HEADER, data, lens, &pkt)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[ctrl] << detach(%s) seq=%u", s, pkt.seq);

    ctrl_ack_send(src_addr, pkt.seq, CTRL_CMD_DETACH, CTRL_STATUS_OK);

    if (s_detach_timer) { xTimerDelete(s_detach_timer, 0); s_detach_timer = NULL; }
    s_detach_timer = xTimerCreate("ctrl_det", pdMS_TO_TICKS(CTRL_EXEC_DELAY_MS),
                                   pdFALSE, NULL, detach_timer_cb);
    if (s_detach_timer) xTimerStart(s_detach_timer, 0);
}

void app_udp_comm_ctrl_whitelist_proc(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    ctrl_node_whitelist_t pkt;
    if (ctrl_parse(CTRL_NODE_WHITELIST_HEADER, data, lens, &pkt)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[ctrl] << whitelist(%s) seq=%u router=%02X%02X%02X%02X%02X%02X%02X%02X",
             s, pkt.seq,
             pkt.router_extaddr[0], pkt.router_extaddr[1],
             pkt.router_extaddr[2], pkt.router_extaddr[3],
             pkt.router_extaddr[4], pkt.router_extaddr[5],
             pkt.router_extaddr[6], pkt.router_extaddr[7]);

    ctrl_ack_send(src_addr, pkt.seq, CTRL_CMD_WHITELIST, CTRL_STATUS_OK);

    memcpy(s_whitelist_extaddr, pkt.router_extaddr, OT_EXT_ADDRESS_SIZE);
    if (s_whitelist_timer) { xTimerDelete(s_whitelist_timer, 0); s_whitelist_timer = NULL; }
    s_whitelist_timer = xTimerCreate("ctrl_wl", pdMS_TO_TICKS(CTRL_EXEC_DELAY_MS),
                                      pdFALSE, NULL, whitelist_timer_cb);
    if (s_whitelist_timer) xTimerStart(s_whitelist_timer, 0);
}

void app_udp_comm_ctrl_child_timeout_proc(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    ctrl_node_child_timeout_t pkt;
    if (ctrl_parse(CTRL_NODE_CHILD_TIMEOUT_HEADER, data, lens, &pkt)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[ctrl] << child_timeout(%s) seq=%u timeout=%lu s",
             s, pkt.seq, (unsigned long)pkt.timeout_secs);

    otThreadSetChildTimeout(otrGetInstance(), pkt.timeout_secs);
    ctrl_ack_send(src_addr, pkt.seq, CTRL_CMD_CHILD_TIMEOUT, CTRL_STATUS_OK);
}

void app_udp_comm_ctrl_reattach_proc(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    ctrl_node_reattach_t pkt;
    if (ctrl_parse(CTRL_NODE_REATTACH_HEADER, data, lens, &pkt)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[ctrl] << reattach(%s) seq=%u", s, pkt.seq);

    ctrl_ack_send(src_addr, pkt.seq, CTRL_CMD_REATTACH, CTRL_STATUS_OK);

    if (s_reattach_exec_timer) {
        xTimerDelete(s_reattach_exec_timer, 0);
        s_reattach_exec_timer = NULL;
    }
    s_reattach_exec_timer = xTimerCreate("ctrl_rtch", pdMS_TO_TICKS(CTRL_EXEC_DELAY_MS),
                                          pdFALSE, NULL, reattach_timer_cb);
    if (s_reattach_exec_timer) xTimerStart(s_reattach_exec_timer, 0);
}

/* -----------------------------------------------------------------------
 * 初始化
 * ----------------------------------------------------------------------- */
void app_ctrl_init(void)
{
    log_info("[ctrl] MTD ctrl init");
}

#endif /* CFG_USE_CENTRAK_CONFIG */
