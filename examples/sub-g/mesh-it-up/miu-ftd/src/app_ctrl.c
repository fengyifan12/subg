/*
 * app_ctrl.c (FTD) — Leader 网络控制协议实现
 *
 * 移植自独立工程 ThreadSubG_FTD/app_ctrl.c，适配 mesh-it-up FreeRTOS 框架。
 *
 * 主要改动：
 *   - otGetInstance()              → otrGetInstance()
 *   - sw_timer_t * / sw_timer_*   → TimerHandle_t / xTimer*
 *   - timer 回调内的 OT API 调用   → ot_app_task_post() 投递到 OT 任务上下文
 *   - info()                       → log_info()
 *   - mem_malloc / mem_free        → pvPortMalloc / vPortFree
 *   - app_udp_send()               → app_udpSend(..., false)
 *
 * 命令说明：
 *   ctrl_detach_send()            Leader → 指定节点，停止 Thread 栈
 *   ctrl_whitelist_send()         Leader → 指定节点，绑定 MAC Allowlist parent
 *   ctrl_child_timeout_send()     Leader → ff03::1 广播，设置 child timeout
 *   ctrl_reattach_send()          Leader → 指定节点，触发重新选 parent
 *   ctrl_reattach_multicast_send()Leader → ff03::1 广播，触发所有 child 重新选 parent
 */

#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <miu_port.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include "log.h"
#include "app_protocol.h"
#include "app_ctrl.h"
#include "app_udp.h"

#if CFG_USE_CENTRAK_CONFIG

/* -----------------------------------------------------------------------
 * 常量
 * ----------------------------------------------------------------------- */
#define CTRL_EXEC_DELAY_MS          1000  /* ACK 发出后，延迟执行的时间 */

#define CTRL_REATTACH_RETRY_MAX     3
#define CTRL_REATTACH_RETRY_MS      3000

#define CTRL_MCAST_REPEAT           1     /* 多播重发次数（含首次共 REPEAT+1 次）*/
#define CTRL_MCAST_INTERVAL_MS      3000
#define CTRL_MCAST_COLLECT_MS       3000  /* 最后一次多播后等待 ACK 收集窗口 */
#define CTRL_MCAST_PAYLOAD_MAX      32
#define CTRL_MCAST_ACKED_MAX        200
#define CTRL_MCAST_UNICAST_WAIT_MS  4000
#define CTRL_MCAST_UNICAST_RETRY    2

/* -----------------------------------------------------------------------
 * 内部状态
 * ----------------------------------------------------------------------- */
static uint8_t ctrl_seq = 0;

static TimerHandle_t s_detach_timer        = NULL;
static TimerHandle_t s_whitelist_timer     = NULL;
static TimerHandle_t s_reattach_exec_timer = NULL;
static TimerHandle_t s_reattach_retry_timer= NULL;
static uint8_t       s_whitelist_extaddr[OT_EXT_ADDRESS_SIZE];
static otIp6Address  s_reattach_retry_addr;
static uint8_t       s_reattach_retry_seq;
static uint8_t       s_reattach_retry_cnt;

/* 多播 ACK 追踪器 */
typedef struct {
    TimerHandle_t timer;
    uint8_t  payload[CTRL_MCAST_PAYLOAD_MAX];
    uint16_t payloadlen;
    const char *name;
    uint8_t  cmd;
    uint8_t  seq;
    uint8_t  mcast_remain;
    uint16_t acked_rlocs[CTRL_MCAST_ACKED_MAX];
    uint16_t acked_count;
    bool     unicast_active;
    int      iter_idx;
    uint16_t cur_rloc;
    uint8_t  cur_retry;
} ctrl_mcast_ctx_t;

static ctrl_mcast_ctx_t s_mcast_ctx;

/* -----------------------------------------------------------------------
 * 序列化 / 反序列化
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

static void ctrl_pack(uint32_t header, uint8_t *payload, uint16_t *out_len, void *src)
{
    uint8_t *p = payload;
    memcpy(p, &header, 4); p += 4;

    if (header == CTRL_NODE_DETACH_HEADER) {
        ctrl_node_detach_t *d = (ctrl_node_detach_t *)src;
        *p++ = d->seq;
    } else if (header == CTRL_NODE_WHITELIST_HEADER) {
        ctrl_node_whitelist_t *w = (ctrl_node_whitelist_t *)src;
        *p++ = w->seq;
        memcpy(p, w->router_extaddr, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
    } else if (header == CTRL_NODE_CHILD_TIMEOUT_HEADER) {
        ctrl_node_child_timeout_t *t = (ctrl_node_child_timeout_t *)src;
        *p++ = t->seq;
        memcpy(p, &t->timeout_secs, 4); p += 4;
    } else if (header == CTRL_NODE_ACK_HEADER) {
        ctrl_node_ack_t *a = (ctrl_node_ack_t *)src;
        *p++ = a->seq;
        *p++ = a->cmd;
        *p++ = a->status;
    } else if (header == CTRL_NODE_REATTACH_HEADER) {
        ctrl_node_reattach_t *r = (ctrl_node_reattach_t *)src;
        *p++ = r->seq;
    }

    *out_len = (uint16_t)(p - payload);
}

/* -----------------------------------------------------------------------
 * 工具
 * ----------------------------------------------------------------------- */
static void ctrl_ack_send(otIp6Address dst, uint8_t seq, uint8_t cmd, uint8_t status)
{
    ctrl_node_ack_t pkt = {
        .header = CTRL_NODE_ACK_HEADER,
        .seq    = seq,
        .cmd    = cmd,
        .status = status
    };
    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_ack_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_ACK_HEADER, buf, &len, &pkt);
        if (app_udpSend(dst, buf, len, false)) {
            log_info("[ctrl] ack send fail");
        }
        vPortFree(buf);
    }
}

static otIp6Address rloc16_to_ip6(uint16_t rloc16)
{
    otIp6Address addr;
    const otMeshLocalPrefix *pfx = otThreadGetMeshLocalPrefix(otrGetInstance());
    memcpy(addr.mFields.m8, pfx->m8, 8);
    addr.mFields.m8[8]  = 0x00;
    addr.mFields.m8[9]  = 0x00;
    addr.mFields.m8[10] = 0x00;
    addr.mFields.m8[11] = 0xff;
    addr.mFields.m8[12] = 0xfe;
    addr.mFields.m8[13] = 0x00;
    addr.mFields.m8[14] = (uint8_t)(rloc16 >> 8);
    addr.mFields.m8[15] = (uint8_t)(rloc16 & 0xff);
    return addr;
}

static bool ip6_to_rloc16(const otIp6Address *addr, uint16_t *out)
{
    if (addr->mFields.m8[8]  == 0x00 && addr->mFields.m8[9]  == 0x00 &&
        addr->mFields.m8[10] == 0x00 && addr->mFields.m8[11] == 0xff &&
        addr->mFields.m8[12] == 0xfe && addr->mFields.m8[13] == 0x00) {
        *out = ((uint16_t)addr->mFields.m8[14] << 8) | addr->mFields.m8[15];
        return true;
    }
    return false;
}

/* -----------------------------------------------------------------------
 * 延迟执行：detach（停止 Thread 栈）
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

/* -----------------------------------------------------------------------
 * 延迟执行：whitelist
 * ----------------------------------------------------------------------- */
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
        log_info("[ctrl] whitelist set, detaching to re-select parent");
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

/* -----------------------------------------------------------------------
 * 延迟执行：reattach（child 角色才执行）
 * ----------------------------------------------------------------------- */
static void do_reattach_task(void *arg)
{
    otInstance *inst = otrGetInstance();
    otDeviceRole role = otThreadGetDeviceRole(inst);
    if (role == OT_DEVICE_ROLE_CHILD) {
        if (otThreadBecomeDetached(inst) != OT_ERROR_NONE) {
            log_info("[ctrl] reattach: BecomeDetached fail");
        } else {
            log_info("[ctrl] reattach: detached, will re-select parent");
        }
    } else {
        log_info("[ctrl] reattach ignored (role=%s)",
                 otThreadDeviceRoleToString(role));
    }
}

static void reattach_exec_timer_cb(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    s_reattach_exec_timer = NULL;
    ot_app_task_post(do_reattach_task, NULL);
}

/* -----------------------------------------------------------------------
 * 单播 reattach 重试
 * ----------------------------------------------------------------------- */
static void reattach_retry_timer_cb(TimerHandle_t xTimer);

static void start_reattach_retry_timer(void)
{
    s_reattach_retry_timer = xTimerCreate(
        "ctrl_rtry", pdMS_TO_TICKS(CTRL_REATTACH_RETRY_MS),
        pdFALSE, NULL, reattach_retry_timer_cb);
    if (s_reattach_retry_timer) xTimerStart(s_reattach_retry_timer, 0);
}

static void reattach_retry_timer_cb(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    s_reattach_retry_timer = NULL;

    if (s_reattach_retry_cnt >= CTRL_REATTACH_RETRY_MAX) {
        log_info("[ctrl] reattach retry exhausted seq=%u", s_reattach_retry_seq);
        return;
    }
    s_reattach_retry_cnt++;

    ctrl_node_reattach_t pkt = { .header = CTRL_NODE_REATTACH_HEADER,
                                  .seq    = s_reattach_retry_seq };
    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_reattach_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_REATTACH_HEADER, buf, &len, &pkt);
        app_udpSend(s_reattach_retry_addr, buf, len, false);
        log_info("[ctrl] >> reattach retry %u/%u seq=%u",
                 s_reattach_retry_cnt, CTRL_REATTACH_RETRY_MAX, s_reattach_retry_seq);
        vPortFree(buf);
    }
    start_reattach_retry_timer();
}

/* -----------------------------------------------------------------------
 * 多播 ACK 追踪器
 * ----------------------------------------------------------------------- */
static void ctrl_mcast_unicast_next(void);
static void ctrl_mcast_unicast_timer_cb(TimerHandle_t xTimer);
static void ctrl_mcast_phase_timer_cb(TimerHandle_t xTimer);

static bool mcast_is_acked(uint16_t rloc)
{
    for (uint16_t i = 0; i < s_mcast_ctx.acked_count; i++) {
        if (s_mcast_ctx.acked_rlocs[i] == rloc) return true;
    }
    return false;
}

static void ctrl_mcast_unicast_next(void)
{
    ctrl_mcast_ctx_t *ctx = &s_mcast_ctx;
    uint16_t rloc = 0;

    while (true) {
        int idx = net_mgm_node_iterate(ctx->iter_idx, &rloc);
        if (idx < 0) {
            log_info("[ctrl] mcast unicast fallback done (%s)", ctx->name);
            ctx->unicast_active = false;
            return;
        }
        ctx->iter_idx = idx + 1;
        if (mcast_is_acked(rloc)) continue;

        /* 发单播 */
        ctx->cur_rloc  = rloc;
        ctx->cur_retry = 0;
        otIp6Address dst = rloc16_to_ip6(rloc);
        uint8_t *buf = pvPortMalloc(ctx->payloadlen);
        if (buf) {
            memcpy(buf, ctx->payload, ctx->payloadlen);
            app_udpSend(dst, buf, ctx->payloadlen, false);
            char s[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&dst, s, sizeof(s));
            log_info("[ctrl] >> %s unicast→%s (rloc=%04X)", ctx->name, s, rloc);
            vPortFree(buf);
        }
        ctx->timer = xTimerCreate(ctx->name, pdMS_TO_TICKS(CTRL_MCAST_UNICAST_WAIT_MS),
                                   pdFALSE, NULL, ctrl_mcast_unicast_timer_cb);
        if (ctx->timer) xTimerStart(ctx->timer, 0);
        return;
    }
}

static void ctrl_mcast_unicast_timer_cb(TimerHandle_t xTimer)
{
    ctrl_mcast_ctx_t *ctx = &s_mcast_ctx;
    xTimerDelete(xTimer, 0);
    ctx->timer = NULL;

    ctx->cur_retry++;
    if (ctx->cur_retry <= CTRL_MCAST_UNICAST_RETRY) {
        otIp6Address dst = rloc16_to_ip6(ctx->cur_rloc);
        uint8_t *buf = pvPortMalloc(ctx->payloadlen);
        if (buf) {
            memcpy(buf, ctx->payload, ctx->payloadlen);
            app_udpSend(dst, buf, ctx->payloadlen, false);
            log_info("[ctrl] >> %s unicast retry %u/%u rloc=%04X",
                     ctx->name, ctx->cur_retry, CTRL_MCAST_UNICAST_RETRY, ctx->cur_rloc);
            vPortFree(buf);
        }
        ctx->timer = xTimerCreate(ctx->name, pdMS_TO_TICKS(CTRL_MCAST_UNICAST_WAIT_MS),
                                   pdFALSE, NULL, ctrl_mcast_unicast_timer_cb);
        if (ctx->timer) xTimerStart(ctx->timer, 0);
    } else {
        log_info("[ctrl] give up rloc=%04X (%s)", ctx->cur_rloc, ctx->name);
        ctrl_mcast_unicast_next();
    }
}

static void ctrl_mcast_phase_timer_cb(TimerHandle_t xTimer)
{
    ctrl_mcast_ctx_t *ctx = &s_mcast_ctx;
    xTimerDelete(xTimer, 0);
    ctx->timer = NULL;

    if (ctx->mcast_remain == 0) goto start_unicast;

    ctx->mcast_remain--;
    {
        otIp6Address mcast;
        otIp6AddressFromString("ff03::1", &mcast);
        uint8_t *buf = pvPortMalloc(ctx->payloadlen);
        if (buf) {
            memcpy(buf, ctx->payload, ctx->payloadlen);
            app_udpSend(mcast, buf, ctx->payloadlen, false);
            log_info("[ctrl] >> %s(ff03::1) mcast repeat remain=%u",
                     ctx->name, ctx->mcast_remain);
            vPortFree(buf);
        }
    }

    if (ctx->mcast_remain > 0) {
        ctx->timer = xTimerCreate(ctx->name, pdMS_TO_TICKS(CTRL_MCAST_INTERVAL_MS),
                                   pdFALSE, NULL, ctrl_mcast_phase_timer_cb);
        if (ctx->timer) xTimerStart(ctx->timer, 0);
        return;
    }

    if (!ctx->unicast_active) {
        ctx->unicast_active = true;
        ctx->timer = xTimerCreate(ctx->name, pdMS_TO_TICKS(CTRL_MCAST_COLLECT_MS),
                                   pdFALSE, NULL, ctrl_mcast_phase_timer_cb);
        if (ctx->timer) xTimerStart(ctx->timer, 0);
        return;
    }
    ctx->unicast_active = false;

start_unicast:
    log_info("[ctrl] mcast done, unicast fallback (%s acked=%u)", ctx->name, ctx->acked_count);
    ctx->unicast_active = true;
    ctx->iter_idx       = 0;
    ctrl_mcast_unicast_next();
}

static void ctrl_mcast_ack_start(const char *name, uint8_t cmd, uint8_t seq,
                                   uint8_t *payload, uint16_t payloadlen)
{
    ctrl_mcast_ctx_t *ctx = &s_mcast_ctx;
    if (payloadlen > CTRL_MCAST_PAYLOAD_MAX) {
        log_info("[ctrl] mcast payload too large (%u)", payloadlen);
        return;
    }
    if (ctx->timer) { xTimerDelete(ctx->timer, 0); ctx->timer = NULL; }

    ctx->name        = name;
    ctx->cmd         = cmd;
    ctx->seq         = seq;
    ctx->payloadlen  = payloadlen;
    ctx->mcast_remain= CTRL_MCAST_REPEAT - 1;
    ctx->acked_count = 0;
    ctx->unicast_active = false;
    ctx->iter_idx    = 0;
    ctx->cur_rloc    = 0;
    ctx->cur_retry   = 0;
    memcpy(ctx->payload, payload, payloadlen);

    if (ctx->mcast_remain > 0) {
        ctx->timer = xTimerCreate(name, pdMS_TO_TICKS(CTRL_MCAST_INTERVAL_MS),
                                   pdFALSE, NULL, ctrl_mcast_phase_timer_cb);
        if (ctx->timer) xTimerStart(ctx->timer, 0);
    } else {
        ctx->unicast_active = true;
        ctx->iter_idx       = 0;
        ctrl_mcast_unicast_next();
    }
}

static void ctrl_mcast_on_ack(uint8_t cmd, uint8_t seq, const otIp6Address *src)
{
    ctrl_mcast_ctx_t *ctx = &s_mcast_ctx;
    if (ctx->cmd != cmd || ctx->seq != seq) return;

    uint16_t rloc = 0;
    if (!ip6_to_rloc16(src, &rloc)) {
        if (!net_mgm_rloc_by_iid(&src->mFields.m8[8], &rloc)) return;
    }
    if (!mcast_is_acked(rloc) && ctx->acked_count < CTRL_MCAST_ACKED_MAX) {
        ctx->acked_rlocs[ctx->acked_count++] = rloc;
    }
    if (ctx->unicast_active && ctx->cur_rloc == rloc && ctx->timer) {
        xTimerDelete(ctx->timer, 0);
        ctx->timer = NULL;
        log_info("[ctrl] mcast unicast ACK rloc=%04X (%s)", rloc, ctx->name);
        ctrl_mcast_unicast_next();
    }
}

/* -----------------------------------------------------------------------
 * 发送接口（Leader 调用）
 * ----------------------------------------------------------------------- */
void ctrl_detach_send(otIp6Address dst_addr)
{
    ctrl_node_detach_t pkt = { .header = CTRL_NODE_DETACH_HEADER, .seq = ctrl_seq++ };
    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_detach_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_DETACH_HEADER, buf, &len, &pkt);
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_addr, s, sizeof(s));
        if (app_udpSend(dst_addr, buf, len, false)) {
            log_info("[ctrl] detach send fail → %s", s);
        } else {
            log_info("[ctrl] >> detach(%s) seq=%u", s, pkt.seq);
        }
        vPortFree(buf);
    }
}

void ctrl_whitelist_send(otIp6Address dst_addr, uint8_t *router_extaddr)
{
    ctrl_node_whitelist_t pkt;
    pkt.header = CTRL_NODE_WHITELIST_HEADER;
    pkt.seq    = ctrl_seq++;
    memcpy(pkt.router_extaddr, router_extaddr, OT_EXT_ADDRESS_SIZE);

    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_whitelist_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_WHITELIST_HEADER, buf, &len, &pkt);
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_addr, s, sizeof(s));
        if (app_udpSend(dst_addr, buf, len, false)) {
            log_info("[ctrl] whitelist send fail → %s", s);
        } else {
            log_info("[ctrl] >> whitelist(%s) seq=%u router=%02X%02X%02X%02X%02X%02X%02X%02X",
                     s, pkt.seq,
                     router_extaddr[0], router_extaddr[1], router_extaddr[2], router_extaddr[3],
                     router_extaddr[4], router_extaddr[5], router_extaddr[6], router_extaddr[7]);
        }
        vPortFree(buf);
    }
}

void ctrl_child_timeout_send(uint32_t timeout_secs)
{
    ctrl_node_child_timeout_t pkt = {
        .header       = CTRL_NODE_CHILD_TIMEOUT_HEADER,
        .seq          = ctrl_seq++,
        .timeout_secs = timeout_secs
    };
    otIp6Address mcast;
    otIp6AddressFromString("ff03::1", &mcast);

    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_child_timeout_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_CHILD_TIMEOUT_HEADER, buf, &len, &pkt);
        if (app_udpSend(mcast, buf, len, false)) {
            log_info("[ctrl] child_timeout send fail");
        } else {
            otThreadSetChildTimeout(otrGetInstance(), timeout_secs);
            log_info("[ctrl] >> child_timeout(ff03::1) seq=%u timeout=%lu s",
                     pkt.seq, (unsigned long)timeout_secs);
            ctrl_mcast_ack_start("ctrl_child_timeout", CTRL_CMD_CHILD_TIMEOUT,
                                  pkt.seq, buf, len);
        }
        vPortFree(buf);
    }
}

void ctrl_reattach_send(otIp6Address dst_addr)
{
    ctrl_node_reattach_t pkt = { .header = CTRL_NODE_REATTACH_HEADER, .seq = ctrl_seq++ };
    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_reattach_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_REATTACH_HEADER, buf, &len, &pkt);
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst_addr, s, sizeof(s));
        if (app_udpSend(dst_addr, buf, len, false)) {
            log_info("[ctrl] reattach send fail → %s", s);
        } else {
            log_info("[ctrl] >> reattach(%s) seq=%u", s, pkt.seq);
            if (s_reattach_retry_timer) {
                xTimerDelete(s_reattach_retry_timer, 0);
                s_reattach_retry_timer = NULL;
            }
            s_reattach_retry_addr = dst_addr;
            s_reattach_retry_seq  = pkt.seq;
            s_reattach_retry_cnt  = 0;
            start_reattach_retry_timer();
        }
        vPortFree(buf);
    }
}

void ctrl_reattach_multicast_send(void)
{
    ctrl_node_reattach_t pkt = { .header = CTRL_NODE_REATTACH_HEADER, .seq = ctrl_seq++ };
    otIp6Address mcast;
    otIp6AddressFromString("ff03::1", &mcast);

    uint8_t *buf = pvPortMalloc(sizeof(ctrl_node_reattach_t));
    if (buf) {
        uint16_t len = 0;
        ctrl_pack(CTRL_NODE_REATTACH_HEADER, buf, &len, &pkt);
        if (app_udpSend(mcast, buf, len, false)) {
            log_info("[ctrl] reattach multicast send fail");
        } else {
            log_info("[ctrl] >> reattach(ff03::1) seq=%u", pkt.seq);
            ctrl_mcast_ack_start("ctrl_reattach", CTRL_CMD_REATTACH,
                                  pkt.seq, buf, len);
        }
        vPortFree(buf);
    }
}

/* -----------------------------------------------------------------------
 * 接收处理（被 app_udp_comm_process 调用，已在 OT 任务上下文）
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
    s_detach_timer = xTimerCreate("ctrl_detach", pdMS_TO_TICKS(CTRL_EXEC_DELAY_MS),
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

void app_udp_comm_ctrl_ack_proc(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    ctrl_node_ack_t pkt;
    if (ctrl_parse(CTRL_NODE_ACK_HEADER, data, lens, &pkt)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[ctrl] << ack(%s) seq=%u cmd=0x%02X status=%s",
             s, pkt.seq, pkt.cmd,
             (pkt.status == CTRL_STATUS_OK) ? "OK" : "FAIL");

    if (pkt.cmd == CTRL_CMD_REATTACH && s_reattach_retry_timer &&
        pkt.seq == s_reattach_retry_seq) {
        xTimerDelete(s_reattach_retry_timer, 0);
        s_reattach_retry_timer = NULL;
        log_info("[ctrl] reattach ACK confirmed, retry cancelled seq=%u", pkt.seq);
    }
    ctrl_mcast_on_ack(pkt.cmd, pkt.seq, &src_addr);
}

void app_udp_comm_ctrl_reattach_proc(uint8_t *data, uint16_t lens, otIp6Address src_addr)
{
    ctrl_node_reattach_t pkt;
    if (ctrl_parse(CTRL_NODE_REATTACH_HEADER, data, lens, &pkt)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[ctrl] << reattach(%s) seq=%u", s, pkt.seq);

    otDeviceRole role = otThreadGetDeviceRole(otrGetInstance());
    if (role != OT_DEVICE_ROLE_CHILD) {
        log_info("[ctrl] reattach ignored (role=%s)", otThreadDeviceRoleToString(role));
        return;
    }

    ctrl_ack_send(src_addr, pkt.seq, CTRL_CMD_REATTACH, CTRL_STATUS_OK);

    if (s_reattach_exec_timer) { xTimerDelete(s_reattach_exec_timer, 0); s_reattach_exec_timer = NULL; }
    s_reattach_exec_timer = xTimerCreate("ctrl_rtch", pdMS_TO_TICKS(CTRL_EXEC_DELAY_MS),
                                          pdFALSE, NULL, reattach_exec_timer_cb);
    if (s_reattach_exec_timer) xTimerStart(s_reattach_exec_timer, 0);
}

/* -----------------------------------------------------------------------
 * 初始化
 * ----------------------------------------------------------------------- */
void app_ctrl_init(void)
{
    log_info("[ctrl] ctrl protocol init");
    memset(&s_mcast_ctx, 0, sizeof(s_mcast_ctx));
}

#endif /* CFG_USE_CENTRAK_CONFIG */
