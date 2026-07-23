/*
 * app_net_mgm.c (MTD) — Centrak 网络管理，Child/End-Device 侧实现
 *
 * MTD 只实现「非 Leader 设备流程」：
 *   1. 入网后发送 Challenge，等待 Leader 的 Accepted 回复
 *   2. Challenge 超时则重试，最多 5 次，超过后重置 Thread 栈
 *   3. 收到 Reset 命令后倒计时执行复位或出厂复位
 *
 * 移植改动（同 FTD 侧）：
 *   - otGetInstance()          → otrGetInstance()
 *   - sw_timer_*               → FreeRTOS repeating timer（1 s 周期）
 *   - timer 回调调用 OT API    → ot_app_task_post()
 *   - info()                   → log_info()
 *   - mem_malloc / mem_free    → pvPortMalloc / vPortFree
 *   - app_udp_send()           → app_udpSend(..., false)
 */

#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <miu_port.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/dataset.h>
#include "log.h"
#include "app_protocol.h"
#include "app_udp.h"
#include "app_led.h"
#include "miu_bin_version.h"

#if CFG_USE_CENTRAK_CONFIG

/* -----------------------------------------------------------------------
 * 配置
 * ----------------------------------------------------------------------- */
#define NET_MGM_CHALLENGE_TIMEOUT_SEC   35

/* -----------------------------------------------------------------------
 * 内部结构（与 FTD wire 格式一致）
 * ----------------------------------------------------------------------- */
typedef struct {
    uint32_t header;
    uint8_t  role;
    uint16_t parent;
    uint16_t self_rloc;
    uint8_t  self_extaddr[OT_EXT_ADDRESS_SIZE];
    int8_t   rssi;
    uint32_t version;
} __attribute__((packed)) net_mgm_node_challenge_t;

typedef struct {
    uint32_t header;
    int      status;
} __attribute__((packed)) net_mgm_node_accepted_t;

typedef struct {
    uint32_t header;
    bool     need_erase;
} __attribute__((packed)) net_mgm_node_reset_t;

/* -----------------------------------------------------------------------
 * 状态
 * ----------------------------------------------------------------------- */
static TimerHandle_t s_mgm_timer         = NULL;
static bool          s_network_complete  = false;
static uint8_t       s_challenge_counts  = 0;
static uint16_t      s_challenge_cntdown = 0;
static uint16_t      s_attach_timer      = 0;
static uint16_t      s_reset_cntdown     = 0xffff;
static bool          s_reset_erase       = false;

/* -----------------------------------------------------------------------
 * 序列化 / 反序列化
 * ----------------------------------------------------------------------- */
static int nwk_parse(uint32_t header, uint8_t *payload, uint16_t len, void *out)
{
    uint8_t *p = payload;

    if (header == NET_MGM_NODE_CHALLENGE_HEADER) {
        net_mgm_node_challenge_t *ch = (net_mgm_node_challenge_t *)out;
        memcpy(&ch->header, p, 4);   p += 4;
        ch->role = *p++;
        memcpy(&ch->parent, p, 2);   p += 2;
        memcpy(&ch->self_rloc, p, 2);p += 2;
        memcpy(ch->self_extaddr, p, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
        ch->rssi = *p++;
        memcpy(&ch->version, p, 4);  p += 4;
    } else if (header == NET_MGM_NODE_ACCEPTED_HEADER) {
        net_mgm_node_accepted_t *acc = (net_mgm_node_accepted_t *)out;
        memcpy(&acc->header, p, 4);  p += 4;
        memcpy(&acc->status, p, 4);  p += 4;
    } else if (header == NET_MGM_NODE_RESET_HEADER) {
        net_mgm_node_reset_t *rst = (net_mgm_node_reset_t *)out;
        memcpy(&rst->header, p, 4);  p += 4;
        rst->need_erase = (bool)(*p++);
    }

    if ((uint16_t)(p - payload) != len) return 1;
    return 0;
}

static void nwk_pack_challenge(uint8_t *payload, uint16_t *out_len,
                                net_mgm_node_challenge_t *ch)
{
    uint8_t *p = payload;
    memcpy(p, &ch->header, 4);   p += 4;
    *p++ = ch->role;
    memcpy(p, &ch->parent, 2);   p += 2;
    memcpy(p, &ch->self_rloc, 2);p += 2;
    memcpy(p, ch->self_extaddr, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
    *p++ = (uint8_t)ch->rssi;
    memcpy(p, &ch->version, 4);  p += 4;
    *out_len = (uint16_t)(p - payload);
}

/* -----------------------------------------------------------------------
 * Challenge 发送
 * ----------------------------------------------------------------------- */
static void net_mgm_challenge_send(void)
{
    otInstance *inst = otrGetInstance();
    net_mgm_node_challenge_t ch;
    otRouterInfo          parent_info;
    otNeighborInfo        nbr_info;
    otNeighborInfoIterator iter = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    otExtAddress ext_addr;

    memset(&ch, 0, sizeof(ch));
    ch.header = NET_MGM_NODE_CHALLENGE_HEADER;

    if (otThreadGetParentInfo(inst, &parent_info) != OT_ERROR_NONE) {
        log_info("[mgm] get parent fail");
        return;
    }

    memset(&nbr_info, 0, sizeof(nbr_info));
    while (otThreadGetNextNeighborInfo(inst, &iter, &nbr_info) == OT_ERROR_NONE) {
        if (nbr_info.mRloc16 == parent_info.mRloc16) break;
    }

    ch.role      = (uint8_t)otThreadGetDeviceRole(inst);
    ch.parent    = parent_info.mRloc16;
    ch.self_rloc = otThreadGetRloc16(inst);
    ext_addr     = *otLinkGetExtendedAddress(inst);
    memcpy(ch.self_extaddr, ext_addr.m8, OT_EXT_ADDRESS_SIZE);
    ch.rssi      = nbr_info.mLastRssi;
    ch.version   = GET_BIN_VERSION(systeminfo.sysinfo);

    otIp6Address dst = *otThreadGetRloc(inst);
    dst.mFields.m8[14] = 0xFC;
    dst.mFields.m8[15] = 0x00;

    uint8_t *buf = pvPortMalloc(sizeof(net_mgm_node_challenge_t));
    if (buf) {
        uint16_t len = 0;
        nwk_pack_challenge(buf, &len, &ch);
        if (app_udpSend(dst, buf, len, false)) {
            log_info("[mgm] challenge send fail");
        } else {
            char s[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&dst, s, sizeof(s));
            log_info("[Networking] >> challenge (%s)", s);
        }
        vPortFree(buf);
    }
}

/* -----------------------------------------------------------------------
 * UDP 分发回调（在 OT 任务上下文）
 * ----------------------------------------------------------------------- */
void app_udp_comm_net_mgm_node_accepted_proc(uint8_t *data, uint16_t lens,
                                              otIp6Address src_addr)
{
    net_mgm_node_accepted_t acc;
    if (!nwk_parse(NET_MGM_NODE_ACCEPTED_HEADER, data, lens, &acc)) {
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&src_addr, s, sizeof(s));
        log_info("[Networking] << accepted [%d] (%s)", acc.status, s);
        if (!acc.status) {
            s_network_complete = true;
        }
    }
}

void app_udp_comm_net_mgm_node_reset_proc(uint8_t *data, uint16_t lens,
                                           otIp6Address src_addr)
{
    net_mgm_node_reset_t rst;
    if (!nwk_parse(NET_MGM_NODE_RESET_HEADER, data, lens, &rst)) {
        log_info("[Networking] << reset %s",
                 rst.need_erase ? "need_erase" : "no_erase");
        s_reset_cntdown = 30;
        s_reset_erase   = rst.need_erase;
        log_info("[mgm] reset in %u seconds", s_reset_cntdown);
    }
}

/* -----------------------------------------------------------------------
 * 1-second 心跳（运行在 OT 任务上下文）
 * ----------------------------------------------------------------------- */
static void net_mgm_heartbeat_task(void *arg)
{
    otInstance *inst = otrGetInstance();

    if (!s_network_complete) {
        s_attach_timer++;
        app_set_led1_toggle();
    } else {
        app_set_led1_on();
    }

    if (s_challenge_cntdown > 0 && --s_challenge_cntdown == 0) {
        if (s_network_complete) {
            s_challenge_cntdown = 0;
            s_challenge_counts  = 0;
        } else {
            s_challenge_counts++;
            if (s_challenge_counts > 5) {
                s_challenge_counts = 0;
                log_info("[mgm] challenge × 5, reset Thread stack");
                otThreadSetEnabled(inst, false);
                otInstanceErasePersistentInfo(inst);
                otThreadSetEnabled(inst, true);
            } else {
                log_info("[Networking] challenge try %d", s_challenge_counts);
                net_mgm_challenge_send();
                s_challenge_cntdown = NET_MGM_CHALLENGE_TIMEOUT_SEC;
            }
        }
    }

    if (s_reset_cntdown != 0xffff) {
        if (s_reset_cntdown == 0) {
            if (s_reset_erase) {
                otInstanceFactoryReset(inst);
            } else {
                otInstanceReset(inst);
            }
        } else {
            log_info("[mgm] reset countdown %u", s_reset_cntdown);
            s_reset_cntdown--;
        }
    }
}

static void net_mgm_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ot_app_task_post(net_mgm_heartbeat_task, NULL);
}

/* -----------------------------------------------------------------------
 * 角色变化回调（在 OT 任务上下文）
 * ----------------------------------------------------------------------- */
void net_mgm_interface_state_change(uint32_t aFlags, void *aContext)
{
    if (!(aFlags & OT_CHANGED_THREAD_ROLE)) return;

    otInstance   *inst = otrGetInstance();
    otDeviceRole  role = otThreadGetDeviceRole(inst);

    switch (role) {
    case OT_DEVICE_ROLE_DETACHED:
        log_info("[mgm] detached");
        s_attach_timer      = 0;
        s_challenge_cntdown = 0;
        s_network_complete  = false;
        break;
    case OT_DEVICE_ROLE_DISABLED:
        log_info("[mgm] disabled");
        break;
    case OT_DEVICE_ROLE_LEADER:
        log_info("[mgm] became leader (unexpected on MTD)");
        app_set_led1_on();
        break;
    case OT_DEVICE_ROLE_ROUTER:
        log_info("[mgm] became router");
        s_network_complete = true;
        break;
    case OT_DEVICE_ROLE_CHILD:
        log_info("[mgm] became child, send challenge");
        s_network_complete  = false;
        s_challenge_cntdown = NET_MGM_CHALLENGE_TIMEOUT_SEC;
        net_mgm_challenge_send();
        break;
    default:
        break;
    }

    /* 打印 IP 地址 */
    char s[OT_IP6_ADDRESS_STRING_SIZE];
    for (const otNetifAddress *addr = otIp6GetUnicastAddresses(inst);
         addr; addr = addr->mNext) {
        otIp6AddressToString(&addr->mAddress, s, sizeof(s));
        log_info("  %s", s);
    }
}

/* -----------------------------------------------------------------------
 * net_mgm_init — 由 app_udp_comm_init() 调用
 * ----------------------------------------------------------------------- */
void net_mgm_init(void)
{
    log_info("[mgm] MTD network management init");

    if (!s_mgm_timer) {
        s_mgm_timer = xTimerCreate("mgm_tick",
                                    pdMS_TO_TICKS(1000),
                                    pdTRUE,
                                    NULL,
                                    net_mgm_timer_cb);
        if (s_mgm_timer) xTimerStart(s_mgm_timer, 0);
    }

    if (otSetStateChangedCallback(otrGetInstance(),
                                   net_mgm_interface_state_change, NULL) != OT_ERROR_NONE) {
        log_warn("[mgm] otSetStateChangedCallback fail");
    }

    if (otIp6SetEnabled(otrGetInstance(), true) != OT_ERROR_NONE) {
        log_warn("[mgm] otIp6SetEnabled fail");
    }
    if (otThreadSetEnabled(otrGetInstance(), true) != OT_ERROR_NONE) {
        log_warn("[mgm] otThreadSetEnabled fail");
    }
}

#endif /* CFG_USE_CENTRAK_CONFIG */
