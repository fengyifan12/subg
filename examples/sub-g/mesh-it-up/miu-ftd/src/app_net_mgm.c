/*
 * app_net_mgm.c (FTD) — Centrak 集中式网络管理，Leader 侧实现
 *
 * 移植自独立工程 ThreadSubG_FTD/app_net_mgm.c，适配 mesh-it-up FreeRTOS 框架。
 *
 * 主要改动：
 *   - otGetInstance()          → otrGetInstance()
 *   - sw_timer_*               → FreeRTOS repeating timer（1s 周期）
 *   - timer 回调调用 OT API    → ot_app_task_post() 投递到 OT 任务上下文
 *   - enter/leave_critical_section → taskENTER/EXIT_CRITICAL()
 *   - info()                   → log_info()
 *   - mem_malloc / mem_free    → pvPortMalloc / vPortFree
 *   - app_udp_send()           → app_udpSend(..., false)
 *   - gpio_pin_get(22)         → hosal_gpio_pin_get(22)
 *   - #include "bin_version.h" → #include "miu_bin_version.h"
 *
 * Leader 检测引脚：GPIO 22，低电平 = 本机是 Leader。
 * 如需更换引脚，修改 NET_MGM_LEADER_GPIO_PIN。
 */

#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <miu_port.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/ip6.h>
#include <openthread/link.h>
#include <openthread/dataset.h>
#include <openthread/random_noncrypto.h>
#include "hosal_gpio.h"
#include "hosal_sysctrl.h"
#include "log.h"
/* 包含项目级 OT 配置，获取 OPENTHREAD_CONFIG_MLE_MAX_CHILDREN 等宏 */
#include "app_miu_config.h"
#include "app_protocol.h"
#include "app_ctrl.h"
#include "app_udp.h"
#include "app_led.h"
#include "miu_bin_version.h"
#include "app_device_table.h"
#include "app_uart_pc.h"
/* EnhancedFlashDataset：替代原 flashds.h，用 key-value 接口存储 Partition ID。
 * EFD_USING_ENV 在 efd_cfg.h 默认已开启，efd_get/set_env_blob 可直接使用。
 * 注意：net_mgm_init() 调用时 enhanced_flash_dataset_init() 需已执行（由框架启动时完成）。*/
#include "EnhancedFlashDataset.h"

#if CFG_USE_CENTRAK_CONFIG

/* -----------------------------------------------------------------------
 * 配置
 * ----------------------------------------------------------------------- */
#define NET_MGM_LEADER_GPIO_PIN         22   /* 低电平 = 本机是 Leader */

#define NET_MGM_NODE_TABLE_MAX          (OPENTHREAD_CONFIG_MLE_MAX_ROUTERS + 300)
#define NET_MGM_POLL_SHORT_SEC          10   /* 询问相邻 Router 的短间隔（秒）*/
#define NET_MGM_POLL_LONG_SEC           120  /* 轮完一轮后的长等待（秒）*/
#define NET_MGM_NODE_SURVIVAL_SEC \
    ((NET_MGM_POLL_SHORT_SEC * OPENTHREAD_CONFIG_MLE_MAX_ROUTERS) + (NET_MGM_POLL_LONG_SEC * 2))
#define NET_MGM_CHALLENGE_TIMEOUT_SEC   35   /* Challenge 超时（秒）*/

/* -----------------------------------------------------------------------
 * 内部数据结构（与 wire 协议对应）
 * ----------------------------------------------------------------------- */

typedef struct {
    uint16_t rloc;
    uint8_t  extaddr[OT_EXT_ADDRESS_SIZE];
    int8_t   rssi;
} __attribute__((packed)) net_mgm_node_info_t;

typedef struct {
    uint32_t header;
    bool     need_erase;
} __attribute__((packed)) net_mgm_node_reset_t;

typedef struct {
    uint32_t header;
} __attribute__((packed)) net_mgm_node_ask_t;

typedef struct {
    uint32_t header;
    uint32_t version;
    uint16_t parent;
    uint16_t self_rloc;
    uint8_t  self_extaddr[OT_EXT_ADDRESS_SIZE];
    uint16_t num;
    net_mgm_node_info_t child_info[OPENTHREAD_CONFIG_MLE_MAX_CHILDREN];
} __attribute__((packed)) net_mgm_node_reply_t;

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

/* 节点表条目 */
typedef struct {
    uint8_t  used;
    uint8_t  role;
    uint16_t parent;
    uint16_t rloc;
    uint8_t  extaddr[OT_EXT_ADDRESS_SIZE];
    int8_t   rssi;
    uint32_t version;
    uint16_t survivaltime;
} net_mgm_node_table_t;

/* -----------------------------------------------------------------------
 * 模块状态
 * ----------------------------------------------------------------------- */
static net_mgm_node_table_t *s_node_table        = NULL;
static TimerHandle_t          s_mgm_timer         = NULL;

static bool     s_node_reset_erase     = false;
static bool     s_network_complete     = false;
static uint8_t  s_challenge_counts     = 0;
static uint8_t  s_last_router_id       = 0xff;
static uint16_t s_poll_countdown       = 0;
static uint16_t s_challenge_countdown  = 0;
static uint16_t s_attach_timer         = 0;
static uint16_t s_reset_countdown      = 0xffff;

static unsigned int s_debug_flags = 0;
#define mgm_dbg(...) do { if (s_debug_flags > 0) log_info(__VA_ARGS__); } while (0)

/* ------------------------------------------------------------------
 * 工具
 * ------------------------------------------------------------------ */
void net_mgm_debug_level(unsigned int level) { s_debug_flags = level; }

static bool net_mgm_check_leader_pin(void)
{
    uint32_t pin_value;
    hosal_gpio_pin_get(NET_MGM_LEADER_GPIO_PIN,&pin_value);
    return (pin_value == 0);
}

/* -----------------------------------------------------------------------
 * 节点表操作（需临界区保护）
 * ----------------------------------------------------------------------- */
static int node_table_add(uint8_t role, uint16_t parent, uint16_t rloc,
                          uint8_t *extaddr, int8_t rssi, uint32_t version)
{
    uint16_t i;
    uint16_t free_router = NET_MGM_NODE_TABLE_MAX;
    uint16_t free_child  = NET_MGM_NODE_TABLE_MAX;

    if (!s_node_table) return -1;

    /* 已存在则更新 */
    for (i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used &&
            memcmp(s_node_table[i].extaddr, extaddr, OT_EXT_ADDRESS_SIZE) == 0) {
            taskENTER_CRITICAL();
            s_node_table[i].parent = parent;
            s_node_table[i].role   = role;
            s_node_table[i].rloc   = rloc;
            s_node_table[i].rssi   = rssi;
            if (version != 0xFFFFFFFF) s_node_table[i].version = version;
            s_node_table[i].survivaltime = NET_MGM_NODE_SURVIVAL_SEC;
            taskEXIT_CRITICAL();
            mgm_dbg("[mgm] update %04X", rloc);
            return 0;
        }
    }

    /* 查空槽 */
    for (i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (!s_node_table[i].used) {
            if (i < OPENTHREAD_CONFIG_MLE_MAX_ROUTERS) {
                if (free_router == NET_MGM_NODE_TABLE_MAX) free_router = i;
            } else {
                if (free_child == NET_MGM_NODE_TABLE_MAX) { free_child = i; break; }
            }
        }
    }

    uint16_t idx = (role == OT_DEVICE_ROLE_ROUTER) ? free_router : free_child;
    if (idx == NET_MGM_NODE_TABLE_MAX) {
        log_info("[mgm] node table full");
        return -1;
    }

    taskENTER_CRITICAL();
    s_node_table[idx].used        = 1;
    s_node_table[idx].parent      = parent;
    s_node_table[idx].role        = role;
    s_node_table[idx].rloc        = rloc;
    s_node_table[idx].rssi        = rssi;
    s_node_table[idx].version     = version;
    s_node_table[idx].survivaltime= NET_MGM_NODE_SURVIVAL_SEC;
    memcpy(s_node_table[idx].extaddr, extaddr, OT_EXT_ADDRESS_SIZE);
    taskEXIT_CRITICAL();

    mgm_dbg("[mgm] add role=%u rloc=%04X", role, rloc);
    return 0;
}

bool net_mgm_node_table_find(uint8_t *aExtAddress)
{
    if (!s_node_table) return false;
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used &&
            memcmp(s_node_table[i].extaddr, aExtAddress, OT_EXT_ADDRESS_SIZE) == 0) {
            return true;
        }
    }
    return false;
}

void net_mgm_node_table_display(void)
{
    uint16_t count = 0;
    if (!s_node_table) return;

    log_info("idx  role    parent rloc   extaddr                  rssi  version    survival");
    log_info("===================================================================");

    /* 先打 Router */
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used && s_node_table[i].role == OT_DEVICE_ROLE_ROUTER) {
            log_info("[%u] %s %04X %04X %02X%02X%02X%02X%02X%02X%02X%02X %d 0x%08x %u",
                     ++count,
                     otThreadDeviceRoleToString((otDeviceRole)s_node_table[i].role),
                     s_node_table[i].parent, s_node_table[i].rloc,
                     s_node_table[i].extaddr[0], s_node_table[i].extaddr[1],
                     s_node_table[i].extaddr[2], s_node_table[i].extaddr[3],
                     s_node_table[i].extaddr[4], s_node_table[i].extaddr[5],
                     s_node_table[i].extaddr[6], s_node_table[i].extaddr[7],
                     s_node_table[i].rssi, s_node_table[i].version,
                     s_node_table[i].survivaltime);
        }
    }
    /* 再打 Child */
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used && s_node_table[i].role == OT_DEVICE_ROLE_CHILD) {
            log_info("[%u] %s %04X %04X %02X%02X%02X%02X%02X%02X%02X%02X %d 0x%08x %u",
                     ++count,
                     otThreadDeviceRoleToString((otDeviceRole)s_node_table[i].role),
                     s_node_table[i].parent, s_node_table[i].rloc,
                     s_node_table[i].extaddr[0], s_node_table[i].extaddr[1],
                     s_node_table[i].extaddr[2], s_node_table[i].extaddr[3],
                     s_node_table[i].extaddr[4], s_node_table[i].extaddr[5],
                     s_node_table[i].extaddr[6], s_node_table[i].extaddr[7],
                     s_node_table[i].rssi, s_node_table[i].version,
                     s_node_table[i].survivaltime);
        }
    }
    log_info("=== total %u nodes ===", count);
}

uint16_t net_mgm_get_children_of_router(uint16_t router_rloc,
                                         uint16_t *rloc_list, uint16_t max_count)
{
    uint16_t count = 0;
    if (!s_node_table || !rloc_list || max_count == 0) return 0;
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX && count < max_count; i++) {
        if (s_node_table[i].used &&
            s_node_table[i].role   == OT_DEVICE_ROLE_CHILD &&
            s_node_table[i].parent == router_rloc) {
            rloc_list[count++] = s_node_table[i].rloc;
        }
    }
    return count;
}

int net_mgm_node_iterate(int start_idx, uint16_t *out_rloc)
{
    if (!s_node_table || !out_rloc || start_idx < 0) return -1;
    for (int i = start_idx; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used) {
            *out_rloc = s_node_table[i].rloc;
            return i;
        }
    }
    return -1;
}

bool net_mgm_rloc_by_iid(const uint8_t iid[8], uint16_t *out_rloc)
{
    if (!s_node_table || !iid || !out_rloc) return false;
    for (int i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used &&
            memcmp(s_node_table[i].extaddr, iid, OT_EXT_ADDRESS_SIZE) == 0) {
            *out_rloc = s_node_table[i].rloc;
            return true;
        }
    }
    return false;
}

void net_mgm_invalidate_all_children(void)
{
    if (!s_node_table) return;
    taskENTER_CRITICAL();
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used && s_node_table[i].role == OT_DEVICE_ROLE_CHILD) {
            s_node_table[i].survivaltime = 0;
            s_node_table[i].role         = OT_DEVICE_ROLE_DETACHED;
        }
    }
    taskEXIT_CRITICAL();
    log_info("[mgm] all child entries invalidated");
}

/* -----------------------------------------------------------------------
 * 序列化 / 反序列化
 * ----------------------------------------------------------------------- */
static int nwk_node_parse(uint32_t header, uint8_t *payload, uint16_t len, void *out)
{
    uint8_t *p = payload;

    if (header == NET_MGM_NODE_ASK_HEADER) {
        net_mgm_node_ask_t *ask = (net_mgm_node_ask_t *)out;
        memcpy(&ask->header, p, 4); p += 4;
    } else if (header == NET_MGM_NODE_REPLY_HEADER) {
        net_mgm_node_reply_t *reply = (net_mgm_node_reply_t *)out;
        memcpy(&reply->header, p, 4);       p += 4;
        memcpy(&reply->version, p, 4);      p += 4;
        memcpy(&reply->parent, p, 2);       p += 2;
        memcpy(&reply->self_rloc, p, 2);    p += 2;
        memcpy(reply->self_extaddr, p, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
        memcpy(&reply->num, p, 2);          p += 2;
        if (reply->num > 0 && reply->num <= OPENTHREAD_CONFIG_MLE_MAX_CHILDREN) {
            memcpy(reply->child_info, p, reply->num * sizeof(net_mgm_node_info_t));
            p += reply->num * sizeof(net_mgm_node_info_t);
        }
    } else if (header == NET_MGM_NODE_CHALLENGE_HEADER) {
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

    if ((p - payload) != len) {
        mgm_dbg("[mgm] parse %08x fail (%u/%u)", header, (unsigned)(p - payload), len);
        return 1;
    }
    return 0;
}

static void nwk_node_pack(uint32_t header, uint8_t *payload, uint16_t *out_len, void *src)
{
    uint8_t *p = payload;
    memcpy(p, src, 4); p += 4;  /* header always first */

    if (header == NET_MGM_NODE_REPLY_HEADER) {
        net_mgm_node_reply_t *reply = (net_mgm_node_reply_t *)src;
        memcpy(p, &reply->version, 4);      p += 4;
        memcpy(p, &reply->parent, 2);       p += 2;
        memcpy(p, &reply->self_rloc, 2);    p += 2;
        memcpy(p, reply->self_extaddr, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
        memcpy(p, &reply->num, 2);          p += 2;
        if (reply->num > 0 && reply->num <= OPENTHREAD_CONFIG_MLE_MAX_CHILDREN) {
            memcpy(p, reply->child_info, reply->num * sizeof(net_mgm_node_info_t));
            p += reply->num * sizeof(net_mgm_node_info_t);
        }
    } else if (header == NET_MGM_NODE_CHALLENGE_HEADER) {
        net_mgm_node_challenge_t *ch = (net_mgm_node_challenge_t *)src;
        *p++ = ch->role;
        memcpy(p, &ch->parent, 2);   p += 2;
        memcpy(p, &ch->self_rloc, 2);p += 2;
        memcpy(p, ch->self_extaddr, OT_EXT_ADDRESS_SIZE); p += OT_EXT_ADDRESS_SIZE;
        *p++ = (uint8_t)ch->rssi;
        memcpy(p, &ch->version, 4);  p += 4;
    } else if (header == NET_MGM_NODE_ACCEPTED_HEADER) {
        net_mgm_node_accepted_t *acc = (net_mgm_node_accepted_t *)src;
        memcpy(p, &acc->status, 4);  p += 4;
    } else if (header == NET_MGM_NODE_RESET_HEADER) {
        net_mgm_node_reset_t *rst = (net_mgm_node_reset_t *)src;
        *p++ = (uint8_t)rst->need_erase;
    }
    /* NET_MGM_NODE_ASK_HEADER: header only, nothing extra */

    *out_len = (uint16_t)(p - payload);
}

/* -----------------------------------------------------------------------
 * UDP 发送函数
 * ----------------------------------------------------------------------- */
static void net_mgm_ask_send(uint16_t rloc16)
{
    otInstance *inst = otrGetInstance();
    otIp6Address dst = *otThreadGetRloc(inst);
    dst.mFields.m8[14] = (uint8_t)((rloc16 & 0xff00) >> 8);
    dst.mFields.m8[15] = (uint8_t)(rloc16 & 0xff);

    net_mgm_node_ask_t ask = { .header = NET_MGM_NODE_ASK_HEADER };
    uint8_t *buf = pvPortMalloc(sizeof(net_mgm_node_ask_t));
    if (buf) {
        uint16_t len = 0;
        nwk_node_pack(NET_MGM_NODE_ASK_HEADER, buf, &len, &ask);
        if (app_udpSend(dst, buf, len, false)) {
            log_info("[mgm] ask send fail → %04X", rloc16);
        } else {
            char s[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&dst, s, sizeof(s));
            log_info("[Networking] >> polling ask (%s)", s);
        }
        vPortFree(buf);
    }
}

static void net_mgm_reply_send(otIp6Address dst_addr)
{
    otInstance *inst = otrGetInstance();
    net_mgm_node_reply_t reply;
    otLeaderData  leader_data;
    otRouterInfo  router_info;
    otExtAddress  ext_addr;
    otChildInfo   child_info;
    uint16_t next_hop;
    uint8_t  path_cost;

    memset(&reply, 0, sizeof(reply));
    reply.header = NET_MGM_NODE_REPLY_HEADER;

    if (otThreadGetLeaderData(inst, &leader_data) != OT_ERROR_NONE) return;
    if (otThreadGetRouterInfo(inst, leader_data.mLeaderRouterId, &router_info) != OT_ERROR_NONE) return;

    otThreadGetNextHopAndPathCost(inst, router_info.mRloc16, &next_hop, &path_cost);
    reply.version   = GET_BIN_VERSION(systeminfo.sysinfo);
    reply.parent    = next_hop;
    reply.self_rloc = otThreadGetRloc16(inst);
    ext_addr        = *otLinkGetExtendedAddress(inst);
    memcpy(reply.self_extaddr, ext_addr.m8, OT_EXT_ADDRESS_SIZE);

    uint16_t max_child = otThreadGetMaxAllowedChildren(inst);
    for (uint16_t i = 0; i < max_child; i++) {
        if (otThreadGetChildInfoByIndex(inst, i, &child_info) != OT_ERROR_NONE ||
            child_info.mIsStateRestoring) continue;
        reply.child_info[reply.num].rloc = child_info.mRloc16;
        memcpy(reply.child_info[reply.num].extaddr, &child_info.mExtAddress, OT_EXT_ADDRESS_SIZE);
        reply.child_info[reply.num].rssi = child_info.mLastRssi;
        reply.num++;
    }

    uint8_t *buf = pvPortMalloc(sizeof(net_mgm_node_reply_t));
    if (buf) {
        uint16_t len = 0;
        nwk_node_pack(NET_MGM_NODE_REPLY_HEADER, buf, &len, &reply);
        if (app_udpSend(dst_addr, buf, len, false)) {
            log_info("[mgm] reply send fail");
        } else {
            char s[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&dst_addr, s, sizeof(s));
            log_info("[Networking] >> polling reply (%s)", s);
        }
        vPortFree(buf);
    }
}

static void net_mgm_challenge_send(void)
{
    otInstance *inst = otrGetInstance();
    net_mgm_node_challenge_t ch;
    otRouterInfo         parent_info;
    otNeighborInfo       nbr_info;
    otNeighborInfoIterator iter = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    otExtAddress ext_addr;

    memset(&ch, 0, sizeof(ch));
    ch.header = NET_MGM_NODE_CHALLENGE_HEADER;

    if (otThreadGetParentInfo(inst, &parent_info) != OT_ERROR_NONE) return;

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
        nwk_node_pack(NET_MGM_NODE_CHALLENGE_HEADER, buf, &len, &ch);
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

static void net_mgm_accepted_send(otIp6Address dst_addr, int status)
{
    net_mgm_node_accepted_t acc = {
        .header = NET_MGM_NODE_ACCEPTED_HEADER,
        .status = status
    };
    uint8_t *buf = pvPortMalloc(sizeof(net_mgm_node_accepted_t));
    if (buf) {
        uint16_t len = 0;
        nwk_node_pack(NET_MGM_NODE_ACCEPTED_HEADER, buf, &len, &acc);
        if (app_udpSend(dst_addr, buf, len, false)) {
            log_info("[mgm] accepted send fail");
        } else {
            char s[OT_IP6_ADDRESS_STRING_SIZE];
            otIp6AddressToString(&dst_addr, s, sizeof(s));
            log_info("[Networking] >> accepted (%s)", s);
        }
        vPortFree(buf);
    }
}

void net_mgm_node_reset_send(bool is_need_erase)
{
    net_mgm_node_reset_t rst = {
        .header     = NET_MGM_NODE_RESET_HEADER,
        .need_erase = is_need_erase
    };
    otIp6Address multicast;
    otIp6AddressFromString("ff03::1", &multicast);

    uint8_t *buf = pvPortMalloc(sizeof(net_mgm_node_reset_t));
    if (buf) {
        uint16_t len = 0;
        nwk_node_pack(NET_MGM_NODE_RESET_HEADER, buf, &len, &rst);
        if (app_udpSend(multicast, buf, len, false)) {
            log_info("[mgm] reset send fail");
        } else {
            log_info("[Networking] >> reset (ff03::1)");
        }
        vPortFree(buf);
    }
}

/* -----------------------------------------------------------------------
 * UDP 分发回调（被 app_udp_comm_process 调用，已在 OT 任务上下文）
 * ----------------------------------------------------------------------- */

/* Router 收到 Leader 的 ASK → 回复本机子节点表 */
void app_udp_comm_net_mgm_node_ack_proc(uint8_t *data, uint16_t lens,
                                          otIp6Address src_addr)
{
    net_mgm_node_ask_t ask;
    if (!nwk_node_parse(NET_MGM_NODE_ASK_HEADER, data, lens, &ask)) {
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&src_addr, s, sizeof(s));
        log_info("[Networking] << polling ask (%s)", s);
        net_mgm_reply_send(src_addr);
    }
}

/* Leader 收到 Router 的子节点表回复 */
void app_udp_comm_net_mgm_node_reply_proc(uint8_t *data, uint16_t lens,
                                           otIp6Address src_addr)
{
    otInstance *inst = otrGetInstance();
    if (otThreadGetDeviceRole(inst) != OT_DEVICE_ROLE_LEADER) return;

    static net_mgm_node_reply_t reply;
    if (nwk_node_parse(NET_MGM_NODE_REPLY_HEADER, data, lens, &reply)) return;

    char s[OT_IP6_ADDRESS_STRING_SIZE];
    otIp6AddressToString(&src_addr, s, sizeof(s));
    log_info("[Networking] << polling reply (%s)", s);

    /* 查 Leader 邻居表得到 Router 的 RSSI */
    int8_t self_rssi = -128;
    otNeighborInfo nbr;
    otNeighborInfoIterator iter = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    while (otThreadGetNextNeighborInfo(inst, &iter, &nbr) == OT_ERROR_NONE) {
        if (nbr.mRloc16 == reply.self_rloc) { self_rssi = nbr.mLastRssi; break; }
    }

    node_table_add(OT_DEVICE_ROLE_ROUTER, reply.parent, reply.self_rloc,
                   reply.self_extaddr, self_rssi, reply.version);

    /* 同步子节点表：移除不再出现在 reply 里的 child */
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (!s_node_table[i].used ||
            s_node_table[i].parent != reply.self_rloc ||
            s_node_table[i].role   != OT_DEVICE_ROLE_CHILD) continue;

        bool found = false;
        for (uint16_t k = 0; k < reply.num; k++) {
            if (memcmp(s_node_table[i].extaddr, reply.child_info[k].extaddr,
                       OT_EXT_ADDRESS_SIZE) == 0) { found = true; break; }
        }
        if (!found) {
            s_node_table[i].role        = OT_DEVICE_ROLE_DETACHED;
            s_node_table[i].survivaltime= 0;
        }
    }

    /* 更新 child 条目 */
    for (uint16_t i = 0; i < reply.num; i++) {
        node_table_add(OT_DEVICE_ROLE_CHILD, reply.self_rloc,
                       reply.child_info[i].rloc, reply.child_info[i].extaddr,
                       reply.child_info[i].rssi, 0xffffffff);
    }
}

/* Leader 收到新节点的入网申请 */
void app_udp_comm_net_mgm_node_challenge_proc(uint8_t *data, uint16_t lens,
                                               otIp6Address src_addr)
{
    net_mgm_node_challenge_t ch;
    if (!nwk_node_parse(NET_MGM_NODE_CHALLENGE_HEADER, data, lens, &ch)) {
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&src_addr, s, sizeof(s));
        log_info("[Networking] << challenge (%s)", s);
        int result = node_table_add(ch.role, ch.parent, ch.self_rloc,
                                    (uint8_t *)ch.self_extaddr, ch.rssi, ch.version);
        net_mgm_accepted_send(src_addr, result);
    }
}

/* 本机收到 accepted（FTD 有时也作为 child 存在） */
void app_udp_comm_net_mgm_node_accepted_proc(uint8_t *data, uint16_t lens,
                                              otIp6Address src_addr)
{
    net_mgm_node_accepted_t acc;
    if (!nwk_node_parse(NET_MGM_NODE_ACCEPTED_HEADER, data, lens, &acc)) {
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&src_addr, s, sizeof(s));
        log_info("[Networking] << accepted [%d] (%s)", acc.status, s);
        if (!acc.status) s_network_complete = true;
    }
}

/* 收到 reset 命令 */
void app_udp_comm_net_mgm_node_reset_proc(uint8_t *data, uint16_t lens,
                                           otIp6Address src_addr)
{
    net_mgm_node_reset_t rst;
    if (!nwk_node_parse(NET_MGM_NODE_RESET_HEADER, data, lens, &rst)) {
        log_info("[Networking] << reset %s",
                 rst.need_erase ? "need_erase" : "no_erase");
        s_reset_countdown  = 30;
        s_node_reset_erase = rst.need_erase;
        log_info("[mgm] reset in %u seconds", s_reset_countdown);
    }
}

/* -----------------------------------------------------------------------
 * 内部节点表维护
 * ----------------------------------------------------------------------- */
static void node_table_router_update(void)
{
    otInstance *inst    = otrGetInstance();
    uint8_t     max_rid = otThreadGetMaxRouterId(inst);
    otRouterInfo r_info;

    if (!s_node_table) return;
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (!s_node_table[i].used || s_node_table[i].role != OT_DEVICE_ROLE_ROUTER) continue;

        bool found = false;
        for (uint8_t j = 0; j <= max_rid; j++) {
            if (otThreadGetRouterInfo(inst, j, &r_info) == OT_ERROR_NONE &&
                memcmp(r_info.mExtAddress.m8, s_node_table[i].extaddr,
                       OT_EXT_ADDRESS_SIZE) == 0) {
                found = true; break;
            }
        }
        if (!found) {
            /* 该 Router 已离网，清除其子节点 */
            for (uint16_t k = 0; k < NET_MGM_NODE_TABLE_MAX; k++) {
                if (s_node_table[k].parent == s_node_table[i].rloc &&
                    s_node_table[k].role   == OT_DEVICE_ROLE_CHILD) {
                    s_node_table[k].role        = OT_DEVICE_ROLE_DETACHED;
                    s_node_table[k].survivaltime= 0;
                }
            }
            s_node_table[i].role        = OT_DEVICE_ROLE_DETACHED;
            s_node_table[i].survivaltime= 0;
        }
    }
}

static void node_table_child_update(void)
{
    otInstance *inst = otrGetInstance();
    uint16_t    max  = otThreadGetMaxAllowedChildren(inst);
    otChildInfo child;

    for (uint16_t i = 0; i < max; i++) {
        if (otThreadGetChildInfoByIndex(inst, i, &child) != OT_ERROR_NONE ||
            child.mIsStateRestoring) continue;
        node_table_add(OT_DEVICE_ROLE_CHILD,
                       otThreadGetRloc16(inst),
                       child.mRloc16,
                       (uint8_t *)child.mExtAddress.m8,
                       child.mAverageRssi,
                       0xffffffff);
    }
}

static void node_survival_tick(void)
{
    if (!s_node_table) return;
    for (uint16_t i = 0; i < NET_MGM_NODE_TABLE_MAX; i++) {
        if (s_node_table[i].used && s_node_table[i].survivaltime > 0) {
            if (--s_node_table[i].survivaltime == 0) {
                s_node_table[i].role = OT_DEVICE_ROLE_DETACHED;
                mgm_dbg("[mgm] node %04X expired", s_node_table[i].rloc);
            }
        }
    }
}

static void node_ask_router(void)
{
    otInstance  *inst     = otrGetInstance();
    uint8_t      max_rid  = otThreadGetMaxRouterId(inst);
    otRouterInfo r_info;
    bool         found_last = false;

    for (uint8_t i = 0; i <= max_rid; i++) {
        if (otThreadGetRouterInfo(inst, i, &r_info) != OT_ERROR_NONE) continue;

        if (r_info.mRloc16 == otThreadGetRloc16(inst)) {
            node_table_child_update();
            continue;
        }

        if (s_last_router_id == 0xff || found_last) {
            s_poll_countdown  = NET_MGM_POLL_SHORT_SEC;
            s_last_router_id  = r_info.mRouterId;
            net_mgm_ask_send(r_info.mRloc16);
            return;
        }
        if (r_info.mRouterId == s_last_router_id) {
            found_last = true;
        }
    }

    /* 轮完一圈 */
    s_poll_countdown = NET_MGM_POLL_LONG_SEC;
    s_last_router_id = 0xff;
    node_table_router_update();
}

/* -----------------------------------------------------------------------
 * 1-second 心跳任务（运行在 OT 任务上下文）
 * ----------------------------------------------------------------------- */
static void net_mgm_heartbeat_task(void *arg)
{
    otInstance *inst = otrGetInstance();

    if (net_mgm_check_leader_pin()) {
        /* Leader 轮询逻辑 */
        mgm_dbg("[mgm] leader poll countdown=%u", s_poll_countdown);
        if (s_poll_countdown > 0 && --s_poll_countdown == 0) {
            node_ask_router();
        }
        node_survival_tick();
    } else {
        /* 非 Leader：Challenge 定时 */
        if (!s_network_complete) {
            s_attach_timer++;
            app_set_led1_toggle();
        } else {
            app_set_led1_on();
        }

        if (s_challenge_countdown > 0 && --s_challenge_countdown == 0) {
            if (s_network_complete) {
                s_challenge_countdown = 0;
                s_challenge_counts    = 0;
            } else {
                s_challenge_counts++;
                if (s_challenge_counts > 5) {
                    /* 超过最大重试次数，重置 Thread 栈 */
                    s_challenge_counts = 0;
                    log_info("[mgm] challenge timeout × 5, reset Thread stack");
                    otThreadSetEnabled(inst, false);
                    otInstanceErasePersistentInfo(inst);
                    otThreadSetEnabled(inst, true);
                } else {
                    log_info("[Networking] challenge try %d", s_challenge_counts);
                    net_mgm_challenge_send();
                    s_challenge_countdown = NET_MGM_CHALLENGE_TIMEOUT_SEC;
                }
            }
        }
    }

    /* 复位倒计时 */
    if (s_reset_countdown != 0xffff) {
        if (s_reset_countdown == 0) {
            if (s_node_reset_erase) {
                otInstanceFactoryReset(inst);
            } else {
                otInstanceReset(inst);
            }
        } else {
            log_info("[mgm] reset countdown %u", s_reset_countdown);
            s_reset_countdown--;
        }
    }
}

/* FreeRTOS timer callback → 投递到 OT 任务 */
static void net_mgm_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ot_app_task_post(net_mgm_heartbeat_task, NULL);
}

/* -----------------------------------------------------------------------
 * 角色变化回调（在 OT 任务上下文，可直接调 OT API）
 * ----------------------------------------------------------------------- */
void net_mgm_interface_state_change(uint32_t aFlags, void *aContext)
{
    log_info("[mgm] state_change has runing");
    otInstance *inst = otrGetInstance();

    if (!(aFlags & OT_CHANGED_THREAD_ROLE)) return;

    otDeviceRole role = otThreadGetDeviceRole(inst);
    if (role == OT_DEVICE_ROLE_DISABLED) {
        log_info("[mgm] disabled");
        return;
    }

    /* 强制 Leader / 非 Leader 逻辑
     * 注意：不得调用 otDatasetSetActive —— 该函数会把仅含 ActiveTimestamp 的
     * 残缺数据集写回 Flash，覆盖 otdatasetInit() 写入的网络密钥/信道/PAN ID，
     * 导致其他节点无法再加入网络。数据集由 otdatasetInit() 统一管理。 */
    if (net_mgm_check_leader_pin()) {
        /* 本机应为 Leader */
        if (role != OT_DEVICE_ROLE_LEADER) {
            log_info("[mgm] force become leader");
            if (otThreadBecomeLeader(inst) != OT_ERROR_NONE) {
                log_info("[mgm] BecomeLeader fail, detach");
                otThreadBecomeDetached(inst);
            }
            return;
        }
    } else {
        /* 本机不应为 Leader（意外选举为 Leader 时退出，等待真正的 Leader）*/
        if (role == OT_DEVICE_ROLE_LEADER) {
            log_info("[mgm] not leader pin, detach");
            otThreadBecomeDetached(inst);
            return;
        }
    }

    switch (role) {
    case OT_DEVICE_ROLE_DETACHED:
        log_info("[mgm] detached");
        s_attach_timer        = 0;
        s_challenge_countdown = 0;
        s_network_complete    = false;
        break;
    case OT_DEVICE_ROLE_LEADER:
        log_info("[mgm] became leader");
        app_set_led1_on();
        s_poll_countdown = NET_MGM_POLL_SHORT_SEC;
        break;
    case OT_DEVICE_ROLE_ROUTER:
        log_info("[mgm] became router");
        s_network_complete = true;
        break;
    case OT_DEVICE_ROLE_CHILD:
        log_info("[mgm] became child, send challenge");
        s_network_complete    = false;
        s_challenge_countdown = NET_MGM_CHALLENGE_TIMEOUT_SEC;
        net_mgm_challenge_send();
        break;
    default:
        break;
    }

    /* 打印本机 IP 地址 */
    const otNetifAddress *addr;
    char s[OT_IP6_ADDRESS_STRING_SIZE];
    for (addr = otIp6GetUnicastAddresses(inst); addr; addr = addr->mNext) {
        otIp6AddressToString(&addr->mAddress, s, sizeof(s));
        log_info("  %s", s);
    }
}

/* -----------------------------------------------------------------------
 * net_mgm_init — 由 app_udp_comm_init() → otrInitUser() 调用
 * ----------------------------------------------------------------------- */
void net_mgm_init(void)
{
    log_info("[mgm] Centrak network management init");

    /* 初始化 Leader 设备表与 PC UART0 桥接 */
    if (net_mgm_check_leader_pin()) {
        app_device_table_init();
        app_uart_pc_init();
    }

    /* 配置 Leader 检测引脚为输入（无中断，轮询电平）
     * 必须使能内部上拉：悬空时读高（非 Leader），接地时读低（是 Leader）。
     * 不加上拉会导致悬空引脚读到 0，设备总是选举为 Leader。 */
    {
        hosal_gpio_input_config_t pin_cfg = {
            .usr_cb       = NULL,
            .param        = NULL,
            .pin_int_mode = HOSAL_GPIO_PIN_NOINT,
        };
        hosal_gpio_cfg_input(NET_MGM_LEADER_GPIO_PIN, pin_cfg);
        hosal_pin_set_pullopt(NET_MGM_LEADER_GPIO_PIN, HOSAL_PULL_UP_100K);
    }

    /* 创建 1s 心跳定时器 */
    if (!s_mgm_timer) {
        s_mgm_timer = xTimerCreate("mgm_tick",
                                    pdMS_TO_TICKS(1000),
                                    pdTRUE,   /* auto-reload */
                                    NULL,
                                    net_mgm_timer_cb);
        if (s_mgm_timer) {
            xTimerStart(s_mgm_timer, 0);
        } else {
            log_warn("[mgm] timer create fail");
        }
    }

    /* 加速路由器选举 */
    otThreadSetRouterSelectionJitter(otrGetInstance(), 5);

    /* 注册角色变化回调（覆盖 otrInitUser 里的默认 callback） */
    otSetStateChangedCallback(otrGetInstance(), net_mgm_interface_state_change, NULL);

    /* Leader 初始化：分配节点表、从 Flash 恢复/递增 Partition ID */
    if (net_mgm_check_leader_pin() && !s_node_table) {
        s_node_table = (net_mgm_node_table_t *)pvPortMalloc(
            sizeof(net_mgm_node_table_t) * NET_MGM_NODE_TABLE_MAX);
        if (s_node_table) {
            memset(s_node_table, 0,
                   sizeof(net_mgm_node_table_t) * NET_MGM_NODE_TABLE_MAX);
        }

        /* 从 EnhancedFlashDataset 读取并递增 Partition ID，确保跨重启分区唯一。
         * key = "centrak_pid"，值为 uint32_t（4 字节 blob）。
         * enhanced_flash_dataset_init() 由框架在 miuStart() 中完成，此处直接读写。 */
        uint32_t partition_id = 0;
        size_t   saved_len   = 0;
        efd_get_env_blob("centrak_pid", &partition_id, sizeof(uint32_t), &saved_len);
        if (saved_len != sizeof(uint32_t)) {
            partition_id = 0;  /* 首次启动或数据损坏，从 0 开始 */
        }
        partition_id++;
        if (efd_set_env_blob("centrak_pid", &partition_id, sizeof(uint32_t)) != EFD_NO_ERR) {
            log_warn("[mgm] partition ID write fail");
        }

        otThreadSetPreferredLeaderPartitionId(otrGetInstance(), partition_id);
        otThreadSetPreferredRouterId(otrGetInstance(), 0);
        otThreadSetLocalLeaderWeight(otrGetInstance(), 128);

        if (otThreadBecomeLeader(otrGetInstance()) != OT_ERROR_NONE) {
            log_warn("[mgm] BecomeLeader fail on init");
        }

        s_poll_countdown = NET_MGM_POLL_SHORT_SEC;
        log_info("[mgm] leader mode, partition_id=%u", (unsigned)partition_id);
    }

    /* 启动 Thread 协议栈（替代 otrInitUser 末尾的 otThreadSetEnabled） */
    if (otIp6SetEnabled(otrGetInstance(), true) != OT_ERROR_NONE) {
        log_warn("[mgm] otIp6SetEnabled fail");
    }
    if (otThreadSetEnabled(otrGetInstance(), true) != OT_ERROR_NONE) {
        log_warn("[mgm] otThreadSetEnabled fail");
    }
}

#endif /* CFG_USE_CENTRAK_CONFIG */
