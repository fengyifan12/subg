/*
 * app_path.c — 多跳路径追踪
 *
 * 移植自独立工程 app_path.c，适配 mesh-it-up FreeRTOS 框架：
 *   - otGetInstance()    → otrGetInstance()
 *   - sw_timer_*         → FreeRTOS xTimer*
 *   - sw_timer_get_tick()→ xTaskGetTickCount()（configTICK_RATE_HZ=1000 时单位相同）
 *   - app_udp_send()     → app_udpSend(..., false)
 *   - info()             → log_info()
 *   - mem_malloc/free    → pvPortMalloc / vPortFree
 */

#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <miu_port.h>
#include <openthread/thread.h>
#include <openthread/thread_ftd.h>
#include <openthread/ip6.h>
#include "log.h"
#include "app_protocol.h"
#include "app_udp.h"

/* -----------------------------------------------------------------------
 * 数据结构
 * ----------------------------------------------------------------------- */
#define PATH_HOP_LIMIT  31

typedef struct {
    uint8_t  path_iid[8];
    uint32_t recv_time;
    uint16_t path_rloc16;
    int8_t   recv_rssi;
} __attribute__((packed)) path_info_t;

typedef struct {
    uint32_t    header;
    uint8_t     ttl;
    uint16_t    src_rloc16;
    uint16_t    dst_rloc16;
    uint8_t     dst_iid[8];
    path_info_t path_info[PATH_HOP_LIMIT];
} __attribute__((packed)) path_request_t;

typedef struct {
    uint32_t    header;
    uint8_t     ttl;
    uint16_t    src_rloc16;
    uint16_t    dst_rloc16;
    path_info_t path_info[PATH_HOP_LIMIT];
} __attribute__((packed)) path_respond_t;

/* -----------------------------------------------------------------------
 * 模块状态
 * ----------------------------------------------------------------------- */
static TimerHandle_t     s_path_req_timer   = NULL;
static volatile uint32_t s_path_req_ts_ms   = 0;  /* 发送 path req 时的时间戳 (ms) */

/* -----------------------------------------------------------------------
 * 工具函数
 * ----------------------------------------------------------------------- */

static int8_t path_find_rloc16_last_rssi(uint16_t rloc16)
{
    int8_t rssi = -128, parent_rssi;
    otNeighborInfo neighborInfo;
    otNeighborInfoIterator iterator = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    otInstance *inst = otrGetInstance();

    while (otThreadGetNextNeighborInfo(inst, &iterator, &neighborInfo) == OT_ERROR_NONE) {
        if (neighborInfo.mRloc16 == rloc16) {
            rssi = neighborInfo.mLastRssi;
            break;
        }
    }
    if (rssi == -128 &&
        otThreadGetParentLastRssi(inst, &parent_rssi) == OT_ERROR_NONE) {
        rssi = parent_rssi;
    }
    return rssi;
}

static uint16_t path_find_dst_ip_rloc16(otIp6Address dst_addr)
{
    uint16_t rloc16 = 0xffff;
    otCacheEntryIterator iter;
    otCacheEntryInfo     eid_entry;
    otNeighborInfo       neighborInfo;
    otNeighborInfoIterator iterator = OT_NEIGHBOR_INFO_ITERATOR_INIT;
    otInstance *inst = otrGetInstance();
    const otIp6Address *rloc_ip = otThreadGetRloc(inst);

    memset(&iter, 0, sizeof(iter));

    do {
        /* 目标是本机 RLOC 地址 */
        if (memcmp(&dst_addr.mFields.m8[8], &rloc_ip->mFields.m8[8], 6) == 0) {
            rloc16 = (uint16_t)((dst_addr.mFields.m8[14] << 8) | dst_addr.mFields.m8[15]);
            break;
        }
        /* 查邻居表 */
        while (otThreadGetNextNeighborInfo(inst, &iterator, &neighborInfo) == OT_ERROR_NONE) {
            if (memcmp(neighborInfo.mExtAddress.m8, &dst_addr.mFields.m8[8], 8) == 0) {
                rloc16 = neighborInfo.mRloc16;
                break;
            }
        }
        if (rloc16 != 0xffff) break;
        /* 查 EID→RLOC 缓存 */
        while (otThreadGetNextCacheEntry(inst, &eid_entry, &iter) == OT_ERROR_NONE) {
            if (memcmp(eid_entry.mTarget.mFields.m8, dst_addr.mFields.m8,
                       OT_IP6_ADDRESS_SIZE) == 0) {
                rloc16 = eid_entry.mRloc16;
                break;
            }
        }
    } while (0);

    return rloc16;
}

static otError path_find_next_hop_ip(uint16_t dst_rloc16, otIp6Address *next_ip)
{
    otError error = OT_ERROR_NOT_FOUND;
    otInstance *inst = otrGetInstance();
    const otIp6Address *rloc_ip = otThreadGetRloc(inst);
    bool is_neighbor = false;
    otRouterInfo parentInfo;
    otNeighborInfo neighborInfo;
    otNeighborInfoIterator iterator = OT_NEIGHBOR_INFO_ITERATOR_INIT;
#if OPENTHREAD_FTD
    uint8_t r_max_id = 0;
    otRouterInfo r_info, n_info;
#endif
    otDeviceRole role = otThreadGetDeviceRole(inst);

    memcpy(next_ip->mFields.m8, rloc_ip->mFields.m8, OT_IP6_ADDRESS_SIZE);

    do {
        /* Child：直接上报给 parent */
        if (role == OT_DEVICE_ROLE_CHILD) {
            error = otThreadGetParentInfo(inst, &parentInfo);
            next_ip->mFields.m8[14] = (uint8_t)((parentInfo.mRloc16 & 0xff00) >> 8);
            next_ip->mFields.m8[15] = (uint8_t)(parentInfo.mRloc16 & 0x00ff);
            break;
        }
        /* MTD（非 FTD）：转发给 parent */
        if (!otThreadGetLinkMode(inst).mDeviceType) {
            otRouterInfo parent;
            if (otThreadGetParentInfo(inst, &parent) == OT_ERROR_NONE) {
                next_ip->mFields.m8[14] = (uint8_t)((parent.mRloc16 & 0xff00) >> 8);
                next_ip->mFields.m8[15] = (uint8_t)(parent.mRloc16 & 0x00ff);
                error = OT_ERROR_NONE;
            }
            break;
        }
        /* FTD：查直连邻居 */
        while (otThreadGetNextNeighborInfo(inst, &iterator, &neighborInfo) == OT_ERROR_NONE) {
            if (neighborInfo.mRloc16 == dst_rloc16) {
                next_ip->mFields.m8[14] = (uint8_t)((neighborInfo.mRloc16 & 0xff00) >> 8);
                next_ip->mFields.m8[15] = (uint8_t)(neighborInfo.mRloc16 & 0x00ff);
                error = OT_ERROR_NONE;
                is_neighbor = true;
                break;
            }
        }
        if (is_neighbor) break;
#if OPENTHREAD_FTD
        /* FTD：查路由表 */
        r_max_id = otThreadGetMaxRouterId(inst);
        for (uint8_t i = 0; i <= r_max_id; i++) {
            if (otThreadGetRouterInfo(inst, i, &r_info) != OT_ERROR_NONE) continue;
            if (r_info.mRloc16 != (dst_rloc16 & 0xFC00)) continue;

            if (!r_info.mLinkEstablished) {
                /* 非直连路由，查 next hop */
                for (uint8_t k = 0; k <= r_max_id; k++) {
                    if (otThreadGetRouterInfo(inst, k, &n_info) == OT_ERROR_NONE &&
                        r_info.mNextHop == n_info.mRouterId) {
                        next_ip->mFields.m8[14] = (uint8_t)((n_info.mRloc16 & 0xff00) >> 8);
                        next_ip->mFields.m8[15] = (uint8_t)(n_info.mRloc16 & 0x00ff);
                        error = OT_ERROR_NONE;
                        break;
                    }
                }
            } else {
                next_ip->mFields.m8[14] = (uint8_t)((r_info.mRloc16 & 0xff00) >> 8);
                next_ip->mFields.m8[15] = (uint8_t)(r_info.mRloc16 & 0x00ff);
            }
            error = OT_ERROR_NONE;
            break;
        }
#endif /* OPENTHREAD_FTD */
    } while (0);

    return error;
}

/* -----------------------------------------------------------------------
 * 序列化 / 反序列化
 * ----------------------------------------------------------------------- */

static int path_req_parse(uint8_t *payload, uint16_t len, path_request_t *out)
{
    uint8_t *p = payload;
    memcpy(&out->header, p, 4); p += 4;
    out->ttl = *p++;
    memcpy(&out->src_rloc16, p, 2); p += 2;
    memcpy(&out->dst_rloc16, p, 2); p += 2;
    memcpy(out->dst_iid, p, 8);     p += 8;
    uint16_t info_bytes = (uint16_t)((PATH_HOP_LIMIT - out->ttl) * sizeof(path_info_t));
    memcpy(out->path_info, p, info_bytes); p += info_bytes;
    if ((p - payload) != len) {
        log_info("[path] req parse fail (%u/%u)", (unsigned)(p - payload), len);
        return 1;
    }
    return 0;
}

static int path_resp_parse(uint8_t *payload, uint16_t len, path_respond_t *out)
{
    uint8_t *p = payload;
    memcpy(&out->header, p, 4); p += 4;
    out->ttl = *p++;
    memcpy(&out->src_rloc16, p, 2); p += 2;
    memcpy(&out->dst_rloc16, p, 2); p += 2;
    uint16_t info_bytes = (uint16_t)((PATH_HOP_LIMIT - out->ttl) * sizeof(path_info_t));
    memcpy(out->path_info, p, info_bytes); p += info_bytes;
    if ((p - payload) != len) {
        log_info("[path] resp parse fail (%u/%u)", (unsigned)(p - payload), len);
        return 1;
    }
    return 0;
}

static void path_req_pack(uint8_t *payload, uint16_t *out_len, path_request_t *req)
{
    uint8_t *p = payload;
    memcpy(p, &req->header, 4); p += 4;
    *p++ = req->ttl;
    memcpy(p, &req->src_rloc16, 2); p += 2;
    memcpy(p, &req->dst_rloc16, 2); p += 2;
    memcpy(p, req->dst_iid, 8);     p += 8;
    uint16_t info_bytes = (uint16_t)((PATH_HOP_LIMIT - req->ttl) * sizeof(path_info_t));
    memcpy(p, req->path_info, info_bytes); p += info_bytes;
    *out_len = (uint16_t)(p - payload);
}

static void path_resp_pack(uint8_t *payload, uint16_t *out_len, path_respond_t *resp)
{
    uint8_t *p = payload;
    memcpy(p, &resp->header, 4); p += 4;
    *p++ = resp->ttl;
    memcpy(p, &resp->src_rloc16, 2); p += 2;
    memcpy(p, &resp->dst_rloc16, 2); p += 2;
    uint16_t info_bytes = (uint16_t)((PATH_HOP_LIMIT - resp->ttl) * sizeof(path_info_t));
    memcpy(p, resp->path_info, info_bytes); p += info_bytes;
    *out_len = (uint16_t)(p - payload);
}

/* -----------------------------------------------------------------------
 * 定时器回调（FreeRTOS timer task 上下文，无需 OT 锁）
 * ----------------------------------------------------------------------- */
static void path_req_timeout_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    log_info("[path] request timeout");
    s_path_req_ts_ms = 0;
}

/* -----------------------------------------------------------------------
 * path_req_send — CLI 触发，在 OT 任务上下文中调用
 * ----------------------------------------------------------------------- */
otError path_req_send(otIp6Address dst_addr, uint32_t timeout_ms)
{
    otError error = OT_ERROR_NONE;
    otInstance *inst = otrGetInstance();
    const otMeshLocalPrefix *mesh_prefix = otThreadGetMeshLocalPrefix(inst);

    if (xTimerIsTimerActive(s_path_req_timer)) {
        return OT_ERROR_ALREADY;
    }

    uint8_t *buf = pvPortMalloc(sizeof(path_request_t));
    if (!buf) {
        log_info("[path] req send: malloc fail");
        return OT_ERROR_NO_BUFS;
    }

    do {
        path_request_t path_req;
        otIp6Address   next_ip;
        uint16_t       buf_len = 0;

        path_req.header     = PATH_REQUEST_HEADER;
        path_req.ttl        = PATH_HOP_LIMIT;
        path_req.src_rloc16 = otThreadGetRloc16(inst);
        path_req.dst_rloc16 = path_find_dst_ip_rloc16(dst_addr);
        memcpy(path_req.dst_iid, &dst_addr.mFields.m8[8], 8);

        if (path_req.dst_rloc16 == 0xffff) {
            log_info("[path] req send: dst RLOC not found");
            error = OT_ERROR_NOT_FOUND;
            break;
        }

        /* 判断是否 mesh 本地前缀，需要 next-hop 路由 */
        if (memcmp(mesh_prefix->m8, dst_addr.mFields.m8, OT_IP6_ADDRESS_SIZE / 2) == 0) {
            if (path_find_next_hop_ip(path_req.dst_rloc16, &next_ip) != OT_ERROR_NONE) {
                log_info("[path] req send: next hop not found");
                error = OT_ERROR_NOT_FOUND;
                break;
            }
            memcpy(dst_addr.mFields.m8, next_ip.mFields.m8, OT_IP6_ADDRESS_SIZE);
        }

        path_req_pack(buf, &buf_len, &path_req);
        log_info("[path] req send → %04X", path_req.dst_rloc16);

        if (app_udpSend(dst_addr, buf, buf_len, false) == 0) {
            /* 启动超时定时器 */
            xTimerChangePeriod(s_path_req_timer, pdMS_TO_TICKS(timeout_ms), 0);
            xTimerStart(s_path_req_timer, 0);
            s_path_req_ts_ms = (uint32_t)xTaskGetTickCount();
        } else {
            log_info("[path] req send: UDP send fail");
        }
    } while (0);

    vPortFree(buf);
    return error;
}

/* -----------------------------------------------------------------------
 * app_udp_comm_path_req_proc — 收到 PATH_REQUEST，转发或回复
 * ----------------------------------------------------------------------- */
void app_udp_comm_path_req_proc(uint8_t *data, uint16_t lens)
{
    path_request_t req, new_req;
    path_respond_t resp;
    otIp6Address   next_ip;
    otInstance    *inst   = otrGetInstance();
    const otIp6Address *rloc_ip    = otThreadGetRloc(inst);
    const otIp6Address *mesh_ip    = otThreadGetMeshLocalEid(inst);
    uint8_t  *buf     = NULL;
    uint16_t  buf_len = 0;
    uint16_t  my_rloc = otThreadGetRloc16(inst);

    if (path_req_parse(data, lens, &req)) return;
    if (req.ttl == 0) {
        log_info("[path] req hop exhausted");
        return;
    }

    log_info("[path] req [ttl=%u] %04X→%04X", req.ttl, req.src_rloc16, req.dst_rloc16);

    if (my_rloc == req.src_rloc16) {
        log_info("[path] req: src is me, ignore");
        return;
    }

    uint8_t idx = PATH_HOP_LIMIT - req.ttl;  /* 当前跳在 path_info 里的写入位置 */

    if (my_rloc == req.dst_rloc16) {
        bool is_me = ((req.dst_iid[6] << 8 | req.dst_iid[7]) == req.dst_rloc16) ||
                     (memcmp(req.dst_iid, &mesh_ip->mFields.m8[8], 8) == 0);

        if (is_me) {
            /* 目的地到达：构造 RESPOND 回包 */
            resp.header     = PATH_RESPOND_HEADER;
            resp.ttl        = req.ttl - 1;
            resp.src_rloc16 = req.dst_rloc16;
            resp.dst_rloc16 = req.src_rloc16;
            memcpy(resp.path_info, req.path_info, idx * sizeof(path_info_t));
            memcpy(resp.path_info[idx].path_iid, &mesh_ip->mFields.m8[8], 8);
            resp.path_info[idx].recv_time   = (uint32_t)xTaskGetTickCount();
            resp.path_info[idx].path_rloc16 = my_rloc;
            resp.path_info[idx].recv_rssi   = (idx == 0)
                ? path_find_rloc16_last_rssi(req.src_rloc16)
                : path_find_rloc16_last_rssi(req.path_info[idx - 1].path_rloc16);

            buf = pvPortMalloc(sizeof(path_respond_t));
            if (buf) {
                path_resp_pack(buf, &buf_len, &resp);
                if (path_find_next_hop_ip(resp.dst_rloc16, &next_ip) == OT_ERROR_NONE) {
                    if (app_udpSend(next_ip, buf, buf_len, false)) {
                        log_info("[path] resp send fail");
                    }
                } else {
                    log_info("[path] resp: route not found");
                }
                vPortFree(buf);
            }
        } else {
#if OPENTHREAD_FTD
            /* 目标 RLOC 是我，但 IID 不匹配 → 转发给子节点 */
            uint16_t max_child = otThreadGetMaxAllowedChildren(inst);
            otChildInfo child_info;
            for (uint16_t i = 0; i < max_child; i++) {
                if (otThreadGetChildInfoByIndex(inst, i, &child_info) != OT_ERROR_NONE) continue;
                if (memcmp(child_info.mExtAddress.m8, req.dst_iid, 8) != 0) continue;

                new_req.header     = PATH_REQUEST_HEADER;
                new_req.src_rloc16 = req.src_rloc16;
                new_req.dst_rloc16 = child_info.mRloc16;
                new_req.ttl        = req.ttl - 1;
                memcpy(new_req.dst_iid, req.dst_iid, 8);
                memcpy(new_req.path_info, req.path_info, idx * sizeof(path_info_t));
                memcpy(new_req.path_info[idx].path_iid, &mesh_ip->mFields.m8[8], 8);
                new_req.path_info[idx].recv_time   = (uint32_t)xTaskGetTickCount();
                new_req.path_info[idx].path_rloc16 = my_rloc;
                new_req.path_info[idx].recv_rssi   = (idx == 0)
                    ? path_find_rloc16_last_rssi(req.src_rloc16)
                    : path_find_rloc16_last_rssi(req.path_info[idx - 1].path_rloc16);

                buf = pvPortMalloc(sizeof(path_request_t));
                if (buf) {
                    path_req_pack(buf, &buf_len, &new_req);
                    memcpy(next_ip.mFields.m8, rloc_ip->mFields.m8, OT_IP6_ADDRESS_SIZE);
                    next_ip.mFields.m8[14] = (uint8_t)((child_info.mRloc16 & 0xff00) >> 8);
                    next_ip.mFields.m8[15] = (uint8_t)(child_info.mRloc16 & 0x00ff);
                    if (app_udpSend(next_ip, buf, buf_len, false)) {
                        log_info("[path] req fwd to child fail");
                    }
                    vPortFree(buf);
                }
                break;
            }
#else
            log_info("[path] req: dst RLOC is me but IID mismatch (MTD has no children)");
#endif /* OPENTHREAD_FTD */
        }
    } else {
        /* 不是目的地：转发给下一跳 */
        if (path_find_next_hop_ip(req.dst_rloc16, &next_ip) != OT_ERROR_NONE) {
            log_info("[path] req fwd: route not found");
            return;
        }
        new_req.header     = PATH_REQUEST_HEADER;
        new_req.src_rloc16 = req.src_rloc16;
        new_req.dst_rloc16 = req.dst_rloc16;
        new_req.ttl        = req.ttl - 1;
        memcpy(new_req.dst_iid, req.dst_iid, 8);
        memcpy(new_req.path_info, req.path_info, idx * sizeof(path_info_t));
        memcpy(new_req.path_info[idx].path_iid, &mesh_ip->mFields.m8[8], 8);
        new_req.path_info[idx].recv_time   = (uint32_t)xTaskGetTickCount();
        new_req.path_info[idx].path_rloc16 = my_rloc;
        new_req.path_info[idx].recv_rssi   = (idx == 0)
            ? path_find_rloc16_last_rssi(req.src_rloc16)
            : path_find_rloc16_last_rssi(req.path_info[idx - 1].path_rloc16);

        buf = pvPortMalloc(sizeof(path_request_t));
        if (buf) {
            path_req_pack(buf, &buf_len, &new_req);
            if (app_udpSend(next_ip, buf, buf_len, false)) {
                log_info("[path] req fwd fail");
            }
            vPortFree(buf);
        }
    }
}

/* -----------------------------------------------------------------------
 * app_udp_comm_path_resp_proc — 收到 PATH_RESPOND，打印或转发
 * ----------------------------------------------------------------------- */
void app_udp_comm_path_resp_proc(uint8_t *data, uint16_t lens)
{
    path_respond_t resp, new_resp;
    otIp6Address   next_ip;
    otInstance    *inst   = otrGetInstance();
    const otIp6Address *rloc_ip = otThreadGetRloc(inst);
    const otIp6Address *mesh_ip = otThreadGetMeshLocalEid(inst);
    const otMeshLocalPrefix *mesh_prefix = otThreadGetMeshLocalPrefix(inst);
    uint8_t  *buf     = NULL;
    uint16_t  buf_len = 0;
    uint16_t  my_rloc = otThreadGetRloc16(inst);

    if (path_resp_parse(data, lens, &resp)) return;
    if (resp.ttl == 0) {
        log_info("[path] resp hop exhausted");
        return;
    }

    log_info("[path] resp [ttl=%u] %04X→%04X", resp.ttl, resp.src_rloc16, resp.dst_rloc16);

    if (my_rloc == resp.src_rloc16) {
        log_info("[path] resp: src is me, ignore");
        return;
    }

    uint8_t total_hops = PATH_HOP_LIMIT - resp.ttl;

    if (my_rloc == resp.dst_rloc16) {
        /* 路径结果到达发起方，打印 RTT 和逐跳信息 */
        uint32_t rtt_ms = (uint32_t)xTaskGetTickCount() - s_path_req_ts_ms;
        log_info("[path] RTT: %lu ms  ttl_remain=%u/%u", (unsigned long)rtt_ms, resp.ttl, PATH_HOP_LIMIT);

        log_info("[s] %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x (%04X)",
                 mesh_ip->mFields.m8[0], mesh_ip->mFields.m8[1],
                 mesh_ip->mFields.m8[2], mesh_ip->mFields.m8[3],
                 mesh_ip->mFields.m8[4], mesh_ip->mFields.m8[5],
                 mesh_ip->mFields.m8[6], mesh_ip->mFields.m8[7],
                 mesh_ip->mFields.m8[8], mesh_ip->mFields.m8[9],
                 mesh_ip->mFields.m8[10], mesh_ip->mFields.m8[11],
                 mesh_ip->mFields.m8[12], mesh_ip->mFields.m8[13],
                 mesh_ip->mFields.m8[14], mesh_ip->mFields.m8[15],
                 my_rloc);

        for (int i = 0; i < total_hops; i++) {
            log_info("[%d] %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x (%04X) rssi=%d",
                     i,
                     mesh_prefix->m8[0], mesh_prefix->m8[1],
                     mesh_prefix->m8[2], mesh_prefix->m8[3],
                     mesh_prefix->m8[4], mesh_prefix->m8[5],
                     mesh_prefix->m8[6], mesh_prefix->m8[7],
                     resp.path_info[i].path_iid[0], resp.path_info[i].path_iid[1],
                     resp.path_info[i].path_iid[2], resp.path_info[i].path_iid[3],
                     resp.path_info[i].path_iid[4], resp.path_info[i].path_iid[5],
                     resp.path_info[i].path_iid[6], resp.path_info[i].path_iid[7],
                     resp.path_info[i].path_rloc16,
                     resp.path_info[i].recv_rssi);
        }

        /* 计算逐跳 RTT（往返折半） */
        uint32_t path_cost[PATH_HOP_LIMIT / 2 + 1];
        path_cost[0] = rtt_ms;
        int half = total_hops / 2;
        for (int i = 0; i < half; i++) {
            for (int j = total_hops; j > half; j--) {
                if (memcmp(resp.path_info[i].path_iid, resp.path_info[j].path_iid, 8) == 0) {
                    path_cost[i + 1] = resp.path_info[j].recv_time - resp.path_info[i].recv_time;
                    break;
                }
            }
        }
        log_info("[s]->[0] RTT: %lu ms", (unsigned long)((path_cost[0] - (half > 0 ? path_cost[1] : 0)) / 2));
        for (int i = 0; i < half; i++) {
            if (i == half - 1) {
                log_info("[%d]->[%d] RTT: %lu ms", i, i + 1, (unsigned long)(path_cost[i + 1] / 2));
            } else {
                log_info("[%d]->[%d] RTT: %lu ms", i, i + 1,
                         (unsigned long)((path_cost[i + 1] - path_cost[i + 2]) / 2));
            }
        }

        if (xTimerIsTimerActive(s_path_req_timer)) {
            xTimerStop(s_path_req_timer, 0);
            s_path_req_ts_ms = 0;
        }
    } else {
        /* 转发 RESPOND 到下一跳 */
        if (path_find_next_hop_ip(resp.dst_rloc16, &next_ip) != OT_ERROR_NONE) {
            log_info("[path] resp fwd: route not found");
            return;
        }
        uint8_t idx = total_hops;
        new_resp.header     = PATH_RESPOND_HEADER;
        new_resp.ttl        = resp.ttl - 1;
        new_resp.src_rloc16 = resp.src_rloc16;
        new_resp.dst_rloc16 = resp.dst_rloc16;
        memcpy(new_resp.path_info, resp.path_info, idx * sizeof(path_info_t));
        memcpy(new_resp.path_info[idx].path_iid, &mesh_ip->mFields.m8[8], 8);
        new_resp.path_info[idx].recv_time   = (uint32_t)xTaskGetTickCount();
        new_resp.path_info[idx].path_rloc16 = my_rloc;
        new_resp.path_info[idx].recv_rssi   = (idx == 0)
            ? path_find_rloc16_last_rssi(resp.dst_rloc16)
            : path_find_rloc16_last_rssi(resp.path_info[idx - 1].path_rloc16);

        buf = pvPortMalloc(sizeof(path_respond_t));
        if (buf) {
            path_resp_pack(buf, &buf_len, &new_resp);
            if (app_udpSend(next_ip, buf, buf_len, false)) {
                log_info("[path] resp fwd fail");
            }
            vPortFree(buf);
        }
    }
    (void)rloc_ip;
}

/* -----------------------------------------------------------------------
 * path_init — 由 app_udp_comm_init() 调用
 * ----------------------------------------------------------------------- */
void path_init(void)
{
    s_path_req_timer = xTimerCreate(
        "path_req",
        pdMS_TO_TICKS(30000),   /* 默认最大超时 30s，发送时会用 xTimerChangePeriod 覆盖 */
        pdFALSE,                /* one-shot */
        NULL,
        path_req_timeout_cb);

    if (!s_path_req_timer) {
        log_warn("[path] timer create fail");
    }
}
