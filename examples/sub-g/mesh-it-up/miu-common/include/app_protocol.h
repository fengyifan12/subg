/*
 * app_protocol.h — 自定义 UDP 应用层协议定义
 *
 * 包含 Centrak 网管、控制、路径追踪、雷达上报等所有私有协议的
 * magic header 常量、公共数据结构及跨模块函数声明。
 * FTD 和 MTD 均包含此文件。
 */

#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <openthread/thread.h>

/* -----------------------------------------------------------------------
 * 全局开关
 * ----------------------------------------------------------------------- */
#define CFG_USE_CENTRAK_CONFIG  1   /* 1 = 启用 Centrak 集中式网管协议 */

/* -----------------------------------------------------------------------
 * UDP 协议 Magic Header（小端 uint32_t 前 4 字节）
 *   高位半字 0x8888 = 私有应用层标识
 *   低位半字区分子命令
 * ----------------------------------------------------------------------- */

/* 路径追踪 */
#define PATH_REQUEST_HEADER             0x88880080U
#define PATH_RESPOND_HEADER             0x88888081U

/* Centrak 网络管理 */
#define NET_MGM_NODE_ASK_HEADER         0x88880082U  /* Leader → Router：请求上报子节点表 */
#define NET_MGM_NODE_REPLY_HEADER       0x88888083U  /* Router → Leader：子节点表回复 */
#define NET_MGM_NODE_CHALLENGE_HEADER   0x88880084U  /* 新节点 → Leader：入网申请 */
#define NET_MGM_NODE_ACCEPTED_HEADER    0x88888085U  /* Leader → 新节点：入网许可 */
#define NET_MGM_NODE_RESET_HEADER       0x88888086U  /* Leader → 节点：复位指令 */

/* 雷达上报（MTD 专属） */
#define RADAR_REPORT_HEADER             0x888800A0U  /* MTD → Leader：人体存在状态 */

/* 节点控制（Leader → 节点） */
#define CTRL_NODE_DETACH_HEADER         0x88880090U  /* 命令节点离线 */
#define CTRL_NODE_WHITELIST_HEADER      0x88880091U  /* 命令节点绑定白名单 parent */
#define CTRL_NODE_CHILD_TIMEOUT_HEADER  0x88880092U  /* 设置 child timeout（广播） */
#define CTRL_NODE_ACK_HEADER            0x88888093U  /* 节点 → Leader：命令回执 */
#define CTRL_NODE_REATTACH_HEADER       0x88880094U  /* 命令节点重新入网 */

/* 公用掩码：低 16 bit 为 0x0000 的均属于私有应用层协议 */
#define UDP_APP_HEADER_MASK             0xFFFF0000U
#define UDP_APP_HEADER_MAGIC            0x88880000U

/* -----------------------------------------------------------------------
 * 公共数据结构
 * ----------------------------------------------------------------------- */

/* 雷达上报帧 */
typedef struct {
    uint32_t header;    /* RADAR_REPORT_HEADER */
    uint8_t  seq;
    uint8_t  status;    /* 0 = 无人，1 = 有人 */
} __attribute__((packed)) radar_report_t;

/* -----------------------------------------------------------------------
 * app_udp_comm.c 公共接口
 * ----------------------------------------------------------------------- */
void app_udp_comm_init(void);
void app_udp_comm_process(uint8_t *data, uint16_t lens, otIp6Address src_addr);
bool app_udp_comm_header_check(uint8_t *head, uint8_t lens);

/**
 * JSON 私有协议入口（UDP payload 以 '{' 开头时调用）。
 * FTD：解析 REGISTER/REPORT/ACK，更新设备表，转发到 PC UART0。
 * MTD：接收 CONTROL/ACK 并执行（各产品固件自行实现业务逻辑）。
 */
void app_udp_comm_json_process(uint8_t *data, uint16_t lens, otIp6Address src_addr);

/* -----------------------------------------------------------------------
 * app_path.c 公共接口
 * ----------------------------------------------------------------------- */
void path_init(void);
void app_udp_comm_path_req_proc(uint8_t *data, uint16_t lens);
void app_udp_comm_path_resp_proc(uint8_t *data, uint16_t lens);
otError path_req_send(otIp6Address dst_addr, uint32_t timeout_ms);

/* -----------------------------------------------------------------------
 * app_net_mgm.c 公共接口（FTD 与 MTD 各自实现，接口相同）
 * ----------------------------------------------------------------------- */
void net_mgm_init(void);

/* FTD Leader 专属 */
void net_mgm_node_table_display(void);
bool net_mgm_node_table_find(uint8_t *aExtAddress);
uint16_t net_mgm_get_children_of_router(uint16_t router_rloc,
                                         uint16_t *rloc_list,
                                         uint16_t max_count);
int  net_mgm_node_iterate(int start_idx, uint16_t *out_rloc);
bool net_mgm_rloc_by_iid(const uint8_t iid[8], uint16_t *out_rloc);
void net_mgm_invalidate_all_children(void);
void net_mgm_node_reset_send(bool is_need_erase);

/* UDP 分发回调（被 app_udp_comm_process 调用） */
void app_udp_comm_net_mgm_node_ack_proc(uint8_t *data, uint16_t lens,
                                          otIp6Address src_addr);   /* FTD Router 响应 Leader 轮询 */
void app_udp_comm_net_mgm_node_reply_proc(uint8_t *data, uint16_t lens,
                                           otIp6Address src_addr);
void app_udp_comm_net_mgm_node_challenge_proc(uint8_t *data, uint16_t lens,
                                               otIp6Address src_addr);
void app_udp_comm_net_mgm_node_accepted_proc(uint8_t *data, uint16_t lens,
                                              otIp6Address src_addr);
void app_udp_comm_net_mgm_node_reset_proc(uint8_t *data, uint16_t lens,
                                           otIp6Address src_addr);

#endif /* __APP_PROTOCOL_H */
