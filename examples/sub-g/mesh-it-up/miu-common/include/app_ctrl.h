/*
 * app_ctrl.h — Leader 网络控制协议帧定义及接口声明
 *
 * 协议帧格式（均以小端序存储）：
 *
 * CTRL_NODE_DETACH (0x88880090)
 *   Leader → 指定节点，命令停止 Thread 协议栈
 *   | Header(4B) | seq(1B) |
 *
 * CTRL_NODE_WHITELIST (0x88880091)
 *   Leader → 指定节点，绑定白名单 parent（router_extaddr 全 0xFF 表示清除）
 *   | Header(4B) | seq(1B) | router_extaddr(8B) |
 *
 * CTRL_NODE_CHILD_TIMEOUT (0x88880092)
 *   Leader → 所有节点（ff03::1 广播），设置 child timeout
 *   | Header(4B) | seq(1B) | timeout_secs(4B) |
 *
 * CTRL_NODE_ACK (0x88888093)
 *   节点 → Leader，命令执行回执
 *   | Header(4B) | seq(1B) | cmd(1B) | status(1B) |
 *
 * CTRL_NODE_REATTACH (0x88880094)
 *   Leader → 指定节点（或广播），命令节点重新入网
 *   | Header(4B) | seq(1B) |
 */

#ifndef __APP_CTRL_H
#define __APP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <openthread/link.h>    /* OT_EXT_ADDRESS_SIZE */
#include <openthread/thread.h>  /* otIp6Address */

/* ----- cmd 字段标识 ----- */
#define CTRL_CMD_DETACH         0x01U
#define CTRL_CMD_WHITELIST      0x02U
#define CTRL_CMD_CHILD_TIMEOUT  0x03U
#define CTRL_CMD_REATTACH       0x04U

/* ----- status 字段 ----- */
#define CTRL_STATUS_OK          0x00U
#define CTRL_STATUS_FAIL        0x01U

/* ----- 离线控制帧 ----- */
typedef struct {
    uint32_t header;
    uint8_t  seq;
} __attribute__((packed)) ctrl_node_detach_t;

/* ----- 白名单绑定帧 ----- */
typedef struct {
    uint32_t header;
    uint8_t  seq;
    uint8_t  router_extaddr[OT_EXT_ADDRESS_SIZE];
} __attribute__((packed)) ctrl_node_whitelist_t;

/* ----- Child Timeout 设置帧 ----- */
typedef struct {
    uint32_t header;
    uint8_t  seq;
    uint32_t timeout_secs;
} __attribute__((packed)) ctrl_node_child_timeout_t;

/* ----- ACK 帧 ----- */
typedef struct {
    uint32_t header;
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  status;
} __attribute__((packed)) ctrl_node_ack_t;

/* ----- Reattach 帧 ----- */
typedef struct {
    uint32_t header;
    uint8_t  seq;
} __attribute__((packed)) ctrl_node_reattach_t;

/* ----- 公共接口 ----- */
void app_ctrl_init(void);

/* 发送接口（FTD Leader 调用） */
void ctrl_detach_send(otIp6Address dst_addr);
void ctrl_whitelist_send(otIp6Address dst_addr, uint8_t *router_extaddr);
void ctrl_child_timeout_send(uint32_t timeout_secs);
void ctrl_reattach_send(otIp6Address dst_addr);
void ctrl_reattach_multicast_send(void);

/* 接收处理接口（被 app_udp_comm_process 调用） */
void app_udp_comm_ctrl_detach_proc(uint8_t *data, uint16_t lens,
                                    otIp6Address src_addr);
void app_udp_comm_ctrl_whitelist_proc(uint8_t *data, uint16_t lens,
                                       otIp6Address src_addr);
void app_udp_comm_ctrl_child_timeout_proc(uint8_t *data, uint16_t lens,
                                           otIp6Address src_addr);
void app_udp_comm_ctrl_reattach_proc(uint8_t *data, uint16_t lens,
                                      otIp6Address src_addr);
void app_udp_comm_ctrl_ack_proc(uint8_t *data, uint16_t lens,
                                 otIp6Address src_addr);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CTRL_H */
