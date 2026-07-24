/*
 * app_device_table.h — Leader 设备表
 *
 * 职责：
 *   - 存储所有已入网子设备的基本信息（设备名、类型、IP、RLOC16、固件版本）
 *   - 缓存各设备最新上报的状态数据（用于 PC 查询）
 *   - 提供按设备名查询 IP 的路由接口（供 UART-PC 桥接转发 CONTROL 使用）
 *
 * 仅在 FTD（Leader）固件中编译，MTD 不引用此文件。
 */

#ifndef APP_DEVICE_TABLE_H
#define APP_DEVICE_TABLE_H

#include <stdint.h>
#include <stdbool.h>
#include <openthread/ip6.h>

/* -----------------------------------------------------------------------
 * 设备类型
 * ----------------------------------------------------------------------- */
typedef enum {
    MIU_DEV_TYPE_SOCKET  = 0,   /* 插座：受控 on/off */
    MIU_DEV_TYPE_LIGHT   = 1,   /* 光照传感器：上报 lux */
    MIU_DEV_TYPE_RADAR   = 2,   /* 雷达：上报 presence，可配置串口参数 */
    MIU_DEV_TYPE_RGBCW   = 3,   /* RGBCW 灯带：受控各通道 PWM 占空比 */
    MIU_DEV_TYPE_UNKNOWN = 0xFF,
} miu_dev_type_t;

/* -----------------------------------------------------------------------
 * 各设备最新状态缓存（随每次 REPORT 更新）
 * ----------------------------------------------------------------------- */

/* 插座：0=off  1=on */
typedef struct {
    uint8_t state;
} miu_socket_data_t;

/* 光照传感器：单位 lux */
typedef struct {
    uint32_t lux;
} miu_light_data_t;

/* 雷达：0=无人  1=有人 */
typedef struct {
    uint8_t presence;
} miu_radar_data_t;

/* RGBCW 灯带：各通道 PWM 占空比 0~100 */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t c;  /* cold white 冷白 */
    uint8_t w;  /* warm white 暖白 */
} miu_rgbcw_data_t;

/* -----------------------------------------------------------------------
 * 字段长度限制
 * ----------------------------------------------------------------------- */
#define MIU_DEV_NAME_MAX   32
#define MIU_FW_VER_MAX     16
#define MIU_HW_VER_MAX      8

/* -----------------------------------------------------------------------
 * 单条设备信息
 * ----------------------------------------------------------------------- */
typedef struct {
    bool             valid;
    miu_dev_type_t   dev_type;
    char             dev_name[MIU_DEV_NAME_MAX]; /* 预烧录，全网唯一标识 */
    otIp6Address     ip;                          /* ML-EID，UDP 路由目标 */
    uint16_t         rloc16;
    char             fw_ver[MIU_FW_VER_MAX];
    char             hw_ver[MIU_HW_VER_MAX];
    uint32_t         last_seen_ms;               /* 最后通信时间戳（FreeRTOS tick ms） */

    /* 最新状态缓存，按设备类型取对应成员 */
    union {
        miu_socket_data_t  socket;
        miu_light_data_t   light;
        miu_radar_data_t   radar;
        miu_rgbcw_data_t   rgbcw;
    } data;
} miu_device_info_t;

/* -----------------------------------------------------------------------
 * 设备表（静态分配，Leader 全局唯一）
 * ----------------------------------------------------------------------- */
#define MIU_MAX_DEVICES  16

typedef struct {
    miu_device_info_t devices[MIU_MAX_DEVICES];
    uint8_t           count;
} miu_device_table_t;

/* -----------------------------------------------------------------------
 * API
 * ----------------------------------------------------------------------- */

/** 初始化设备表，清零所有条目 */
void app_device_table_init(void);

/**
 * 按设备名查找条目。
 * @return 指向条目的指针，未找到返回 NULL。
 */
miu_device_info_t *app_device_table_find(const char *dev_name);

/**
 * 新增或更新一条设备信息（入网 REGISTER 时调用）。
 * @return 指向条目的指针，表满时返回 NULL。
 */
miu_device_info_t *app_device_table_add(const char        *dev_name,
                                         miu_dev_type_t     type,
                                         const otIp6Address *ip,
                                         uint16_t            rloc16,
                                         const char         *fw_ver,
                                         const char         *hw_ver);

/**
 * 查找设备名对应的 ML-EID，用于 CONTROL 路由。
 * @return true=找到并写入 out_ip；false=未找到。
 */
bool app_device_table_get_ip(const char *dev_name, otIp6Address *out_ip);

/** 打印当前设备表到日志 */
void app_device_table_dump(void);

/** JSON 字符串 → 设备类型枚举 */
miu_dev_type_t app_device_type_from_str(const char *str);

/** 设备类型枚举 → JSON 字符串 */
const char *app_device_type_to_str(miu_dev_type_t type);

/**
 * 遍历设备表，对每条有效条目构造 REGISTER 格式 JSON，调用 cb(json_str)。
 * json_str 使用栈上缓冲，cb 返回后即失效，cb 内勿保存指针。
 * 用于 PC 请求设备表重播（QUERY_TABLE 命令）。
 */
void app_device_table_iter_register_json(void (*cb)(const char *json_str));

#endif /* APP_DEVICE_TABLE_H */
