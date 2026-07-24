/*
 * app_task.c (miu-radar)
 *
 * 覆盖弱符号 app_udp_comm_json_process()：
 *   - ACK     → 确认 REGISTER 已被 Leader 接受
 *   - CONTROL → 解析雷达控制命令，调用 app_radar_uart 驱动
 *
 * 按 PROTOCOL.md 第 6.3 节，RADAR CONTROL data 字段：
 *   SET：{"delay_s":<秒>, "dist_m":<米>}         可单独出现，也可同时出现（串行执行）
 *   GET：{"get":"delay_s"|"dist_m"|"all"}        立即回 ACK，结果通过 REPORT 返回
 */

#include <FreeRTOS.h>
#include <string.h>
#include <stdio.h>
#include <task.h>
#include <timers.h>
#include "app_led.h"
#include "app_mac_raw.h"
#include "app_protocol.h"
#include "app_miu_config.h"
#include "app_task.h"
#include "app_udp.h"
#include "app_radar_uart.h"
#include "cli.h"
#include "hosal_lpm.h"
#include "hosal_rf.h"
#include "lmac15p4.h"
#include "log.h"
#include "main.h"
#include "miu_bin_version.h"
#include "miu_ext_mem.h"
#include "subg_ctrl.h"
#include "miu_json.h"
#include <openthread/logging.h>
#include <openthread/ip6.h>
#include <openthread/thread.h>

#if defined(CONFIG_RF1301)
#define BIN_TYPE_ARR 'm', 'i', 'u', '-', 'r', 'a', 'd', 'a' , 'r'
#elif defined(CONFIG_RT581) || defined(CONFIG_RT582)
#define BIN_TYPE_ARR 'm', 'i', 'u', '-', 'r', 'a', 'd', 'a' , 'r'
#else
#error "Unknown IC type, please define BIN_TYPE_ARR"
#endif
const sys_information_t systeminfo = SYSTEMINFO_INIT(BIN_TYPE_ARR);

#define PHY_PIB_TURNAROUND_TIMER  1000
#define PHY_PIB_CCA_DETECTED_TIME 640
#define PHY_PIB_CCA_DETECT_MODE   0
#define PHY_PIB_CCA_THRESHOLD     65
#define MAC_PIB_UNIT_BACKOFF_PERIOD \
    1640
#define MAC_PIB_MAC_ACK_WAIT_DURATION         2000
#define MAC_PIB_MAC_MAX_FRAME_TOTAL_WAIT_TIME 82080
#define MAC_PIB_MAC_MAX_FRAME_RETRIES         4
#define MAC_PIB_MAC_MAX_CSMACA_BACKOFFS       2
#define MAC_PIB_MAC_MAX_BE                    3
#define MAC_PIB_MAC_MIN_BE                    2
static uint16_t cca_duration_table[] = {0, 0, 0, 380, 700, 1140, 260, 480};
static uint32_t frame_total_wait_time_table[] = {0,      0,      0,     70000,
                                                  100000, 150080, 55000, 80000};
static const char* const data_rate_str[] = {
    "2M", "1M", "500K", "200K", "100K", "50K", "300K", "150K", "75K"};
static const char* const band_str[] = {"SubG_915M", "2P4G",      "SubG_868M",
                                       "SubG_433M", "SubG_315M", "SubG_470M"};
uint16_t cca_duration = 0;
uint32_t frame_total_wait_time = 0;
uint32_t backof_period = 0;

#if CONFIG_SUBG_FREQUENCY_BAND_915
static uint8_t sPhyFrequencyBand = HOSAL_RF_BAND_SUBG_915M;
#elif CONFIG_SUBG_FREQUENCY_BAND_868
static uint8_t sPhyFrequencyBand = HOSAL_RF_BAND_SUBG_868M;
#elif CONFIG_SUBG_FREQUENCY_BAND_470
static uint8_t sPhyFrequencyBand = HOSAL_RF_BAND_SUBG_470M;
#elif CONFIG_SUBG_FREQUENCY_BAND_433
static uint8_t sPhyFrequencyBand = HOSAL_RF_BAND_SUBG_433M;
#else
static uint8_t sPhyFrequencyBand = HOSAL_RF_BAND_SUBG_915M;
#endif

#if CONFIG_SUBG_DATA_RATE_FSK_300K
static uint8_t sPhyDataRate = HOSAL_RF_PHY_DATA_RATE_300K;
#elif CONFIG_SUBG_DATA_RATE_FSK_200K
static uint8_t sPhyDataRate = HOSAL_RF_PHY_DATA_RATE_200K;
#elif CONFIG_SUBG_DATA_RATE_FSK_100K
static uint8_t sPhyDataRate = HOSAL_RF_PHY_DATA_RATE_100K;
#elif CONFIG_SUBG_DATA_RATE_FSK_50K
static uint8_t sPhyDataRate = HOSAL_RF_PHY_DATA_RATE_50K;
#elif CONFIG_SUBG_DATA_RATE_OQPSK_25K
static uint8_t sPhyDataRate = HOSAL_RF_PHY_DATA_RATE_25K;
#else
static uint8_t sPhyDataRate = HOSAL_RF_PHY_DATA_RATE_300K;
#endif

typedef struct {
    char networkName[OT_NETWORK_NAME_MAX_SIZE + 1];
    uint8_t extPanId[OT_EXT_PAN_ID_SIZE];
    uint8_t networkKey[OT_NETWORK_KEY_SIZE];
    uint8_t meshLocalPrefix[OT_MESH_LOCAL_PREFIX_SIZE];
    uint8_t pskc[OT_PSKC_MAX_SIZE];
    uint8_t channel;
    uint16_t panId;
} __attribute__((packed)) AppNetworkConfig;

/* -----------------------------------------------------------------------
 * 雷达参数应答回调
 * 由 app_radar_uart.c 在 OT 任务上下文中调用
 * ----------------------------------------------------------------------- */

/* 本设备在 JSON 协议里的标识（需与 app_net_mgm.c 保持一致） */
#define RADAR_DEV_TYPE  "RADAR"
#define RADAR_DEV_NAME  "radar_01"
#define RADAR_FW_VER    "1.0.0"

/* -----------------------------------------------------------------------
 * 出站序列号与链式执行状态
 * ----------------------------------------------------------------------- */
static uint8_t  s_json_seq           = 0;   /* REPORT / ACK 共用，单调递增 */
static int      s_ctrl_seq           = 0;   /* 当前在途 CONTROL 的 seq，用于回 ACK */

/* 链式 SET：delay_s ACK 之后继续设 dist_m */
static bool     s_chain_set_dist     = false;
static uint32_t s_chain_set_dist_val = 0;

/* 链式 GET：第一个 GET ACK 之后继续请求下一个参数 */
static uint16_t s_chain_get_param    = 0;

/* -----------------------------------------------------------------------
 * 发送辅助
 * ----------------------------------------------------------------------- */
static void radar_udp_send(const char *buf, uint16_t len)
{
    otInstance   *inst = otrGetInstance();
    otIp6Address  dst  = *otThreadGetRloc(inst);
    dst.mFields.m8[14] = 0xFC;
    dst.mFields.m8[15] = 0x00;
    uint8_t *p = pvPortMalloc(len);
    if (!p) { log_info("[radar] send alloc fail"); return; }
    memcpy(p, buf, len);
    if (app_udpSend(dst, p, len, false) != 0)
        log_info("[radar] send fail");
    vPortFree(p);
}

/* 发送 REPORT；data_content 为 "data":{} 花括号内的内容 */
static void radar_send_json_report(const char *data_content)
{
    otInstance  *inst = otrGetInstance();
    char         ip_str[OT_IP6_ADDRESS_STRING_SIZE];
    char         buf[256];

    const otIp6Address *ml_eid = otThreadGetMeshLocalEid(inst);
    uint16_t rloc16 = otThreadGetRloc16(inst);
    otIp6AddressToString(ml_eid, ip_str, sizeof(ip_str));

    s_json_seq++;
    snprintf(buf, sizeof(buf),
             "{\"ver\":1,\"type\":\"REPORT\","
             "\"dev_type\":\"" RADAR_DEV_TYPE "\","
             "\"dev_name\":\"" RADAR_DEV_NAME "\","
             "\"ip\":\"%s\","
             "\"rloc16\":%u,"
             "\"seq\":%u,"
             "\"data\":{%s}}",
             ip_str, (unsigned)rloc16, (unsigned)s_json_seq,
             data_content);

    log_info("[radar] >> REPORT %s", buf);
    radar_udp_send(buf, (uint16_t)strlen(buf));
}

/* 发送 ACK；seq_ack 为待确认的 CONTROL seq */
static void radar_send_json_ack(int seq_ack, int code)
{
    char buf[256];
    s_json_seq++;
    snprintf(buf, sizeof(buf),
             "{\"ver\":1,\"type\":\"ACK\","
             "\"dev_type\":\"" RADAR_DEV_TYPE "\","
             "\"dev_name\":\"" RADAR_DEV_NAME "\","
             "\"seq\":%u,"
             "\"data\":{\"seq_ack\":%d,\"code\":%d,\"msg\":\"%s\"}}",
             (unsigned)s_json_seq,
             seq_ack, code,
             code == 0 ? "ok" : "fail");

    log_info("[radar] >> ACK seq_ack=%d code=%d", seq_ack, code);
    radar_udp_send(buf, (uint16_t)strlen(buf));
}

/* -----------------------------------------------------------------------
 * 雷达参数应答回调
 * 由 app_radar_uart.c 在 OT 任务上下文中调用
 * ----------------------------------------------------------------------- */

/* SET_PARAM ACK 回调：UART 执行完成，回 ACK 给 Leader */
static void on_set_param_ack(uint16_t param_id, uint16_t status)
{
    log_info("[radar] set_ack param=0x%04X status=%u",
             (unsigned)param_id, (unsigned)status);

    /* 链式执行：delay_s 设完后继续设 dist_m */
    if (param_id == RADAR_PARAM_DISAPPEAR_DELAY && s_chain_set_dist) {
        s_chain_set_dist = false;
        app_radar_uart_set_param(RADAR_PARAM_MAX_DISTANCE, s_chain_set_dist_val);
        return;   /* 等 dist_m 的 ACK 再回 CONTROL ACK */
    }

    radar_send_json_ack(s_ctrl_seq, status == 0 ? 0 : 2);
}

/* GET_PARAM ACK 回调：UART 返回数据，通过 REPORT 上报参数值 */
static void on_get_param_ack(uint16_t param_id, uint16_t status, uint32_t value)
{
    log_info("[radar] get_ack param=0x%04X status=%u value=%lu",
             (unsigned)param_id, (unsigned)status, (unsigned long)value);

    if (status == 0) {
        char frag[48];
        if (param_id == RADAR_PARAM_MAX_DISTANCE) {
            /* UART 单位 0.1m → 转回米 */
            unsigned long dist_m_int = value / 10u;      // 整数部分
            unsigned long dist_m_frac = value % 10u;     // 小数部分（一位）
            snprintf(frag, sizeof(frag),
                    "\"dist_m\":%lu.%lu", dist_m_int, dist_m_frac);
        } else if (param_id == RADAR_PARAM_DISAPPEAR_DELAY) {
            snprintf(frag, sizeof(frag),
                     "\"delay_s\":%lu", (unsigned long)value);
        } else {
            snprintf(frag, sizeof(frag),
                     "\"param_0x%04X\":%lu",
                     (unsigned)param_id, (unsigned long)value);
        }
        radar_send_json_report(frag);
    }

    /* 链式执行："all" 时继续请求下一个参数 */
    if (s_chain_get_param) {
        uint16_t next = s_chain_get_param;
        s_chain_get_param = 0;
        app_radar_uart_get_param(next);
    }
}

/* -----------------------------------------------------------------------
 * app_udp_comm_json_process 覆盖（弱符号替换）
 * 接收来自 Leader 的 ACK 或 CONTROL
 * ----------------------------------------------------------------------- */
void app_udp_comm_json_process(uint8_t *data, uint16_t lens,
                                otIp6Address src_addr)
{
    char *json = (char *)data;
    char msg_type[16] = {0};
    char dev_name[32] = {0};

    if (miu_json_get_str(json, "type",     msg_type, sizeof(msg_type)) < 0 ||
        miu_json_get_str(json, "dev_name", dev_name, sizeof(dev_name)) < 0) {
        log_info("[radar] rx: malformed JSON");
        return;
    }

    /* ---- ACK：Leader 确认 REGISTER ---- */
    if (strcmp(msg_type, "ACK") == 0) {
        int code = 0, seq_ack = 0;
        miu_json_get_int(json, "code",    &code);
        miu_json_get_int(json, "seq_ack", &seq_ack);
        log_info("[radar] << ACK dev=%s seq_ack=%d code=%d",
                 dev_name, seq_ack, code);
        if (code == 0)
            log_info("[radar] REGISTER confirmed by leader");

    /* ---- CONTROL：Leader 转发 PC 的控制命令 ---- */
    } else if (strcmp(msg_type, "CONTROL") == 0) {
        /* 保存 CONTROL seq，用于回 ACK */
        miu_json_get_int(json, "seq", &s_ctrl_seq);

        /* ---- GET_PARAM：data 中含 "get" 字段 ---- */
        char get_field[16] = {0};
        if (miu_json_get_str(json, "get", get_field, sizeof(get_field)) >= 0) {
            log_info("[radar] << CONTROL GET '%s' seq=%d", get_field, s_ctrl_seq);
            radar_send_json_ack(s_ctrl_seq, 0);   /* 立即回 ACK，结果通过 REPORT 异步返回 */

            s_chain_get_param = 0;
            if (strcmp(get_field, "delay_s") == 0) {
                app_radar_uart_get_param(RADAR_PARAM_DISAPPEAR_DELAY);
            } else if (strcmp(get_field, "dist_m") == 0) {
                app_radar_uart_get_param(RADAR_PARAM_MAX_DISTANCE);
            } else if (strcmp(get_field, "all") == 0) {
                /* 先 GET delay_s，ACK 回来后链式 GET dist_m */
                s_chain_get_param = RADAR_PARAM_MAX_DISTANCE;
                app_radar_uart_get_param(RADAR_PARAM_DISAPPEAR_DELAY);
            } else {
                log_info("[radar] << CONTROL GET: unknown field '%s'", get_field);
            }
            return;
        }

        /* ---- SET_PARAM：data 中含语义字段 delay_s / dist_m ---- */
        int delay_s = -1, dist_m = -1;
        bool has_delay = (miu_json_get_int(json, "delay_s", &delay_s) == 0);
        bool has_dist  = (miu_json_get_int(json, "dist_m",  &dist_m)  == 0);

        if (!has_delay && !has_dist) {
            log_info("[radar] << CONTROL: no recognized param (seq=%d)", s_ctrl_seq);
            radar_send_json_ack(s_ctrl_seq, 1);   /* code=1: 未知命令 */
            return;
        }

        log_info("[radar] << CONTROL SET delay_s=%d dist_m=%d seq=%d",
                 has_delay ? delay_s : -1,
                 has_dist  ? dist_m  : -1,
                 s_ctrl_seq);

        s_chain_set_dist = false;
        if (has_delay && has_dist) {
            /* 两个参数同时下发：先设 delay_s，ACK 后再设 dist_m */
            s_chain_set_dist     = true;
            s_chain_set_dist_val = (uint32_t)((unsigned)dist_m * 10u); /* 米→0.1m */
            app_radar_uart_set_param(RADAR_PARAM_DISAPPEAR_DELAY, (uint32_t)delay_s);
        } else if (has_delay) {
            app_radar_uart_set_param(RADAR_PARAM_DISAPPEAR_DELAY, (uint32_t)delay_s);
        } else {
            /* dist_m 单位米，UART 参数单位 0.1m */
            app_radar_uart_set_param(RADAR_PARAM_MAX_DISTANCE,
                                     (uint32_t)((unsigned)dist_m * 10u));
        }

    } else {
        log_info("[radar] << unknown type '%s'", msg_type);
    }
}

/* -----------------------------------------------------------------------
 * OpenThread 回调（仅打印角色变化日志）
 * ----------------------------------------------------------------------- */
static void ot_stateChangeCallback(uint32_t flags, void* p_context)
{
    otInstance* instance = (otInstance*)p_context;

    if (flags & OT_CHANGED_THREAD_ROLE) {
        log_info("Current role : %s",
                 otThreadDeviceRoleToString(otThreadGetDeviceRole(instance)));

        if (otThreadGetDeviceRole(instance) > OT_DEVICE_ROLE_DETACHED) {
            char string[OT_IP6_ADDRESS_STRING_SIZE];
            log_info("Rloc16       : %x", otThreadGetRloc16(instance));
            otIp6AddressToString(otThreadGetMeshLocalEid(instance),
                                  string, sizeof(string));
            log_info("ML-EID       : %s", string);
        }
    }
}

static void otsleepInit(otInstance* instance)
{
    otLinkModeConfig mode;
    mode.mDeviceType   = false;
    mode.mNetworkData  = false;
    mode.mRxOnWhenIdle = true;
    otThreadSetLinkMode(instance, mode);
}

static void otdatasetInit(otInstance* instance)
{
    otOperationalDatasetTlvs app_dataset_tlv;
    otOperationalDataset app_dataset;
    bool load_default_config = false;
    const char* const desired_network_name = "Rafael Miu";

    AppNetworkConfig netconfig = {
        .networkName = "Rafael Miu",
        .extPanId = {0x00, 0x0d, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x00},
        .networkKey = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                       0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
        .meshLocalPrefix = {0xfd, 0x00, 0x0d, 0xb8, 0x00, 0x00, 0x00, 0x00},
        .pskc = {0x74, 0x68, 0x72, 0x65, 0x61, 0x64, 0x6a, 0x70,
                 0x61, 0x6b, 0x65, 0x74, 0x65, 0x73, 0x74, 0x00},
        .channel = 2,
        .panId = 0xabcd};

    if (otDatasetGetActiveTlvs(instance, &app_dataset_tlv) != OT_ERROR_NONE) {
        load_default_config = true;
    } else if (otDatasetParseTlvs(&app_dataset_tlv, &app_dataset) != OT_ERROR_NONE) {
        load_default_config = true;
    } else if (!app_dataset.mComponents.mIsNetworkNamePresent) {
        load_default_config = true;
    } else if (strncmp((const char*)app_dataset.mNetworkName.m8,
                       desired_network_name, strlen(desired_network_name)) != 0) {
        load_default_config = true;
    }

    otExtAddress extAddress;
    otLinkGetFactoryAssignedIeeeEui64(instance, &extAddress);
    otLinkSetExtendedAddress(instance, &extAddress);

    otIp6InterfaceIdentifier iid;
    memcpy(iid.mFields.m8, extAddress.m8, OT_EXT_ADDRESS_SIZE);
    otIp6SetMeshLocalIid(instance, &iid);

    if (load_default_config) {
        memset(&app_dataset, 0, sizeof(app_dataset));
        memset(&app_dataset_tlv, 0, sizeof(app_dataset_tlv));
        app_dataset.mActiveTimestamp.mSeconds = 0;
        app_dataset.mActiveTimestamp.mTicks = 0;
        app_dataset.mActiveTimestamp.mAuthoritative = false;
        app_dataset.mComponents.mIsActiveTimestampPresent = true;
        app_dataset.mChannel = netconfig.channel;
        app_dataset.mComponents.mIsChannelPresent = true;
        app_dataset.mPanId = (otPanId)netconfig.panId;
        app_dataset.mComponents.mIsPanIdPresent = true;
        app_dataset.mWakeupChannel = netconfig.channel;
        app_dataset.mComponents.mIsWakeupChannelPresent = true;
        memcpy(app_dataset.mExtendedPanId.m8, netconfig.extPanId, OT_EXT_PAN_ID_SIZE);
        app_dataset.mComponents.mIsExtendedPanIdPresent = true;
        memcpy(app_dataset.mNetworkKey.m8, netconfig.networkKey, OT_NETWORK_KEY_SIZE);
        app_dataset.mComponents.mIsNetworkKeyPresent = true;
        memcpy(app_dataset.mPskc.m8, netconfig.pskc, OT_PSKC_MAX_SIZE);
        app_dataset.mComponents.mIsPskcPresent = true;
        size_t length = strlen(netconfig.networkName);
        memcpy(app_dataset.mNetworkName.m8, netconfig.networkName, length);
        app_dataset.mComponents.mIsNetworkNamePresent = true;
        memcpy(app_dataset.mMeshLocalPrefix.m8, netconfig.meshLocalPrefix, OT_MESH_LOCAL_PREFIX_SIZE);
        app_dataset.mComponents.mIsMeshLocalPrefixPresent = true;
        otDatasetUpdateTlvs(&app_dataset, &app_dataset_tlv);
        otDatasetSetActiveTlvs(instance, &app_dataset_tlv);
    }
}

void otrInitUser(otInstance* instance)
{
#if OPENTHREAD_CONFIG_LOG_LEVEL_DYNAMIC_ENABLE
    otLoggingSetLevel(OT_LOG_LEVEL_WARN);
#endif
    otAppCliInit(instance);
    otsleepInit(instance);
    otSetStateChangedCallback(instance, ot_stateChangeCallback, instance);
    app_sockInit(instance, CONFIG_APP_TASK_UDP_LISTEN_PORT);
    app_macRawInit(instance);
    app_led_pin_init();
    otdatasetInit(instance);
    app_radar_uart_init(on_set_param_ack, on_get_param_ack);
    app_udp_comm_init();
#if !CFG_USE_CENTRAK_CONFIG
    otIp6SetEnabled(instance, true);
    otThreadSetEnabled(instance, true);
#endif
}

void app_task(void)
{
    while (true) {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    }
}

void app_common_init()
{
    hosal_rf_init(HOSAL_RF_MODE_RUCI_CMD);

    log_info("Mesh It Up TEST device");
    log_info("Band      : %s", band_str[sPhyFrequencyBand]);
    log_info("Data Rate : %s", data_rate_str[sPhyDataRate]);

    cca_duration = cca_duration_table[sPhyDataRate];
    frame_total_wait_time = frame_total_wait_time_table[sPhyDataRate];
    backof_period = PHY_PIB_TURNAROUND_TIMER + cca_duration;

#if CONFIG_SUBG_DATA_RATE_OQPSK_25K
    lmac15p4_init(LMAC15P4_SUBG_OQPSK, sPhyFrequencyBand);
#else
    lmac15p4_init(LMAC15P4_SUBG_FSK, sPhyFrequencyBand);
#endif

    lmac15p4_phy_pib_set(PHY_PIB_TURNAROUND_TIMER, PHY_PIB_CCA_DETECT_MODE,
                         PHY_PIB_CCA_THRESHOLD, cca_duration);
    lmac15p4_mac_pib_set(backof_period, MAC_PIB_MAC_ACK_WAIT_DURATION,
                         MAC_PIB_MAC_MAX_BE, MAC_PIB_MAC_MAX_CSMACA_BACKOFFS,
                         frame_total_wait_time, MAC_PIB_MAC_MAX_FRAME_RETRIES,
                         MAC_PIB_MAC_MIN_BE);

    subg_ctrl_sleep_set(false);
    subg_ctrl_idle_set();
#if CONFIG_SUBG_DATA_RATE_OQPSK_25K
    subg_ctrl_modem_config_set(LMAC15P4_SUBG_OQPSK, sPhyDataRate, SUBG_CTRL_FSK_MOD_1);
    subg_ctrl_mac_set(LMAC15P4_SUBG_OQPSK, SUBG_CTRL_CRC_TYPE_16, SUBG_CTRL_WHITEN_DISABLE);
    subg_ctrl_preamble_set(LMAC15P4_SUBG_OQPSK, 8);
    subg_ctrl_sfd_set(LMAC15P4_SUBG_OQPSK, 0x00007209);
    subg_ctrl_filter_set(LMAC15P4_SUBG_OQPSK, SUBG_CTRL_FILTER_TYPE_GFSK);
#else
    subg_ctrl_modem_config_set(SUBG_CTRL_MODU_FSK, sPhyDataRate, SUBG_CTRL_FSK_MOD_1);
    subg_ctrl_mac_set(SUBG_CTRL_MODU_FSK, SUBG_CTRL_CRC_TYPE_16, SUBG_CTRL_WHITEN_DISABLE);
    subg_ctrl_preamble_set(SUBG_CTRL_MODU_FSK, 8);
    subg_ctrl_sfd_set(SUBG_CTRL_MODU_FSK, 0x00007209);
    subg_ctrl_filter_set(SUBG_CTRL_MODU_FSK, SUBG_CTRL_FILTER_TYPE_GFSK);
#endif

    log_info("Channel Range : %d ~ %d",
             OPENTHREAD_CONFIG_PLATFORM_RADIO_PROPRIETARY_CHANNEL_MIN,
             OPENTHREAD_CONFIG_PLATFORM_RADIO_PROPRIETARY_CHANNEL_MAX);

    otRadioChRange_t radiochrange;
    radiochrange.minChannel     = OPENTHREAD_CONFIG_PLATFORM_RADIO_PROPRIETARY_CHANNEL_MIN;
    radiochrange.maxChannel     = OPENTHREAD_CONFIG_PLATFORM_RADIO_PROPRIETARY_CHANNEL_MAX;
    radiochrange.frequencyBase  = OPENTHREAD_CONFIG_CHANNEL_FREQUENCY;
    radiochrange.frequencySpacing = OPENTHREAD_CONFIG_CHANNEL_SPACING;
    otPlatRadioSetChannelRange(radiochrange);

    miuStart();
}
