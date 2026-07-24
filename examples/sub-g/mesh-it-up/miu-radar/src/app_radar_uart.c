/*
 * app_radar_uart.c — 雷达传感器 UART 协议驱动（中断驱动 RX）
 *
 * TX: hosal_uart_send(&s_radar_uart1, frame, len)
 * RX: 在独立 UART1 上注册私有 RX ISR，数据写入私有环形缓冲，
 *     ISR 通过 ot_app_task_post() 通知 OT 任务处理，无轮询定时器。
 *
 * UART1 引脚通过 Kconfig 配置（默认 TX=28, RX=29）；
 * miu-common 零修改。
 *
 * 帧格式（LD2410 兼容）：
 *   Header  (4B): FD FC FB FA
 *   DataLen (2B, LE): Cmd(2) + Payload 的总字节数
 *   Cmd     (2B, LE): 见 RADAR_CMD_* / RADAR_ACK_*
 *   Payload (可变)
 *   Tail    (4B): 04 03 02 01
 *
 *   ACK 命令字 = 请求命令字 | 0x0100
 *     0x0007 SET_PARAM  → ACK 0x0107, Payload: status(2B)
 *     0x0008 READ_PARAM → ACK 0x0108, Payload: status(2B) + value(4B LE)
 */

#include "app_radar_uart.h"
#include "hosal_uart.h"     /* hosal_uart_send(), hosal_uart_callback_set(), hosal_uart_receive() */
#include "miu_port.h"       /* ot_app_task_post() */
#include "mcu.h"            /* __NVIC_SetPriority, Uart1_IRQn */
#include "log.h"
#include <string.h>
#include <FreeRTOS.h>
#include <timers.h>         /* ACK 超时定时器 */

/* -----------------------------------------------------------------------
 * UART1 设备（雷达专用，与 UART0 stdio 互不干扰）
 * TX/RX 引脚通过 Kconfig 选项 CONFIG_RADAR_UART1_TX_PIN /
 * CONFIG_RADAR_UART1_RX_PIN 配置，默认 28 / 29。
 * ----------------------------------------------------------------------- */
#ifndef CONFIG_RADAR_UART1_TX_PIN
#define CONFIG_RADAR_UART1_TX_PIN  28
#endif
#ifndef CONFIG_RADAR_UART1_RX_PIN
#define CONFIG_RADAR_UART1_RX_PIN  29
#endif

HOSAL_UART_DEV_DECL(s_radar_uart1, 1,
                    CONFIG_RADAR_UART1_TX_PIN,
                    CONFIG_RADAR_UART1_RX_PIN,
                    UART_BAUDRATE_Baud115200)

/* -----------------------------------------------------------------------
 * 常量
 * ----------------------------------------------------------------------- */
static const uint8_t k_hdr[4]  = { 0xFD, 0xFC, 0xFB, 0xFA };
static const uint8_t k_tail[4] = { 0x04, 0x03, 0x02, 0x01 };

#define RX_DATA_MAX          32U    /* 单帧 Data 区最大字节数 */
#define RADAR_ACK_TIMEOUT_MS 500U
#define RADAR_UART_RX_BUF    256U   /* 私有 RX 环形缓冲大小（必须为 2 的幂） */

/* -----------------------------------------------------------------------
 * 私有 RX 环形缓冲（ISR 写，OT 任务读，单生产者-单消费者，无需加锁）
 * ----------------------------------------------------------------------- */
static uint8_t           s_rx_ring[RADAR_UART_RX_BUF];
static volatile uint16_t s_rx_wr = 0;   /* ISR 写指针 */
static volatile uint16_t s_rx_rd = 0;   /* OT 任务读指针 */

/* -----------------------------------------------------------------------
 * 回调
 * ----------------------------------------------------------------------- */
static radar_set_ack_cb_t s_set_cb = NULL;
static radar_get_ack_cb_t s_get_cb = NULL;

/* -----------------------------------------------------------------------
 * 待应答请求（同一时间仅允许一条在途）
 * ----------------------------------------------------------------------- */
typedef struct {
    bool     active;
    uint16_t cmd;
    uint16_t param_id;
} radar_pending_t;

static radar_pending_t s_pending;
static TimerHandle_t   s_ack_tmr = NULL;

/* -----------------------------------------------------------------------
 * RX 字节状态机
 * ----------------------------------------------------------------------- */
typedef enum {
    RX_IDLE,
    RX_HDR1, RX_HDR2, RX_HDR3,
    RX_LEN_L, RX_LEN_H,
    RX_DATA,
    RX_TAIL,
} rx_state_t;

static rx_state_t s_rx_state    = RX_IDLE;
static uint8_t    s_rx_frame[RX_DATA_MAX];
static uint16_t   s_rx_expected = 0;
static uint16_t   s_rx_idx      = 0;
static uint8_t    s_rx_tail_idx = 0;

/* -----------------------------------------------------------------------
 * ACK 超时定时器
 * ----------------------------------------------------------------------- */
static void ack_timer_cb(TimerHandle_t xTimer)
{
    xTimerDelete(xTimer, 0);
    s_ack_tmr = NULL;
    if (!s_pending.active) return;

    uint16_t pid = s_pending.param_id;
    uint16_t cmd = s_pending.cmd;
    s_pending.active = false;
    log_info("[radar_uart] ACK timeout cmd=0x%04X param=0x%04X", cmd, pid);
    if (cmd == RADAR_CMD_SET_PARAM  && s_set_cb) s_set_cb(pid, 0xFFFFU);
    if (cmd == RADAR_CMD_READ_PARAM && s_get_cb) s_get_cb(pid, 0xFFFFU, 0);
}

static void ack_timer_start(void)
{
    if (s_ack_tmr) { xTimerDelete(s_ack_tmr, 0); s_ack_tmr = NULL; }
    s_ack_tmr = xTimerCreate("rdr_ack",
                              pdMS_TO_TICKS(RADAR_ACK_TIMEOUT_MS),
                              pdFALSE, NULL, ack_timer_cb);
    if (s_ack_tmr) xTimerStart(s_ack_tmr, 0);
}

static void ack_timer_stop(void)
{
    if (s_ack_tmr) { xTimerDelete(s_ack_tmr, 0); s_ack_tmr = NULL; }
}

/* -----------------------------------------------------------------------
 * ACK 帧处理（data = cmd(2B) + payload，在 OT 任务上下文中调用）
 * ----------------------------------------------------------------------- */
static void process_ack(const uint8_t *data, uint16_t len)
{
    if (len < 2) return;
    uint16_t ack_cmd = (uint16_t)(data[0] | ((uint16_t)data[1] << 8));

    if (ack_cmd == RADAR_ACK_SET_PARAM) {
        if (len < 4) { log_info("[radar_uart] SET ACK too short"); return; }
        uint16_t status = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
        log_info("[radar_uart] << SET ACK param=0x%04X status=0x%04X",
                 s_pending.param_id, status);
        ack_timer_stop();
        uint16_t pid = s_pending.param_id;
        s_pending.active = false;
        if (s_set_cb) s_set_cb(pid, status);

    } else if (ack_cmd == RADAR_ACK_READ_PARAM) {
        if (len < 8) { log_info("[radar_uart] GET ACK too short"); return; }
        uint16_t status = (uint16_t)(data[2] | ((uint16_t)data[3] << 8));
        uint32_t value  = (uint32_t)data[4]
                        | ((uint32_t)data[5] << 8)
                        | ((uint32_t)data[6] << 16)
                        | ((uint32_t)data[7] << 24);
        log_info("[radar_uart] << GET ACK param=0x%04X status=0x%04X value=%lu",
                 s_pending.param_id, status, (unsigned long)value);
        ack_timer_stop();
        uint16_t pid = s_pending.param_id;
        s_pending.active = false;
        if (s_get_cb) s_get_cb(pid, status, value);

    } else {
        log_info("[radar_uart] << unknown ACK cmd=0x%04X", ack_cmd);
    }
}

/* -----------------------------------------------------------------------
 * 状态机：逐字节解帧
 * ----------------------------------------------------------------------- */
static void rx_reset(void)
{
    s_rx_state    = RX_IDLE;
    s_rx_expected = 0;
    s_rx_idx      = 0;
    s_rx_tail_idx = 0;
}

static void rx_feed(uint8_t b)
{
    switch (s_rx_state) {
    case RX_IDLE:
        if (b == k_hdr[0]) s_rx_state = RX_HDR1;
        break;
    case RX_HDR1:
        s_rx_state = (b == k_hdr[1]) ? RX_HDR2 : RX_IDLE;
        break;
    case RX_HDR2:
        s_rx_state = (b == k_hdr[2]) ? RX_HDR3 : RX_IDLE;
        break;
    case RX_HDR3:
        if (b == k_hdr[3]) s_rx_state = RX_LEN_L;
        else                rx_reset();
        break;
    case RX_LEN_L:
        s_rx_expected = b;
        s_rx_idx      = 0;
        s_rx_state    = RX_LEN_H;
        break;
    case RX_LEN_H:
        s_rx_expected |= (uint16_t)((uint16_t)b << 8);
        if (s_rx_expected == 0 || s_rx_expected > RX_DATA_MAX) {
            log_info("[radar_uart] bad frame len=%u", (unsigned)s_rx_expected);
            rx_reset();
        } else {
            s_rx_state = RX_DATA;
        }
        break;
    case RX_DATA:
        s_rx_frame[s_rx_idx++] = b;
        if (s_rx_idx >= s_rx_expected) {
            s_rx_tail_idx = 0;
            s_rx_state    = RX_TAIL;
        }
        break;
    case RX_TAIL:
        if (b == k_tail[s_rx_tail_idx]) {
            if (++s_rx_tail_idx == 4) {
                process_ack(s_rx_frame, s_rx_expected);
                rx_reset();
            }
        } else {
            log_info("[radar_uart] bad tail[%u]=0x%02X",
                     (unsigned)s_rx_tail_idx, (unsigned)b);
            rx_reset();
        }
        break;
    }
}

/* -----------------------------------------------------------------------
 * OT 任务上下文：排空私有环形缓冲，喂给状态机
 * ----------------------------------------------------------------------- */
static void radar_rx_process(void *arg)
{
    (void)arg;
    while (s_rx_rd != s_rx_wr) {
        uint8_t b = s_rx_ring[s_rx_rd];
        s_rx_rd = (uint16_t)((s_rx_rd + 1u) & (RADAR_UART_RX_BUF - 1u));
        rx_feed(b);
    }
}

/* -----------------------------------------------------------------------
 * UART1 RX 中断回调（ISR 上下文）：
 *   填充私有环形缓冲，然后通知 OT 任务处理，不使用轮询定时器。
 * ----------------------------------------------------------------------- */
static int radar_uart1_rx_isr(void *p_arg)
{
    hosal_uart_dev_t *uart = (hosal_uart_dev_t *)p_arg;
    uint8_t tmp[32];
    int     len;

    while ((len = hosal_uart_receive(uart, tmp, sizeof(tmp))) > 0) {
        for (int i = 0; i < len; i++) {
            uint16_t next = (uint16_t)((s_rx_wr + 1u) & (RADAR_UART_RX_BUF - 1u));
            if (next != s_rx_rd) {   /* 缓冲未满则写入 */
                s_rx_ring[s_rx_wr] = tmp[i];
                s_rx_wr = next;
            }
        }
    }
    ot_app_task_post(radar_rx_process, NULL);
    return 0;
}

/* -----------------------------------------------------------------------
 * 发帧辅助（TX 用 hosal_uart_send(&s_radar_uart1, ...)）
 * ----------------------------------------------------------------------- */
static void radar_send_frame(uint16_t cmd,
                              const uint8_t *payload, uint16_t payload_len)
{
    uint8_t  frame[32];
    uint8_t *p        = frame;
    uint16_t data_len = 2U + payload_len;

    memcpy(p, k_hdr,  4); p += 4;
    *p++ = (uint8_t)(data_len & 0xFFU);
    *p++ = (uint8_t)(data_len >> 8);
    *p++ = (uint8_t)(cmd & 0xFFU);
    *p++ = (uint8_t)(cmd >> 8);
    if (payload && payload_len) { memcpy(p, payload, payload_len); p += payload_len; }
    memcpy(p, k_tail, 4); p += 4;

    uint16_t total = (uint16_t)(p - frame);
    hosal_uart_send(&s_radar_uart1, frame, total);
    log_info_hexdump("[radar_uart] tx", frame, total);
}

/* -----------------------------------------------------------------------
 * 公开接口
 * ----------------------------------------------------------------------- */
void app_radar_uart_init(radar_set_ack_cb_t set_cb, radar_get_ack_cb_t get_cb)
{
    s_set_cb  = set_cb;
    s_get_cb  = get_cb;
    memset(&s_pending, 0, sizeof(s_pending));
    rx_reset();
    s_rx_wr = 0;
    s_rx_rd = 0;

    /* 初始化 UART1，独立于 UART0 stdio。
     * 配置为中断 RX 模式，注册私有 ISR。
     * NVIC priority 3：高于射频（4）防止 FIFO 溢出。 */
    hosal_uart_init(&s_radar_uart1);
    hosal_uart_callback_set(&s_radar_uart1, HOSAL_UART_RX_CALLBACK,
                            radar_uart1_rx_isr, &s_radar_uart1);
    hosal_uart_ioctl(&s_radar_uart1, HOSAL_UART_MODE_SET,
                     (void *)HOSAL_UART_MODE_INT_RX);
    __NVIC_SetPriority(Uart1_IRQn, 3);

    log_info("[radar_uart] init OK (UART1, TX=%d, RX=%d, interrupt-driven)",
             CONFIG_RADAR_UART1_TX_PIN, CONFIG_RADAR_UART1_RX_PIN);
}

void app_radar_uart_set_param(uint16_t param_id, uint32_t value)
{
    uint8_t payload[6];
    payload[0] = (uint8_t)(param_id & 0xFFU);
    payload[1] = (uint8_t)(param_id >> 8);
    payload[2] = (uint8_t)(value & 0xFFU);
    payload[3] = (uint8_t)((value >>  8) & 0xFFU);
    payload[4] = (uint8_t)((value >> 16) & 0xFFU);
    payload[5] = (uint8_t)((value >> 24) & 0xFFU);

    s_pending.active   = true;
    s_pending.cmd      = RADAR_CMD_SET_PARAM;
    s_pending.param_id = param_id;
    ack_timer_start();

    radar_send_frame(RADAR_CMD_SET_PARAM, payload, sizeof(payload));
    log_info("[radar_uart] >> SET param=0x%04X value=%lu",
             param_id, (unsigned long)value);
}

void app_radar_uart_get_param(uint16_t param_id)
{
    uint8_t payload[2];
    payload[0] = (uint8_t)(param_id & 0xFFU);
    payload[1] = (uint8_t)(param_id >> 8);

    s_pending.active   = true;
    s_pending.cmd      = RADAR_CMD_READ_PARAM;
    s_pending.param_id = param_id;
    ack_timer_start();

    radar_send_frame(RADAR_CMD_READ_PARAM, payload, sizeof(payload));
    log_info("[radar_uart] >> GET param=0x%04X", param_id);
}
