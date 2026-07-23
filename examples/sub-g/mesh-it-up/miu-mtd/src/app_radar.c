/*
 * app_radar.c (MTD) — 雷达存在感应上报
 *
 * 功能：
 *   MTD 节点检测到雷达 GPIO 触发后，通过 UDP 向 Leader 上报
 *   radar_report_t 格式的数据包（RADAR_REPORT_HEADER）。
 *
 * 使用方式：
 *   1. 在 otrInitUser()（miu-mtd/src/app_task.c）中调用 app_radar_init()
 *   2. 当雷达 GPIO 中断触发时，在中断回调中调用 app_radar_irq_trigger()
 *   3. 也可直接调用 radar_report_send(status) 发送指定状态
 *
 * 引脚配置：
 *   默认雷达中断引脚为 GPIO APP_RADAR_GPIO_PIN（修改下方宏可调整）。
 *   引脚电平含义：1 = 有人，0 = 无人。
 *
 * 移植说明：
 *   - 中断不能直接调用 OT API，使用 ot_app_task_post() 安全投递任务
 *   - UDP 发送目标为 Leader（使用 RLOC Leader 地址 0xFC00）
 */

#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <miu_port.h>
#include <openthread/thread.h>
#include <openthread/ip6.h>
#include "hosal_gpio.h"
#include "hosal_sysctrl.h"
#include "log.h"
#include "app_protocol.h"
#include "app_udp.h"

/* -----------------------------------------------------------------------
 * 配置
 * ----------------------------------------------------------------------- */
#ifndef APP_RADAR_GPIO_PIN
#define APP_RADAR_GPIO_PIN  23   /* 雷达感应器 GPIO 引脚，根据实际硬件调整 */
#endif

/* -----------------------------------------------------------------------
 * 状态
 * ----------------------------------------------------------------------- */
static uint8_t s_radar_seq = 0;

/* -----------------------------------------------------------------------
 * 发送上报
 * ----------------------------------------------------------------------- */
void radar_report_send(uint8_t status)
{
    otInstance  *inst = otrGetInstance();
    otDeviceRole role = otThreadGetDeviceRole(inst);

    if (role != OT_DEVICE_ROLE_CHILD && role != OT_DEVICE_ROLE_ROUTER) {
        log_info("[radar] not attached, skip report");
        return;
    }

    radar_report_t report;
    report.header = RADAR_REPORT_HEADER;
    report.seq    = s_radar_seq++;
    report.status = status;

    /* 发给 Leader（RLOC Leader 地址末两字节为 0xFC00） */
    otIp6Address dst = *otThreadGetRloc(inst);
    dst.mFields.m8[14] = 0xFC;
    dst.mFields.m8[15] = 0x00;

    if (app_udpSend(dst, (uint8_t *)&report, sizeof(report), false)) {
        log_info("[radar] report send fail seq=%u status=%u", report.seq, status);
    } else {
        char s[OT_IP6_ADDRESS_STRING_SIZE];
        otIp6AddressToString(&dst, s, sizeof(s));
        log_info("[radar] >> report(%s) seq=%u status=%u", s, report.seq, status);
    }
}

/* -----------------------------------------------------------------------
 * GPIO 中断触发入口（从中断上下文调用）
 *
 * 注意：此函数在 ISR / GPIO 中断回调中调用，不可直接调用 OT API 或
 * 阻塞操作，使用 ot_app_task_post() 投递任务到 OT 任务上下文。
 * ----------------------------------------------------------------------- */
typedef struct {
    uint8_t status;
} radar_report_arg_t;

static void radar_report_task(void *arg)
{
    uint8_t status = (arg != NULL) ? ((radar_report_arg_t *)arg)->status : 1;
    radar_report_send(status);
}

void app_radar_irq_trigger(uint8_t status)
{
    /* 注意：ot_app_task_post 的 arg 生命周期须持续到任务执行完毕。
     * 由于 status 是个简单 uint8_t，这里用 (void*)(uintptr_t) 转换传递。
     * 如果 ot_app_task_post 的 arg 不支持直接传值，改用 pvPortMalloc 分配。 */
    ot_app_task_post(radar_report_task, (void *)(uintptr_t)status);
}

/* -----------------------------------------------------------------------
 * 初始化（在 otrInitUser() 中调用）
 * ----------------------------------------------------------------------- */
void app_radar_init(void)
{
    log_info("[radar] init (GPIO pin %d)", APP_RADAR_GPIO_PIN);

    /* 配置 GPIO 为输入，下拉 + 双边沿中断 */
    hosal_pin_set_pullopt(APP_RADAR_GPIO_PIN, HOSAL_PULL_DOWN_100K);
    {
        hosal_gpio_input_config_t pin_cfg = {
            .usr_cb       = NULL,
            .param        = NULL,
            .pin_int_mode = HOSAL_GPIO_PIN_INT_BOTH_EDGE,
        };
        hosal_gpio_cfg_input(APP_RADAR_GPIO_PIN, pin_cfg);
    }

    /* 中断回调注册示例（具体 API 以 SDK 实际提供为准）：
     *
     * hosal_gpio_irq_cfg_t irq_cfg = {
     *     .pin    = APP_RADAR_GPIO_PIN,
     *     .mode   = HOSAL_IRQ_TRIG_EDGE_BOTH,   // 双边沿
     *     .cb     = NULL,                        // 在 app_task.c 中注册
     * };
     * hosal_gpio_irq_set(&irq_cfg);
     *
     * 或者在 otrInitUser() 中直接调用：
     *   hosal_gpio_irq_set_cb(APP_RADAR_GPIO_PIN, my_gpio_irq_cb, NULL);
     *
     * 在 GPIO 中断回调中调用 app_radar_irq_trigger(gpio_level);
     */
    log_info("[radar] GPIO interrupt setup: register irq in app_task.c otrInitUser()");
}
