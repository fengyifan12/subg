/*
 * app_radar_gpio.c (miu-radar) — 雷达存在感应 GPIO 输入
 *
 * 功能：
 *   检测 GPIO 电平变化，高电平上报有人，低电平上报无人。
 *   通过 app_radar_net_report_presence() 向 Leader 发送 JSON REPORT。
 *
 * 使用方式：
 *   在 otrInitUser() 中调用 app_radar_gpio_init()
 *
 * 引脚配置：
 *   默认 GPIO30（CONFIG_RADAR_PRESENCE_GPIO_PIN），高电平 = 有人，低电平 = 无人。
 *
 * 注意（RF1301/RT584）：
 *   gpio_cfg(INPUT) 内部会强制 PULLUP_100K，因此 pull 必须在 cfg_input 之后设置。
 *   必须 NVIC_EnableIRQ(Gpio_IRQn)，仅 hosal_gpio_int_enable() 不够。
 */

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <miu_port.h>
#include "mcu.h"
#include "hosal_gpio.h"
#include "hosal_sysctrl.h"
#include "log.h"
#include "app_radar_gpio.h"

#ifndef CONFIG_RADAR_PRESENCE_GPIO_PIN
#define CONFIG_RADAR_PRESENCE_GPIO_PIN  30
#endif

extern void app_radar_net_report_presence(uint8_t presence);

/* -----------------------------------------------------------------------
 * OT 任务：实际上报
 * ----------------------------------------------------------------------- */
static void presence_report_task(void *arg)
{
    uint8_t presence = (uint8_t)(uintptr_t)arg;
    log_info("[radar_gpio] report presence=%d", presence);
    app_radar_net_report_presence(presence);
}

void app_radar_gpio_irq_trigger(uint8_t presence)
{
    ot_app_task_post(presence_report_task, (void *)(uintptr_t)presence);
}

/* -----------------------------------------------------------------------
 * GPIO 中断回调（ISR 上下文）
 * ----------------------------------------------------------------------- */
static void radar_presence_gpio_cb(uint32_t pin_number, void *isr_param)
{
    uint32_t level = 0;

    (void)isr_param;
    if (hosal_gpio_pin_get(pin_number, &level) != STATUS_SUCCESS) {
        return;
    }

    app_radar_gpio_irq_trigger(level ? 1u : 0u);
}

/* -----------------------------------------------------------------------
 * 初始化（在 otrInitUser() 中调用）
 * ----------------------------------------------------------------------- */
void app_radar_gpio_init(void)
{
    uint32_t level = 0;

    log_info("[radar_gpio] init (GPIO pin %d)", CONFIG_RADAR_PRESENCE_GPIO_PIN);

    /* 1) 输入 + 双边沿中断 + 回调
     *    注意：RT584 gpio_cfg(INPUT) 会强制上拉，下面再改回下拉。 */
    {
        hosal_gpio_input_config_t pin_cfg = {
            .usr_cb       = radar_presence_gpio_cb,
            .param        = NULL,
            .pin_int_mode = HOSAL_GPIO_PIN_INT_BOTH_EDGE,
        };
        hosal_gpio_cfg_input(CONFIG_RADAR_PRESENCE_GPIO_PIN, pin_cfg);
    }

    /* 2) 空闲低电平：必须在 cfg_input 之后设置，否则会被覆盖成上拉 */
    hosal_pin_set_pullopt(CONFIG_RADAR_PRESENCE_GPIO_PIN, HOSAL_PULL_DOWN_100K);

    /* 3) 开启引脚中断门控 */
    hosal_gpio_int_enable(CONFIG_RADAR_PRESENCE_GPIO_PIN);

    /* 4) 使能 NVIC（与 UART 一致设优先级，允许 ISR 里调 ot_app_task_post） */
    NVIC_SetPriority(Gpio_IRQn, 4);
    NVIC_EnableIRQ(Gpio_IRQn);

    /* 5) 上报当前电平（上电时若已是高/低，不会再产生边沿） */
    if (hosal_gpio_pin_get(CONFIG_RADAR_PRESENCE_GPIO_PIN, &level) == STATUS_SUCCESS) {
        log_info("[radar_gpio] initial level=%lu", (unsigned long)level);
        app_radar_gpio_irq_trigger(level ? 1u : 0u);
    }
}
