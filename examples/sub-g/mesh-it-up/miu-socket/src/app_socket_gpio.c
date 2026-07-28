/*
 * app_socket_gpio.c (miu-socket) — 继电器 GPIO 控制
 *
 * GPIO0: 输入，常态低电平（下拉），检测模块物理按键按下（上升沿）
 *   → 在 OT 任务上下文中翻转 GPIO1 状态并上报
 *
 * GPIO1: 推挽输出，直接控制继电器
 *   HIGH (1) = 吸合（ON），LOW (0) = 断开（OFF），上电默认 OFF
 *
 * 翻转动作（app_socket_relay_toggle）由两路触发：
 *   1. GPIO0 上升沿（物理按键按下）
 *   2. 网络 CONTROL 命令（app_task.c 收到 toggle 字段后调用）
 *
 * 注意（RT584）：
 *   hosal_gpio_cfg_input() 内部强制 PULLUP_100K，
 *   需在 cfg_input 之后调用 hosal_pin_set_pullopt() 改为下拉。
 *   必须 NVIC_EnableIRQ(Gpio_IRQn)，仅 hosal_gpio_int_enable() 不够。
 */

#include <stdint.h>
#include <FreeRTOS.h>
#include <miu_port.h>
#include "mcu.h"
#include "hosal_gpio.h"
#include "hosal_sysctrl.h"
#include "log.h"
#include "app_socket_gpio.h"

#ifndef CONFIG_SOCKET_BTN_INPUT_PIN
#define CONFIG_SOCKET_BTN_INPUT_PIN     0   /* GPIO0: 物理按键输入，常态低电平 */
#endif

#ifndef CONFIG_SOCKET_RELAY_CTRL_PIN
#define CONFIG_SOCKET_RELAY_CTRL_PIN    1   /* GPIO1: 继电器控制输出 */
#endif

/* 继电器当前状态：0=OFF，1=ON */
static volatile uint8_t s_relay_state = 0;

/* 声明上层网络上报函数（定义在 app_net_mgm.c） */
extern void app_socket_net_report_state(uint8_t state);

/* -----------------------------------------------------------------------
 * GPIO1 — 继电器控制输出
 * ----------------------------------------------------------------------- */
uint8_t app_socket_relay_state_get(void)
{
    return s_relay_state;
}

/*
 * 翻转继电器状态并上报。
 * 调用方必须在 OT 任务上下文（app_udp_comm_json_process 或 ot_app_task_post 回调）。
 */
void app_socket_relay_toggle(void)
{
    s_relay_state ^= 1u;
    if (s_relay_state) {
        hosal_gpio_pin_set(CONFIG_SOCKET_RELAY_CTRL_PIN);
    } else {
        hosal_gpio_pin_clear(CONFIG_SOCKET_RELAY_CTRL_PIN);
    }
    log_info("[socket_gpio] relay -> %s (GPIO%d=%d)",
             s_relay_state ? "ON" : "OFF",
             CONFIG_SOCKET_RELAY_CTRL_PIN, s_relay_state);
    app_socket_net_report_state(s_relay_state);
}

/* -----------------------------------------------------------------------
 * GPIO0 — 物理按键输入
 * ----------------------------------------------------------------------- */
static void btn_toggle_task(void *arg)
{
    (void)arg;
    app_socket_relay_toggle();
    log_info("[socket_gpio] btn_toggle_task");
}

/* ISR 上下文：仅 post 到 OT 任务，禁止直接调用 OT API */
static void btn_input_gpio_cb(uint32_t pin_number, void *isr_param)
{
    (void)pin_number;
    (void)isr_param;
    ot_app_task_post(btn_toggle_task, NULL);
}

/* -----------------------------------------------------------------------
 * 初始化（在 otrInitUser() 中调用）
 * ----------------------------------------------------------------------- */
void app_socket_gpio_init(void)
{
    log_info("[socket_gpio] init (BTN_IN=GPIO%d, RELAY_OUT=GPIO%d)",
             CONFIG_SOCKET_BTN_INPUT_PIN, CONFIG_SOCKET_RELAY_CTRL_PIN);

    /* GPIO1: 推挽输出，初始 LOW（继电器断开） */
    s_relay_state = 0;
    hosal_gpio_cfg_output(CONFIG_SOCKET_RELAY_CTRL_PIN);
    hosal_gpio_pin_clear(CONFIG_SOCKET_RELAY_CTRL_PIN);

    /* GPIO0: 输入，上升沿中断（按键按下 LOW→HIGH） */
    {
        hosal_gpio_input_config_t pin_cfg = {
            .usr_cb       = btn_input_gpio_cb,
            .param        = NULL,
            .pin_int_mode = HOSAL_GPIO_PIN_INT_EDGE_RISING,
        };
        hosal_gpio_cfg_input(CONFIG_SOCKET_BTN_INPUT_PIN, pin_cfg);
    }
    hosal_pin_set_pullopt(CONFIG_SOCKET_BTN_INPUT_PIN, HOSAL_PULL_DOWN_100K);

    /* 使能去抖，避免按键抖动导致多次触发 */
    hosal_gpio_debounce_enable(CONFIG_SOCKET_BTN_INPUT_PIN);

    hosal_gpio_int_enable(CONFIG_SOCKET_BTN_INPUT_PIN);
    NVIC_SetPriority(Gpio_IRQn, 4);
    NVIC_EnableIRQ(Gpio_IRQn);
}
