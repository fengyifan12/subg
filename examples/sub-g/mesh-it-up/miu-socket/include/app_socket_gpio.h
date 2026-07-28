#ifndef APP_SOCKET_GPIO_H
#define APP_SOCKET_GPIO_H

#include <stdint.h>

/*
 * GPIO0 — 输入，常态低电平（下拉）
 *   检测模块物理按键按下（上升沿）→ 翻转 GPIO1 继电器状态并上报
 *
 * GPIO1 — 推挽输出，控制继电器
 *   HIGH (1) = 吸合（ON），LOW (0) = 断开（OFF），上电默认 OFF
 *
 * app_socket_relay_toggle(): 翻转继电器状态并上报（PC CONTROL 或按键均调用此函数）
 * app_socket_relay_state_get(): 返回当前继电器状态（0/1）
 */

void    app_socket_gpio_init(void);
void    app_socket_relay_toggle(void);
uint8_t app_socket_relay_state_get(void);

#endif /* APP_SOCKET_GPIO_H */
