#ifndef APP_RADAR_GPIO_H
#define APP_RADAR_GPIO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化雷达存在感应 GPIO（在 otrInitUser() 中调用）
 */
void app_radar_gpio_init(void);

/**
 * @brief GPIO 中断触发入口（在中断回调中调用，会安全投递到 OT 任务）
 * @param presence  当前 GPIO 电平：1 = 有人，0 = 无人
 */
void app_radar_gpio_irq_trigger(uint8_t presence);

#ifdef __cplusplus
}
#endif

#endif /* APP_RADAR_GPIO_H */
