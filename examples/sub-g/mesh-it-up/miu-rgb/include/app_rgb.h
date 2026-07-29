#ifndef APP_RGB_H
#define APP_RGB_H

#include <stdint.h>

/*
 * RGB PWM 映射：
 *   R → PWM0 (GPIO1)
 *   G → PWM1 (GPIO6)
 *   B → PWM2 (GPIO7)
 *
 * 占空比范围：0～100（百分比）
 */

void app_rgb_init(void);
void app_rgb_set(uint8_t r, uint8_t g, uint8_t b);

/* 获取当前值 */
void app_rgb_get(uint8_t *r, uint8_t *g, uint8_t *b);

/* 向 Leader 发送 REPORT（定义在 app_net_mgm.c） */
void app_rgb_net_report(void);

#endif /* APP_RGB_H */
