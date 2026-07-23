#ifndef __APP_RADAR_H
#define __APP_RADAR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化雷达 GPIO 引脚（在 otrInitUser() 中调用）
 */
void app_radar_init(void);

/**
 * @brief 直接发送雷达状态上报（在 OT 任务上下文中调用）
 * @param status  0 = 无人，1 = 有人
 */
void radar_report_send(uint8_t status);

/**
 * @brief GPIO 中断触发入口（在中断回调中调用，会安全投递到 OT 任务）
 * @param status  当前 GPIO 电平（即雷达输出状态）
 */
void app_radar_irq_trigger(uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* __APP_RADAR_H */
