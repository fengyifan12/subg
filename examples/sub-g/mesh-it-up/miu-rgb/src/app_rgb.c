/*
 * app_rgb.c (miu-rgb) — RGB PWM 控制
 *
 * 通道映射：
 *   R → PWM0，GPIO1
 *   G → PWM1，GPIO6
 *   B → PWM2，GPIO7
 *
 * 使用 Format-1 单通道 PWM（hosal_pwm_init_fmt1 / hosal_pwm_fmt1_duty）。
 * 频率 1 kHz，占空比 0～100（百分比）。
 */

#include "app_rgb.h"
#include "hosal_pwm.h"
#include "log.h"

#define RGB_PWM_FREQUENCY   16000    /* 16 kHz */

#define RGB_R_PIN   1   /* GPIO1 → PWM0 */
#define RGB_G_PIN   6   /* GPIO6 → PWM1 */
#define RGB_B_PIN   7   /* GPIO7 → PWM2 */

static hosal_pwm_dev_t s_pwm_r;
static hosal_pwm_dev_t s_pwm_g;
static hosal_pwm_dev_t s_pwm_b;

static uint8_t s_r = 0;
static uint8_t s_g = 0;
static uint8_t s_b = 0;

static void pwm_channel_init(hosal_pwm_dev_t *dev, uint32_t id, uint32_t pin)
{
    dev->config.id        = id;
    dev->config.frequency = RGB_PWM_FREQUENCY;
    dev->config.pin_out   = pin;
    hosal_pwm_init_fmt0(dev);
    hosal_pwm_fmt0_duty(id, 0);
    hosal_pwm_start(id);
}

void app_rgb_init(void)
{
    log_info("[rgb] PWM init: R=GPIO%d(PWM%u) G=GPIO%d(PWM%u) B=GPIO%d(PWM%u)",
             RGB_R_PIN, HOSAL_PWM_ID_0,
             RGB_G_PIN, HOSAL_PWM_ID_1,
             RGB_B_PIN, HOSAL_PWM_ID_2);

    pwm_channel_init(&s_pwm_r, HOSAL_PWM_ID_0, RGB_R_PIN);
    pwm_channel_init(&s_pwm_g, HOSAL_PWM_ID_1, RGB_G_PIN);
    pwm_channel_init(&s_pwm_b, HOSAL_PWM_ID_2, RGB_B_PIN);
}

void app_rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (r > 100) r = 100;
    if (g > 100) g = 100;
    if (b > 100) b = 100;

    r = 100 - r;
    g = 100 - g;
    b = 100 - b;

    s_r = r;
    s_g = g;
    s_b = b;

    hosal_pwm_fmt0_duty(HOSAL_PWM_ID_0, r);
    hosal_pwm_fmt0_duty(HOSAL_PWM_ID_1, g);
    hosal_pwm_fmt0_duty(HOSAL_PWM_ID_2, b);

    log_info("[rgb] set R=%u G=%u B=%u", r, g, b);
}

void app_rgb_get(uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = s_r;
    *g = s_g;
    *b = s_b;
}
