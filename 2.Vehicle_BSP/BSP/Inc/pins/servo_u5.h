#ifndef BSP_PINS_SERVO_U5_H
#define BSP_PINS_SERVO_U5_H
#include "stm32f4xx_hal.h"

/* U5 舵机（机械臂 / 夹爪）信号脚。
 *
 * 新底板 V2 as-built：网络 `SERVO_J_PWM` -> **PB14 / TIM12_CH1**，50 Hz。
 * 两路舵机（U5 夹爪、U6 载物台）共用 TIM12：同一个 ARR，天然同频，见 `pins/servo_u6.h`。
 *
 * 历史（老板，2026-10）：舵机先接 PE5/TIM9_CH1，PE5 实测烧坏后飞线到 PA7/TIM14_CH1，
 * 当时用 `U5_SERVO_USE_PA7` 开关切换。新板把这两套都作废——PA7/SPI1 还给 SPI1，
 * PE6 改作 B 轴 STEP（TIM9_CH2）——所以开关已删除，不要再加回来。 */
#define U5_PWM_PORT      GPIOB
#define U5_PWM_PIN       GPIO_PIN_14
#define U5_PWM_AF        GPIO_AF9_TIM12
#define U5_PWM_CHANNEL   TIM_CHANNEL_1
#define U5_PWM_TIMER     TIM12

#endif
