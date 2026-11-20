#ifndef BSP_PINS_SERVO_U6_H
#define BSP_PINS_SERVO_U6_H
#include "stm32f4xx_hal.h"

/* U6 舵机（载物台）信号脚。
 *
 * 新底板 V2 as-built：网络 `SERVO_X_PWM` -> **PB15 / TIM12_CH2**，50 Hz。
 * 与 U5 共用 TIM12（见 `pins/servo_u5.h`）：同一个 PSC/ARR，两路都是 50 Hz。
 *
 * 历史：老板上是 PE6 / TIM9_CH2；新板把 PE6 改作 B 轴 STEP，舵机搬到 PB15，
 * 所以 TIM9 与舵机不再有任何关系。 */
#define U6_PWM_PORT      GPIOB
#define U6_PWM_PIN       GPIO_PIN_15
#define U6_PWM_AF        GPIO_AF9_TIM12
#define U6_PWM_CHANNEL   TIM_CHANNEL_2

#endif
