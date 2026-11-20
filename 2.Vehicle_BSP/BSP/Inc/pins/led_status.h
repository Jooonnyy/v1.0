#ifndef BSP_PINS_LED_STATUS_H
#define BSP_PINS_LED_STATUS_H
#include "stm32f4xx_hal.h"

/* 核心板（FK407M3-VET6）板载用户 LED，接在 PC0，**低电平点亮**。
   极性取自原厂 LED 例程：Project/1.LED闪烁/Core/Inc/led.h 的 LED1_ON()/LED1_OFF()。
   该脚不属于任何外设，只当"固件还在跑"的状态指示用（见 board.h 的 Led_Status*）。 */
#define LED_STATUS_PORT GPIOC
#define LED_STATUS_PIN GPIO_PIN_0
#define LED_STATUS_ON_LEVEL GPIO_PIN_RESET
#define LED_STATUS_OFF_LEVEL GPIO_PIN_SET

#endif
