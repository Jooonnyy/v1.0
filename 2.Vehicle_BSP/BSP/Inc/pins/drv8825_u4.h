#ifndef BSP_PINS_DRV8825_U4_H
#define BSP_PINS_DRV8825_U4_H
#include "stm32f4xx_hal.h"

/* U4 DRV8825 驱动器。 */
#define U4_STEP_PORT GPIOD
#define U4_STEP_PIN GPIO_PIN_4
#define U4_DIR_PORT GPIOD
#define U4_DIR_PIN GPIO_PIN_5
#define U4_EN_PORT GPIOC
#define U4_EN_PIN GPIO_PIN_9
#define U4_FAULT_PORT GPIOD
#define U4_FAULT_PIN GPIO_PIN_15

#endif
