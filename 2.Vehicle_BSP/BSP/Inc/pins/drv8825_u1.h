#ifndef BSP_PINS_DRV8825_U1_H
#define BSP_PINS_DRV8825_U1_H
#include "stm32f4xx_hal.h"

/* U1 DRV8825 驱动器。 */
#define U1_STEP_PORT GPIOD
#define U1_STEP_PIN GPIO_PIN_0
#define U1_DIR_PORT GPIOD
#define U1_DIR_PIN GPIO_PIN_1
#define U1_EN_PORT GPIOC
#define U1_EN_PIN GPIO_PIN_8
#define U1_FAULT_PORT GPIOD
#define U1_FAULT_PIN GPIO_PIN_14

#endif
