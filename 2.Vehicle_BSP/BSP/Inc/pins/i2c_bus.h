#ifndef BSP_PINS_I2C_BUS_H
#define BSP_PINS_I2C_BUS_H
#include "stm32f4xx_hal.h"

/* MPU1 与 TOF400C 共用的 I2C 总线。 */
#define I2C_SCL_PORT GPIOB
#define I2C_SCL_PIN GPIO_PIN_6
#define I2C_SDA_PORT GPIOB
#define I2C_SDA_PIN GPIO_PIN_7

#endif
