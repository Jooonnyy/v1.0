#ifndef BSP_PINS_MPU6050_MPU1_H
#define BSP_PINS_MPU6050_MPU1_H
#include "stm32f4xx_hal.h"

/* MPU1 为 MPU6050，SCL/SDA 定义在 i2c_bus.h 共用总线中。 */
#define MPU1_INT_PORT GPIOC
#define MPU1_INT_PIN GPIO_PIN_3
#define MPU1_I2C_ADDR 0x68U

#endif
