#ifndef BSP_PINS_TOF400C_H
#define BSP_PINS_TOF400C_H
#include "stm32f4xx_hal.h"

/* TOF400C 的 SCL/SDA 定义在 i2c_bus.h 共用总线中。 */
#define TOF_INT_PORT GPIOC
#define TOF_INT_PIN GPIO_PIN_2
#define TOF_SHUT_PORT GPIOC
#define TOF_SHUT_PIN GPIO_PIN_1
#define TOF_I2C_ADDR 0x29U
/* 当前原理图中 TOF400C 的 SHUT（6 脚）悬空，没有连接到 PC1。 */
#define TOF_SHUT_CONNECTED 0U

#endif
