#ifndef BSP_PINS_RPI_UART_H
#define BSP_PINS_RPI_UART_H
#include "stm32f4xx_hal.h"

/* 树莓派串口连接口。 */
#define RPI_TX_PORT GPIOC
#define RPI_TX_PIN GPIO_PIN_10
#define RPI_RX_PORT GPIOC
#define RPI_RX_PIN GPIO_PIN_11
#define RPI_UART_AF GPIO_AF7_USART3

#endif
