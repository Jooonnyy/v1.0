#ifndef BSP_PINS_DEBUG_UART_H
#define BSP_PINS_DEBUG_UART_H
#include "stm32f4xx_hal.h"

/* 核心板调试串口。 */
#define DEBUG_TX_PORT GPIOA
#define DEBUG_TX_PIN GPIO_PIN_9
#define DEBUG_RX_PORT GPIOA
#define DEBUG_RX_PIN GPIO_PIN_10
#define DEBUG_UART_AF GPIO_AF7_USART1

#endif
