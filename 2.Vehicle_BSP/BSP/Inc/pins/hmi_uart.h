#ifndef BSP_PINS_HMI_UART_H
#define BSP_PINS_HMI_UART_H
#include "stm32f4xx_hal.h"

/* 淘晶驰串口屏连接口。 */
#define HMI_TX_PORT GPIOC
#define HMI_TX_PIN GPIO_PIN_6
#define HMI_RX_PORT GPIOC
#define HMI_RX_PIN GPIO_PIN_7
#define HMI_UART_AF GPIO_AF8_USART6

#endif
