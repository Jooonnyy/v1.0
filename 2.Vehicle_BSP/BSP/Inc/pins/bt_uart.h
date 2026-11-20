#ifndef BSP_PINS_BT_UART_H
#define BSP_PINS_BT_UART_H
#include "stm32f4xx_hal.h"

/* 蓝牙透传模块（HC-05）连接口。
   新底板 V2：SPARE 4P 座 = UART5，PC12 = UART5_TX（AF8，接模块 RXD）、
   PD2 = UART5_RX（AF8，接模块 TXD）。USART2（PA2/PA3）已让给 HWT101 陀螺，
   蓝牙与陀螺各占一路，不再抢同一串口；详见 Docs/交接文档_新底板V2.md。 */
#define BT_TX_PORT GPIOC
#define BT_TX_PIN GPIO_PIN_12
#define BT_RX_PORT GPIOD
#define BT_RX_PIN GPIO_PIN_2
#define BT_UART_AF GPIO_AF8_UART5

#endif
