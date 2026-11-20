#ifndef BSP_PINS_IMU_UART_H
#define BSP_PINS_IMU_UART_H
#include "stm32f4xx_hal.h"

/* HWT101 陀螺串口（维特 0x55 帧，115200，约 100 Hz）。
   新底板 V2：PA2 = USART2_TX（接模块 RX）、PA3 = USART2_RX（接模块 TX），AF7。
   同一个 4P 座上 PA2/PA3 与蓝牙（UART5 / PC12/PD2）是**两路不同的串口**，
   不再像老板那样抢 USART2；详见 Docs/交接文档_新底板V2.md §2.4。

   ⚠ PA2/PA3 不是 5V 容忍脚，只能接 3.3V 逻辑；模块 VCC 走 3V3_IO。 */
#define IMU_TX_PORT GPIOA
#define IMU_TX_PIN  GPIO_PIN_2
#define IMU_RX_PORT GPIOA
#define IMU_RX_PIN  GPIO_PIN_3
#define IMU_UART_AF GPIO_AF7_USART2

#endif
