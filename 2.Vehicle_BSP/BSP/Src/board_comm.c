#include "board_internal.h"
#include "pins/debug_uart.h"
#include "pins/rpi_uart.h"
#include "pins/hmi_uart.h"
#include "pins/bt_uart.h"
#include "pins/imu_uart.h"

/* 串口路数，必须与 board.h 的 SerialPort 枚举、以及下面的 ports[] 一一对应。
   加一路串口时同步改这三处，别只改数组。 */
#define PORT_COUNT 5U
#define RX_SIZE 256U
typedef struct {
  USART_TypeDef *hw;
  volatile uint16_t head, tail;
  volatile uint32_t dropped;
  uint8_t rx[RX_SIZE];
} PortState;
static PortState ports[PORT_COUNT]={{USART1,0,0,0,{0}},
                                    {USART3,0,0,0,{0}},
                                    {USART6,0,0,0,{0}},
                                    {UART5,0,0,0,{0}},
                                    {USART2,0,0,0,{0}}};

static void uart_pins(GPIO_TypeDef *port,uint16_t pins,uint32_t af)
{
  GPIO_InitTypeDef cfg={0};
  cfg.Pin=pins; cfg.Mode=GPIO_MODE_AF_PP; cfg.Pull=GPIO_PULLUP;
  cfg.Speed=GPIO_SPEED_FREQ_VERY_HIGH; cfg.Alternate=af;
  HAL_GPIO_Init(port,&cfg);
}
static void uart_start(USART_TypeDef *hw,uint32_t bus_hz,uint32_t baud)
{
  hw->CR1=0; hw->CR2=0; hw->CR3=0;
  hw->BRR=(bus_hz+baud/2U)/baud; /* 使用 16 倍过采样。 */
  hw->CR1=USART_CR1_UE|USART_CR1_TE|USART_CR1_RE|USART_CR1_RXNEIE;
}
HAL_StatusTypeDef Board_CommInit(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_UART5_CLK_ENABLE();  /* 蓝牙：PC12/PD2 */
  __HAL_RCC_USART2_CLK_ENABLE(); /* 陀螺 HWT101：PA2/PA3 */
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();
  uart_pins(DEBUG_TX_PORT,DEBUG_TX_PIN,DEBUG_UART_AF);
  uart_pins(DEBUG_RX_PORT,DEBUG_RX_PIN,DEBUG_UART_AF);
  uart_pins(RPI_TX_PORT,RPI_TX_PIN,RPI_UART_AF);
  uart_pins(RPI_RX_PORT,RPI_RX_PIN,RPI_UART_AF);
  uart_pins(HMI_TX_PORT,HMI_TX_PIN,HMI_UART_AF);
  uart_pins(HMI_RX_PORT,HMI_RX_PIN,HMI_UART_AF);
  uart_pins(BT_TX_PORT,BT_TX_PIN,BT_UART_AF);
  uart_pins(BT_RX_PORT,BT_RX_PIN,BT_UART_AF);
  uart_pins(IMU_TX_PORT,IMU_TX_PIN,IMU_UART_AF);
  uart_pins(IMU_RX_PORT,IMU_RX_PIN,IMU_UART_AF);
  uart_start(USART1,HAL_RCC_GetPCLK2Freq(),BOARD_DEBUG_BAUD);
  /* UART5/USART2/USART3 挂在 APB1，USART1/USART6 挂在 APB2，分频系数不同，必须分别取频率。 */
  uart_start(UART5,HAL_RCC_GetPCLK1Freq(),BOARD_BT_BAUD);
  uart_start(USART2,HAL_RCC_GetPCLK1Freq(),BOARD_IMU_BAUD);
  uart_start(USART3,HAL_RCC_GetPCLK1Freq(),BOARD_RPI_BAUD);
  uart_start(USART6,HAL_RCC_GetPCLK2Freq(),BOARD_HMI_BAUD);
  HAL_NVIC_SetPriority(USART1_IRQn,7,0); HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(UART5_IRQn,7,0); HAL_NVIC_EnableIRQ(UART5_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn,7,0); HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn,7,0); HAL_NVIC_EnableIRQ(USART3_IRQn);
  HAL_NVIC_SetPriority(USART6_IRQn,7,0); HAL_NVIC_EnableIRQ(USART6_IRQn);
  return HAL_OK;
}
void Board_SerialIRQ(SerialPort port)
{
  PortState *p;
  uint32_t status;
  uint8_t byte;
  uint16_t next;
  if ((unsigned)port>=PORT_COUNT) return;
  p=&ports[port]; status=p->hw->SR;
  if (status&(USART_SR_RXNE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
    byte=(uint8_t)p->hw->DR; /* 依次读取 SR、DR 可清除 RXNE 和错误标志。 */
    if (!(status&USART_SR_RXNE)) return;
    next=(uint16_t)((p->head+1U)&(RX_SIZE-1U));
    if (next==p->tail) p->dropped++;
    else { p->rx[p->head]=byte; p->head=next; }
  }
}
int Serial_ReadByte(SerialPort port)
{
  PortState *p;
  uint8_t byte;
  if ((unsigned)port>=PORT_COUNT) return -1;
  p=&ports[port];
  if (p->tail==p->head) return -1;
  byte=p->rx[p->tail];
  p->tail=(uint16_t)((p->tail+1U)&(RX_SIZE-1U));
  return byte;
}
uint32_t Serial_Dropped(SerialPort port)
{
  return (unsigned)port>=PORT_COUNT?0:ports[port].dropped;
}
HAL_StatusTypeDef Serial_Write(SerialPort port,const uint8_t *data,
                                uint16_t len,uint32_t timeout_ms)
{
  USART_TypeDef *hw;
  uint32_t start_ms=HAL_GetTick();
  if ((unsigned)port>=PORT_COUNT || (!data && len)) return HAL_ERROR;
  if (!len) return HAL_OK;
  hw=ports[port].hw;
  for (uint16_t i=0;i<len;i++) {
    while (!(hw->SR&USART_SR_TXE)) {
      if ((uint32_t)(HAL_GetTick()-start_ms)>timeout_ms) return HAL_TIMEOUT;
    }
    hw->DR=data[i];
  }
  while (!(hw->SR&USART_SR_TC)) {
    if ((uint32_t)(HAL_GetTick()-start_ms)>timeout_ms) return HAL_TIMEOUT;
  }
  return HAL_OK;
}
