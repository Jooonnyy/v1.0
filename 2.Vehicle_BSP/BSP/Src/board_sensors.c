#include "board_internal.h"
#include "pins/i2c_bus.h"

static void i2c_delay(void) { Board_DelayUs(5); }
static void sda(uint8_t high)
{
  HAL_GPIO_WritePin(I2C_SDA_PORT,I2C_SDA_PIN,high?GPIO_PIN_SET:GPIO_PIN_RESET);
}
static HAL_StatusTypeDef scl(uint8_t high)
{
  uint32_t start_counts;
  HAL_GPIO_WritePin(I2C_SCL_PORT,I2C_SCL_PIN,high?GPIO_PIN_SET:GPIO_PIN_RESET);
  if (!high) { i2c_delay(); return HAL_OK; }
  start_counts=DWT->CYCCNT;
  while (HAL_GPIO_ReadPin(I2C_SCL_PORT,I2C_SCL_PIN)==GPIO_PIN_RESET) {
    if ((uint32_t)(DWT->CYCCNT-start_counts)>SystemCoreClock/1000U) return HAL_TIMEOUT;
  }
  i2c_delay(); return HAL_OK;
}
static HAL_StatusTypeDef i2c_start(void)
{
  sda(1);
  if (scl(1)!=HAL_OK) return HAL_TIMEOUT;
  if (HAL_GPIO_ReadPin(I2C_SDA_PORT,I2C_SDA_PIN)==GPIO_PIN_RESET) return HAL_BUSY;
  i2c_delay(); sda(0); i2c_delay(); return scl(0);
}
static void i2c_stop(void)
{
  sda(0); (void)scl(1); sda(1); i2c_delay();
}
static HAL_StatusTypeDef i2c_write_byte(uint8_t value)
{
  HAL_StatusTypeDef status;
  for (uint8_t bit=0;bit<8;bit++) {
    sda((value&0x80U)!=0U); i2c_delay();
    status=scl(1); if (status!=HAL_OK) return status;
    (void)scl(0); value<<=1;
  }
  sda(1); i2c_delay();
  status=scl(1); if (status!=HAL_OK) return status;
  status=HAL_GPIO_ReadPin(I2C_SDA_PORT,I2C_SDA_PIN)==GPIO_PIN_RESET?HAL_OK:HAL_ERROR;
  (void)scl(0); return status;
}
static HAL_StatusTypeDef i2c_read_byte(uint8_t *value,uint8_t ack)
{
  uint8_t v=0;
  sda(1);
  for (uint8_t bit=0;bit<8;bit++) {
    v<<=1;
    if (scl(1)!=HAL_OK) return HAL_TIMEOUT;
    if (HAL_GPIO_ReadPin(I2C_SDA_PORT,I2C_SDA_PIN)==GPIO_PIN_SET) v|=1U;
    (void)scl(0);
  }
  sda(ack?0:1); i2c_delay();
  if (scl(1)!=HAL_OK) return HAL_TIMEOUT;
  (void)scl(0); sda(1); *value=v; return HAL_OK;
}
static HAL_StatusTypeDef send_register(uint16_t reg,uint8_t reg_len)
{
  if (reg_len==2 && i2c_write_byte((uint8_t)(reg>>8))!=HAL_OK) return HAL_ERROR;
  return i2c_write_byte((uint8_t)reg);
}
HAL_StatusTypeDef Board_I2C_Read(uint8_t addr7,uint16_t reg,uint8_t reg_len,
                                  uint8_t *data,uint16_t len)
{
  HAL_StatusTypeDef status=HAL_ERROR;
  if (addr7>0x7fU || (reg_len!=1U && reg_len!=2U) || (!data && len)) return HAL_ERROR;
  if (i2c_start()!=HAL_OK) goto done;
  if (i2c_write_byte((uint8_t)(addr7<<1))!=HAL_OK) goto done;
  if (send_register(reg,reg_len)!=HAL_OK) goto done;
  if (i2c_start()!=HAL_OK) goto done;
  if (i2c_write_byte((uint8_t)((addr7<<1)|1U))!=HAL_OK) goto done;
  for (uint16_t i=0;i<len;i++) {
    if (i2c_read_byte(&data[i],i+1U<len)!=HAL_OK) goto done;
  }
  status=HAL_OK;
done:
  i2c_stop();
  return status;
}

/* 诊断用：分别在两种配置下读 I2C 两条线的电平。
 *
 * 为什么要读两次：STM32 的上下拉（PUPDR）在"开漏输出"模式下是否生效，
 * 是个容易踩的点。用同一条线在两种配置下各读一次，差异就能区分原因：
 *
 *   输入上拉=1 且 开漏=0  -> 开漏模式下内部上拉不生效，
 *                            软件 I2C 根本读不到高电平，必须加外部上拉
 *   两组都是 0            -> 外部有东西把线摁住了（短路 / 未上电的器件钳位）
 *   两组都是 1            -> 总线空闲电平正常，问题在器件侧
 *
 * 读完会把引脚恢复成软件 I2C 用的"开漏输出 + 上拉"。 */
void Board_I2C_LineLevels(uint8_t *scl_in_high, uint8_t *sda_in_high,
                          uint8_t *scl_od_high, uint8_t *sda_od_high)
{
  GPIO_InitTypeDef cfg={0};

  /* 第一遍：输入 + 上拉。这是读 I2C 线最物理正确的方式。 */
  cfg.Pin=(uint32_t)(I2C_SCL_PIN|I2C_SDA_PIN);
  cfg.Mode=GPIO_MODE_INPUT; cfg.Pull=GPIO_PULLUP;
  cfg.Speed=GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(I2C_SCL_PORT,&cfg);
  HAL_Delay(1U);
  if (scl_in_high) *scl_in_high=(HAL_GPIO_ReadPin(I2C_SCL_PORT,I2C_SCL_PIN)==GPIO_PIN_SET)?1U:0U;
  if (sda_in_high) *sda_in_high=(HAL_GPIO_ReadPin(I2C_SDA_PORT,I2C_SDA_PIN)==GPIO_PIN_SET)?1U:0U;

  /* 第二遍：恢复成开漏输出并写高（= 释放总线），再读一次。 */
  HAL_GPIO_WritePin(I2C_SCL_PORT,I2C_SCL_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C_SDA_PORT,I2C_SDA_PIN,GPIO_PIN_SET);
  cfg.Mode=GPIO_MODE_OUTPUT_OD; cfg.Pull=GPIO_PULLUP;
  cfg.Pin=I2C_SCL_PIN; HAL_GPIO_Init(I2C_SCL_PORT,&cfg);
  cfg.Pin=I2C_SDA_PIN; HAL_GPIO_Init(I2C_SDA_PORT,&cfg);
  HAL_Delay(1U);
  if (scl_od_high) *scl_od_high=(HAL_GPIO_ReadPin(I2C_SCL_PORT,I2C_SCL_PIN)==GPIO_PIN_SET)?1U:0U;
  if (sda_od_high) *sda_od_high=(HAL_GPIO_ReadPin(I2C_SDA_PORT,I2C_SDA_PIN)==GPIO_PIN_SET)?1U:0U;
}

/* 只发送地址，不碰设备寄存器。扫描时不能假设目标使用 8 位或 16 位寄存器地址。 */
static HAL_StatusTypeDef i2c_probe_address(uint8_t address7)
{
  HAL_StatusTypeDef status=HAL_ERROR;
  if (i2c_start()!=HAL_OK) goto done;
  status=i2c_write_byte((uint8_t)(address7<<1));
done:
  i2c_stop();
  return status;
}

/* 诊断用：扫描 7 位地址 0x08..0x77，返回应答的个数。
 * 应答地址依次写进 ack_list（最多 capacity_counts 个，可为 NULL）。 */
uint8_t Board_I2C_Scan(uint8_t *ack_list, uint8_t capacity_counts)
{
  uint8_t found_counts=0U;
  uint8_t address7;
  for (address7=0x08U;address7<=0x77U;address7++) {
    if (i2c_probe_address(address7)==HAL_OK) {
      if (ack_list && found_counts<capacity_counts) ack_list[found_counts]=address7;
      found_counts++;
    }
  }
  return found_counts;
}
HAL_StatusTypeDef Board_I2C_Write(uint8_t addr7,uint16_t reg,uint8_t reg_len,
                                   const uint8_t *data,uint16_t len)
{
  HAL_StatusTypeDef status=HAL_ERROR;
  if (addr7>0x7fU || (reg_len!=1U && reg_len!=2U) || (!data && len)) return HAL_ERROR;
  if (i2c_start()!=HAL_OK) goto done;
  if (i2c_write_byte((uint8_t)(addr7<<1))!=HAL_OK) goto done;
  if (send_register(reg,reg_len)!=HAL_OK) goto done;
  for (uint16_t i=0;i<len;i++)
    if (i2c_write_byte(data[i])!=HAL_OK) goto done;
  status=HAL_OK;
done:
  i2c_stop();
  return status;
}
void Board_SensorInit(void)
{
  GPIO_InitTypeDef cfg={0};

  /* PA2/PA3 原为 ADC1_IN2/IN3 预留（信号源从未确认，固件里也没有调用者），
   * 现让给 HWT101 陀螺 USART2（新底板 V2；蓝牙已挪到 UART5 / PC12/PD2，
   * 见 pins/bt_uart.h）。这里**不能**再把它们配成
   * 模拟输入：Board_CommInit() 在 Board_SensorInit() 之前跑，重新配置会把
   * 复用功能覆盖掉，陀螺串口就哑了。将来要做电流检测必须换引脚。 */

  /* PB6/PB7 实现软件 I2C1。开漏输出释放为高电平，
     电气上升时间由板上的外部上拉电阻决定。 */
  HAL_GPIO_WritePin(I2C_SCL_PORT,I2C_SCL_PIN,GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C_SDA_PORT,I2C_SDA_PIN,GPIO_PIN_SET);
  cfg.Mode=GPIO_MODE_OUTPUT_OD; cfg.Pull=GPIO_PULLUP;
  cfg.Speed=GPIO_SPEED_FREQ_HIGH;
  cfg.Pin=I2C_SCL_PIN; HAL_GPIO_Init(I2C_SCL_PORT,&cfg);
  cfg.Pin=I2C_SDA_PIN; HAL_GPIO_Init(I2C_SDA_PORT,&cfg);
}
