#include "board_internal.h"
#include "board_encoder.h"
#include "board_wheel.h"
#include "pins/tb6612_u2.h"
#include "pins/tb6612_u3.h"
#include "pins/drv8825_u1.h"
#include "pins/drv8825_u4.h"
#include "pins/tof400c.h"
#include "pins/mpu6050_mpu1.h"
#include "pins/encoder_d.h"
#include "pins/led_status.h"
#include "pins/key.h"
#include "pins/limit_top.h"
#include "pins/limit_bottom.h"
#include "pins/home.h"

static volatile uint32_t input_events[6];

static void pin_init(GPIO_TypeDef *port, uint16_t pin, uint32_t mode,
                     uint32_t pull, uint32_t speed)
{
  GPIO_InitTypeDef cfg = {0};
  cfg.Pin=pin; cfg.Mode=mode; cfg.Pull=pull;
  cfg.Speed=speed;
  HAL_GPIO_Init(port,&cfg);
}
static void output_init(GPIO_TypeDef *port,uint16_t pin,GPIO_PinState initial,
                        uint32_t mode)
{
  HAL_GPIO_WritePin(port,pin,initial);
  pin_init(port,pin,mode,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
}
/* 四路电机方向与待机的 GPIO 初始化，完整工程与电机相关台架共用。 */
static void motor_gpio_init(void)
{
  output_init(U2_STBY_PORT,U2_STBY_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U3_STBY_PORT,U3_STBY_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U2_AIN1_PORT,U2_AIN1_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U2_AIN2_PORT,U2_AIN2_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U2_BIN1_PORT,U2_BIN1_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U2_BIN2_PORT,U2_BIN2_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U3_AIN1_PORT,U3_AIN1_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U3_AIN2_PORT,U3_AIN2_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U3_BIN1_PORT,U3_BIN1_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U3_BIN2_PORT,U3_BIN2_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
}
void Board_DelayUs(uint32_t us)
{
  uint32_t start_counts=DWT->CYCCNT;
  uint32_t delay_counts=(SystemCoreClock/1000000U)*us;
  while ((uint32_t)(DWT->CYCCNT-start_counts)<delay_counts) {}
}
HAL_StatusTypeDef Board_GPIOInit(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_SYSCFG_CLK_ENABLE();
  CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT=0; DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;

  /* 低有效 nEN 默认置高，其余驱动输出默认置低。 */
  output_init(U1_EN_PORT,U1_EN_PIN,GPIO_PIN_SET,GPIO_MODE_OUTPUT_PP);
  output_init(U4_EN_PORT,U4_EN_PIN,GPIO_PIN_SET,GPIO_MODE_OUTPUT_PP);
  output_init(U1_STEP_PORT,U1_STEP_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U1_DIR_PORT,U1_DIR_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U4_STEP_PORT,U4_STEP_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  output_init(U4_DIR_PORT,U4_DIR_PIN,GPIO_PIN_RESET,GPIO_MODE_OUTPUT_PP);
  motor_gpio_init();
#if TOF_SHUT_CONNECTED
  /* 仅在原理图确实连接 SHUT 时才驱动；开漏可避免与核心板按键发生推挽冲突。 */
  HAL_GPIO_WritePin(TOF_SHUT_PORT,TOF_SHUT_PIN,GPIO_PIN_RESET);
  pin_init(TOF_SHUT_PORT,TOF_SHUT_PIN,GPIO_MODE_OUTPUT_OD,
           GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
#endif

  pin_init(KEY_PORT,KEY_PIN,GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(MPU1_INT_PORT,MPU1_INT_PIN,GPIO_MODE_IT_RISING,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
  pin_init(TOF_INT_PORT,TOF_INT_PIN,GPIO_MODE_IT_RISING,GPIO_NOPULL,GPIO_SPEED_FREQ_LOW);
  pin_init(HOME_PORT,HOME_PIN,GPIO_MODE_IT_RISING_FALLING,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(LIMIT_TOP_PORT,LIMIT_TOP_PIN,GPIO_MODE_IT_RISING_FALLING,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(LIMIT_BOTTOM_PORT,LIMIT_BOTTOM_PIN,GPIO_MODE_IT_RISING_FALLING,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(ENCD_A_PORT,ENCD_A_PIN,GPIO_MODE_IT_RISING_FALLING,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(ENCD_B_PORT,ENCD_B_PIN,GPIO_MODE_IT_RISING_FALLING,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(U1_FAULT_PORT,U1_FAULT_PIN,GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  pin_init(U4_FAULT_PORT,U4_FAULT_PIN,GPIO_MODE_INPUT,GPIO_PULLUP,GPIO_SPEED_FREQ_LOW);
  HAL_NVIC_SetPriority(EXTI2_IRQn,6,0); HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_NVIC_SetPriority(EXTI3_IRQn,6,0); HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn,5,0); HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,6,0); HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  return HAL_OK;
}
HAL_StatusTypeDef Board_Init(void)
{
  if (Board_GPIOInit()!=HAL_OK) return HAL_ERROR;
  if (Board_MotionInit()!=HAL_OK) return HAL_ERROR;
  /* 底盘运动层必须在 tick 起来之前初始化：第一个 tick 就会调 Wheel_Update()。
   * 它把四路输出保持在 0 且未使能，所以这里不会让车动。 */
  Wheel_Init();
  /* 编码器测速 tick 必须在 Board_MotionInit() 之后：它要读 TIM2/3/4。 */
  if (Encoder_Init()!=HAL_OK) return HAL_ERROR;
  if (Board_CommInit()!=HAL_OK) return HAL_ERROR;
  Board_SensorInit();
  return HAL_OK;
}
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin==ENCD_A_PIN || pin==ENCD_B_PIN) Board_EncoderDEdge();
  else if (pin==MPU1_INT_PIN) input_events[INPUT_MPU_IRQ]++;
  else if (pin==TOF_INT_PIN) input_events[INPUT_TOF_IRQ]++;
  else if (pin==LIMIT_TOP_PIN) input_events[INPUT_LIMIT_TOP]++;
  else if (pin==LIMIT_BOTTOM_PIN) input_events[INPUT_LIMIT_BOTTOM]++;
  else if (pin==HOME_PIN) input_events[INPUT_HOME]++;
}
uint8_t Input_Active(InputId id)
{
  GPIO_TypeDef *port; uint16_t pin; uint8_t active_low=1;
  switch (id) {
    case INPUT_KEY: port=KEY_PORT; pin=KEY_PIN; break;
    case INPUT_LIMIT_TOP: port=LIMIT_TOP_PORT; pin=LIMIT_TOP_PIN; break;
    case INPUT_LIMIT_BOTTOM: port=LIMIT_BOTTOM_PORT; pin=LIMIT_BOTTOM_PIN; break;
    case INPUT_HOME: port=HOME_PORT; pin=HOME_PIN; break;
    case INPUT_MPU_IRQ: port=MPU1_INT_PORT; pin=MPU1_INT_PIN; active_low=0; break;
    case INPUT_TOF_IRQ: port=TOF_INT_PORT; pin=TOF_INT_PIN; active_low=0; break;
    default: return 0;
  }
  return HAL_GPIO_ReadPin(port,pin)==(active_low?GPIO_PIN_RESET:GPIO_PIN_SET);
}
uint32_t Input_EventCount(InputId id)
{
  if ((unsigned)id>=6U) return 0;
  return input_events[id];
}

/* 短临界区保留进入前的中断屏蔽状态，可嵌套使用。 */
uint32_t Board_NowMs(void) { return HAL_GetTick(); }
uint32_t Board_EnterCritical(void)
{
  uint32_t saved_mask = __get_PRIMASK();
  __disable_irq();
  return saved_mask;
}
void Board_ExitCritical(uint32_t saved_mask) { __set_PRIMASK(saved_mask); }

/* 板载状态 LED（PC0，低电平点亮）。配置成推挽输出并先写"灭"，所以初始化本身不会闪灯。
 * 自带 GPIOC 时钟：不依赖其它初始化的调用顺序。
 * 只由显式调用它的入口使用（当前只有 RIG 台架）；生产固件不调用，PC0 保持复位默认状态。 */
void Led_StatusInit(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  output_init(LED_STATUS_PORT,LED_STATUS_PIN,LED_STATUS_OFF_LEVEL,GPIO_MODE_OUTPUT_PP);
}
void Led_StatusSet(uint8_t on)
{
  HAL_GPIO_WritePin(LED_STATUS_PORT,LED_STATUS_PIN,
                    (on!=0U)?LED_STATUS_ON_LEVEL:LED_STATUS_OFF_LEVEL);
}
void Led_StatusToggle(void)
{
  HAL_GPIO_TogglePin(LED_STATUS_PORT,LED_STATUS_PIN);
}
