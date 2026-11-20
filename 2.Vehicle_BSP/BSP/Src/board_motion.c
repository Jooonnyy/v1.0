#include "board_internal.h"
/* 引脚头文件名仍是 tb6612_*（沿用 2026-09 的引脚事实），但 2026-10 起板上
 * 实际装的是 AT8236 双路模块：每通道只有 PWM + DIR 两个输入，没有 AIN2/BIN2/STBY。
 * 这些宏里的 AIN2/BIN2/STBY 只保留作历史引脚定义，Motor_Set() 不再使用。 */
#include "pins/tb6612_u2.h"
#include "pins/tb6612_u3.h"
#include "pins/drv8825_u1.h"
#include "pins/drv8825_u4.h"
#include "pins/servo_u5.h"
#include "pins/servo_u6.h"
#include "pins/encoder_a.h"
#include "pins/encoder_b.h"
#include "pins/encoder_c.h"
#include "pins/encoder_d.h"

static TIM_HandleTypeDef pwm, servo, enc_a, enc_b, enc_c;
/* 两路舵机（U5 夹爪 PB14 / U6 载物台 PB15）共用 TIM12：
 * 同一个 PSC/ARR，两路都是 50 Hz，见 pins/servo_u5.h 与 pins/servo_u6.h。 */
static uint8_t servo_ready;     /* Board_MotionInit() 里舵机定时器配好之前不许调用 */
static uint8_t servo_running;
static uint8_t servo2_running;

/* AT8236 的 DIR 极性：1 = 正转时 DIR 输出高，0 = 正转时 DIR 输出低。
 * 如果整车测试发现所有轮子都反了，改这里比改四路符号省事；
 * 单路反了则用 board_wheel.h 的 WHEEL_MOTOR_SIGN_x 或交换电机线。 */
#define MOTOR_DIR_FORWARD_LEVEL 1U
#if (MOTOR_DIR_FORWARD_LEVEL != 0U && MOTOR_DIR_FORWARD_LEVEL != 1U)
#error MOTOR_DIR_FORWARD_LEVEL must be 0 or 1
#endif

/* AT8236 模块的输入结构：
 *   1 = 模块是 IN1/IN2 两输入（AIN1=IN1，PWM1=IN2），没有独立 PWM 脚；
 *   0 = 模块是 PWM+DIR（PWM1 是专用 PWM，AIN1 是方向）。
 * 现象“D a 0 全速、D a 300 稍慢、反向很小不走”就是 IN1/IN2 结构：
 *   IN1=1、IN2=PWM 时，PWM 高是刹车、低才是正转，所以正转要取反；
 *   IN1=0、IN2=PWM 时，PWM 高是反转，反向不用取反。
 * 如果模块丝印明确写 PWM/DIR，再把它改成 0。 */
#define MOTOR_AT8236_IN1_IN2 1U
#if (MOTOR_AT8236_IN1_IN2 != 0U && MOTOR_AT8236_IN1_IN2 != 1U)
#error MOTOR_AT8236_IN1_IN2 must be 0 or 1
#endif
static volatile int32_t encoder_d_counts;
static volatile uint8_t d_previous_state;
/* 4 路累计计数。**唯一写入点是 Encoder_SampleCounts()**（由 board_encoder.c 的
 * 100 Hz tick 调用）；读侧 Encoder_ReadCounts() 只做原子读取，绝不改变状态。
 * TIM3/TIM4 是 16 位计数器，靠下面的 last_raw 把计数扩展到 32 位；
 * 这个扩展必须只在 tick 里做一次，否则中断和主循环会互相吃掉增量。 */
static volatile int32_t encoder_total_counts[4];
static uint16_t encoder_last_raw_b, encoder_last_raw_c;

static void af(GPIO_TypeDef *port, uint16_t pins, uint32_t alternate)
{
  GPIO_InitTypeDef cfg = {0};
  cfg.Pin=pins; cfg.Mode=GPIO_MODE_AF_PP; cfg.Pull=GPIO_NOPULL;
  cfg.Speed=GPIO_SPEED_FREQ_HIGH; cfg.Alternate=alternate;
  HAL_GPIO_Init(port,&cfg);
}

static HAL_StatusTypeDef pwm_channel(TIM_HandleTypeDef *timer, uint32_t channel)
{
  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode=TIM_OCMODE_PWM1; oc.Pulse=0;
  oc.OCPolarity=TIM_OCPOLARITY_HIGH; oc.OCFastMode=TIM_OCFAST_DISABLE;
  return HAL_TIM_PWM_ConfigChannel(timer,&oc,channel);
}

static HAL_StatusTypeDef encoder_init(TIM_HandleTypeDef *timer,
                                      TIM_TypeDef *instance, uint32_t period_counts)
{
  TIM_Encoder_InitTypeDef cfg = {0};
  timer->Instance=instance;
  timer->Init.Prescaler=0; timer->Init.CounterMode=TIM_COUNTERMODE_UP;
  timer->Init.Period=period_counts; timer->Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  timer->Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
  cfg.EncoderMode=TIM_ENCODERMODE_TI12;
  cfg.IC1Polarity=TIM_ICPOLARITY_RISING;
  cfg.IC1Selection=TIM_ICSELECTION_DIRECTTI;
  cfg.IC1Prescaler=TIM_ICPSC_DIV1; cfg.IC1Filter=6;
  cfg.IC2Polarity=TIM_ICPOLARITY_RISING;
  cfg.IC2Selection=TIM_ICSELECTION_DIRECTTI;
  cfg.IC2Prescaler=TIM_ICPSC_DIV1; cfg.IC2Filter=6;
  if (HAL_TIM_Encoder_Init(timer,&cfg)!=HAL_OK) return HAL_ERROR;
  return HAL_TIM_Encoder_Start(timer,TIM_CHANNEL_ALL);
}

/* 四路电机 PWM 共用初始化；不启动编码器或舵机。 */
HAL_StatusTypeDef Board_MotorPwmInit(void)
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  af(U2_PWMA_PORT,U2_PWMA_PIN,U2_PWMA_AF);
  af(U2_PWMB_PORT,U2_PWMB_PIN,U2_PWMB_AF);
  af(U3_PWMA_PORT,U3_PWMA_PIN,U3_PWMA_AF);
  af(U3_PWMB_PORT,U3_PWMB_PIN,U3_PWMB_AF);
  pwm.Instance=TIM1;
  pwm.Init.Prescaler=0; pwm.Init.CounterMode=TIM_COUNTERMODE_UP;
  pwm.Init.Period=8399; /* 168 MHz / 8400 = 20 kHz。 */
  pwm.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  pwm.Init.RepetitionCounter=0;
  pwm.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&pwm)!=HAL_OK) return HAL_ERROR;
  if (pwm_channel(&pwm,U2_PWMA_CHANNEL)!=HAL_OK ||
      pwm_channel(&pwm,U2_PWMB_CHANNEL)!=HAL_OK ||
      pwm_channel(&pwm,U3_PWMA_CHANNEL)!=HAL_OK ||
      pwm_channel(&pwm,U3_PWMB_CHANNEL)!=HAL_OK) return HAL_ERROR;
  if (HAL_TIM_PWM_Start(&pwm,U2_PWMA_CHANNEL)!=HAL_OK ||
      HAL_TIM_PWM_Start(&pwm,U2_PWMB_CHANNEL)!=HAL_OK ||
      HAL_TIM_PWM_Start(&pwm,U3_PWMA_CHANNEL)!=HAL_OK ||
      HAL_TIM_PWM_Start(&pwm,U3_PWMB_CHANNEL)!=HAL_OK) return HAL_ERROR;

  return HAL_OK;
}

/* 两路舵机（PB14/TIM12_CH1、PB15/TIM12_CH2），50 Hz。
 * 单独成一个函数：新底板生产入口 Board_InitStep6() 也调它，
 * 不必去碰 Board_MotionInit() 里那些老板的 AT8236/DRV8825/编码器初始化。
 * 初始化后两个通道都**不启动**，要动必须显式调 Servo_SetPulseUs() / Servo2_SetPulseUs()。 */
HAL_StatusTypeDef Servo_Init(void)
{
  uint32_t apb1_tim_clk;

  __HAL_RCC_TIM12_CLK_ENABLE();
  af(U5_PWM_PORT,U5_PWM_PIN,U5_PWM_AF);   /* 夹爪：PB14 -> TIM12_CH1（见 pins/servo_u5.h） */
  af(U6_PWM_PORT,U6_PWM_PIN,U6_PWM_AF);   /* 载物台：PB15 -> TIM12_CH2，漏了这行 PWM 出不了管脚 */

  /* TIM12 同时承载两路舵机（U5/CH1、U6/CH2），同一个 PSC/ARR → 天然同频。
   * TIM12 挂 APB1；APB1 分频不为 1 时定时器时钟 = PCLK1 × 2，
   * 所以预分频按实际频率算，保证计数频率 1 MHz（1 MHz / 20000 = 50 Hz）。 */
  apb1_tim_clk = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) apb1_tim_clk *= 2U;
  servo.Instance=U5_PWM_TIMER;
  servo.Init.Prescaler=(uint16_t)(apb1_tim_clk/1000000U - 1U);
  servo.Init.CounterMode=TIM_COUNTERMODE_UP; servo.Init.Period=19999;
  servo.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
  servo.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&servo)!=HAL_OK) return HAL_ERROR;
  if (pwm_channel(&servo,U5_PWM_CHANNEL)!=HAL_OK ||
      pwm_channel(&servo,U6_PWM_CHANNEL)!=HAL_OK) return HAL_ERROR;
  servo_ready=1U;
  return HAL_OK;
}

HAL_StatusTypeDef Board_MotionInit(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE(); __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  af(ENCA_A_PORT,ENCA_A_PIN,ENCA_AF);
  af(ENCA_B_PORT,ENCA_B_PIN,ENCA_AF);
  af(ENCB_A_PORT,ENCB_A_PIN,ENCB_AF);
  af(ENCB_B_PORT,ENCB_B_PIN,ENCB_AF);
  af(ENCC_A_PORT,ENCC_A_PIN,ENCC_AF);
  af(ENCC_B_PORT,ENCC_B_PIN,ENCC_AF);

  if (Board_MotorPwmInit()!=HAL_OK) return HAL_ERROR;
  if (Servo_Init()!=HAL_OK) return HAL_ERROR;

  if (encoder_init(&enc_a,TIM2,0xffffffffU)!=HAL_OK ||
      encoder_init(&enc_b,TIM3,0xffffU)!=HAL_OK ||
      encoder_init(&enc_c,TIM4,0xffffU)!=HAL_OK) return HAL_ERROR;
  encoder_last_raw_b=(uint16_t)TIM3->CNT;
  encoder_last_raw_c=(uint16_t)TIM4->CNT;
  d_previous_state=(uint8_t)((HAL_GPIO_ReadPin(ENCD_A_PORT,ENCD_A_PIN)?2U:0U) |
                             (HAL_GPIO_ReadPin(ENCD_B_PORT,ENCD_B_PIN)?1U:0U));
  Motor_StopAll();
  return HAL_OK;
}

void Motor_Standby(uint8_t driver, uint8_t awake)
{
  /* AT8236 双路模块没有 STBY/nSLEEP 引脚。保留这个 API 只是为了不改
   * Wheel/测试入口的调用点；真正停机由 Motor_StopAll() 把 PWM 和 DIR 清零完成。 */
  (void)driver;
  (void)awake;
}

/* 四条通道的 PWM 比较寄存器与 DIR 引脚表。Motor_Set() 和 Motor_Brake() 共用。 */
static const uint32_t motor_channels[4]={U2_PWMA_CHANNEL,U2_PWMB_CHANNEL,
                                         U3_PWMA_CHANNEL,U3_PWMB_CHANNEL};
/* AT8236 每通道只有 PWM + DIR：这里用各通道的 AIN1/BIN1 做 DIR。
 * AIN2/BIN2 在新板上没有接到模块，STBY 也不存在，所以不再操作。 */
static GPIO_TypeDef *const motor_dir_ports[4]={U2_AIN1_PORT,U2_BIN1_PORT,
                                               U3_AIN1_PORT,U3_BIN1_PORT};
static const uint16_t motor_dir_pins[4]={U2_AIN1_PIN,U2_BIN1_PIN,
                                         U3_AIN1_PIN,U3_BIN1_PIN};

HAL_StatusTypeDef Motor_Set(MotorId id, int16_t permille)
{
  uint32_t duty_counts;
  uint8_t dir_level;

  if ((unsigned)id>=4U || permille < -1000 || permille > 1000) return HAL_ERROR;

  duty_counts=(uint32_t)(permille<0?-permille:permille);
  duty_counts=(duty_counts*8400U)/1000U;

  if (permille > 0) {
    dir_level = MOTOR_DIR_FORWARD_LEVEL;
#if MOTOR_AT8236_IN1_IN2
    /* IN1=1、IN2=PWM 时，PWM 高是刹车、低才是正转 -> 正转取反。 */
    duty_counts = 8400U - duty_counts;
#endif
  } else if (permille < 0) {
    dir_level = (MOTOR_DIR_FORWARD_LEVEL == 0U) ? 1U : 0U;
    /* IN1=0、IN2=PWM 时，PWM 高是反转 -> 反向不取反。 */
  } else {
    dir_level = 0U;
    duty_counts = 0U;
  }

  __HAL_TIM_SET_COMPARE(&pwm,motor_channels[id],0);
  HAL_GPIO_WritePin(motor_dir_ports[id],motor_dir_pins[id],
                    dir_level ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&pwm,motor_channels[id],duty_counts);
  return HAL_OK;
}

/* 主动刹车（动态制动）：IN1 拉高、IN2 恒高。
 * AT8236 的 IN1=1 / IN2=1 会把电机两端同时接到 V+，绕组被短接，靠反电动势产生制动转矩，
 * 比 Motor_Set(id,0)（IN1=0/IN2=0，两端悬空 -> 滑行）停得快得多。
 * ⚠ 这是电气刹车，减速过程中电流会灌回电源，只在低速短时间用（见 WHEEL_BRAKE_MS）。
 *   机械上也更容易让麦轮瞬间失去抓地，高速急刹可能甩尾。 */
HAL_StatusTypeDef Motor_Brake(MotorId id)
{
  if ((unsigned)id>=4U) return HAL_ERROR;
  __HAL_TIM_SET_COMPARE(&pwm,motor_channels[id],0);
  HAL_GPIO_WritePin(motor_dir_ports[id],motor_dir_pins[id],
                    (MOTOR_DIR_FORWARD_LEVEL != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&pwm,motor_channels[id],8400U);   /* IN2 一直高 -> 刹车 */
  return HAL_OK;
}

void Motor_BrakeAll(void)
{
  for (uint8_t i=0;i<4;i++) (void)Motor_Brake((MotorId)i);
}

void Motor_StopAll(void)
{
  for (uint8_t i=0;i<4;i++) (void)Motor_Set((MotorId)i,0);
}

void Stepper_Enable(StepperId id,uint8_t enable)
{
  if ((unsigned)id>1U) return;
  HAL_GPIO_WritePin(id==0?U1_EN_PORT:U4_EN_PORT,
                    id==0?U1_EN_PIN:U4_EN_PIN,
                    enable?GPIO_PIN_RESET:GPIO_PIN_SET);
}
void Stepper_Direction(StepperId id,uint8_t forward)
{
  if ((unsigned)id>1U) return;
  HAL_GPIO_WritePin(id==0?U1_DIR_PORT:U4_DIR_PORT,
                    id==0?U1_DIR_PIN:U4_DIR_PIN,
                    forward?GPIO_PIN_SET:GPIO_PIN_RESET);
}
/* 输出一个可调脉宽的 STEP 脉冲。高、低电平由调用方指定，单位 us。
 * Board_DelayUs() 走 DWT 周期计数器，所以这里的时序是准确的。 */
void Stepper_PulseUs(StepperId id, uint16_t high_us, uint16_t low_us)
{
  GPIO_TypeDef *port; uint16_t pin;
  if ((unsigned)id>1U) return;
  port=id==0?U1_STEP_PORT:U4_STEP_PORT;
  pin=id==0?U1_STEP_PIN:U4_STEP_PIN;
  HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET); Board_DelayUs(high_us);
  HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET); Board_DelayUs(low_us);
}

void Stepper_Pulse(StepperId id)
{
  /* 保持原行为不变：高、低各 3 us，合计 6 us。 */
  Stepper_PulseUs(id, 3U, 3U);
}
uint8_t Stepper_Fault(StepperId id)
{
  if ((unsigned)id>1U) return 0;
  return HAL_GPIO_ReadPin(id==0?U1_FAULT_PORT:U4_FAULT_PORT,
                          id==0?U1_FAULT_PIN:U4_FAULT_PIN)==GPIO_PIN_RESET;
}
HAL_StatusTypeDef Servo_SetPulseUs(uint16_t pulse_us)
{
  if (pulse_us<500 || pulse_us>2500) return HAL_ERROR;
  if (!servo_ready) return HAL_ERROR;               /* Board_MotionInit 之前调用 */
  __HAL_TIM_SET_COMPARE(&servo,U5_PWM_CHANNEL,pulse_us);
  if (!servo_running) {
    if (HAL_TIM_PWM_Start(&servo,U5_PWM_CHANNEL)!=HAL_OK) return HAL_ERROR;
    servo_running=1;
  }
  return HAL_OK;
}
void Servo_Disable(void)
{
  if (servo_running) {
    (void)HAL_TIM_PWM_Stop(&servo,U5_PWM_CHANNEL);
    servo_running=0;
  }
}
HAL_StatusTypeDef Servo2_SetPulseUs(uint16_t pulse_us)
{
  if (pulse_us<500 || pulse_us>2500) return HAL_ERROR;
  if (!servo_ready) return HAL_ERROR;
  __HAL_TIM_SET_COMPARE(&servo,U6_PWM_CHANNEL,pulse_us);
  if (!servo2_running) {
    if (HAL_TIM_PWM_Start(&servo,U6_PWM_CHANNEL)!=HAL_OK) return HAL_ERROR;
    servo2_running=1;
  }
  return HAL_OK;
}
void Servo2_Disable(void)
{
  if (servo2_running) { (void)HAL_TIM_PWM_Stop(&servo,U6_PWM_CHANNEL); servo2_running=0; }
}

void Board_EncoderDEdge(void)
{
  static const int8_t delta[16]={0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
  uint8_t now_state=(uint8_t)((HAL_GPIO_ReadPin(ENCD_A_PORT,ENCD_A_PIN)?2U:0U) |
                              (HAL_GPIO_ReadPin(ENCD_B_PORT,ENCD_B_PIN)?1U:0U));
  encoder_d_counts += delta[(d_previous_state<<2)|now_state];
  d_previous_state=now_state;
}
/* 把 4 路原始计数扩展成累计 32 位计数。
 * **只允许由 board_encoder.c 的 tick 单一位置调用** —— 见文件头的说明。 */
void Encoder_SampleCounts(void)
{
  uint16_t raw_counts;

  encoder_total_counts[ENCODER_A] = (int32_t)TIM2->CNT;   /* TIM2 本身就是 32 位 */

  raw_counts=(uint16_t)TIM3->CNT;                          /* TIM3 是 16 位，要扩展 */
  encoder_total_counts[ENCODER_B] += (int16_t)(raw_counts-encoder_last_raw_b);
  encoder_last_raw_b=raw_counts;

  raw_counts=(uint16_t)TIM4->CNT;                          /* TIM4 同理 */
  encoder_total_counts[ENCODER_C] += (int16_t)(raw_counts-encoder_last_raw_c);
  encoder_last_raw_c=raw_counts;

  encoder_total_counts[ENCODER_D] = encoder_d_counts;      /* EXTI 里已经是累计值 */
}

/* 无副作用读取累计计数：任何上下文、调用多少次，返回值都一样且不改变状态。
 * 这就是替换旧 Encoder_Read() 的地方——旧版对 B/C 每次调用都会累加并更新
 * last_*_counts，控制中断和主循环各读一次就会互相吃掉增量。 */
int32_t Encoder_ReadCounts(EncoderId id)
{
  if ((unsigned)id>=4U) return 0;
  return encoder_total_counts[id];
}

void Encoder_Zero(EncoderId id)
{
  uint32_t mask=__get_PRIMASK(); __disable_irq();
  switch (id) {
    case ENCODER_A:
      TIM2->CNT=0; encoder_total_counts[ENCODER_A]=0; break;
    case ENCODER_B:
      TIM3->CNT=0; encoder_last_raw_b=0U;
      encoder_total_counts[ENCODER_B]=0; break;
    case ENCODER_C:
      TIM4->CNT=0; encoder_last_raw_c=0U;
      encoder_total_counts[ENCODER_C]=0; break;
    case ENCODER_D:
      encoder_d_counts=0;
      encoder_total_counts[ENCODER_D]=0;
      d_previous_state=(uint8_t)((HAL_GPIO_ReadPin(ENCD_A_PORT,ENCD_A_PIN)?2U:0U) |
                                 (HAL_GPIO_ReadPin(ENCD_B_PORT,ENCD_B_PIN)?1U:0U));
      break;
    default: break;
  }
  __set_PRIMASK(mask);
}
