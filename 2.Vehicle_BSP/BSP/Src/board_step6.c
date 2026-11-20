#include "board_internal.h"
#include "board_step6.h"
#include "pins/stepper6.h"

/* 六轴步进：一个轴 = 一个定时器的一个 PWM 通道。
 *
 * 为什么不用"一个定时器四个通道"：同一定时器的所有通道共用 ARR，
 * 只能出一种频率；麦轮转向/横移时四轮速度不同，必须一轴一个定时器。
 *
 * STEP 频率 = 定时器计数频率 / (PSC+1) / (ARR+1)，占空比固定 50%。
 * 16 位定时器在低频时 ARR 会溢出，所以按需抬高 PSC —— 见 axis_run()。 */

typedef struct {
  GPIO_TypeDef *step_port;
  uint16_t      step_pin;
  uint8_t       step_af;
  GPIO_TypeDef *dir_port;
  uint16_t      dir_pin;
  TIM_TypeDef  *timer;
  uint32_t      channel;
} StepPinDef;

static const StepPinDef axis_pins[STEP6_COUNT] = {
  { ST6_A_STEP_PORT,    ST6_A_STEP_PIN,    ST6_A_STEP_AF,
    ST6_A_DIR_PORT,     ST6_A_DIR_PIN,     ST6_A_TIM,    ST6_A_CH    },
  { ST6_B_STEP_PORT,    ST6_B_STEP_PIN,    ST6_B_STEP_AF,
    ST6_B_DIR_PORT,     ST6_B_DIR_PIN,     ST6_B_TIM,    ST6_B_CH    },
  { ST6_C_STEP_PORT,    ST6_C_STEP_PIN,    ST6_C_STEP_AF,
    ST6_C_DIR_PORT,     ST6_C_DIR_PIN,     ST6_C_TIM,    ST6_C_CH    },
  { ST6_D_STEP_PORT,    ST6_D_STEP_PIN,    ST6_D_STEP_AF,
    ST6_D_DIR_PORT,     ST6_D_DIR_PIN,     ST6_D_TIM,    ST6_D_CH    },
  { ST6_ROT_STEP_PORT,  ST6_ROT_STEP_PIN,  ST6_ROT_STEP_AF,
    ST6_ROT_DIR_PORT,   ST6_ROT_DIR_PIN,   ST6_ROT_TIM,  ST6_ROT_CH  },
  { ST6_LIFT_STEP_PORT, ST6_LIFT_STEP_PIN, ST6_LIFT_STEP_AF,
    ST6_LIFT_DIR_PORT,  ST6_LIFT_DIR_PIN,  ST6_LIFT_TIM, ST6_LIFT_CH },
};

/* 正转时 DIR 输出高电平。整车全反了才改这里；单轴反了用下面的轴级取反表。 */
#define STEP6_DIR_FORWARD_LEVEL   1U
/* 张大头闭环伺服的 EN 输入是低有效（和原来 DRV8825 的 nEN 一致）。 */
#define STEP6_EN_ACTIVE_LOW       1U

/* 轴级方向取反表（2026-11-03 台架实测）：
 * 网络名 - 装车位（俯视图，上=车头）：A=前左、B=前右、C=后左、D=后右。
 * DIR=正电平时实测 B/D 推车向前、A/C 推车向后（左右轮镜像安装，符合预期），
 * 所以 A/C 取反，让"+ 号 = 四轮一起推车向前"。
 * 换车 / 换轮布局后先 J 逐轴复核再改这里，不要动接线。 */
#define STEP6_DIR_INVERT_A    1U
#define STEP6_DIR_INVERT_B    0U
#define STEP6_DIR_INVERT_C    1U
#define STEP6_DIR_INVERT_D    0U
#define STEP6_DIR_INVERT_ROT  0U   /* 旋转台方向待单独 J 验证后定 */
#define STEP6_DIR_INVERT_LIFT 0U   /* 升降台方向待单独 J 验证后定 */

static TIM_HandleTypeDef axis_htim[STEP6_COUNT];
static const uint8_t axis_dir_invert[STEP6_COUNT] = {
  STEP6_DIR_INVERT_A, STEP6_DIR_INVERT_B, STEP6_DIR_INVERT_C,
  STEP6_DIR_INVERT_D, STEP6_DIR_INVERT_ROT, STEP6_DIR_INVERT_LIFT,
};
static uint32_t axis_hz[STEP6_COUNT];       /* 绝对值，0 = 停 */
static int8_t   axis_dir[STEP6_COUNT];      /* +1 / -1 */
static uint32_t step6_max_hz = STEP6_MAX_HZ_DEFAULT;
static uint8_t  step6_enabled;

static void pin_af(GPIO_TypeDef *port, uint16_t pin, uint8_t af)
{
  GPIO_InitTypeDef cfg = {0};
  cfg.Pin = pin; cfg.Mode = GPIO_MODE_AF_PP; cfg.Pull = GPIO_NOPULL;
  cfg.Speed = GPIO_SPEED_FREQ_HIGH; cfg.Alternate = af;
  HAL_GPIO_Init(port, &cfg);
}

static void pin_out(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState level)
{
  GPIO_InitTypeDef cfg = {0};
  HAL_GPIO_WritePin(port, pin, level);
  cfg.Pin = pin; cfg.Mode = GPIO_MODE_OUTPUT_PP; cfg.Pull = GPIO_NOPULL;
  cfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &cfg);
}

/* 三根 nEN 一起写：底盘四路（PD3）、旋转（PC8）、升降（PC9）。
 * 旋转/升降单独断电要靠硬件，固件这一层统一开关。 */
static void en_pins_write(uint8_t enable)
{
  GPIO_PinState level;

  if (STEP6_EN_ACTIVE_LOW != 0U) level = (enable != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  else                           level = (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(ST6_EN_PORT, ST6_EN_PIN, level);
  HAL_GPIO_WritePin(ST6_EN_ROT_PORT, ST6_EN_ROT_PIN, level);
  HAL_GPIO_WritePin(ST6_EN_LIFT_PORT, ST6_EN_LIFT_PIN, level);
}

/* TIM1/TIM8~TIM11 挂 APB2，其余挂 APB1。APB 分频不为 1 时定时器时钟 = PCLK × 2。
 * 直接抄 HAL_RCC_GetPCLKxFreq() 会算错一倍频，六个轴的频率就全偏了。 */
static uint32_t timer_clk(TIM_TypeDef *timer)
{
  uint32_t clk;
  if (timer == TIM1 || timer == TIM8 || timer == TIM9 ||
      timer == TIM10 || timer == TIM11) {
    clk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) clk *= 2U;
  } else {
    clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) clk *= 2U;
  }
  return clk;
}

static void axis_dir_pin(uint8_t i, int8_t dir)
{
  const StepPinDef *p = &axis_pins[i];
  GPIO_PinState level;

  if (dir > 0) {
    level = (STEP6_DIR_FORWARD_LEVEL != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  } else {
    level = (STEP6_DIR_FORWARD_LEVEL != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  /* 轴级取反：左右轮镜像安装导致个别轴物理方向相反，见方向取反表注释。 */
  if (axis_dir_invert[i] != 0U) {
    level = (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(p->dir_port, p->dir_pin, level);
  axis_dir[i] = dir;
}

static void axis_stop(uint8_t i)
{
  const StepPinDef *p = &axis_pins[i];
  (void)HAL_TIM_PWM_Stop(&axis_htim[i], p->channel);
  axis_hz[i] = 0U;
}

/* 把某轴设到指定频率（绝对值）并保证输出。返回 HAL_ERROR 表示这个频率做不到。 */
static HAL_StatusTypeDef axis_run(uint8_t i, uint32_t hz)
{
  const StepPinDef *p = &axis_pins[i];
  TIM_TypeDef *timer = p->timer;
  uint32_t clk = timer_clk(timer);
  uint32_t psc = 0U, div, arr;

  if (hz == 0U) { axis_stop(i); return HAL_OK; }
  if (hz > STEP6_HZ_HARD_LIMIT) hz = STEP6_HZ_HARD_LIMIT;

  /* 16 位定时器：低频时 (clk/hz) 会超过 65535，必须抬高预分频。
   * PSC 本身也是 16 位，极端低速（<约 1.3 Hz@84MHz）仍然做不到，直接报错。 */
  while (psc < 0xFFFFU) {
    div = clk / (psc + 1U);
    if (div == 0U) return HAL_ERROR;
    if ((div / hz) <= 0xFFFFU) break;
    psc++;
  }
  div = clk / (psc + 1U);
  arr = div / hz;
  if (arr < 4U) arr = 4U;               /* 50% 时高低各 ≥2 个计数 */
  if (arr > 0xFFFFU) return HAL_ERROR;

  /* 改 PSC/ARR 期间先停通道，避免打出畸形脉冲被驱动器当成一步。 */
  (void)HAL_TIM_PWM_Stop(&axis_htim[i], p->channel);
  timer->PSC = (uint16_t)psc;
  timer->ARR = (uint16_t)arr;
  __HAL_TIM_SET_COMPARE(&axis_htim[i], p->channel, arr / 2U);
  timer->EGR = TIM_EGR_UG;              /* 立即把 PSC/ARR 装进影子寄存器 */
  if (HAL_TIM_PWM_Start(&axis_htim[i], p->channel) != HAL_OK) return HAL_ERROR;

  axis_hz[i] = hz;
  return HAL_OK;
}

static HAL_StatusTypeDef axis_init(uint8_t i)
{
  const StepPinDef *p = &axis_pins[i];
  TIM_OC_InitTypeDef oc = {0};
  TIM_HandleTypeDef *h = &axis_htim[i];

  pin_af(p->step_port, p->step_pin, p->step_af);
  pin_out(p->dir_port, p->dir_pin, GPIO_PIN_RESET);
  axis_dir_pin(i, 1);      /* 初始化就要和"正转"的电平一致，否则第一次正转会沿用低电平 */

  h->Instance = p->timer;
  h->Init.Prescaler = 0U;
  h->Init.CounterMode = TIM_COUNTERMODE_UP;
  h->Init.Period = 0xFFFFU;
  h->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  h->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  h->Init.RepetitionCounter = 0U;
  if (HAL_TIM_PWM_Init(h) != HAL_OK) return HAL_ERROR;

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(h, &oc, p->channel) != HAL_OK) return HAL_ERROR;

  /* 先不启动通道：不发脉冲，等命令。 */
  return HAL_OK;
}

HAL_StatusTypeDef Board_InitStep6(void)
{
  uint8_t i;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* Board_DelayUs() 用 DWT，靠这里打开。 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_TIM9_CLK_ENABLE();
  __HAL_RCC_TIM13_CLK_ENABLE();

  /* nEN 默认置成"失能"，等命令再开（三根：底盘四路 / 旋转 / 升降）。 */
  pin_out(ST6_EN_PORT, ST6_EN_PIN,
          (STEP6_EN_ACTIVE_LOW != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  pin_out(ST6_EN_ROT_PORT, ST6_EN_ROT_PIN,
          (STEP6_EN_ACTIVE_LOW != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  pin_out(ST6_EN_LIFT_PORT, ST6_EN_LIFT_PIN,
          (STEP6_EN_ACTIVE_LOW != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  step6_enabled = 0U;

  for (i = 0U; i < STEP6_COUNT; i++) {
    if (axis_init(i) != HAL_OK) return HAL_ERROR;
    axis_hz[i] = 0U;
  }

  /* 两路舵机（PB14/PB15 + TIM12）：只配好、不启动通道，要动必须显式发脉宽。
   * 这一步以前只在老板的 Board_MotionInit() 里做，新板生产入口够不着。 */
  if (Servo_Init() != HAL_OK) return HAL_ERROR;

  if (Board_CommInit() != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

void Step6_EnableAll(uint8_t enable)
{
  if (!enable) Step6_StopAll();      /* 先停再断使能，避免带速失能丢步/溜车 */

  en_pins_write((enable != 0U) ? 1U : 0U);
  step6_enabled = (enable != 0U) ? 1U : 0U;
}

uint8_t Step6_IsEnabled(void) { return step6_enabled; }

HAL_StatusTypeDef Step6_SetHz(Step6Axis id, int32_t hz)
{
  uint8_t i = (uint8_t)id;
  int8_t  dir;
  uint32_t mag;

  if ((unsigned)id >= STEP6_COUNT) return HAL_ERROR;

  if (hz == 0) { axis_stop(i); return HAL_OK; }
  if (!step6_enabled) return HAL_ERROR;   /* 未使能不转 */

  dir = (hz > 0) ? 1 : -1;
  mag = (uint32_t)((hz > 0) ? hz : -hz);

  /* 换向必须"停 → 改 DIR → 等 5us → 再起"，否则驱动器可能按旧方向吃掉几个脉冲。 */
  if (axis_dir[i] != dir) {
    axis_stop(i);
    axis_dir_pin(i, dir);
    Board_DelayUs(5U);
  }
  return axis_run(i, mag);
}

HAL_StatusTypeDef Step6_SetPermille(Step6Axis id, int16_t permille)
{
  int32_t hz;
  if (permille > 1000) permille = 1000;
  if (permille < -1000) permille = -1000;
  hz = (int32_t)(((int32_t)step6_max_hz * (int32_t)permille) / 1000);
  if (hz == 0 && permille != 0) hz = (permille > 0) ? 1 : -1;  /* 别把小数吃成停 */
  return Step6_SetHz(id, hz);
}

void Step6_Stop(Step6Axis id)
{
  if ((unsigned)id < STEP6_COUNT) axis_stop((uint8_t)id);
}

void Step6_StopAll(void)
{
  uint8_t i;
  for (i = 0U; i < STEP6_COUNT; i++) axis_stop(i);
}

uint32_t Step6_GetHz(Step6Axis id)
{
  if ((unsigned)id >= STEP6_COUNT) return 0U;
  return axis_hz[id];
}

int8_t Step6_GetDir(Step6Axis id)
{
  if ((unsigned)id >= STEP6_COUNT) return 1;
  return axis_dir[id];
}

void Step6_SetMaxHz(uint32_t hz)
{
  if (hz < 100U) hz = 100U;
  if (hz > STEP6_HZ_HARD_LIMIT) hz = STEP6_HZ_HARD_LIMIT;
  step6_max_hz = hz;
}

uint32_t Step6_GetMaxHz(void) { return step6_max_hz; }
