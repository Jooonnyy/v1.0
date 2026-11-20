#include "board_encoder.h"
#include "board_internal.h"
#include "board_wheel.h"

/* 车轮每转的直线距离，单位 um。用 355/113 近似 pi，全程不引入浮点。
 * 例：75 mm 轮 -> 75000 x 355 / 113 = 235619 um。 */
#define WHEEL_CIRCUMFERENCE_UM \
  ((int32_t)(((int64_t)WHEEL_DIAMETER_MM * 355000LL) / 113LL))

/* 方向符号查表，避免在 tick 里写 switch。 */
static const int8_t encoder_sign[4] = {
  (int8_t)ENCODER_A_SIGN, (int8_t)ENCODER_B_SIGN,
  (int8_t)ENCODER_C_SIGN, (int8_t)ENCODER_D_SIGN
};

static TIM_HandleTypeDef tick_timer;

/* 以下 4 个数组都由 Encoder_Tick() 写、被主循环读。
 * Cortex-M4 上对齐的 32 位读写是单指令，不会撕裂，所以不需要临界区。 */
static volatile int32_t counts_signed[4];
static volatile int32_t speed_counts_per_s[4];
static uint32_t still_ticks[4];
static volatile uint32_t tick_count;
static volatile EncoderWindow odom_window;

/* APB1 分频不为 1 时，定时器时钟是 PCLK1 的两倍。
 * 本工程 APB1 = HCLK/4 = 42 MHz，所以 TIM7 时钟是 84 MHz。 */
static uint32_t tick_timer_clock_hz(void)
{
  uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
  uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  return (pclk1_hz == hclk_hz) ? pclk1_hz : (pclk1_hz * 2U);
}

HAL_StatusTypeDef Encoder_Init(void)
{
  uint32_t timer_clock_hz = tick_timer_clock_hz();
  uint32_t prescaler_div = timer_clock_hz / 1000000U;   /* 计数频率 1 MHz */

  __HAL_RCC_TIM7_CLK_ENABLE();

  tick_timer.Instance = TIM7;
  tick_timer.Init.Prescaler = (prescaler_div == 0U) ? 0U : (prescaler_div - 1U);
  tick_timer.Init.CounterMode = TIM_COUNTERMODE_UP;
  tick_timer.Init.Period = (1000000U / ENCODER_TICK_HZ) - 1U;
  tick_timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tick_timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&tick_timer) != HAL_OK) return HAL_ERROR;

  /* 先把 4 路计数清零，再放中断进来，避免第一拍就带着上电前的残余计数。 */
  Encoder_ResetAll();

  HAL_NVIC_SetPriority(TIM7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  return HAL_TIM_Base_Start_IT(&tick_timer);
}

/* TIM7 更新中断：本模块自己占用，不需要动 Core/Src/stm32f4xx_it.c。
 * 一拍的顺序是"测速 → 控制"：先把 4 路计数采成快照、算好轮速，
 * 紧接着让 board_wheel.c 用这份快照跑 PI 并写 PWM。
 * 只做计数采样、四则运算和寄存器写入，不发串口、不限循环 ——
 * 符合 AGENTS.md 第 6 节"控制环 ISR"的五条硬性条件。 */
void TIM7_IRQHandler(void)
{
  if ((TIM7->SR & TIM_SR_UIF) != 0U) {
    TIM7->SR = ~TIM_SR_UIF;   /* 只清 UIF，其余位写 1 无副作用 */
    Encoder_Tick();
    Wheel_Update();
  }
}

void Encoder_Tick(void)
{
  uint32_t i;

  /* 先让硬件层把 4 路原始计数扩展成累计计数。这是全局唯一的更新点。 */
  Encoder_SampleCounts();

  for (i = 0U; i < 4U; ++i) {
    int32_t now_signed = Encoder_ReadCounts((EncoderId)i) * (int32_t)encoder_sign[i];
    int32_t delta_counts = (int32_t)((uint32_t)now_signed - (uint32_t)counts_signed[i]);
    uint32_t magnitude_counts = delta_counts < 0 ? 0U - (uint32_t)delta_counts : (uint32_t)delta_counts;
    odom_window.delta_counts[i] = (int32_t)((uint32_t)odom_window.delta_counts[i] + (uint32_t)delta_counts);
    if (magnitude_counts > odom_window.peak_delta_counts[i])
      odom_window.peak_delta_counts[i] = magnitude_counts;
    counts_signed[i] = now_signed;

    if (delta_counts == 0) {
      if (still_ticks[i] < ENCODER_STILL_TICKS) still_ticks[i]++;
      if (still_ticks[i] >= ENCODER_STILL_TICKS) {
        speed_counts_per_s[i] = 0;   /* 确实停了，直接归零，别让低通拖残速 */
        continue;
      }
    } else {
      still_ticks[i] = 0U;
    }

    /* 10 ms 窗口的 M 法：速度 = 窗口内计数 x 每秒钟的窗口数。 */
    {
      int32_t instant_counts_per_s = delta_counts * (int32_t)ENCODER_TICK_HZ;
      speed_counts_per_s[i] += (instant_counts_per_s - speed_counts_per_s[i]) /
                               (int32_t)ENCODER_SPEED_FILTER_DIV;
    }
  }

  odom_window.ticks++;
  odom_window.sampled_ms = HAL_GetTick();
  tick_count++;
}

int32_t Encoder_Counts(EncoderId id)
{
  if ((unsigned)id >= 4U) return 0;
  return counts_signed[id];
}

int32_t Encoder_SpeedCountsPerSec(EncoderId id)
{
  if ((unsigned)id >= 4U) return 0;
  return speed_counts_per_s[id];
}

int32_t Encoder_SpeedUmPerSec(EncoderId id)
{
  if ((unsigned)id >= 4U) return 0;
  /* um/s = counts/s x 轮周长(um) / 每转计数 */
  return (int32_t)(((int64_t)speed_counts_per_s[id] *
                    (int64_t)WHEEL_CIRCUMFERENCE_UM) /
                   (int64_t)WHEEL_COUNTS_PER_REV);
}

int32_t Encoder_CountsPerSecFromUmPerSec(int32_t um_per_s)
{
  /* 上面那个式子的逆运算，用同一组常数，保证正反换算一致。 */
  return (int32_t)(((int64_t)um_per_s * (int64_t)WHEEL_COUNTS_PER_REV) /
                   (int64_t)WHEEL_CIRCUMFERENCE_UM);
}

void Encoder_ResetAll(void)
{
  uint32_t i;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();   /* 别让 tick 在清零中途插进来 */

  Encoder_Zero(ENCODER_A);
  Encoder_Zero(ENCODER_B);
  Encoder_Zero(ENCODER_C);
  Encoder_Zero(ENCODER_D);
  for (i = 0U; i < 4U; ++i) {
    odom_window.delta_counts[i] = 0;
    odom_window.peak_delta_counts[i] = 0U;
    counts_signed[i] = 0;
    speed_counts_per_s[i] = 0;
    still_ticks[i] = 0U;
  }
  odom_window.ticks = 0U;
  odom_window.sampled_ms = HAL_GetTick();
  odom_window.generation++;
  tick_count = 0U;

  __set_PRIMASK(primask);
}

uint32_t Encoder_Ticks(void)
{
  return tick_count;
}

/* 只搬运固定大小快照并清窗口，临界区内不做换算、I2C 或串口。 */
void Encoder_TakeWindow(EncoderWindow *window)
{
  uint32_t saved_mask;
  if (window == NULL) return;
  saved_mask = Board_EnterCritical();
  *window = odom_window;
  for (uint32_t i = 0U; i < 4U; ++i) {
    odom_window.delta_counts[i] = 0;
    odom_window.peak_delta_counts[i] = 0U;
  }
  odom_window.ticks = 0U;
  Board_ExitCritical(saved_mask);
}
