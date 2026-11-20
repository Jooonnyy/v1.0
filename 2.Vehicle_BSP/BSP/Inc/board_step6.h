#ifndef BOARD_STEP6_H
#define BOARD_STEP6_H
#include "board.h"

/* 新底板六轴步进（张大头闭环伺服，STEP/DIR）。
 *
 * 与 board.h 里老的 Stepper_*（DRV8825 U1/U4，阻塞发脉冲）是两套东西：
 * 那套是"软件阻塞翻转 + 只支持两轴"，这套是"一个定时器一个轴、硬件出脉冲"。
 * 名字用 Step6_ 前缀，避免和 Stepper_Enable/Stepper_Pulse 撞。
 *
 * 语义：
 *   - 每个轴一个定时器 PWM 通道，50% 占空比，频率 = 步进频率。
 *   - Step6_SetHz(id, 0) 停；正数正转、负数反转；换向时自动先停、改 DIR、再起。
 *   - **未使能时任何非零速度都会被拒绝**（返回 HAL_ERROR），防止误动作。
 *   - 上电默认：全部停止、全部失能。
 *
 * 里程/位置：本层只给速度，不计数。要位置由调用方按 hz × 时间积分
 * （底盘台架测试就是这么做的），或者以后换成带回读的驱动。 */

typedef enum {
  STEP6_A = 0,     /* 底盘轮 A（原理图网络名） */
  STEP6_B = 1,     /* 底盘轮 B */
  STEP6_C = 2,     /* 底盘轮 C */
  STEP6_D = 3,     /* 底盘轮 D */
  STEP6_ROT = 4,   /* 旋转台 */
  STEP6_LIFT = 5,  /* 升降台 */
  STEP6_COUNT = 6
} Step6Axis;

/* 满速默认值。8000 步/秒 ≈ 2.5 转/秒 ≈ 0.59 m/s
 * （按 16 细分、200 步/转、轮径 75 mm、周长 235.6 mm 估算）。 */
#define STEP6_MAX_HZ_DEFAULT   8000U
/* 硬上限：再高 42 步进会丢步，只用于参数校验。 */
#define STEP6_HZ_HARD_LIMIT   40000U

/* 板级初始化：只做 GPIO + 六个定时器 + 串口，**不碰编码器/底盘运动层**。
 * 新板上 TIM2/TIM3/TIM4 已经从编码器改成了 STEP 输出，
 * 所以这个 Target 不能走 Board_Init()（它会把 TIM2/3/4 配成编码器模式）。 */
HAL_StatusTypeDef Board_InitStep6(void);

void     Step6_EnableAll(uint8_t enable);
uint8_t  Step6_IsEnabled(void);

/* hz > 0 正转、hz < 0 反转、hz == 0 停。未使能时非零值返回 HAL_ERROR。 */
HAL_StatusTypeDef Step6_SetHz(Step6Axis id, int32_t hz);
/* ±1000 → ±满速（Step6_GetMaxHz）。 */
HAL_StatusTypeDef Step6_SetPermille(Step6Axis id, int16_t permille);

void     Step6_StopAll(void);
void     Step6_Stop(Step6Axis id);

uint32_t Step6_GetHz(Step6Axis id);      /* 取绝对值，0 = 停 */
int8_t   Step6_GetDir(Step6Axis id);     /* +1 / -1 */
void     Step6_SetMaxHz(uint32_t hz);
uint32_t Step6_GetMaxHz(void);

#endif
