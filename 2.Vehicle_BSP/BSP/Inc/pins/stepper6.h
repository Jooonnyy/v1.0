#ifndef BSP_PINS_STEPPER6_H
#define BSP_PINS_STEPPER6_H
#include "stm32f4xx_hal.h"

/* 新底板（2026-11 打样）六轴步进：张大头闭环伺服，STEP/DIR 输入。
 *
 * 引脚事实来源：嘉立创EDA 工程 `gshwww5/STEP` / 板 `电控板` 的网表（2026-10-29/30 两次核对），
 * 与 `Docs/新板引脚分配_6路步进.md` 一致。
 *
 * ⚠ 核心约束：**一个定时器的所有通道共用 ARR，只能输出一种频率**。
 *   麦轮四轮在转向/横移时速度各不相同，所以六个轴必须各占一个独立定时器。
 *
 * | 轴 | STEP | 定时器      | DIR  |
 * |----|------|-------------|------|
 * | A  | PE9  | TIM1_CH1    | PE7  |
 * | B  | PE6  | TIM9_CH2    | PE8  |
 * | C  | PA0  | TIM2_CH1    | PE10 |
 * | D  | PB0  | TIM3_CH3    | PE12 |
 * | 旋转 | PD12 | TIM4_CH1  | PE13 |
 * | 升降 | PA6  | TIM13_CH1   | PE14 |
 *
 * A/B/C/D 的物理对应关系 **2026-11-03 已在台架确认**（俯视图，上=车头）：
 * A=前左、B=前右、C=后左、D=后右。DIR=正电平时 B/D 推车向前、A/C 向后
 * （左右镜像安装的正常现象），A/C 的取反在 `board_step6.c` 的
 * `STEP6_DIR_INVERT_*` 表里，改方向去那里，不要动接线。
 *
 * DIR 六个都在 GPIOE，可一次 ODR 写完。
 * 底盘四路 nEN 并成一根（PD3）；旋转/升降各有独立 nEN（PC8 / PC9）。 */

/* ---- A：左前轮 ---- */
#define ST6_A_STEP_PORT   GPIOE
#define ST6_A_STEP_PIN    GPIO_PIN_9
#define ST6_A_STEP_AF     GPIO_AF1_TIM1
#define ST6_A_TIM         TIM1
#define ST6_A_CH          TIM_CHANNEL_1
#define ST6_A_DIR_PORT    GPIOE
#define ST6_A_DIR_PIN     GPIO_PIN_7

/* ---- B：右前轮 ---- */
#define ST6_B_STEP_PORT   GPIOE
#define ST6_B_STEP_PIN    GPIO_PIN_6
#define ST6_B_STEP_AF     GPIO_AF3_TIM9
#define ST6_B_TIM         TIM9
#define ST6_B_CH          TIM_CHANNEL_2
#define ST6_B_DIR_PORT    GPIOE
#define ST6_B_DIR_PIN     GPIO_PIN_8

/* ---- C：左后轮 ---- */
#define ST6_C_STEP_PORT   GPIOA
#define ST6_C_STEP_PIN    GPIO_PIN_0
#define ST6_C_STEP_AF     GPIO_AF1_TIM2
#define ST6_C_TIM         TIM2
#define ST6_C_CH          TIM_CHANNEL_1
#define ST6_C_DIR_PORT    GPIOE
#define ST6_C_DIR_PIN     GPIO_PIN_10

/* ---- D：右后轮 ---- */
#define ST6_D_STEP_PORT   GPIOB
#define ST6_D_STEP_PIN    GPIO_PIN_0
#define ST6_D_STEP_AF     GPIO_AF2_TIM3
#define ST6_D_TIM         TIM3
#define ST6_D_CH          TIM_CHANNEL_3
#define ST6_D_DIR_PORT    GPIOE
#define ST6_D_DIR_PIN     GPIO_PIN_12

/* ---- 旋转台 ---- */
#define ST6_ROT_STEP_PORT GPIOD
#define ST6_ROT_STEP_PIN  GPIO_PIN_12
#define ST6_ROT_STEP_AF   GPIO_AF2_TIM4
#define ST6_ROT_TIM       TIM4
#define ST6_ROT_CH        TIM_CHANNEL_1
#define ST6_ROT_DIR_PORT  GPIOE
#define ST6_ROT_DIR_PIN   GPIO_PIN_13

/* ---- 升降台 ---- */
#define ST6_LIFT_STEP_PORT GPIOA
#define ST6_LIFT_STEP_PIN  GPIO_PIN_6
#define ST6_LIFT_STEP_AF   GPIO_AF9_TIM13
#define ST6_LIFT_TIM       TIM13
#define ST6_LIFT_CH        TIM_CHANNEL_1
#define ST6_LIFT_DIR_PORT  GPIOE
#define ST6_LIFT_DIR_PIN   GPIO_PIN_14

/* ---- 使能（nEN，低有效）---- */
/* 底盘四路 nEN 并成一根（PD3），不能单独禁某一个轮子；
 * 旋转（PC8）、升降（PC9）各有独立使能，可单独断电。 */
#define ST6_EN_PORT        GPIOD
#define ST6_EN_PIN         GPIO_PIN_3
#define ST6_EN_ROT_PORT    GPIOC
#define ST6_EN_ROT_PIN     GPIO_PIN_8
#define ST6_EN_LIFT_PORT   GPIOC
#define ST6_EN_LIFT_PIN    GPIO_PIN_9

#endif
