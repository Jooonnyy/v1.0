#ifndef CHASSIS_MOTION_H
#define CHASSIS_MOTION_H
#include <stdint.h>

/* 底盘开环运动：点动、四轮协同（含三段加减速）、平移、自旋。
 * 不管命令文本，也不管比赛路线。步数 = 频率 × 时间，不能当定位。 */

typedef struct {
  uint8_t timed_out;     /* 1 = 协同运动超时且没走完。任务在跑时由调用方中止任务。 */
  uint8_t set_heading;   /* 1 = 本段正常走完，调用方应把航向写成 heading_deg。 */
  int16_t heading_deg;
} ChassisMotionEvent;

void ChassisMotion_Init(void);
void ChassisMotion_Cancel(void);          /* 只清标志，不停车、不发串口 */
uint8_t ChassisMotion_Busy(void);         /* 点动或协同运动还在跑 */
uint8_t ChassisMotion_JogActive(void);
uint8_t ChassisMotion_JogAxis(void);
uint8_t ChassisMotion_MoveActive(void);
const char *ChassisMotion_AxisName(uint8_t axis);

/* 航向保持运行期开关（只有平移受影响，闭环转身不受影响）。 */
void ChassisMotion_SetHoldEnable(uint8_t enable);
uint8_t ChassisMotion_GetHoldEnable(void);

/* 失败时自己回 ERR，成功不回（调用方接着回 "J ..."）。 */
void ChassisMotion_JogStart(uint8_t axis, int32_t steps, uint32_t hz);
/* 返回 0 = 没转起来。符号 +1/-1，速度和步数在内部按标度换算。 */
uint8_t ChassisMotion_Translate(int8_t sign_a, int8_t sign_b,
                                int8_t sign_c, int8_t sign_d,
                                uint32_t mm, uint32_t mm_per_s);
/* 同上，但航向保持的目标是**绝对 yaw**（0.001 度，TASK 起点清零那套坐标系）。
 * 用于 TASK 的 FWD/BACK/LEFT/RIGHT：每段都往同一个基准拉，残差不逐段累积。
 * 没有新鲜陀螺帧时自动退回相对模式（等于 ChassisMotion_Translate）。 */
uint8_t ChassisMotion_TranslateAbs(int8_t sign_a, int8_t sign_b,
                                   int8_t sign_c, int8_t sign_d,
                                   uint32_t mm, uint32_t mm_per_s,
                                   int32_t yaw_target_mdeg);
/* 正角度 = 逆时针。heading_now_deg 是指令航向，转完要写成的角存在本段里。 */
uint8_t ChassisMotion_Spin(int16_t delta_deg, int16_t heading_now_deg);

/* 按**绝对 yaw** 转身（比赛路线的 TURN_ABS 用）：每站都回到同一套绝对航向，
 * 不会把上一站的残差带下去。返回 0 = 没有新鲜陀螺帧，调用方应退回 Spin()。 */
uint8_t ChassisMotion_SpinAbs(int16_t heading_deg, int32_t yaw_target_mdeg);

void ChassisMotion_PollJog(void);
/* task_active = 1 时，超时不回 "MOVE timeout"，交给调用方发任务中止。 */
void ChassisMotion_PollMove(uint8_t task_active, ChassisMotionEvent *event);

#endif
