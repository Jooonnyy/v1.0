#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include <stdint.h>

/* 比赛路线。航点和到站朝向沿用旧 WHEEL v0.6，走法是前后左右，不开编码器。
 * 运动本身在 chassis_motion，这里只排步骤。 */

void ChassisTask_Init(void);
void ChassisTask_Cancel(void);            /* 只清标志 */
void ChassisTask_Abort(const char *reason);
uint8_t ChassisTask_IsActive(void);
uint8_t ChassisTask_StepNumber(void);     /* 在跑时为 1..N，否则 0 */
int16_t ChassisTask_HeadingDeg(void);
void ChassisTask_SetHeadingDeg(int16_t heading_deg);

void ChassisTask_Start(void);
void ChassisTask_StartDry(void);   /* 不接视觉：只跑底盘/换环，跳过扫码和机构 */
uint8_t ChassisTask_StartStep(uint8_t step);   /* 1..7：只从 stp n 开始跑 */
uint8_t ChassisTask_SetPlan(const char *code); /* 解析 15 位任务码，存两批颜色/环号 */
void ChassisTask_Poll(void);

#endif
