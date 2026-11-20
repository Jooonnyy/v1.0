#ifndef CHASSIS_STEP_SMOKE_H
#define CHASSIS_STEP_SMOKE_H

/* 底盘步进台架（CHASSIS_STEP Target）：验证新底板四个轮子的 STEP/DIR 接线与转向。
 * 只做速度/方向/点动，不含闭环（新板没有编码器）。
 * 入口只有 Begin / Poll 两个。 */
void ChassisStepSmoke_Begin(void);
void ChassisStepSmoke_Poll(void);

#endif
