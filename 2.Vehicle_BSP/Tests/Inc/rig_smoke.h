#ifndef RIG_SMOKE_H
#define RIG_SMOKE_H

/* 蓝牙联调台架：两个张大头步进（升降台 U1 / 旋转台 U4）+ 两路舵机
 * （机械臂 U5/PE5、载物台 U6/PE6）
 * + 四路底盘直流电机（TB6612 U2/U3 的 A/B/C/D）。
 * 入口只有 Begin / Poll 两个。 */
void RigSmoke_Begin(void);
void RigSmoke_Poll(void);

#endif
