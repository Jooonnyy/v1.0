#ifndef VEHICLE_APP_CONFIG_H
#define VEHICLE_APP_CONFIG_H

/* 参数取自最近一次台架测试；装车后仍须重新核对。 */
#define APP_MOTOR_A_DUTY_PERMILLE 250
#define APP_MOTOR_A_MAX_RUN_MS 5000U

/* 当前舵机测试使用 125 度夹紧、70 度松开。 */
#define APP_GRIP_CLOSED_DEG 125U
#define APP_GRIP_OPEN_DEG 70U
#define APP_SERVO_0_DEG_US 1000U
#define APP_SERVO_180_DEG_US 2000U

#endif
