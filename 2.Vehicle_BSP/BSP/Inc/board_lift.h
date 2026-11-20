#ifndef BOARD_LIFT_H
#define BOARD_LIFT_H

#include <stdint.h>

/* ============================== 参数区 ============================== */

/* 丝杆每转的直线位移，单位 0.001 mm。 */
#define LIFT_SCREW_LEAD_UM             8000
/* 1.8 度电机每转 200 个整步。 */
#define LIFT_MOTOR_FULL_STEPS           200
/* 必须与 DRV8825 的 M0/M1/M2 实际接线一致。 */
#define LIFT_DRV8825_MICROSTEPS           8
#define LIFT_MICROSTEPS_PER_REV \
  (LIFT_MOTOR_FULL_STEPS * LIFT_DRV8825_MICROSTEPS)
#define LIFT_FULL_TURN_MDEG          360000L
/* DIR 为高时是否上升；实物方向相反时改为 0。 */
#define LIFT_DIR_HIGH_IS_UP               1
/* STEP 周期及起步周期，单位微秒。 */
#define LIFT_STEP_PERIOD_US            1000
#define LIFT_START_PERIOD_US           4000
#define LIFT_RAMP_PULSES                200
/* 相对上电软零位的允许窗口。 */
#define LIFT_MIN_UM                  -50000
#define LIFT_MAX_UM                   50000
/* 限位有效电平确认后才能启用。 */
#define LIFT_USE_HW_LIMITS                0
/* 移动后是否保持驱动器使能。 */
#define LIFT_HOLD_AFTER_MOVE              1

#if (LIFT_SCREW_LEAD_UM <= 0)
#error LIFT_SCREW_LEAD_UM must be positive
#endif
#if (LIFT_MOTOR_FULL_STEPS <= 0)
#error LIFT_MOTOR_FULL_STEPS must be positive
#endif
#if (LIFT_DRV8825_MICROSTEPS != 1 && LIFT_DRV8825_MICROSTEPS != 2 && \
     LIFT_DRV8825_MICROSTEPS != 4 && LIFT_DRV8825_MICROSTEPS != 8 && \
     LIFT_DRV8825_MICROSTEPS != 16 && LIFT_DRV8825_MICROSTEPS != 32)
#error LIFT_DRV8825_MICROSTEPS must be 1, 2, 4, 8, 16 or 32
#endif
#if (LIFT_STEP_PERIOD_US < 20)
#error LIFT_STEP_PERIOD_US must be at least 20
#endif
#if (LIFT_START_PERIOD_US < LIFT_STEP_PERIOD_US)
#error LIFT_START_PERIOD_US must not be faster than LIFT_STEP_PERIOD_US
#endif
#if (LIFT_RAMP_PULSES < 0)
#error LIFT_RAMP_PULSES must not be negative
#endif
#if (LIFT_MIN_UM >= LIFT_MAX_UM)
#error LIFT_MIN_UM must be smaller than LIFT_MAX_UM
#endif
#if (LIFT_DIR_HIGH_IS_UP != 0 && LIFT_DIR_HIGH_IS_UP != 1)
#error LIFT_DIR_HIGH_IS_UP must be 0 or 1
#endif
#if (LIFT_USE_HW_LIMITS != 0 && LIFT_USE_HW_LIMITS != 1)
#error LIFT_USE_HW_LIMITS must be 0 or 1
#endif
#if (LIFT_HOLD_AFTER_MOVE != 0 && LIFT_HOLD_AFTER_MOVE != 1)
#error LIFT_HOLD_AFTER_MOVE must be 0 or 1
#endif

typedef enum {
  LIFT_OK = 0,
  LIFT_BUSY,
  LIFT_ERR_RANGE,
  LIFT_ERR_FAULT
} LiftStatus;

typedef enum {
  LIFT_RUN_IDLE = 0,
  LIFT_RUN_COMPLETE,
  LIFT_RUN_STOPPED,
  LIFT_RUN_LIMIT,
  LIFT_RUN_FAULT
} LiftRunResult;

int32_t Lift_PulsesFromHeightUm(int32_t height_um);
int32_t Lift_HeightUmFromPulses(int32_t pulses);
int32_t Lift_AngleMdegFromHeightUm(int32_t height_um);
int32_t Lift_HeightUmFromAngleMdeg(int32_t angle_mdeg);

/* 上电位置作为软零位；该位置没有绝对传感器确认。 */
void Lift_Init(void);
int32_t Lift_PositionUm(void);
int32_t Lift_PositionPulses(void);
void Lift_SetZero(void);
uint8_t Lift_Busy(void);
LiftStatus Lift_MoveToUm(int32_t height_um);
LiftStatus Lift_JogUm(int32_t delta_um);
void Lift_RequestStop(void);
void Lift_Cancel(void);

/* 执行一段已登记的移动；当前实现会阻塞到该段结束。 */
LiftRunResult Lift_RunPending(void);
uint32_t Lift_LastRequestedPulses(void);
uint32_t Lift_LastIssuedPulses(void);
const char *Lift_StatusText(LiftStatus status);
const char *Lift_RunResultText(LiftRunResult result);

#endif
