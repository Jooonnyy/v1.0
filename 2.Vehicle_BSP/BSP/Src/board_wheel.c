#include "board_wheel.h"

/* 角速度换算用的整数系数：
 *   v_rot[um/s] = L[um] × wz[mdeg/s] × (π / 180 / 1000)
 *               = L × wz × 17453 / 1e9        （17453 ≈ 1e9 × π / 180000）
 * 验算：L = 180000 um、wz = 1000000 mdeg/s（= 1000 dps）
 *       -> 180000 × 1e6 × 17453 / 1e9 = 3.14e6 um/s = 3.14 m/s ✓（0.18 m 臂长、1000 dps） */
#define WHEEL_MDEG_TO_RAD_NUM    17453LL
#define WHEEL_MDEG_TO_RAD_DEN 1000000000LL

/* 全体由 Wheel_Update() 写、被主循环读。对齐的 32 位读写是单指令，不会撕裂。 */
static int32_t target_cps[4];        /* 上层下发的目标，单位 counts/s */
static int32_t ramp_cps[4];          /* 软启动之后的实际目标 */
static int32_t output_permille[4];   /* 本拍写出去的占空比（限幅后） */
static int32_t integral_permille[4]; /* 积分项，单位千分比 */
static uint32_t stall_ms[4];
static uint8_t enabled;
static WheelState state;
static uint32_t stall_events;
/* 停机刹车的剩余时间，单位 ms。>0 期间四路保持短接制动，到点后转滑行。 */
static uint16_t brake_ms_left;

static int16_t kp_gain = (int16_t)WHEEL_KP_Q10;
static int16_t ki_gain = (int16_t)WHEEL_KI_Q10;
static int16_t ff_gain = (int16_t)WHEEL_FF_PERMILLE_PER_1000CPS;

static const int8_t motor_sign[4] = {
  (int8_t)WHEEL_MOTOR_SIGN_A, (int8_t)WHEEL_MOTOR_SIGN_B,
  (int8_t)WHEEL_MOTOR_SIGN_C, (int8_t)WHEEL_MOTOR_SIGN_D
};

/* 每个电机对应车体的哪个角（0=FL 1=FR 2=RL 3=RR）。 */
static const uint8_t corner_of_motor[4] = {
  (uint8_t)WHEEL_CORNER_A, (uint8_t)WHEEL_CORNER_B,
  (uint8_t)WHEEL_CORNER_C, (uint8_t)WHEEL_CORNER_D
};

static int32_t clamp_target(int32_t value_cps)
{
  if (value_cps > (int32_t)WHEEL_TARGET_LIMIT_CPS) return (int32_t)WHEEL_TARGET_LIMIT_CPS;
  if (value_cps < -(int32_t)WHEEL_TARGET_LIMIT_CPS) return -(int32_t)WHEEL_TARGET_LIMIT_CPS;
  return value_cps;
}

void Wheel_Init(void)
{
  uint32_t i;

  for (i = 0U; i < 4U; ++i) {
    target_cps[i] = 0;
    ramp_cps[i] = 0;
    output_permille[i] = 0;
    integral_permille[i] = 0;
    stall_ms[i] = 0U;
  }
  enabled = 0U;
  state = WHEEL_STATE_IDLE;
  stall_events = 0U;
  brake_ms_left = 0U;
  kp_gain = (int16_t)WHEEL_KP_Q10;
  ki_gain = (int16_t)WHEEL_KI_Q10;
  ff_gain = (int16_t)WHEEL_FF_PERMILLE_PER_1000CPS;
}

void Wheel_Enable(uint8_t enable)
{
  uint32_t i;

  if (enable == 0U) {
    Wheel_Stop();
    return;
  }
  brake_ms_left = 0U;   /* 刹车窗口内被重新使能：立刻取消刹车，接管四路输出 */
  /* AT8236 没有 STBY，Motor_Standby() 现在是空操作；保留这次调用是为了
   * 兼容旧调用点，并保证以后换回带 STBY 的驱动时行为不变。 */
  Motor_Standby(0U, 1U);
  Motor_Standby(1U, 1U);
  for (i = 0U; i < 4U; ++i) {
    target_cps[i] = 0;
    ramp_cps[i] = 0;
    integral_permille[i] = 0;
    stall_ms[i] = 0U;
  }
  enabled = 1U;
  state = WHEEL_STATE_RUN;
}

uint8_t Wheel_IsEnabled(void)
{
  return enabled;
}

uint8_t Wheel_IsBraking(void)
{
  return (brake_ms_left != 0U) ? 1U : 0U;
}

void Wheel_BrakeCancel(void)
{
  /* 只清计时，不动输出：调用方紧接着会自己写 Motor_Set()。
   * 开环命令（D / ALL / CMP）必须调它，否则刹车计时器到点会把开环占空比清零。 */
  brake_ms_left = 0U;
}

void Wheel_Stop(void)
{
  uint32_t i;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();   /* 别让 tick 在清零中途往电机写 PWM */
  for (i = 0U; i < 4U; ++i) {
    target_cps[i] = 0;
    ramp_cps[i] = 0;
    integral_permille[i] = 0;
    output_permille[i] = 0;
    stall_ms[i] = 0U;
  }
  enabled = 0U;
  state = WHEEL_STATE_IDLE;
  __set_PRIMASK(primask);

#if (WHEEL_BRAKE_MS > 0U)
  /* 主动刹车：短接电机做动态制动，比滑行停得快。到点由 Wheel_Update() 释放。 */
  Motor_BrakeAll();
  brake_ms_left = (uint16_t)WHEEL_BRAKE_MS;
#else
  Motor_StopAll();   /* 四路 PWM=0、DIR=0（AT8236 滑行） */
#endif
}

void Wheel_SetTargetCountsPerSec(MotorId id, int32_t target_counts_per_s)
{
  if ((unsigned)id >= 4U) return;
  target_cps[(uint32_t)id] = clamp_target(target_counts_per_s);
}

void Wheel_SetBodyVelocity(int32_t vx_um_per_s, int32_t vy_um_per_s, int32_t wz_mdps)
{
  int32_t corner_um_per_s[4];
  int32_t vy_signed = vy_um_per_s * (int32_t)WHEEL_LATERAL_SIGN;
  int32_t rot_um_per_s;
  int64_t rot_wide;
  uint32_t i;

  /* L·ωz：L = 半轴距 + 半轮距，是麦轮自转项的唯一标度。 */
  rot_wide = ((int64_t)(CHASSIS_HALF_LENGTH_UM + CHASSIS_HALF_WIDTH_UM) *
              (int64_t)wz_mdps * WHEEL_MDEG_TO_RAD_NUM) / WHEEL_MDEG_TO_RAD_DEN;
  rot_um_per_s = (int32_t)rot_wide;
  if (WHEEL_SPIN_SIGN < 0) rot_um_per_s = -rot_um_per_s;

  /* X 型麦轮逆解算（x 向前、y 向左、ωz 逆时针为正）。 */
  corner_um_per_s[0] = vx_um_per_s - vy_signed - rot_um_per_s;   /* 左前 */
  corner_um_per_s[1] = vx_um_per_s + vy_signed + rot_um_per_s;   /* 右前 */
  corner_um_per_s[2] = vx_um_per_s + vy_signed - rot_um_per_s;   /* 左后 */
  corner_um_per_s[3] = vx_um_per_s - vy_signed + rot_um_per_s;   /* 右后 */

  for (i = 0U; i < 4U; ++i) {
    target_cps[i] = clamp_target(
        Encoder_CountsPerSecFromUmPerSec(corner_um_per_s[corner_of_motor[i]]));
  }
}

void Wheel_Update(void)
{
  uint32_t i;

  /* 停机刹车倒计时。到点松成滑行，避免电机被长时间短接发热。
   * 这段要在 enabled 判断之前处理：刹车期间闭环本来就是关着的。 */
  if (brake_ms_left != 0U) {
    if (brake_ms_left > WHEEL_CTRL_PERIOD_MS) {
      brake_ms_left = (uint16_t)(brake_ms_left - WHEEL_CTRL_PERIOD_MS);
    } else {
      brake_ms_left = 0U;
      Motor_StopAll();
    }
    return;
  }

  if (enabled == 0U) return;

  for (i = 0U; i < 4U; ++i) {
    int32_t meas_cps = Encoder_SpeedCountsPerSec((EncoderId)i);
    int32_t err, p_term, ff_term, out_raw, i_step, i_limit;
    int32_t abs_out, abs_meas, abs_ramp;
    uint8_t sat_high, sat_low;

    /* 软启动：本拍真正使用的目标朝 target_cps 逼近，避免阶跃造成打滑和电流冲击。 */
    if (target_cps[i] > ramp_cps[i] + (int32_t)WHEEL_TARGET_SLEW_CPS) {
      ramp_cps[i] += (int32_t)WHEEL_TARGET_SLEW_CPS;
    } else if (target_cps[i] < ramp_cps[i] - (int32_t)WHEEL_TARGET_SLEW_CPS) {
      ramp_cps[i] -= (int32_t)WHEEL_TARGET_SLEW_CPS;
    } else {
      ramp_cps[i] = target_cps[i];
    }

    /* 目标为 0：直接断电停机并清积分，避免"给了 0 还在转"。 */
    if (ramp_cps[i] == 0) {
      integral_permille[i] = 0;
      output_permille[i] = 0;
      stall_ms[i] = 0U;
      (void)Motor_Set((MotorId)i, 0);
      continue;
    }

    err = ramp_cps[i] - meas_cps;
    p_term = ((int32_t)kp_gain * err) >> 10;
    ff_term = (ramp_cps[i] * (int32_t)ff_gain) / 1000;
    /* 摩擦前馈：低速时电机要先克服静摩擦，纯比例前馈会低估，这部分差额由积分补，
     * 积分一补就 windup。所以单独加一个与目标方向一致的常数项。 */
    if (ramp_cps[i] > 0) ff_term += (int32_t)WHEEL_FF_OFFSET_PERMILLE;
    else ff_term -= (int32_t)WHEEL_FF_OFFSET_PERMILLE;
    /* 反转是 AT8236 的快衰减弱方向：同样转速要多给 1.5~2 倍占空比，而上面这套前馈
     * 只在正转上标定过。不补这一截，缺口全由积分扛，一旦顶到积分限幅那一路就永远差速。
     * 2026-10-27 实测见参数区注释。 */
    if (ramp_cps[i] < 0)
      ff_term = ff_term * (int32_t)WHEEL_FF_REVERSE_BOOST_PERMILLE / 1000;
    out_raw = ff_term + p_term + integral_permille[i];

    /* 条件积分（anti-windup）：输出已经顶到限幅、误差还往同方向推时不再累加。 */
    sat_high = (out_raw >= (int32_t)WHEEL_DUTY_LIMIT_PERMILLE) ? 1U : 0U;
    sat_low = (out_raw <= -(int32_t)WHEEL_DUTY_LIMIT_PERMILLE) ? 1U : 0U;
    if (!((sat_high != 0U && err > 0) || (sat_low != 0U && err < 0))) {
      i_step = ((int32_t)ki_gain * err) >> 10;
      integral_permille[i] += i_step;
      /* 反转方向要更大的积分权限：快衰减下同样的转速要多给一大截占空比，
       * 前馈补不完的部分只能靠积分。正转仍用 150，避免起步超调。 */
      i_limit = (ramp_cps[i] < 0) ? (int32_t)WHEEL_INTEGRAL_LIMIT_REVERSE_PERMILLE
                                  : (int32_t)WHEEL_INTEGRAL_LIMIT_PERMILLE;
      if (integral_permille[i] > i_limit) {
        integral_permille[i] = i_limit;
      } else if (integral_permille[i] < -i_limit) {
        integral_permille[i] = -i_limit;
      }
      out_raw = ff_term + p_term + integral_permille[i];
    }

    /* 先限幅，再写 PWM（AGENTS.md 第 6 节硬性要求）。 */
    if (out_raw > (int32_t)WHEEL_DUTY_LIMIT_PERMILLE) out_raw = (int32_t)WHEEL_DUTY_LIMIT_PERMILLE;
    if (out_raw < -(int32_t)WHEEL_DUTY_LIMIT_PERMILLE) out_raw = -(int32_t)WHEEL_DUTY_LIMIT_PERMILLE;
    output_permille[i] = out_raw;
    (void)Motor_Set((MotorId)i, (int16_t)(out_raw * (int32_t)motor_sign[i]));

    /* 堵转检测：输出顶到限幅、实际几乎不动、而目标要得更多 -> 判定堵转。 */
    if (WHEEL_STALL_CPS > 0) {
      abs_out = (out_raw < 0) ? -out_raw : out_raw;
      abs_meas = (meas_cps < 0) ? -meas_cps : meas_cps;
      abs_ramp = (ramp_cps[i] < 0) ? -ramp_cps[i] : ramp_cps[i];
      if (abs_out >= (int32_t)WHEEL_DUTY_LIMIT_PERMILLE - 20 &&
          abs_meas < (int32_t)WHEEL_STALL_CPS &&
          abs_ramp > (int32_t)WHEEL_STALL_CPS * 2) {
        stall_ms[i] += WHEEL_CTRL_PERIOD_MS;
        if (stall_ms[i] >= WHEEL_STALL_MS) {
          stall_events++;
          Wheel_Stop();
          state = WHEEL_STATE_STALL;   /* 要在 Wheel_Stop() 之后置，否则会被它清成 IDLE */
          return;
        }
      } else {
        stall_ms[i] = 0U;
      }
    }
  }
}

int32_t Wheel_TargetCountsPerSec(MotorId id)
{
  if ((unsigned)id >= 4U) return 0;
  return target_cps[(uint32_t)id];
}

int32_t Wheel_OutputPermille(MotorId id)
{
  if ((unsigned)id >= 4U) return 0;
  return output_permille[(uint32_t)id];
}

WheelState Wheel_State(void)
{
  return state;
}

uint32_t Wheel_StallEvents(void)
{
  return stall_events;
}

void Wheel_SetGains(int16_t kp_q10, int16_t ki_q10, int16_t ff_per_1000cps)
{
  if (kp_q10 >= 0) kp_gain = kp_q10;
  if (ki_q10 >= 0) ki_gain = ki_q10;
  if (ff_per_1000cps >= 0) ff_gain = ff_per_1000cps;
}

void Wheel_GetGains(int16_t *kp_q10, int16_t *ki_q10, int16_t *ff_per_1000cps)
{
  if (kp_q10 != NULL) *kp_q10 = kp_gain;
  if (ki_q10 != NULL) *ki_q10 = ki_gain;
  if (ff_per_1000cps != NULL) *ff_per_1000cps = ff_gain;
}
