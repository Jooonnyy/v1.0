#include "chassis_motion.h"
#include "chassis_config.h"
#include "chassis_io.h"
#include "board.h"
#include "board_imu.h"
#include "board_step6.h"
#include <stdio.h>

/* 点动和四轮协同都是开环：步数 = 频率 × 时间。
 * 三段速度靠 phase 推进，base_steps / base_ms 是本段起点。
 *
 * 陀螺辅助（2026-11-09 加）：
 *   - 转身（set_heading=1）有新鲜陀螺帧时改成闭环**提前停**：按 yaw 误差判到位，
 *     步数仍保留作安全上限，超时用 CS_GYRO_TURN_TIMEOUT_MS；
 *   - 平移（set_heading=0）有新鲜陀螺帧时做航向保持：每 CS_GYRO_HOLD_UPDATE_MS
 *     按 yaw 误差给左右两侧差速（B/D 加、A/C 减 = 逆时针修正）。
 * 没有陀螺帧时这些分支一个都不进，行为和以前完全一样。 */

#define MOVE_PH_RAMP_UP      0U
#define MOVE_PH_CRUISE       1U
#define MOVE_PH_RAMP_DOWN    2U

typedef struct {
  uint8_t  active;
  uint8_t  axis;
  uint32_t target_steps;
  uint32_t hz;
  uint32_t start_ms;
} Jog;

typedef struct {
  uint8_t  active;
  uint8_t  set_heading;
  uint8_t  phase;
  uint8_t  gyro_hold;      /* 1 = 平移时用陀螺保持航向 */
  uint8_t  gyro_turn;      /* 1 = 转身用陀螺判到位 */
  int16_t  heading_deg;
  int8_t   sgn[4];
  int32_t  yaw_target_mdeg;
  int32_t  corr_hz;        /* 航向保持当前下发的差速，用来判"要不要重发" */
  int32_t  corr_i_hz;      /* 航向保持的积分项 */
  int32_t  rate_f_mdps;    /* 角速度低通后的值，D 项用它，避免 100 Hz 下把噪声放大 */
  int32_t  ramp_hz_now;    /* 平滑启动过程中当前这一档的速度 */
  int32_t  ramp_unit_hz;   /* S 型档距的单位量（每档乘权重） */
  uint8_t  ramp_idx;       /* 当前是第几档 */
  uint32_t ramp_step_ms;   /* 每一档持续多少 ms */
  uint32_t ramp_up_ms;     /* 本段实际使用的斜坡时长（短段会按距离缩短） */
  int32_t  turn_hz;        /* 闭环转身当前下发的频率，0 = 还没下发过 */
  int32_t  turn_prev_err_mdeg;   /* 上一拍的转身误差，用来判"越过目标" */
  int8_t   turn_dir;       /* 闭环转身当前方向 */
  uint8_t  turn_have_prev;
  uint32_t target_steps;
  uint32_t gyro_cap_steps;  /* 闭环转身的步数兜底（= deg_to_steps × 余量），只给 gyro_turn_step 用 */
  uint32_t hz;
  uint32_t ramp_hz;
  uint32_t base_steps;
  uint32_t base_ms;
  uint32_t start_ms;
  uint32_t corr_ms;        /* 上一次航向保持修正的时间 */
  uint32_t timeout_ms;     /* 本段超时（开环 120 s / 闭环转身 15 s） */
} Move;

static Jog jog;
static Move move;
static uint8_t hold_enable = 1U;   /* 航向保持开关：现场用 X 0/1 做 A/B 对比 */

void ChassisMotion_SetHoldEnable(uint8_t enable) { hold_enable = (enable != 0U) ? 1U : 0U; }
uint8_t ChassisMotion_GetHoldEnable(void) { return hold_enable; }

/* 把角度差折算到 ±180000 mdeg。 */
static int32_t wrap_mdeg(int32_t d)
{
  while (d > 180000) d -= 360000;
  while (d < -180000) d += 360000;
  return d;
}

/* S 型档位权重：中间快、两头慢（限制加加速度，起步不"窜"）。 */
static int32_t ramp_weight(uint8_t idx, uint8_t steps)
{
  uint8_t half = (uint8_t)(steps / 2U);

  if (half == 0U) return 1;
  if (idx < half) return (int32_t)(idx + 1U);
  return (int32_t)(steps - idx);
}

void ChassisMotion_Init(void)
{
  jog.active = 0U;
  move.active = 0U;
}

void ChassisMotion_Cancel(void)
{
  jog.active = 0U;
  move.active = 0U;
}

uint8_t ChassisMotion_Busy(void)
{
  return (uint8_t)(jog.active | move.active);
}

uint8_t ChassisMotion_JogActive(void) { return jog.active; }
uint8_t ChassisMotion_JogAxis(void) { return jog.axis; }
uint8_t ChassisMotion_MoveActive(void) { return move.active; }

const char *ChassisMotion_AxisName(uint8_t axis)
{
  static const char *names[STEP6_COUNT] = {"A", "B", "C", "D", "ROT", "LIFT"};
  return (axis < STEP6_COUNT) ? names[axis] : "?";
}

void ChassisMotion_JogStart(uint8_t axis, int32_t steps, uint32_t hz)
{
  int8_t dir = (steps > 0) ? 1 : -1;
  uint32_t mag;

  if (steps == 0) { ChassisIo_Reply("ERR steps=0\r\n"); return; }
  if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return; }

  mag = (uint32_t)((steps > 0) ? steps : -steps);
  if (Step6_SetHz((Step6Axis)axis, (int32_t)hz * dir) != HAL_OK) {
    ChassisIo_Reply("ERR set speed failed\r\n");
    return;
  }
  jog.active = 1U;
  jog.axis = axis;
  jog.target_steps = mag;
  jog.hz = hz;
  jog.start_ms = Board_NowMs();
}

static uint32_t hz_from_mm_per_s(uint32_t mm_per_s)
{
  uint32_t hz = (uint32_t)(((uint64_t)mm_per_s * 1000ULL * CS_STEPS_PER_REV) /
                           CS_WHEEL_CIRC_UM);
  if (hz < 1U) hz = 1U;
  if (hz > STEP6_HZ_HARD_LIMIT) hz = STEP6_HZ_HARD_LIMIT;
  return hz;
}

static uint32_t mm_to_steps(uint32_t mm)
{
  return (uint32_t)(((uint64_t)mm * 1000ULL * CS_STEPS_PER_REV) / CS_WHEEL_CIRC_UM);
}

/* 自旋弧长 = 角度 × L。L = lx+ly，与旧车轮速环同一约定。 */
static uint32_t deg_to_steps(int32_t deg)
{
  uint32_t mag = (uint32_t)((deg < 0) ? -deg : deg);
  return (uint32_t)(((uint64_t)mag * CS_SPIN_L_MM * 3142ULL * CS_STEPS_PER_REV) /
                    (180ULL * CS_WHEEL_CIRC_UM));
}

static int16_t norm_deg(int32_t deg)
{
  while (deg < 0) deg += 360;
  while (deg >= 360) deg -= 360;
  return (int16_t)deg;
}

/* 每轴速度微调：轮径/装配差异会让某个轮实际线速度差百分之几，
 * 用 1000 = 不变的千分比系数把四个轮配平（先查机械，最后才动这里）。 */
static int32_t trimmed(int8_t sgn, uint32_t hz, uint16_t trim_permille)
{
  return (int32_t)sgn * (int32_t)(((uint32_t)hz * trim_permille) / 1000U);
}

static uint8_t wheels_set_hz(const int8_t sgn[4], uint32_t hz)
{
  if (Step6_SetHz(STEP6_A, trimmed(sgn[0], hz, CS_TRIM_A_PERMILLE)) != HAL_OK ||
      Step6_SetHz(STEP6_B, trimmed(sgn[1], hz, CS_TRIM_B_PERMILLE)) != HAL_OK ||
      Step6_SetHz(STEP6_C, trimmed(sgn[2], hz, CS_TRIM_C_PERMILLE)) != HAL_OK ||
      Step6_SetHz(STEP6_D, trimmed(sgn[3], hz, CS_TRIM_D_PERMILLE)) != HAL_OK) {
    Step6_StopAll();
    return 0U;
  }
  return 1U;
}

/* 带航向修正的四轮输出：corr_hz > 0 = 想让它逆时针（右侧 B/D 加、左侧 A/C 减）。
 * 平移方向由 sgn[] 决定，修正量与平移方向无关 —— 麦轮逆解里 wz 项就是这样叠加的。 */
static uint8_t wheels_set_corrected(const int8_t sgn[4], uint32_t hz, int32_t corr_hz)
{
  if (Step6_SetHz(STEP6_A, trimmed(sgn[0], hz, CS_TRIM_A_PERMILLE) - corr_hz) != HAL_OK ||
      Step6_SetHz(STEP6_B, trimmed(sgn[1], hz, CS_TRIM_B_PERMILLE) + corr_hz) != HAL_OK ||
      Step6_SetHz(STEP6_C, trimmed(sgn[2], hz, CS_TRIM_C_PERMILLE) - corr_hz) != HAL_OK ||
      Step6_SetHz(STEP6_D, trimmed(sgn[3], hz, CS_TRIM_D_PERMILLE) + corr_hz) != HAL_OK) {
    Step6_StopAll();
    return 0U;
  }
  return 1U;
}

/* turn_delta_deg：转身时相对当前朝向要转多少度（平移传 0）。陀螺用它算目标 yaw。
 * abs_hold = 1：平移的航向目标用调用方给的绝对 yaw（yaw_abs_mdeg），
 *   而不是"下命令那一瞬间的 yaw"。TASK 用它，段与段之间的残差会被下一段拉回去。 */
static uint8_t wheels_run(int32_t a_hz, int32_t b_hz, int32_t c_hz, int32_t d_hz,
                          uint32_t steps, uint8_t set_heading, int16_t heading_deg,
                          int16_t turn_delta_deg, uint8_t abs_hold, int32_t yaw_abs_mdeg)
{
  uint32_t hz, ramp_hz, ramp_ms;
  uint8_t gyro_ok;

  if (steps == 0U) return 0U;

  hz = (uint32_t)((a_hz < 0) ? -a_hz : a_hz);
  ramp_hz = (hz * CS_MOVE_RAMP_PERMILLE) / 1000U;
  if (ramp_hz < 1U) ramp_hz = 1U;
  if (ramp_hz >= hz) ramp_hz = 0U;

  /* 斜坡时长按距离缩：短段（150 mm 对齐那种）如果照用 CS_MOVE_RAMP_UP_MS，
   * 加速+减速两段就把整段距离吃完，车根本到不了巡航速度，还会在斜坡中途被硬停。
   * 这里保证两段斜坡最多各占半段行程，短段自动变成三角形速度曲线。 */
  ramp_ms = CS_MOVE_RAMP_UP_MS;
  if (ramp_hz != 0U && steps > 1U) {
    uint32_t ramp_steps = (uint32_t)((((uint64_t)hz + ramp_hz) / 2U) * ramp_ms / 1000U);

    if (ramp_steps > 0U && (uint64_t)ramp_steps * 2U >= (uint64_t)steps) {
      uint32_t half_steps = (uint32_t)((steps > 2U) ? (steps / 2U) : 1U);

      ramp_ms = (uint32_t)(((uint64_t)CS_MOVE_RAMP_UP_MS * (uint64_t)half_steps) /
                           (uint64_t)ramp_steps);
      if (ramp_ms < CS_MOVE_RAMP_MIN_MS) ramp_ms = CS_MOVE_RAMP_MIN_MS;
    }
  }

  move.sgn[0] = (int8_t)((a_hz < 0) ? -1 : 1);
  move.sgn[1] = (int8_t)((b_hz < 0) ? -1 : 1);
  move.sgn[2] = (int8_t)((c_hz < 0) ? -1 : 1);
  move.sgn[3] = (int8_t)((d_hz < 0) ? -1 : 1);

  if (wheels_set_hz(move.sgn, (ramp_hz != 0U) ? ramp_hz : hz) == 0U) return 0U;

  move.active = 1U;
  move.set_heading = set_heading;
  move.heading_deg = heading_deg;
  move.phase = (ramp_hz != 0U) ? MOVE_PH_RAMP_UP : MOVE_PH_CRUISE;
  move.target_steps = steps;
  move.hz = hz;
  move.ramp_hz = ramp_hz;
  /* 平滑启动：从 ramp_hz 起，分 CS_MOVE_RAMP_UP_STEPS 档爬到全速。
   * 档距按 S 型权重分配（12 档是 1,2,3,4,5,6,6,5,4,3,2,1），即"限制加加速度"：
   * 起步那一下加速度是从 0 慢慢加上去的，冲击最小。 */
  move.ramp_hz_now = (int32_t)((ramp_hz != 0U) ? ramp_hz : hz);
  move.ramp_idx = 0U;
  move.ramp_up_ms = ramp_ms;
  if (ramp_hz != 0U && hz > ramp_hz) {
    int32_t sum_w = 0;
    for (uint8_t i = 0U; i < CS_MOVE_RAMP_UP_STEPS; i++) {
      sum_w += ramp_weight(i, CS_MOVE_RAMP_UP_STEPS);
    }
    if (sum_w < 1) sum_w = 1;
    move.ramp_unit_hz = (int32_t)((hz - ramp_hz) / (uint32_t)sum_w);
    if (move.ramp_unit_hz < 1) move.ramp_unit_hz = 1;
    move.ramp_step_ms = ramp_ms / CS_MOVE_RAMP_UP_STEPS;
    if (move.ramp_step_ms < 1U) move.ramp_step_ms = 1U;
  } else {
    move.ramp_unit_hz = 0;
    move.ramp_step_ms = 1U;
  }
  move.base_steps = 0U;
  move.base_ms = Board_NowMs();
  move.start_ms = move.base_ms;
  move.corr_ms = move.base_ms;
  move.timeout_ms = CS_MOVE_TIMEOUT_MS;
  move.gyro_hold = 0U;
  move.gyro_turn = 0U;
  move.gyro_cap_steps = 0U;
  move.turn_hz = 0;
  move.turn_dir = 0;
  move.corr_hz = 0;
  move.corr_i_hz = 0;
  move.rate_f_mdps = 0;
  move.turn_prev_err_mdeg = 0;
  move.turn_have_prev = 0U;

  /* 有新鲜陀螺帧才启用辅助；没有就完全走原来的开环逻辑。 */
  gyro_ok = (uint8_t)((CS_GYRO_ENABLE != 0U && Imu_IsFresh() != 0U) ? 1U : 0U);
  if (gyro_ok != 0U) {
    if (set_heading != 0U && turn_delta_deg != 0) {
      move.gyro_turn = 1U;
      move.yaw_target_mdeg = Imu_ReadYawMdeg() + (int32_t)turn_delta_deg * 1000;
      move.timeout_ms = CS_GYRO_TURN_TIMEOUT_MS;
      /* 步数只是兜底，必须比几何值留余量：deg_to_steps() 算的是"按模型正好转这么多度"
       * 所需的步数，等于理论值、余量为 0。一旦轮地打滑（转身时麦轮侧滑比直行大），
       * 转到位时的步数会略超这个值，就会被上限截断、报 TURN capped 并留下十几度残差
       * ——2026-11-11 日志里最后那个 180° 就是这么坏的（err=-11674 mdeg）。
       * 真正的失败由 CS_GYRO_TURN_TIMEOUT_MS 兜底，所以这里放宽到 1.5 倍是安全的。 */
      move.gyro_cap_steps = (uint32_t)(((uint64_t)steps * CS_GYRO_TURN_CAP_PERMILLE) / 1000U);
    } else if (set_heading == 0U && hold_enable != 0U) {
      int32_t bias = 0;

      /* 横移（A、B 符号相反）时叠加"蟹行"校准偏置，把斜掉的轨迹掰直。 */
      if (CS_GYRO_STRAFE_BIAS_MDEG != 0 && ((a_hz < 0) != (b_hz < 0))) {
        bias = (a_hz < 0) ? (int32_t)CS_GYRO_STRAFE_BIAS_MDEG
                          : -(int32_t)CS_GYRO_STRAFE_BIAS_MDEG;
      }
      move.gyro_hold = 1U;
      move.yaw_target_mdeg = (abs_hold != 0U) ? (yaw_abs_mdeg + bias)
                                              : (Imu_ReadYawMdeg() + bias);
    }
  }
  return 1U;
}

uint8_t ChassisMotion_Translate(int8_t sign_a, int8_t sign_b,
                                int8_t sign_c, int8_t sign_d,
                                uint32_t mm, uint32_t mm_per_s)
{
  uint32_t hz = hz_from_mm_per_s(mm_per_s);
  uint32_t steps = mm_to_steps(mm);
  return wheels_run((int32_t)sign_a * (int32_t)hz,
                    (int32_t)sign_b * (int32_t)hz,
                    (int32_t)sign_c * (int32_t)hz,
                    (int32_t)sign_d * (int32_t)hz,
                    steps, 0U, 0, 0, 0U, 0);
}

/* 同上，但航向保持用**绝对 yaw**当目标（TASK 的前后左右段用）。
 * 相对模式下每段都把"下命令那一刻的 yaw"当零点，一段偏了下一段接着偏，
 * 一条 28 步的路线跑下来航向能累出十几度；绝对模式下每段都往同一个基准拉。
 * 没有新鲜陀螺帧时 abs_hold 不生效，行为等于 ChassisMotion_Translate。 */
uint8_t ChassisMotion_TranslateAbs(int8_t sign_a, int8_t sign_b,
                                   int8_t sign_c, int8_t sign_d,
                                   uint32_t mm, uint32_t mm_per_s,
                                   int32_t yaw_target_mdeg)
{
  uint32_t hz = hz_from_mm_per_s(mm_per_s);
  uint32_t steps = mm_to_steps(mm);
  return wheels_run((int32_t)sign_a * (int32_t)hz,
                    (int32_t)sign_b * (int32_t)hz,
                    (int32_t)sign_c * (int32_t)hz,
                    (int32_t)sign_d * (int32_t)hz,
                    steps, 0U, 0, 0, 1U, yaw_target_mdeg);
}

/* 按相对角度转身（`A <deg>` 用）：heading_deg 是转完之后记进指令航向的值。 */
static uint8_t spin_by_delta(int16_t delta_deg, int16_t heading_deg)
{
  int32_t hz;
  int8_t ccw;
  uint32_t steps;

  if (delta_deg == 0) return 0U;
  ccw = (delta_deg > 0) ? 1 : -1;
  hz = (int32_t)hz_from_mm_per_s(CS_TASK_SPIN_MM_PER_S);
  steps = deg_to_steps(delta_deg);
  /* 正角度 = 逆时针。符号 A- B+ C- D+，与直行/横移同一套正方向。 */
  return wheels_run((int32_t)(-ccw) * hz, (int32_t)ccw * hz,
                    (int32_t)(-ccw) * hz, (int32_t)ccw * hz,
                    steps, 1U, heading_deg, delta_deg, 0U, 0);
}

uint8_t ChassisMotion_Spin(int16_t delta_deg, int16_t heading_now_deg)
{
  return spin_by_delta(delta_deg, norm_deg((int32_t)heading_now_deg + delta_deg));
}

/* 按**绝对 yaw** 转身（TASK 的 TURN_ABS 用）：每站都对着同一套绝对航向，
 * 前几站留下的 0.3° 残差不会一站站累积。
 * 没有新鲜陀螺帧时返回 0，调用方应退回 ChassisMotion_Spin()。 */
uint8_t ChassisMotion_SpinAbs(int16_t heading_deg, int32_t yaw_target_mdeg)
{
  int32_t delta_mdeg;
  int16_t delta_deg;

  if (Imu_IsFresh() == 0U) return 0U;

  delta_mdeg = wrap_mdeg(yaw_target_mdeg - Imu_ReadYawMdeg());
  delta_deg = (int16_t)(delta_mdeg / 1000);
  if (delta_deg == 0) delta_deg = (delta_mdeg > 0) ? 1 : -1;   /* 差不到 1° 也给一个微调 */

  return spin_by_delta(delta_deg, heading_deg);
}

void ChassisMotion_PollJog(void)
{
  char line[64];
  uint32_t elapsed_ms, done;
  uint32_t hz_now;

  if (!jog.active) return;

  elapsed_ms = Board_NowMs() - jog.start_ms;
  hz_now = Step6_GetHz((Step6Axis)jog.axis);
  done = (uint32_t)(((uint64_t)hz_now * (uint64_t)elapsed_ms) / 1000U);

  if (done >= jog.target_steps) {
    Step6_Stop((Step6Axis)jog.axis);
    jog.active = 0U;
    (void)snprintf(line, sizeof(line), "J %s done, %lu steps, %lu ms\r\n",
                   ChassisMotion_AxisName(jog.axis),
                   (unsigned long)jog.target_steps,
                   (unsigned long)elapsed_ms);
    ChassisIo_Reply(line);
  } else if (elapsed_ms > CS_JOG_TIMEOUT_MS) {
    Step6_Stop((Step6Axis)jog.axis);
    jog.active = 0U;
    (void)snprintf(line, sizeof(line), "J %s timeout at %lu steps\r\n",
                   ChassisMotion_AxisName(jog.axis), (unsigned long)done);
    ChassisIo_Reply(line);
  }
}

/* 陀螺闭环转身：单独一条控制路径。
 * 和开环那条的区别：
 *   - 转速按剩余误差比例给（越接近目标越慢），所以到点前角速度自然降下来；
 *   - 步数积分用"上一拍实际下发的频率"，因为频率每拍都在变，不能再拿 move.hz 估；
 *   - 到位判据 = 误差小 且 转得慢；步数上限和超时只作兜底。 */
static void gyro_turn_step(uint32_t now_ms, uint32_t elapsed_ms, ChassisMotionEvent *event)
{
  char line[72];
  int32_t err, rate, mag, hz_cmd, dir;
  uint32_t hz_last, done;

  err = wrap_mdeg(move.yaw_target_mdeg - Imu_ReadYawMdeg());
  mag = (err < 0) ? -err : err;
  rate = Imu_ReadRateMdps();

  hz_last = Step6_GetHz(STEP6_B);
  done = move.base_steps +
         (uint32_t)(((uint64_t)hz_last * (uint64_t)(now_ms - move.base_ms)) / 1000U);
  move.base_steps = done;
  move.base_ms = now_ms;

  /* 越过目标点就停：进减速区后误差符号一变，说明刚扫过目标，
   * 此时误差只剩"一帧走过的角量"，比放宽的容差准得多。 */
  if (move.turn_have_prev != 0U &&
      ((err > 0) != (move.turn_prev_err_mdeg > 0)) &&
      move.turn_prev_err_mdeg <= CS_GYRO_TURN_SLOW_ZONE_MDEG &&
      move.turn_prev_err_mdeg >= -CS_GYRO_TURN_SLOW_ZONE_MDEG) {
    Step6_StopAll();
    move.active = 0U;
    if (move.set_heading) {
      event->set_heading = 1U;
      event->heading_deg = move.heading_deg;
    }
    (void)snprintf(line, sizeof(line), "TURN done (gyro crossed, err=%ld mdeg, %lu ms)\r\n",
                   (long)err, (unsigned long)elapsed_ms);
    ChassisIo_Reply(line);
    return;
  }

  /* 到位：只看位置误差。速度已经按误差比例降下来了，不需要再等"角速度变小"，
   * 否则低速时永远满足不了判据（会一直转到超时）。 */
  if (mag <= CS_GYRO_TURN_TOL_MDEG) {
    Step6_StopAll();
    move.active = 0U;
    if (move.set_heading) {
      event->set_heading = 1U;
      event->heading_deg = move.heading_deg;
    }
    (void)snprintf(line, sizeof(line), "TURN done (gyro err=%ld mdeg, rate=%ld, %lu ms)\r\n",
                   (long)err, (long)rate, (unsigned long)elapsed_ms);
    ChassisIo_Reply(line);
    return;
  }

  /* 兜底：超时或步数上限到了 */
  if (elapsed_ms > move.timeout_ms || done >= move.gyro_cap_steps) {
    Step6_StopAll();
    move.active = 0U;
    if (move.set_heading) {
      event->set_heading = 1U;
      event->heading_deg = move.heading_deg;
    }
    (void)snprintf(line, sizeof(line), "TURN capped (err=%ld mdeg, %lu steps, %lu ms)\r\n",
                   (long)err, (unsigned long)done, (unsigned long)elapsed_ms);
    ChassisIo_Reply(line);
    return;
  }

  /* 两段速：远离目标用恒速，进入减速区降一次速，到位停。
   * 每拍改频率都会让 Step6_SetHz() 重配 PSC/ARR（通道要停一下），手感就是"一顿一顿"，
   * 所以整条转身只允许两次变速：进减速区一次、过冲后反向纠偏一次。 */
  hz_cmd = (mag <= CS_GYRO_TURN_SLOW_ZONE_MDEG)
             ? (int32_t)CS_GYRO_TURN_SLOW_HZ : (int32_t)move.hz;

  /* err > 0：还差逆时针 → B/D 正、A/C 负（与 Spin 的正角度符号一致） */
  dir = (err > 0) ? 1 : -1;
  if (hz_cmd != move.turn_hz || dir != move.turn_dir) {
    move.turn_hz = hz_cmd;
    move.turn_dir = (int8_t)dir;
    (void)Step6_SetHz(STEP6_A, -dir * hz_cmd);
    (void)Step6_SetHz(STEP6_B, dir * hz_cmd);
    (void)Step6_SetHz(STEP6_C, -dir * hz_cmd);
    (void)Step6_SetHz(STEP6_D, dir * hz_cmd);
  }

  move.turn_prev_err_mdeg = err;
  move.turn_have_prev = 1U;
}

void ChassisMotion_PollMove(uint8_t task_active, ChassisMotionEvent *event)
{
  uint32_t now, elapsed_ms, done, hz_now, decel_steps;
  char line[64];

  event->timed_out = 0U;
  event->set_heading = 0U;
  event->heading_deg = 0;
  if (!move.active) return;

  now = Board_NowMs();
  elapsed_ms = now - move.start_ms;

  /* 陀螺闭环转身走独立路径，不参与下面的开环积分/斜坡逻辑。 */
  if (move.gyro_turn != 0U && Imu_IsFresh() != 0U) {
    gyro_turn_step(now, elapsed_ms, event);
    return;
  }

  if (move.phase == MOVE_PH_RAMP_UP) {
    if (move.ramp_unit_hz == 0) {
      move.ramp_hz_now = (int32_t)move.hz;
      move.phase = MOVE_PH_CRUISE;
    } else if ((uint32_t)(now - move.base_ms) >= move.ramp_step_ms) {
      /* 一档一档往上爬，档距按 S 型权重（1,2,3,4,4,3,2,1）：
       * 起步加速度从 0 缓增、到中段最大、末段再收，冲击比等分档更小。 */
      uint32_t step_ms = move.ramp_step_ms;

      move.base_steps += (uint32_t)(((uint64_t)move.ramp_hz_now * step_ms) / 1000U);
      move.base_ms += step_ms;
      move.ramp_hz_now += ramp_weight(move.ramp_idx, CS_MOVE_RAMP_UP_STEPS) * move.ramp_unit_hz;
      if (move.ramp_idx + 1U < CS_MOVE_RAMP_UP_STEPS) move.ramp_idx++;
      if (move.ramp_hz_now >= (int32_t)move.hz) {
        move.ramp_hz_now = (int32_t)move.hz;
        move.phase = MOVE_PH_CRUISE;
      }
      if (wheels_set_corrected(move.sgn, (uint32_t)move.ramp_hz_now, move.corr_hz) == 0U) {
        move.active = 0U;
        ChassisIo_Reply("ERR ramp up failed\r\n");
        return;
      }
    }
  }

  /* 加减速两段都用 ramp_hz_now，恒速段用目标频率。 */
  hz_now = (move.phase == MOVE_PH_CRUISE) ? move.hz : (uint32_t)move.ramp_hz_now;
  done = move.base_steps +
         (uint32_t)(((uint64_t)hz_now * (uint64_t)(now - move.base_ms)) / 1000U);

  /* ---- 陀螺航向保持：平移时每 CS_GYRO_HOLD_UPDATE_MS 修一次左右差速 ---- */
  if (move.gyro_hold != 0U) {
    if ((uint32_t)(now - move.corr_ms) >= CS_GYRO_HOLD_UPDATE_MS) {
      int32_t err, rate, corr, limit, kd, p_hz;

      move.corr_ms = now;
      if (Imu_IsFresh() != 0U) {
        err = wrap_mdeg(move.yaw_target_mdeg - Imu_ReadYawMdeg());
        rate = Imu_ReadRateMdps();
        /* 角速度低通：100 Hz 采样下差分噪声不小，D 项直接用会把抖动放大。 */
        move.rate_f_mdps += (rate - move.rate_f_mdps) / (int32_t)(1 << CS_GYRO_HOLD_RATE_SHIFT);
        rate = move.rate_f_mdps;
        limit = (int32_t)(((uint64_t)hz_now * CS_GYRO_HOLD_MAX_PERMILLE) / 1000U);
        if (limit < 1) limit = 1;

        /* 比例项：先用 mdeg 乘再除。旧写法 (err/1000) 会把 1 度以内的误差**全截成 0**，
         * 也就是说实际死区是 1 度，不是 CS_GYRO_HOLD_DEADBAND_MDEG；最后那 1 度永远修不掉，
         * 每段结束都留 0.7~1.0 度的残差，TASK 里还会一段段累下去。 */
        p_hz = 0;
        if (err > CS_GYRO_HOLD_DEADBAND_MDEG || err < -CS_GYRO_HOLD_DEADBAND_MDEG) {
          p_hz = (err * CS_GYRO_HOLD_KP_HZ_PER_DEG) / 1000;
        }
        corr = p_hz;

        /* 起步阶段位置误差还没长起来，只能靠阻尼抢在它前面：把 D 增益临时加大，
         * 一有偏航角速度就反打，压制"起步甩尾"。 */
        kd = ((uint32_t)(now - move.start_ms) < CS_GYRO_HOLD_LAUNCH_MS)
               ? CS_GYRO_HOLD_KD_LAUNCH_HZ_PER_DPS : CS_GYRO_HOLD_KD_HZ_PER_DPS;
        corr -= (rate / 1000) * kd;

        /* 积分项：横移时的偏航扰动是恒定的，纯 P 一定留固定残差，靠 I 消掉。
         * 旧实现有三处会把自己锁死，横移一个 1 m 段就能把积分顶到 40% 基速（≈2170 Hz），
         * 然后车被推过头、误差超过 ±5 度、累积条件不成立 → 积分**永远退不回来**，
         * 平衡在 I/Kp ≈ 14 度的固定偏角上。这就是"横移偏差之后修正不过来"。
         * 现在三条约束一起上：
         *   ① 只在 |误差| > 死区 时累积（死区内累积会自己长出一个偏置）；
         *   ② 上限用独立的 CS_GYRO_HOLD_I_LIMIT_HZ，远小于总限幅；
         *   ③ 输出已经顶到限幅、误差还往同一方向推时**不累积**（条件积分，防 windup）。 */
        if (err > CS_GYRO_HOLD_DEADBAND_MDEG || err < -CS_GYRO_HOLD_DEADBAND_MDEG) {
          int32_t total_hz = p_hz + move.corr_i_hz;
          uint8_t sat_hi = (uint8_t)(total_hz >= limit);
          uint8_t sat_lo = (uint8_t)(total_hz <= -limit);

          if (!((err > 0) && sat_hi) && !((err < 0) && sat_lo)) {
            move.corr_i_hz += (err * CS_GYRO_HOLD_KI_HZ_PER_DEG_S) / 100000;
          }
        }
        if (move.corr_i_hz > (int32_t)CS_GYRO_HOLD_I_LIMIT_HZ) {
          move.corr_i_hz = (int32_t)CS_GYRO_HOLD_I_LIMIT_HZ;
        }
        if (move.corr_i_hz < -(int32_t)CS_GYRO_HOLD_I_LIMIT_HZ) {
          move.corr_i_hz = -(int32_t)CS_GYRO_HOLD_I_LIMIT_HZ;
        }
        corr += move.corr_i_hz;

        if (corr > limit) corr = limit;
        if (corr < -limit) corr = -limit;
        /* 只有修正量明显变化才重发：每次重发都要重配四个定时器的 PSC/ARR。 */
        if (corr - move.corr_hz >= CS_GYRO_HOLD_MIN_STEP_HZ ||
            move.corr_hz - corr >= CS_GYRO_HOLD_MIN_STEP_HZ ||
            (corr == 0 && move.corr_hz != 0)) {
          if (wheels_set_corrected(move.sgn, hz_now, corr) == 0U) {
            move.active = 0U;
            ChassisIo_Reply("ERR heading hold failed\r\n");
            return;
          }
          move.corr_hz = corr;
        }
      }
    }
  }

  if (move.phase == MOVE_PH_CRUISE && move.ramp_unit_hz != 0 &&
      done < move.target_steps) {
    /* 减速窗口 = 从全速按同样档数爬回起步速度所覆盖的步数，
     * 这样减速和加速对称，停车不会再有"顿一下"的甩尾。
     * 斜坡时长用 move.ramp_up_ms（短段被按距离缩短过），不能再用编译期常量。 */
    decel_steps = (uint32_t)((((uint64_t)move.hz + move.ramp_hz) / 2U) *
                             move.ramp_up_ms / 1000U);
    if (move.target_steps - done <= decel_steps) {
      move.ramp_hz_now = (int32_t)move.hz;
      move.ramp_idx = 0U;
      move.base_steps = done;
      move.base_ms = now;
      move.phase = MOVE_PH_RAMP_DOWN;
    }
  }

  if (move.phase == MOVE_PH_RAMP_DOWN && move.ramp_unit_hz != 0 &&
      (uint32_t)(now - move.base_ms) >= move.ramp_step_ms) {
    uint32_t step_ms = move.ramp_step_ms;
    int32_t w = ramp_weight((uint8_t)(CS_MOVE_RAMP_UP_STEPS - 1U - move.ramp_idx),
                            CS_MOVE_RAMP_UP_STEPS);

    move.base_steps += (uint32_t)(((uint64_t)move.ramp_hz_now * step_ms) / 1000U);
    move.base_ms += step_ms;
    move.ramp_hz_now -= w * move.ramp_unit_hz;
    if (move.ramp_idx + 1U < CS_MOVE_RAMP_UP_STEPS) move.ramp_idx++;
    if (move.ramp_hz_now < (int32_t)move.ramp_hz) {
      move.ramp_hz_now = (int32_t)move.ramp_hz;
    }
    if (wheels_set_corrected(move.sgn, (uint32_t)move.ramp_hz_now, move.corr_hz) == 0U) {
      move.active = 0U;
      ChassisIo_Reply("ERR ramp down failed\r\n");
      return;
    }
  }

  /* ---- 陀螺闭环转身由 gyro_turn_step() 单独处理，这里只剩开环兜底 ---- */

  if (done < move.target_steps && elapsed_ms <= move.timeout_ms) return;

  Step6_StopAll();
  move.active = 0U;
  if (elapsed_ms > move.timeout_ms && done < move.target_steps) {
    if (task_active) {
      event->timed_out = 1U;
    } else {
      (void)snprintf(line, sizeof(line), "MOVE timeout at %lu/%lu steps\r\n",
                     (unsigned long)done, (unsigned long)move.target_steps);
      ChassisIo_Reply(line);
    }
    return;
  }
  if (move.set_heading) {
    event->set_heading = 1U;
    event->heading_deg = move.heading_deg;
  }
  /* 有陀螺保持时把这一段的残余航向误差报出来，便于判断保持强度够不够。 */
  if (move.gyro_hold != 0U && Imu_IsFresh() != 0U) {
    int32_t herr = wrap_mdeg(move.yaw_target_mdeg - Imu_ReadYawMdeg());
    (void)snprintf(line, sizeof(line), "MOVE done %lu steps %lu ms (hdg err=%ld mdeg)\r\n",
                   (unsigned long)move.target_steps, (unsigned long)elapsed_ms, (long)herr);
  } else {
    (void)snprintf(line, sizeof(line), "MOVE done %lu steps %lu ms\r\n",
                   (unsigned long)move.target_steps, (unsigned long)elapsed_ms);
  }
  ChassisIo_Reply(line);
}
