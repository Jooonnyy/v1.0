#include "pick_app.h"
#include "pick_config.h"
#include "align_app.h"
#include "chassis_io.h"
#include "board.h"
#include "board_step6.h"
#include <stdio.h>

/* 机构动作层实现。所有等待都是"记时间戳 + 每轮比一次"，没有 HAL_Delay。 */

/* 单轴点动（旋转台 / 升降台共用）：步数 = 频率 × 时间，和底盘 `J` 同一套估算。
 * 三段速度：起转 40% → 恒速 → 收转 40%。提速必须配斜坡，否则驱动器跟不上阶跃。 */
typedef struct {
  uint8_t  active;
  uint8_t  axis;
  uint8_t  phase;
  int8_t   dir;
  uint32_t target_pulses;
  uint32_t hz;
  uint32_t ramp_hz;
  uint32_t base_pulses;      /* 本段起点的累计脉冲 */
  uint32_t base_ms;          /* 本段起点的时间戳 */
  uint32_t start_ms;         /* 整条命令的起点，用来判超时 */
} AxisMove;

#define AXIS_PH_RAMP_UP      0U
#define AXIS_PH_CRUISE       1U
#define AXIS_PH_RAMP_DOWN    2U

static AxisMove axis_move;

/* 舵机状态（只为回显和 STOP，不影响 BSP 内部状态）。 */
static uint8_t  arm_on;
static uint16_t arm_pulse_us;
static uint8_t  plat_on;
static int16_t  plat_permille;
static uint8_t  plat_burst_active;   /* PLT 定时点动在进行 */
static uint32_t plat_burst_stop_ms;
/* 升降累计位置（脉冲，正 = 下降）。起点 0；每次 Pick_LiftPulses 成功后累加。
 * ⚠ 这是**指令值**不是实测：中途 STOP、超时、或丝杆打滑都会让它和真实位置对不上，
 * 需要时把升降手动开到最高处再发 `LIFT ZERO` 重新对齐。 */
static int32_t  lift_pos_pulses;
/* 转台累计角度（毫度），只记我们发过的 ROT 命令；用来告诉视觉当前臂角。 */
/* 上电默认转台在载物盘侧（CYC 放料侧，-PK_CYC_ROT_DEG），TASK stp2 会先回取料侧。 */
static int32_t  rot_pos_mdeg = -(int32_t)PK_CYC_ROT_DEG * 1000;

/* 任务循环 CYC（三段式：开场 → 每轮 → 收尾） */
static uint8_t  cycle_active;
static uint8_t  cycle_phase;      /* CYC_PHASE_* */
static uint8_t  cycle_step;       /* 当前阶段表里的第几步 */
static uint8_t  cycle_round;      /* 已跑完的轮数 */
static uint8_t  cycle_rounds;     /* 本次任务的总轮数 */
static uint8_t  cycle_last_ok;    /* 上一次 CYC 是否**正常跑满**（1）；被中止/出错为 0 */
static uint32_t cycle_step_ms;
static uint32_t cycle_wait_ms;    /* 舵机到位的等待截止时刻 */
/* CYC 在跑时把单步的回显关掉：一轮 7 步 × 3 轮 × 6 个工位会把串口刷爆，
 * 现场根本看不到 TASK 的走位行。只保留 CYC 自己的开始/结束行和真正的错误/超时。 */
static uint8_t  cyc_quiet;

/* 卸料序列 TRG：载物台夹起 → 降到地面 → 松开 → 升回最高处 */
static uint8_t  trg_active;
static uint8_t  trg_step;
static uint8_t  trg_round;
static uint8_t  trg_rounds;
static uint32_t trg_step_ms;
static uint32_t trg_wait_ms;

/* 视觉对准之后的单次抓取。和 TRG 同一套非阻塞骨架，只跑一遍，结束时回到最高处。 */
static uint8_t  grab_active;
static uint8_t  grab_step;
static uint32_t grab_step_ms;
static uint32_t grab_wait_ms;

/* 圆环放置 DROP：空爪对准 → 取料 → 下放。对准之后底盘不再动。 */
static uint8_t  drop_active;
static uint8_t  drop_dry;
static uint8_t  drop_ok;
static uint32_t drop_deadline_ms;
static uint8_t  drop_step;
static uint32_t drop_step_ms;
static uint32_t drop_wait_ms;
static uint8_t  home_pending;           /* 动作结束/中止后自动回最高处 */
static uint8_t  drop_ring;              /* DROP 对准第几个圆环（1..9）*/

static uint16_t arm_us_from_deg(int32_t deg)
{
  int32_t span_us = (int32_t)(PK_ARM_US_MAX - PK_ARM_US_MIN);
  int32_t span_deg = (int32_t)(PK_ARM_DEG_MAX - PK_ARM_DEG_MIN);
  return (uint16_t)((int32_t)PK_ARM_US_MIN + (deg * span_us) / span_deg);
}

static uint16_t plat_us_from_permille(int16_t permille)
{
  int32_t us = (int32_t)PK_PLAT_STOP_US;

  if (permille > PK_PLAT_DEADBAND_PERMILLE || permille < -PK_PLAT_DEADBAND_PERMILLE) {
    us += (int32_t)PK_PLAT_SIGN * (int32_t)permille * (int32_t)PK_PLAT_SPAN_US / 1000;
  }
  if (us < (int32_t)PK_SERVO_MIN_US) us = (int32_t)PK_SERVO_MIN_US;
  if (us > (int32_t)PK_SERVO_MAX_US) us = (int32_t)PK_SERVO_MAX_US;
  return (uint16_t)us;
}

static uint32_t rot_pulses_from_deg(int32_t deg)
{
  int32_t mag = (deg < 0) ? -deg : deg;
  return (uint32_t)(((int32_t)mag * (int32_t)PK_ROT_PULSES_PER_REV + 180) / 360);
}

/* 下发角度，不回显（CYC 用它，自己打步骤行）。返回 0 表示 BSP 拒绝。 */
static uint8_t arm_apply_deg(int32_t deg)
{
  uint16_t pulse_us;

  if (deg < (int32_t)PK_ARM_DEG_MIN || deg > (int32_t)PK_ARM_DEG_MAX) return 0U;
  pulse_us = arm_us_from_deg(deg);
  if (Servo_SetPulseUs(pulse_us) != HAL_OK) return 0U;
  arm_on = 1U;
  arm_pulse_us = pulse_us;
  return 1U;
}

/* 下发载物台速度，不回显。 */
static uint8_t plat_apply(int16_t permille)
{
  uint16_t pulse_us;

  if (permille < -1000) permille = -1000;
  if (permille > 1000) permille = 1000;
  pulse_us = plat_us_from_permille(permille);
  if (Servo2_SetPulseUs(pulse_us) != HAL_OK) return 0U;
  plat_on = 1U;
  plat_permille = permille;
  return 1U;
}

/* 开一次单轴点动。返回 0 = 失败（已回显原因）。
 * 行程足够长才加三段斜坡；太短的移动直接单速，避免"起转还没完就已经该停了"。 */
static uint8_t axis_move_start(uint8_t axis, int32_t pulses, uint32_t hz)
{
  int8_t dir;
  uint32_t mag, ramp_hz, ramp_pulses;

  if (pulses == 0 || hz == 0U) { ChassisIo_Reply("ERR pulses=0\r\n"); return 0U; }
  if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return 0U; }

  dir = (pulses > 0) ? 1 : -1;
  mag = (uint32_t)((pulses > 0) ? pulses : -pulses);

  ramp_hz = (hz * PK_AXIS_RAMP_PERMILLE) / 1000U;
  if (ramp_hz < 1U) ramp_hz = 1U;
  ramp_pulses = (ramp_hz * PK_AXIS_RAMP_MS) / 1000U;
  if (ramp_hz >= hz || mag <= (ramp_pulses * 2U)) ramp_hz = 0U;

  if (Step6_SetHz((Step6Axis)axis,
                  (int32_t)((ramp_hz != 0U) ? ramp_hz : hz) * (int32_t)dir) != HAL_OK) {
    ChassisIo_Reply("ERR set speed failed\r\n");
    return 0U;
  }

  axis_move.active = 1U;
  axis_move.axis = axis;
  axis_move.dir = dir;
  axis_move.phase = (ramp_hz != 0U) ? AXIS_PH_RAMP_UP : AXIS_PH_CRUISE;
  axis_move.target_pulses = mag;
  axis_move.hz = hz;
  axis_move.ramp_hz = ramp_hz;
  axis_move.base_pulses = 0U;
  axis_move.base_ms = Board_NowMs();
  axis_move.start_ms = axis_move.base_ms;
  return 1U;
}

static const char *axis_name(uint8_t axis)
{
  return (axis == (uint8_t)STEP6_ROT) ? "ROT" : "LIFT";
}

static void axis_move_poll(void)
{
  char line[64];
  uint32_t now_ms, elapsed_ms, done, hz_now, seg_ms, decel_pulses;

  if (!axis_move.active) return;

  now_ms = Board_NowMs();

  /* 起转段走完 → 切恒速 */
  if (axis_move.phase == AXIS_PH_RAMP_UP &&
      (uint32_t)(now_ms - axis_move.base_ms) >= PK_AXIS_RAMP_MS) {
    seg_ms = now_ms - axis_move.base_ms;
    if (seg_ms > PK_AXIS_RAMP_MS) seg_ms = PK_AXIS_RAMP_MS;
    done = axis_move.base_pulses +
           (uint32_t)(((uint64_t)axis_move.ramp_hz * (uint64_t)seg_ms) / 1000U);
    if (Step6_SetHz((Step6Axis)axis_move.axis,
                    (int32_t)axis_move.hz * (int32_t)axis_move.dir) != HAL_OK) {
      axis_move.active = 0U;
      ChassisIo_Reply("ERR ramp up failed\r\n");
      return;
    }
    axis_move.base_pulses = done;
    axis_move.base_ms = now_ms;
    axis_move.phase = AXIS_PH_CRUISE;
  }

  hz_now = (axis_move.phase == AXIS_PH_CRUISE) ? axis_move.hz : axis_move.ramp_hz;
  done = axis_move.base_pulses +
         (uint32_t)(((uint64_t)hz_now * (uint64_t)(now_ms - axis_move.base_ms)) / 1000U);

  /* 离目标不到一个收转段的脉冲数 → 切收转 */
  if (axis_move.phase == AXIS_PH_CRUISE && axis_move.ramp_hz != 0U &&
      done < axis_move.target_pulses) {
    decel_pulses = (axis_move.ramp_hz * PK_AXIS_RAMP_MS) / 1000U;
    if ((axis_move.target_pulses - done) <= decel_pulses) {
      if (Step6_SetHz((Step6Axis)axis_move.axis,
                      (int32_t)axis_move.ramp_hz * (int32_t)axis_move.dir) != HAL_OK) {
        axis_move.active = 0U;
        ChassisIo_Reply("ERR ramp down failed\r\n");
        return;
      }
      axis_move.base_pulses = done;
      axis_move.base_ms = now_ms;
      axis_move.phase = AXIS_PH_RAMP_DOWN;
    }
  }

  elapsed_ms = now_ms - axis_move.start_ms;

  if (done < axis_move.target_pulses) {
    if (elapsed_ms > PK_AXIS_TIMEOUT_MS) {
      Step6_Stop((Step6Axis)axis_move.axis);
      axis_move.active = 0U;
      (void)snprintf(line, sizeof(line), "%s timeout at %lu/%lu pulses\r\n",
                     axis_name(axis_move.axis),
                     (unsigned long)done, (unsigned long)axis_move.target_pulses);
      ChassisIo_Reply(line);
    }
    return;
  }

  Step6_Stop((Step6Axis)axis_move.axis);
  axis_move.active = 0U;
  if (!cyc_quiet) {
    (void)snprintf(line, sizeof(line), "%s done, %lu pulses, %lu ms\r\n",
                   axis_name(axis_move.axis),
                   (unsigned long)axis_move.target_pulses, (unsigned long)elapsed_ms);
    ChassisIo_Reply(line);
  }
}

/* PLT 定时点动：到点自动回停止脉宽。 */
static void plat_burst_poll(void)
{
  if (!plat_burst_active) return;
  if ((int32_t)(Board_NowMs() - plat_burst_stop_ms) < 0) return;

  plat_burst_active = 0U;
  (void)plat_apply(0);
  if (!cyc_quiet) ChassisIo_Reply("PLT done, stopped\r\n");
}

/* ---------------- 命令层 API ---------------- */

uint8_t Pick_RotDeg(int32_t deg, uint32_t hz)
{
  char line[64];
  uint32_t pulses;

  if (deg == 0 || deg < -(int32_t)PK_ROT_MAX_DEG || deg > (int32_t)PK_ROT_MAX_DEG) {
    (void)snprintf(line, sizeof(line), "ERR ROT %ld out of -%ld..%ld\r\n",
                   (long)deg, (long)PK_ROT_MAX_DEG, (long)PK_ROT_MAX_DEG);
    ChassisIo_Reply(line);
    return 0U;
  }

  pulses = rot_pulses_from_deg(deg);
  if (!axis_move_start((uint8_t)STEP6_ROT, (int32_t)((deg > 0) ? pulses : -(int32_t)pulses),
                       hz)) {
    return 0U;
  }
  rot_pos_mdeg += deg * 1000;
  (void)snprintf(line, sizeof(line), "OK ROT %ld deg (%lu pulses @ %lu Hz)\r\n",
                 (long)deg, (unsigned long)pulses, (unsigned long)hz);
  if (!cyc_quiet) ChassisIo_Reply(line);
  return 1U;
}

/* 升降点动：走够 N 个脉冲就停（点动距离，不是速度）。hz 只是"走这段用多快"。
 * 没有限位，所以脉冲数超过行程会顶死——上板先小步数试方向。 */
uint8_t Pick_LiftPulses(int32_t pulses, uint32_t hz)
{
  char line[64];

  if (pulses == 0) {
    Step6_Stop(STEP6_LIFT);
    axis_move.active = 0U;
    ChassisIo_Reply("OK LIFT stopped\r\n");
    return 1U;
  }
  if (pulses < -(int32_t)PK_LIFT_MAX_PULSES || pulses > (int32_t)PK_LIFT_MAX_PULSES) {
    (void)snprintf(line, sizeof(line), "ERR LIFT -%ld..%ld pulses\r\n",
                   (long)PK_LIFT_MAX_PULSES, (long)PK_LIFT_MAX_PULSES);
    ChassisIo_Reply(line);
    return 0U;
  }
  if (hz < 100U || hz > STEP6_HZ_HARD_LIMIT) {
    ChassisIo_Reply("ERR LIFT hz 100..40000\r\n");
    return 0U;
  }

  if (!axis_move_start((uint8_t)STEP6_LIFT, pulses, hz)) return 0U;

  /* 累计升降位置：正 = 下降。所有走 LIFT 的路径（手动命令、CYC、TRG）都从这里过，
   * 所以这一个数就是"离起点差多少脉冲"。`LIFT HOME` 用它的相反数走回起点。 */
  lift_pos_pulses += pulses;

  (void)snprintf(line, sizeof(line), "OK LIFT %ld pulses @ %lu Hz\r\n",
                 (long)pulses, (unsigned long)hz);
  if (!cyc_quiet) ChassisIo_Reply(line);
  return 1U;
}

/* ---------------- 升降回位 ---------------- */

int32_t Pick_LiftPosPulses(void) { return lift_pos_pulses; }

/* 把当前位置定义成原点（0）。升降手动开到最高处后发一次，就重新对齐了。 */
void Pick_LiftZeroPos(void)
{
  lift_pos_pulses = 0;
  home_pending = 0U;
  ChassisIo_Reply("OK LIFT zero, pos=0\r\n");
}

/* 回到起点：把累计值取反走回去。正累计（在起点下方）就上升，负累计就下降。
 * 已经在原点时只回一行提示，不发脉冲。
 * 返回 0 = 没走（已在起点 / 未使能 / 超范围），1 = 已经开始走。 */
uint8_t Pick_LiftGoHome(uint32_t hz)
{
  char line[64];
  int32_t back;

  home_pending = 0U;

  if (lift_pos_pulses == 0) {
    ChassisIo_Reply("OK LIFT already home (pos=0)\r\n");
    return 0U;
  }
  if (!Step6_IsEnabled()) {
    ChassisIo_Reply("ERR not enabled (send E 1)\r\n");
    return 0U;
  }
  if (axis_move.active) {
    ChassisIo_Reply("ERR LIFT busy, send STOP\r\n");
    return 0U;
  }

  back = -lift_pos_pulses;
  if (back < -(int32_t)PK_LIFT_MAX_PULSES || back > (int32_t)PK_LIFT_MAX_PULSES) {
    (void)snprintf(line, sizeof(line), "ERR LIFT home %ld pulses, over the %ld limit\r\n",
                   (long)back, (long)PK_LIFT_MAX_PULSES);
    ChassisIo_Reply(line);
    return 0U;
  }

  (void)snprintf(line, sizeof(line), "OK LIFT home, %ld pulses\r\n", (long)back);
  ChassisIo_Reply(line);
  /* 注意：这里不要走 Pick_LiftPulses 的回显分支，它会把上面的行再打一遍；
   * 直接启动轴，再手工把累计值归零。 */
  if (!axis_move_start((uint8_t)STEP6_LIFT, back, hz)) return 0U;
  lift_pos_pulses = 0;
  return 1U;
}

uint8_t Pick_ArmDeg(int32_t deg)
{
  char line[64];
  uint16_t pulse_us;

  if (deg < (int32_t)PK_ARM_DEG_MIN || deg > (int32_t)PK_ARM_DEG_MAX) {
    (void)snprintf(line, sizeof(line), "ERR ARM %ld..%ld deg\r\n",
                   (long)PK_ARM_DEG_MIN, (long)PK_ARM_DEG_MAX);
    ChassisIo_Reply(line);
    return 0U;
  }
  if (!arm_apply_deg(deg)) {
    ChassisIo_Reply("ERR ARM pulse\r\n");
    return 0U;
  }
  pulse_us = arm_pulse_us;
  (void)snprintf(line, sizeof(line), "OK ARM %ld deg (%u us)\r\n",
                 (long)deg, (unsigned)pulse_us);
  if (!cyc_quiet) ChassisIo_Reply(line);
  return 1U;
}

void Pick_ArmOff(void)
{
  Servo_Disable();
  arm_on = 0U;
  ChassisIo_Reply("OK ARM off\r\n");
}

/* 载物台定时点动：连续旋转舵机没有位置反馈，"走一段"只能按时间。
 * 速度由调用方给（命令层固定 PK_PLAT_JOG_PERMILLE）；ms 到了自动回停止脉宽并回显 `PLT done`。 */
uint8_t Pick_PlatMove(int16_t permille, uint32_t ms)
{
  char line[64];

  if (permille < -1000 || permille > 1000) {
    ChassisIo_Reply("ERR PLT -1000..1000\r\n");
    return 0U;
  }
  if (ms == 0 || ms > PK_PLAT_MAX_MS) {
    (void)snprintf(line, sizeof(line), "ERR PLT ms 1..%lu\r\n", (unsigned long)PK_PLAT_MAX_MS);
    ChassisIo_Reply(line);
    return 0U;
  }
  if (!plat_apply(permille)) {
    ChassisIo_Reply("ERR PLT pulse\r\n");
    return 0U;
  }
  plat_burst_active = 1U;
  plat_burst_stop_ms = Board_NowMs() + ms;
  (void)snprintf(line, sizeof(line), "OK PLT %d/1000 for %lu ms\r\n",
                 (int)permille, (unsigned long)ms);
  if (!cyc_quiet) ChassisIo_Reply(line);
  return 1U;
}

void Pick_PlatOff(void)
{
  Servo2_Disable();
  plat_on = 0U;
  plat_permille = 0;
  ChassisIo_Reply("OK PLT off\r\n");
}

void Pick_StopAll(void)
{
  Step6_Stop(STEP6_ROT);
  Step6_Stop(STEP6_LIFT);
  axis_move.active = 0U;
  plat_burst_active = 0U;
  cycle_active = 0U;
  cycle_last_ok = 0U;
  trg_active = 0U;
  grab_active = 0U;
  drop_active = 0U;
  cyc_quiet = 0U;
  home_pending = 1U;   /* STOP / E 0 之后也回最高处（E 0 时服务会因未使能跳过）*/
  if (plat_on) (void)plat_apply(0);
}

/* ---------------- 任务循环 CYC（三段式）----------------
 * 结构：**开场一次 → 每轮重复 → 收尾一次**。
 *   prologue：先张开 → 快降 → 慢降贴近取料位
 *   body    ：夹紧 → 慢抬离 → 快升到放料高度 → 转 -132° → 松开 → 载物盘走一格 →
 *             转 +132° → 快降 → 慢降回取料位
 *   epilogue：从取料位升回最高处
 * 下降末段和夹紧后的第一段都是慢速，避免 12000 Hz 撞料或把料拽翻。
 * 每轮结束时高度和朝向都与开始时一致，连跑多轮不会漂。 */

typedef enum {
  CYC_KIND_LIFT = 0,
  CYC_KIND_ARM,
  CYC_KIND_ROT,
  CYC_KIND_PLT,
  CYC_KIND_ALIGN   /* 视觉对准：value = cls，hz = tol_mm */
} CycKind;

typedef struct {
  uint8_t  kind;
  int32_t  value;          /* LIFT: 脉冲（正 = 下降）；ARM: 度；ROT: 度；PLT: ms */
  uint32_t hz;             /* LIFT / ROT: 频率；ARM: 到位等待 ms（0 = 默认）；PLT 不用 */
} CycStep;

#define CYC_PHASE_PROLOGUE  0U
#define CYC_PHASE_BODY      1U
#define CYC_PHASE_EPILOGUE  2U

/* ARM 步的 hz 字段是到位等待（毫秒），0 = 用 PK_CYC_ARM_SETTLE_MS。
 * LIFT / ROT 的 hz 仍是频率。不要混用。 */
static const CycStep cycle_prologue[] = {
  /* 先张开，闭合的夹爪下降会把物料撞飞 */
  { CYC_KIND_ARM,  PK_GRIP_APPROACH_DEG, PK_GRIP_OPEN_SETTLE_MS },
  /* 快降，留出末段 */
  { CYC_KIND_LIFT, (int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_GRIP_APPROACH_PULSES), PK_CYC_LIFT_HZ },
  /* 慢降贴近 */
  { CYC_KIND_LIFT, (int32_t)PK_GRIP_APPROACH_PULSES, PK_GRIP_APPROACH_HZ }
};

static const CycStep cycle_body[] = {
  /* 1 夹紧，等待比松开长 */ { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS },
  /* 2 先慢抬离，避免把料从台面拽翻 */
  { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES, PK_GRIP_LIFTOFF_HZ },
  /* 3 再快升到放料高度（扣掉已经抬起的） */
  { CYC_KIND_LIFT, -(int32_t)(PK_CYC_LIFT_UP_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_CYC_LIFT_HZ },
  /* 4 转到放料侧 */     { CYC_KIND_ROT,  -(int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ },
  /* 5 松开 */           { CYC_KIND_ARM,   PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS },
  /* 6 载物盘走一格 */   { CYC_KIND_PLT,   (int32_t)PK_CYC_PLAT_MS, 0U },
  /* 7 转回取料侧 */     { CYC_KIND_ROT,   (int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ },
  /* 8 快降，留出末段 */ { CYC_KIND_LIFT,  (int32_t)(PK_CYC_LIFT_UP_PULSES - PK_GRIP_APPROACH_PULSES), PK_CYC_LIFT_HZ },
  /* 9 慢降贴近下一件 */ { CYC_KIND_LIFT,  (int32_t)PK_GRIP_APPROACH_PULSES, PK_GRIP_APPROACH_HZ }
};

static const CycStep cycle_epilogue[] = {
  /* 1 取料位 → 最高处 */ { CYC_KIND_LIFT, -(int32_t)PK_CYC_LIFT_DOWN_PULSES, PK_CYC_LIFT_HZ }
};

#define CYC_PROLOGUE_COUNT  ((uint8_t)(sizeof(cycle_prologue) / sizeof(cycle_prologue[0])))
#define CYC_BODY_COUNT      ((uint8_t)(sizeof(cycle_body) / sizeof(cycle_body[0])))
#define CYC_EPILOGUE_COUNT  ((uint8_t)(sizeof(cycle_epilogue) / sizeof(cycle_epilogue[0])))

/* 取某个阶段的表；返回 0 表示阶段非法。 */
static const CycStep *cycle_phase_table(uint8_t phase, uint8_t *count)
{
  switch (phase) {
    case CYC_PHASE_PROLOGUE: *count = CYC_PROLOGUE_COUNT; return cycle_prologue;
    case CYC_PHASE_BODY:     *count = CYC_BODY_COUNT;     return cycle_body;
    case CYC_PHASE_EPILOGUE: *count = CYC_EPILOGUE_COUNT; return cycle_epilogue;
    default:                 *count = 0U;                 return 0;
  }
}

static void cycle_stop(void)
{
  cycle_active = 0U;
  cycle_phase = CYC_PHASE_PROLOGUE;
  cycle_step = 0U;
  cycle_round = 0U;
  cycle_last_ok = 0U;      /* 正常跑完的分支会在调用后再把它置 1 */
  cyc_quiet = 0U;          /* 单步回显恢复，结束行由调用方打 */
  home_pending = 1U;       /* 收尾/中止后自动回最高处（已在 0 时不动作）*/
}

/* 下发一步。ARM 的 hz 是到位等待毫秒（0 = 默认）；其它种类的 hz 是频率或不用。
 * 返回 0 = BSP 拒绝，调用方负责中止序列。 */
static uint8_t step_apply(uint8_t kind, int32_t value, uint32_t hz,
                          uint32_t *wait_ms, uint32_t now_ms, uint8_t stage)
{
  switch (kind) {
    case CYC_KIND_LIFT:
      return Pick_LiftPulses(value, hz);
    case CYC_KIND_ARM:
      if (!Pick_ArmDeg(value)) return 0U;
      *wait_ms = now_ms + ((hz != 0U) ? hz : (uint32_t)PK_CYC_ARM_SETTLE_MS);
      return 1U;
    case CYC_KIND_ROT:
      return Pick_RotDeg(value, hz);
    case CYC_KIND_PLT:
      return Pick_PlatMove((int16_t)PK_PLAT_JOG_PERMILLE, (uint32_t)value);
    case CYC_KIND_ALIGN:
      /* DRY：不接视觉，直接跳过对准步骤。 */
      if (drop_dry != 0U) return 1U;
      /* 只启动一次；进度由 step_busy 的 Align_IsActive() 等出来。
       * 用 ALIGN 一次性模式（track=0）：到容差内就 done，不能像 TRACK 那样
       * 锁定后一直跟住，否则 DROP 会等到 25 s 超时。
       * stage 0 = 顶部粗对，1 = 底部精对；视觉据此切 px/mm。 */
      return Align_StartExAuxTask((uint8_t)value, (int16_t)hz, 0U, drop_ring, stage);
    default:
      return 0U;
  }
}

static uint8_t step_busy(uint8_t kind, uint32_t wait_ms, uint32_t now_ms)
{
  switch (kind) {
    case CYC_KIND_LIFT:
    case CYC_KIND_ROT:
      return axis_move.active;
    case CYC_KIND_ARM:
      return (uint8_t)((int32_t)(now_ms - wait_ms) < 0);
    case CYC_KIND_PLT:
      return plat_burst_active;
    case CYC_KIND_ALIGN:
      return Align_IsActive();
    default:
      return 0U;
  }
}

/* 进入"某个阶段的第 index 步"并把它发下去。 */
static void cycle_enter(uint8_t phase, uint8_t index, uint32_t now_ms)
{
  uint8_t count = 0U;
  const CycStep *table = cycle_phase_table(phase, &count);
  const CycStep *step;
  uint8_t ok;

  if (table == 0 || index >= count) {
    cycle_stop();
    ChassisIo_Reply("CYC aborted: bad step\r\n");
    return;
  }

  step = &table[index];
  cycle_phase = phase;
  cycle_step = index;
  cycle_step_ms = now_ms;
  ok = step_apply(step->kind, step->value, step->hz, &cycle_wait_ms, now_ms, 0U);

  if (!ok) {
    cycle_stop();
    ChassisIo_Reply("CYC aborted\r\n");
  }
}

static void cycle_poll(uint32_t now_ms)
{
  uint8_t count = 0U;
  const CycStep *table = cycle_phase_table(cycle_phase, &count);
  const CycStep *step;
  char line[64];

  if (!cycle_active || table == 0) return;

  step = &table[cycle_step];

  if ((int32_t)(now_ms - cycle_step_ms) > (int32_t)PK_CYC_STEP_TIMEOUT_MS) {
    cycle_stop();
    Pick_StopAll();
    ChassisIo_Reply("CYC aborted: step timeout\r\n");
    return;
  }

  if (step_busy(step->kind, cycle_wait_ms, now_ms)) return;

  /* 这个阶段还有下一步 */
  if ((uint8_t)(cycle_step + 1U) < count) {
    cycle_enter(cycle_phase, (uint8_t)(cycle_step + 1U), now_ms);
    return;
  }

  /* 本阶段走完：开场 → 第一轮；每轮 → 下一轮或收尾；收尾 → 结束 */
  switch (cycle_phase) {
    case CYC_PHASE_PROLOGUE:
      cycle_enter(CYC_PHASE_BODY, 0U, now_ms);
      return;

    case CYC_PHASE_BODY:
      (void)plat_apply(0);
      cycle_round++;
      if (cycle_round < cycle_rounds) {
        cycle_enter(CYC_PHASE_BODY, 0U, now_ms);
      } else {
        cycle_enter(CYC_PHASE_EPILOGUE, 0U, now_ms);
      }
      return;

    default:   /* CYC_PHASE_EPILOGUE：整个 CYC 结束 */
      cycle_stop();
      cycle_last_ok = 1U;    /* 完整跑满 N 轮并回到最高处，TASK 可以继续走位 */
      (void)snprintf(line, sizeof(line), "CYC done (%u rounds)\r\n", (unsigned)cycle_rounds);
      ChassisIo_Reply(line);
      return;
  }
}

void Pick_CycleStart(uint8_t rounds)
{
  char line[64];

  if (cycle_active) {
    ChassisIo_Reply("ERR CYC busy, send STOP\r\n");
    return;
  }
  if (trg_active) {
    ChassisIo_Reply("ERR TRG running, send STOP\r\n");
    return;
  }
  if (grab_active) {
    ChassisIo_Reply("ERR GRAB running, send STOP\r\n");
    return;
  }
  if (drop_active) {
    ChassisIo_Reply("ERR DROP running, send STOP\r\n");
    return;
  }
  if (!Step6_IsEnabled()) {
    ChassisIo_Reply("ERR not enabled (send E 1)\r\n");
    return;
  }
  if (rounds == 0U || rounds > PK_CYC_MAX_ROUNDS) {
    ChassisIo_Reply("ERR CYC rounds\r\n");
    return;
  }

  Step6_Stop(STEP6_ROT);
  Step6_Stop(STEP6_LIFT);
  axis_move.active = 0U;
  plat_burst_active = 0U;
  (void)plat_apply(0);

  home_pending = 0U;
  cycle_active = 1U;
  cycle_phase = CYC_PHASE_PROLOGUE;
  cycle_step = 0U;
  cycle_round = 0U;
  cycle_rounds = rounds;
  cycle_last_ok = 0U;      /* 跑完才会置 1 */

  (void)snprintf(line, sizeof(line), "CYC start, %u rounds (STOP aborts)\r\n",
                 (unsigned)rounds);
  ChassisIo_Reply(line);
  cyc_quiet = 1U;          /* 从这里开始关掉单步回显 */

  cycle_enter(CYC_PHASE_PROLOGUE, 0U, Board_NowMs());
}

uint8_t Pick_CycleIsActive(void) { return cycle_active; }
uint8_t Pick_CycleRound(void) { return cycle_round; }
uint8_t Pick_CycleRounds(void) { return cycle_rounds; }
uint8_t Pick_CycleLastOk(void) { return cycle_last_ok; }

/* ---------------- 卸料序列 TRG ----------------
 * 和 CYC 同一套骨架（表驱动 + 非阻塞等待），动作按现场流程重写：
 *   降 4000 → 夹紧 → 升 4000（回最高处）→ 转 +132° → 降 41000 → 松开 → 转 -132° → 升 41000 回最高处。
 * 起点**和终点**都在丝杆最高处，所以能连着跑多轮、也能接着发 CYC。
 * 高度都是实测：顶部→地面 41000（2026-11-17），顶部→载物台取料位 4000，
 * 顶部→物料台取料位 8000，顶部→旋转台放料位 2000（= 8000 - 6000）。 */

typedef struct {
  uint8_t  kind;    /* 复用 CYC_KIND_* */
  int32_t  value;
  uint32_t hz;
  uint8_t  stage;   /* 仅 ALIGN 用：0 = 顶部粗对，1 = 底部精对 */
} TrgStep;

static const TrgStep trg_table[] = {
  /* 1 先张开再下降 */     { CYC_KIND_ARM,   PK_GRIP_APPROACH_DEG,              PK_GRIP_OPEN_SETTLE_MS , 0U },
  /* 2 快降到取料位附近 */ { CYC_KIND_LIFT,  (int32_t)(PK_TRG_PICK_PULSES - PK_GRIP_APPROACH_PULSES), PK_TRG_LIFT_HZ , 0U },
  /* 3 慢降贴近 */         { CYC_KIND_LIFT,  (int32_t)PK_GRIP_APPROACH_PULSES,  PK_GRIP_APPROACH_HZ , 0U },
  /* 4 夹紧 */             { CYC_KIND_ARM,   PK_GRIP_CLOSE_DEG,                 PK_GRIP_CLOSE_SETTLE_MS , 0U },
  /* 5 慢抬离 */           { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES,   PK_GRIP_LIFTOFF_HZ , 0U },
  /* 6 快升回最高处 */     { CYC_KIND_LIFT, -(int32_t)(PK_TRG_PICK_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_TRG_LIFT_HZ , 0U },
  /* 7 转到放料侧 */       { CYC_KIND_ROT,   (int32_t)PK_TRG_ROT_DEG,           PK_CYC_ROT_HZ , 0U },
  /* 8 快降，留出末段 */   { CYC_KIND_LIFT,  (int32_t)(PK_TRG_DROP_PULSES - PK_GRIP_APPROACH_PULSES), PK_TRG_LIFT_HZ , 0U },
  /* 9 慢降放到地面 */     { CYC_KIND_LIFT,  (int32_t)PK_GRIP_APPROACH_PULSES,  PK_GRIP_APPROACH_HZ , 0U },
  /* 10 松开 */            { CYC_KIND_ARM,   PK_GRIP_OPEN_DEG,                  PK_GRIP_OPEN_SETTLE_MS , 0U },
  /* 11 先抬离再转，避免扫到刚放下的料 */
  { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES, PK_GRIP_LIFTOFF_HZ, 0U },
  /* 12 转回取料侧 */      { CYC_KIND_ROT,  -(int32_t)PK_TRG_ROT_DEG,           PK_CYC_ROT_HZ , 0U },
  /* 13 升回最高处 */      { CYC_KIND_LIFT, -(int32_t)(PK_TRG_DROP_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_TRG_LIFT_HZ , 0U }
};

#define TRG_STEP_COUNT  ((uint8_t)(sizeof(trg_table) / sizeof(trg_table[0])))

static void trg_stop(void)
{
  trg_active = 0U;
  trg_step = 0U;
  trg_round = 0U;
  cyc_quiet = 0U;
  home_pending = 1U;
}

static void trg_enter(uint8_t index, uint32_t now_ms)
{
  const TrgStep *step = &trg_table[index];
  uint8_t ok;

  trg_step = index;
  trg_step_ms = now_ms;

  ok = step_apply(step->kind, step->value, step->hz, &trg_wait_ms, now_ms, step->stage);

  if (!ok) {
    trg_stop();
    ChassisIo_Reply("TRG aborted\r\n");
  }
}

static void trg_poll(uint32_t now_ms)
{
  const TrgStep *step;
  char line[64];

  if (!trg_active) return;

  step = &trg_table[trg_step];

  if ((int32_t)(now_ms - trg_step_ms) > (int32_t)PK_CYC_STEP_TIMEOUT_MS) {
    trg_stop();
    Pick_StopAll();
    ChassisIo_Reply("TRG aborted: step timeout\r\n");
    return;
  }

  if (step_busy(step->kind, trg_wait_ms, now_ms)) return;

  if ((uint8_t)(trg_step + 1U) < TRG_STEP_COUNT) {
    trg_enter((uint8_t)(trg_step + 1U), now_ms);
    return;
  }

  trg_round++;
  if (trg_round >= trg_rounds) {
    trg_stop();
    cyc_quiet = 0U;
    (void)snprintf(line, sizeof(line), "TRG done (%u rounds)\r\n", (unsigned)trg_rounds);
    ChassisIo_Reply(line);
    return;
  }
  trg_enter(0U, now_ms);
}

void Pick_TrgStart(uint8_t rounds)
{
  char line[64];

  if (trg_active) {
    ChassisIo_Reply("ERR TRG busy, send STOP\r\n");
    return;
  }
  if (cycle_active) {
    ChassisIo_Reply("ERR CYC running, send STOP\r\n");
    return;
  }
  if (grab_active) {
    ChassisIo_Reply("ERR GRAB running, send STOP\r\n");
    return;
  }
  if (drop_active) {
    ChassisIo_Reply("ERR DROP running, send STOP\r\n");
    return;
  }
  if (!Step6_IsEnabled()) {
    ChassisIo_Reply("ERR not enabled (send E 1)\r\n");
    return;
  }
  if (rounds == 0U || rounds > PK_TRG_MAX_ROUNDS) {
    ChassisIo_Reply("ERR TRG rounds\r\n");
    return;
  }

  Step6_Stop(STEP6_ROT);
  Step6_Stop(STEP6_LIFT);
  axis_move.active = 0U;
  plat_burst_active = 0U;

  home_pending = 0U;
  trg_active = 1U;
  trg_step = 0U;
  trg_round = 0U;
  trg_rounds = rounds;

  (void)snprintf(line, sizeof(line), "TRG start, %u rounds (STOP aborts)\r\n",
                 (unsigned)rounds);
  ChassisIo_Reply(line);
  cyc_quiet = 1U;          /* 和 CYC 一样，跑的时候不刷单步回显 */
  trg_enter(0U, Board_NowMs());
}

uint8_t Pick_TrgIsActive(void) { return trg_active; }

/* ---------------- 单次抓取 GRAB / 放置 PLACE ----------------
 * 给视觉对准用：车已经把物料对到夹爪正下方，这里负责下降夹起来。
 *
 * 两种结束方式：
 *   GRAB ... HOLD  只夹起来、升回最高处，夹着料停住（调试用）
 *   GRAB           夹起来 → 升到放料高度 → 转到放料侧 → 松开 → 载物台走一格 →
 *                  转回取料侧 → 升回最高处（和 CYC 每轮的后半段完全一致）
 *   PLACE          手里已经夹着料（在最高处）时，只做上面的后半段
 *
 * 脉冲净值：+(DOWN-APP)+APP -LIFTOFF -(UP-LIFTOFF) -(DOWN-UP) = 0，
 * 结束时累计位置回到起点。前提是发命令时人在最高处；lift_pos != 0 直接拒绝，
 * 避免从半截高度再降 DOWN。 */
static const TrgStep grab_table[] = {
  { CYC_KIND_ARM,  PK_GRIP_APPROACH_DEG, PK_GRIP_OPEN_SETTLE_MS , 0U},
  { CYC_KIND_LIFT, (int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_GRIP_APPROACH_PULSES), PK_CYC_LIFT_HZ , 0U},
  { CYC_KIND_LIFT, (int32_t)PK_GRIP_APPROACH_PULSES, PK_GRIP_APPROACH_HZ , 0U},
  { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS , 0U},
  { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES, PK_GRIP_LIFTOFF_HZ, 0U },
  { CYC_KIND_LIFT, -(int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_CYC_LIFT_HZ , 0U}
};

/* 抓 + 放：夹起来后只升到放料高度（比最高处低 DOWN-UP），转过去放完再升回最高处。 */
static const TrgStep grab_place_table[] = {
  { CYC_KIND_ARM,  PK_GRIP_APPROACH_DEG, PK_GRIP_OPEN_SETTLE_MS , 0U},
  { CYC_KIND_LIFT, (int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_GRIP_APPROACH_PULSES), PK_CYC_LIFT_HZ , 0U},
  { CYC_KIND_LIFT, (int32_t)PK_GRIP_APPROACH_PULSES, PK_GRIP_APPROACH_HZ , 0U},
  { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS , 0U},
  { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES, PK_GRIP_LIFTOFF_HZ, 0U },
  { CYC_KIND_LIFT, -(int32_t)(PK_CYC_LIFT_UP_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_CYC_LIFT_HZ , 0U},
  { CYC_KIND_ROT,  -(int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ , 0U},
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS , 0U},
  { CYC_KIND_PLT,  (int32_t)PK_CYC_PLAT_MS, 0U , 0U},
  { CYC_KIND_ROT,  (int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ , 0U},
  { CYC_KIND_LIFT, -(int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_CYC_LIFT_UP_PULSES), PK_CYC_LIFT_HZ , 0U}
};

/* 只放置：从最高处带着料下去放。 */
static const TrgStep place_table[] = {
  { CYC_KIND_LIFT, (int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_CYC_LIFT_UP_PULSES), PK_CYC_LIFT_HZ , 0U},
  { CYC_KIND_ROT,  -(int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ , 0U},
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS , 0U},
  { CYC_KIND_PLT,  (int32_t)PK_CYC_PLAT_MS, 0U , 0U},
  { CYC_KIND_ROT,  (int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ , 0U},
  { CYC_KIND_LIFT, -(int32_t)(PK_CYC_LIFT_DOWN_PULSES - PK_CYC_LIFT_UP_PULSES), PK_CYC_LIFT_HZ , 0U}
};

/* 地面抓取：夹料高度从 PK_CYC_LIFT_DOWN_PULSES(8000) 换成 PK_DROP_RELEASE_PULSES(41000)。 */
static const TrgStep ground_grab_table[] = {
  { CYC_KIND_ARM,  PK_GRIP_APPROACH_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, (int32_t)(PK_DROP_RELEASE_PULSES - PK_GRIP_APPROACH_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_LIFT, (int32_t)PK_GRIP_APPROACH_PULSES, PK_GRIP_APPROACH_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES, PK_GRIP_LIFTOFF_HZ, 0U },
  { CYC_KIND_LIFT, -(int32_t)(PK_DROP_RELEASE_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_TRG_LIFT_HZ, 0U }
};
static const TrgStep ground_grab_place_table[] = {
  { CYC_KIND_ARM,  PK_GRIP_APPROACH_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, (int32_t)(PK_DROP_RELEASE_PULSES - PK_GRIP_APPROACH_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_LIFT, (int32_t)PK_GRIP_APPROACH_PULSES, PK_GRIP_APPROACH_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, -(int32_t)PK_GRIP_LIFTOFF_PULSES, PK_GRIP_LIFTOFF_HZ, 0U },
  { CYC_KIND_LIFT, -(int32_t)(PK_DROP_RELEASE_PULSES - PK_CYC_PLACE_DOWN_PULSES - PK_GRIP_LIFTOFF_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ROT,  -(int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_PLT,  (int32_t)PK_CYC_PLAT_MS, 0U, 0U },
  { CYC_KIND_ROT,  (int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ, 0U },
  { CYC_KIND_LIFT, -(int32_t)PK_CYC_PLACE_DOWN_PULSES, PK_TRG_LIFT_HZ, 0U }
};

#define GRAB_STEP_COUNT        ((uint8_t)(sizeof(grab_table) / sizeof(grab_table[0])))
#define GRAB_PLACE_STEP_COUNT  ((uint8_t)(sizeof(grab_place_table) / sizeof(grab_place_table[0])))
#define GROUND_GRAB_STEP_COUNT ((uint8_t)(sizeof(ground_grab_table) / sizeof(ground_grab_table[0])))
#define GROUND_GRAB_PLACE_STEP_COUNT ((uint8_t)(sizeof(ground_grab_place_table) / sizeof(ground_grab_place_table[0])))
#define PLACE_STEP_COUNT       ((uint8_t)(sizeof(place_table) / sizeof(place_table[0])))

/* 当前在跑哪张表：0 = 单抓，1 = 抓+放，2 = 只放（只影响回显）。 */
static const TrgStep *grab_seq;
static uint8_t grab_seq_count;
static uint8_t grab_mode;

static void grab_stop(void)
{
  grab_active = 0U;
  grab_step = 0U;
  grab_mode = 0U;
  cyc_quiet = 0U;
  home_pending = 1U;
}

static void grab_enter(uint8_t index, uint32_t now_ms)
{
  const TrgStep *step = &grab_seq[index];
  uint8_t ok;

  grab_step = index;
  grab_step_ms = now_ms;
  ok = step_apply(step->kind, step->value, step->hz, &grab_wait_ms, now_ms, step->stage);
  if (!ok) {
    grab_stop();
    ChassisIo_Reply("GRAB aborted\r\n");
  }
}

static void grab_poll(uint32_t now_ms)
{
  const TrgStep *step;

  if (!grab_active) return;

  step = &grab_seq[grab_step];
  if ((int32_t)(now_ms - grab_step_ms) > (int32_t)PK_CYC_STEP_TIMEOUT_MS) {
    grab_stop();
    Pick_StopAll();
    ChassisIo_Reply("GRAB aborted: step timeout\r\n");
    return;
  }
  if (step_busy(step->kind, grab_wait_ms, now_ms)) return;

  if ((uint8_t)(grab_step + 1U) < grab_seq_count) {
    grab_enter((uint8_t)(grab_step + 1U), now_ms);
    return;
  }
  grab_stop();
  if (grab_mode == 1U) {
    ChassisIo_Reply("GRAB done, placed, held at top\r\n");
  } else if (grab_mode == 2U) {
    ChassisIo_Reply("PLACE done, held at top\r\n");
  } else {
    ChassisIo_Reply("GRAB done, held at top\r\n");
  }
}

/* 两个序列共同的启动前检查。返回 1 = 可以跑。 */
static uint8_t grab_prepare(const char *name)
{
  char line[64];

  if (grab_active) {
    (void)snprintf(line, sizeof(line), "ERR %s busy, send STOP\r\n", name);
    ChassisIo_Reply(line);
    return 0U;
  }
  if (cycle_active) {
    ChassisIo_Reply("ERR CYC running, send STOP\r\n");
    return 0U;
  }
  if (trg_active) {
    ChassisIo_Reply("ERR TRG running, send STOP\r\n");
    return 0U;
  }
  if (drop_active) {
    ChassisIo_Reply("ERR DROP running, send STOP\r\n");
    return 0U;
  }
  if (!Step6_IsEnabled()) {
    ChassisIo_Reply("ERR not enabled (send E 1)\r\n");
    return 0U;
  }
  if (axis_move.active) {
    ChassisIo_Reply("ERR axis busy, send STOP\r\n");
    return 0U;
  }
  /* 累计位置不是 0 就说明不在我们记的最高处。从那儿再降 DOWN 会顶死或够不着。 */
  if (lift_pos_pulses != 0) {
    (void)snprintf(line, sizeof(line), "ERR %s lift not home, send LIFT HOME\r\n", name);
    ChassisIo_Reply(line);
    return 0U;
  }

  Step6_Stop(STEP6_ROT);
  Step6_Stop(STEP6_LIFT);
  axis_move.active = 0U;
  plat_burst_active = 0U;
  (void)plat_apply(0);
  home_pending = 0U;
  return 1U;
}

void Pick_GrabStart(uint8_t place)
{
  if (grab_prepare("GRAB") == 0U) return;

  grab_seq = (place != 0U) ? grab_place_table : grab_table;
  grab_seq_count = (place != 0U) ? GRAB_PLACE_STEP_COUNT : GRAB_STEP_COUNT;
  grab_mode = (place != 0U) ? 1U : 0U;
  grab_active = 1U;
  grab_step = 0U;
  ChassisIo_Reply((place != 0U)
                    ? "GRAB start, pick then place (STOP aborts)\r\n"
                    : "GRAB start, open-down-close-up (STOP aborts)\r\n");
  cyc_quiet = 1U;
  grab_enter(0U, Board_NowMs());
}

void Pick_GroundGrabStart(uint8_t place)
{
  if (grab_prepare("GRAB_GND") == 0U) return;

  grab_seq = (place != 0U) ? ground_grab_place_table : ground_grab_table;
  grab_seq_count = (place != 0U) ? GROUND_GRAB_PLACE_STEP_COUNT : GROUND_GRAB_STEP_COUNT;
  grab_mode = 3U;
  grab_active = 1U;
  grab_step = 0U;
  ChassisIo_Reply("GRAB_GND start, ground pick then place (STOP aborts)\r\n");
  cyc_quiet = 1U;
  grab_enter(0U, Board_NowMs());
}

void Pick_PlaceStart(void)
{
  if (grab_prepare("PLACE") == 0U) return;

  grab_seq = place_table;
  grab_seq_count = PLACE_STEP_COUNT;
  grab_mode = 2U;
  grab_active = 1U;
  grab_step = 0U;
  ChassisIo_Reply("PLACE start, turn-open-step-back (STOP aborts)\r\n");
  cyc_quiet = 1U;
  grab_enter(0U, Board_NowMs());
}

uint8_t Pick_GrabIsActive(void) { return grab_active; }

/* ---------------- 圆环放置 DROP ----------------
 * 用户 2026-11-17 实测顺序（连续小脉冲合成大值）：
 *  1 张开 30°
 *  2 转 +132° 到放料侧
 *  3 顶部粗对准（stage 0）
 *  4 降 41000 到地面
 *  5 底部精对准（stage 1）
 *  6 升 37000 到取料位 4000
 *  7 转 -132° 回取料侧
 *  8 再降 2000 到 6000（夹料位）
 *  9 夹紧 180°
 * 10 升 4000 到 2000
 * 11 转 +132° 到放料侧
 * 12 降 38000 到 40000（离地 1 mm）
 * 13 张开 30° 放料
 * 14 升 40000 回最高处
 * 15 转 -132° 回取料侧
 *
 * 精对之后底盘不再动；10/12/15 的 ROT 回程精度决定落点。 */
static const TrgStep drop_table[] = {
  /* 从放料侧 0° 开始：先空爪对准圆环 → 回最高 → 转 -132° 到载物盘取料
   * → 转回 0° 放地面。stp4 每个环重复一次，结束后停在 0°。 */
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_ALIGN, 11, PK_DROP_ALIGN_COARSE_MM, 0U },
  { CYC_KIND_LIFT, (int32_t)PK_DROP_RELEASE_PULSES, PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ALIGN, 11, PK_DROP_ALIGN_TOL_MM, 1U },
  { CYC_KIND_LIFT, -(int32_t)PK_DROP_RELEASE_PULSES, PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ROT,  -(int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ, 0U },
  { CYC_KIND_LIFT, (int32_t)(PK_TRG_PICK_PULSES + PK_CYC_PLACE_DOWN_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, -(int32_t)PK_TRG_PICK_PULSES, PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ROT,  (int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ, 0U },
  { CYC_KIND_LIFT, (int32_t)(PK_DROP_RELEASE_PULSES - 1000U - PK_CYC_PLACE_DOWN_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, -(int32_t)(PK_DROP_RELEASE_PULSES - 1000U), PK_TRG_LIFT_HZ, 0U }
};
#define DROP_STEP_COUNT ((uint8_t)(sizeof(drop_table) / sizeof(drop_table[0])))

/* DRY：不接视觉，直接从载物台取料→转到放料侧→降 40000→松开→回最高，
 * 不做空爪粗对/精对，也不先降到地面。 */
static const TrgStep dry_drop_table[] = {
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_ROT,  -(int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ, 0U },
  { CYC_KIND_LIFT, (int32_t)(PK_TRG_PICK_PULSES + PK_CYC_PLACE_DOWN_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_CLOSE_DEG, PK_GRIP_CLOSE_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, -(int32_t)PK_TRG_PICK_PULSES, PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ROT,  (int32_t)PK_CYC_ROT_DEG, PK_CYC_ROT_HZ, 0U },
  { CYC_KIND_LIFT, (int32_t)(PK_DROP_RELEASE_PULSES - 1000U - PK_CYC_PLACE_DOWN_PULSES), PK_TRG_LIFT_HZ, 0U },
  { CYC_KIND_ARM,  PK_GRIP_OPEN_DEG, PK_GRIP_OPEN_SETTLE_MS, 0U },
  { CYC_KIND_LIFT, -(int32_t)(PK_DROP_RELEASE_PULSES - 1000U), PK_TRG_LIFT_HZ, 0U }
};
#define DRY_DROP_STEP_COUNT ((uint8_t)(sizeof(dry_drop_table) / sizeof(dry_drop_table[0])))

static const TrgStep *drop_seq = drop_table;
static uint8_t drop_seq_count = DROP_STEP_COUNT;

static void drop_stop(void)
{
  drop_active = 0U;
  drop_dry = 0U;
  drop_step = 0U;
  cyc_quiet = 0U;
  home_pending = 1U;
}

static void drop_enter(uint8_t index, uint32_t now_ms)
{
  const TrgStep *step = &drop_seq[index];
  uint8_t ok;

  drop_step = index;
  drop_step_ms = now_ms;
  ok = step_apply(step->kind, step->value, step->hz, &drop_wait_ms, now_ms, step->stage);
  if (!ok) {
    drop_stop();
    ChassisIo_Reply("DROP aborted\r\n");
  }
}

static void drop_poll(uint32_t now_ms)
{
  const TrgStep *step;

  if (!drop_active) return;

  step = &drop_seq[drop_step];

  if (step->kind == CYC_KIND_ALIGN) {
    if (Align_IsActive()) {
      if ((int32_t)(now_ms - drop_step_ms) > (int32_t)PK_DROP_ALIGN_TIMEOUT_MS) {
        Align_Cancel();
        drop_stop();
        ChassisIo_Reply("DROP aborted: align timeout\r\n");
      }
      return;
    }
    if (Align_LastReason() != ALIGN_REASON_DONE) {
      if (Align_LastReason() == ALIGN_REASON_NO_TARGET &&
          (int32_t)(drop_deadline_ms - now_ms) > 0) {
        drop_enter(drop_step, now_ms);
        return;
      }
      drop_stop();
      ChassisIo_Reply("DROP aborted: align failed\r\n");
      return;
    }
  } else {
    if ((int32_t)(now_ms - drop_step_ms) > (int32_t)PK_CYC_STEP_TIMEOUT_MS) {
      drop_stop();
      Pick_StopAll();
      ChassisIo_Reply("DROP aborted: step timeout\r\n");
      return;
    }
    if (step_busy(step->kind, drop_wait_ms, now_ms)) return;
  }

  if ((uint8_t)(drop_step + 1U) < drop_seq_count) {
    drop_enter((uint8_t)(drop_step + 1U), now_ms);
    return;
  }
  drop_ok = 1U;
  drop_stop();
  ChassisIo_Reply("DROP done, ring placed\r\n");
}

void Pick_DropStart(uint8_t ring)
{
  char drop_line[80];

  if (grab_prepare("DROP") == 0U) return;

  drop_ok = 0U;
  drop_deadline_ms = Board_NowMs() + 20000U;
  drop_ring = (ring == 0U) ? 1U : ring;
  drop_seq = drop_table;
  drop_seq_count = DROP_STEP_COUNT;
  drop_dry = 0U;
  drop_active = 1U;
  drop_step = 0U;
  home_pending = 0U;
  (void)snprintf(drop_line, sizeof(drop_line),
                 "DROP start ring %u: align empty, then pick+place (STOP aborts)\r\n",
                 (unsigned)drop_ring);
  ChassisIo_Reply(drop_line);
  cyc_quiet = 1U;
  drop_enter(0U, Board_NowMs());
}

void Pick_DropStartDry(uint8_t ring)
{
  char dry_line[64];

  if (grab_prepare("DROP DRY") == 0U) return;
  drop_ok = 0U;
  drop_deadline_ms = Board_NowMs() + 20000U;
  drop_ring = (ring == 0U) ? 1U : ring;
  drop_seq = dry_drop_table;
  drop_seq_count = DRY_DROP_STEP_COUNT;
  drop_dry = 1U;
  drop_active = 1U;
  drop_step = 0U;
  home_pending = 0U;
  (void)snprintf(dry_line, sizeof(dry_line),
                 "DROP DRY ring %u: no vision align (STOP aborts)\r\n",
                 (unsigned)drop_ring);
  ChassisIo_Reply(dry_line);
  cyc_quiet = 1U;
  drop_enter(0U, Board_NowMs());
}

uint8_t Pick_DropIsActive(void) { return drop_active; }

uint8_t Pick_DropLastOk(void) { return drop_ok; }

/* 用户按 STOP / 手动命令抢占时调用：停轴，再把升降从低位升回最高处。 */
void Pick_DropAbort(void)
{
  if (!drop_active) return;
  drop_stop();
  Pick_StopAll();
}

void Pick_Poll(void)
{
  axis_move_poll();
  plat_burst_poll();
  cycle_poll(Board_NowMs());
  trg_poll(Board_NowMs());
  grab_poll(Board_NowMs());
  drop_poll(Board_NowMs());
  if (home_pending) {
    home_pending = 0U;
    if (Step6_IsEnabled() != 0U && lift_pos_pulses != 0 && !axis_move.active) {
      (void)Pick_LiftGoHome(PK_LIFT_HZ);
    }
  }
}

/* ---------------- 状态查询 ---------------- */

uint8_t Pick_ArmIsOn(void) { return arm_on; }
uint16_t Pick_ArmPulseUs(void) { return arm_pulse_us; }
int16_t Pick_PlatGetPermille(void) { return plat_permille; }
uint8_t Pick_RotIsMoving(void) { return axis_move.active; }
int16_t Pick_RotGetDeg(void) { return (int16_t)(rot_pos_mdeg / 1000); }
