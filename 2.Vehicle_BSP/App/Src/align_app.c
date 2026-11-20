#include "align_app.h"
#include "align_config.h"
#include "board.h"
#include "board_imu.h"
#include "board_step6.h"
#include "board_vision.h"
#include "chassis_io.h"
#include "chassis_motion.h"
#include "chassis_task.h"
#include "pick_app.h"
#include <stdio.h>

/* 连续小幅度调整：每 AL_SERVO_PERIOD_MS 读一次 $V 快照，按误差给四个轮子一个速度。
 * 已经在容差里的轴不再出力。同一帧最多再推 AL_FRAME_HOLD_MS，然后停车等新帧，
 * 避免视觉慢一拍时车还按旧误差往前冲。
 *
 * 两种模式共用这套节拍：
 *   ALIGN  进容差并停稳 AL_SETTLE_HITS 帧就报 done 并结束。
 *   TRACK  目标动就一直跟：用相邻两帧的位置差估计目标速度，按 AL_TRACK_FF_MILLI
 *          前馈到轮速上（纯比例跟随在匀速目标下会固定差"速度×延迟"那么多），
 *          目标停稳且进容差才报一次 `TRACK locked`，之后目标再动继续跟。
 *
 * 轮速合成（符号照抄 chassis_task.c 的 TASK 定义）：
 *   前 (+1,+1,+1,+1)   左 (-1,+1,+1,-1)
 * 叠加：A = vx - vy，B = vx + vy，C = vx + vy，D = vx - vy
 * 航向修正 corr > 0 = 逆时针：A/C 减、B/D 加（与 chassis_motion 的差速相同）。 */

typedef enum {
  ALIGN_STATE_IDLE = 0,
  ALIGN_STATE_RUN
} AlignState;

static struct {
  uint8_t    active;
  uint8_t    cls;
  uint8_t    track;              /* 1 = TRACK 连续追踪，0 = ALIGN 一次性对准 */
  uint8_t    allow_in_task;      /* 1 = 由 TASK 内部启动，允许任务在跑时对准 */
  uint8_t    locked;             /* 1 = 追踪模式下目标停稳且已在容差内 */
  uint8_t    tick;
  uint8_t    settle_hits;
  uint8_t    cmd_valid;          /* 已经按某一 seq 下过速度 */
  uint8_t    yaw_hold;           /* 1 = 启动时锁过 yaw 目标 */
  uint8_t    vel_valid;          /* 1 = 已有上一帧样本，可以算目标速度 */
  AlignState state;
  uint32_t   start_ms;
  uint32_t   last_ms;            /* 上一次控制拍的时刻 */
  uint32_t   lost_ms;            /* 0 = 当前有目标 */
  uint32_t   cmd_seq;            /* 当前速度是按哪一帧算的 */
  uint32_t   cmd_deadline_ms;    /* 这一帧最多推到这个时刻 */
  uint32_t   settle_seq;         /* 最近一次计入 settle 的 seq */
  uint32_t   vel_seq;            /* 上一个速度样本的 seq */
  uint32_t   vel_stamp_ms;       /* 上一个速度样本的接受时刻 */
  int16_t    tol_mm;             /* 本次对准/追踪的容差 */
  int16_t    last_x_mm;
  int16_t    last_y_mm;
  int16_t    last_vx_mm_per_s;
  int16_t    last_vy_mm_per_s;
  int16_t    vel_x_mm;           /* 上一个速度样本的误差，用于差分 */
  int16_t    vel_y_mm;
  int32_t    vff_x_mm_per_s;     /* 目标速度估计（前馈用），mm/s */
  int32_t    vff_y_mm_per_s;
  int32_t    yaw_target_mdeg;
  int32_t    rate_f_mdps;
  AlignReason reason;
} align;

/* 回显缓冲放文件内静态区，避免 Align_Poll 的调用链把 1 KB 栈吃紧。 */
static char align_line[80];

#define ALIGN_STATUS_EVERY  5U   /* 每 5 拍（0.5 s）打一行状态，避免刷屏 */

static int16_t abs_i16(int16_t value)
{
  return (value < 0) ? (int16_t)(-value) : value;
}

static int32_t i32_abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t wrap_mdeg(int32_t mdeg)
{
  if (mdeg >= 360000 || mdeg <= -360000) mdeg %= 360000;
  if (mdeg > 180000) mdeg -= 360000;
  if (mdeg < -180000) mdeg += 360000;
  return mdeg;
}

static int32_t clamp_i32(int32_t value, int32_t limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

/* 用相邻两帧的目标位置差估计目标速度（mm/s，正方向同车体 x / y）。
 *
 * 只在 seq 变了、且两帧的接受时刻差在 AL_TRACK_VEL_DT_MIN..MAX_MS 之间才更新：
 * 同 seq 是重发帧（差分为 0，会把估计拉低）；间隔太短是抖动、太长说明中途断过，
 * 这两种都不可信，直接把估计清零。一阶低通取 1/4，避免单帧噪声变成轮子抖。 */
static void update_target_velocity(const VisionTarget *target, int16_t dx_mm, int16_t dy_mm)
{
  int32_t dt_ms;
  int32_t vx_mm_per_s;
  int32_t vy_mm_per_s;

  if (align.vel_valid != 0U && target->seq != align.vel_seq) {
    dt_ms = (int32_t)(target->stamp_ms - align.vel_stamp_ms);
    if (dt_ms < (int32_t)AL_TRACK_VEL_DT_MIN_MS ||
        dt_ms > (int32_t)AL_TRACK_VEL_DT_MAX_MS) {
      align.vff_x_mm_per_s = 0;
      align.vff_y_mm_per_s = 0;
    } else {
      vx_mm_per_s = ((int32_t)dx_mm - (int32_t)align.vel_x_mm) * 1000 / dt_ms;
      vy_mm_per_s = ((int32_t)dy_mm - (int32_t)align.vel_y_mm) * 1000 / dt_ms;
      if (i32_abs(vx_mm_per_s) > (int32_t)AL_TRACK_FF_REJECT_MM_PER_S ||
          i32_abs(vy_mm_per_s) > (int32_t)AL_TRACK_FF_REJECT_MM_PER_S) {
        /* 阶跃不是速度：手碰一下、遮挡后重新捕获都会这样。前馈清零，
         * 只让比例项把误差拉回来，不要把车甩出去一大步。 */
        align.vff_x_mm_per_s = 0;
        align.vff_y_mm_per_s = 0;
      } else {
        vx_mm_per_s = clamp_i32(vx_mm_per_s, AL_TRACK_FF_MAX_MM_PER_S);
        vy_mm_per_s = clamp_i32(vy_mm_per_s, AL_TRACK_FF_MAX_MM_PER_S);
        align.vff_x_mm_per_s += (vx_mm_per_s - align.vff_x_mm_per_s) / 4;
        align.vff_y_mm_per_s += (vy_mm_per_s - align.vff_y_mm_per_s) / 4;
      }
    }
  }

  align.vel_valid = 1U;
  align.vel_seq = target->seq;
  align.vel_stamp_ms = target->stamp_ms;
  align.vel_x_mm = dx_mm;
  align.vel_y_mm = dy_mm;
}

/* 目标是否"停住了"：两轴速度估计都在锁定门限内。追踪模式用它区分
 * "跟着一个移动目标"和"已经对准、可以夹"——只要目标还在动就不报 locked。 */
static uint8_t target_is_still(void)
{
  if (i32_abs(align.vff_x_mm_per_s) > (int32_t)AL_TRACK_FF_STILL_MM_PER_S) return 0U;
  if (i32_abs(align.vff_y_mm_per_s) > (int32_t)AL_TRACK_FF_STILL_MM_PER_S) return 0U;
  return 1U;
}

/* 把目标速度按 AL_TRACK_FF_MILLI 叠加到这一轴的比例速度上。
 * 死区内原样返回（目标基本没动，别把视觉抖动放大）；追移动目标时把
 * hold_ms 拉满，否则靠近零点只点动一下根本跟不上。 */
static int16_t feedforward_speed(int16_t base_mm_per_s, int32_t vff_mm_per_s,
                                 uint32_t *hold_ms)
{
  int32_t out;

  if (vff_mm_per_s <= (int32_t)AL_TRACK_FF_MIN_MM_PER_S &&
      vff_mm_per_s >= -(int32_t)AL_TRACK_FF_MIN_MM_PER_S) {
    return base_mm_per_s;
  }
  out = (int32_t)base_mm_per_s + (vff_mm_per_s * AL_TRACK_FF_MILLI) / 1000;
  if (i32_abs(vff_mm_per_s) >= (int32_t)AL_TRACK_FF_HOLD_MM_PER_S) {
    *hold_ms = AL_FRAME_HOLD_MS;
  }
  return (int16_t)clamp_i32(out, AL_SERVO_MAX_MM_PER_S);
}

/* 只停底盘四个轮，不碰旋转台/升降台。频率没变就不重发，避免把 PWM 打断。 */
static void wheels_apply(int32_t hz_a, int32_t hz_b, int32_t hz_c, int32_t hz_d)
{
  static const Step6Axis ids[4] = { STEP6_A, STEP6_B, STEP6_C, STEP6_D };
  int32_t cmd[4];
  uint8_t i;

  cmd[0] = hz_a;
  cmd[1] = hz_b;
  cmd[2] = hz_c;
  cmd[3] = hz_d;

  for (i = 0U; i < 4U; i++) {
    int32_t now_hz = 0;
    uint32_t mag = Step6_GetHz(ids[i]);

    if (mag != 0U) now_hz = (int32_t)Step6_GetDir(ids[i]) * (int32_t)mag;
    if (cmd[i] == now_hz) continue;
    if (cmd[i] != 0 && now_hz != 0 &&
        i32_abs(cmd[i] - now_hz) < (int32_t)AL_HZ_UPDATE_MIN) {
      continue;
    }
    (void)Step6_SetHz(ids[i], cmd[i]);
  }
}

static void stop_chassis(void)
{
  wheels_apply(0, 0, 0, 0);
  align.last_vx_mm_per_s = 0;
  align.last_vy_mm_per_s = 0;
}

/* mm/s → STEP 频率（带符号）。用千倍整数换算，不引入浮点。 */
static int32_t mm_per_s_to_hz(int32_t mm_per_s)
{
  return (mm_per_s * AL_HZ_PER_MM_PER_S_MILLI) / 1000;
}

/* 误差 → 期望速度。hold_ms 写出这一拍最多允许保持多久：
 * 正常帧用 AL_FRAME_HOLD_MS；靠近零点时改成短点动，避免最低速冲过容差。 */
static int16_t error_to_speed(int16_t error_mm, int16_t tol_mm, uint32_t *hold_ms)
{
  int32_t mag;
  int32_t speed;
  int32_t gain;
  int32_t horizon_mm;
  int32_t min_horizon_mm;

  *hold_ms = AL_FRAME_HOLD_MS;
  if (error_mm == 0) return 0;
  mag = (error_mm < 0) ? -(int32_t)error_mm : (int32_t)error_mm;
  if (mag <= (int32_t)tol_mm) return 0;

  gain = (mag <= (int32_t)AL_SERVO_NEAR_MM) ? (int32_t)AL_SERVO_GAIN_NEAR_MILLI
                                             : (int32_t)AL_SERVO_GAIN_MILLI;
  speed = (mag * gain) / 1000;
  if (speed > AL_SERVO_MAX_MM_PER_S) speed = AL_SERVO_MAX_MM_PER_S;
  if (speed < 1) speed = 1;

  /* 按"这一帧还会再保持 hold_ms"来限速，不要在下一帧到来前冲过零点。
   * 向上取整：整除会把 1.8 mm 看成 1 mm，安全判断偏乐观。 */
  horizon_mm = (speed * (int32_t)AL_FRAME_HOLD_MS + 999) / 1000;
  if (horizon_mm >= mag) {
    speed = ((mag - 1) * 1000) / (int32_t)AL_FRAME_HOLD_MS;
    if (speed < 1) speed = 1;
  }

  min_horizon_mm = ((int32_t)AL_SERVO_MIN_MM_PER_S * (int32_t)AL_FRAME_HOLD_MS + 999) / 1000;
  if (speed < (int32_t)AL_SERVO_MIN_MM_PER_S) {
    if (min_horizon_mm < mag) {
      speed = AL_SERVO_MIN_MM_PER_S;
    } else {
      /* 最低速保持一整帧会冲过零点：只点一下。 */
      speed = AL_SERVO_MIN_MM_PER_S;
      *hold_ms = AL_NUDGE_MS;
    }
  }
  return (error_mm < 0) ? (int16_t)(-speed) : (int16_t)speed;
}

/* 斜向时单轮线速度是 |vx|+|vy|，不限的话斜着走比直着走快一截。 */
static void limit_translation(int16_t *vx_mm_per_s, int16_t *vy_mm_per_s)
{
  int32_t ax = abs_i16(*vx_mm_per_s);
  int32_t ay = abs_i16(*vy_mm_per_s);
  int32_t peak = ax + ay;

  if (peak > (int32_t)AL_SERVO_MAX_MM_PER_S && peak > 0) {
    *vx_mm_per_s = (int16_t)(((int32_t)*vx_mm_per_s * AL_SERVO_MAX_MM_PER_S) / peak);
    *vy_mm_per_s = (int16_t)(((int32_t)*vy_mm_per_s * AL_SERVO_MAX_MM_PER_S) / peak);
  }
}

/* 平移过程中把 yaw 拉回启动时的值。没在平移、或没有新鲜帧时返回 0，
 * 避免停车确认期间差速把车重新推出容差。 */
static int32_t yaw_corr_hz(int16_t vx_mm_per_s, int16_t vy_mm_per_s)
{
  int32_t err_mdeg;
  int32_t rate_mdps;
  int32_t corr;

  if (AL_YAW_HOLD_ENABLE == 0 || align.yaw_hold == 0U) return 0;
  if (vx_mm_per_s == 0 && vy_mm_per_s == 0) return 0;
  if (Imu_IsFresh() == 0U) return 0;

  err_mdeg = wrap_mdeg(align.yaw_target_mdeg - Imu_ReadYawMdeg());
  rate_mdps = Imu_ReadRateMdps();
  align.rate_f_mdps += (rate_mdps - align.rate_f_mdps) / 4;

  corr = 0;
  if (err_mdeg > AL_YAW_DEADBAND_MDEG || err_mdeg < -AL_YAW_DEADBAND_MDEG) {
    corr = (err_mdeg * AL_YAW_KP_HZ_PER_DEG) / 1000;
  }
  corr -= (align.rate_f_mdps / 1000) * AL_YAW_KD_HZ_PER_DPS;
  if (corr > AL_YAW_MAX_HZ) corr = AL_YAW_MAX_HZ;
  if (corr < -AL_YAW_MAX_HZ) corr = -AL_YAW_MAX_HZ;
  return corr;
}

static void notify_detect(uint8_t enable)
{
  static const char on[] = "@DETECT 1\n";
  static const char off[] = "@DETECT 0\n";
  const char *text = (enable != 0U) ? on : off;
  (void)Serial_Write(SERIAL_RPI, (const uint8_t *)text, 10U, 20U);
}

static void finish(AlignReason reason, const char *text, uint8_t announce)
{
  notify_detect(0U);
  stop_chassis();
  align.active = 0U;
  align.state = ALIGN_STATE_IDLE;
  align.locked = 0U;
  align.vel_valid = 0U;
  align.vff_x_mm_per_s = 0;
  align.vff_y_mm_per_s = 0;
  align.reason = reason;
  if (announce) ChassisIo_Reply(text);
}

static uint8_t target_usable(const VisionTarget *target, uint32_t now_ms)
{
  if (Vision_TargetFresh(now_ms, AL_STALE_MS) == 0U) return 0U;
  if (target->valid == 0U) return 0U;
  if (target->class_code != (uint16_t)align.cls) return 0U;
  if (target->confidence_permille < AL_CONF_MIN_PERMILLE) return 0U;
  if (target->age_ms > AL_VISION_AGE_MS) return 0U;
  return 1U;
}

void Align_Init(void)
{
  align.active = 0U;
  align.cls = 0U;
  align.track = 0U;
  align.locked = 0U;
  align.tol_mm = AL_TOL_DEFAULT_MM;
  align.tick = 0U;
  align.settle_hits = 0U;
  align.cmd_valid = 0U;
  align.yaw_hold = 0U;
  align.state = ALIGN_STATE_IDLE;
  align.start_ms = 0U;
  align.last_ms = 0U;
  align.lost_ms = 0U;
  align.cmd_seq = 0U;
  align.cmd_deadline_ms = 0U;
  align.settle_seq = 0xFFFFFFFFU;
  align.vel_valid = 0U;
  align.vel_seq = 0U;
  align.vel_stamp_ms = 0U;
  align.last_x_mm = 0;
  align.last_y_mm = 0;
  align.last_vx_mm_per_s = 0;
  align.last_vy_mm_per_s = 0;
  align.vel_x_mm = 0;
  align.vel_y_mm = 0;
  align.vff_x_mm_per_s = 0;
  align.vff_y_mm_per_s = 0;
  align.yaw_target_mdeg = 0;
  align.rate_f_mdps = 0;
  align.reason = ALIGN_REASON_NONE;
}

void Align_Cancel(void)
{
  if (align.active == 0U) return;
  finish(ALIGN_REASON_CANCELLED, "ALIGN aborted: cancelled\r\n", 0U);
}

uint8_t Align_IsActive(void)
{
  return align.active;
}

uint8_t Align_IsTracking(void)
{
  return align.track;
}

uint8_t Align_TryCompleteForGrab(void)
{
  if (align.active == 0U || align.track == 0U || align.locked == 0U) return 0U;
  /* 锁定只是"连续几帧稳定"，夹取还要求这一拍真的在容差内。 */
  if (abs_i16(align.last_x_mm) > align.tol_mm) return 0U;
  if (abs_i16(align.last_y_mm) > align.tol_mm) return 0U;

  (void)snprintf(align_line, sizeof(align_line),
                 "ALIGN done, e=(%d,%d) mm\r\n",
                 (int)align.last_x_mm, (int)align.last_y_mm);
  finish(ALIGN_REASON_DONE, align_line, 1U);
  return 1U;
}

AlignReason Align_LastReason(void)
{
  return align.reason;
}

void Align_PrintStatus(void)
{
  static const char *const names[] = {
    "none", "done", "timeout", "no target", "iterations",
    "move start", "disabled", "cancelled"
  };
  const char *reason = "unknown";

  if ((unsigned)align.reason <= (unsigned)ALIGN_REASON_CANCELLED) {
    reason = names[(unsigned)align.reason];
  }

  (void)snprintf(align_line, sizeof(align_line),
                 "ALIGN active=%u mode=%s cls=%u tol=%d mm hit=%u locked=%u\r\n",
                 (unsigned)align.active,
                 (align.track != 0U) ? "track" : "align",
                 (unsigned)align.cls, (int)align.tol_mm,
                 (unsigned)align.settle_hits, (unsigned)align.locked);
  ChassisIo_Reply(align_line);

  (void)snprintf(align_line, sizeof(align_line),
                 "      e=(%d,%d) mm  v=(%d,%d) mm/s  ff=(%ld,%ld) mm/s  yaw=%s\r\n",
                 (int)align.last_x_mm, (int)align.last_y_mm,
                 (int)align.last_vx_mm_per_s, (int)align.last_vy_mm_per_s,
                 (long)align.vff_x_mm_per_s, (long)align.vff_y_mm_per_s,
                 (align.yaw_hold != 0U) ? "hold" : "off");
  ChassisIo_Reply(align_line);

  (void)snprintf(align_line, sizeof(align_line), "      reason=%s\r\n", reason);
  ChassisIo_Reply(align_line);
}

/* 把"这次要跟哪个类别"告诉树莓派：`$C,<cls>\n`。
 *
 * 树莓派侧 align_send.py / vision_node.py 收到后切换目标色，不用重启脚本；
 * 蓝牙（UART5）和 USART1 发的 GRAB/TRACK/ALIGN 都会走到 Align_StartEx()，所以两条口一样生效。
 *
 * 阻塞写，但只在启动对准时发不到 12 字节（115200 下约 1 ms），而且此刻车是停的、
 * 没有脉冲在进行，不会拖长 STEP 间隔。 */
static void send_class_hint_aux(uint8_t cls, uint8_t aux, int16_t rot_deg, uint8_t stage)
{
  char line[32];
  int length;
  if (cls == 11U) {
    /* 环模式带当前转台角 + 对准阶段：0 = 顶部粗对，1 = 底部精对。 */
    length = snprintf(line, sizeof(line), "$C,%u,%u,%d,%u\n",
                      (unsigned)cls, (unsigned)aux, (int)rot_deg, (unsigned)stage);
  } else if (aux > 0U) {
    length = snprintf(line, sizeof(line), "$C,%u,%u\n", (unsigned)cls, (unsigned)aux);
  } else {
    length = snprintf(line, sizeof(line), "$C,%u\n", (unsigned)cls);
  }

  if (length > 0 && length < (int)sizeof(line)) {
    (void)Serial_Write(SERIAL_RPI, (const uint8_t *)line, (uint16_t)length, 20U);
  }
}

static void send_class_hint(uint8_t cls)
{
  send_class_hint_aux(cls, 0U, (cls == 11U) ? Pick_RotGetDeg() : 0, 0U);
}

void Align_SendClassHint(uint8_t cls)
{
  send_class_hint(cls);
}

uint8_t Align_Start(uint8_t cls, int16_t tol_mm)
{
  return Align_StartEx(cls, tol_mm, 0U);
}

uint8_t Align_StartEx(uint8_t cls, int16_t tol_mm, uint8_t track)
{
  return Align_StartExAux(cls, tol_mm, track, 0U);
}

/* TASK 内部用：任务本身在跑，Align_Poll 不能因此判 motion 取消。 */
uint8_t Align_StartExTask(uint8_t cls, int16_t tol_mm, uint8_t track)
{
  uint8_t ok = Align_StartExAux(cls, tol_mm, track, 0U);
  if (ok != 0U) align.allow_in_task = 1U;
  return ok;
}

uint8_t Align_StartExAux(uint8_t cls, int16_t tol_mm, uint8_t track, uint8_t aux)
{
  return Align_StartExAuxStage(cls, tol_mm, track, aux, 0U);
}

/* TASK 内部 DROP 用：和 StartExTask 一样允许任务在跑时对准。 */
uint8_t Align_StartExAuxTask(uint8_t cls, int16_t tol_mm, uint8_t track,
                            uint8_t aux, uint8_t stage)
{
  uint8_t ok = Align_StartExAuxStage(cls, tol_mm, track, aux, stage);
  if (ok != 0U) align.allow_in_task = 1U;
  return ok;
}

uint8_t Align_StartExAuxStage(uint8_t cls, int16_t tol_mm, uint8_t track,
                              uint8_t aux, uint8_t stage)
{
  if (cls != 10U && cls != 11U && (cls < 1U || cls > 6U)) {
    ChassisIo_Reply("ERR ALIGN/TRACK cls 1-6|10|11\r\n");
    return 0U;
  }
  if (tol_mm == 0) tol_mm = AL_TOL_DEFAULT_MM;
  if (tol_mm < AL_TOL_MIN_MM || tol_mm > AL_TOL_MAX_MM) {
    (void)snprintf(align_line, sizeof(align_line), "ERR tol %d..%d mm\r\n",
                   (int)AL_TOL_MIN_MM, (int)AL_TOL_MAX_MM);
    ChassisIo_Reply(align_line);
    return 0U;
  }
  if (Step6_IsEnabled() == 0U) {
    ChassisIo_Reply("ERR not enabled\r\n");
    return 0U;
  }
  if (align.active != 0U) {
    ChassisIo_Reply("ERR ALIGN/TRACK already running\r\n");
    return 0U;
  }
  if (Pick_CycleIsActive() != 0U || Pick_TrgIsActive() != 0U ||
      Pick_GrabIsActive() != 0U) {
    ChassisIo_Reply("ERR sequence running, send STOP\r\n");
    return 0U;
  }

  align.active = 1U;
  align.allow_in_task = 0U;
  align.cls = cls;
  align.track = (track != 0U) ? 1U : 0U;
  align.locked = 0U;
  align.tol_mm = tol_mm;
  align.tick = 0U;
  align.settle_hits = 0U;
  align.cmd_valid = 0U;
  align.state = ALIGN_STATE_RUN;
  align.start_ms = Board_NowMs();
  align.last_ms = 0U;            /* 第一拍立即执行，不等一个周期 */
  align.lost_ms = 0U;
  align.cmd_seq = 0U;
  align.cmd_deadline_ms = 0U;
  align.settle_seq = 0xFFFFFFFFU;
  align.vel_valid = 0U;
  align.vel_seq = 0U;
  align.vel_stamp_ms = 0U;
  align.last_x_mm = 0;
  align.last_y_mm = 0;
  align.last_vx_mm_per_s = 0;
  align.last_vy_mm_per_s = 0;
  align.vel_x_mm = 0;
  align.vel_y_mm = 0;
  align.vff_x_mm_per_s = 0;
  align.vff_y_mm_per_s = 0;
  align.rate_f_mdps = 0;
  align.reason = ALIGN_REASON_NONE;
  align.yaw_hold = 0U;
  align.yaw_target_mdeg = 0;
  if (AL_YAW_HOLD_ENABLE != 0 && Imu_IsFresh() != 0U) {
    align.yaw_target_mdeg = Imu_ReadYawMdeg();
    align.yaw_hold = 1U;
  }

  (void)snprintf(align_line, sizeof(align_line),
                 "OK %s cls=%u tol=%d mm ofs=(%d,%d) yaw=%s\r\n",
                 (align.track != 0U) ? "TRACK" : "ALIGN",
                 (unsigned)cls, (int)tol_mm,
                 (int)AL_GRIPPER_OFFSET_X_MM, (int)AL_GRIPPER_OFFSET_Y_MM,
                 (align.yaw_hold != 0U) ? "hold" : "off");
  ChassisIo_Reply(align_line);
  send_class_hint_aux(cls, aux, (cls == 11U) ? Pick_RotGetDeg() : 0, stage);
  notify_detect(1U);
  return 1U;
}

void Align_Poll(void)
{
  const VisionTarget *target;
  int16_t dx_mm;
  int16_t dy_mm;
  int16_t vx_mm_per_s;
  int16_t vy_mm_per_s;
  int32_t vff_x_mm_per_s;
  int32_t vff_y_mm_per_s;
  int32_t corr_hz;
  int32_t hz_a;
  int32_t hz_b;
  uint32_t now_ms;
  uint32_t hold_x_ms;
  uint32_t hold_y_ms;
  uint32_t hold_ms;
  uint32_t lost_limit_ms;
  uint8_t still;

  if (align.active == 0U) return;

  now_ms = Board_NowMs();

  if (align.track != 0U) {
    if (AL_TRACK_TIMEOUT_MS != 0U &&
        (uint32_t)(now_ms - align.start_ms) >= AL_TRACK_TIMEOUT_MS) {
      finish(ALIGN_REASON_TIMEOUT, "TRACK aborted: timeout\r\n", 1U);
      return;
    }
  } else if ((uint32_t)(now_ms - align.start_ms) >= AL_TIMEOUT_MS) {
    finish(ALIGN_REASON_TIMEOUT, "ALIGN aborted: timeout\r\n", 1U);
    return;
  }
  if (Step6_IsEnabled() == 0U) {
    finish(ALIGN_REASON_DISABLED, "ALIGN aborted: disabled\r\n", 1U);
    return;
  }
  /* 任务或点动起来之后不能再抢四轮；机构在下降时车必须停着。 */
  if ((ChassisTask_IsActive() != 0U && align.allow_in_task == 0U) ||
      ChassisMotion_Busy() != 0U ||
      Pick_CycleIsActive() != 0U || Pick_TrgIsActive() != 0U ||
      Pick_GrabIsActive() != 0U) {
    finish(ALIGN_REASON_CANCELLED, "ALIGN aborted: motion\r\n", 1U);
    return;
  }
  if (align.state != ALIGN_STATE_RUN) {
    align.active = 0U;
    return;
  }

  /* 节拍没到也要看两件事：帧断了要马上停；这一帧允许推的时间到了也要停。
   * 否则要等下一个 100 ms 节拍，盲走会比 AL_FRAME_HOLD_MS 多出一拍。 */
  if ((align.last_ms != 0U) &&
      ((uint32_t)(now_ms - align.last_ms) < AL_SERVO_PERIOD_MS)) {
    target = Vision_ReadTarget();
    if (target_usable(target, now_ms) == 0U) {
      stop_chassis();
    } else if (align.cmd_valid != 0U && target->seq == align.cmd_seq &&
               (int32_t)(now_ms - align.cmd_deadline_ms) >= 0) {
      stop_chassis();
    }
    return;
  }
  align.last_ms = now_ms;

  /* 中途才收到第一帧陀螺，也把当前 yaw 锁成保持目标。不追溯已经走过的角度。 */
  if (align.yaw_hold == 0U && AL_YAW_HOLD_ENABLE != 0 && Imu_IsFresh() != 0U) {
    align.yaw_target_mdeg = Imu_ReadYawMdeg();
    align.yaw_hold = 1U;
  }

  lost_limit_ms = (align.track != 0U) ? AL_TRACK_LOST_MS : AL_LOST_MS;

  target = Vision_ReadTarget();
  if (target_usable(target, now_ms) == 0U) {
    stop_chassis();
    align.cmd_valid = 0U;
    align.settle_hits = 0U;
    align.locked = 0U;
    if (align.lost_ms == 0U) {
      align.lost_ms = now_ms;
    } else if ((uint32_t)(now_ms - align.lost_ms) >= lost_limit_ms) {
      finish(ALIGN_REASON_NO_TARGET,
             (align.track != 0U) ? "TRACK aborted: no target\r\n"
                                 : "ALIGN aborted: no target\r\n", 1U);
    }
    return;
  }

  align.lost_ms = 0U;
  dx_mm = (int16_t)((int16_t)target->x_mm - (int16_t)AL_GRIPPER_OFFSET_X_MM);
  dy_mm = (int16_t)((int16_t)target->y_mm - (int16_t)AL_GRIPPER_OFFSET_Y_MM);
  align.last_x_mm = dx_mm;
  align.last_y_mm = dy_mm;

  /* 追踪模式先更新目标速度估计：容差内靠它决定"要不要跟着走"，锁定时靠它判"停没停"。 */
  if (align.track != 0U) {
    update_target_velocity(target, dx_mm, dy_mm);
  }
  vff_x_mm_per_s = align.vff_x_mm_per_s;
  vff_y_mm_per_s = align.vff_y_mm_per_s;
  still = target_is_still();

  hold_ms = AL_FRAME_HOLD_MS;

  if ((abs_i16(dx_mm) <= align.tol_mm) && (abs_i16(dy_mm) <= align.tol_mm)) {
    vx_mm_per_s = 0;
    vy_mm_per_s = 0;
    /* 边走边量到的那一帧不算：先停稳，后面的新帧才开始计数。
     * 追踪时还要求目标本身停了，不然路过容差带也会被当成锁定。 */
    if (still != 0U && align.last_vx_mm_per_s == 0 && align.last_vy_mm_per_s == 0 &&
        target->seq != align.settle_seq) {
      if (align.settle_hits < AL_SETTLE_HITS) align.settle_hits++;
      align.settle_seq = target->seq;
    }

    if (align.track == 0U) {
      if (align.settle_hits >= AL_SETTLE_HITS) {
        (void)snprintf(align_line, sizeof(align_line),
                       "ALIGN done, e=(%d,%d) mm\r\n",
                       (int)dx_mm, (int)dy_mm);
        finish(ALIGN_REASON_DONE, align_line, 1U);
        return;
      }
    } else if (align.settle_hits >= AL_SETTLE_HITS) {
      if (align.locked == 0U) {
        align.locked = 1U;
        (void)snprintf(align_line, sizeof(align_line),
                       "TRACK locked, e=(%d,%d) mm\r\n",
                       (int)dx_mm, (int)dy_mm);
        ChassisIo_Reply(align_line);
      }
    } else if (still == 0U) {
      /* 目标在容差带里还在走：按目标速度跟，别等误差涨出去再追回来。 */
      vx_mm_per_s = (int16_t)clamp_i32((vff_x_mm_per_s * AL_TRACK_FF_MILLI) / 1000,
                                       AL_SERVO_MAX_MM_PER_S);
      vy_mm_per_s = (int16_t)clamp_i32((vff_y_mm_per_s * AL_TRACK_FF_MILLI) / 1000,
                                       AL_SERVO_MAX_MM_PER_S);
      limit_translation(&vx_mm_per_s, &vy_mm_per_s);
      align.settle_hits = 0U;
      /* 已经锁过、只是目标在容差带里慢慢挪：不反复报 locked/unlocked，
       * 速度明显起来（> 2 倍静止门限）或误差出容差才解锁。 */
      if (align.locked != 0U &&
          (i32_abs(vff_x_mm_per_s) > (2 * (int32_t)AL_TRACK_FF_STILL_MM_PER_S) ||
           i32_abs(vff_y_mm_per_s) > (2 * (int32_t)AL_TRACK_FF_STILL_MM_PER_S))) {
        align.locked = 0U;
        ChassisIo_Reply("TRACK unlocked\r\n");
      }
    }
  } else {
    align.settle_hits = 0U;
    /* 解锁留一点滞回：刚出容差就翻状态会在边界上反复刷 locked/unlocked。 */
    if (align.locked != 0U &&
        (abs_i16(dx_mm) > (int16_t)(align.tol_mm + AL_TRACK_UNLOCK_MARGIN_MM) ||
         abs_i16(dy_mm) > (int16_t)(align.tol_mm + AL_TRACK_UNLOCK_MARGIN_MM))) {
      align.locked = 0U;
      ChassisIo_Reply("TRACK unlocked\r\n");
    }
    vx_mm_per_s = error_to_speed(dx_mm, align.tol_mm, &hold_x_ms);
    vy_mm_per_s = error_to_speed(dy_mm, align.tol_mm, &hold_y_ms);
    hold_ms = (hold_x_ms < hold_y_ms) ? hold_x_ms : hold_y_ms;
    if (align.track != 0U) {
      vx_mm_per_s = feedforward_speed(vx_mm_per_s, vff_x_mm_per_s, &hold_ms);
      vy_mm_per_s = feedforward_speed(vy_mm_per_s, vff_y_mm_per_s, &hold_ms);
    }
    limit_translation(&vx_mm_per_s, &vy_mm_per_s);
  }

  /* 有速度才需要按帧限时；零速度就是停车，不用改 cmd_valid。 */
  if ((vx_mm_per_s != 0) || (vy_mm_per_s != 0)) {
    if (align.cmd_valid != 0U && target->seq == align.cmd_seq &&
        (int32_t)(now_ms - align.cmd_deadline_ms) >= 0) {
      /* 这一帧已经推完允许的时间，停下来等下一帧，不要盲走。 */
      vx_mm_per_s = 0;
      vy_mm_per_s = 0;
    } else if (align.cmd_valid == 0U || target->seq != align.cmd_seq) {
      align.cmd_valid = 1U;
      align.cmd_seq = target->seq;
      align.cmd_deadline_ms = now_ms + hold_ms;
    }
  }

  align.last_vx_mm_per_s = vx_mm_per_s;
  align.last_vy_mm_per_s = vy_mm_per_s;
  corr_hz = yaw_corr_hz(vx_mm_per_s, vy_mm_per_s);

  hz_a = mm_per_s_to_hz((int32_t)vx_mm_per_s - (int32_t)vy_mm_per_s);
  hz_b = mm_per_s_to_hz((int32_t)vx_mm_per_s + (int32_t)vy_mm_per_s);
  wheels_apply(hz_a - corr_hz, hz_b + corr_hz, hz_b - corr_hz, hz_a + corr_hz);

  if ((align.tick % ALIGN_STATUS_EVERY) == 0U) {
    if (align.track != 0U) {
      (void)snprintf(align_line, sizeof(align_line),
                     "TRACK e=(%d,%d) mm v=(%d,%d) mm/s ff=(%ld,%ld) locked=%u\r\n",
                     (int)dx_mm, (int)dy_mm,
                     (int)vx_mm_per_s, (int)vy_mm_per_s,
                     (long)vff_x_mm_per_s, (long)vff_y_mm_per_s,
                     (unsigned)align.locked);
    } else {
      (void)snprintf(align_line, sizeof(align_line),
                     "ALIGN e=(%d,%d) mm  v=(%d,%d) mm/s c=%ld\r\n",
                     (int)dx_mm, (int)dy_mm,
                     (int)vx_mm_per_s, (int)vy_mm_per_s, (long)corr_hz);
    }
    ChassisIo_Reply(align_line);
  }
  align.tick++;
}
