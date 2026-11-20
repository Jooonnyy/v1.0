#include "chassis_task.h"
#include "chassis_config.h"
#include "chassis_io.h"
#include "chassis_motion.h"
#include "pick_app.h"
#include "pick_config.h"
#include "board.h"
#include "board_imu.h"
#include "board_step6.h"
#include <stdio.h>
#include <string.h>
#include "align_app.h"
#include "align_config.h"

/* 航点和到站朝向仍是 WHEEL v0.6 原样，改的只是走路方式：
 * 用当前车头直接前/后/左/右走，转身只留在到站前那一次。
 * 发车朝南 = 0°，90=东 180=北 270=西，正 = 逆时针。
 *   W1 扫码 (-150,-1050) 朝 0    W2 工位 (-1050,-100) 朝 270
 *   W3 工位 (-1050,-1950) 朝 90  W4 工位 (-1950,-1050) 朝 0
 *   W5 = W2  W6 = W3  W7 = W4    收车 (0,0)，**不再做最后的 180° 转身**（2026-11-11 删） */

#define TASK_FWD       0U
#define TASK_BACK      1U
#define TASK_LEFT      2U
#define TASK_RIGHT     3U
#define TASK_TURN      4U
#define TASK_WAIT      5U
#define TASK_RESET     6U
#define TASK_TURN_ABS  7U
#define TASK_CYC       8U   /* 跑 N 轮机构循环（CYC），跑完再走下一步 */
#define TASK_SCAN      9U   /* 往树莓派发 @SCAN，等 value ms */
#define TASK_TRG      10U   /* 载物台取料 → 放地面，N 轮（TRG） */
#define TASK_GRAB     11U   /* 视觉对准颜色 → 取到载物台（台面 8000），value = 0..5 */
#define TASK_GRAB_GND 12U   /* 移到环位 → 视觉对准 → 地面 41000 取到载物台 */
#define TASK_DROP     13U   /* 移到环位 → DROP 放进圆环 */
#define TASK_CENTER   14U   /* 回到 2 号环前 */
#define TASK_PLT      15U   /* 载物盘走一格，value = ms */
#define TASK_ROT_HOME 16U   /* 把转台转回取料侧 0° */
#define TASK_ROT_CAROUSEL 17U /* 把转台转到载物盘侧 -PK_CYC_ROT_DEG */
#define TASK_STP4      18U  /* stp4 封装步：3 个环依次 移环→DROP→PLT */

typedef struct {
  uint8_t kind;
  int16_t value;          /* FWD/BACK/LEFT/RIGHT: mm；TURN/TURN_ABS: deg；WAIT: ms；CYC: 轮数 */
} TaskStep;

/* 比赛流程拆成 stp1..stp7；每一步都是一段连续的走位/机构动作。
 * 站点与距离沿用旧 24 步路线的 CS_MAP_*。 */
static const TaskStep task_steps[] = {
  /* stp1 起点 → 扫码区 → 物料区 */
  {TASK_RIGHT,    CS_MAP_ALIGN_MM},
  {TASK_FWD,      CS_MAP_QR_DROP_MM},
  {TASK_SCAN,     CS_MAP_SCAN_WAIT_MS},
  {TASK_BACK,     CS_MAP_SCAN_BACK_MM},
  {TASK_RIGHT,    CS_MAP_RAW_LEG_MM},
  {TASK_TURN_ABS, 270},
  /* stp2 先回取料侧，再装 3 件，最后转到载物盘侧 */
  {TASK_ROT_HOME, 0},
  {TASK_GRAB,     0}, {TASK_GRAB, 1}, {TASK_GRAB, 2},
  {TASK_ROT_CAROUSEL, 0},
  /* stp3 物料区 → 粗加工区（机械臂在载物盘上方） */
  {TASK_LEFT,     CS_MAP_RAW_DROP_MM},
  {TASK_TURN_ABS, 90},
  /* stp4 一个封装步：内部按环循环 移动→DROP→PLT */
  {TASK_STP4,     0},
  /* stp5 地面取回第一批 */
  {TASK_GRAB_GND, 0}, {TASK_GRAB_GND, 1}, {TASK_GRAB_GND, 2},
  {TASK_CENTER,   0},
  /* stp6 暂存区放第一批 */
  {TASK_LEFT,     CS_MAP_LANE_CROSS_MM},
  {TASK_BACK,     CS_MAP_STORE_LEG_MM},
  {TASK_TURN_ABS, 0},
  {TASK_DROP,     0}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_DROP,     1}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_DROP,     2}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_CENTER,   0},
  /* stp7 暂存区 → 原料区，第二批全流程 */
  {TASK_LEFT,     CS_MAP_STORE_EXIT_MM},
  {TASK_BACK,     CS_MAP_RAW2_LEG_MM},
  {TASK_TURN_ABS, 270},
  {TASK_GRAB,     3}, {TASK_GRAB, 4}, {TASK_GRAB, 5},
  {TASK_ROT_CAROUSEL, 0},
  {TASK_LEFT,     CS_MAP_RAW_DROP_MM},
  {TASK_TURN_ABS, 90},
  {TASK_ROT_HOME, 0},
  {TASK_DROP,     3}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_DROP,     4}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_DROP,     5}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_GRAB_GND, 3}, {TASK_GRAB_GND, 4}, {TASK_GRAB_GND, 5},
  {TASK_CENTER,   0},
  {TASK_LEFT,     CS_MAP_LANE_CROSS_MM},
  {TASK_BACK,     CS_MAP_STORE_LEG_MM},
  {TASK_TURN_ABS, 0},
  {TASK_DROP,     3}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_DROP,     4}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_DROP,     5}, {TASK_PLT, PK_CYC_PLAT_MS},
  {TASK_CENTER,   0},
  {TASK_LEFT,     CS_MAP_RETURN_EAST_MM},
  {TASK_BACK,     CS_MAP_RETURN_NORTH_MM}
};
#define TASK_COUNT ((uint8_t)(sizeof(task_steps) / sizeof(task_steps[0])))
typedef char TaskCountOk[(TASK_COUNT >= 1U && TASK_COUNT <= 200U) ? 1 : -1];

/* stp1..stp7 在 task_steps[] 里的起始下标；STP n 从对应段开始跑。 */
#define STP_COUNT 7U
static const uint8_t stp_start[STP_COUNT] = {0U, 6U, 11U, 13U, 14U, 18U, 28U};

typedef struct {
  uint8_t  active;
  uint8_t  index;
  uint8_t  sub;            /* 整边横移的子阶段，见 TASK_SUB_* */
  uint32_t wait_until_ms;
  int16_t  heading_deg;
  int16_t  edge_heading;   /* 整边横移前的航向，走完要转回来 */
  uint16_t edge_mm;        /* 整边横移的距离 */
  uint8_t  action_phase;   /* 0 无；1 对准；2 抓取；3 移环；4 DROP */
  uint8_t  action_ring;    /* 当前目标环号 */
  uint8_t  action_index;   /* 0..5，对应两批第几件 */
  uint8_t  cur_ring;       /* 车当前对着几号环（2 = 中间） */
  uint32_t action_deadline_ms; /* 等物料的目标时刻 */
  uint8_t  action_retry;   /* 已重试次数（调试用） */
  uint8_t  dry_run;        /* 1 = 不接视觉，只跑底盘/换环 */
} Task;

/* 整边横移被拆成"转 90° → 直行 → 转回"三步，用 sub 记录进度。 */
#define TASK_SUB_NONE   0U
#define TASK_SUB_OUT    1U   /* 已发出 90° 转向，等它转完 */
#define TASK_SUB_DRIVE  2U   /* 已发出直行，等它走完 */
#define TASK_SUB_BACK   3U   /* 已发出转回，等它转完 */

static Task task;

/* stp4 封装状态：每个环 移动 → DROP → PLT。 */
typedef enum {
  STP4_NONE = 0,
  STP4_HOME,
  STP4_MOVE,
  STP4_DROP,
  STP4_PLT
} Stp4Phase;
static struct {
  uint8_t index;
  uint8_t phase;
  uint8_t ring;
  uint32_t wait_until_ms;
} stp4;

/* 任务码解析出的两批颜色/环号；默认 1/2/3 方便不扫码先测机构。 */
static uint8_t plan_colors[2][3] = {{1U, 2U, 3U}, {1U, 2U, 3U}};
static uint8_t plan_rings[2][3] = {{1U, 2U, 3U}, {1U, 2U, 3U}};
static uint8_t plan_valid;

uint8_t ChassisTask_SetPlan(const char *code)
{
  uint8_t group, item;
  uint8_t value;

  if (code == 0 || strlen(code) != 15U) return 0U;
  for (group = 0U; group < 4U; group++) {
    for (item = 0U; item < 3U; item++) {
      char character = code[group * 4U + item];
      if (character < '1' || character > '6') return 0U;
      value = (uint8_t)(character - '0');
      if (group == 0U) plan_colors[0][item] = value;
      else if (group == 1U) plan_rings[0][item] = value;
      else if (group == 2U) plan_colors[1][item] = value;
      else plan_rings[1][item] = value;
    }
  }
  for (group = 0U; group < 2U; group++) {
    for (item = 0U; item < 3U; item++) {
      if (plan_rings[group][item] < 1U || plan_rings[group][item] > 3U) return 0U;
    }
  }
  plan_valid = 1U;
  return 1U;
}

static int16_t wrap_deg(int32_t deg)
{
  while (deg > 180) deg -= 360;
  while (deg < -180) deg += 360;
  return (int16_t)deg;
}

static int16_t norm_deg360(int32_t deg)
{
  while (deg >= 360) deg -= 360;
  while (deg < 0) deg += 360;
  return (int16_t)deg;
}

static const char *kind_name(uint8_t kind)
{
  switch (kind) {
    case TASK_FWD:      return "FWD";
    case TASK_BACK:     return "BACK";
    case TASK_LEFT:     return "LEFT";
    case TASK_RIGHT:    return "RIGHT";
    case TASK_TURN:     return "TURN";
    case TASK_WAIT:     return "WAIT";
    case TASK_RESET:    return "RESET";
    case TASK_TURN_ABS: return "TURN_ABS";
    case TASK_CYC:      return "CYC";
    case TASK_SCAN:     return "SCAN";
    case TASK_TRG:      return "TRG";
    case TASK_GRAB:     return "GRAB";
    case TASK_GRAB_GND: return "GRAB_GND";
    case TASK_DROP:     return "DROP";
    case TASK_CENTER:   return "CENTER";
    case TASK_PLT:      return "PLT";
    case TASK_ROT_HOME: return "ROT_HOME";
    case TASK_ROT_CAROUSEL: return "ROT_CAROUSEL";
    case TASK_STP4: return "STP4";
    default:            return "?";
  }
}

void ChassisTask_Init(void)
{
  task.active = 0U;
  task.index = 0U;
  task.sub = TASK_SUB_NONE;
  task.wait_until_ms = 0U;
  task.heading_deg = 0;
  task.edge_heading = 0;
  task.edge_mm = 0U;
}

void ChassisTask_Cancel(void) { task.active = 0U; task.sub = TASK_SUB_NONE; }

void ChassisTask_Abort(const char *reason)
{
  task.active = 0U;
  task.sub = TASK_SUB_NONE;
  ChassisMotion_Cancel();
  Step6_StopAll();
  Pick_StopAll();          /* CYC 跑到一半被中止时，把机构两轴和载物台也停掉 */
  ChassisIo_Reply("TASK aborted: ");
  ChassisIo_Reply(reason);
  ChassisIo_Reply("\r\n");
}

uint8_t ChassisTask_IsActive(void) { return task.active; }

uint8_t ChassisTask_StepNumber(void)
{
  return task.active ? (uint8_t)(task.index + 1U) : 0U;
}

int16_t ChassisTask_HeadingDeg(void) { return task.heading_deg; }

void ChassisTask_SetHeadingDeg(int16_t heading_deg)
{
  task.heading_deg = heading_deg;
}

/* 转到指定航向：有陀螺走绝对 yaw，没有就退回相对角开环。 */
static uint8_t spin_to_heading(int16_t target_deg)
{
  int16_t delta;

  if (Imu_IsFresh() != 0U) {
    return ChassisMotion_SpinAbs(target_deg, (int32_t)target_deg * 1000);
  }
  delta = wrap_deg((int32_t)target_deg - task.heading_deg);
  return (delta == 0) ? 1U : ChassisMotion_Spin(delta, task.heading_deg);
}

static void advance_step(void);
static void stp4_start(void);
static void stp4_poll(void);
static uint8_t stp4_begin_ring(void);
static uint8_t stp4_start_drop(void);

/* 找下一个"会动"的步骤：WAIT / CYC 都不改位置也不改朝向，可以直接跳过。
 * 返回 0xFF 表示后面已经没有步骤了。 */
static uint8_t next_moving_kind(void)
{
  uint8_t i = (uint8_t)(task.index + 1U);

  while (i < TASK_COUNT &&
         (task_steps[i].kind == TASK_WAIT || task_steps[i].kind == TASK_CYC)) {
    i++;
  }
  return (i < TASK_COUNT) ? task_steps[i].kind : 0xFFU;
}

/* 整边横移的子阶段推进：转向 → 直行 → 转回 → 本步结束。 */
static void edge_step(void)
{
  if (task.sub == TASK_SUB_OUT) {
    task.sub = TASK_SUB_DRIVE;
    if (ChassisMotion_TranslateAbs(1, 1, 1, 1, task.edge_mm, CS_TASK_FWD_MM_PER_S,
                                   (int32_t)task.heading_deg * 1000) == 0U) {
      ChassisTask_Abort("edge straight failed");
      return;
    }
    ChassisIo_Reply("TASK EDGE straight\r\n");
    return;
  }
  if (task.sub == TASK_SUB_DRIVE) {
    /* 下一步本来就是 TURN_ABS 时，"转回原航向"是白转一次：
     * 日志里步骤 8→9 是 270° 转 +90 走完 → 再转回 270° → 再转 180° 到 90°，
     * 一共转了 270°，其中 180° 是多余的（约 4.5 s）。
     * 这种情况直接交给下一步的绝对转身，从当前朝向一次性转过去。 */
    if (next_moving_kind() == TASK_TURN_ABS) {
      task.sub = TASK_SUB_NONE;
      advance_step();
      return;
    }
    task.sub = TASK_SUB_BACK;
    if (spin_to_heading(task.edge_heading) == 0U) {
      ChassisTask_Abort("edge turn back failed");
      return;
    }
    ChassisIo_Reply("TASK EDGE turn back\r\n");
    return;
  }
  task.sub = TASK_SUB_NONE;
  advance_step();
}

/* 从当前环位平移到目标环位（1/2/3）；*started=1 表示已发起运动。 */
static uint8_t move_to_ring(uint8_t target, uint8_t *started)
{
  int8_t diff;
  uint16_t mm;
  uint8_t ok;
  uint8_t right;
  char line[64];
  int32_t yaw = (int32_t)task.heading_deg * 1000;

  *started = 0U;
  if (target < 1U || target > 3U) return 0U;
  diff = (int8_t)target - (int8_t)task.cur_ring;
  if (diff == 0) return 1U;
  mm = (uint16_t)((diff > 0) ? (uint16_t)(diff * STP_RING_STEP_MM)
                             : (uint16_t)(-diff * STP_RING_STEP_MM));
  right = (diff > 0) ? 1U : 0U;
#if STP_RING_REVERSE
  right = (right != 0U) ? 0U : 1U;
#endif
  (void)snprintf(line, sizeof(line), "TASK RING %u->%u %s %umm\r\n",
                 (unsigned)task.cur_ring, (unsigned)target,
                 right ? "RIGHT" : "LEFT", (unsigned)mm);
  ChassisIo_Reply(line);
  if (right != 0U) {
    ok = ChassisMotion_TranslateAbs(1, -1, -1, 1, mm, CS_TASK_STRAFE_MM_PER_S, yaw);
  } else {
    ok = ChassisMotion_TranslateAbs(-1, 1, 1, -1, mm, CS_TASK_STRAFE_MM_PER_S, yaw);
  }
  if (ok != 0U) {
    task.cur_ring = target;
    *started = 1U;
  }
  return ok;
}

static uint8_t run_step(void)
{
  const TaskStep *step = &task_steps[task.index];
  char line[80];
  int16_t delta;
  uint8_t ok = 1U;

  /* 先打步名再执行：否则 CYC 的 "CYC start" 会跑到 "TASK CYC 3" 前面，
   * 日志读起来像是上一步打印的。 */
  (void)snprintf(line, sizeof(line), "TASK %s %d\r\n",
                 kind_name(step->kind), (int)step->value);
  ChassisIo_Reply(line);

  switch (step->kind) {
    /* 前后左右平移用**绝对航向**当保持目标（TASK 起点 Imu_Zero 后的坐标系）：
     * 每一段都往 {task.heading_deg} 拉，上一段留下的残差不会变成下一段的新零点。
     * 没有陀螺时 TranslateAbs 会自动退回和 Translate 一样的行为。 */
    case TASK_FWD:
      ok = ChassisMotion_TranslateAbs(1, 1, 1, 1, (uint32_t)step->value, CS_TASK_FWD_MM_PER_S,
                                      (int32_t)task.heading_deg * 1000);
      break;
    case TASK_BACK:
      ok = ChassisMotion_TranslateAbs(-1, -1, -1, -1, (uint32_t)step->value, CS_TASK_FWD_MM_PER_S,
                                      (int32_t)task.heading_deg * 1000);
      break;
    case TASK_LEFT:
    case TASK_RIGHT: {
      /* 整边横移（>= CS_TASK_EDGE_MIN_MM）改成"转 90° → 直行 → 转回"：
       * 横移靠辊子侧向摩擦，越长越容易斜；直行是纯滚动，几乎不打滑，
       * 而且直行用 CS_TASK_FWD_MM_PER_S，和其它直行段同速。
       * 半个边（< 阈值）仍然直接横移。 */
      uint8_t left = (uint8_t)(step->kind == TASK_LEFT);

      if (step->value >= (int16_t)CS_TASK_EDGE_MIN_MM) {
        task.edge_mm = (uint16_t)step->value;
        task.edge_heading = task.heading_deg;
        task.sub = TASK_SUB_OUT;
        ok = spin_to_heading(norm_deg360((int32_t)task.heading_deg + (left ? 90 : -90)));
        ChassisIo_Reply(left ? "TASK EDGE left: turn +90, straight, turn back\r\n"
                             : "TASK EDGE right: turn -90, straight, turn back\r\n");
      } else {
        ok = ChassisMotion_TranslateAbs(left ? -1 : 1, left ? 1 : -1,
                                        left ? 1 : -1, left ? -1 : 1,
                                        (uint32_t)step->value, CS_TASK_STRAFE_MM_PER_S,
                                        (int32_t)task.heading_deg * 1000);
      }
      break;
    }
    case TASK_TURN:
      if (step->value == 0) ok = 1U;
      else ok = ChassisMotion_Spin(step->value, task.heading_deg);
      break;
    case TASK_TURN_ABS:
      /* 有陀螺就按**绝对 yaw** 转：每站都回到同一套航向，前几站的残差不累积。
       * 没有陀螺才退回"指令航向 + 相对角"的开环算法。 */
      if (Imu_IsFresh() != 0U) {
        ok = ChassisMotion_SpinAbs((int16_t)step->value, (int32_t)step->value * 1000);
      } else {
        delta = wrap_deg((int32_t)step->value - task.heading_deg);
        ok = (delta == 0) ? 1U : ChassisMotion_Spin(delta, task.heading_deg);
      }
      break;
    case TASK_RESET:
      task.heading_deg = 0;
      if (Imu_IsFresh() != 0U) Imu_Zero();   /* 陀螺零点跟着一起清 */
      ChassisIo_Reply("TASK reset heading=0\r\n");
      break;
    case TASK_WAIT:
      task.wait_until_ms = Board_NowMs() + (uint32_t)step->value;
      break;
    case TASK_CYC:
      /* 到站后跑 step->value 轮机构循环；真正的"等"在 ChassisTask_Poll 里，
       * 等 Pick_CycleIsActive() 变 0 才继续下一段走位。 */
      if (step->value < 1) { ok = 0U; break; }
      Pick_CycleStart((uint8_t)step->value);
      ok = Pick_CycleIsActive();      /* 启动被拒（未使能/正在跑）就中止任务 */
      break;
    case TASK_SCAN:
      /* 到扫码区：发 @SCAN，然后等树莓派把 TASK <code> 交回来（plan_valid）。 */
      if (task.dry_run != 0U) { ok = 1U; break; }
      plan_valid = 0U;
      (void)Serial_Write(SERIAL_RPI, (const uint8_t *)"@SCAN\n", 6U, 20U);
      task.wait_until_ms = Board_NowMs() + (uint32_t)step->value;
      ChassisIo_Reply("TASK SCAN sent, waiting for task code\r\n");
      break;
    case TASK_TRG:
      /* 载物台取料 → 放地面，跑 value 轮；TRG 自己处理重复。 */
      if (step->value < 1) { ok = 0U; break; }
      Pick_TrgStart((uint8_t)step->value);
      ok = Pick_TrgIsActive();
      break;
    case TASK_GRAB:
      if (step->value < 0 || step->value > 5) { ok = 0U; break; }
      if (task.dry_run != 0U) {
        Pick_GrabStart(1U);
        task.action_phase = 2U;
        ok = Pick_GrabIsActive();
        break;
      }
      task.action_index = (uint8_t)step->value;
      task.action_deadline_ms = Board_NowMs() + STP_GRAB_TARGET_WAIT_MS;
      task.action_retry = 0U;
      ok = Align_StartExTask(plan_colors[task.action_index / 3U][task.action_index % 3U],
                             AL_TOL_DEFAULT_MM, 1U);
      task.action_phase = (ok != 0U) ? 1U : 0U;
      break;
    case TASK_GRAB_GND:
      if (step->value < 0 || step->value > 5) { ok = 0U; break; }
      task.action_index = (uint8_t)step->value;
      if (task.dry_run != 0U) {
        uint8_t dry_started = 0U;
        task.action_ring = plan_rings[task.action_index / 3U][task.action_index % 3U];
        ok = move_to_ring(task.action_ring, &dry_started);
        if (ok != 0U && dry_started == 0U) {
          Pick_GroundGrabStart(1U);
          task.action_phase = 2U;
          ok = Pick_GrabIsActive();
        } else {
          task.action_phase = (ok != 0U) ? 3U : 0U;
        }
        break;
      }
      task.action_deadline_ms = Board_NowMs() + STP_GRAB_TARGET_WAIT_MS;
      task.action_retry = 0U;
      task.action_ring = plan_rings[task.action_index / 3U][task.action_index % 3U];
      {
        uint8_t started = 0U;
        ok = move_to_ring(task.action_ring, &started);
        if (ok != 0U && started == 0U) {
          ok = Align_StartEx(plan_colors[task.action_index / 3U][task.action_index % 3U],
                             AL_TOL_DEFAULT_MM, 0U);
          task.action_phase = (ok != 0U) ? 1U : 0U;
        } else if (ok != 0U) {
          task.action_phase = 3U;
        }
      }
      break;
    case TASK_STP4:
      stp4_start();
      ok = (stp4.phase != STP4_NONE) ? 1U : 0U;
      break;
    case TASK_DROP:
      if (step->value < 0 || step->value > 5) { ok = 0U; break; }
      task.action_index = (uint8_t)step->value;
      if (task.dry_run != 0U) {
        uint8_t dry_started = 0U;
        task.action_ring = plan_rings[task.action_index / 3U][task.action_index % 3U];
        ok = move_to_ring(task.action_ring, &dry_started);
        if (ok != 0U && dry_started == 0U) {
          Pick_DropStartDry(task.action_ring);
          task.action_phase = Pick_DropIsActive() ? 4U : 0U;
          ok = (task.action_phase != 0U) ? 1U : 0U;
        } else {
          task.action_phase = (ok != 0U) ? 3U : 0U;
        }
        break;
      }
      task.action_ring = plan_rings[task.action_index / 3U][task.action_index % 3U];
      {
        uint8_t started = 0U;
        ok = move_to_ring(task.action_ring, &started);
        if (ok != 0U && started == 0U) {
          Pick_DropStart(task.action_ring);
          task.action_phase = Pick_DropIsActive() ? 4U : 0U;
          ok = (task.action_phase != 0U) ? 1U : 0U;
        } else if (ok != 0U) {
          task.action_phase = 3U;
        }
      }
      break;
    case TASK_CENTER:
      {
        uint8_t started = 0U;
        ok = move_to_ring(2U, &started);
        task.action_phase = (ok != 0U && started != 0U) ? 3U : 0U;
      }
      break;
    case TASK_PLT:
      if (step->value < 1) { ok = 0U; break; }
      ok = Pick_PlatMove(PK_PLAT_JOG_PERMILLE, (uint32_t)step->value);
      task.wait_until_ms = Board_NowMs() + (uint32_t)step->value;
      break;
    case TASK_ROT_HOME:
      {
        int16_t deg = Pick_RotGetDeg();
        int32_t delta = -(int32_t)deg;
        (void)snprintf(line, sizeof(line), "ROT_HOME tracked=%d target=%ld\r\n",
                       (int)deg, (long)delta);
        ChassisIo_Reply(line);
        if (delta == 0) { ok = 1U; break; }
        if (delta > (int32_t)PK_ROT_MAX_DEG || delta < -(int32_t)PK_ROT_MAX_DEG) {
          ChassisIo_Reply("ROT_HOME skipped: tracked out of range\r\n");
          ok = 1U;
          break;
        }
        ok = Pick_RotDeg(delta, PK_ROT_HZ);
      }
      break;
    case TASK_ROT_CAROUSEL:
      {
        int16_t deg = Pick_RotGetDeg();
        int16_t target = -(int16_t)PK_CYC_ROT_DEG;
        if (deg == target) { ok = 1U; break; }
        ok = Pick_RotDeg((int32_t)target - (int32_t)deg, PK_ROT_HZ);
      }
      break;
    default:
      return 0U;
  }

  return ok;
}

static void start_at(uint8_t index, uint8_t dry)
{
  char line[64];

  if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return; }
  ChassisMotion_Cancel();
  Step6_StopAll();
  Pick_StopAll();      /* 上一次 CYC 还没跑完就重新开 TASK 的话，先把机构停干净 */
  task.index = index;
  task.sub = TASK_SUB_NONE;
  task.heading_deg = 0;
  task.wait_until_ms = 0U;
  task.action_phase = 0U;
  task.action_index = 0U;
  task.action_ring = 2U;
  task.cur_ring = 2U;
  task.action_deadline_ms = 0U;
  task.action_retry = 0U;
  task.dry_run = dry;
  task.active = 1U;
  /* 有陀螺就把当前朝向定为 0，之后所有 TURN_ABS 都按实测 yaw 闭环；没有就照旧开环。 */
  if (Imu_IsFresh() != 0U) {
    Imu_Zero();
    (void)snprintf(line, sizeof(line),
                   "TASK start steps=%u, HWT101 heading hold ON\r\n", (unsigned)TASK_COUNT);
  } else {
    (void)snprintf(line, sizeof(line),
                   "TASK start steps=%u open-loop no gyro\r\n", (unsigned)TASK_COUNT);
  }
  ChassisIo_Reply(line);
  (void)snprintf(line, sizeof(line), "TASK%s step %u/%u\r\n",
                 (dry != 0U) ? " DRY" : "",
                 (unsigned)index + 1U, (unsigned)TASK_COUNT);
  ChassisIo_Reply(line);
  if (run_step() == 0U) ChassisTask_Abort("step start failed");
}

void ChassisTask_Start(void)
{
  start_at(0U, 0U);
}

void ChassisTask_StartDry(void)
{
  start_at(0U, 1U);
}

uint8_t ChassisTask_StartStep(uint8_t step)
{
  if (step < 1U || step > STP_COUNT) {
    ChassisIo_Reply("ERR STP 1..7\r\n");
    return 0U;
  }
  start_at(stp_start[step - 1U], 0U);
  return 1U;
}

/* 推进到下一步：打印步号并启动；启动失败就中止任务。 */
static void advance_step(void)
{
  char line[64];

  task.index++;
  if (task.index >= TASK_COUNT) {
    task.active = 0U;
    (void)snprintf(line, sizeof(line), "TASK done steps=%u heading=%d\r\n",
                   (unsigned)TASK_COUNT, (int)task.heading_deg);
    ChassisIo_Reply(line);
    return;
  }
  (void)snprintf(line, sizeof(line), "TASK step %u/%u\r\n",
                 (unsigned)task.index + 1U, (unsigned)TASK_COUNT);
  ChassisIo_Reply(line);
  if (run_step() == 0U) ChassisTask_Abort("step start failed");
}

/* ---------------- stp4 封装 ---------------- */
static uint8_t stp4_start_drop(void)
{
  if (task.dry_run != 0U) Pick_DropStartDry(stp4.ring);
  else Pick_DropStart(stp4.ring);
  if (Pick_DropIsActive() == 0U) return 0U;
  stp4.phase = STP4_DROP;
  return 1U;
}

static uint8_t stp4_begin_ring(void)
{
  uint8_t started = 0U;
  uint8_t ok;

  if (stp4.index >= 3U) {
    stp4.phase = STP4_NONE;
    advance_step();
    return 1U;
  }
  stp4.ring = plan_rings[0][stp4.index];
  ok = move_to_ring(stp4.ring, &started);
  if (ok == 0U) return 0U;
  if (started == 0U) return stp4_start_drop();
  stp4.phase = STP4_MOVE;
  return 1U;
}

static void stp4_start(void)
{
  int16_t deg;

  stp4.index = 0U;
  stp4.phase = STP4_HOME;
  deg = Pick_RotGetDeg();
  if (deg != 0) {
    if (Pick_RotDeg(-(int32_t)deg, PK_ROT_HZ) == 0U) {
      stp4.phase = STP4_NONE;
      ChassisTask_Abort("stp4 rot home failed");
      return;
    }
  } else if (stp4_begin_ring() == 0U) {
    stp4.phase = STP4_NONE;
    ChassisTask_Abort("stp4 ring start failed");
  }
}

static void stp4_poll(void)
{
  uint32_t now_ms = Board_NowMs();

  if (stp4.phase == STP4_NONE) return;
  if (stp4.phase == STP4_HOME) {
    if (Pick_RotIsMoving()) return;
    if (stp4_begin_ring() == 0U) ChassisTask_Abort("stp4 ring start failed");
    return;
  }
  if (stp4.phase == STP4_MOVE) {
    if (ChassisMotion_MoveActive()) return;
    if (stp4_start_drop() == 0U) ChassisTask_Abort("stp4 drop start failed");
    return;
  }
  if (stp4.phase == STP4_DROP) {
    if (Pick_DropIsActive()) return;
    if (Pick_DropLastOk() == 0U) { ChassisTask_Abort("stp4 drop failed"); return; }
    if (Pick_PlatMove(PK_PLAT_JOG_PERMILLE, PK_CYC_PLAT_MS) == 0U) {
      ChassisTask_Abort("stp4 plt start failed");
      return;
    }
    stp4.wait_until_ms = now_ms + PK_CYC_PLAT_MS;
    stp4.phase = STP4_PLT;
    return;
  }
  if (stp4.phase == STP4_PLT) {
    if ((int32_t)(now_ms - stp4.wait_until_ms) < 0) return;
    stp4.index++;
    if (stp4_begin_ring() == 0U) ChassisTask_Abort("stp4 next ring failed");
  }
}

void ChassisTask_Poll(void)
{
  const TaskStep *step;

  if (!task.active) return;
  step = &task_steps[task.index];

  if (step->kind == TASK_STP4) {
    stp4_poll();
    return;
  }

  if (task.action_phase == 3U) {
    if (ChassisMotion_MoveActive()) return;
    if (task.dry_run != 0U) {
      if (step->kind == TASK_DROP) {
        Pick_DropStartDry(task.action_ring);
        task.action_phase = Pick_DropIsActive() ? 4U : 0U;
        if (task.action_phase == 0U) ChassisTask_Abort("dry drop start failed");
      } else if (step->kind == TASK_GRAB_GND) {
        Pick_GroundGrabStart(1U);
        task.action_phase = Pick_GrabIsActive() ? 2U : 0U;
        if (task.action_phase == 0U) ChassisTask_Abort("dry grab start failed");
      } else {
        task.action_phase = 0U;
        advance_step();
      }
      return;
    }
    if (step->kind == TASK_DROP) {
      Pick_DropStart(task.action_ring);
      task.action_phase = Pick_DropIsActive() ? 4U : 0U;
      if (task.action_phase == 0U) ChassisTask_Abort("drop start failed");
    } else if (step->kind == TASK_GRAB_GND) {
      if (Align_StartExTask(plan_colors[task.action_index / 3U][task.action_index % 3U],
                            AL_TOL_DEFAULT_MM, 0U) == 0U) {
        ChassisTask_Abort("align start failed");
        return;
      }
      task.action_phase = 1U;
    } else {
      task.action_phase = 0U;
      advance_step();
    }
    return;
  }

  if (task.action_phase == 1U) {
    if (step->kind == TASK_GRAB || step->kind == TASK_GRAB_GND) {
      (void)Align_TryCompleteForGrab();
    }
    if (Align_IsActive()) return;
    if (Align_LastReason() == ALIGN_REASON_NO_TARGET &&
        (int32_t)(task.action_deadline_ms - Board_NowMs()) > 0) {
      task.action_retry++;
      task.action_phase = 6U;
      task.wait_until_ms = Board_NowMs() + 500U;
      return;
    }
    if (Align_LastReason() != ALIGN_REASON_DONE) {
      ChassisTask_Abort("align failed");
      return;
    }
    if (step->kind == TASK_GRAB_GND) Pick_GroundGrabStart(1U);
    else Pick_GrabStart(1U);
    task.action_phase = 2U;
    return;
  }

  if (task.action_phase == 6U) {
    if ((int32_t)(Board_NowMs() - task.wait_until_ms) < 0) return;
    if (Align_StartExTask(plan_colors[task.action_index / 3U][task.action_index % 3U],
                          AL_TOL_DEFAULT_MM, 1U) == 0U) {
      ChassisTask_Abort("align restart failed");
      return;
    }
    task.action_phase = 1U;
    return;
  }

  if (task.action_phase == 2U) {
    if (Pick_GrabIsActive()) return;
    task.action_phase = 0U;
    advance_step();
    return;
  }
  if (task.action_phase == 4U) {
    if (Pick_DropIsActive()) return;
    task.action_phase = 0U;
    advance_step();
    return;
  }

  if (ChassisMotion_MoveActive() || Pick_RotIsMoving() || Pick_CycleIsActive() ||
      Pick_TrgIsActive() || Pick_DropIsActive() || Pick_GrabIsActive()) return;

  if (task.sub != TASK_SUB_NONE) {
    edge_step();
    return;
  }

  if (step->kind == TASK_CYC && Pick_CycleLastOk() == 0U) {
    ChassisTask_Abort("cycle not finished");
    return;
  }
  if (step->kind == TASK_SCAN) {
    if (task.dry_run != 0U || plan_valid != 0U) { advance_step(); return; }
    if ((int32_t)(Board_NowMs() - task.wait_until_ms) < 0) return;
    ChassisTask_Abort("scan timeout, no task code");
    return;
  }
  if ((step->kind == TASK_WAIT || step->kind == TASK_PLT) &&
      (int32_t)(Board_NowMs() - task.wait_until_ms) < 0) return;

  advance_step();
}
