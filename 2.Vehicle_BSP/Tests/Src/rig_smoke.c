#include "rig_smoke.h"
#include "board.h"
#include "pins/tb6612_u2.h"
#include "pins/tb6612_u3.h"
#include "pins/servo_u5.h"     /* 舵机引脚：2026-11-09 起按 V2 as-built = PB14/PB15 + TIM12 */
#include <stdio.h>
#include <string.h>

/* 机械臂舵机的人读标签。引脚固定为 V2 的 PB14 / TIM12_CH1
 * （老板那套 PA7/PE5 的 U5_SERVO_USE_PA7 开关已删除）。 */
#define ARM_LABEL      "ARM(U5/PB14)"
#define ARM_PIN_NAME   "PB14"

/*
 * 蓝牙联调台架（RIG Target）。台架上的执行器一次接齐，用蓝牙文本命令联调：
 *
 *   LIFT 升降台 = 张大头 ZDT STEP/DIR：STEP PD0、DIR PD1、EN PC8（低有效）、ALM PD14
 *   ROT  旋转台 = 张大头 ZDT STEP/DIR：STEP PD4、DIR PD5、EN PC9（低有效）、ALM PD15
 *   ARM  机械臂舵机 = U5：PA7/TIM14_CH1（默认）或 PE5/TIM9_CH1，见 pins/servo_u5.h 的开关
 *   PLT  载物台舵机 = U6 / PE6 / TIM9_CH2（⚠ 待实物确认第二路舵机口就是 PE6）
 *   MOTOR 四路底盘直流电机 = TB6612 U2（A/B）+ U3（C/D），TIM1 四路 20 kHz PWM
 *
 * 设计要点：
 *   - **上电不发脉冲、两轴失能、两个舵机不出 PWM、四路电机输出 0 且两片 TB6612 待机**，
 *     一切等命令（Motors_All 那套开机自动跑序列的行为**没有**搬过来）；
 *   - 发脉冲是阻塞的（脉宽由 DWT 保证），所以主循环里按"时间预算"分批发，
 *     两个轴每轮最多各占 RIG_PULSE_BUDGET_US，保证 STOP 和命令随时响应；
 *   - **运动期间的回显延迟到停稳后发**（AGENTS.md 第 6 节：发脉冲过程中不允许阻塞串口），
 *     只暂存一行，多余的丢弃并提示，与 Stepper_ZDT 的做法一致；
 *   - 遥测默认关闭，要波形才发 T 1。
 *
 * 四路电机那部分是从 `Tests/Src/motors_all_smoke.c`（`Motors_All` Target）搬过来的
 * 开环测试/调试：同一张 12 段序列、同一个 ±250‰ 上限、同一条"Motor_Set 不碰 STBY"的坑，
 * 区别只是**由命令触发**（MR / MALL），并且和步进、舵机共用一条蓝牙命令口。
 *
 * 命令一律 ASCII、末尾 CR/LF/CRLF：
 *   ? / S / STOP / T 0|1
 *   LE 0|1  RE 0|1        轴使能（EN 低有效）
 *   LV us   RV us         脉冲周期 20..20000 us
 *   LD 0|1  RD 0|1        方向
 *   LJ [n]  RJ [n]        点动 n 步（默认 20）
 *   LC 0|1  RC 0|1        连续转
 *   A <deg> | AU <us> | AO        机械臂舵机
 *   P <permille> | PU <us> | PO        载物台舵机
 *   PM <ms>                           载物台：固定速度只给时间（± = 方向，调试用）
 *   PICK                              一键抓取序列：开→降→夹→升→转→松→回位（单次）
 *   M / MM / M <a-d> <permille> / MALL <permille> / MR / MS   四路底盘电机
 */

/* ============================== 参数区 ============================== */

#define RIG_PORT                  SERIAL_BT   /* 蓝牙；改 SERIAL_DEBUG 就走有线调试口 */
#define RIG_TX_HZ                    5U       /* 遥测频率，默认关闭 */
/* ---- 运行指示灯（心跳）----
 * 板载 LED 在 PC0、低电平点亮（引脚事实见 BSP/Inc/pins/led_status.h）。
 * 它只用来说明"固件还在主循环里跑"，与两轴、两个舵机、四路电机的状态都无关：
 * 灯停在半亮/长灭 = 主循环卡在某个阻塞调用里（串口超时、脉冲预算吃满），比串口回显更早暴露。
 * ⚠ 这与核心板上那颗硬连 USB 5V 的**电源指示灯**不是一回事，别混。 */
#define RIG_LED_HEARTBEAT_MS          500U       /* 每 500 ms 翻转一次 = 1 Hz 闪烁 */
#define RIG_CMD_LEN                 40U
#define RIG_PULSE_PERIOD_MIN_US      20U
#define RIG_PULSE_PERIOD_MAX_US   20000U
#define RIG_PULSE_PERIOD_DEF_US    2000U      /* 名义 500 脉冲/秒 */
#define RIG_JOG_DEF_STEPS            20
#define RIG_JOG_MAX_STEPS        200000
#define RIG_PULSE_BUDGET_US        4000U      /* 每轮每轴最多发这么久的脉冲 */
#define RIG_PULSE_BURST_MAX         400U      /* 每轮每轴最多这么多个脉冲 */
#define RIG_SERVO_MIN_US            500U   /* 电气极限，只用于 AU/PU 的参数校验 */
#define RIG_SERVO_MAX_US           2500U
#define RIG_SERVO_DEF_US           1500U
/* 位置型舵机的**机械可用范围**。老的 Tests/Src/servo_smoke.c（MG90S）用的是
 * 1000..2000 us，并注明"500/2500 是电气极限、不是机械范围"。这里与它保持一致：
 * 500..2500 打过去舵机会顶死在机械极限或干脆不动，看起来像"坏了"。 */
#define RIG_SERVO_POS_MIN_US       1000U
#define RIG_SERVO_POS_MAX_US       2000U

/* ---- 载物台"按角度转"（时间推算）----
 * 连续旋转舵机没有位置反馈：转到某个角度只能"用固定速度跑一段时间"。
 * 角速度必须**实测**：让平台从标线起转 5 秒，量转过的角度 -> deg_per_s = 角度 / 5。 */
/* 转角度时固定用的速度。**必须和标定 `RIG_PLT_DEG_PER_S` 时用的速度一致**：
 * 下面那个常量是"这个千分比下每秒多少度"，两者是配对的。
 * 2026-10-17：用户调试改成 40%（400‰），对应脉宽 1500+200 = 1700 us。
 * 蓝牙测试就用同样的 `P 400` 计时间，测出来的 deg/s 直接能填进去。 */
#define RIG_PLT_SPEED_PERMILLE       400
#define RIG_PLT_DEG_PER_S             90    /* ⚠ 待标定：SPEED_PERMILLE 这个速度下每秒转多少度 */
#define RIG_PLT_DIR_SIGN               1    /* 方向反了改 -1，不用动接线 */
#define RIG_PLT_ONEWAY                 1    /* 1 = 总是朝一个方向转到位（消回差，路径可能更长） */
#define RIG_PLT_MIN_RUN_MS            80U   /* 最短运行时间，避免只是抖一下 */
#define RIG_PLT_MAX_RUN_MS         20000U   /* 单次最长运行时间，防参数写错转飞 */

/* ---- 一键抓取序列（`PICK`）----
 * 一条命令跑完一轮：夹爪开 → 升降台下降 → 夹紧 → 上升 → 旋转台顺时针 →
 *                  松开 → 旋转台逆时针 + 载物台同时转 → 完成。
 * 流程和参数都在这一段；改动作只改这里，不用动状态机。
 * ⚠ LV=20 us 时 14000 步 ≈ 280 ms（50 kHz），先确认 ZDT 跟得上、机构不撞限位。 */
#define RIG_SEQ_GRIP_OPEN_DEG          45   /* 夹爪松开角（A <deg>） */
#define RIG_SEQ_GRIP_CLOSE_DEG         73   /* 夹爪夹紧角 */
#define RIG_SEQ_SERVO_SETTLE_MS       500U  /* 舵机命令后等它到位，再动升降台 */
#define RIG_SEQ_LIFT_PERIOD_US         20U  /* 升降台脉冲周期（= LV） */
#define RIG_SEQ_LIFT_DIR_DOWN           0U  /* LD 0 = 向下 */
#define RIG_SEQ_LIFT_DIR_UP             1U  /* LD 1 = 向上 */
#define RIG_SEQ_LIFT_JOG_STEPS      14000   /* 单程步数 */
#define RIG_SEQ_ROT_PERIOD_US         500U  /* 旋转台脉冲周期（= RV） */
#define RIG_SEQ_ROT_DIR_CW              1U  /* RD 1 = 顺时针 */
#define RIG_SEQ_ROT_DIR_CCW             0U  /* RD 0 = 逆时针 */
#define RIG_SEQ_ROT_JOG_STEPS         1200  /* 单程步数 */
#define RIG_SEQ_PLATFORM_MS           420U  /* PM 420，与回程 RJ 同时开始 */
#define RIG_SEQ_STEP_TIMEOUT_MS     10000U  /* 单步超时，防轴不动时序列卡死；改大 LV/RV 周期要同步放大 */

/* ---- 夹爪动作流程（GRIP 命令，值沿用老工程 servo_smoke.c 的实测工作点）----
 * 老工程：上电 125° 夹紧 → 保持 5 s → 每 400 ms 退 2° → 70° 松开。
 * 这里改成按脉宽走（2° ≈ 11 us），并加了"超压"：夹紧时多压 OVERSHOOT_US，
 * 让舵机内部误差最大，吃满它本来就有的夹持力。 */
#define RIG_GRIP_OPEN_US            1389U   /* 老工程 70° */
#define RIG_GRIP_CLOSE_US           1694U   /* 老工程 125° */
#define RIG_GRIP_OVERSHOOT_US         30U   /* 夹紧超压；0 = 不加（用来做对照） */
#define RIG_GRIP_HOLD_MS            5000U   /* 夹紧保持时间 = 最长堵转时间 */
#define RIG_GRIP_STEP_US              12U
#define RIG_GRIP_STEP_MS             400U
/* ---- 舵机类型（2026-10-11 用户确认）----
 * 位置舵机：命令给角度 0..180 -> 500..2500 us。
 * 连续旋转型：命令给速度千分比 -1000..1000 -> 中位±span，**0 = 停**（脉宽 1500us 就是停）。
 * 中位附近留死区，避免"给 0 还在慢慢爬"。 */
#define RIG_ARM_CONTINUOUS            0U   /* 机械臂：位置舵机（待确认，若是连续型改 1） */
#define RIG_PLT_CONTINUOUS            1U   /* 载物台：连续旋转型 */
#define RIG_SERVO_NEUTRAL_US       1500U
#define RIG_SERVO_SPAN_US           500U   /* 满速 = 中位 ± span -> 1000 / 2000 us */
#define RIG_SERVO_DEADBAND_US        25U
/* ALM（驱动器报警输入，PD14/PD15，低有效）默认忽略：这两根脚固件里配了内部上拉，
 * 悬空读高=无报警，所以不接也不会误停；接上以后用命令 ALM 1 打开保护。 */
#define RIG_ALM_DEFAULT               0U
/* 运动期间的回显暂存槽数。只有一格时，连续发几条命令就会丢回显（用户已踩到）。 */
#define RIG_DEFER_SLOTS               4U
#define RIG_DEFER_LEN                96U

/* ---- 四路底盘直流电机（TB6612 U2/U3，M 命令）----
 * 值域与已实板验证过的 `Motor_A` / `Motors_All` 台架一致：输出上限 **±250‰（25%）**。
 * 12V 电机在 15% 以下可能低于启动死区而不转，所以不要再往下调；
 * 往上放宽到 600‰ 以内的路**没有实板验证记录**，改之前先想清楚。 */
#define RIG_MOTOR_MAX_PERMILLE       250
/* MR 序列里每个"转"段 / "停"段各持续多久，单位 ms（与 Motors_All 相同）。 */
#define RIG_MOTOR_SEQ_RUN_MS        1500U
#define RIG_MOTOR_SEQ_GAP_MS         800U
/* 整个序列的硬超时，单位 ms：到时无条件四路全停，是这套开环输出唯一的兜底。 */
#define RIG_MOTOR_SEQ_TIMEOUT_MS   40000U

#if RIG_PULSE_PERIOD_MIN_US < 10U || RIG_PULSE_PERIOD_MAX_US > 60000U
#error RIG pulse period range is invalid
#endif
#if RIG_PULSE_PERIOD_DEF_US < RIG_PULSE_PERIOD_MIN_US || RIG_PULSE_PERIOD_DEF_US > RIG_PULSE_PERIOD_MAX_US
#error RIG_PULSE_PERIOD_DEF_US out of range
#endif
#if RIG_PULSE_BUDGET_US < 500U || RIG_PULSE_BUDGET_US > 20000U
#error RIG_PULSE_BUDGET_US must be within 500..20000
#endif
#if RIG_PULSE_BURST_MAX < 1U || RIG_PULSE_BURST_MAX > 5000U
#error RIG_PULSE_BURST_MAX must be within 1..5000
#endif
#if RIG_SERVO_MIN_US < 400U || RIG_SERVO_MAX_US > 2600U || RIG_SERVO_MIN_US >= RIG_SERVO_MAX_US
#error RIG servo pulse range is invalid
#endif
#if RIG_ARM_CONTINUOUS != 0U && RIG_ARM_CONTINUOUS != 1U
#error RIG_ARM_CONTINUOUS must be 0 or 1
#endif
#if RIG_PLT_CONTINUOUS != 0U && RIG_PLT_CONTINUOUS != 1U
#error RIG_PLT_CONTINUOUS must be 0 or 1
#endif
#if RIG_SERVO_SPAN_US < 100U || RIG_SERVO_SPAN_US > 1000U
#error RIG_SERVO_SPAN_US must be within 100..1000
#endif
#if RIG_SERVO_POS_MIN_US < RIG_SERVO_MIN_US || RIG_SERVO_POS_MAX_US > RIG_SERVO_MAX_US || \
    RIG_SERVO_POS_MIN_US >= RIG_SERVO_POS_MAX_US
#error RIG_SERVO_POS_MIN_US/MAX_US must be a valid sub-range of the electrical limits
#endif
#if RIG_PLT_DEG_PER_S < 1 || RIG_PLT_DEG_PER_S > 2000
#error RIG_PLT_DEG_PER_S must be within 1..2000 (calibrate it: degrees per second at RIG_PLT_SPEED_PERMILLE)
#endif
#if RIG_PLT_DIR_SIGN != 1 && RIG_PLT_DIR_SIGN != -1
#error RIG_PLT_DIR_SIGN must be 1 or -1
#endif
#if RIG_PLT_ONEWAY != 0U && RIG_PLT_ONEWAY != 1U
#error RIG_PLT_ONEWAY must be 0 or 1
#endif
#if RIG_PLT_SPEED_PERMILLE < 100 || RIG_PLT_SPEED_PERMILLE > 1000
#error RIG_PLT_SPEED_PERMILLE must be within 100..1000
#endif
#if RIG_PLT_MIN_RUN_MS < 10U || RIG_PLT_MAX_RUN_MS > 60000U
#error platform run time limits are invalid
#endif
#if RIG_GRIP_OPEN_US < RIG_SERVO_MIN_US || RIG_GRIP_CLOSE_US > RIG_SERVO_MAX_US || \
    RIG_GRIP_OPEN_US >= RIG_GRIP_CLOSE_US
#error RIG_GRIP_OPEN_US/CLOSE_US must satisfy 500 <= OPEN < CLOSED <= 2500
#endif
#if (RIG_GRIP_CLOSE_US + RIG_GRIP_OVERSHOOT_US) > RIG_SERVO_MAX_US
#error close + overshoot exceeds the 2500 us electrical limit
#endif
#if RIG_GRIP_STEP_US < 1U || RIG_GRIP_STEP_US > (RIG_GRIP_CLOSE_US - RIG_GRIP_OPEN_US)
#error RIG_GRIP_STEP_US out of range
#endif
#if RIG_GRIP_STEP_MS < 20U || RIG_GRIP_STEP_MS > 5000U
#error RIG_GRIP_STEP_MS must be within 20..5000
#endif
#if (RIG_SERVO_NEUTRAL_US + RIG_SERVO_SPAN_US) > RIG_SERVO_MAX_US || \
    (RIG_SERVO_NEUTRAL_US - RIG_SERVO_SPAN_US) < RIG_SERVO_MIN_US
#error neutral +/- span must stay inside the servo pulse range
#endif
#if (1000U % RIG_TX_HZ) != 0U
#error RIG_TX_HZ must divide 1000
#endif
#if RIG_LED_HEARTBEAT_MS < 50U || RIG_LED_HEARTBEAT_MS > 5000U
#error RIG_LED_HEARTBEAT_MS must be within 50..5000
#endif
#if RIG_JOG_MAX_STEPS < 1
#error RIG_JOG_MAX_STEPS must be positive
#endif
#if RIG_DEFER_SLOTS < 1U || RIG_DEFER_SLOTS > 16U
#error RIG_DEFER_SLOTS must be within 1..16
#endif
#if RIG_DEFER_LEN < 48U || RIG_DEFER_LEN > 160U
#error RIG_DEFER_LEN must be within 48..160
#endif
#if RIG_MOTOR_MAX_PERMILLE < 1 || RIG_MOTOR_MAX_PERMILLE > 600
#error RIG_MOTOR_MAX_PERMILLE must be within 1..600
#endif
#if RIG_MOTOR_SEQ_RUN_MS == 0U || RIG_MOTOR_SEQ_RUN_MS > 10000U
#error RIG_MOTOR_SEQ_RUN_MS must be within 1..10000
#endif
#if RIG_MOTOR_SEQ_GAP_MS == 0U || RIG_MOTOR_SEQ_GAP_MS > 10000U
#error RIG_MOTOR_SEQ_GAP_MS must be within 1..10000
#endif
#if RIG_MOTOR_SEQ_TIMEOUT_MS < 5000U
#error RIG_MOTOR_SEQ_TIMEOUT_MS must be at least 5000
#endif
#if RIG_SEQ_GRIP_OPEN_DEG < 0 || RIG_SEQ_GRIP_OPEN_DEG > 180 || \
    RIG_SEQ_GRIP_CLOSE_DEG < 0 || RIG_SEQ_GRIP_CLOSE_DEG > 180
#error RIG_SEQ gripper angles must be within 0..180
#endif
#if RIG_SEQ_SERVO_SETTLE_MS > 5000U
#error RIG_SEQ_SERVO_SETTLE_MS must be within 0..5000
#endif
#if RIG_SEQ_LIFT_PERIOD_US < RIG_PULSE_PERIOD_MIN_US || \
    RIG_SEQ_LIFT_PERIOD_US > RIG_PULSE_PERIOD_MAX_US
#error RIG_SEQ_LIFT_PERIOD_US is out of the pulse period range
#endif
#if RIG_SEQ_ROT_PERIOD_US < RIG_PULSE_PERIOD_MIN_US || \
    RIG_SEQ_ROT_PERIOD_US > RIG_PULSE_PERIOD_MAX_US
#error RIG_SEQ_ROT_PERIOD_US is out of the pulse period range
#endif
#if RIG_SEQ_LIFT_JOG_STEPS < 1 || RIG_SEQ_LIFT_JOG_STEPS > RIG_JOG_MAX_STEPS
#error RIG_SEQ_LIFT_JOG_STEPS is out of range
#endif
#if RIG_SEQ_ROT_JOG_STEPS < 1 || RIG_SEQ_ROT_JOG_STEPS > RIG_JOG_MAX_STEPS
#error RIG_SEQ_ROT_JOG_STEPS is out of range
#endif
#if RIG_SEQ_PLATFORM_MS < RIG_PLT_MIN_RUN_MS || RIG_SEQ_PLATFORM_MS > RIG_PLT_MAX_RUN_MS
#error RIG_SEQ_PLATFORM_MS is out of the platform run-time range
#endif
#if RIG_SEQ_STEP_TIMEOUT_MS < 1000U || RIG_SEQ_STEP_TIMEOUT_MS > 120000U
#error RIG_SEQ_STEP_TIMEOUT_MS must be within 1000..120000
#endif

/* ============================== 状态 ============================== */

typedef struct {
  uint8_t  enabled;
  uint8_t  forward;
  uint8_t  continuous;
  uint8_t  alm_on;            /* 1 = 采信 ALM 输入（接了就开）；0 = 忽略（没接时默认） */
  uint8_t  moving_reported;   /* 上一次 Poll 是否在动，用来报"点动完成" */
  uint8_t  fault_reported;
  int32_t  remaining;         /* 点动剩余步数 */
  int32_t  position_steps;    /* 累计步数（正 = 当前 DIR 正向） */
  uint16_t period_us;
} Axis;

/* 轴 0 = 升降台（U1），轴 1 = 旋转台（U4）。要互换只改这一张表。 */
static const StepperId axis_stepper[2] = { STEPPER_1, STEPPER_2 };
static const char *const axis_label[2] = { "LIFT(U1)", "ROT(U4)" };

static Axis axes[2];
static uint16_t servo_pulse_us[2];
static uint8_t  servo_on[2];
/* 载物台的角度推算状态（开环：靠"速度 x 时间"）。 */
static int32_t  plt_angle_deg;
static int32_t  plt_target_deg;
static uint8_t  plt_move_active;
static uint32_t plt_move_stop_ms;
/* PM <ms>：固定速度、只给时间的一段转动（调试/标定用，不走角度换算）。 */
static uint8_t  plt_timed_active;
static uint32_t plt_timed_stop_ms;
/* PICK 一键抓取序列的状态机（流程实现在下方"一键抓取序列"一节）。 */
typedef enum {
  SEQ_IDLE = 0,
  SEQ_GRIP_OPEN,
  SEQ_LOWER,
  SEQ_GRIP_CLOSE,
  SEQ_RAISE,
  SEQ_ROT_CW,
  SEQ_GRIP_OPEN2,
  SEQ_RETURN,
  SEQ_DONE
} SeqState;
static SeqState seq_state;
static uint32_t seq_step_start_ms;
static uint32_t seq_deadline_ms;
/* 夹爪流程状态（GRIP 命令）。 */
typedef enum { GRIP_IDLE = 0, GRIP_HOLDING, GRIP_OPENING } GripState;
static GripState grip_state;
static uint16_t grip_pulse_us;
static uint32_t grip_next_ms;
static uint8_t  command_line[RIG_CMD_LEN];
static uint8_t  command_length;
static uint8_t  command_overflow;
static uint8_t  telemetry_enabled;
static uint32_t next_tx_ms;
/* 心跳灯的下一次翻转时刻（防回绕比较，见 AGENTS.md 第 6 节）。 */
static uint32_t led_toggle_due_ms;

/* 运动期间延迟回显：环形队列，满了才丢，并统计丢了多少条（同 Stepper_ZDT 的思路，但不止一格）。 */
static char deferred[RIG_DEFER_SLOTS][RIG_DEFER_LEN];
static uint8_t deferred_head;
static uint8_t deferred_count;
static uint8_t deferred_dropped;

/* ---- 四路底盘电机：状态 + 12 段序列（内容与 Motors_All 的 sequence[] 一致）---- */
typedef struct {
  const char *label_p;      /* 段落标签，串口原样打印（ASCII） */
  uint8_t     mask;         /* bit0..bit3 依次对应 A / B / C / D；0 = 这一段全停 */
  int16_t     permille;
  uint16_t    duration_ms;
} MotorStep;

static const MotorStep motor_sequence[] = {
  {"A forward only",    0x01U,  RIG_MOTOR_MAX_PERMILLE, RIG_MOTOR_SEQ_RUN_MS},
  {"all stop",          0x00U,  0,                      RIG_MOTOR_SEQ_GAP_MS},
  {"B forward only",    0x02U,  RIG_MOTOR_MAX_PERMILLE, RIG_MOTOR_SEQ_RUN_MS},
  {"all stop",          0x00U,  0,                      RIG_MOTOR_SEQ_GAP_MS},
  {"C forward only",    0x04U,  RIG_MOTOR_MAX_PERMILLE, RIG_MOTOR_SEQ_RUN_MS},
  {"all stop",          0x00U,  0,                      RIG_MOTOR_SEQ_GAP_MS},
  {"D forward only",    0x08U,  RIG_MOTOR_MAX_PERMILLE, RIG_MOTOR_SEQ_RUN_MS},
  {"all stop",          0x00U,  0,                      RIG_MOTOR_SEQ_GAP_MS},
  {"all four forward",  0x0FU,  RIG_MOTOR_MAX_PERMILLE, RIG_MOTOR_SEQ_RUN_MS},
  {"all stop",          0x00U,  0,                      RIG_MOTOR_SEQ_GAP_MS},
  {"all four reverse",  0x0FU, -RIG_MOTOR_MAX_PERMILLE, RIG_MOTOR_SEQ_RUN_MS},
  {"all stop",          0x00U,  0,                      RIG_MOTOR_SEQ_GAP_MS},
};
#define RIG_MOTOR_SEQ_LEN ((uint8_t)(sizeof(motor_sequence) / sizeof(motor_sequence[0])))

static const MotorId motor_ids[4] = { MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D };

static uint8_t  motor_sequence_active;      /* MR 的 12 段序列是否在跑 */
static uint8_t  motor_step_index;
static uint32_t motor_step_until_ms;
static uint32_t motor_sequence_start_ms;
/* 四路当前指令值。Motor_Set() 没有读回接口，所以自己记一份给 S / M / H 看。 */
static int16_t  motor_permille[4];

/* ============================== 输出 ============================== */

static void send_now(const char *text)
{
  (void)Serial_Write(RIG_PORT, (const uint8_t *)text, (uint16_t)strlen(text), 100U);
}
static uint8_t axis_moving(const Axis *axis)
{
  return (axis->continuous != 0U || axis->remaining > 0) ? 1U : 0U;
}
static uint8_t any_axis_moving(void)
{
  return (uint8_t)(axis_moving(&axes[0]) || axis_moving(&axes[1]));
}
/* 运动期间回显排队，停稳后按顺序补发，避免阻塞串口把脉冲节奏拉长。 */
static void reply(const char *text)
{
  if (any_axis_moving()) {
    if (deferred_count < (uint8_t)RIG_DEFER_SLOTS) {
      uint8_t slot = (uint8_t)((deferred_head + deferred_count) % RIG_DEFER_SLOTS);
      (void)snprintf(deferred[slot], sizeof(deferred[slot]), "%s", text);
      deferred_count++;
    } else {
      deferred_dropped++;
    }
    return;
  }
  send_now(text);
}
static void flush_deferred(void)
{
  char line[64];
  if (any_axis_moving()) return;
  while (deferred_count != 0U) {
    send_now(deferred[deferred_head]);
    deferred_head = (uint8_t)((deferred_head + 1U) % RIG_DEFER_SLOTS);
    deferred_count--;
  }
  if (deferred_dropped != 0U) {
    (void)snprintf(line, sizeof(line), "WARN %u deferred replies dropped\r\n",
                   (unsigned)deferred_dropped);
    deferred_dropped = 0U;
    send_now(line);
  }
}

/* ============================== 运行指示灯 ============================== */

/* 板载 PC0 LED 的心跳：非阻塞，只看 Board_NowMs() 的时间戳。
 * 故意放在主循环里（而不是定时器中断里），这样"灯不动"就等价于"主循环卡住了"，
 * 是台架上判断固件死没死最快的一条判据。 */
static void led_heartbeat_poll(uint32_t now_ms)
{
  if ((int32_t)(now_ms - led_toggle_due_ms) < 0) return;
  led_toggle_due_ms = now_ms + (uint32_t)RIG_LED_HEARTBEAT_MS;
  Led_StatusToggle();
}

/* ============================== 脉冲引擎 ============================== */

/* 一个轴这一轮最多发 RIG_PULSE_BUDGET_US 的脉冲，然后交还主循环。
 * 脉宽本身由 Stepper_PulseUs() 用 DWT 保证，所以这里只控制"每轮占多久"。 */
static void axis_service(Axis *axis, StepperId id)
{
  uint32_t high_us, low_us, budget_cycles, elapsed_cycles, start_cycles;
  uint32_t emitted = 0U;

  if (axis_moving(axis) == 0U) return;
  if (axis->alm_on != 0U && Stepper_Fault(id) != 0U) {
    axis->continuous = 0U;
    axis->remaining = 0;
    if (axis->fault_reported == 0U) {
      axis->fault_reported = 1U;
      /* 报警先停脉冲，提示在 Poll 里发（不在这里发串口）。 */
    }
    return;
  }

  high_us = (uint32_t)axis->period_us / 2U;
  if (high_us == 0U) high_us = 1U;
  low_us = (uint32_t)axis->period_us - high_us;
  if (low_us == 0U) low_us = 1U;

  budget_cycles = RIG_PULSE_BUDGET_US * (SystemCoreClock / 1000000U);
  start_cycles = DWT->CYCCNT;

  while (axis_moving(axis) != 0U && emitted < RIG_PULSE_BURST_MAX) {
    Stepper_PulseUs(id, (uint16_t)high_us, (uint16_t)low_us);
    if (axis->continuous == 0U) axis->remaining--;
    axis->position_steps += (axis->forward != 0U) ? 1 : -1;
    emitted++;
    elapsed_cycles = (uint32_t)(DWT->CYCCNT - start_cycles);
    if (elapsed_cycles >= budget_cycles) break;
  }

  if (axis->alm_on != 0U && Stepper_Fault(id) != 0U) {
    axis->continuous = 0U;
    axis->remaining = 0;
    axis->fault_reported = 1U;
  }
}

static void stop_axes(void)
{
  uint8_t i;
  for (i = 0U; i < 2U; ++i) {
    axes[i].continuous = 0U;
    axes[i].remaining = 0;
  }
}

/* 每轮报一次状态变化：点动完成、报警。 */
static void report_changes(void)
{
  char line[96];
  uint8_t i;
  for (i = 0U; i < 2U; ++i) {
    Axis *axis = &axes[i];
    uint8_t moving = axis_moving(axis);
    if (axis->fault_reported != 0U) {
      (void)snprintf(line, sizeof(line), "ALM %s: pulses stopped (check driver alarm)\r\n",
                     axis_label[i]);
      reply(line);
      axis->fault_reported = 0U;   /* 只在边沿报一次；释放/重启后才会再报 */
      axis->moving_reported = 0U;
      continue;
    }
    if (axis->moving_reported != 0U && moving == 0U) {
      (void)snprintf(line, sizeof(line), "%s stopped, pos=%ld steps\r\n",
                     axis_label[i], (long)axis->position_steps);
      reply(line);
    }
    axis->moving_reported = moving;
  }
}

/* ============================== 命令 ============================== */

static uint8_t parse_arg(const char *text, int32_t *value)
{
  int32_t magnitude = 0;
  uint8_t digits = 0U, negative = 0U;
  while (*text == ' ' || *text == '\t') ++text;
  if (*text == '-') { negative = 1U; ++text; }
  else if (*text == '+') ++text;
  while (*text >= '0' && *text <= '9') {
    if (magnitude > 2000000) return 0U;
    magnitude = magnitude * 10 + (int32_t)(*text - '0');
    ++text;
    digits = 1U;
  }
  while (*text == ' ' || *text == '\t') ++text;
  if (digits == 0U || *text != '\0') return 0U;
  *value = (negative != 0U) ? -magnitude : magnitude;
  return 1U;
}

/* ------------------- 四路底盘直流电机（TB6612 U2/U3，M 命令） -------------------
 * 通道映射（对着实物核对接线）：
 *   A -> U2 chA  PWM PE9  AIN1 PE7  AIN2 PE8   STBY PD2
 *   B -> U2 chB  PWM PE11 BIN1 PE10 BIN2 PE12  STBY PD2
 *   C -> U3 chA  PWM PE13 CIN1 PB11 CIN2 PB12  STBY PD3
 *   D -> U3 chB  PWM PE14 DIN1 PB13 DIN2 PB14  STBY PD3
 *
 * 坑（Motors_All 已经踩过一次，这里照抄处理）：Motor_Set() **不碰 STBY**，而
 * Motor_StopAll() 会把两片 TB6612 压回待机。所以只要这一路要给非零输出，就必须先
 * Motor_Standby(n, 1) 唤醒，否则 CCR 写进去了电机也不动。 */

/* A/B 在 U2（driver 0），C/D 在 U3（driver 1）。 */
static uint8_t motor_driver(uint8_t channel)
{
  return (channel >= 2U) ? 1U : 0U;
}

/* 按 mask/permille 写四路：mask 里没有的通道写 0；要动就先唤醒两片驱动器。 */
static void motor_apply(uint8_t mask, int32_t permille)
{
  uint8_t i;
  if (mask != 0U) {
    Motor_Standby(0U, 1U);
    Motor_Standby(1U, 1U);
  }
  for (i = 0U; i < 4U; ++i) {
    int16_t value = ((mask & (uint8_t)(1U << i)) != 0U) ? (int16_t)permille : 0;
    (void)Motor_Set(motor_ids[i], value);
    motor_permille[i] = value;
  }
}

/* 四路全停并中止序列：Motor_StopAll() 会把两片 TB6612 一起压回待机。 */
static void motor_stop_all(void)
{
  uint8_t i;
  Motor_StopAll();
  motor_sequence_active = 0U;
  motor_step_index = 0U;
  for (i = 0U; i < 4U; ++i) motor_permille[i] = 0;
}

/* 手动命令的公共校验：越限就回一行 ERR 并返回 0。 */
static uint8_t motor_limit_ok(int32_t permille)
{
  char line[64];
  if (permille > (int32_t)RIG_MOTOR_MAX_PERMILLE ||
      permille < -(int32_t)RIG_MOTOR_MAX_PERMILLE) {
    (void)snprintf(line, sizeof(line), "ERR motor limit +/-%d/1000\r\n",
                   (int)RIG_MOTOR_MAX_PERMILLE);
    reply(line);
    return 0U;
  }
  return 1U;
}

/* 手动单独驱动一路（手动优先：先中止 MR 序列）。只改这一路，其它路保持原值。 */
static void motor_manual_channel(uint8_t channel, int32_t permille)
{
  char line[80];
  if (motor_limit_ok(permille) == 0U) return;
  if (motor_sequence_active != 0U) motor_stop_all();
  if (permille != 0) Motor_Standby(motor_driver(channel), 1U);
  (void)Motor_Set(motor_ids[channel], (int16_t)permille);
  motor_permille[channel] = (int16_t)permille;
  (void)snprintf(line, sizeof(line), "MOTOR %c = %ld/1000\r\n",
                 (char)('A' + (int)channel), (long)permille);
  reply(line);
}

/* 四路一起（MALL）。给 0 就是全停回待机，不去唤醒驱动器。 */
static void motor_manual_all(int32_t permille)
{
  char line[80];
  if (motor_limit_ok(permille) == 0U) return;
  if (motor_sequence_active != 0U) motor_stop_all();
  if (permille == 0) {
    motor_stop_all();
    reply("MOTOR all stopped (drivers standby)\r\n");
    return;
  }
  motor_apply(0x0FU, permille);
  (void)snprintf(line, sizeof(line), "MOTOR all four = %ld/1000\r\n", (long)permille);
  reply(line);
}

static void motor_print_mapping(void)
{
  reply("MOTOR channel mapping (verify against the real wiring):\r\n");
  reply("  A -> U2 chA  PWM PE9  AIN1 PE7  AIN2 PE8   STBY PD2\r\n");
  reply("  B -> U2 chB  PWM PE11 BIN1 PE10 BIN2 PE12  STBY PD2\r\n");
  reply("  C -> U3 chA  PWM PE13 CIN1 PB11 CIN2 PB12  STBY PD3\r\n");
  reply("  D -> U3 chB  PWM PE14 DIN1 PB13 DIN2 PB14  STBY PD3\r\n");
}

static void motor_print_status(void)
{
  char line[112];
  (void)snprintf(line, sizeof(line),
                 "MOTOR A=%d B=%d C=%d D=%d /1000 limit=%d seq=%u step=%u/%u\r\n",
                 (int)motor_permille[0], (int)motor_permille[1],
                 (int)motor_permille[2], (int)motor_permille[3],
                 (int)RIG_MOTOR_MAX_PERMILLE, (unsigned)motor_sequence_active,
                 (unsigned)motor_step_index, (unsigned)RIG_MOTOR_SEQ_LEN);
  reply(line);
}

static void motor_print_help(void)
{
  reply("M / MM          motor status / channel mapping (also: M ?)\r\n");
  reply("M <a|b|c|d> <permille>   drive one channel, limit +/-250 per mille\r\n");
  reply("MALL <permille>          drive all four at once (0 = all stop)\r\n");
  reply("MR / MS                  12-step open-loop sequence / stop the motors\r\n");
}

static void motor_step_print(uint8_t index)
{
  const MotorStep *step_p = &motor_sequence[index];
  char line[96];
  if (step_p->mask == 0U) {
    (void)snprintf(line, sizeof(line), "MOTOR [%u/%u] %s (%u ms)\r\n",
                   (unsigned)(index + 1U), (unsigned)RIG_MOTOR_SEQ_LEN,
                   step_p->label_p, (unsigned)step_p->duration_ms);
  } else {
    (void)snprintf(line, sizeof(line), "MOTOR [%u/%u] %s, duty %d/1000 (%u ms)\r\n",
                   (unsigned)(index + 1U), (unsigned)RIG_MOTOR_SEQ_LEN,
                   step_p->label_p, (int)step_p->permille,
                   (unsigned)step_p->duration_ms);
  }
  reply(line);
}

static void motor_sequence_start(void)
{
  motor_stop_all();                 /* 先全停，顺便清掉上一次的进度 */
  motor_sequence_active = 1U;
  motor_step_index = 0U;
  motor_sequence_start_ms = Board_NowMs();
  motor_apply(motor_sequence[0].mask, motor_sequence[0].permille);
  motor_step_until_ms = motor_sequence_start_ms + (uint32_t)motor_sequence[0].duration_ms;
  reply("MOTOR sequence start; LIFT THE WHEELS OFF THE GROUND; MS/STOP to abort\r\n");
  motor_step_print(0U);
}

/* 序列推进：非阻塞，只看 Board_NowMs() 时间戳；MS / STOP 随时能打断。 */
static void motor_sequence_poll(uint32_t now_ms)
{
  uint8_t next;
  if (motor_sequence_active == 0U) return;
  if ((int32_t)(now_ms - motor_sequence_start_ms) > (int32_t)RIG_MOTOR_SEQ_TIMEOUT_MS) {
    motor_stop_all();
    reply("MOTOR hard timeout, all stopped\r\n");
    return;
  }
  if ((int32_t)(now_ms - motor_step_until_ms) < 0) return;

  next = (uint8_t)(motor_step_index + 1U);
  if (next >= RIG_MOTOR_SEQ_LEN) {
    motor_stop_all();
    reply("MOTOR sequence done, all stopped. MR to rerun\r\n");
    return;
  }
  motor_step_index = next;
  motor_apply(motor_sequence[next].mask, motor_sequence[next].permille);
  motor_step_until_ms = now_ms + (uint32_t)motor_sequence[next].duration_ms;
  motor_step_print(next);
}

static void motor_command(const char *arg)
{
  int32_t value = 0;
  while (*arg == ' ' || *arg == '\t') ++arg;

  if (*arg == '\0') { motor_print_status(); return; }
  if (*arg == '?') { motor_print_mapping(); motor_print_help(); return; }
  if (arg[0] == 'M' && arg[1] == '\0') { motor_print_mapping(); return; }
  if (arg[0] == 'R' && arg[1] == '\0') { motor_sequence_start(); return; }
  if (arg[0] == 'S' && arg[1] == '\0') {
    motor_stop_all();
    reply("MOTOR stopped, drivers standby (sequence aborted)\r\n");
    return;
  }
  if (strncmp(arg, "ALL", 3) == 0) {
    if (parse_arg(arg + 3, &value) == 0U) { reply("ERR use MALL <permille>\r\n"); return; }
    motor_manual_all(value);
    return;
  }
  if (arg[0] >= 'A' && arg[0] <= 'D') {
    uint8_t channel = (uint8_t)(arg[0] - 'A');
    if (parse_arg(arg + 1, &value) == 0U) {
      reply("ERR use M <a|b|c|d> <permille>\r\n");
      return;
    }
    motor_manual_channel(channel, value);
    return;
  }
  reply("ERR unknown motor command; send M ?\r\n");
}

/* ------------------- 载物台：脉宽映射 + 按角度转（时间推算） -------------------
 * 连续旋转型舵机没有位置反馈，"转到 N 度"只能是：用固定速度跑 N/角速度 秒，然后回中位停。
 * 所以精度取决于 RIG_PLT_DEG_PER_S 标得准不准、以及电池电压稳不稳。 */

/* 逻辑千分比 -> 实际脉宽（含方向修正与中位死区）。 */
static uint16_t platform_pulse_from_permille(int32_t permille)
{
  int32_t pulse;
  if (permille > 1000) permille = 1000;
  if (permille < -1000) permille = -1000;
  permille *= (int32_t)RIG_PLT_DIR_SIGN;
  pulse = (int32_t)RIG_SERVO_NEUTRAL_US + (permille * (int32_t)RIG_SERVO_SPAN_US) / 1000;
  if (pulse > (int32_t)RIG_SERVO_NEUTRAL_US - (int32_t)RIG_SERVO_DEADBAND_US &&
      pulse < (int32_t)RIG_SERVO_NEUTRAL_US + (int32_t)RIG_SERVO_DEADBAND_US) {
    pulse = (int32_t)RIG_SERVO_NEUTRAL_US;   /* 死区内直接给中位，别让它爬 */
  }
  return (uint16_t)pulse;
}
static void platform_apply_permille(int32_t permille)
{
  uint16_t pulse = platform_pulse_from_permille(permille);
  if (Servo2_SetPulseUs(pulse) != HAL_OK) { reply("ERR servo pulse\r\n"); return; }
  servo_pulse_us[1] = pulse;
  servo_on[1] = 1U;
}
/* 归一化角度差：默认总是朝一个方向转到位（消回差）；RIG_PLT_ONEWAY=0 时走最短路径。 */
static int32_t platform_normalize_delta(int32_t delta)
{
#if RIG_PLT_ONEWAY
  while (delta < 0) delta += 360;
  while (delta >= 360) delta -= 360;
#else
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;
#endif
  return delta;
}
static void platform_move_deg(int32_t value, uint8_t relative)
{
  char line[112];
  int32_t delta, run_ms;

  if (relative != 0U) value = plt_angle_deg + value;
  delta = platform_normalize_delta(value - plt_angle_deg);
  if (delta == 0) { reply("PLT already at target\r\n"); return; }

  run_ms = (int32_t)(((int64_t)((delta < 0) ? -delta : delta) * 1000) / (int64_t)RIG_PLT_DEG_PER_S);
  if (run_ms < (int32_t)RIG_PLT_MIN_RUN_MS) run_ms = (int32_t)RIG_PLT_MIN_RUN_MS;
  if (run_ms > (int32_t)RIG_PLT_MAX_RUN_MS) run_ms = (int32_t)RIG_PLT_MAX_RUN_MS;

  plt_target_deg = plt_angle_deg + delta;
  plt_move_stop_ms = Board_NowMs() + (uint32_t)run_ms;
  plt_move_active = 1U;
  platform_apply_permille((delta > 0) ? (int32_t)RIG_PLT_SPEED_PERMILLE
                                      : -(int32_t)RIG_PLT_SPEED_PERMILLE);
  (void)snprintf(line, sizeof(line),
                 "PLT turning %ld deg in %ld ms (open loop, %d permille)\r\n",
                 (long)delta, (long)run_ms, (int)RIG_PLT_SPEED_PERMILLE);
  reply(line);
}
static void platform_poll(uint32_t now_ms)
{
  char line[80];
  if (plt_move_active == 0U) return;
  if ((int32_t)(now_ms - plt_move_stop_ms) < 0) return;
  plt_angle_deg = plt_target_deg;
  /* ⚠ 必须先清标志：不清的话下一轮 Poll 还会再进来一次，串口会每轮刷一行 "PLT done"、
     并反复写 1500 us（舵机中位有偏差时就成了"慢慢爬"）。 */
  plt_move_active = 0U;
  platform_apply_permille(0);            /* 时间到 -> 回中位停 */
  (void)snprintf(line, sizeof(line), "PLT done, angle=%ld deg\r\n", (long)plt_angle_deg);
  reply(line);
}
static void platform_angle_command(char sub, const char *arg)
{
  int32_t value = 0;
  char line[80];
  plt_timed_active = 0U;                 /* 角度命令优先，中止 PM 的定时段 */
  if (parse_arg(arg, &value) == 0U || value < -3600 || value > 3600) {
    reply("ERR angle -3600..3600 deg\r\n");
    return;
  }
  if (sub == 'S') {                      /* PS <deg>：把当前角度设为该值（标定/对零） */
    plt_angle_deg = value;
    plt_target_deg = value;
    plt_move_active = 0U;
    (void)snprintf(line, sizeof(line), "PLT angle set to %ld deg\r\n", (long)value);
    reply(line);
    return;
  }
  platform_move_deg(value, (sub == 'T') ? 1U : 0U);   /* PT 相对 / PA 绝对 */
}

/* PM <ms>：用 RIG_PLT_SPEED_PERMILLE 这个**固定速度**转 ms 毫秒，然后回中位。
 * 负的 ms 表示反方向（`PM -1500` = 反向 1.5 秒）。
 * 这就是"速度固件定、时间命令给"的调试口：改时间不用重编，蓝牙直接发。
 * ⚠ 连续旋转舵机没有位置反馈，跑完以后"当前角度"就不可知了；要再按角度转，先 `PS <deg>` 或 `Z`。 */
/* 载物台定时段的底层入口：只给时间和方向，不解析、不回显（`PM` 命令和 PICK 序列共用）。 */
static void platform_timed_start(uint32_t ms, uint8_t forward)
{
  plt_move_active = 0U;                  /* 手动/序列优先，中止角度运动 */
  plt_timed_active = 1U;
  plt_timed_stop_ms = Board_NowMs() + ms;
  platform_apply_permille((forward != 0U) ? (int32_t)RIG_PLT_SPEED_PERMILLE
                                          : -(int32_t)RIG_PLT_SPEED_PERMILLE);
}
static void platform_timed_command(const char *arg)
{
  char line[112];
  int32_t value = 0;
  int32_t ms;

  if (parse_arg(arg, &value) == 0U || value == 0) {
    reply("ERR use PM <ms> (+/- = direction)\r\n");
    return;
  }
  ms = (value < 0) ? -value : value;
  if (ms < (int32_t)RIG_PLT_MIN_RUN_MS || ms > (int32_t)RIG_PLT_MAX_RUN_MS) {
    (void)snprintf(line, sizeof(line), "ERR time %u..%u ms\r\n",
                   (unsigned)RIG_PLT_MIN_RUN_MS, (unsigned)RIG_PLT_MAX_RUN_MS);
    reply(line);
    return;
  }
  plt_move_active = 0U;                  /* 手动优先，中止角度运动 */
  platform_timed_start((uint32_t)ms, (uint8_t)((value > 0) ? 1U : 0U));
  (void)snprintf(line, sizeof(line),
                 "PLT timed %d permille for %ld ms\r\n",
                 (int)RIG_PLT_SPEED_PERMILLE, (long)ms);
  reply(line);
}
static void platform_timed_poll(uint32_t now_ms)
{
  if (plt_timed_active == 0U) return;
  if ((int32_t)(now_ms - plt_timed_stop_ms) < 0) return;
  plt_timed_active = 0U;
  platform_apply_permille(0);
  reply("PLT timed done; angle ref lost, PS or Z to re-zero\r\n");
}

/* ------------------------- 机械臂 / 夹爪（U5 舵机，接法见 servo_u5.h） ------------------------- */
/* 角度 -> 脉宽（位置型舵机的机械可用范围端点）。`A` 命令和 PICK 序列共用这一份映射。 */
static uint16_t arm_angle_to_pulse_us(int32_t deg)
{
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  return (uint16_t)((int32_t)RIG_SERVO_POS_MIN_US +
                    (deg * ((int32_t)RIG_SERVO_POS_MAX_US - (int32_t)RIG_SERVO_POS_MIN_US) + 90) / 180);
}
static void arm_apply_pulse(uint16_t pulse_us)
{
  if (Servo_SetPulseUs(pulse_us) != HAL_OK) { reply("ERR servo pulse\r\n"); return; }
  servo_pulse_us[0] = pulse_us;
  servo_on[0] = 1U;
}
/* mode: 0 = 跑完整流程（夹紧→保持→渐退松开）；1 = 只夹紧保持；2 = 直接松开。 */
static void grip_start(uint8_t mode)
{
  char line[112];
  if (mode == 2U) {
    grip_state = GRIP_IDLE;
    grip_pulse_us = (uint16_t)RIG_GRIP_OPEN_US;
    arm_apply_pulse(grip_pulse_us);
    (void)snprintf(line, sizeof(line), "GRIP open at %u us\r\n", (unsigned)grip_pulse_us);
    reply(line);
    return;
  }
  grip_pulse_us = (uint16_t)(RIG_GRIP_CLOSE_US + RIG_GRIP_OVERSHOOT_US);
  arm_apply_pulse(grip_pulse_us);
  if (mode == 1U) {
    grip_state = GRIP_IDLE;   /* 保持夹紧，等 GRIP O 或 STOP */
    (void)snprintf(line, sizeof(line),
                   "GRIP hold closed at %u us (=%u+%u). GRIP O to open.\r\n",
                   (unsigned)grip_pulse_us, (unsigned)RIG_GRIP_CLOSE_US,
                   (unsigned)RIG_GRIP_OVERSHOOT_US);
    reply(line);
    return;
  }
  grip_state = GRIP_HOLDING;
  grip_next_ms = Board_NowMs() + (uint32_t)RIG_GRIP_HOLD_MS;
  (void)snprintf(line, sizeof(line),
                 "GRIP close %u us (=%u+%u), hold %u ms then open\r\n",
                 (unsigned)grip_pulse_us, (unsigned)RIG_GRIP_CLOSE_US,
                 (unsigned)RIG_GRIP_OVERSHOOT_US, (unsigned)RIG_GRIP_HOLD_MS);
  reply(line);
}
static void grip_poll(uint32_t now_ms)
{
  char line[64];
  if (grip_state == GRIP_IDLE) return;
  if ((int32_t)(now_ms - grip_next_ms) < 0) return;

  if (grip_state == GRIP_HOLDING) {
    grip_state = GRIP_OPENING;
    reply("GRIP hold done; opening\r\n");
  }
  grip_next_ms = now_ms + (uint32_t)RIG_GRIP_STEP_MS;

  if (grip_pulse_us > (uint16_t)(RIG_GRIP_OPEN_US + RIG_GRIP_STEP_US)) {
    grip_pulse_us = (uint16_t)(grip_pulse_us - RIG_GRIP_STEP_US);
  } else {
    grip_pulse_us = (uint16_t)RIG_GRIP_OPEN_US;
  }
  arm_apply_pulse(grip_pulse_us);
  (void)snprintf(line, sizeof(line), "pulse=%u us\r\n", (unsigned)grip_pulse_us);
  reply(line);
  if (grip_pulse_us == (uint16_t)RIG_GRIP_OPEN_US) {
    grip_state = GRIP_IDLE;
    reply("GRIP open; done\r\n");
  }
}
static void grip_command(const char *arg)
{
  while (*arg == ' ' || *arg == '\t') ++arg;
  if (*arg == '\0' || *arg == 'R') { grip_start(0U); return; }
  if (*arg == 'H') { grip_start(1U); return; }
  if (*arg == 'O') { grip_start(2U); return; }
  reply("ERR use GRIP | GRIP H (hold closed) | GRIP O (open)\r\n");
}

/* ============================== 一键抓取序列（PICK） ==============================
 * 一条命令跑完一轮，单次执行：
 *   夹爪 A45（开）→ 等 500 ms → 升降台向下 LJ14000 → 等停 →
 *   夹爪 A73（夹）→ 等 500 ms → 升降台向上 LJ14000 → 等停 →
 *   旋转台顺时针 RJ1200 → 等停 → 夹爪 A45（松开）→ 等 500 ms →
 *   旋转台逆时针 RJ1200 + 载物台 PM420（同时开始）→ 等两轴都停 → 完成
 *
 * 设计：
 *   - 每步只做一件事，用 axis_moving()（Axis.remaining）和 plt_timed_active 判断"到位"；
 *   - 时序参数全部在文件顶部 RIG_SEQ_* 区，改动作不用动这个状态机；
 *   - 序列开始时自动使能两轴，结束后保持使能（不自动释放，避免带载掉落）；
 *   - `STOP` 或任何手动运动命令都会中止序列（手动优先）；
 *   - 每步有 RIG_SEQ_STEP_TIMEOUT_MS 超时，轴不动/驱动器报警时不会卡死。 */

static uint8_t seq_running(void)
{
  return (seq_state != SEQ_IDLE) ? 1U : 0U;
}

/* 给某个轴排一次点动：等价于 LD + RV + LJ，但不回显（序列自己在步骤开始时报一行）。 */
static void seq_jog(uint8_t index, uint16_t period_us, uint8_t forward, int32_t steps)
{
  Axis *axis = &axes[index];
  axis->continuous = 0U;
  axis->period_us = period_us;
  axis->forward = forward;
  axis->remaining = steps;
  axis->moving_reported = 1U;
  Stepper_Direction(axis_stepper[index], forward);
}

static void seq_enter(SeqState next, uint32_t now_ms)
{
  char line[96];

  seq_state = next;
  seq_step_start_ms = now_ms;

  switch (next) {
  case SEQ_GRIP_OPEN:
  case SEQ_GRIP_OPEN2:
    (void)snprintf(line, sizeof(line), "SEQ: gripper -> %d deg (open)\r\n",
                   (int)RIG_SEQ_GRIP_OPEN_DEG);
    reply(line);                       /* 先回显再动，避免运动期间回显排队打乱顺序 */
    arm_apply_pulse(arm_angle_to_pulse_us(RIG_SEQ_GRIP_OPEN_DEG));
    seq_deadline_ms = now_ms + (uint32_t)RIG_SEQ_SERVO_SETTLE_MS;
    break;
  case SEQ_GRIP_CLOSE:
    (void)snprintf(line, sizeof(line), "SEQ: gripper -> %d deg (close)\r\n",
                   (int)RIG_SEQ_GRIP_CLOSE_DEG);
    reply(line);
    arm_apply_pulse(arm_angle_to_pulse_us(RIG_SEQ_GRIP_CLOSE_DEG));
    seq_deadline_ms = now_ms + (uint32_t)RIG_SEQ_SERVO_SETTLE_MS;
    break;
  case SEQ_LOWER:
    (void)snprintf(line, sizeof(line), "SEQ: lift down %ld steps @ %u us\r\n",
                   (long)RIG_SEQ_LIFT_JOG_STEPS, (unsigned)RIG_SEQ_LIFT_PERIOD_US);
    reply(line);
    seq_jog(0U, (uint16_t)RIG_SEQ_LIFT_PERIOD_US, (uint8_t)RIG_SEQ_LIFT_DIR_DOWN,
            RIG_SEQ_LIFT_JOG_STEPS);
    break;
  case SEQ_RAISE:
    (void)snprintf(line, sizeof(line), "SEQ: lift up %ld steps @ %u us\r\n",
                   (long)RIG_SEQ_LIFT_JOG_STEPS, (unsigned)RIG_SEQ_LIFT_PERIOD_US);
    reply(line);
    seq_jog(0U, (uint16_t)RIG_SEQ_LIFT_PERIOD_US, (uint8_t)RIG_SEQ_LIFT_DIR_UP,
            RIG_SEQ_LIFT_JOG_STEPS);
    break;
  case SEQ_ROT_CW:
    (void)snprintf(line, sizeof(line), "SEQ: rotate CW %ld steps @ %u us\r\n",
                   (long)RIG_SEQ_ROT_JOG_STEPS, (unsigned)RIG_SEQ_ROT_PERIOD_US);
    reply(line);
    seq_jog(1U, (uint16_t)RIG_SEQ_ROT_PERIOD_US, (uint8_t)RIG_SEQ_ROT_DIR_CW,
            RIG_SEQ_ROT_JOG_STEPS);
    break;
  case SEQ_RETURN:
    (void)snprintf(line, sizeof(line), "SEQ: rotate CCW %ld steps + platform %u ms\r\n",
                   (long)RIG_SEQ_ROT_JOG_STEPS, (unsigned)RIG_SEQ_PLATFORM_MS);
    reply(line);
    seq_jog(1U, (uint16_t)RIG_SEQ_ROT_PERIOD_US, (uint8_t)RIG_SEQ_ROT_DIR_CCW,
            RIG_SEQ_ROT_JOG_STEPS);
    platform_timed_start((uint32_t)RIG_SEQ_PLATFORM_MS, 1U);   /* 与 RJ 同时开始 */
    break;
  case SEQ_DONE:
    reply("SEQ: done (one pick cycle)\r\n");
    break;
  default:
    break;
  }
}

static void sequence_start(void)
{
  if (seq_state != SEQ_IDLE) {
    reply("SEQ already running; send STOP to abort\r\n");
    return;
  }
  /* 自动使能两轴；清一次故障标志，让 ALM 边沿能重新报。 */
  axes[0].fault_reported = 0U;
  axes[1].fault_reported = 0U;
  if (axes[0].enabled == 0U) { Stepper_Enable(STEPPER_1, 1U); axes[0].enabled = 1U; }
  if (axes[1].enabled == 0U) { Stepper_Enable(STEPPER_2, 1U); axes[1].enabled = 1U; }
  /* 开跑前清场：夹爪流程、载物台角度/定时段、四路电机序列都先停掉，免得和序列打架。 */
  grip_state = GRIP_IDLE;
  if (plt_move_active != 0U || plt_timed_active != 0U) {
    plt_move_active = 0U;
    plt_timed_active = 0U;
    platform_apply_permille(0);
  }
  motor_stop_all();
  reply("SEQ: start one pick cycle (STOP aborts)\r\n");
  seq_enter(SEQ_GRIP_OPEN, Board_NowMs());
}

static void sequence_abort(const char *why)
{
  char line[96];
  if (seq_state == SEQ_IDLE) return;
  seq_state = SEQ_IDLE;
  axes[0].continuous = 0U;
  axes[0].remaining = 0;
  axes[1].continuous = 0U;
  axes[1].remaining = 0;
  if (plt_timed_active != 0U) {
    plt_timed_active = 0U;
    platform_apply_permille(0);
  }
  (void)snprintf(line, sizeof(line), "SEQ aborted: %s\r\n", why);
  reply(line);
}

static void sequence_poll(uint32_t now_ms)
{
  if (seq_state == SEQ_IDLE) return;

  if (axes[0].alm_on != 0U && Stepper_Fault(STEPPER_1) != 0U) {
    sequence_abort("LIFT driver alarm");
    return;
  }
  if (axes[1].alm_on != 0U && Stepper_Fault(STEPPER_2) != 0U) {
    sequence_abort("ROT driver alarm");
    return;
  }

  if ((int32_t)(now_ms - seq_step_start_ms) > (int32_t)RIG_SEQ_STEP_TIMEOUT_MS) {
    sequence_abort("step timeout");
    return;
  }

  switch (seq_state) {
  case SEQ_GRIP_OPEN:
    if ((int32_t)(now_ms - seq_deadline_ms) < 0) return;
    seq_enter(SEQ_LOWER, now_ms);
    return;
  case SEQ_LOWER:
    if (axis_moving(&axes[0]) != 0U) return;
    seq_enter(SEQ_GRIP_CLOSE, now_ms);
    return;
  case SEQ_GRIP_CLOSE:
    if ((int32_t)(now_ms - seq_deadline_ms) < 0) return;
    seq_enter(SEQ_RAISE, now_ms);
    return;
  case SEQ_RAISE:
    if (axis_moving(&axes[0]) != 0U) return;
    seq_enter(SEQ_ROT_CW, now_ms);
    return;
  case SEQ_ROT_CW:
    if (axis_moving(&axes[1]) != 0U) return;
    seq_enter(SEQ_GRIP_OPEN2, now_ms);
    return;
  case SEQ_GRIP_OPEN2:
    if ((int32_t)(now_ms - seq_deadline_ms) < 0) return;
    seq_enter(SEQ_RETURN, now_ms);
    return;
  case SEQ_RETURN:
    if (axis_moving(&axes[1]) != 0U) return;
    if (plt_timed_active != 0U) return;          /* 等载物台也停 */
    seq_enter(SEQ_DONE, now_ms);
    return;
  case SEQ_DONE:
    seq_state = SEQ_IDLE;
    return;
  default:
    seq_state = SEQ_IDLE;
    return;
  }
}

static void print_help(void)
{
  reply("? help | S status | H hw regs | STOP | Z zero pos | T 0|1 stream\r\n");
  reply("LE 0|1  RE 0|1   enable LIFT(U1) / ROT(U4); EN is active LOW\r\n");
  reply("LV us   RV us    pulse period 20..20000 us (default 2000)\r\n");
  reply("LD 0|1  RD 0|1   direction\r\n");
  reply("LJ [n]  RJ [n]   jog n steps (default 20)\r\n");
  reply("LC 0|1  RC 0|1   continuous run\r\n");
  reply("A <deg> | AU <us> | AO   arm U5/" ARM_PIN_NAME ": positional, 0..180 deg -> 1000..2000 us\r\n");
  reply("P <permille> | PU <us> | PO  platform U6/PE6: CONTINUOUS speed, -1000..1000, 0=stop\r\n");
  reply("PT <deg> | PA <deg> | PS <deg>  platform: turn by / turn to / set angle\r\n");
  reply("PM <ms>          platform: fixed speed for ms (+/- = direction), no angle calc\r\n");
  reply("PICK             one full pick cycle (open-lower-close-raise-rotate-release-return)\r\n");
  reply("GRIP | GRIP H | GRIP O   gripper: close-hold-open / hold closed / open\r\n");
  reply("ALM 0|1          honor driver ALM input (default 0 = ignored; PD14/PD15 not wired)\r\n");
  motor_print_help();
  reply("Moves are non-blocking; replies are deferred while moving.\r\n");
  reply("PC0 LED blinks = firmware alive (heartbeat; NOT the power LED).\r\n");
}

/* H：把定时器的关键寄存器原样打出来，用来证明"固件到底有没有把 PWM 发出去"。
 *   机械臂 U5：PA7 方案看 TIM14/CCR1，PE5 方案看 TIM9/CCR1（由 servo_u5.h 的开关决定）。
 *   载物台 U6：始终是 TIM9/CCR2。周期 20000 = 20 ms = 50 Hz，CCR 单位 1 us。
 *   TIM1 = 四路电机：CEN/MOE 和四个 CCR（周期 8400 = 20 kHz）。
 *   另外读引脚 MODER/AFR，区分"通道在输出但引脚没驱动"和"引脚没配成 AF"。 */
static void print_hardware(void)
{
  char line[128];
  (void)snprintf(line, sizeof(line), "H SERVO TIM12 CEN=%u CCER=0x%lX CCR1=%lu CCR2=%lu\r\n",
                 (unsigned)((TIM12->CR1 & TIM_CR1_CEN) ? 1U : 0U), (unsigned long)TIM12->CCER,
                 (unsigned long)TIM12->CCR1, (unsigned long)TIM12->CCR2);
  reply(line);
  /* 引脚本身的状态：MODER 该位 = 2 才是复用功能；AFR 该半字节 = 9 才是 AF9(TIM12)。
   * 用于区分"通道在输出但引脚没被驱动"和"引脚根本没配成 AF"。 */
  (void)snprintf(line, sizeof(line), "H SERVO PINS PB14 MODER=%lu AFR=0x%lX | PB15 MODER=%lu AFR=0x%lX\r\n",
                 (unsigned long)((GPIOB->MODER >> (14U * 2U)) & 3UL),
                 (unsigned long)((GPIOB->AFR[1] >> ((14U - 8U) * 4U)) & 0xFUL),
                 (unsigned long)((GPIOB->MODER >> (15U * 2U)) & 3UL),
                 (unsigned long)((GPIOB->AFR[1] >> ((15U - 8U) * 4U)) & 0xFUL));
  reply(line);
  (void)snprintf(line, sizeof(line), "H MOTOR TIM1 CEN=%u MOE=%u CCR=%lu,%lu,%lu,%lu\r\n",
                 (unsigned)((TIM1->CR1 & TIM_CR1_CEN) ? 1U : 0U),
                 (unsigned)((TIM1->BDTR & TIM_BDTR_MOE) ? 1U : 0U),
                 (unsigned long)TIM1->CCR1, (unsigned long)TIM1->CCR2,
                 (unsigned long)TIM1->CCR3, (unsigned long)TIM1->CCR4);
  reply(line);
  (void)snprintf(line, sizeof(line), "H MOTOR AT8236: no STBY (PD2/PD3 not used)\r\n");
  reply(line);
}

static void print_status(void)
{
  char line[240];
  uint8_t i;
  for (i = 0U; i < 2U; ++i) {
    (void)snprintf(line, sizeof(line),
                   "%s en=%u dir=%u period=%uus moving=%u rem=%ld pos=%ld alm=%u almen=%u\r\n",
                   axis_label[i], (unsigned)axes[i].enabled, (unsigned)axes[i].forward,
                   (unsigned)axes[i].period_us, (unsigned)axis_moving(&axes[i]),
                   (long)axes[i].remaining, (long)axes[i].position_steps,
                   (unsigned)Stepper_Fault(axis_stepper[i]), (unsigned)axes[i].alm_on);
    reply(line);
  }
  {
    int32_t plt_speed = ((int32_t)servo_pulse_us[1] - (int32_t)RIG_SERVO_NEUTRAL_US) * 1000 /
                        (int32_t)RIG_SERVO_SPAN_US;
    (void)snprintf(line, sizeof(line),
                   "%s %s on=%u pulse=%uus grip=%u | PLT(U6/PE6) %s on=%u pulse=%uus speed=%ld/1000 ang=%ld turn=%u | stream=%u\r\n",
                   ARM_LABEL,
                   (RIG_ARM_CONTINUOUS != 0U) ? "cont" : "pos",
                   (unsigned)servo_on[0], (unsigned)servo_pulse_us[0], (unsigned)grip_state,
                   (RIG_PLT_CONTINUOUS != 0U) ? "cont" : "pos",
                   (unsigned)servo_on[1], (unsigned)servo_pulse_us[1],
                   (long)plt_speed, (long)plt_angle_deg, (unsigned)plt_move_active,
                   (unsigned)telemetry_enabled);
  }
  reply(line);
  motor_print_status();
  if (seq_state != SEQ_IDLE) {
    (void)snprintf(line, sizeof(line), "SEQ state=%u running (PICK); STOP to abort\r\n",
                   (unsigned)seq_state);
    reply(line);
  }
}

static void axis_command(uint8_t index, char sub, const char *arg)
{
  Axis *axis = &axes[index];
  StepperId id = axis_stepper[index];
  int32_t value = 0;
  char line[96];

  if (sub == 'E') {
    /* 运动中不许使能/释放：ZDT 在脉冲中途掉使能会丢步，带载释放会直接掉平台。 */
    if (axis_moving(axis) != 0U) { reply("ERR axis is moving; send STOP first\r\n"); return; }
    if (parse_arg(arg, &value) == 0U || (value != 0 && value != 1)) {
      reply("ERR use LE 0|1 / RE 0|1\r\n");
      return;
    }
    if (value != 0) {
      axis->fault_reported = 0U;
      Stepper_Enable(id, 1U);
      axis->enabled = 1U;
      (void)snprintf(line, sizeof(line), "%s enabled\r\n", axis_label[index]);
    } else {
      axis->continuous = 0U;
      axis->remaining = 0;
      Stepper_Enable(id, 0U);
      axis->enabled = 0U;
      (void)snprintf(line, sizeof(line), "%s released (hold the load first)\r\n",
                     axis_label[index]);
    }
    reply(line);
    return;
  }

  if (sub == 'V') {
    if (parse_arg(arg, &value) == 0U || value < (int32_t)RIG_PULSE_PERIOD_MIN_US ||
        value > (int32_t)RIG_PULSE_PERIOD_MAX_US) {
      (void)snprintf(line, sizeof(line), "ERR period %u..%u us\r\n",
                     (unsigned)RIG_PULSE_PERIOD_MIN_US, (unsigned)RIG_PULSE_PERIOD_MAX_US);
      reply(line);
      return;
    }
    axis->period_us = (uint16_t)value;
    (void)snprintf(line, sizeof(line), "%s period=%ld us\r\n", axis_label[index], (long)value);
    reply(line);
    return;
  }

  if (sub == 'D') {
    /* 脉冲中途换向会让驱动器丢步/报警，所以要求先停。 */
    if (axis_moving(axis) != 0U) { reply("ERR axis is moving; send STOP first\r\n"); return; }
    if (parse_arg(arg, &value) == 0U || (value != 0 && value != 1)) {
      reply("ERR use LD 0|1 / RD 0|1\r\n");
      return;
    }
    axis->forward = (uint8_t)value;
    Stepper_Direction(id, axis->forward);
    (void)snprintf(line, sizeof(line), "%s dir=%ld\r\n", axis_label[index], (long)value);
    reply(line);
    return;
  }

  if (sub == 'J') {
    if (*arg == '\0') value = RIG_JOG_DEF_STEPS;
    else if (parse_arg(arg, &value) == 0U) { reply("ERR jog steps\r\n"); return; }
    if (value < 1 || value > RIG_JOG_MAX_STEPS) { reply("ERR jog steps out of range\r\n"); return; }
    if (axis->enabled == 0U) { reply("ERR axis disabled; send LE 1 / RE 1 first\r\n"); return; }
    axis->continuous = 0U;
    axis->remaining = value;
    axis->moving_reported = 1U;
    (void)snprintf(line, sizeof(line), "%s jog %ld steps\r\n", axis_label[index], (long)value);
    reply(line);
    return;
  }

  if (sub == 'C') {
    if (parse_arg(arg, &value) == 0U || (value != 0 && value != 1)) {
      reply("ERR use LC 0|1 / RC 0|1\r\n");
      return;
    }
    if (value != 0 && axis->enabled == 0U) {
      reply("ERR axis disabled; send LE 1 / RE 1 first\r\n");
      return;
    }
    axis->continuous = (uint8_t)value;
    axis->remaining = 0;
    axis->moving_reported = (uint8_t)value;
    (void)snprintf(line, sizeof(line), "%s continuous=%ld\r\n", axis_label[index], (long)value);
    reply(line);
    return;
  }

  reply("ERR unknown axis command\r\n");
}

static void servo_command(uint8_t index, const char *arg)
{
  const char *label = (index == 0U) ? ARM_LABEL : "PLT(U6/PE6)";
  int32_t value = 0;
  char line[96];

  /* 手动给载物台命令（P / PU / PO）时，中止 PM 的定时段，否则时间到会把它改回 1500。 */
  if (index == 1U) plt_timed_active = 0U;

  /* 手动给机械臂命令（A / AU / AO）时，中止正在跑的 GRIP 流程，手动优先。 */
  if (index == 0U) grip_state = GRIP_IDLE;

  if (arg[0] == 'O' && arg[1] == '\0') {
    if (index == 0U) Servo_Disable(); else Servo2_Disable();
    servo_on[index] = 0U;
    (void)snprintf(line, sizeof(line), "%s off\r\n", label);
    reply(line);
    return;
  }

  if (arg[0] == 'U') {
    if (parse_arg(arg + 1, &value) == 0U || value < (int32_t)RIG_SERVO_MIN_US ||
        value > (int32_t)RIG_SERVO_MAX_US) {
      (void)snprintf(line, sizeof(line), "ERR pulse %u..%u us\r\n",
                     (unsigned)RIG_SERVO_MIN_US, (unsigned)RIG_SERVO_MAX_US);
      reply(line);
      return;
    }
  } else {
    uint8_t continuous = (index == 0U) ? (uint8_t)RIG_ARM_CONTINUOUS : (uint8_t)RIG_PLT_CONTINUOUS;
    if (parse_arg(arg, &value) == 0U) { reply("ERR missing value\r\n"); return; }
    if (continuous != 0U) {
      /* 连续旋转型：千分比 = 速度，0 = 停；走统一的脉宽映射（含方向修正与死区）。
       * 手动给速度会中止进行中的角度运动。 */
      if (value < -1000 || value > 1000) {
        reply("ERR speed -1000..1000 permille (0 = stop)\r\n");
        return;
      }
      plt_move_active = 0U;
      value = (int32_t)platform_pulse_from_permille(value);
    } else {
      /* 位置舵机：0..180 度映射到机械可用范围（默认 1000..2000 us，与老 Servo 台架一致）。 */
      int32_t deg = value;
      if (deg < 0 || deg > 180) {
        reply("ERR angle 0..180 deg\r\n");
        return;
      }
      value = (int32_t)arm_angle_to_pulse_us(deg);
    }
  }

  if ((index == 0U ? Servo_SetPulseUs((uint16_t)value) : Servo2_SetPulseUs((uint16_t)value)) != HAL_OK) {
    reply("ERR servo pulse\r\n");
    return;
  }
  servo_pulse_us[index] = (uint16_t)value;
  servo_on[index] = 1U;
  (void)snprintf(line, sizeof(line), "%s pulse=%ld us\r\n", label, (long)value);
  reply(line);
}

static void run_command(void)
{
  char *text = (char *)command_line;
  size_t length = strlen(text);
  size_t index;

  if (command_overflow != 0U) { reply("ERR command too long\r\n"); return; }
  while (length != 0U && (text[length - 1U] == ' ' || text[length - 1U] == '\t'))
    text[--length] = '\0';
  for (index = 0U; index < length; ++index)
    if (text[index] >= 'a' && text[index] <= 'z')
      text[index] = (char)(text[index] - 'a' + 'A');
  if (length == 0U) return;

  if (strcmp(text, "?") == 0 || strcmp(text, "HELP") == 0) { print_help(); return; }
  if (strcmp(text, "S") == 0 || strcmp(text, "STATUS") == 0) { print_status(); return; }
  if (strcmp(text, "H") == 0) { print_hardware(); return; }
  if (strcmp(text, "STOP") == 0) {
    sequence_abort("STOP");
    stop_axes();
    if (plt_move_active != 0U || plt_timed_active != 0U) {
      plt_move_active = 0U;
      plt_timed_active = 0U;
      platform_apply_permille(0);
    }
    motor_stop_all();            /* 四路底盘电机一起停，并中止 MR 序列 */
    reply("stopped: axes halted (still enabled), motors off\r\n");
    return;
  }
  if (strcmp(text, "PICK") == 0) { sequence_start(); return; }
  /* 序列运行中，任何其它命令都按"手动优先"处理：先中止序列，再执行该命令。 */
  if (seq_running() != 0U) { sequence_abort("manual command"); }
  if (strcmp(text, "Z") == 0) {
    /* 位置清零：pos 是累计步数，标定与回零对比时需要它当参考。 */
    axes[0].position_steps = 0;
    axes[1].position_steps = 0;
    plt_angle_deg = 0;
    plt_target_deg = 0;
    plt_move_active = 0U;
    plt_timed_active = 0U;
    reply("pos and platform angle zeroed\r\n");
    return;
  }
  if (strncmp(text, "GRIP", 4) == 0) { grip_command(text + 4); return; }
  if (text[0] == 'T') {
    int32_t value = 0;
    if (parse_arg(text + 1, &value) == 0U || (value != 0 && value != 1)) {
      reply("ERR use T 0|1\r\n");
      return;
    }
    telemetry_enabled = (uint8_t)value;
    next_tx_ms = Board_NowMs();
    reply((value != 0) ? "stream on\r\n" : "stream off\r\n");
    return;
  }
  if (strncmp(text, "ALM", 3) == 0) {
    int32_t value = 0;
    if (parse_arg(text + 3, &value) == 0U || (value != 0 && value != 1)) {
      reply("ERR use ALM 0|1 (1 = honor the alarm input)\r\n");
      return;
    }
    axes[0].alm_on = (uint8_t)value;
    axes[1].alm_on = (uint8_t)value;
    reply((value != 0) ? "ALM honored: stops on low ALM input\r\n"
                       : "ALM ignored: input not wired\r\n");
    return;
  }
  if (text[0] == 'A') { servo_command(0U, text + 1); return; }
  if (text[0] == 'M') { motor_command(text + 1); return; }
  if (text[0] == 'P') {
    /* PT 相对转 / PA 绝对转 / PS 设定当前角度 / PM 固定速度+时间；
       其余（PO / PU / 千分比）交给舵机命令。 */
    if (text[1] == 'T' || text[1] == 'A' || text[1] == 'S') {
      platform_angle_command(text[1], text + 2);
      return;
    }
    if (text[1] == 'M') { platform_timed_command(text + 2); return; }
    servo_command(1U, text + 1);
    return;
  }
  if ((text[0] == 'L' || text[0] == 'R') &&
      (text[1] == 'E' || text[1] == 'V' || text[1] == 'D' ||
       text[1] == 'J' || text[1] == 'C')) {
    axis_command((text[0] == 'L') ? 0U : 1U, text[1], text + 2);
    return;
  }
  reply("ERR unknown command; send ?\r\n");
}

static void poll_commands(void)
{
  int received;
  while ((received = Serial_ReadByte(RIG_PORT)) >= 0) {
    char byte = (char)received;
    if (byte == '\r' || byte == '\n') {
      if (command_length != 0U || command_overflow != 0U) {
        command_line[command_length] = '\0';
        run_command();
      }
      command_length = 0U;
      command_overflow = 0U;
    } else if (byte < 32 || byte > 126) {
      command_overflow = 1U;
    } else if (command_length < RIG_CMD_LEN - 1U && command_overflow == 0U) {
      command_line[command_length++] = (uint8_t)byte;
    } else {
      command_overflow = 1U;
    }
  }
}

static void send_telemetry(void)
{
  char line[96];
  (void)snprintf(line, sizeof(line), "%ld,%ld,%ld,%ld,%u,%u\n",
                 (long)axes[0].position_steps, (long)axes[1].position_steps,
                 (long)axes[0].remaining, (long)axes[1].remaining,
                 (unsigned)servo_pulse_us[0], (unsigned)servo_pulse_us[1]);
  (void)Serial_Write(RIG_PORT, (const uint8_t *)line, (uint16_t)strlen(line), 50U);
}

/* ============================== 入口 ============================== */

void RigSmoke_Begin(void)
{
  uint8_t i;

  /* 上电初态：不发脉冲、两轴失能、两个舵机不出 PWM、四路电机输出 0。 */
  for (i = 0U; i < 2U; ++i) {
    axes[i].enabled = 0U;
    axes[i].forward = 1U;
    axes[i].continuous = 0U;
    axes[i].alm_on = (uint8_t)RIG_ALM_DEFAULT;
    axes[i].moving_reported = 0U;
    axes[i].fault_reported = 0U;
    axes[i].remaining = 0;
    axes[i].position_steps = 0;
    axes[i].period_us = (uint16_t)RIG_PULSE_PERIOD_DEF_US;
    Stepper_Enable(axis_stepper[i], 0U);
    Stepper_Direction(axis_stepper[i], 1U);
    servo_pulse_us[i] = (uint16_t)RIG_SERVO_DEF_US;
    servo_on[i] = 0U;
  }
  Servo_Disable();
  Servo2_Disable();
  motor_stop_all();             /* 四路输出 0，两片 TB6612 回待机 */
  Led_StatusInit();             /* 板载 PC0 心跳灯：先灭，翻转交给 Poll */
  Led_StatusSet(0U);

  command_length = 0U;
  command_overflow = 0U;
  plt_angle_deg = 0;
  plt_target_deg = 0;
  plt_move_active = 0U;
  plt_move_stop_ms = 0U;
  plt_timed_active = 0U;
  plt_timed_stop_ms = 0U;
  seq_state = SEQ_IDLE;
  seq_step_start_ms = 0U;
  seq_deadline_ms = 0U;
  grip_state = GRIP_IDLE;
  grip_pulse_us = (uint16_t)RIG_GRIP_OPEN_US;
  grip_next_ms = 0U;
  deferred_head = 0U;
  deferred_count = 0U;
  deferred_dropped = 0U;
  telemetry_enabled = 0U;
  next_tx_ms = Board_NowMs();
  led_toggle_due_ms = Board_NowMs();

  send_now("RIG ready: LIFT=U1(PD0/PD1) ROT=U4(PD4/PD5) ARM=U5/" ARM_PIN_NAME " PLT=U6/PE6\r\n"
           "MOTOR=A/B/C/D off (TB6612 U2/U3 standby); axes DISABLED. Send ? for help.\r\n"
           "LED=PC0 blinking = firmware alive (that is NOT the power LED).\r\n");
  print_status();
}

void RigSmoke_Poll(void)
{
  uint32_t now_ms;

  axis_service(&axes[0], STEPPER_1);
  axis_service(&axes[1], STEPPER_2);
  poll_commands();
  now_ms = Board_NowMs();
  platform_poll(now_ms);        /* 载物台"按角度转"的到点停：不依赖遥测开关 */
  platform_timed_poll(now_ms);  /* PM 定时段的到点停：同上 */
  grip_poll(now_ms);            /* 夹爪流程的到点推进：同样不依赖遥测开关 */
  motor_sequence_poll(now_ms);  /* 四路电机的 12 段序列 + 硬超时：同样不依赖遥测开关 */
  led_heartbeat_poll(now_ms);   /* PC0 心跳灯：滞留在阻塞调用里时它就不翻，是最快的"死了"判据 */
  report_changes();
  sequence_poll(now_ms);        /* PICK 一键抓取序列：放在 report_changes 之后，先报"轴停"再报下一步 */
  flush_deferred();

  if (telemetry_enabled == 0U) return;
  if ((int32_t)(now_ms - next_tx_ms) < 0) return;
  next_tx_ms = now_ms + (1000U / RIG_TX_HZ);
  send_telemetry();
}
