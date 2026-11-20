#ifndef PICK_CONFIG_H
#define PICK_CONFIG_H

/* 机构参数区：升降台、旋转台、两路舵机，以及 CYC 任务循环。
 *
 * 全部开环：升降台没有限位、旋转台没有编码器，所以这里的数都是"指令值"，
 * 不是实测值。改完必须重新编译；第一次上板请先小步数试方向（见 Docs/机构指令_升降旋转舵机.md）。
 *
 * 单位后缀按 AGENTS.md 第 3 节：_DEG / _US / _HZ / _MS / _PERMILLE / _PULSES。 */

/* ---- 舵机公共 ---- */
#define PK_SERVO_MIN_US             500U   /* Servo_SetPulseUs() 接受的软件下限 */
#define PK_SERVO_MAX_US            2500U   /* 上限 */

/* ---- U5 夹爪 / 机械臂（PB14 / TIM12_CH1）：位置舵机 ---- */
#define PK_ARM_DEG_MIN                0    /* 机械可用范围起点 */
#define PK_ARM_DEG_MAX              180    /* 机械可用范围终点 */
#define PK_ARM_US_MIN              1000U   /* 0 deg 对应脉宽 */
#define PK_ARM_US_MAX              2000U   /* 180 deg 对应脉宽 */

/* ---- U6 载物台（PB15 / TIM12_CH2）：连续旋转型舵机 ---- */
#define PK_PLAT_STOP_US            1500U   /* 停转脉宽 */
#define PK_PLAT_SPAN_US             500U   /* |permille| = 1000 时的偏置量：1500 ± 500 */
#define PK_PLAT_DEADBAND_PERMILLE    20    /* 小于这个量级直接给停止脉宽，避免嗡嗡抖 */
#define PK_PLAT_SIGN                  1    /* 方向反了改 -1，不要动接线 */
#define PK_PLAT_MAX_MS            10000U   /* PLT 定时点动允许的最长时长 */
#define PK_PLAT_JOG_PERMILLE        400    /* PLT 定时点动的固定速度：40%；命令只给时间 */

/* ---- 旋转台（PD12 / TIM4_CH1）---- */
#define PK_ROT_PULSES_PER_REV      3200U   /* 16 细分 × 200 步/转，平台 1:1；实测后标定 */
#define PK_ROT_HZ                  2000U   /* ROT <deg> 默认走速，可在命令第二个参数上改 */
#define PK_ROT_MAX_DEG              360    /* 单条命令允许的最大角度 */

/* ---- 升降台（PA6 / TIM13_CH1）----
 * LIFT 是"点动距离"：给**脉冲个数**，不是频率、也不是时间。
 * 脉冲 → 距离的换算要先量丝杆导程：导程 mm/转 ÷ PK_LIFT_PULSES_PER_REV 就是 mm/脉冲。
 * 3200 = 16 细分 × 200 步/转；换细分或换丝杆要重标。 */
#define PK_LIFT_PULSES_PER_REV     3200U   /* 一圈多少脉冲（标定用，暂未参与计算） */
#define PK_LIFT_HZ                 30000U   /* LIFT 默认走速（2026-11-17 +150%: 12000→30000）*/
                                            /* Step6 硬上限 40000；再高看负载是否丢步 */
#define PK_LIFT_MAX_PULSES       200000U   /* 单条命令允许的最大脉冲数 */
#define PK_AXIS_TIMEOUT_MS        30000U   /* 单轴点动超时，防"永远不动却一直算" */

/* ---- 两轴共用的三段加减速 ----
 * 提速必须配斜坡：STEP/DIR 模式下驱动器不插补，2000 → 8000 Hz 的阶跃会让升降台跟丢或报过载。
 * 起转 PK_AXIS_RAMP_PERMILLE → 恒速 → 收转同值，两头各 PK_AXIS_RAMP_MS；斜坡不改变总脉冲数。
 * 行程太短（两端斜坡的脉冲都凑不出来）时自动退化成单速，不硬套。 */
#define PK_AXIS_RAMP_MS             120U   /* 每段时长（再提速后同步加长，防起停丢步）*/
#define PK_AXIS_RAMP_PERMILLE       400U   /* 起转/收转占恒速的千分比 */

/* ---- 夹爪工位角（CYC / TRG / GRAB 共用）----
 * 本机构角度越大越闭：180 夹紧，130 松开。下降前用 APPROACH，
 * 放到盘上用 OPEN。两者现在一样；若下降时夹爪还会碰到物料，
 * 把 APPROACH 改小（更开），不要改 OPEN，否则放料时张得太大。 */
#define PK_GRIP_CLOSE_DEG          180     /* 夹紧 */
#define PK_GRIP_OPEN_DEG            30     /* 张开角（2026-11-17 实测：30° 为张开）*/
#define PK_GRIP_APPROACH_DEG        30     /* 下降前的张开角，跟张开角一致 */

/* 下降末段和夹紧后的抬离用慢速，避免 12000 Hz 直接撞上物料或把物料拽翻。
 * 这两个脉冲数必须小于所有"取料下降"和"取料位往复"的行程，否则快段会变成 0，
 * 而 Pick_LiftPulses(0) 的意思是停车，不是"这一步跳过"。 */
#define PK_GRIP_APPROACH_PULSES    1200U   /* 下降最后这一段改慢速 */
#define PK_GRIP_APPROACH_HZ        3000U
#define PK_GRIP_LIFTOFF_PULSES      800U   /* 夹紧后先慢抬这么多，再切回快升 */
#define PK_GRIP_LIFTOFF_HZ         3000U
#define PK_GRIP_CLOSE_SETTLE_MS     800U   /* 夹紧后等多久再升降。闭合要比松开久 */
#define PK_GRIP_OPEN_SETTLE_MS      400U   /* 张开后等多久再下降 / 转盘 */

/* ---- 任务循环 CYC（2026-11-16：开场先张开，下降末段和抬离放慢）----
 * 开场（只一次）  ：张开 → 快降 → 慢降到取料位。整段下降仍是 8000，只出现一次。
 * 每轮（跑 N 遍）  ：夹紧 → 慢抬离 → 快升到放料高度 → 转 -132° → 松开 →
 *                    载物盘走一格 → 转 +132° → 快降 → 慢降回取料位。
 * 收尾（只一次）  ：取料位 → 最高处，上升 8000。
 * 每轮结束时高度和朝向都与开始时一致，连跑多轮不会漂。 */
#define PK_CYC_LIFT_DOWN_PULSES     8000U   /* 最高处 ↔ 取料位（开场下降 / 收尾上升）*/
#define PK_CYC_LIFT_UP_PULSES       6000U   /* 取料位 ↔ 放料高度（每轮上下一趟）*/
/* 派生：最高处 → 旋转台放料位（夹爪松开的那一刻）= DOWN - UP = 2000。
 * 料从这里落稳到载物台上；TRG 的取料位（4000）比它低 2000，是料落稳后的位置。 */
#define PK_CYC_PLACE_DOWN_PULSES (PK_CYC_LIFT_DOWN_PULSES - PK_CYC_LIFT_UP_PULSES)
#define PK_CYC_LIFT_HZ        PK_LIFT_HZ    /* 升降速度：改这一个数就改整轮升降快慢（现 30000）*/
#define PK_CYC_ROT_DEG               132    /* 取料侧 ↔ 放料侧的旋转角 */
#define PK_CYC_ROT_HZ         PK_ROT_HZ     /* 旋转台速度 */
#define PK_CYC_PLAT_MS             413U     /* 载物盘走一格（每轮放完料后转一格，三轮占三格）*/
#define PK_CYC_ROUNDS_DEFAULT        3U     /* `CYC` 不带参数时跑几轮 */
#define PK_CYC_MAX_ROUNDS           20U     /* `CYC <n>` 上限 */
#define PK_CYC_ARM_SETTLE_MS        500U    /* 舵机下发后等它到位再走下一步 */
#define PK_CYC_STEP_TIMEOUT_MS    10000U    /* 单步超时，防轴不动时序列卡死 */

/* ---- 卸料序列 TRG（2026-11-11 按现场流程重写，含回位）----
 * 一轮动作：下降 4000 → 夹紧 → 上升 4000（回最高处）→ 转 +132° →
 *           下降 41000 → 松开 → 转 -132° → **上升 41000（回最高处）**。
 * 起点和终点都在丝杆最高处，所以能连着跑多轮、也能接着发 CYC。
 *
 * 高度已实测（2026-11-17）：顶部→地面 41000、顶部→载物台取料位 4000；
 * 顶部→物料台取料位 8000、顶部→旋转台放料位 2000（= 8000 - 6000）。
 * 第一次上板仍建议分步 `LIFT 20000` → `LIFT 30000` → `LIFT 41000` 确认行程。
 * 想换速度只改 `PK_TRG_LIFT_HZ`，行程不受影响。 */
#define PK_TRG_PICK_PULSES          4000U   /* 最高处 → 载物台取料位（下降）*/
#define PK_TRG_DROP_PULSES         41000U   /* 最高处 → 地面放料位（下降）
                                             * 2026-11-17 实测：顶部到地面全程 41000；
                                             * 旧值 37000 偏小 4000，会离地还有一段就松爪。 */
#define PK_TRG_LIFT_HZ        PK_CYC_LIFT_HZ  /* 跟升降默认速度走（现 30000 Hz） */
#define PK_TRG_ROT_DEG               132    /* 取料侧 ↔ 放料侧的旋转角 */
#define PK_TRG_ROUNDS_DEFAULT         1U    /* `TRG` 不带参数时跑几轮 */
#define PK_TRG_MAX_ROUNDS            20U    /* `TRG <n>` 上限 */

/* ---- 圆环放置 DROP（2026-11-17）----
 * 顺序：空爪转到放料侧 → 降到释放高度 → 视觉对准圆环中心（此步底盘动）→
 *       升回最高处 → 转回取料侧 → 降到取料位 → 夹料 → 升回最高处 →
 *       转到放料侧 → 降到释放高度 → 松开 → 抬离 → 回最高处 → 转回。
 * **对准高度 = 释放高度**：对准之后底盘不再动，ROT 的回程重复精度直接决定落点。 */
#define PK_DROP_RELEASE_PULSES    41000U   /* 最高处 → 释放/对准高度（同 41000）*/
#define PK_DROP_ALIGN_COARSE_MM      15    /* 顶部粗对准容差（宽视野，先把环拉到画面中间）*/
#define PK_DROP_ALIGN_TOL_MM          2    /* 底部精对准容差；1 环净空 1.5 mm，先 2 mm 打稳再压到 1 */
#define PK_DROP_ALIGN_TIMEOUT_MS  25000U   /* 对准阶段超时（含底盘微调）*/

#if (PK_SERVO_MIN_US < 100U || PK_SERVO_MAX_US > 3000U)
#error PK_SERVO_MIN_US/MAX_US out of range
#endif
#if (PK_ARM_DEG_MIN < -90 || PK_ARM_DEG_MAX > 270 || PK_ARM_DEG_MIN >= PK_ARM_DEG_MAX)
#error PK_ARM_DEG_MIN/MAX invalid
#endif
#if (PK_ARM_US_MIN < PK_SERVO_MIN_US || PK_ARM_US_MAX > PK_SERVO_MAX_US || \
     PK_ARM_US_MIN >= PK_ARM_US_MAX)
#error PK_ARM_US_MIN/MAX must stay inside 500..2500 and be ascending
#endif
#if (PK_PLAT_STOP_US < PK_SERVO_MIN_US || PK_PLAT_STOP_US > PK_SERVO_MAX_US)
#error PK_PLAT_STOP_US out of range
#endif
#if (PK_PLAT_SPAN_US < 50U || PK_PLAT_SPAN_US > 500U)
#error PK_PLAT_SPAN_US must be 50..500
#endif
#if ((PK_PLAT_STOP_US + PK_PLAT_SPAN_US) > PK_SERVO_MAX_US || \
     (PK_PLAT_STOP_US - PK_PLAT_SPAN_US) < PK_SERVO_MIN_US)
#error PK_PLAT_STOP_US +/- PK_PLAT_SPAN_US leaves the servo range
#endif
#if (PK_PLAT_DEADBAND_PERMILLE < 0 || PK_PLAT_DEADBAND_PERMILLE > 200)
#error PK_PLAT_DEADBAND_PERMILLE must be 0..200
#endif
#if (PK_PLAT_JOG_PERMILLE <= PK_PLAT_DEADBAND_PERMILLE || PK_PLAT_JOG_PERMILLE > 1000)
#error PK_PLAT_JOG_PERMILLE must be above the deadband and <= 1000
#endif
#if (PK_PLAT_SIGN != 1 && PK_PLAT_SIGN != -1)
#error PK_PLAT_SIGN must be 1 or -1
#endif
#if (PK_ROT_PULSES_PER_REV < 200U || PK_ROT_PULSES_PER_REV > 51200U)
#error PK_ROT_PULSES_PER_REV out of range
#endif
#if (PK_ROT_HZ < 100U || PK_ROT_HZ > 20000U)
#error PK_ROT_HZ out of range
#endif
#if (PK_ROT_MAX_DEG < 1 || PK_ROT_MAX_DEG > 360)
#error PK_ROT_MAX_DEG must be 1..360
#endif
#if (PK_LIFT_HZ < 100U || PK_LIFT_HZ > 40000U)
#error PK_LIFT_HZ must be 100..40000
#endif
#if (PK_LIFT_MAX_PULSES < 1U || PK_LIFT_MAX_PULSES > 1000000U)
#error PK_LIFT_MAX_PULSES must be 1..1000000
#endif
#if (PK_LIFT_PULSES_PER_REV < 200U || PK_LIFT_PULSES_PER_REV > 51200U)
#error PK_LIFT_PULSES_PER_REV out of range
#endif
#if (PK_PLAT_MAX_MS < 20U || PK_PLAT_MAX_MS > 60000U)
#error PK_PLAT_MAX_MS must be 20..60000
#endif
#if (PK_AXIS_TIMEOUT_MS < 1000U || PK_AXIS_TIMEOUT_MS > 300000U)
#error PK_AXIS_TIMEOUT_MS must be 1000..300000
#endif
#if (PK_AXIS_RAMP_MS < 10U || PK_AXIS_RAMP_MS > 500U)
#error PK_AXIS_RAMP_MS must be 10..500
#endif
#if (PK_AXIS_RAMP_PERMILLE < 100U || PK_AXIS_RAMP_PERMILLE > 900U)
#error PK_AXIS_RAMP_PERMILLE must be 100..900
#endif
#if (PK_GRIP_CLOSE_DEG < PK_ARM_DEG_MIN || PK_GRIP_CLOSE_DEG > PK_ARM_DEG_MAX || \
     PK_GRIP_OPEN_DEG < PK_ARM_DEG_MIN || PK_GRIP_OPEN_DEG > PK_ARM_DEG_MAX || \
     PK_GRIP_APPROACH_DEG < PK_ARM_DEG_MIN || PK_GRIP_APPROACH_DEG > PK_ARM_DEG_MAX)
#error PK_GRIP_CLOSE_DEG/OPEN_DEG/APPROACH_DEG outside the mechanical range
#endif
#if (PK_GRIP_APPROACH_PULSES < 1U || \
     PK_GRIP_APPROACH_PULSES >= PK_CYC_LIFT_UP_PULSES || \
     PK_GRIP_APPROACH_PULSES >= PK_CYC_LIFT_DOWN_PULSES || \
     PK_GRIP_APPROACH_PULSES >= PK_TRG_PICK_PULSES || \
     PK_GRIP_APPROACH_PULSES >= PK_TRG_DROP_PULSES)
#error PK_GRIP_APPROACH_PULSES must be shorter than every pick/drop stroke
#endif
#if (PK_GRIP_LIFTOFF_PULSES < 1U || \
     PK_GRIP_LIFTOFF_PULSES >= PK_CYC_LIFT_UP_PULSES || \
     PK_GRIP_LIFTOFF_PULSES >= PK_TRG_PICK_PULSES || \
     PK_GRIP_LIFTOFF_PULSES >= PK_TRG_DROP_PULSES)
#error PK_GRIP_LIFTOFF_PULSES must be shorter than every lift-off stroke
#endif
#if (PK_GRIP_APPROACH_HZ < 100U || PK_GRIP_APPROACH_HZ > PK_LIFT_HZ)
#error PK_GRIP_APPROACH_HZ must be 100..PK_LIFT_HZ
#endif
#if (PK_GRIP_LIFTOFF_HZ < 100U || PK_GRIP_LIFTOFF_HZ > PK_LIFT_HZ)
#error PK_GRIP_LIFTOFF_HZ must be 100..PK_LIFT_HZ
#endif
#if (PK_GRIP_CLOSE_SETTLE_MS < 100U || PK_GRIP_CLOSE_SETTLE_MS > 5000U)
#error PK_GRIP_CLOSE_SETTLE_MS must be 100..5000
#endif
#if (PK_GRIP_OPEN_SETTLE_MS < 100U || PK_GRIP_OPEN_SETTLE_MS > 5000U)
#error PK_GRIP_OPEN_SETTLE_MS must be 100..5000
#endif
#if (PK_CYC_LIFT_DOWN_PULSES < 1U || PK_CYC_LIFT_DOWN_PULSES > PK_LIFT_MAX_PULSES || \
     PK_CYC_LIFT_UP_PULSES < 1U || PK_CYC_LIFT_UP_PULSES > PK_LIFT_MAX_PULSES)
#error PK_CYC_LIFT_DOWN_PULSES/UP_PULSES out of range
#endif
/* GRAB/PLACE 的放料高度 = UP，最高处 = DOWN。放料点必须比最高处低，
 * 否则"从最高处再降 UP 到放料高度"会变成往上走（DOWN-UP 为负）。 */
#if (PK_CYC_LIFT_UP_PULSES >= PK_CYC_LIFT_DOWN_PULSES)
#error PK_CYC_LIFT_UP_PULSES must be smaller than PK_CYC_LIFT_DOWN_PULSES
#endif
#if (PK_CYC_LIFT_HZ < 100U || PK_CYC_LIFT_HZ > 40000U)
#error PK_CYC_LIFT_HZ out of range
#endif
#if (PK_CYC_ROT_DEG < 1 || PK_CYC_ROT_DEG > 360)
#error PK_CYC_ROT_DEG out of range
#endif
#if (PK_CYC_ROT_HZ < 100U || PK_CYC_ROT_HZ > 20000U)
#error PK_CYC_ROT_HZ out of range
#endif
#if (PK_CYC_PLAT_MS < 20U || PK_CYC_PLAT_MS > PK_PLAT_MAX_MS)
#error PK_CYC_PLAT_MS out of range
#endif
#if (PK_CYC_ROUNDS_DEFAULT < 1U || PK_CYC_ROUNDS_DEFAULT > PK_CYC_MAX_ROUNDS)
#error PK_CYC_ROUNDS_DEFAULT out of range
#endif
#if (PK_CYC_ARM_SETTLE_MS > 5000U)
#error PK_CYC_ARM_SETTLE_MS must be 0..5000
#endif
#if (PK_CYC_STEP_TIMEOUT_MS < 1000U || PK_CYC_STEP_TIMEOUT_MS > 120000U)
#error PK_CYC_STEP_TIMEOUT_MS must be 1000..120000
#endif
#if (PK_TRG_PICK_PULSES < 1U || PK_TRG_PICK_PULSES > PK_LIFT_MAX_PULSES || \
     PK_TRG_DROP_PULSES < 1U || PK_TRG_DROP_PULSES > PK_LIFT_MAX_PULSES)
#error PK_TRG_PICK_PULSES/DROP_PULSES out of range
#endif
#if (PK_TRG_LIFT_HZ < 100U || PK_TRG_LIFT_HZ > 40000U)
#error PK_TRG_LIFT_HZ out of range
#endif
#if (PK_TRG_ROT_DEG < 1 || PK_TRG_ROT_DEG > 360)
#error PK_TRG_ROT_DEG out of range
#endif
#if (PK_TRG_ROUNDS_DEFAULT < 1U || PK_TRG_ROUNDS_DEFAULT > PK_TRG_MAX_ROUNDS)
#error PK_TRG_ROUNDS_DEFAULT out of range
#endif

#if (PK_DROP_RELEASE_PULSES < 1U || PK_DROP_RELEASE_PULSES > PK_LIFT_MAX_PULSES)
#error PK_DROP_RELEASE_PULSES out of range
#endif
#if (PK_DROP_ALIGN_COARSE_MM < 1 || PK_DROP_ALIGN_COARSE_MM > 200)
#error PK_DROP_ALIGN_COARSE_MM out of range
#endif
#if (PK_DROP_ALIGN_TOL_MM < 1 || PK_DROP_ALIGN_TOL_MM > 200)
#error PK_DROP_ALIGN_TOL_MM out of range
#endif
#if (PK_DROP_ALIGN_TIMEOUT_MS < 1000U || PK_DROP_ALIGN_TIMEOUT_MS > 120000U)
#error PK_DROP_ALIGN_TIMEOUT_MS out of range
#endif
#if (PK_GRIP_APPROACH_PULSES >= PK_DROP_RELEASE_PULSES || PK_GRIP_LIFTOFF_PULSES >= PK_DROP_RELEASE_PULSES)
#error PK_GRIP_APPROACH/LIFTOFF_PULSES must be shorter than PK_DROP_RELEASE_PULSES
#endif

#endif
