#ifndef PICK_APP_H
#define PICK_APP_H
#include <stdint.h>

/* 机构动作层（App）：升降台、旋转台、两路舵机，以及 CYC 任务循环。
 *
 * 分层：BSP 只给"这一轴怎么出脉冲"（`Step6_*`）和"这个舵机怎么给脉宽"（`Servo_*`）；
 * 这里回答"机构该按什么顺序做什么"，所以按 AGENTS.md 第 2 节留在 App。
 *
 * 全部非阻塞：`Pick_Poll()` 由主循环每轮调用，等待都用 `Board_NowMs()` 时间戳，
 * 没有任何 `HAL_Delay`，所以不会卡住底盘命令和心跳灯。
 *
 * 安全约定：
 *   - 升降台**没有限位**，速度模式超过 `PK_LIFT_MAX_RUN_MS` 自动停；
 *   - 旋转台按角度计步（`PK_ROT_PULSES_PER_REV`），本身有界；
 *   - 步进两轴都要求 `E 1`（`Step6_IsEnabled()`），和底盘命令一致；
 *   - `STOP` / `E 0` 会停掉两轴、把载物台舵机收回到停止脉宽、中止 CYC。 */

/* ---- 单个动作（命令层直接调） ---- */

/* 旋转台按角度点动。deg 正负 = 两个方向（物理方向未实测，反了改 board_step6.c 的
 * STEP6_DIR_INVERT_ROT）；|deg| ≤ PK_ROT_MAX_DEG 且不能为 0；hz 是走这段用的频率。
 * 成功回 `OK ROT <deg> deg (<pulses> pulses @ <hz> Hz)`，走完回 `ROT done, ...`。 */
uint8_t Pick_RotDeg(int32_t deg, uint32_t hz);

/* 升降台按**脉冲距离**点动：走够 N 个脉冲就停，正负 = 上下。
 * |pulses| ≤ PK_LIFT_MAX_PULSES；hz 是走这段用的频率（默认 PK_LIFT_HZ）。
 * 成功回 `OK LIFT <pulses> pulses @ <hz> Hz`，走完回 `LIFT done, ...`。
 * ⚠ 每走一次都会累加到"升降累计位置"，`LIFT HOME` 靠它回起点。 */
uint8_t Pick_LiftPulses(int32_t pulses, uint32_t hz);

/* ---- 升降回位（2026-11-11）----
 * 内部维护一个累计值（脉冲，正 = 下降，起点 0），手动 `LIFT`、`CYC`、`TRG` 都会累加。 */

/* 当前累计位置（脉冲）。 */
int32_t Pick_LiftPosPulses(void);

/* 把当前位置定义成原点：升降手动开到最高处后发一次，用来重新对齐。 */
void Pick_LiftZeroPos(void);

/* 回到起点：按累计值的相反数走回去。已在起点时只回显、不发脉冲。
 * 返回 1 = 已开始走，0 = 没走（已在起点 / 未使能 / 超范围）。 */
uint8_t Pick_LiftGoHome(uint32_t hz);

/* 夹爪 / 机械臂舵机（U5，PB14）按角度定位，PK_ARM_DEG_MIN..MAX。
 * 成功回 `OK ARM <deg> deg (<us> us)`。 */
uint8_t Pick_ArmDeg(int32_t deg);

/* 关掉夹爪舵机 PWM（松开保持力）。 */
void Pick_ArmOff(void);

/* 载物台"点动一段"：连续旋转舵机没有位置反馈，距离只能折成时间——按 permille 转 ms
 * 毫秒后自动回停止脉宽并回显 `PLT done, stopped`。ms 上限 PK_PLAT_MAX_MS。
 * 命令层只给时间，速度固定 PK_PLAT_JOG_PERMILLE（40%）。 */
uint8_t Pick_PlatMove(int16_t permille, uint32_t ms);

/* 关掉载物台舵机 PWM。 */
void Pick_PlatOff(void);

/* ---- 任务循环 CYC：一轮 7 步，可连跑 N 轮 ---- */

/* 启动 N 轮（1..PK_CYC_MAX_ROUNDS）。未使能或 CYC 在跑时回 ERR。
 * 每轮：降 → 夹 → 升 → 转 → 松 → 盘走一格 → 转回；跑完回 `CYC done (N rounds)`。
 * 起点是"载物台/上位"，所以跑之前升降要停在上位。 */
void Pick_CycleStart(uint8_t rounds);
uint8_t Pick_CycleIsActive(void);
uint8_t Pick_CycleRound(void);    /* 已跑完的轮数 */
uint8_t Pick_CycleRounds(void);   /* 总轮数 */
/* 上一次 CYC 是否**正常跑满**：1 = `CYC done`，0 = 被 STOP/超时/单步失败中止。
 * TASK 用它决定跑完这一站能不能继续走位（启动时和 Pick_StopAll 里都会清 0）。 */
uint8_t Pick_CycleLastOk(void);

/* ---- 单次抓取 GRAB：视觉对准成功之后用 ----
 * 起点必须是丝杆最高处（lift_pos == 0）。
 *   place = 0：张开 → 快降 → 慢降贴近 → 夹紧 → 慢抬离 → 升回最高处（夹着料停住）
 *   place = 1：同样的取料，然后升到放料高度 → 转到放料侧 → 松开 → 载物台走一格 →
 *              转回取料侧 → 升回最高处；和 CYC 每轮的后半段完全一致。
 * 对准失败时不要调用。 */
void Pick_GrabStart(uint8_t place);

/* 地面抓取：同 GRAB，但下降到 PK_DROP_RELEASE_PULSES（41000）取料，再放回载物台。 */
void Pick_GroundGrabStart(uint8_t place);
uint8_t Pick_GrabIsActive(void);

/* ---- 只放置 PLACE ----
 * 手里已经夹着料、且停在最高处（lift_pos == 0）时调用：
 * 降到放料高度 → 转到放料侧 → 松开 → 载物台走一格 → 转回取料侧 → 升回最高处。
 * 用于 `GRAB ... HOLD` 之后手动补一次放料，或半路救回来。 */
void Pick_PlaceStart(void);

/* ---- 卸料序列 TRG：载物台取料 → 转到放料侧 → 降到地面 → 松开 → 转回 ---- */

/* 启动 N 轮（1..PK_TRG_MAX_ROUNDS）。起点必须是丝杆最高处（和 CYC 同一个停机位）。
 * 未使能、CYC 或 TRG 正在跑时回 ERR。跑完回 `TRG done (N rounds)`。
 * ⚠ 下降到地面那步 `PK_TRG_DROP_PULSES` 是从最高处量的，升降没有限位开关，
 *   上板前先用 `LIFT` 单独确认下行程够不够。 */
void Pick_TrgStart(uint8_t rounds);
uint8_t Pick_TrgIsActive(void);

/* ---- 圆环放置 DROP（2026-11-17）----
 * 起点必须是丝杆最高处、载物台上有料。顺序：**先空爪对准，再取料下放**。
 *   空爪转到放料侧 → 降到释放高度（= 对准高度）→ 对准圆环中心（底盘动）→
 *   升回最高处 → 转回取料侧 → 降到取料位 → 夹料 → 升回最高处 →
 *   转到放料侧 → 降到释放高度 → 松开 → 抬离 → 回最高处 → 转回。
 * 对准之后底盘不再动，ROT 的回程重复精度直接决定落点。
 * 对准失败/超时/STEP 会先停轴再把升降升回最高处（不会停在半空）。 */
void Pick_DropStart(uint8_t ring);   /* ring = 1/2/3…，对准哪个圆环 */
void Pick_DropStartDry(uint8_t ring); /* 不接视觉：跳过 ALIGN，其余取料/转台/下放照跑 */
uint8_t Pick_DropLastOk(void);       /* 最近一次 DROP 是否完整成功 */
uint8_t Pick_DropIsActive(void);
void Pick_DropAbort(void);

/* 停掉机构动作（两轴 + 载物台），不回显；`STOP` / `E 0` 用。 */
void Pick_StopAll(void);

/* 主循环轮询：推进单轴点动、载物台定时、CYC 状态机。 */
void Pick_Poll(void);

/* ---- 状态查询（给 `S` 用） ---- */
uint8_t  Pick_ArmIsOn(void);
uint16_t Pick_ArmPulseUs(void);     /* 最近一次下发的脉宽 */
int16_t  Pick_PlatGetPermille(void);/* 最近一次下发的千分比 */
uint8_t  Pick_RotIsMoving(void);    /* 旋转台点动在跑 */
int16_t  Pick_RotGetDeg(void);      /* 我们发过的 ROT 累计角度（度） */

#endif
