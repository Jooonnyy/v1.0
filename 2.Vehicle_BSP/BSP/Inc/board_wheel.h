#ifndef BOARD_WHEEL_H
#define BOARD_WHEEL_H

#include <stdint.h>
#include "board.h"
#include "board_encoder.h"

/*
 * 四轮底盘运动层：麦轮逆解算 + 每轮速度闭环。
 *
 * 为什么放在 BSP（AGENTS.md 第 2 节）：它回答的是"这套四轮底盘现在是什么状态、
 * 怎么驱动它"—— 输入是车体速度、输出是四路 PWM，里面没有任何"整车该做什么"的决策。
 * 上层（App）只决定"什么时候、以什么速度走"。
 *
 * 控制周期与测速周期共用 TIM7 的 100 Hz：
 *   采样计数 → 算轮速 → 跑这一拍的 PI → 写 PWM
 * 所以 Wheel_Update() 由 board_encoder.c 的 TIM7 中断调用（声明在 board_internal.h）。
 * 它必须遵守 AGENTS.md 第 6 节"控制环 ISR"的五条硬性条件：
 * 周期固定、只算不发、只写快照、执行有界、输出先限幅再写 PWM。
 *
 * 接线约定：MOTOR_A/B/C/D 与 ENCODER_A/B/C/D 一一对应（A 电机配 A 编码器……）。
 */

/* ============================== 参数区 ============================== */

/* 控制周期直接跟随测速 tick，不另开定时器。 */
#define WHEEL_CTRL_HZ                ENCODER_TICK_HZ
#define WHEEL_CTRL_PERIOD_MS         (1000U / WHEEL_CTRL_HZ)

/* 车体几何：轮心到车体中心的纵向 / 横向距离，单位 um。
 * 2026-10-27 用户实测：前后轮心距 220 mm、左右轮距 230 mm
 *   -> 半轴距 110 mm、半轮距 115 mm -> L = 225 mm（原 180/180 是占位值）。
 * `L = 半轴距 + 半轮距` 是 wz -> 四轮轮速的**唯一标度**：
 * 填小了，同样的 wz 指令实际转得比预期慢（等效环路增益偏低），反之偏快。
 * 注意它**不影响"转多少度"** —— 航向环闭在陀螺仪 yaw 上，最终到位精度与 L 无关。 */
#define CHASSIS_HALF_LENGTH_UM       110000
#define CHASSIS_HALF_WIDTH_UM        115000

/* A/B/C/D 各在车体的哪个角：0 = 左前、1 = 右前、2 = 左后、3 = 右后。
 * 出厂按 A=左前 B=右前 C=左后 D=右后；实车不符就改这四行（比改接线省事）。 */
#define WHEEL_CORNER_A                   0
#define WHEEL_CORNER_B                   1
#define WHEEL_CORNER_C                   2
#define WHEEL_CORNER_D                   3

/* 每路的电机极性：+1 表示 Motor_Set(id, +permille) 使该轮朝"编码器正方向"转。
 * 装反了就把对应那个改成 -1。用 D 开环命令逐个确认，不要靠猜。
 *
 * 2026-10-26 二次定案（AT8236 真值表 + 实测）：
 *   模块每通道是 IN1/IN2 两输入，PWM 接在 IN2 上。
 *     IN1=1, IN2=PWM -> 正转 **慢衰减**（下管续流，平均电流大）
 *     IN1=0, IN2=PWM -> 反转 **快衰减**（二极管续流，平均电流小）
 *   同样占空比下反向明显更弱：实测 A +300->1866 / -300->1282，C +300->1305 / -300->122。
 *   所以**四路都应该跑在正向**。A/C 原来是 -1（跑在弱方向），现改为 +1；
 *   ⚠ 必须同时把 **A、C 两个电机的两根线对调**，否则整车前进方向会反。
 *   编码器符号 A/C 保持 +1 不变。
 *   历史：2026-10-26 早些时候按"左侧电机线与右侧镜像"把 A/C 设成 -1，
 *   那次只看方向、没注意正反两个方向的强弱，才让左侧跑进了快衰减。 */
#define WHEEL_MOTOR_SIGN_A               1
#define WHEEL_MOTOR_SIGN_B               1
#define WHEEL_MOTOR_SIGN_C               1
#define WHEEL_MOTOR_SIGN_D               1

/* 整机方向的符号修正：横移整体反向改 WHEEL_LATERAL_SIGN，自转整体反向改 WHEEL_SPIN_SIGN。
 * 只影响 Wheel_SetBodyVelocity() 的解算结果。 */
#define WHEEL_LATERAL_SIGN               1
#define WHEEL_SPIN_SIGN                  1

/* 前馈 = 斜率 + 摩擦常数项（单位都是千分比）。
 * 2026-10-26 闭环实测两点拟合：560 counts/s -> 145‰、840 counts/s -> 180‰，
 * 解出斜率 ≈125‰/1000cps、常数项 ≈70‰。旧值 170‰/1000cps 是纯比例、没有常数项，
 * 低速时前馈偏小（560 时只有 95‰，实际要 145‰），差的 50‰ 全靠积分补 ——
 * 积分一补就会 windup，这就是启动超调 23~25% 的来源。 */
#define WHEEL_FF_PERMILLE_PER_1000CPS   125
/* 摩擦前馈常数项：目标非零时按方向叠加。低速起步要先克服静摩擦，纯比例前馈估不到。 */
#define WHEEL_FF_OFFSET_PERMILLE         70
/* 反转方向的前馈放大系数，千分比。
 * 理由：AT8236 只引出一根 PWM + 一根 DIR，PWM 打进 IN2；正转是慢衰减、反转是快衰减，
 * 同样转速反转要多给 1.5~2 倍的占空比。而上面那套前馈只在正转上标定过，
 * 反转时前馈少给一大截，缺口全压给积分 —— 一旦顶到 WHEEL_INTEGRAL_LIMIT_PERMILLE
 * 就彻底补不上，表现为那一路转速永远差一截。
 * 2026-10-27 实测（目标 ±600 counts/s）：
 *   正转四路 117/127/166/122‰；反转 203/299/384/231‰，
 *   反推积分项 -74/-149/-151/-99，B 与 C 正好卡在 150 的限幅上（C 只到 -382 counts/s）。
 * 取 1700（1.7 倍）后前馈 145‰ -> 246‰，B/C 的积分需求降到约 -48/-50，回到限幅内。
 * 1000 = 关闭（正反用同一套前馈）。 */
#define WHEEL_FF_REVERSE_BOOST_PERMILLE 1700

/* PI 增益，Q10 定点（实际值 = 参数 / 1024）。
 * 2026-10-24 A 轮实测整定：Kp=414、Ki=34，先作为四轮共用起点。
 * 单路差异大时再考虑改成每轮独立参数。 */
#define WHEEL_KP_Q10                   414
#define WHEEL_KI_Q10                    34
/* 积分限幅（anti-windup 的第二道保险），单位千分比。
 * 2026-10-26：加上摩擦前馈之后，稳态积分只需要 ~5‰；原来 400 太大 ——
 * 起步段误差长期在 400~500，积分很快就顶满，速度冲过头后要 2~3 s 才退下来。
 * 收到 150 仍能覆盖单路差异（C 轮比别的多要 ~20‰），但超调明显变小。
 * ⚠ 这个 150 只对正转成立。2026-10-27 实测反转时有一路在 384‰ 输出下只跑到 382 counts/s，
 *   要跑到 600 得把输出推到接近 600‰，即积分要补约 354‰ —— 150 根本不够，
 *   于是那一路永远差速。所以反转单独给一个大的限幅。 */
#define WHEEL_INTEGRAL_LIMIT_PERMILLE  150
/* 反转方向的积分限幅。加大不会引起正转那种起步超调：反转起步时输出很快就顶到
 * WHEEL_DUTY_LIMIT_PERMILLE，已有的条件积分（饱和且同向误差时不再累加）会把它挡住。 */
#define WHEEL_INTEGRAL_LIMIT_REVERSE_PERMILLE 400

/* 输出限幅，单位千分比。AGENTS.md 第 6 节：闭环输出必须明显低于 1000‰。 */
#define WHEEL_DUTY_LIMIT_PERMILLE      600

/* 停机时的主动刹车保持时间，单位 ms。0 = 不刹车（直接滑行，原来的行为）。
 * 原来 Motor_Set(0) 是 IN1=0/IN2=0，电机两端悬空，靠惯性滑行 —— 0.5 m/s 要将近 1 s 才停，
 * 停不准、还会"自己往前蹭一段"。改成先短接电机做动态制动，再转滑行。
 * 默认 150 ms：够把 0.5 m/s 拉下来，又不至于长时间短接发热。
 * ⚠ 高速急刹会让麦轮瞬间失抓、可能甩尾；如果发现停车姿态不稳，先把这个数调小。 */
#define WHEEL_BRAKE_MS                 150U
/* 目标速度限幅，单位 counts/s（防上层给离谱值）。 */
#define WHEEL_TARGET_LIMIT_CPS        5000
/* 目标速度每拍最大变化量，单位 counts/s（软启动，减小打滑与电流冲击）。0 = 关闭。
 * 400 表示 100 Hz 下 1.25 s 从 0 加到 5000 counts/s。 */
#define WHEEL_TARGET_SLEW_CPS          400

/* 堵转检测：输出已顶到限幅附近、实际速度仍低于 WHEEL_STALL_CPS、
 * 且目标明显大于它，持续 WHEEL_STALL_MS 就判定堵转 → 急停 + 置 WHEEL_STATE_STALL。
 * 0 = 关闭。 */
#define WHEEL_STALL_CPS                 80
#define WHEEL_STALL_MS                 600U

#if (CHASSIS_HALF_LENGTH_UM <= 0 || CHASSIS_HALF_WIDTH_UM <= 0)
#error CHASSIS_HALF_LENGTH_UM / CHASSIS_HALF_WIDTH_UM must be positive
#endif
#if ((1U << WHEEL_CORNER_A) | (1U << WHEEL_CORNER_B) | \
     (1U << WHEEL_CORNER_C) | (1U << WHEEL_CORNER_D)) != 0x0FU
#error WHEEL_CORNER_A..D must be a permutation of 0..3 (0=FL 1=FR 2=RL 3=RR)
#endif
#if (WHEEL_LATERAL_SIGN != 1 && WHEEL_LATERAL_SIGN != -1)
#error WHEEL_LATERAL_SIGN must be 1 or -1
#endif
#if (WHEEL_SPIN_SIGN != 1 && WHEEL_SPIN_SIGN != -1)
#error WHEEL_SPIN_SIGN must be 1 or -1
#endif
#if (WHEEL_MOTOR_SIGN_A != 1 && WHEEL_MOTOR_SIGN_A != -1)
#error WHEEL_MOTOR_SIGN_A must be 1 or -1
#endif
#if (WHEEL_MOTOR_SIGN_B != 1 && WHEEL_MOTOR_SIGN_B != -1)
#error WHEEL_MOTOR_SIGN_B must be 1 or -1
#endif
#if (WHEEL_MOTOR_SIGN_C != 1 && WHEEL_MOTOR_SIGN_C != -1)
#error WHEEL_MOTOR_SIGN_C must be 1 or -1
#endif
#if (WHEEL_MOTOR_SIGN_D != 1 && WHEEL_MOTOR_SIGN_D != -1)
#error WHEEL_MOTOR_SIGN_D must be 1 or -1
#endif
#if (WHEEL_KP_Q10 < 0 || WHEEL_KI_Q10 < 0)
#error WHEEL gains must not be negative
#endif
#if (WHEEL_DUTY_LIMIT_PERMILLE < 100 || WHEEL_DUTY_LIMIT_PERMILLE > 900)
#error WHEEL_DUTY_LIMIT_PERMILLE must be within 100..900
#endif
#if (WHEEL_BRAKE_MS > 1000U)
#error WHEEL_BRAKE_MS must be <= 1000 (0 = no active brake)
#endif
#if (WHEEL_INTEGRAL_LIMIT_PERMILLE < 0 || WHEEL_INTEGRAL_LIMIT_PERMILLE > WHEEL_DUTY_LIMIT_PERMILLE)
#error WHEEL_INTEGRAL_LIMIT_PERMILLE must be within 0..WHEEL_DUTY_LIMIT_PERMILLE
#endif
#if (WHEEL_INTEGRAL_LIMIT_REVERSE_PERMILLE < 0 || \
     WHEEL_INTEGRAL_LIMIT_REVERSE_PERMILLE > WHEEL_DUTY_LIMIT_PERMILLE)
#error WHEEL_INTEGRAL_LIMIT_REVERSE_PERMILLE must be within 0..WHEEL_DUTY_LIMIT_PERMILLE
#endif
#if (WHEEL_TARGET_LIMIT_CPS <= 0)
#error WHEEL_TARGET_LIMIT_CPS must be positive
#endif
#if (WHEEL_TARGET_SLEW_CPS < 0)
#error WHEEL_TARGET_SLEW_CPS must not be negative
#endif
#if (WHEEL_STALL_CPS < 0)
#error WHEEL_STALL_CPS must not be negative
#endif
#if (WHEEL_FF_PERMILLE_PER_1000CPS < 0)
#error WHEEL_FF_PERMILLE_PER_1000CPS must not be negative
#endif
#if (WHEEL_FF_OFFSET_PERMILLE < 0 || WHEEL_FF_OFFSET_PERMILLE > 300)
#error WHEEL_FF_OFFSET_PERMILLE must be within 0..300
#endif
#if (WHEEL_FF_REVERSE_BOOST_PERMILLE < 500 || WHEEL_FF_REVERSE_BOOST_PERMILLE > 4000)
#error WHEEL_FF_REVERSE_BOOST_PERMILLE must be within 500..4000 (1000 = off)
#endif

/* ============================== 接口 ============================== */

typedef enum {
  WHEEL_STATE_IDLE = 0,   /* 未使能，四路 PWM=0、DIR=0（AT8236 滑行） */
  WHEEL_STATE_RUN,        /* 闭环运行中 */
  WHEEL_STATE_STALL       /* 检测到堵转，已急停，等上层处理（需重新 Wheel_Enable） */
} WheelState;

/* 清零状态。必须在 Encoder_Init() 之前调用 —— 第一个 tick 就会调 Wheel_Update()。 */
void Wheel_Init(void);

/* 使能 / 失能整台底盘。AT8236 没有 STBY，使能只是清目标/积分并允许闭环输出。
 * 失能等价于 Wheel_Stop()。 */
void Wheel_Enable(uint8_t enable);
uint8_t Wheel_IsEnabled(void);
/* 是否正处于停机刹车窗口（WHEEL_BRAKE_MS 之内）。上层在刹车期间不要再调
 * Motor_StopAll()，那会把短接制动变回滑行、白白丢掉刹车效果。 */
uint8_t Wheel_IsBraking(void);
/* 取消刹车窗口（只清计时，不改输出）。开环命令 D/ALL/CMP 在写占空比前必须先调它，
 * 否则刹车计时器到点会 Motor_StopAll()，把刚设的开环占空比清掉。 */
void Wheel_BrakeCancel(void);

/* 急停：清目标与积分、输出 0、两片 TB6612 进待机。任何异常路径都该调它。 */
void Wheel_Stop(void);

/* 单轮目标速度，单位 counts/s（正 = 该轮前进方向）。会自动限幅。 */
void Wheel_SetTargetCountsPerSec(MotorId id, int32_t target_counts_per_s);

/* 车体速度 → 四轮目标速度（麦轮逆解算，X 型布局）。
 *   vx_um_per_s : 前向速度，单位 um/s（正 = 车头方向）
 *   vy_um_per_s : 左向速度，单位 um/s（正 = 车体左侧）
 *   wz_mdps     : 逆时针角速度，单位 0.001 °/s
 * 解算：v_FL = vx - vy - L·ωz，v_FR = vx + vy + L·ωz，
 *       v_RL = vx + vy - L·ωz，v_RR = vx - vy + L·ωz，其中 L = 半轴距 + 半轮距。 */
void Wheel_SetBodyVelocity(int32_t vx_um_per_s, int32_t vy_um_per_s, int32_t wz_mdps);

/* 100 Hz 一拍的闭环计算，由 TIM7 tick 调用。不要在别处调用。 */
void Wheel_Update(void);

/* 观测量（供遥测与调参）。 */
int32_t Wheel_TargetCountsPerSec(MotorId id);
int32_t Wheel_OutputPermille(MotorId id);
WheelState Wheel_State(void);
uint32_t Wheel_StallEvents(void);

/* 运行时改增益（台架调参用）。kp_q10 / ki_q10 是 Q10，
 * ff_per_1000cps 是"每 1000 counts/s 的千分比"。 */
void Wheel_SetGains(int16_t kp_q10, int16_t ki_q10, int16_t ff_per_1000cps);
void Wheel_GetGains(int16_t *kp_q10, int16_t *ki_q10, int16_t *ff_per_1000cps);

#endif
