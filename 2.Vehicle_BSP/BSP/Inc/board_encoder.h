#ifndef BOARD_ENCODER_H
#define BOARD_ENCODER_H

#include <stdint.h>
#include "board.h"

/*
 * 车轮编码器测量层。
 *
 * 分工（两层都在 BSP）：
 *   board_motion.c  —— 拥有 4 路计数的硬件：TIM2 / TIM3 / TIM4 编码器模式，
 *                      外加 PD8/PD9 的 EXTI 软件正交解码（编码器 D）。
 *                      对外只提供 Encoder_SampleCounts()：**唯一的累计计数更新点**。
 *   board_encoder.c —— 本文件：按固定 tick 采样、算速度、做量纲换算和方向归一化。
 *
 * 为什么必须这样分：TIM3 / TIM4 是 16 位计数器，要靠"上次原始值"把计数扩展到 32 位。
 * 如果读函数自己带这个累加状态（旧的 Encoder_Read() 就是这样），
 * 那么控制中断和主循环遥测各读一次，就会互相吃掉对方的增量，两边读数都错。
 * 所以：**扩展只在 tick 里做一次，读侧永远无副作用。**
 */

/* ============================== 参数区 ============================== */

/* 车轮参数：JGB37-520 有刷减速电机 + 霍尔编码器
 *   编码器单相 11 线，AB 四倍频 -> 电机轴每转 11 x 4 = 44 计数
 * 换电机或换减速比只改这两个数。**PID 用 counts/s，所以改这里不影响已调好的参数。**
 *
 * 2026-10-27 实测重标（两处原来的假设都是错的，别再按铭牌回填）：
 *   手把轮子朝同一方向**转满一整圈**，`Z` 后 `N` 读数：
 *     A=-815 B=+802 C=-803 D=+815（原始计数，四路都没做符号修正）
 *   幅值 802~815、一致性 1.6%，均值 **809** -> `WHEEL_COUNTS_PER_REV = 810`。
 *   ⚠ 原来的 1320 是按"44 x 减速比 1:30"估的；实测 809/44 ≈ 18.4，
 *     说明**这颗电机的减速比不是 1:30**（或编码器倍率不是 4 倍），总之以实测为准。
 *   轮径用同一批数据反推：手推 1 m 时四路均值 3426 counts，
 *     周长 = 809 / 3426 = 0.2362 m -> 直径 **75.2 mm**，
 *     即名义 75 mm 是对的（之前按 70 mm 改过一次，是错的，已改回）。
 *   A/C 原始计数为负、B/D 为正，是两侧编码器装法不同，由 ENCODER_*_SIGN 的
 *   +1/-1 抵消，不是故障 —— 四路归一化之后方向是一致的。
 * ⚠ 这两个数只影响 counts <-> um 的换算（里程、定距离、车体速度），不影响速度环。 */
#define WHEEL_COUNTS_PER_REV          810
#define WHEEL_DIAMETER_MM               75

/* 测速 tick 频率，单位 Hz。10 ms = 100 Hz，与 PI 控制周期一致。
 * 810 counts/rev 下 10 ms 窗口在 0.5 m/s 能数到约 17 个计数（6% 量化），够用。 */
#define ENCODER_TICK_HZ                100U

/* 每路编码器的方向符号：装反了（轮子正转却读到负数）就把对应那个改成 -1。
 * 2026-10-26 四轮实测：左侧 A/C 电机符号 -1、编码器 +1；右侧 B/D 电机 +1、
 * 编码器 -1。日志依据：B 100000 时 A/B/D actual 为负、C 为正。 */
#define ENCODER_A_SIGN                   1
#define ENCODER_B_SIGN                  -1
#define ENCODER_C_SIGN                   1
#define ENCODER_D_SIGN                  -1

/* 连续这么多个 tick 没有新计数就判定为真的停了，单位 tick。
 * 没有这一条，速度低通会把残速拖很久，控制环会以为车还在动。 */
#define ENCODER_STILL_TICKS              3U

/* 速度一阶低通：speed += (instant - speed) / N。1 = 不滤波，4~8 常用。 */
#define ENCODER_SPEED_FILTER_DIV         4U

#if (WHEEL_COUNTS_PER_REV <= 0)
#error WHEEL_COUNTS_PER_REV must be positive
#endif
#if (WHEEL_DIAMETER_MM <= 0)
#error WHEEL_DIAMETER_MM must be positive
#endif
#if (ENCODER_TICK_HZ < 10U || ENCODER_TICK_HZ > 1000U)
#error ENCODER_TICK_HZ must be within 10..1000
#endif
#if (ENCODER_STILL_TICKS < 1U)
#error ENCODER_STILL_TICKS must be at least 1
#endif
#if (ENCODER_SPEED_FILTER_DIV < 1U)
#error ENCODER_SPEED_FILTER_DIV must be at least 1
#endif
#if (ENCODER_A_SIGN != 1 && ENCODER_A_SIGN != -1)
#error ENCODER_A_SIGN must be 1 or -1
#endif
#if (ENCODER_B_SIGN != 1 && ENCODER_B_SIGN != -1)
#error ENCODER_B_SIGN must be 1 or -1
#endif
#if (ENCODER_C_SIGN != 1 && ENCODER_C_SIGN != -1)
#error ENCODER_C_SIGN must be 1 or -1
#endif
#if (ENCODER_D_SIGN != 1 && ENCODER_D_SIGN != -1)
#error ENCODER_D_SIGN must be 1 or -1
#endif

/* ============================== 接口 ============================== */

/* 启动 TIM7 的 ENCODER_TICK_HZ 中断并把计数清零。
 * 必须在 Board_MotionInit() 之后调用（要读 TIM2/3/4，它们先要初始化好）。 */
HAL_StatusTypeDef Encoder_Init(void);

/* 累计计数，已经按 ENCODER_x_SIGN 归一化方向（正数 = 该轮"前进"方向）。
 * 每个 tick 刷新一次，所以最多滞后 1/ENCODER_TICK_HZ 秒。 */
int32_t Encoder_Counts(EncoderId id);

/* 轮速。PID 用 counts/s；um/s 只用于显示和换挡判断。
 * 1024 counts/s 的原始分辨率就是 0.1785 mm/s，所以整数里有足够精度。 */
int32_t Encoder_SpeedCountsPerSec(EncoderId id);
int32_t Encoder_SpeedUmPerSec(EncoderId id);

/* 逆换算：把"想要的车轮线速度"换成目标 counts/s，供底盘运动学（Wheel_SetBodyVelocity）用。
 * 与 Encoder_SpeedUmPerSec() 共用同一组轮径 / 每转计数，保证正反换算一致。 */
int32_t Encoder_CountsPerSecFromUmPerSec(int32_t um_per_s);

/* 4 路计数与速度同时清零，作为新一轮测量的基准。 */
void Encoder_ResetAll(void);

/* 由 TIM7 中断每 1/ENCODER_TICK_HZ 秒调用一次。自测时也可以在主循环手动调。 */
void Encoder_Tick(void);

/* 已经跑了多少个 tick，用来判断采样是否在正常推进（不涨就是中断没进来）。 */
uint32_t Encoder_Ticks(void);

/* 里程计专用窗口：只由 TIM7 累加，只有一个主循环消费者。
 * 原有 Counts/Speed 读接口仍无副作用。峰值保留单拍尖峰，不能被反向计数抵消。 */
typedef struct {
  int32_t delta_counts[4];
  uint32_t peak_delta_counts[4];
  uint32_t sampled_ms;
  uint32_t generation;
  uint32_t ticks;
} EncoderWindow;
void Encoder_TakeWindow(EncoderWindow *window);

#endif
