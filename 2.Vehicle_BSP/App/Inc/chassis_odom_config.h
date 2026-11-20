#ifndef CHASSIS_ODOM_CONFIG_H
#define CHASSIS_ODOM_CONFIG_H
#include "board_wheel.h"
/* ============================== 参数区 ============================== */
/* 几何与角映射复用 BSP 唯一来源；轮径、方向、陀螺仪标度均待实测标定。
 * 超过十二项参数按 AGENTS.md 拆到独立配置头。 */
#define WHEEL_EFFECTIVE_DIAMETER_UM 75000
#define ODOM_SIGN_WHEEL_A 1
#define ODOM_SIGN_WHEEL_B 1
#define ODOM_SIGN_WHEEL_C 1
#define ODOM_SIGN_WHEEL_D 1
#define ODOM_LATERAL_SIGN 1
#define GYRO_Z_SIGN 1
#define GYRO_SCALE_PERMILLE 1000
/* 单拍位移上限，单位 um，超过就丢弃该拍并计一次 resync（防编码器毛刺/丢边沿）。
 * ⚠ 2026-10-07 由 3000 提到 15000：这个上限是"每拍(10 ms)位移"，3000 um 只允许
 *   3000/178.5 ≈ 16.8 counts/拍 = 1680 counts/s ≈ 0.30 m/s，而 12V 空载就有
 *   7326 counts/s（≈1.31 m/s），正常巡航（1500~3000 counts/s）会一直触发 resync，
 *   表现为"里程计在跑起来之后就不走了"。
 *   15000 um/拍 = 1.5 m/s/轮，仍高于 12V 空载 1.31 m/s，毛刺照样能拦住。 */
#define ODOM_RESYNC_MAX_UM 15000
#define ODOM_UPDATE_MS 10U
#define ODOM_MAX_GAP_MS 100U
#define ODOM_GYRO_DEADZONE_MDPS 150
#define ODOM_STILL_GYRO_MDPS 1500
#define ODOM_STILL_PP_MDPS 1000
#define ODOM_STILL_COUNTS_PER_S 80
#define ODOM_STILL_MS 200U
#define ODOM_NOISE_SAMPLES 20U
/* 默认以陀螺仪定航向；0 只供编码器算法隔离验证，不是故障自动降级。 */
#define ODOM_USE_GYRO 1
#if WHEEL_EFFECTIVE_DIAMETER_UM < 1000 || WHEEL_EFFECTIVE_DIAMETER_UM > 1000000
#error WHEEL_EFFECTIVE_DIAMETER_UM must be within 1000..1000000
#endif
#if (ODOM_SIGN_WHEEL_A != 1 && ODOM_SIGN_WHEEL_A != -1) ||     (ODOM_SIGN_WHEEL_B != 1 && ODOM_SIGN_WHEEL_B != -1) ||     (ODOM_SIGN_WHEEL_C != 1 && ODOM_SIGN_WHEEL_C != -1) ||     (ODOM_SIGN_WHEEL_D != 1 && ODOM_SIGN_WHEEL_D != -1)
#error ODOM_SIGN_WHEEL_A..D must be 1 or -1
#endif
#if (ODOM_LATERAL_SIGN != 1 && ODOM_LATERAL_SIGN != -1) || (GYRO_Z_SIGN != 1 && GYRO_Z_SIGN != -1)
#error ODOM_LATERAL_SIGN and GYRO_Z_SIGN must be 1 or -1
#endif
#if GYRO_SCALE_PERMILLE < 500 || GYRO_SCALE_PERMILLE > 2000
#error GYRO_SCALE_PERMILLE must be within 500..2000
#endif
#if ODOM_UPDATE_MS < 1U || ODOM_UPDATE_MS > 100U || ODOM_MAX_GAP_MS < ODOM_UPDATE_MS || ODOM_MAX_GAP_MS > 1000U
#error Invalid odometry update or gap period
#endif
#if ODOM_RESYNC_MAX_UM < 1 || ODOM_RESYNC_MAX_UM > 100000
#error ODOM_RESYNC_MAX_UM must be within 1..100000
#endif
#if ODOM_GYRO_DEADZONE_MDPS < 0 || ODOM_GYRO_DEADZONE_MDPS > ODOM_STILL_GYRO_MDPS || ODOM_STILL_GYRO_MDPS > 500000
#error Invalid gyro deadzone or still limit
#endif
#if ODOM_STILL_PP_MDPS < 1 || ODOM_STILL_PP_MDPS > 500000 || ODOM_STILL_COUNTS_PER_S < 0 || ODOM_STILL_COUNTS_PER_S > 5000
#error Invalid stationary noise or wheel speed limit
#endif
#if ODOM_STILL_MS < ODOM_UPDATE_MS || ODOM_STILL_MS > 10000U || ODOM_NOISE_SAMPLES < 2U || ODOM_NOISE_SAMPLES > 100U
#error Invalid stationary duration or sample window
#endif
#if ODOM_USE_GYRO != 0 && ODOM_USE_GYRO != 1
#error ODOM_USE_GYRO must be 0 or 1
#endif
#endif
