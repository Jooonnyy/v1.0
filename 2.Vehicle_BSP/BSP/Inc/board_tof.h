#ifndef BOARD_TOF_H
#define BOARD_TOF_H

#include <stdint.h>
#include "board.h"
#include "pins/tof400c.h"

/*
 * TOF400C 激光测距（VL53L1X）。
 *
 * 硬件：模块挂在板子的软件 I2C 上（PB6=SCL / PB7=SDA，与 MPU6050 共用总线），
 *       7 位地址 0x29；PC2 = INT。当前原理图里模块 SHUT 悬空，PC1 没有接过去。
 *       固件用 TOF_SHUT_CONNECTED 区分硬件版本，避免误驱动 PC1。
 */

/* ============================== 参数区 ============================== */

/* VL53L1X 的 7 位 I2C 地址以引脚映射为唯一来源。 */
#define TOF_I2C_ADDR7               TOF_I2C_ADDR
/* 测距积分时间，单位 ms。越大噪声越低、刷新越慢；100 ms 是精度与速度的折中，
 * 抖动仍大时可试 200（同时记得改 TOF_INTER_MEASUREMENT_MS）。 */
#define TOF_TIMING_BUDGET_MS         100U
/* 连续测距的两次测量间隔，单位 ms。必须 >= 积分时间，也必须 <= 应用侧的读取周期。 */
#define TOF_INTER_MEASUREMENT_MS     110U
/* 距离模式：1 = 短距（最远 1.3 m，抗环境光更好、噪声更低）；2 = 长距（暗处可到 4 m）。
 * 2026-10-02 定为 1：实测目标都在 1.3 m 以内，短距模式的抖动明显更小。
 * 以后需要量 1.3 m 以上再改回 2，但要知道长距模式的噪声和环境光敏感性都更差。 */
#define TOF_DISTANCE_MODE              1U
/* 释放 XSHUT 后等器件启动。数据手册给的启动时间上限是 1.2 ms，这里留足余量。 */
#define TOF_BOOT_DELAY_MS              5U
/* 复位脉冲里 XSHUT 保持低电平的时间，单位 ms。
 * 官方 STM32 例程初始化前也是先 ResetId(0) 再 ResetId(1)，保证器件从干净状态启动。 */
#define TOF_RESET_HOLD_MS              5U
/* 等固件就绪 / 等一次测量完成的上限，单位 ms。 */
#define TOF_BOOT_TIMEOUT_MS          200U
#define TOF_READY_TIMEOUT_MS         200U
/* VL53L1X 的型号 ID（读寄存器 0x010F）。 */
#define TOF_MODEL_ID               0xEACCU
/* 读取失败时回给调用方的距离值，单位 mm。 */
#define TOF_DISTANCE_INVALID_MM    0xFFFFU

/* ------------------------------ 精度相关 ------------------------------ */

/* 距离滤波分两级，都只吃"RangeStatus == 0"的有效样本：
 *   1) 中值滤波：最近 TOF_MEDIAN_WINDOW 个有效样本取中值，专治孤立跳变（尖峰）。
 *   2) 滑动平均：对中值结果再做 TOF_AVERAGE_WINDOW 点平均，压随机噪声（噪声约按 1/sqrt(N) 降）。
 * 取 1 表示关闭该级。中值窗口必须是奇数（偶数个取中值有两义性）。 */
#define TOF_MEDIAN_WINDOW              5U
#define TOF_AVERAGE_WINDOW             4U

/* 器件侧的 sigma 门槛，单位 mm。sigma 是器件自己估计的标准差：超过门槛的测量
 * 会被器件直接判为无效（RangeStatus 变 1），等于在源头挡掉噪声样本。
 * ST 出厂默认 90 mm，对近距离测量太宽松。设 0 表示不改动器件默认值。
 * 注意：写得过小会导致"所有测量都无效"，可从 25 起试。 */
#define TOF_SIGMA_THRESHOLD_MM        25U

/* ------------------------------ 视场（ROI） ------------------------------ */

/* VL53L1X 的接收端是 16x16 的 SPAD 阵列，默认全用上，视场约 27°（对角）。
 * 缩小 ROI = 只选通阵列中的一部分 SPAD，视场近似按比例收窄：
 *   16x16 = 全视场（约 27° 对角，单轴约 19°）
 *    8x8  ≈ 单轴 9.5°      4x4 ≈ 单轴 4.8°（ULD 允许的最小值）
 *
 * 代价：参与收光的 SPAD 变少，信号率（串口的 sig 列）按比例下降，
 *       信噪比变差、最大量程变短 —— 想维持精度就要相应加大 TOF_TIMING_BUDGET_MS。
 *
 * ⚠ ROI 是"选通 SPAD"，不是光学光圈：发射光斑仍然是全视场，
 *   近距离高反射目标、盖板反射的光仍可能进到被选中的 SPAD 里。
 *   所以它能挡掉视场外的物体，但不能消除串扰，也不能修正最小量程问题。 */
#define TOF_ROI_WIDTH_SPADS           16U
#define TOF_ROI_HEIGHT_SPADS          16U
/* ROI 中心 SPAD 的序号，199 = 阵列的光学中心（水平 14、垂直 14 那一格）。
 * 只有宽或高 <= 10 时才允许偏心：ULD 自己的规则是 X 或 Y > 10 就把中心强制回 199。
 * 偏心越界不会在配置时报错，而是测距时被判无效，所以这里用 #error 提前挡住。 */
#define TOF_ROI_CENTER_SPAD          199U

#if (TOF_I2C_ADDR7 > 0x7FU)
#error TOF_I2C_ADDR7 must be a 7-bit address
#endif
#if (TOF_DISTANCE_MODE != 1U && TOF_DISTANCE_MODE != 2U)
#error TOF_DISTANCE_MODE must be 1 (short) or 2 (long)
#endif
#if (TOF_TIMING_BUDGET_MS < 20U || TOF_TIMING_BUDGET_MS > 1000U)
#error TOF_TIMING_BUDGET_MS must be within 20..1000
#endif
#if (TOF_INTER_MEASUREMENT_MS < TOF_TIMING_BUDGET_MS)
#error TOF_INTER_MEASUREMENT_MS must be >= TOF_TIMING_BUDGET_MS
#endif
#if (TOF_BOOT_DELAY_MS < 2U)
#error TOF_BOOT_DELAY_MS must be at least 2 ms
#endif
#if (TOF_RESET_HOLD_MS < 1U || TOF_RESET_HOLD_MS > 100U)
#error TOF_RESET_HOLD_MS must be within 1..100
#endif
#if (TOF_MEDIAN_WINDOW < 1U || TOF_MEDIAN_WINDOW > 9U || (TOF_MEDIAN_WINDOW % 2U) == 0U)
#error TOF_MEDIAN_WINDOW must be an odd number within 1..9
#endif
#if (TOF_AVERAGE_WINDOW < 1U || TOF_AVERAGE_WINDOW > 9U)
#error TOF_AVERAGE_WINDOW must be within 1..9
#endif
#if (TOF_SIGMA_THRESHOLD_MM > 1000U)
#error TOF_SIGMA_THRESHOLD_MM must be within 0..1000
#endif
#if (TOF_ROI_WIDTH_SPADS < 4U || TOF_ROI_WIDTH_SPADS > 16U)
#error TOF_ROI_WIDTH_SPADS must be within 4..16 (4 is the smallest the ULD accepts)
#endif
#if (TOF_ROI_HEIGHT_SPADS < 4U || TOF_ROI_HEIGHT_SPADS > 16U)
#error TOF_ROI_HEIGHT_SPADS must be within 4..16 (4 is the smallest the ULD accepts)
#endif
#if (TOF_ROI_CENTER_SPAD > 255U)
#error TOF_ROI_CENTER_SPAD must be within 0..255
#endif
#if ((TOF_ROI_WIDTH_SPADS > 10U || TOF_ROI_HEIGHT_SPADS > 10U) && (TOF_ROI_CENTER_SPAD != 199U))
#error A ROI wider or taller than 10 cannot be off-centre; keep TOF_ROI_CENTER_SPAD at 199
#endif

/* ============================== 接口 ============================== */

/* 在硬件支持时复位 XSHUT，随后校验型号、等固件、配置并启动连续测距。
 * 全部成功返回 HAL_OK；任何一步失败返回 HAL_ERROR/HAL_TIMEOUT，此时不要再去读距离。 */
HAL_StatusTypeDef Tof_Init(void);

/* 读型号 ID（寄存器 0x010F）。VL53L1X 应返回 0xEACC。 */
HAL_StatusTypeDef Tof_ModelId(uint16_t *model_id);

/* 读一次距离。内部会等数据就绪（最多 TOF_READY_TIMEOUT_MS），再清中断。
 * 返回 HAL_OK 时 *distance_mm 有效，*range_status 是 VL53L1X 的距离状态码
 * （0 = 有效，其它值表示信号弱/超量程等）。range_status 允许传 NULL。 */
HAL_StatusTypeDef Tof_ReadMm(uint16_t *distance_mm, uint8_t *range_status);

/* 一次带滤波的测量结果。 */
typedef struct {
  uint16_t distance_mm;            /* 滤波后的距离；还没有有效样本时为 TOF_DISTANCE_INVALID_MM */
  uint16_t distance_raw_mm;        /* 本次新测到的原始距离 */
  uint16_t signal_kcps_per_spad;   /* 信号率：目标反射的强度，越大反射越强 */
  uint16_t ambient_kcps_per_spad;  /* 环境光率：越大说明环境越亮、噪声也越大 */
  uint8_t  range_status;           /* 本次原始状态码，0 = 有效 */
  uint8_t  window_counts;          /* 中值窗口里当前的有效样本数（0..TOF_MEDIAN_WINDOW） */
} TofReading;

/* 读一次并做中值 + 滑动平均滤波（参数见文件顶部"精度相关"一节）。
 * 返回 HAL_OK 表示拿到了新测量，此时 range_status 可能是无效码：那一次不进滤波窗口，
 * distance_mm 保持上一个有效滤波值。返回 HAL_ERROR/HAL_TIMEOUT 表示总线或超时问题。
 * 只吃 RangeStatus == 0 的样本，所以坏点不会污染滤波结果。 */
HAL_StatusTypeDef Tof_ReadFiltered(TofReading *reading);

/* 清空滤波窗口。换目标或挪安装位置后调用，避免旧样本混进新结果；Tof_Init() 内部会调用。 */
void Tof_ResetFilter(void);

/* 1 = 拉低 XSHUT，0 = 释放；SHUT 未连接的硬件上此接口只清软件就绪状态。 */
void Tof_Shutdown(uint8_t shutdown);

/* 读回 XSHUT 引脚的实际电平：1 = 高（已被上拉释放），0 = 低（被按在复位里）。
 * 用来诊断"开漏输出没有外部上拉"这个坑。 */
uint8_t Tof_ShutLineHigh(void);

/* 当前硬件是否把 MCU 控制脚实际连接到了模块 SHUT。 */
uint8_t Tof_ShutControllable(void);

/* 上面 Tof_Init() 是否已经成功过。 */
uint8_t Tof_IsReady(void);

#endif
