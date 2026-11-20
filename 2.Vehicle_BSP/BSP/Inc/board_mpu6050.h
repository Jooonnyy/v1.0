#ifndef BOARD_MPU6050_H
#define BOARD_MPU6050_H

#include "board.h"
#include <stdint.h>

/* DLPF 开启后陀螺仪输出速率为 1 kHz；分频 9 得到 100 Hz。 */
#define MPU6050_SAMPLE_RATE_HZ       100U
#define MPU6050_DLPF_CFG               3U
#define MPU6050_GYRO_FS_SEL            1U /* 正负 500 dps。 */
#define MPU6050_ACCEL_FS_SEL           1U /* 正负 4 g。 */
#define MPU6050_RESET_DELAY_MS        100U

/* WHO_AM_I 允许的值。这颗传感器有一堆脚位与寄存器兼容的型号，市面上标着"MPU6050"
 * 的廉价模块里塞 6500 很常见，所以不能只认 0x68。
 *   0x68 = MPU6050 / MPU6000 / MPU9150
 *   0x70 = MPU6500      0x71 = MPU9250      0x73 = MPU9255      0x74 = MPU6515
 * 在本驱动用到的寄存器子集上它们是兼容的。唯一的实际差异见
 * MPU6050_REG_ACCEL_CONFIG2：6500/9250 这一类有独立的加速度 DLPF，MPU6050 没有。 */
#define MPU6050_WHO_AM_I_MPU6050     0x68U
#define MPU6050_WHO_AM_I_MPU6500     0x70U
#define MPU6050_WHO_AM_I_MPU9250     0x71U
#define MPU6050_WHO_AM_I_MPU9255     0x73U
#define MPU6050_WHO_AM_I_MPU6515     0x74U

/* ------------------------------ 精度相关 ------------------------------ */

/* 输出滑动平均的窗口长度，单位个样本（1 = 关闭）。
 * 采样率 100 Hz 时窗口 8 相当于 80 ms、带宽降到十几 Hz，噪声约按 1/sqrt(N) 降。 */
#define MPU6050_AVERAGE_WINDOW          8U
/* 加速度增益修正，单位千分比（1000 = 不修正）。
 * 用法：模块静止、水平时看串口的 |a|，正确值应为 1000 mg。
 * 若稳定读成 1102，则填 1000 * 1000 / 1102 = 907。
 * 注意这是"增益"修正：只在 1 g 附近有效，量程换掉就不适用了。
 * 偏差超过 ±5% 说明片子本身的初始灵敏度不太好（MPU 系列规格是 ±3%）。 */
#define MPU6050_ACCEL_SCALE_PERMILLE 1000U
/* 陀螺仪零偏的温漂很大（最大 0.24 dps/°C）。开机校准是在芯片还凉的时候做的，
 * 跑热以后零偏会重新长出来，所以测试里提供 C 命令随时重校。
 * 校准前建议先让模块通电静置 1~2 分钟。 */
/* 静止零偏校准：采样次数，以及两次采样之间的阻塞间隔（单位 ms）。
 * 间隔必须 >= 一个采样周期（100 Hz 对应 10 ms），保证每次读到的都是新转换。 */
#define MPU6050_CALIB_SAMPLES         200U
#define MPU6050_CALIB_SAMPLE_MS        10U
/* 校准期间允许的最大运动量（原始计数，峰峰值）。超过就判定"校准时动了"，零偏不采纳。
 * 参考尺度：陀螺仪约 65.5 LSB/dps，加速度约 8192 LSB/g。 */
#define MPU6050_CALIB_MOTION_PP_LIMIT  400U

#if (MPU6050_SAMPLE_RATE_HZ < 4U || MPU6050_SAMPLE_RATE_HZ > 1000U)
#error MPU6050_SAMPLE_RATE_HZ must be within 4..1000
#endif
#if ((1000U % MPU6050_SAMPLE_RATE_HZ) != 0U)
#error MPU6050_SAMPLE_RATE_HZ must divide 1000 exactly
#endif
#if (MPU6050_DLPF_CFG < 1U || MPU6050_DLPF_CFG > 6U)
#error MPU6050_DLPF_CFG must be within 1..6
#endif
#if (MPU6050_GYRO_FS_SEL > 3U || MPU6050_ACCEL_FS_SEL > 3U)
#error MPU6050 full-scale selectors must be within 0..3
#endif
#if (MPU6050_RESET_DELAY_MS < 100U)
#error MPU6050_RESET_DELAY_MS must be at least 100 ms
#endif
/* 单位换算用的是固定系数，量程一改就会静默算错，所以在编译期拦死。 */
#if (MPU6050_GYRO_FS_SEL != 1U)
#error The mdps conversion assumes GYRO_FS_SEL = 1 (+/-500 dps, 65.5 LSB/dps); update the divisor first
#endif
#if (MPU6050_ACCEL_FS_SEL != 1U)
#error The mg conversion assumes ACCEL_FS_SEL = 1 (+/-4 g, 8192 LSB/g); update the divisor first
#endif
#if (MPU6050_AVERAGE_WINDOW < 1U || MPU6050_AVERAGE_WINDOW > 16U)
#error MPU6050_AVERAGE_WINDOW must be within 1..16
#endif
#if (MPU6050_ACCEL_SCALE_PERMILLE < 500U || MPU6050_ACCEL_SCALE_PERMILLE > 2000U)
#error MPU6050_ACCEL_SCALE_PERMILLE must be within 500..2000 (1000 = no correction)
#endif
#if (MPU6050_CALIB_SAMPLES < 50U || MPU6050_CALIB_SAMPLES > 2000U)
#error MPU6050_CALIB_SAMPLES must be within 50..2000
#endif
#if (MPU6050_CALIB_SAMPLE_MS < (1000U / MPU6050_SAMPLE_RATE_HZ))
#error MPU6050_CALIB_SAMPLE_MS must be >= one sample period, otherwise samples repeat
#endif

typedef struct {
  uint8_t sample_rate_divider;
  uint8_t config;
  uint8_t gyro_config;
  uint8_t accel_config;
  uint8_t int_pin_config;
  uint8_t int_enable;
  uint8_t power_management_1;
  uint8_t power_management_2;
} Mpu6050ConfigRegisters;

HAL_StatusTypeDef MPU6050_WhoAmI(uint8_t *device_id);

/* 这个 WHO_AM_I 值是不是本驱动支持的型号（兼容家族，见文件顶部）。 */
uint8_t MPU6050_WhoAmISupported(uint8_t device_id);

/* 把 WHO_AM_I 值翻译成一句人能看懂的话，用于串口打印；不认识时返回 "unknown"。 */
const char *MPU6050_ModelName(uint8_t device_id);
/* 软复位后配置 100 Hz、DLPF、量程和数据就绪中断，并读回全部关键寄存器。 */
HAL_StatusTypeDef MPU6050_Configure(void);
HAL_StatusTypeDef MPU6050_ReadConfig(Mpu6050ConfigRegisters *config);
HAL_StatusTypeDef MPU6050_VerifyConfig(void);
HAL_StatusTypeDef MPU6050_ReadRawCounts(int16_t accel_counts[3], int16_t gyro_counts[3],
                                        int16_t *temperature_counts);

/* ------------------------------ 校准与换算 ------------------------------ */

/* 静止零偏校准的结果，全部是整数（不使用浮点）。 */
typedef struct {
  int16_t  gyro_bias_counts[3];  /* 陀螺仪零偏，单位原始计数；ReadScaled() 内部减的就是它 */
  int32_t  gyro_bias_mdps[3];    /* 同上，换算成 0.001 dps 便于读数 */
  int32_t  accel_rest_mg[3];     /* 校准期间的加速度均值：模块水平放置时应约为 [0,0,1000] */
  uint16_t motion_pp_counts;     /* 校准期间三轴里最大的峰峰值（原始计数） */
  uint8_t  calibrated;           /* 1 = 零偏有效并已生效；0 = 检测到运动，未采纳 */
} Mpu6050Bias;

/* 一次换算后的读数。 */
typedef struct {
  int32_t  accel_mg[3];          /* 毫克，1 g = 1000 mg */
  int32_t  gyro_mdps[3];         /* 0.001 dps，已扣除零偏 */
  int32_t  temperature_mdeg_c;   /* 0.001 摄氏度 */
  uint16_t accel_magnitude_mg;   /* 三轴合矢量模长：静止、水平时应约为 1000 mg */
  uint8_t  averaged_counts;      /* 本次平均用到的样本数（1..MPU6050_AVERAGE_WINDOW） */
} Mpu6050Reading;

/* 静止零偏校准：连续采 samples 次原始读数求均值，得到陀螺仪零偏并立即生效。
 * 期间模块必须静止；检测到运动时 calibrated = 0 且零偏全部置 0（宁可不补，也不补错）。
 * 采样之间有 MPU6050_CALIB_SAMPLE_MS 的阻塞延时，整个调用约
 * samples * MPU6050_CALIB_SAMPLE_MS 毫秒，只应在初始化阶段调用。 */
HAL_StatusTypeDef MPU6050_CalibrateBias(Mpu6050Bias *bias, uint16_t samples);

/* 读一次，做滑动平均 → 扣零偏 → 换算成 mg / mdps / mdegC。 */
HAL_StatusTypeDef MPU6050_ReadScaled(Mpu6050Reading *reading);

/* 清空滑动平均窗口（重新校准、或改了安装姿态后调用）。 */
void MPU6050_ResetFilter(void);

/* 查询当前生效的零偏；从未校准时 calibrated 为 0 且各偏置为 0。 */
const Mpu6050Bias *MPU6050_GetBias(void);

#endif
