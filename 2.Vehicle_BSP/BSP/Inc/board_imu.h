#ifndef BOARD_IMU_H
#define BOARD_IMU_H
#include "board.h"

/* HWT101 陀螺仪（维特智能），USART2 / PA2/PA3 / 115200（`SERIAL_IMU`）。
 *
 * 只解析 `0x55 0x53`（角度）帧取 yaw；`0x51`（加速度）/`0x52`（角速度）帧按长度吞掉。
 * 拼帧和校验在 `Imu_Poll()`（主循环）里做，串口 ISR 只入队 —— 见 AGENTS.md 第 6 节。
 *
 * 提供的 yaw：**已减零点、已换向**，正值 = 车体逆时针，单位 0.001 度。
 * 模块自己的 yaw 方向不一定是逆时针，由下面的 `IMU_YAW_SIGN` 统一。 */

/* ---- 参数区 ---- */
#define IMU_TIMEOUT_MS          500U   /* 超过这么久没有新帧就认为掉线 */
#define IMU_YAW_SIGN              1    /* 模块 yaw 增大对应逆时针时为 1，反了改 -1（不用动线） */
#define IMU_FRAME_LEN            11U   /* 维特 0x55 帧固定 11 字节 */
#define IMU_MAX_PERIOD_MS       200U   /* 相邻两帧间隔超过这个值就不算角速度，防跳变 */

#if (IMU_TIMEOUT_MS < 50U || IMU_TIMEOUT_MS > 5000U)
#error IMU_TIMEOUT_MS must be 50..5000
#endif
#if (IMU_YAW_SIGN != 1 && IMU_YAW_SIGN != -1)
#error IMU_YAW_SIGN must be 1 or -1
#endif

typedef struct {
  int32_t  yaw_mdeg;       /* 已减零点、已换向，0.001 deg，不 wrap（会超过 ±180） */
  int32_t  yaw_rate_mdps;  /* 相邻两帧差分的角速度，0.001 deg/s */
  uint32_t frames;         /* 校验通过的帧数 */
  uint32_t bad_frames;     /* 校验失败的分片数 */
  uint32_t age_ms;         /* 距最后一帧多久 */
  uint8_t  valid;          /* 至少收到过一帧 */
  uint8_t  fresh;          /* age_ms <= IMU_TIMEOUT_MS */
} ImuSnapshot;

/* 主循环调用：把 USART2 里收到的字节拼成帧。每轮调一次即可。 */
void Imu_Poll(void);

/* 原子读一份快照（返回 0 = 还没收到过帧）。 */
uint8_t Imu_Read(ImuSnapshot *out);

int32_t Imu_ReadYawMdeg(void);
int32_t Imu_ReadRateMdps(void);
uint8_t Imu_IsFresh(void);
uint8_t Imu_HasFrame(void);

/* 从 USART2 收到的**原始字节数**。用来区分"线上一个字节都没有"（接线/供电/波特率）
 * 和"有字节但拼不出帧"（协议不对/波特率不对/模块不在自动上报模式）。 */
uint32_t Imu_RawByteCount(void);

/* 把当前 yaw 记成 0（TASK 起点、`Z` 命令用）。 */
void Imu_Zero(void);

#endif
