#ifndef BOARD_VISION_H
#define BOARD_VISION_H

#include <stdint.h>

/*
 * 视觉目标流接收（树莓派 → USART3）。
 *
 * 分工：树莓派负责"看图"，本模块只负责"把一行目标数据变成一份可读的快照"，
 *       不做任何运动决策（选哪个目标、怎么对位属 App 层）。
 *
 * 链路事实（2026-10-30）：
 *   树莓派 4B --USB/CSI 相机--> OpenCV --USART3(PC10/PC11, 115200)--> 本模块
 *   USART3 与任务码链路是同一条线（SERIAL_RPI），所以二维码 TASK 行和
 *   视觉目标行共用一路串口：本模块只认 '$' 开头的行，其余行一律忽略，
 *   不会吃掉 hmi/任务码那一路的解析（那条链路由 Tests/Src/hmi_smoke.c 处理）。
 *
 * 协议（ASCII 单行，LF 结尾，字段全为整数，无浮点）：
 *   $V,<seq>,<cls>,<x_mm>,<y_mm>,<yaw_mdeg>,<conf_permille>,<age_ms>*<crc8>
 *
 *   seq             视觉侧帧序号，逐帧 +1，用于去重和丢帧统计
 *   cls             0 = 本帧无目标；1..6 = 颜色（与任务码一致：1红2黄3蓝4绿5黑6浅蓝）
 *                   10 = 载物盘；11 = 色环
 *   x_mm            车体坐标，车头正前方为正
 *   y_mm            车体坐标，车体左侧为正（右手系：x 前、y 左、z 上）
 *   yaw_mdeg        目标主轴与车体 x 轴夹角，逆时针为正，单位 0.001 度
 *   conf_permille   置信度，0..1000
 *   age_ms          视觉侧从采集到发出这一行的耗时（帧龄）
 *   crc8            CRC-8/ATM（多项式 0x07，初值 0x00），
 *                   计算范围是 '$' 之后、'*' 之前的全部字符
 *
 * 示例（形状示例，校验值以实际计算为准）：
 *   $V,1024,3,-42,315,1234,920,38*4E
 *   $V,1025,0,0,0,0,0,35*7A
 */

/* ============================== 参数区 ============================== */

/* 一行最多多少字节（含 '\n' 之前的全部字符）。正常一行约 45 字节，留一倍余量。 */
#define VD_LINE_BYTES                 64U
/* 一次 Vision_Poll() 最多消化多少字节。主循环每轮调用一次，
 * 15 Hz × 45 字节 ≈ 675 B/s，115200 下每轮消化 64 B 绰绰有余。 */
#define VD_DRAIN_BYTES                64U
/* 允许的目标类别上限。cls 超过它的行直接判坏帧，防止脏数据驱动下游。 */
#define VD_CLASS_MAX                  20U
/* 车体坐标的合理量程，单位 mm。超出说明标定或协议出错，整行丢弃。 */
#define VD_TARGET_LIMIT_MM          2000
/* 偏航角量程，单位 0.001 度（±180°）。 */
#define VD_YAW_LIMIT_MDEG         180000L
/* 置信度上限，单位千分比。 */
#define VD_CONF_MAX_PERMILLE        1000U
/* 帧龄上限，单位 ms。超过说明视觉侧卡顿，整行丢弃。 */
#define VD_AGE_LIMIT_MS             2000U
/* 同一 seq 重发视为"视觉侧重发上一帧"，只计数、不刷新时间戳；
 * 但如果距上一帧已超过这么久，就认为视觉侧重启过（seq 归零），允许重新接受。 */
#define VD_RESTART_GAP_MS           1000U
/* 应用层判断目标是否可用的默认新鲜度上限，单位 ms。0.6 m/s 下 200 ms 约走 120 mm，
 * 超过这个值就应该降速或退回里程计，不能再拿它做闭环。 */
#define VD_STALE_MS                  200U

#if (VD_LINE_BYTES < 48U || VD_LINE_BYTES > 128U)
#error VD_LINE_BYTES must be within 48..128
#endif
#if (VD_DRAIN_BYTES < 16U || VD_DRAIN_BYTES > 256U)
#error VD_DRAIN_BYTES must be within 16..256
#endif
#if (VD_CLASS_MAX < 6U || VD_CLASS_MAX > 63U)
#error VD_CLASS_MAX must be within 6..63
#endif
#if (VD_TARGET_LIMIT_MM < 200 || VD_TARGET_LIMIT_MM > 10000)
#error VD_TARGET_LIMIT_MM must be within 200..10000
#endif
#if (VD_YAW_LIMIT_MDEG != 180000L)
#error VD_YAW_LIMIT_MDEG must stay 180000 (180 deg)
#endif
#if (VD_CONF_MAX_PERMILLE != 1000U)
#error VD_CONF_MAX_PERMILLE must stay 1000
#endif
#if (VD_AGE_LIMIT_MS < 100U || VD_AGE_LIMIT_MS > 10000U)
#error VD_AGE_LIMIT_MS must be within 100..10000
#endif
#if (VD_STALE_MS > VD_AGE_LIMIT_MS)
#error VD_STALE_MS must not exceed VD_AGE_LIMIT_MS
#endif

/* ============================== 接口 ============================== */

/* 最近一帧被接受的目标。valid = 0 表示视觉侧明确报告"视野里没有目标"，
 * 与"链路断了"（stamp_ms 不再更新）是两回事，下游要分开处理。 */
typedef struct {
  uint32_t seq;                    /* 视觉侧帧序号 */
  uint32_t stamp_ms;               /* 本帧被接受的时刻（Board 时基） */
  int32_t  yaw_mdeg;               /* 目标主轴相对车体 x 轴，逆时针为正 */
  int16_t  x_mm;                   /* 车体坐标：前向距离 */
  int16_t  y_mm;                   /* 车体坐标：左向偏移 */
  uint16_t class_code;             /* 0 无目标 / 1..6 颜色 / 10 载物盘 / 11 色环 */
  uint16_t confidence_permille;    /* 视觉侧置信度 */
  uint16_t age_ms;                 /* 视觉侧帧龄 */
  uint8_t  valid;                  /* 1 = 本帧有目标 */
} VisionTarget;

/* 链路统计。用来区分"没识别到"和"收到了但校验失败"。 */
typedef struct {
  uint32_t frames_ok;              /* 通过 CRC 与量程检查的帧 */
  uint32_t frames_bad;             /* 帧头/字段/CRC/量程任一不过的帧 */
  uint32_t frames_repeat;          /* 与上一帧同 seq（视觉侧重发） */
  uint32_t frames_gap;             /* 按 seq 推断出的丢失帧累计 */
  uint32_t bytes_overflow;         /* 超过 VD_LINE_BYTES 被丢弃的行数 */
  uint32_t bytes_dropped;          /* 串口环形缓冲溢出（来自 Serial_Dropped） */
} VisionStats;

/* 清空快照与统计。第一次调用 Vision_Poll() 之前必须先调它。 */
void Vision_Init(void);

/* 主循环里周期调用：把 SERIAL_RPI 收到的字节拼成行并解析。
 * 不发送任何数据、不阻塞、不含 HAL_Delay，可以在每轮主循环里直接调。 */
void Vision_Poll(void);

/* 读最近一帧（只读，返回内部静态地址，不要保存指针后跨帧使用）。 */
const VisionTarget *Vision_ReadTarget(void);

/* 读链路统计。 */
void Vision_ReadStats(VisionStats *stats);

/* 目标是否新鲜：距该帧被接受不超过 max_age_ms。回绕安全。 */
uint8_t Vision_TargetFresh(uint32_t now_ms, uint32_t max_age_ms);

/* 非 `$` 行（例如树莓派下发的 TASK / STAT）交给应用层处理。
 * 不注册就丢弃；注册函数里不要做阻塞操作。 */
typedef void (*VisionLineHandler)(const char *line);
void Vision_SetLineHandler(VisionLineHandler handler);

#endif
