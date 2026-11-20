#ifndef _VL53L1_PLATFORM_H_
#define _VL53L1_PLATFORM_H_

/*
 * VL53L1X ULD 的平台层接口。
 *
 * 为什么这个文件在 BSP 而不在 Drivers：
 *   ST 的 ULD 源码里 VL53L1X_api.h 硬编码了 #include "vl53l1_platform.h"，
 *   文件名不能改。而平台层是"往 STM32 + 本板软件 I2C 上接"的胶水代码，
 *   属于自有代码 —— 按 AGENTS.md 第 1 节，自有代码不能放 Drivers/。
 *   所以：ULD 内核留在 Drivers/VL53L1X/（厂商代码，只读），
 *   这个平台头和 BSP/Src/board_tof.c 里的实现是我们自己的。
 *   Drivers/VL53L1X/ 里**故意不放**同名文件，避免包含路径歧义。
 *
 * 真实实现在 BSP/Src/board_tof.c。
 */

#include "vl53l1_types.h"

#define VL53L1_DEFAULT_TIMEOUT 50

#ifdef __cplusplus
extern "C" {
#endif

/* ULD 只把这个结构体当占位用；真正的器件地址由我们自己的宏传进去。 */
typedef struct {
  uint32_t dummy;
} VL53L1_Dev_t;

typedef VL53L1_Dev_t *VL53L1_DEV;

/* 以下 9 个函数由 BSP/Src/board_tof.c 实现，全部返回 0 表示成功。 */
int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count);
int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data);
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data);
int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data);
int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata);
int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata);
int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata);
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms);

#ifdef __cplusplus
}
#endif

#endif
