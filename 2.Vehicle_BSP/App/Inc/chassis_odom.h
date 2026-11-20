#ifndef CHASSIS_ODOM_H
#define CHASSIS_ODOM_H
#include <stdint.h>
#include "chassis_odom_config.h"
typedef struct {
  int32_t x_um, y_um;
  int32_t heading_mdeg;
  int32_t vx_um_per_s, vy_um_per_s;
  int32_t wz_mdps;
  uint32_t updated_ms;
} ChassisPose;
/* 诊断不扩充七通道协议，由 S 状态行查询。 */
typedef struct {
  uint8_t gyro_ready, stationary;
  uint32_t resync_events, gyro_read_errors;
  int32_t encoder_wz_mdps, gyro_pp_mdps;
} ChassisOdomStatus;
void ChassisOdom_Init(void);
void ChassisOdom_Reset(void);
void ChassisOdom_SetPose(int32_t x_um, int32_t y_um, int32_t heading_mdeg);
void ChassisOdom_Update(void);
void ChassisOdom_GetPose(ChassisPose *pose);
int32_t ChassisOdom_HeadingMdeg(void);
/* 台架重校后显式传入成功状态，失败时不沿用驱动可能保留的旧零偏。 */
void ChassisOdom_SetGyroCalibrationValid(uint8_t valid);
void ChassisOdom_GetStatus(ChassisOdomStatus *status);
#endif
