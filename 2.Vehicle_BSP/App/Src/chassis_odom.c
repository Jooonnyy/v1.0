#include "chassis_odom.h"
#include "board_encoder.h"
#include "board_mpu6050.h"
#include <limits.h>
#include <string.h>

/* 四轮跨器件融合放在 App；BSP 只供应计数窗口和传感器读数。
 * 查表常数由离线工具生成，固件仅使用整数。正端点取 32767 避免 int16 溢出。 */
static const int16_t sin_table_q15[257] = {
  0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210,
  2410, 2611, 2811, 3012, 3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609,
  4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195, 6393, 6590, 6786, 6983,
  7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319,
  9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605,
  11793, 11980, 12167, 12353, 12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828,
  14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269, 15446, 15623, 15800, 15976,
  16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037,
  18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000,
  20159, 20317, 20475, 20631, 20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856,
  22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027, 23170, 23311, 23452, 23592,
  23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201,
  25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674,
  26790, 26905, 27019, 27133, 27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001,
  28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803, 28898, 28992, 29085, 29177,
  29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195,
  30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050,
  31113, 31176, 31237, 31297, 31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736,
  31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098, 32137, 32176, 32213, 32250,
  32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589,
  32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752,
  32757, 32761, 32765, 32766, 32767,
};
static const uint8_t wheel_corner[4] = {WHEEL_CORNER_A, WHEEL_CORNER_B, WHEEL_CORNER_C, WHEEL_CORNER_D};
static const int8_t wheel_sign[4] = {ODOM_SIGN_WHEEL_A, ODOM_SIGN_WHEEL_B, ODOM_SIGN_WHEEL_C, ODOM_SIGN_WHEEL_D};
static volatile ChassisPose published_pose;
static volatile ChassisOdomStatus published_status;
static ChassisPose working_pose;
static ChassisOdomStatus working_status;
static EncoderWindow window;
static int64_t wheel_remainder[4], kinematic_remainder[3];
static int64_t rotation_remainder[2], gyro_remainder;
static uint32_t next_update_ms, last_sample_ms, encoder_generation;
static uint32_t quiet_ms;
static uint8_t gyro_calibration_valid;
static uint32_t noise_index, noise_count;
#if ODOM_USE_GYRO
static int32_t gyro_noise_mdps[ODOM_NOISE_SAMPLES];
static Mpu6050Reading gyro_reading;
static uint32_t last_gyro_irq, last_gyro_read_ms;
#endif

static int32_t saturate_i32(int64_t value)
{
  if (value > INT32_MAX) return INT32_MAX;
  if (value < INT32_MIN) return INT32_MIN;
  return (int32_t)value;
}
static int32_t normalize_mdeg(int64_t heading_mdeg)
{
  heading_mdeg %= 360000;
  if (heading_mdeg >= 180000) heading_mdeg -= 360000;
  if (heading_mdeg < -180000) heading_mdeg += 360000;
  return (int32_t)heading_mdeg;
}
static int32_t quarter_sin_q15(int32_t angle_mdeg)
{
  uint32_t index_q8;
  int32_t base_q15, next_q15;
  if (angle_mdeg >= 90000) return 32767;
  /* 原设计少了 Q8 小数的乘 256；这里保留小数再线性插值。 */
  index_q8 = (uint32_t)((int64_t)angle_mdeg * 65536 / 90000);
  base_q15 = sin_table_q15[index_q8 >> 8];
  next_q15 = sin_table_q15[(index_q8 >> 8) + 1U];
  return base_q15 + (int32_t)(((next_q15 - base_q15) * (int32_t)(index_q8 & 255U)) / 256);
}
static int32_t sin_q15(int32_t angle_mdeg)
{
  int32_t positive_mdeg = normalize_mdeg(angle_mdeg);
  int32_t result_q15;
  uint8_t negative = positive_mdeg < 0;
  if (negative) positive_mdeg = -positive_mdeg;
  if (positive_mdeg > 90000) positive_mdeg = 180000 - positive_mdeg;
  result_q15 = quarter_sin_q15(positive_mdeg);
  return negative ? -result_q15 : result_q15;
}
static void publish(void)
{
  uint32_t saved_mask = Board_EnterCritical();
  published_pose = working_pose;
  published_status = working_status;
  Board_ExitCritical(saved_mask);
}
static void clear_remainders(void)
{
  memset(wheel_remainder, 0, sizeof(wheel_remainder));
  memset(kinematic_remainder, 0, sizeof(kinematic_remainder));
  memset(rotation_remainder, 0, sizeof(rotation_remainder));
  gyro_remainder = 0;
}
void ChassisOdom_SetPose(int32_t x_um, int32_t y_um, int32_t heading_mdeg)
{
  Encoder_TakeWindow(&window);
  encoder_generation = window.generation;
  last_sample_ms = window.sampled_ms;
  clear_remainders();
  quiet_ms = 0U; noise_count = 0U; noise_index = 0U;
  memset(&working_pose, 0, sizeof(working_pose));
  working_pose.x_um = x_um; working_pose.y_um = y_um;
  working_pose.heading_mdeg = normalize_mdeg(heading_mdeg);
  working_pose.updated_ms = Board_NowMs();
  working_status.stationary = 0U;
  working_status.gyro_ready = 0U;
  working_status.gyro_pp_mdps = 0;
  working_status.encoder_wz_mdps = 0;
#if ODOM_USE_GYRO
  last_gyro_irq = Input_EventCount(INPUT_MPU_IRQ);
  last_gyro_read_ms = working_pose.updated_ms;
  MPU6050_ResetFilter();
#endif
  next_update_ms = working_pose.updated_ms + ODOM_UPDATE_MS;
  publish();
}
void ChassisOdom_Reset(void) { ChassisOdom_SetPose(0, 0, 0); }
void ChassisOdom_Init(void)
{
  memset(&working_status, 0, sizeof(working_status));
  /* 只在初始化阻塞校准，不使能任何运动输出。 */
#if ODOM_USE_GYRO
  static Mpu6050Bias bias;
  gyro_calibration_valid = 0U;
  if (MPU6050_Configure() == HAL_OK &&
      MPU6050_CalibrateBias(&bias, MPU6050_CALIB_SAMPLES) == HAL_OK && bias.calibrated)
    gyro_calibration_valid = 1U;
  MPU6050_ResetFilter();
#endif
  ChassisOdom_Reset();
}
void ChassisOdom_SetGyroCalibrationValid(uint8_t valid)
{
  gyro_calibration_valid = valid ? 1U : 0U;
  working_status.gyro_ready = 0U;
  working_status.stationary = 0U;
  gyro_remainder = 0; quiet_ms = 0U; noise_index = 0U; noise_count = 0U;
  publish();
}
void ChassisOdom_GetPose(ChassisPose *pose)
{
  uint32_t saved_mask;
  if (pose == NULL) return;
  saved_mask = Board_EnterCritical(); *pose = published_pose; Board_ExitCritical(saved_mask);
}
void ChassisOdom_GetStatus(ChassisOdomStatus *status)
{
  uint32_t saved_mask;
  if (status == NULL) return;
  saved_mask = Board_EnterCritical(); *status = published_status; Board_ExitCritical(saved_mask);
}
int32_t ChassisOdom_HeadingMdeg(void) { return published_pose.heading_mdeg; }

#if ODOM_USE_GYRO
/* 轮速与窗口峰值双判据，不让低通残速或正反抵消冒充静止。 */
static uint8_t wheels_quiet(void)
{
  for (uint32_t i = 0U; i < 4U; ++i) {
    int32_t speed_counts_per_s = Encoder_SpeedCountsPerSec((EncoderId)i);
    if (speed_counts_per_s < -ODOM_STILL_COUNTS_PER_S || speed_counts_per_s > ODOM_STILL_COUNTS_PER_S ||
        (uint64_t)window.peak_delta_counts[i] * ENCODER_TICK_HZ > ODOM_STILL_COUNTS_PER_S) return 0U;
  }
  return 1U;
}
static int32_t gyro_heading_delta_mdeg(uint32_t elapsed_ms)
{
  int32_t wz_mdps, low_mdps, high_mdps;
  uint32_t irq = Input_EventCount(INPUT_MPU_IRQ), now_ms = Board_NowMs();
  uint8_t wheel_still = wheels_quiet();
  working_status.stationary = 0U;
  working_status.gyro_ready = 0U;
  working_pose.wz_mdps = 0;
  /* 数据就绪计数优先；INT 未接时最多按传感器采样周期轮询一次。
   * 没有新读数不能把旧角速度跨窗口积分。 */
  if (!gyro_calibration_valid || !MPU6050_GetBias()->calibrated ||
      (irq == last_gyro_irq && (int32_t)(now_ms - (last_gyro_read_ms + 1000U / MPU6050_SAMPLE_RATE_HZ)) < 0))
    goto invalid;
  last_gyro_irq = irq; last_gyro_read_ms = now_ms;
  if (MPU6050_ReadScaled(&gyro_reading) != HAL_OK) {
    working_status.gyro_read_errors++; goto invalid;
  }
  wz_mdps = saturate_i32((int64_t)gyro_reading.gyro_mdps[2] * GYRO_Z_SIGN * GYRO_SCALE_PERMILLE / 1000);
  gyro_noise_mdps[noise_index] = wz_mdps;
  noise_index = (noise_index + 1U) % ODOM_NOISE_SAMPLES;
  if (noise_count < ODOM_NOISE_SAMPLES) noise_count++;
  low_mdps = wz_mdps; high_mdps = wz_mdps;
  for (uint32_t i = 0U; i < noise_count; ++i) {
    if (gyro_noise_mdps[i] < low_mdps) low_mdps = gyro_noise_mdps[i];
    if (gyro_noise_mdps[i] > high_mdps) high_mdps = gyro_noise_mdps[i];
  }
  working_status.gyro_pp_mdps = saturate_i32((int64_t)high_mdps - low_mdps);
  /* 轮子静止而角速度噪声超限，拒绝该航向窗口并报告不可用。
   * 这不是校准成功的证据，也无法辨别打滑时的外部转动。 */
  if (wheel_still && noise_count == ODOM_NOISE_SAMPLES && working_status.gyro_pp_mdps > ODOM_STILL_PP_MDPS)
    goto invalid;
  working_status.gyro_ready = 1U;
  if (wheel_still && wz_mdps >= -ODOM_STILL_GYRO_MDPS && wz_mdps <= ODOM_STILL_GYRO_MDPS &&
      working_status.gyro_pp_mdps <= ODOM_STILL_PP_MDPS) {
    if (quiet_ms < ODOM_STILL_MS) quiet_ms += elapsed_ms;
    if (quiet_ms >= ODOM_STILL_MS && noise_count == ODOM_NOISE_SAMPLES) {
      working_status.stationary = 1U; wz_mdps = 0; gyro_remainder = 0;
    }
  } else quiet_ms = 0U;
  if (wz_mdps >= -ODOM_GYRO_DEADZONE_MDPS && wz_mdps <= ODOM_GYRO_DEADZONE_MDPS) wz_mdps = 0;
  working_pose.wz_mdps = wz_mdps;
  gyro_remainder += (int64_t)wz_mdps * elapsed_ms;
  wz_mdps = (int32_t)(gyro_remainder / 1000);
  gyro_remainder %= 1000;
  return wz_mdps;
invalid:
  quiet_ms = 0U; gyro_remainder = 0;
  return 0;
}
#endif

/* 中点航向旋转，余数跨拍保留，避免低速截断长期积累。 */
static void integrate_pose(int32_t dx_local_um, int32_t dy_local_um, int32_t delta_heading_mdeg)
{
  int32_t midpoint_mdeg = normalize_mdeg((int64_t)working_pose.heading_mdeg + delta_heading_mdeg / 2);
  int32_t s_q15 = sin_q15(midpoint_mdeg), c_q15 = sin_q15(midpoint_mdeg + 90000);
  rotation_remainder[0] += (int64_t)dx_local_um * c_q15 - (int64_t)dy_local_um * s_q15;
  rotation_remainder[1] += (int64_t)dx_local_um * s_q15 + (int64_t)dy_local_um * c_q15;
  working_pose.x_um = saturate_i32((int64_t)working_pose.x_um + rotation_remainder[0] / 32768);
  working_pose.y_um = saturate_i32((int64_t)working_pose.y_um + rotation_remainder[1] / 32768);
  rotation_remainder[0] %= 32768; rotation_remainder[1] %= 32768;
  working_pose.heading_mdeg = normalize_mdeg((int64_t)working_pose.heading_mdeg + delta_heading_mdeg);
}
void ChassisOdom_Update(void)
{
  uint32_t now_ms = Board_NowMs(), elapsed_ms;
  int32_t ds_um[4], dx_local_um, dy_local_um, spin_um, delta_heading_mdeg;
  int64_t circumference_um = (int64_t)WHEEL_EFFECTIVE_DIAMETER_UM * 355 / 113;
  if ((int32_t)(now_ms - next_update_ms) < 0) return;
  next_update_ms = now_ms + ODOM_UPDATE_MS;
  Encoder_TakeWindow(&window);
  elapsed_ms = window.sampled_ms - last_sample_ms;
  if (!window.ticks) return;
  last_sample_ms = window.sampled_ms;
  if (window.generation != encoder_generation || !elapsed_ms || elapsed_ms > ODOM_MAX_GAP_MS) goto resync;
  for (uint32_t i = 0U; i < 4U; ++i) {
    if ((int64_t)window.peak_delta_counts[i] * circumference_um >
        (int64_t)ODOM_RESYNC_MAX_UM * WHEEL_COUNTS_PER_REV) goto resync;
    wheel_remainder[i] += (int64_t)window.delta_counts[i] * wheel_sign[i] * circumference_um;
    ds_um[wheel_corner[i]] = saturate_i32(wheel_remainder[i] / WHEEL_COUNTS_PER_REV);
    wheel_remainder[i] %= WHEEL_COUNTS_PER_REV;
  }
  /* 与逆解自洽：四轮和=4vx；-FL+FR+RL-RR=4vy；-FL+FR-RL+RR=4Lω。 */
  kinematic_remainder[0] += (int64_t)ds_um[0] + ds_um[1] + ds_um[2] + ds_um[3];
  kinematic_remainder[1] += ((int64_t)-ds_um[0] + ds_um[1] + ds_um[2] - ds_um[3]) * ODOM_LATERAL_SIGN;
  kinematic_remainder[2] += (int64_t)-ds_um[0] + ds_um[1] - ds_um[2] + ds_um[3];
  dx_local_um = (int32_t)(kinematic_remainder[0] / 4);
  dy_local_um = (int32_t)(kinematic_remainder[1] / 4);
  spin_um = (int32_t)(kinematic_remainder[2] / 4);
  for (uint32_t i = 0U; i < 3U; ++i) kinematic_remainder[i] %= 4;
  working_pose.vx_um_per_s = saturate_i32((int64_t)dx_local_um * 1000 / elapsed_ms);
  working_pose.vy_um_per_s = saturate_i32((int64_t)dy_local_um * 1000 / elapsed_ms);
  working_status.encoder_wz_mdps = saturate_i32((int64_t)spin_um * 57296 * 1000 /
                                              (CHASSIS_HALF_LENGTH_UM + CHASSIS_HALF_WIDTH_UM) / elapsed_ms);
#if ODOM_USE_GYRO
  delta_heading_mdeg = gyro_heading_delta_mdeg(elapsed_ms);
#else
  /* 编码器隔离验证保留正解航向；正常模式不自动回退到轮子航向。 */
  working_pose.wz_mdps = 0;
  delta_heading_mdeg = (int32_t)((int64_t)spin_um * 57296 /
                               (CHASSIS_HALF_LENGTH_UM + CHASSIS_HALF_WIDTH_UM));
#endif
  integrate_pose(dx_local_um, dy_local_um, delta_heading_mdeg);
  working_pose.updated_ms = window.sampled_ms; publish(); return;
resync:
  encoder_generation = window.generation;
  clear_remainders(); quiet_ms = 0U; noise_count = 0U; noise_index = 0U;
  working_status.resync_events++;
  working_status.stationary = 0U;
  working_status.gyro_ready = 0U;
  working_pose.vx_um_per_s = 0; working_pose.vy_um_per_s = 0; working_pose.wz_mdps = 0;
  working_pose.updated_ms = window.sampled_ms; publish();
}
