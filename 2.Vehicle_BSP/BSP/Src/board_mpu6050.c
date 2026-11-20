#include "board_mpu6050.h"
#include "pins/mpu6050_mpu1.h"

#define MPU6050_REG_SMPLRT_DIV       0x19U
#define MPU6050_REG_CONFIG           0x1AU
#define MPU6050_REG_GYRO_CONFIG      0x1BU
#define MPU6050_REG_ACCEL_CONFIG     0x1CU
/* 只有 MPU6500 / 9250 这一类才有：独立的加速度数字低通。MPU6050 上这个地址是保留的。 */
#define MPU6050_REG_ACCEL_CONFIG2    0x1DU
#define MPU6050_REG_FIFO_EN          0x23U
#define MPU6050_REG_INT_PIN_CFG      0x37U
#define MPU6050_REG_INT_ENABLE       0x38U
#define MPU6050_REG_INT_STATUS       0x3AU
#define MPU6050_REG_ACCEL_XOUT_H     0x3BU
#define MPU6050_REG_USER_CTRL        0x6AU
#define MPU6050_REG_PWR_MGMT_1       0x6BU
#define MPU6050_REG_PWR_MGMT_2       0x6CU
#define MPU6050_REG_WHO_AM_I         0x75U

#define MPU6050_DEVICE_RESET         0x80U
#define MPU6050_CLOCK_PLL_X_GYRO     0x01U
#define MPU6050_DATA_READY_INT       0x01U
#define MPU6050_SAMPLE_RATE_DIVIDER ((1000U / MPU6050_SAMPLE_RATE_HZ) - 1U)

static HAL_StatusTypeDef write_register(uint8_t address, uint8_t value)
{
  return Board_I2C_Write(MPU1_I2C_ADDR,address,1U,&value,1U);
}

static HAL_StatusTypeDef read_register(uint8_t address, uint8_t *value)
{
  if (value==NULL) return HAL_ERROR;
  return Board_I2C_Read(MPU1_I2C_ADDR,address,1U,value,1U);
}

HAL_StatusTypeDef MPU6050_WhoAmI(uint8_t *device_id)
{
  return read_register(MPU6050_REG_WHO_AM_I,device_id);
}

uint8_t MPU6050_WhoAmISupported(uint8_t device_id)
{
  switch (device_id) {
    case MPU6050_WHO_AM_I_MPU6050:
    case MPU6050_WHO_AM_I_MPU6500:
    case MPU6050_WHO_AM_I_MPU9250:
    case MPU6050_WHO_AM_I_MPU9255:
    case MPU6050_WHO_AM_I_MPU6515:
      return 1U;
    default:
      return 0U;
  }
}

const char *MPU6050_ModelName(uint8_t device_id)
{
  switch (device_id) {
    case MPU6050_WHO_AM_I_MPU6050: return "MPU6050/6000/9150 class";
    case MPU6050_WHO_AM_I_MPU6500: return "MPU6500 class";
    case MPU6050_WHO_AM_I_MPU9250: return "MPU9250 class";
    case MPU6050_WHO_AM_I_MPU9255: return "MPU9255 class";
    case MPU6050_WHO_AM_I_MPU6515: return "MPU6515 class";
    default:                       return "unknown";
  }
}

HAL_StatusTypeDef MPU6050_ReadConfig(Mpu6050ConfigRegisters *config)
{
  if (config==NULL) return HAL_ERROR;
  if (read_register(MPU6050_REG_SMPLRT_DIV,&config->sample_rate_divider)!=HAL_OK ||
      read_register(MPU6050_REG_CONFIG,&config->config)!=HAL_OK ||
      read_register(MPU6050_REG_GYRO_CONFIG,&config->gyro_config)!=HAL_OK ||
      read_register(MPU6050_REG_ACCEL_CONFIG,&config->accel_config)!=HAL_OK ||
      read_register(MPU6050_REG_INT_PIN_CFG,&config->int_pin_config)!=HAL_OK ||
      read_register(MPU6050_REG_INT_ENABLE,&config->int_enable)!=HAL_OK ||
      read_register(MPU6050_REG_PWR_MGMT_1,&config->power_management_1)!=HAL_OK ||
      read_register(MPU6050_REG_PWR_MGMT_2,&config->power_management_2)!=HAL_OK) return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_VerifyConfig(void)
{
  Mpu6050ConfigRegisters config;
  if (MPU6050_ReadConfig(&config)!=HAL_OK) return HAL_ERROR;
  if (config.sample_rate_divider!=(uint8_t)MPU6050_SAMPLE_RATE_DIVIDER ||
      (config.config&0x07U)!=(uint8_t)MPU6050_DLPF_CFG ||
      (config.gyro_config&0x18U)!=(uint8_t)(MPU6050_GYRO_FS_SEL<<3) ||
      (config.accel_config&0x18U)!=(uint8_t)(MPU6050_ACCEL_FS_SEL<<3) ||
      config.int_pin_config!=0x00U ||
      config.int_enable!=MPU6050_DATA_READY_INT ||
      config.power_management_1!=MPU6050_CLOCK_PLL_X_GYRO ||
      config.power_management_2!=0x00U) return HAL_ERROR;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_Configure(void)
{
  uint8_t device_id=0U;
  uint8_t int_status=0U;

  if (write_register(MPU6050_REG_PWR_MGMT_1,MPU6050_DEVICE_RESET)!=HAL_OK) return HAL_ERROR;
  HAL_Delay(MPU6050_RESET_DELAY_MS);
  if (MPU6050_WhoAmI(&device_id)!=HAL_OK || MPU6050_WhoAmISupported(device_id)==0U) return HAL_ERROR;

  /* 先唤醒并选择 X 轴陀螺仪 PLL，再配置采样链和中断。 */
  if (write_register(MPU6050_REG_PWR_MGMT_1,MPU6050_CLOCK_PLL_X_GYRO)!=HAL_OK ||
      write_register(MPU6050_REG_PWR_MGMT_2,0x00U)!=HAL_OK ||
      write_register(MPU6050_REG_USER_CTRL,0x00U)!=HAL_OK ||
      write_register(MPU6050_REG_FIFO_EN,0x00U)!=HAL_OK ||
      write_register(MPU6050_REG_SMPLRT_DIV,(uint8_t)MPU6050_SAMPLE_RATE_DIVIDER)!=HAL_OK ||
      write_register(MPU6050_REG_CONFIG,(uint8_t)MPU6050_DLPF_CFG)!=HAL_OK ||
      write_register(MPU6050_REG_GYRO_CONFIG,(uint8_t)(MPU6050_GYRO_FS_SEL<<3))!=HAL_OK ||
      write_register(MPU6050_REG_ACCEL_CONFIG,(uint8_t)(MPU6050_ACCEL_FS_SEL<<3))!=HAL_OK ||
      write_register(MPU6050_REG_INT_PIN_CFG,0x00U)!=HAL_OK ||
      write_register(MPU6050_REG_INT_ENABLE,MPU6050_DATA_READY_INT)!=HAL_OK) return HAL_ERROR;

  /* 读 INT_STATUS 清掉配置期间可能产生的旧中断。 */
  if (read_register(MPU6050_REG_INT_STATUS,&int_status)!=HAL_OK) return HAL_ERROR;
  (void)int_status;

  /* MPU6500 / 9250 这一类还有独立的加速度数字低通（ACCEL_CONFIG2）。
   * 不配的话加速度带宽是 218 Hz，比陀螺仪（DLPF_CFG=3 约 41 Hz）吵得多，
   * 静止时的 |a| 和倾角都会明显抖。MPU6050 没有这个寄存器，跳过。 */
  if (device_id != MPU6050_WHO_AM_I_MPU6050) {
    uint8_t accel_config2=0U;
    if (write_register(MPU6050_REG_ACCEL_CONFIG2,(uint8_t)MPU6050_DLPF_CFG)!=HAL_OK) return HAL_ERROR;
    if (read_register(MPU6050_REG_ACCEL_CONFIG2,&accel_config2)!=HAL_OK) return HAL_ERROR;
    if ((accel_config2&0x07U)!=(uint8_t)MPU6050_DLPF_CFG) return HAL_ERROR;
  }
  return MPU6050_VerifyConfig();
}

HAL_StatusTypeDef MPU6050_ReadRawCounts(int16_t accel_counts[3], int16_t gyro_counts[3],
                                        int16_t *temperature_counts)
{
  uint8_t data[14];
  uint8_t axis;
  if (accel_counts==NULL || gyro_counts==NULL || temperature_counts==NULL) return HAL_ERROR;
  if (Board_I2C_Read(MPU1_I2C_ADDR,MPU6050_REG_ACCEL_XOUT_H,1U,data,14U)!=HAL_OK) return HAL_ERROR;
  for (axis=0U;axis<3U;axis++) {
    accel_counts[axis]=(int16_t)(((uint16_t)data[2U*axis]<<8)|data[2U*axis+1U]);
    gyro_counts[axis]=(int16_t)(((uint16_t)data[8U+2U*axis]<<8)|data[9U+2U*axis]);
  }
  *temperature_counts=(int16_t)(((uint16_t)data[6]<<8)|data[7]);
  return HAL_OK;
}

/* ============================== 校准、滤波与换算 ============================== */

/* 换算系数与 MPU6050_*_FS_SEL 绑定，头文件里有编译期校验；改量程必须同步改这里。
 *   加速度 ±4 g    -> 8192 LSB/g      mg   = counts * 1000 / 8192
 *   陀螺仪 ±500 dps -> 65.5 LSB/dps   mdps = counts * 2000 / 131
 *   温度           -> mdegC = counts * 50 / 17 + 36530
 * 全部整数运算，不把浮点库拖进来。 */
#define MPU6050_ACCEL_MG_DIV        8192L
#define MPU6050_GYRO_MDPS_MUL       2000L
#define MPU6050_GYRO_MDPS_DIV        131L
#define MPU6050_TEMP_MDEG_MUL         50L
#define MPU6050_TEMP_MDEG_DIV         17L
#define MPU6050_TEMP_MDEG_OFFSET   36530L

static Mpu6050Bias mpu_bias;
static int16_t mpu_accel_window[3][MPU6050_AVERAGE_WINDOW];
static int16_t mpu_gyro_window[3][MPU6050_AVERAGE_WINDOW];
static uint8_t mpu_window_index;
static uint8_t mpu_window_counts;

static int32_t accel_mg_from_counts(int32_t counts)
{
  /* 先按数据手册换算，再乘增益修正（千分比）。 */
  return ((counts * 1000L) / MPU6050_ACCEL_MG_DIV) * (int32_t)MPU6050_ACCEL_SCALE_PERMILLE / 1000L;
}

static int32_t gyro_mdps_from_counts(int32_t counts)
{
  return (counts * MPU6050_GYRO_MDPS_MUL) / MPU6050_GYRO_MDPS_DIV;
}

static int32_t temperature_mdeg_c_from_counts(int32_t counts)
{
  return (counts * MPU6050_TEMP_MDEG_MUL) / MPU6050_TEMP_MDEG_DIV + MPU6050_TEMP_MDEG_OFFSET;
}

/* 整数平方根：算加速度合矢量模长要用，避免引入浮点库。 */
static uint32_t isqrt_u32(uint32_t value)
{
  uint32_t root = 0U;
  uint32_t bit = 1UL << 30;

  while (bit > value) bit >>= 2;
  while (bit != 0U) {
    if (value >= root + bit) {
      value -= root + bit;
      root = (root >> 1) + bit;
    } else {
      root >>= 1;
    }
    bit >>= 2;
  }
  return root;
}

void MPU6050_ResetFilter(void)
{
  uint8_t axis, index;

  for (axis = 0U; axis < 3U; ++axis) {
    for (index = 0U; index < MPU6050_AVERAGE_WINDOW; ++index) {
      mpu_accel_window[axis][index] = 0;
      mpu_gyro_window[axis][index] = 0;
    }
  }
  mpu_window_index = 0U;
  mpu_window_counts = 0U;
}

const Mpu6050Bias *MPU6050_GetBias(void)
{
  return &mpu_bias;
}

HAL_StatusTypeDef MPU6050_CalibrateBias(Mpu6050Bias *bias, uint16_t samples)
{
  int32_t accel_sum[3] = {0L, 0L, 0L};
  int32_t gyro_sum[3] = {0L, 0L, 0L};
  int16_t accel_min[3], accel_max[3], gyro_min[3], gyro_max[3];
  int16_t accel_counts[3], gyro_counts[3], temperature_counts;
  uint32_t motion_pp_counts = 0U;
  uint16_t index;
  uint8_t axis;

  if (bias == NULL || samples == 0U) return HAL_ERROR;

  for (axis = 0U; axis < 3U; ++axis) {
    accel_min[axis] = 0x7FFF; accel_max[axis] = -0x8000;
    gyro_min[axis]  = 0x7FFF; gyro_max[axis]  = -0x8000;
  }

  for (index = 0U; index < samples; ++index) {
    if (MPU6050_ReadRawCounts(accel_counts, gyro_counts, &temperature_counts) != HAL_OK) return HAL_ERROR;
    for (axis = 0U; axis < 3U; ++axis) {
      accel_sum[axis] += accel_counts[axis];
      gyro_sum[axis] += gyro_counts[axis];
      if (accel_counts[axis] < accel_min[axis]) accel_min[axis] = accel_counts[axis];
      if (accel_counts[axis] > accel_max[axis]) accel_max[axis] = accel_counts[axis];
      if (gyro_counts[axis] < gyro_min[axis]) gyro_min[axis] = gyro_counts[axis];
      if (gyro_counts[axis] > gyro_max[axis]) gyro_max[axis] = gyro_counts[axis];
    }
    HAL_Delay(MPU6050_CALIB_SAMPLE_MS);
  }

  /* 校准期间三轴里最大的峰峰值：用来判断"到底动没动"。 */
  for (axis = 0U; axis < 3U; ++axis) {
    uint32_t accel_pp_counts = (uint32_t)((int32_t)accel_max[axis] - (int32_t)accel_min[axis]);
    uint32_t gyro_pp_counts = (uint32_t)((int32_t)gyro_max[axis] - (int32_t)gyro_min[axis]);
    if (accel_pp_counts > motion_pp_counts) motion_pp_counts = accel_pp_counts;
    if (gyro_pp_counts > motion_pp_counts) motion_pp_counts = gyro_pp_counts;
  }

  mpu_bias.motion_pp_counts = (uint16_t)motion_pp_counts;
  mpu_bias.calibrated = (motion_pp_counts <= MPU6050_CALIB_MOTION_PP_LIMIT) ? 1U : 0U;

  for (axis = 0U; axis < 3U; ++axis) {
    int32_t gyro_mean_counts = gyro_sum[axis] / (int32_t)samples;
    int32_t accel_mean_counts = accel_sum[axis] / (int32_t)samples;
    mpu_bias.gyro_bias_counts[axis] = (int16_t)gyro_mean_counts;
    mpu_bias.gyro_bias_mdps[axis] = gyro_mdps_from_counts(gyro_mean_counts);
    mpu_bias.accel_rest_mg[axis] = accel_mg_from_counts(accel_mean_counts);
  }

  if (mpu_bias.calibrated == 0U) {
    /* 校准时动了：宁可不补，也不要把错误偏置减进去。 */
    for (axis = 0U; axis < 3U; ++axis) {
      mpu_bias.gyro_bias_counts[axis] = 0;
      mpu_bias.gyro_bias_mdps[axis] = 0L;
    }
  }

  MPU6050_ResetFilter();
  *bias = mpu_bias;
  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadScaled(Mpu6050Reading *reading)
{
  int32_t accel_sum[3] = {0L, 0L, 0L};
  int32_t gyro_sum[3] = {0L, 0L, 0L};
  int32_t accel_avg[3], gyro_avg[3];
  int16_t accel_counts[3], gyro_counts[3], temperature_counts;
  uint32_t square_sum = 0U;
  uint8_t axis, index;

  if (reading == NULL) return HAL_ERROR;
  if (MPU6050_ReadRawCounts(accel_counts, gyro_counts, &temperature_counts) != HAL_OK) return HAL_ERROR;

  for (axis = 0U; axis < 3U; ++axis) {
    mpu_accel_window[axis][mpu_window_index] = accel_counts[axis];
    mpu_gyro_window[axis][mpu_window_index] = gyro_counts[axis];
  }
  mpu_window_index = (uint8_t)((mpu_window_index + 1U) % MPU6050_AVERAGE_WINDOW);
  if (mpu_window_counts < MPU6050_AVERAGE_WINDOW) mpu_window_counts++;

  for (axis = 0U; axis < 3U; ++axis) {
    for (index = 0U; index < mpu_window_counts; ++index) {
      accel_sum[axis] += mpu_accel_window[axis][index];
      gyro_sum[axis] += mpu_gyro_window[axis][index];
    }
    accel_avg[axis] = accel_sum[axis] / (int32_t)mpu_window_counts;
    gyro_avg[axis] = gyro_sum[axis] / (int32_t)mpu_window_counts;
  }

  for (axis = 0U; axis < 3U; ++axis) {
    reading->accel_mg[axis] = accel_mg_from_counts(accel_avg[axis]);
    reading->gyro_mdps[axis] = gyro_mdps_from_counts(gyro_avg[axis] - mpu_bias.gyro_bias_counts[axis]);
    square_sum += (uint32_t)(reading->accel_mg[axis] * reading->accel_mg[axis]);
  }
  reading->accel_magnitude_mg = (uint16_t)isqrt_u32(square_sum);
  reading->temperature_mdeg_c = temperature_mdeg_c_from_counts(temperature_counts);
  reading->averaged_counts = mpu_window_counts;
  return HAL_OK;
}
