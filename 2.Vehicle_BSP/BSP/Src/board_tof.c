#include "board_tof.h"
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"
#include "pins/tof400c.h"
#include <string.h>

/*
 * TOF400C（VL53L1X）驱动，两层：
 *   1) 本文件上半部分是 ULD 要求的 9 个平台函数，接到板子的软件 I2C 上；
 *   2) 下半部分是给应用用的 Tof_* 接口（初始化、读距离、XSHUT 控制）。
 *
 * 时序上的一点说明：VL53L1X 的寄存器地址是 16 位、数据是 8 位，正好对上
 * Board_I2C_Read/Write(addr7, reg, reg_len=2, ...) 的形态，所以平台层只是
 * 一层薄封装，不需要自己拼起始/停止条件。
 *
 * ⚠ 已知风险：ST 的 VL53L1X_SensorInit() 里有一个 **没有超时的**
 *   `while (tmp == 0) CheckForDataReady(...)`。如果 I2C 中途坏掉，
 *   它会**永久卡住**。所以 Tof_Init() 一定先读型号 ID 和固件状态，
 *   两者都通过才调用 SensorInit()，把卡死的概率压到最低。
 */

/* ============================== 平台层（ULD 调用） ============================== */

/* 下面 9 个函数全部返回 0 表示成功，非 0 表示失败。
 * dev 就是 VL53L1X 的 **7 位**地址，被 ULD 原样透传，不做任何移位。 */

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  if (pdata == NULL || count == 0U || count > 0xFFFFU) return 1;
  return (Board_I2C_Write((uint8_t)dev, index, 2U, pdata, (uint16_t)count) == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
  if (pdata == NULL || count == 0U || count > 0xFFFFU) return 1;
  return (Board_I2C_Read((uint8_t)dev, index, 2U, pdata, (uint16_t)count) == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
  return VL53L1_WriteMulti(dev, index, &data, 1U);
}

/* 多字节寄存器一律大端：先高字节。 */
int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
  uint8_t buffer[2];
  buffer[0] = (uint8_t)(data >> 8);
  buffer[1] = (uint8_t)(data & 0x00FFU);
  return VL53L1_WriteMulti(dev, index, buffer, 2U);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
  uint8_t buffer[4];
  buffer[0] = (uint8_t)(data >> 24);
  buffer[1] = (uint8_t)(data >> 16);
  buffer[2] = (uint8_t)(data >> 8);
  buffer[3] = (uint8_t)(data & 0x000000FFU);
  return VL53L1_WriteMulti(dev, index, buffer, 4U);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *pdata)
{
  return VL53L1_ReadMulti(dev, index, pdata, 1U);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *pdata)
{
  uint8_t buffer[2];
  if (pdata == NULL) return 1;
  if (VL53L1_ReadMulti(dev, index, buffer, 2U) != 0) return 1;
  *pdata = (uint16_t)(((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);
  return 0;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *pdata)
{
  uint8_t buffer[4];
  if (pdata == NULL) return 1;
  if (VL53L1_ReadMulti(dev, index, buffer, 4U) != 0) return 1;
  *pdata = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) |
           ((uint32_t)buffer[2] << 8) | (uint32_t)buffer[3];
  return 0;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
  (void)dev;
  if (wait_ms <= 0) return 0;
  HAL_Delay((uint32_t)wait_ms);
  return 0;
}

/* ============================== 内部状态 ============================== */

/* Tof_Init() 是否已经成功过；为 0 时不允许读距离。 */
static uint8_t tof_ready;

/* ------------------------------ 滤波状态 ------------------------------ */

/* 中值窗口的环形缓冲，只放 RangeStatus == 0 的有效样本。 */
static uint16_t tof_median_buffer[TOF_MEDIAN_WINDOW];
static uint8_t  tof_median_index;
static uint8_t  tof_median_counts;

/* 滑动平均窗口。求和用整数累加，全程不引入浮点。 */
static uint16_t tof_average_buffer[TOF_AVERAGE_WINDOW];
static uint8_t  tof_average_index;
static uint8_t  tof_average_counts;
static uint32_t tof_average_sum_mm;

/* 最近一次有效滤波结果；还没有有效样本时保持 TOF_DISTANCE_INVALID_MM。 */
static uint16_t tof_filtered_mm = TOF_DISTANCE_INVALID_MM;

/* 对当前中值窗口里的有效样本排序后取中值。
 * 样本最多 9 个，插入排序比 qsort 更省栈（sorted 只占 18 B）。 */
static uint16_t tof_median_of_window(void)
{
  uint16_t sorted[TOF_MEDIAN_WINDOW];
  uint16_t key;
  uint8_t i, j;

  for (i = 0U; i < tof_median_counts; ++i) sorted[i] = tof_median_buffer[i];
  for (i = 1U; i < tof_median_counts; ++i) {
    key = sorted[i];
    j = i;
    while (j > 0U && sorted[j - 1U] > key) {
      sorted[j] = sorted[j - 1U];
      --j;
    }
    sorted[j] = key;
  }
  return sorted[tof_median_counts / 2U];
}

/* 把一个中值结果推进滑动平均窗口，并更新 tof_filtered_mm。 */
static void tof_push_average(uint16_t distance_mm)
{
  if (tof_average_counts < TOF_AVERAGE_WINDOW) {
    tof_average_buffer[tof_average_counts] = distance_mm;
    tof_average_sum_mm += distance_mm;
    tof_average_counts++;
  } else {
    tof_average_sum_mm -= tof_average_buffer[tof_average_index];
    tof_average_buffer[tof_average_index] = distance_mm;
    tof_average_sum_mm += distance_mm;
    tof_average_index = (uint8_t)((tof_average_index + 1U) % TOF_AVERAGE_WINDOW);
  }
  tof_filtered_mm = (uint16_t)(tof_average_sum_mm / tof_average_counts);
}

/* ============================== XSHUT 控制 ============================== */

/* 释放复位：把 PC1 切成"输入 + 内部上拉"。
 * 为什么不用"开漏输出写高"：开漏写高只是高阻，没有外部上拉时 XSHUT 会悬空，
 * 传感器可能一直停在复位里。切成输入上拉就提供了约 40k 的真实上拉，
 * 而且因为不是推挽输出，即使和它共用的板载按键被按下，也只是被按键拉到地，
 * 不会出现推挽对地短路。 */
static void tof_shut_release(void)
{
#if TOF_SHUT_CONNECTED
  GPIO_InitTypeDef cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.Pin = TOF_SHUT_PIN;
  cfg.Mode = GPIO_MODE_INPUT;
  cfg.Pull = GPIO_PULLUP;
  cfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TOF_SHUT_PORT, &cfg);
#endif
}

/* 拉低复位：开漏输出 + 输出低。开漏保证不会和按键打架。 */
static void tof_shut_assert(void)
{
#if TOF_SHUT_CONNECTED
  GPIO_InitTypeDef cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.Pin = TOF_SHUT_PIN;
  cfg.Mode = GPIO_MODE_OUTPUT_OD;
  cfg.Pull = GPIO_NOPULL;
  cfg.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(TOF_SHUT_PORT, TOF_SHUT_PIN, GPIO_PIN_RESET);
  HAL_GPIO_Init(TOF_SHUT_PORT, &cfg);
  HAL_GPIO_WritePin(TOF_SHUT_PORT, TOF_SHUT_PIN, GPIO_PIN_RESET);
#endif
}

void Tof_Shutdown(uint8_t shutdown)
{
  if (shutdown != 0U) {
    tof_ready = 0U;
    tof_shut_assert();
  } else {
    tof_shut_release();
  }
}

uint8_t Tof_ShutControllable(void)
{
  return (uint8_t)TOF_SHUT_CONNECTED;
}

uint8_t Tof_ShutLineHigh(void)
{
  return (HAL_GPIO_ReadPin(TOF_SHUT_PORT, TOF_SHUT_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/* ============================== 模块 API ============================== */

uint8_t Tof_IsReady(void)
{
  return tof_ready;
}

void Tof_ResetFilter(void)
{
  uint8_t i;

  for (i = 0U; i < TOF_MEDIAN_WINDOW; ++i) tof_median_buffer[i] = 0U;
  for (i = 0U; i < TOF_AVERAGE_WINDOW; ++i) tof_average_buffer[i] = 0U;
  tof_median_index = 0U;
  tof_median_counts = 0U;
  tof_average_index = 0U;
  tof_average_counts = 0U;
  tof_average_sum_mm = 0U;
  tof_filtered_mm = TOF_DISTANCE_INVALID_MM;
}

HAL_StatusTypeDef Tof_ModelId(uint16_t *model_id)
{
  uint16_t id = 0U;
  if (model_id == NULL) return HAL_ERROR;
  if (VL53L1X_GetSensorId((uint16_t)TOF_I2C_ADDR7, &id) != 0) return HAL_ERROR;
  *model_id = id;
  return HAL_OK;
}

HAL_StatusTypeDef Tof_Init(void)
{
  uint16_t model_id = 0U;
  uint8_t boot_state = 0U;
  uint32_t start_ms;
  uint16_t dev = (uint16_t)TOF_I2C_ADDR7;

  tof_ready = 0U;
  Tof_ResetFilter();

  /* 1) 有 SHUT 连线时做复位脉冲；当前硬件没有这条连线，只能依赖模块上电启动。 */
#if TOF_SHUT_CONNECTED
  tof_shut_assert();
  HAL_Delay(TOF_RESET_HOLD_MS);
  tof_shut_release();
#endif
  HAL_Delay(TOF_BOOT_DELAY_MS);

  /* 2) 型号 ID 与固件状态都通了，才敢调用有死循环风险的 SensorInit()。 */
  if (Tof_ModelId(&model_id) != HAL_OK) return HAL_ERROR;
  if (model_id != TOF_MODEL_ID) return HAL_ERROR;

  start_ms = HAL_GetTick();
  for (;;) {
    if (VL53L1X_BootState(dev, &boot_state) == 0 && boot_state != 0U) break;
    if ((uint32_t)(HAL_GetTick() - start_ms) >= TOF_BOOT_TIMEOUT_MS) return HAL_TIMEOUT;
    HAL_Delay(2U);
  }

  /* 3) 下发 ST 的默认配置表（SensorInit 内部完成），再设距离模式与时序。 */
  if (VL53L1X_SensorInit(dev) != 0) return HAL_ERROR;
  if (VL53L1X_SetDistanceMode(dev, TOF_DISTANCE_MODE) != 0) return HAL_ERROR;
  if (VL53L1X_SetTimingBudgetInMs(dev, TOF_TIMING_BUDGET_MS) != 0) return HAL_ERROR;
  if (VL53L1X_SetInterMeasurementInMs(dev, TOF_INTER_MEASUREMENT_MS) != 0) return HAL_ERROR;
  if (TOF_SIGMA_THRESHOLD_MM != 0U) {
    /* 器件侧的 sigma 门槛（单位 mm）。失败不致命，最多退回出厂的 90 mm 默认值。 */
    (void)VL53L1X_SetSigmaThreshold(dev, TOF_SIGMA_THRESHOLD_MM);
  }
  /* ROI（视场）：先设大小，再单独设中心（SetROI 内部会把中心写回光学中心）。
   * 必须在 StartRanging 之前配置。 */
  if (VL53L1X_SetROI(dev, TOF_ROI_WIDTH_SPADS, TOF_ROI_HEIGHT_SPADS) != 0) return HAL_ERROR;
  if (TOF_ROI_CENTER_SPAD != 199U) {
    if (VL53L1X_SetROICenter(dev, TOF_ROI_CENTER_SPAD) != 0) return HAL_ERROR;
  }
  if (VL53L1X_StartRanging(dev) != 0) return HAL_ERROR;

  tof_ready = 1U;
  return HAL_OK;
}

HAL_StatusTypeDef Tof_ReadMm(uint16_t *distance_mm, uint8_t *range_status)
{
  uint16_t distance = 0U;
  uint8_t status_byte = 0U;
  uint8_t is_ready = 0U;
  uint32_t start_ms;
  uint16_t dev = (uint16_t)TOF_I2C_ADDR7;

  if (distance_mm == NULL) return HAL_ERROR;
  *distance_mm = TOF_DISTANCE_INVALID_MM;
  if (range_status != NULL) *range_status = 0xFFU;
  if (tof_ready == 0U) return HAL_ERROR;

  /* 等这一轮测量完成。 */
  start_ms = HAL_GetTick();
  for (;;) {
    if (VL53L1X_CheckForDataReady(dev, &is_ready) != 0) return HAL_ERROR;
    if (is_ready != 0U) break;
    if ((uint32_t)(HAL_GetTick() - start_ms) >= TOF_READY_TIMEOUT_MS) return HAL_TIMEOUT;
    HAL_Delay(1U);
  }

  if (VL53L1X_GetDistance(dev, &distance) != 0) return HAL_ERROR;
  if (VL53L1X_GetRangeStatus(dev, &status_byte) != 0) return HAL_ERROR;
  if (VL53L1X_ClearInterrupt(dev) != 0) return HAL_ERROR;

  *distance_mm = distance;
  if (range_status != NULL) *range_status = status_byte;
  return HAL_OK;
}

HAL_StatusTypeDef Tof_ReadFiltered(TofReading *reading)
{
  uint16_t distance = 0U;
  uint16_t signal_per_spad = 0U;
  uint16_t ambient_per_spad = 0U;
  uint8_t status_byte = 0U;
  HAL_StatusTypeDef read_status;
  uint16_t dev = (uint16_t)TOF_I2C_ADDR7;

  if (reading == NULL) return HAL_ERROR;
  reading->distance_mm = TOF_DISTANCE_INVALID_MM;
  reading->distance_raw_mm = TOF_DISTANCE_INVALID_MM;
  reading->signal_kcps_per_spad = 0U;
  reading->ambient_kcps_per_spad = 0U;
  reading->range_status = 0xFFU;
  reading->window_counts = tof_median_counts;

  read_status = Tof_ReadMm(&distance, &status_byte);
  if (read_status != HAL_OK) return read_status;

  /* 质量指标仅供参考，读失败不影响距离结果。 */
  (void)VL53L1X_GetSignalPerSpad(dev, &signal_per_spad);
  (void)VL53L1X_GetAmbientPerSpad(dev, &ambient_per_spad);

  reading->distance_raw_mm = distance;
  reading->range_status = status_byte;
  reading->signal_kcps_per_spad = signal_per_spad;
  reading->ambient_kcps_per_spad = ambient_per_spad;

  /* 只有器件认为有效的样本才进滤波窗口，坏点不许污染中值。 */
  if (status_byte == 0U) {
    tof_median_buffer[tof_median_index] = distance;
    tof_median_index = (uint8_t)((tof_median_index + 1U) % TOF_MEDIAN_WINDOW);
    if (tof_median_counts < TOF_MEDIAN_WINDOW) tof_median_counts++;
    tof_push_average(tof_median_of_window());
  }

  reading->window_counts = tof_median_counts;
  reading->distance_mm = tof_filtered_mm;
  return HAL_OK;
}
