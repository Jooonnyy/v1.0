#include "board_internal.h"
#include "board_imu.h"

/* HWT101 帧解析（维特 0x55 协议）：
 *   0x55 | 类型 | data0L | data0H | ... | 校验和（前面 10 字节求和取低 8 位）
 *   类型 0x51 = 加速度，0x52 = 角速度，0x53 = 角度（roll/pitch/yaw 各 2 字节）。
 * 我们只用 0x53 里的 yaw（第 6、7 字节，有符号、满量程 32768 对应 180°）。 */

static uint8_t  frame[IMU_FRAME_LEN];
static uint8_t  frame_len;

static int32_t  raw_yaw_mdeg;      /* 模块原始 yaw，已换向，未减零点 */
static int32_t  yaw_cont_mdeg;     /* 连续累积的绝对 yaw（不 wrap，跨 ±180 继续加） */
static int32_t  yaw_mdeg;          /* 减过零点的对外值 */
static int32_t  zero_mdeg;         /* 零点（连续累积坐标系里的值） */
static int32_t  yaw_rate_mdps;
static int32_t  prev_raw_mdeg;
static uint32_t prev_ms;
static uint8_t  have_prev;

static uint32_t frames;
static uint32_t bad_frames;
static uint32_t raw_bytes;
static uint32_t last_ms;

/* 把角度差折算到 ±180000，避免过 ±180 时算出巨大的角速度。 */
static int32_t wrap_mdeg(int32_t d)
{
  while (d > 180000) d -= 360000;
  while (d < -180000) d += 360000;
  return d;
}

static void on_frame(const uint8_t *f, uint32_t now_ms)
{
  int16_t yaw_raw;
  int32_t new_raw;

  if (f[1] != 0x53U) return;      /* 只关心角度帧 */

  yaw_raw = (int16_t)((uint16_t)f[6] | ((uint16_t)f[7] << 8));
  new_raw = (int32_t)(((int64_t)yaw_raw * 180000LL) / 32768LL) * IMU_YAW_SIGN;

  /* 模块 yaw 在 ±180 处会跳变，这里按增量累积成连续角度：
   * 第一帧直接取绝对值，之后每次只加"折算到 ±180 的增量"。 */
  if (have_prev == 0U) yaw_cont_mdeg = new_raw;
  else yaw_cont_mdeg += wrap_mdeg(new_raw - prev_raw_mdeg);

  raw_yaw_mdeg = new_raw;
  yaw_mdeg = yaw_cont_mdeg - zero_mdeg;   /* 不 wrap：转过 360 就是 360000 */

  if (have_prev != 0U) {
    uint32_t dt = now_ms - prev_ms;
    if (dt > 0U && dt <= IMU_MAX_PERIOD_MS) {
      yaw_rate_mdps = (int32_t)(((int64_t)wrap_mdeg(new_raw - prev_raw_mdeg) * 1000) /
                                (int64_t)dt);
    } else {
      yaw_rate_mdps = 0;   /* 帧间隔太短/太长都不算，避免噪声被放大 */
    }
  }
  prev_raw_mdeg = new_raw;
  prev_ms = now_ms;
  have_prev = 1U;
  frames++;
  last_ms = now_ms;
}

void Imu_Poll(void)
{
  int byte;
  uint32_t now_ms = Board_NowMs();

  while ((byte = Serial_ReadByte(SERIAL_IMU)) >= 0) {
    uint8_t b = (uint8_t)byte;

    raw_bytes++;
    if (frame_len == 0U) {
      if (b == 0x55U) frame[frame_len++] = b;
    } else if (frame_len == 1U) {
      /* 0x51/0x52/0x53 都是 11 字节帧，长度一样，这里只按类型区分要不要用。 */
      if (b == 0x53U || b == 0x51U || b == 0x52U) {
        frame[frame_len++] = b;
      } else {
        frame_len = (b == 0x55U) ? 1U : 0U;
      }
    } else {
      frame[frame_len++] = b;
      if (frame_len == IMU_FRAME_LEN) {
        uint8_t i, sum = 0U;

        for (i = 0U; i < (IMU_FRAME_LEN - 1U); ++i) sum = (uint8_t)(sum + frame[i]);
        if (sum == frame[IMU_FRAME_LEN - 1U]) on_frame(frame, now_ms);
        else bad_frames++;
        frame_len = 0U;
      }
    }
  }
}

uint8_t Imu_Read(ImuSnapshot *out)
{
  uint32_t now_ms;

  if (frames == 0U) {
    if (out != 0) {
      out->yaw_mdeg = 0;
      out->yaw_rate_mdps = 0;
      out->frames = 0U;
      out->bad_frames = bad_frames;
      out->age_ms = 0xFFFFFFFFU;
      out->valid = 0U;
      out->fresh = 0U;
    }
    return 0U;
  }

  now_ms = Board_NowMs();
  if (out != 0) {
    out->yaw_mdeg = yaw_mdeg;
    out->yaw_rate_mdps = yaw_rate_mdps;
    out->frames = frames;
    out->bad_frames = bad_frames;
    out->age_ms = now_ms - last_ms;
    out->valid = 1U;
    out->fresh = ((now_ms - last_ms) <= IMU_TIMEOUT_MS) ? 1U : 0U;
  }
  return 1U;
}

int32_t Imu_ReadYawMdeg(void) { return yaw_mdeg; }

int32_t Imu_ReadRateMdps(void)
{
  if (frames == 0U) return 0;
  if ((uint32_t)(Board_NowMs() - last_ms) > IMU_TIMEOUT_MS) return 0;
  return yaw_rate_mdps;
}

uint8_t Imu_IsFresh(void)
{
  if (frames == 0U) return 0U;
  return ((uint32_t)(Board_NowMs() - last_ms) <= IMU_TIMEOUT_MS) ? 1U : 0U;
}

uint8_t Imu_HasFrame(void) { return (frames != 0U) ? 1U : 0U; }

uint32_t Imu_RawByteCount(void) { return raw_bytes; }

void Imu_Zero(void)
{
  zero_mdeg = yaw_cont_mdeg;
  yaw_mdeg = 0;
  yaw_rate_mdps = 0;
}
