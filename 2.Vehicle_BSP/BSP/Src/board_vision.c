#include "board.h"
#include "board_vision.h"
#include <string.h>

/* 视觉目标流行解析：只认 '$' 开头的行，其余（TASK / PING 等）原样丢弃。
   解析全部在主循环的 Vision_Poll() 里做，USART3 中断只往环形缓冲塞字节。 */

static VisionTarget target;
static VisionStats stats;
static char line[VD_LINE_BYTES];
static uint16_t line_length;
static uint8_t line_overflow;
static VisionLineHandler line_handler;

static uint8_t crc8_compute(const char *data, uint16_t length)
{
  uint8_t crc = 0U;
  for (uint16_t i = 0U; i < length; ++i) {
    crc ^= (uint8_t)data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      crc = (crc & 0x80U) ? (uint8_t)((uint8_t)(crc << 1) ^ 0x07U)
                          : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static uint8_t hex_value(char character, uint8_t *value)
{
  if (character >= '0' && character <= '9') {
    *value = (uint8_t)(character - '0');
    return 1U;
  }
  if (character >= 'A' && character <= 'F') {
    *value = (uint8_t)(character - 'A' + 10);
    return 1U;
  }
  if (character >= 'a' && character <= 'f') {
    *value = (uint8_t)(character - 'a' + 10);
    return 1U;
  }
  return 0U;
}

/* 解析一个十进制整数，允许前导 '-'，停在第一个非数字字符上。
   空字段、只有 '-'、超出内部量程都返回 0（调用方据此判坏帧）。 */
static uint8_t parse_int32(const char **cursor, int32_t *value)
{
  const char *p = *cursor;
  int32_t accumulated = 0;
  int32_t sign = 1;
  uint8_t digits = 0U;

  if (*p == '-') {
    sign = -1;
    ++p;
  }
  while (*p >= '0' && *p <= '9') {
    if (accumulated > 200000000L) return 0U;
    accumulated = accumulated * 10 + (int32_t)(*p - '0');
    ++p;
    ++digits;
  }
  if (!digits) return 0U;
  *value = sign * accumulated;
  *cursor = p;
  return 1U;
}

static void accept_frame(const int32_t *field)
{
  uint32_t now_ms = Board_NowMs();
  uint32_t seq = (uint32_t)field[0];

  /* 同 seq = 视觉侧重发上一帧：只计数，不刷新时间戳，
     这样"视觉卡住但还在重复发"不会被误判成新鲜数据。 */
  if (stats.frames_ok &&
      seq == target.seq &&
      (uint32_t)(now_ms - target.stamp_ms) < VD_RESTART_GAP_MS) {
    stats.frames_repeat++;
    return;
  }
  /* 视觉侧重启后 seq 会归零，上面的条件因为时间间隔已超时而不成立，正常接受。 */
  if (stats.frames_ok && seq > target.seq && (seq - target.seq) > 1U) {
    stats.frames_gap += (seq - target.seq) - 1U;
  }

  target.seq = seq;
  target.stamp_ms = now_ms;
  target.class_code = (uint16_t)field[1];
  target.x_mm = (int16_t)field[2];
  target.y_mm = (int16_t)field[3];
  target.yaw_mdeg = field[4];
  target.confidence_permille = (uint16_t)field[5];
  target.age_ms = (uint16_t)field[6];
  target.valid = (uint8_t)(field[1] != 0);
  stats.frames_ok++;
}

static void parse_line(const char *text)
{
  int32_t field[7];
  const char *cursor;
  uint16_t length = (uint16_t)strlen(text);
  uint8_t high = 0U;
  uint8_t low = 0U;

  /* 最短一条无目标帧约 24 字节；比这还短一定是错的。 */
  if (length < 16U || text[0] != '$' || text[1] != 'V' || text[2] != ',') {
    stats.frames_bad++;
    return;
  }
  if (text[length - 3U] != '*' ||
      !hex_value(text[length - 2U], &high) ||
      !hex_value(text[length - 1U], &low)) {
    stats.frames_bad++;
    return;
  }
  if (crc8_compute(text + 1, (uint16_t)(length - 4U)) != (uint8_t)((high << 4) | low)) {
    stats.frames_bad++;
    return;
  }

  cursor = text + 3;
  for (uint8_t index = 0U; index < 7U; ++index) {
    if (!parse_int32(&cursor, &field[index])) {
      stats.frames_bad++;
      return;
    }
    if (index < 6U) {
      if (*cursor != ',') {
        stats.frames_bad++;
        return;
      }
      ++cursor;
    }
  }
  if (*cursor != '*') {
    stats.frames_bad++;
    return;
  }

  /* 量程检查：任何一项越界都整帧丢弃，不让脏数据流到下游。 */
  if (field[0] < 0 ||
      field[1] < 0 || field[1] > (int32_t)VD_CLASS_MAX ||
      field[2] < -(int32_t)VD_TARGET_LIMIT_MM || field[2] > (int32_t)VD_TARGET_LIMIT_MM ||
      field[3] < -(int32_t)VD_TARGET_LIMIT_MM || field[3] > (int32_t)VD_TARGET_LIMIT_MM ||
      field[4] < -VD_YAW_LIMIT_MDEG || field[4] > VD_YAW_LIMIT_MDEG ||
      field[5] < 0 || field[5] > (int32_t)VD_CONF_MAX_PERMILLE ||
      field[6] < 0 || field[6] > (int32_t)VD_AGE_LIMIT_MS) {
    stats.frames_bad++;
    return;
  }

  accept_frame(field);
}

void Vision_SetLineHandler(VisionLineHandler handler)
{
  line_handler = handler;
}

void Vision_Init(void)
{
  memset(&target, 0, sizeof(target));
  memset(&stats, 0, sizeof(stats));
  line_length = 0U;
  line_overflow = 0U;
}

void Vision_Poll(void)
{
  int received;
  uint16_t drained = 0U;

  while (drained < VD_DRAIN_BYTES && (received = Serial_ReadByte(SERIAL_RPI)) >= 0) {
    char character = (char)received;
    ++drained;

    if (character == '\r') continue;
    if (character == '\n') {
      if (line_overflow) {
        stats.bytes_overflow++;
      } else if (line_length) {
        line[line_length] = '\0';
        if (line[0] == '$') {
          parse_line(line);
        } else if (line_handler != 0) {
          line_handler(line);   /* TASK / STAT 等应用层下行 */
        }
      }
      line_length = 0U;
      line_overflow = 0U;
      continue;
    }

    if (line_length < VD_LINE_BYTES - 1U && !line_overflow) {
      line[line_length++] = character;
    } else {
      line_overflow = 1U;
    }
  }

  stats.bytes_dropped = Serial_Dropped(SERIAL_RPI);
}

const VisionTarget *Vision_ReadTarget(void)
{
  return &target;
}

void Vision_ReadStats(VisionStats *snapshot)
{
  if (snapshot) *snapshot = stats;
}

uint8_t Vision_TargetFresh(uint32_t now_ms, uint32_t max_age_ms)
{
  /* 一帧都没接受过时 stamp_ms 是 0。上电几百毫秒内 (now - 0) 仍小于阈值，
   * 会被误判成"新鲜目标"。必须先看有没有真正收过帧。 */
  if (stats.frames_ok == 0U) return 0U;
  return (uint8_t)((uint32_t)(now_ms - target.stamp_ms) <= max_age_ms);
}
