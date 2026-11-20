#include "board_hmi.h"
#include "board.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ==================================================================== */
/* 内部状态                                                              */
/* ==================================================================== */

/* 接收拼帧。缓冲区都放成 static：本工程启动文件的栈只有 0x400 = 1 KB，
   在函数里开大数组很容易把栈压爆。 */
static uint8_t  rx_frame[HMI_MAX_FRAME_LEN];
static uint16_t rx_len;
static uint8_t  rx_ff_run;      /* 连续收到多少个 0xFF */
static uint8_t  rx_overflow;    /* 本帧超长，丢弃到下一个结束符 */

/* 发送拼帧。 */
static char     tx_body[HMI_TX_BUFFER_SIZE];
static uint8_t  tx_out[HMI_TX_BUFFER_SIZE + 3U];

static uint32_t frames;
static uint32_t app_frames;
static uint32_t touch_frames;
static uint32_t bad_frames;
static uint32_t return_codes;
static uint32_t tx_bytes;
static uint32_t tx_truncated;
static uint8_t  last_return_code;

static void send_debug(const char *message)
{
  (void)Serial_Write(SERIAL_DEBUG, (const uint8_t *)message,
                     (uint16_t)strlen(message), 100U);
}

/* ==================================================================== */
/* 发送                                                                  */
/* ==================================================================== */

/* 把 body 的前 len 字节加上 FF FF FF 一次性发出去。
   一次 Serial_Write 而不是逐字节发，是因为 Serial_Write 是阻塞式、
   每字节等到 TC 才返回；逐字节调用会把阻塞时间放大很多倍。 */
static void hmi_write_frame(const char *body, uint16_t len)
{
  if (len > HMI_TX_BUFFER_SIZE) {
    len = HMI_TX_BUFFER_SIZE;
    tx_truncated++;
  }
  memcpy(tx_out, body, len);
  tx_out[len] = 0xFFU;
  tx_out[len + 1U] = 0xFFU;
  tx_out[len + 2U] = 0xFFU;
  (void)Serial_Write(SERIAL_HMI, tx_out, (uint16_t)(len + 3U), HMI_TX_TIMEOUT_MS);
  tx_bytes += (uint32_t)(len + 3U);
}

/* 用 snprintf 把内容写进 tx_body；返回值是实际长度，超长会被截断并计数。 */
static uint16_t build(const char *format, ...)
{
  va_list args;
  int written;

  va_start(args, format);
  written = vsnprintf(tx_body, sizeof(tx_body), format, args);
  va_end(args);

  if (written < 0) return 0U;
  if ((size_t)written >= sizeof(tx_body)) {
    tx_truncated++;
    return (uint16_t)(sizeof(tx_body) - 1U);
  }
  return (uint16_t)written;
}

void Hmi_SendRaw(const char *command)
{
  size_t len;
  if (!command) return;
  len = strlen(command);
  if (len > HMI_TX_BUFFER_SIZE) len = HMI_TX_BUFFER_SIZE;  /* hmi_write_frame 会计数 */
  hmi_write_frame(command, (uint16_t)len);
}

void Hmi_SetText(const char *object, const char *text)
{
  uint16_t len;
  if (!object || !text) return;
  len = build("%s.txt=\"%s\"", object, text);
  hmi_write_frame(tx_body, len);
}

void Hmi_SetValue(const char *object, int32_t value)
{
  uint16_t len;
  if (!object) return;
  len = build("%s.val=%ld", object, (long)value);
  hmi_write_frame(tx_body, len);
}

void Hmi_Click(const char *object, uint8_t pressed)
{
  uint16_t len;
  if (!object) return;
  len = build("click %s,%u", object, (unsigned)(pressed ? 1U : 0U));
  hmi_write_frame(tx_body, len);
}

void Hmi_SetVisible(const char *object, uint8_t visible)
{
  uint16_t len;
  if (!object) return;
  len = build("vis %s,%u", object, (unsigned)(visible ? 1U : 0U));
  hmi_write_frame(tx_body, len);
}

void Hmi_SetPage(uint8_t page)
{
  uint16_t len = build("page %u", (unsigned)page);
  hmi_write_frame(tx_body, len);
}

void Hmi_SetFont(const char *object, uint16_t font_id)
{
  uint16_t len;
  if (!object) return;
  len = build("%s.font=%u", object, (unsigned)font_id);
  hmi_write_frame(tx_body, len);
}

void Hmi_Printf(const char *format, ...)
{
  va_list args;
  int written;

  if (!format) return;
  va_start(args, format);
  written = vsnprintf(tx_body, sizeof(tx_body), format, args);
  va_end(args);

  if (written < 0) return;
  if ((size_t)written >= sizeof(tx_body)) {
    tx_truncated++;
    written = (int)(sizeof(tx_body) - 1U);
  }
  hmi_write_frame(tx_body, (uint16_t)written);
}

/* ==================================================================== */
/* 返回码                                                                */
/* ==================================================================== */

const char *Hmi_ReturnCodeText(uint8_t code)
{
  /* 常见返回码，完整列表见 http://wiki.tjc1688.com/return/index.html */
  switch (code) {
    case 0x00: return "invalid instruction";
    case 0x01: return "instruction successful";
    case 0x02: return "object or variable name invalid";
    case 0x03: return "operation invalid";
    case 0x04: return "assignment invalid";
    case 0x05: return "EEPROM operation failed";
    case 0x06: return "invalid escape character";
    case 0x11: return "invalid baud rate setting";
    case 0x12: return "invalid curve control";
    case 0x1A: return "variable name invalid";
    case 0x1B: return "variable name too long";
    case 0x1C: return "parameter invalid";
    case 0x1D: return "page ID invalid";
    case 0x1E: return "picture ID invalid";
    case 0x1F: return "IO operation failed";
    case 0x20: return "escape character out of range";
    case 0x23: return "variable already exists";
    case 0x24: return "variable name too long";
    case 0x26: return "invalid file ID";
    case 0x27: return "invalid variable name";
    case 0xFF: return "operation failed";
    default:   return "unknown or reserved code";
  }
}

static void handle_return_code(uint8_t code)
{
  char line[96];

  last_return_code = code;
  return_codes++;

  /* 0x01 = 成功。每次发指令都成功的话不打印，否则返回码会刷屏。 */
  if (code == 0x01U) return;

  (void)snprintf(line, sizeof(line), "HMI return 0x%02X: %s\r\n",
                 (unsigned)code, Hmi_ReturnCodeText(code));
  send_debug(line);
}

/* ==================================================================== */
/* 接收拼帧                                                              */
/* ==================================================================== */

static void reset_frame(void)
{
  rx_len = 0U;
  rx_ff_run = 0U;
  rx_overflow = 0U;
}

/* 一帧收齐（已经看到 FF FF FF）。payload 长度 = rx_len - 3。 */
static void dispatch_frame(void)
{
  uint16_t len;
  uint8_t *payload;

  if (rx_overflow) {          /* 超长帧，整帧作废 */
    bad_frames++;
    return;
  }
  if (rx_len < 3U) return;    /* 理论上不会发生 */

  len = (uint16_t)(rx_len - 3U);
  payload = rx_frame;
  if (len == 0U) return;      /* 只收到一个结束符，忽略 */

  frames++;

  /* 只有 1 个字节 = 屏幕的返回码。 */
  if (len == 1U) {
    handle_return_code(payload[0]);
    return;
  }

  if (len == HMI_APP_FRAME_LEN && payload[0] == HMI_APP_HEADER) {
    app_frames++;
    Hmi_OnAppFrame(payload[1], payload[2], payload[3]);
    return;
  }

  if (len == HMI_TOUCH_FRAME_LEN && payload[0] == HMI_TOUCH_HEADER) {
    touch_frames++;
    Hmi_OnTouch(payload[1], payload[2], payload[3]);
    return;
  }

  bad_frames++;
}

static void feed_byte(uint8_t byte)
{
  if (rx_len < HMI_MAX_FRAME_LEN) rx_frame[rx_len++] = byte;
  else rx_overflow = 1U;      /* 不下标越界，丢掉到下一个结束符为止 */

  if (byte == 0xFFU) {
    rx_ff_run++;
    if (rx_ff_run >= 3U) {    /* 三个连续 0xFF = 帧结束 */
      dispatch_frame();
      reset_frame();
    }
  } else {
    rx_ff_run = 0U;
  }
}

/* ==================================================================== */
/* 对外接口                                                              */
/* ==================================================================== */

void Hmi_Init(void)
{
  reset_frame();

  frames = 0U;
  app_frames = 0U;
  touch_frames = 0U;
  bad_frames = 0U;
  return_codes = 0U;
  tx_bytes = 0U;
  tx_truncated = 0U;
  last_return_code = 0U;
}

void Hmi_Poll(void)
{
  int byte;

  /* 把这一轮能读到的字节全部取走。Serial_ReadByte 返回 -1 表示空。 */
  while ((byte = Serial_ReadByte(SERIAL_HMI)) >= 0) {
    feed_byte((uint8_t)byte);
  }
}

uint32_t Hmi_Frames(void)          { return frames; }
uint32_t Hmi_AppFrames(void)       { return app_frames; }
uint32_t Hmi_TouchFrames(void)     { return touch_frames; }
uint32_t Hmi_BadFrames(void)       { return bad_frames; }
uint32_t Hmi_ReturnCodes(void)     { return return_codes; }
uint32_t Hmi_TxBytes(void)         { return tx_bytes; }
uint32_t Hmi_TxTruncated(void)     { return tx_truncated; }
uint8_t  Hmi_LastReturnCode(void)  { return last_return_code; }

/* ---- 默认空实现：应用不覆盖也能链接通过 ---- */
#if defined(__GNUC__)
__attribute__((weak))
#endif
void Hmi_OnAppFrame(uint8_t param1, uint8_t param2, uint8_t param3)
{
  (void)param1;
  (void)param2;
  (void)param3;
}

#if defined(__GNUC__)
__attribute__((weak))
#endif
void Hmi_OnTouch(uint8_t page, uint8_t component, uint8_t event)
{
  (void)page;
  (void)component;
  (void)event;
}
