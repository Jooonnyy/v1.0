#include "vehicle_app.h"
#include "align_app.h"
#include "align_config.h"
#include "chassis_config.h"
#include "chassis_io.h"
#include "chassis_motion.h"
#include "chassis_task.h"
#include "pick_app.h"
#include "pick_config.h"
#include "board.h"
#include "board_imu.h"
#include "board_step6.h"
#include "board_vision.h"
#include "board_hmi.h"
#include <stdio.h>
#include <string.h>

/* 新底板唯一应用入口。命令和回复与原 CHASSIS_STEP 台架一致，只是按层拆开：
 *   board_step6   驱动层：STEP/DIR/EN，不管路线
 *   chassis_motion 开环点动 / 平移 / 自旋
 *   chassis_task   比赛路线表
 *   本文件         命令解析和上电节拍
 * 不走 Board_Init()：那会把 TIM2/TIM3/TIM4 配成编码器，抢走 C/D/旋转的 STEP。 */

#define CMD_PORT        SERIAL_DEBUG
#define BT_PORT         SERIAL_BT
#define INPUT_COUNT     2U

typedef struct {
  uint8_t  port;
  uint8_t  len;
  uint8_t  overflow;
  uint8_t  idle_frame;
  uint32_t last_ms;
  char     line[CS_CMD_LEN];
} Input;

static Input inputs[INPUT_COUNT];
static uint32_t led_ms;
static uint8_t grab_pending;   /* 1 = ALIGN/TRACK 锁定后自动跑一次 GRAB */
static uint8_t grab_with_place;/* 1 = 这次 GRAB 抓完顺手放到载物台（GRAB ... HOLD 可关） */

/* 固件标识：`VER` 打出来，用来确认板上跑的确实是刚烧的那版。
 * 改功能时顺手改一次日期，省得"命令发了没反应"其实是烧的旧 HEX。 */
#define VEHICLE_FW_TAG  "2026-11-18 drop35"

static void cancel_auto(uint8_t announce)
{
  uint8_t was = (uint8_t)(ChassisTask_IsActive() | ChassisMotion_Busy());

  if (announce && ChassisTask_IsActive()) ChassisIo_Reply("TASK aborted\r\n");
  if (announce && grab_pending) ChassisIo_Reply("GRAB aborted\r\n");
  grab_pending = 0U;
  grab_with_place = 0U;
  Align_Cancel();          /* 视觉对准也属于自动运动，手动命令一来就退出 */
  Pick_DropAbort();        /* DROP 也停，升降由 Pick_Poll 延迟升回 */
  ChassisTask_Cancel();
  ChassisMotion_Cancel();
  if (was) Step6_StopAll();
}

static uint8_t split_tokens(char *line, char *tok[], uint8_t max_tokens)
{
  uint8_t count = 0U;
  char *p = line;

  while (*p != '\0' && count < max_tokens) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') break;
    tok[count++] = p;
    while (*p != '\0' && *p != ' ' && *p != '\t') p++;
    if (*p != '\0') { *p = '\0'; p++; }
  }
  return count;
}

static void to_upper(char *s)
{
  while (*s != '\0') {
    if (*s >= 'a' && *s <= 'z') *s = (char)(*s - 'a' + 'A');
    s++;
  }
}

static uint8_t to_int(const char *s, int32_t *out)
{
  int32_t value = 0;
  uint8_t negative = 0U;

  if (*s == '-') { negative = 1U; s++; }
  if (*s == '\0') return 1U;
  while (*s != '\0') {
    if (*s < '0' || *s > '9') return 1U;
    value = value * 10 + (int32_t)(*s - '0');
    if (value > 100000000) return 1U;
    s++;
  }
  *out = negative ? -value : value;
  return 0U;
}

/* 颜色/目标名 → 类别号：蓝牙上直接打 `GRAB BLUE` 比记数字方便。
 * 数字形式（1..6 / 10 / 11）仍然支持。返回 1 = 认出来了。 */
static uint8_t class_from_text(const char *text, int32_t *out)
{
  static const struct {
    const char *name;
    int32_t cls;
  } names[] = {
    { "RED", 1 }, { "YELLOW", 2 }, { "BLUE", 3 }, { "GREEN", 4 },
    { "BLACK", 5 }, { "LIGHTBLUE", 6 }, { "TRAY", 10 }, { "RING", 11 }
  };
  uint8_t i;

  for (i = 0U; i < (uint8_t)(sizeof(names) / sizeof(names[0])); i++) {
    if (strcmp(text, names[i].name) == 0) {
      *out = names[i].cls;
      return 1U;
    }
  }
  return 0U;
}

static int8_t axis_from_char(char c)
{
  switch (c) {
    case 'A': case 'a': return (int8_t)STEP6_A;
    case 'B': case 'b': return (int8_t)STEP6_B;
    case 'C': case 'c': return (int8_t)STEP6_C;
    case 'D': case 'd': return (int8_t)STEP6_D;
    default:            return -1;
  }
}

static void print_status(uint8_t port)
{
  char line[96];
  uint8_t i;

  (void)snprintf(line, sizeof(line),
                 "STATUS en=%u max=%lu Hz jog=%s task=%s step=%u hdg=%d\r\n",
                 (unsigned)Step6_IsEnabled(),
                 (unsigned long)Step6_GetMaxHz(),
                 ChassisMotion_JogActive()
                   ? ChassisMotion_AxisName(ChassisMotion_JogAxis()) : "off",
                 ChassisTask_IsActive() ? "on" : "off",
                 (unsigned)ChassisTask_StepNumber(),
                 (int)ChassisTask_HeadingDeg());
  ChassisIo_Say(port, line);
  for (i = 0U; i < 4U; i++) {
    (void)snprintf(line, sizeof(line), "  %s hz=%lu dir=%d\r\n",
                   ChassisMotion_AxisName(i),
                   (unsigned long)Step6_GetHz((Step6Axis)i),
                   (int)Step6_GetDir((Step6Axis)i));
    ChassisIo_Say(port, line);
  }
  (void)snprintf(line, sizeof(line), "  ROT hz=%lu dir=%d  LIFT hz=%lu dir=%d pos=%ld\r\n",
                 (unsigned long)Step6_GetHz(STEP6_ROT), (int)Step6_GetDir(STEP6_ROT),
                 (unsigned long)Step6_GetHz(STEP6_LIFT), (int)Step6_GetDir(STEP6_LIFT),
                 (long)Pick_LiftPosPulses());
  ChassisIo_Say(port, line);
  if (Pick_ArmIsOn()) {
    (void)snprintf(line, sizeof(line), "  SERVO arm=%u us plt=%d/1000\r\n",
                   (unsigned)Pick_ArmPulseUs(), (int)Pick_PlatGetPermille());
  } else {
    (void)snprintf(line, sizeof(line), "  SERVO arm=off plt=%d/1000\r\n",
                   (int)Pick_PlatGetPermille());
  }
  ChassisIo_Say(port, line);
  (void)snprintf(line, sizeof(line), "  CYC %s round=%u/%u  TRG %s  GRAB %s  DROP %s\r\n",
                 Pick_CycleIsActive() ? "on" : "off",
                 (unsigned)Pick_CycleRound(), (unsigned)Pick_CycleRounds(),
                 Pick_TrgIsActive() ? "on" : "off",
                 Pick_GrabIsActive() ? "on" : "off",
                 Pick_DropIsActive() ? "on" : "off");
  ChassisIo_Say(port, line);
  (void)snprintf(line, sizeof(line), "  ALIGN %s %s\r\n",
                 (Align_IsTracking() != 0U) ? "TRACK" : "ALIGN",
                 (Align_IsActive() != 0U) ? "on" : "off");
  ChassisIo_Say(port, line);
  (void)snprintf(line, sizeof(line), "  FW %s\r\n", VEHICLE_FW_TAG);
  ChassisIo_Say(port, line);
  {
    ImuSnapshot imu;
    (void)Imu_Read(&imu);
    (void)snprintf(line, sizeof(line), "  YAW %s yaw=%ld mdeg rate=%ld frames=%lu\r\n",
                   (imu.fresh != 0U) ? "ok" : ((imu.valid != 0U) ? "stale" : "none"),
                   (long)imu.yaw_mdeg, (long)imu.yaw_rate_mdps,
                   (unsigned long)imu.frames);
    ChassisIo_Say(port, line);
  }
}

static void print_pinmap(uint8_t port)
{
  ChassisIo_Say(port, "PINMAP  A STEP/PE9 TIM1_CH1  DIR/PE7\r\n");
  ChassisIo_Say(port, "        B STEP/PE6 TIM9_CH2  DIR/PE8\r\n");
  ChassisIo_Say(port, "        C STEP/PA0 TIM2_CH1  DIR/PE10\r\n");
  ChassisIo_Say(port, "        D STEP/PB0 TIM3_CH3  DIR/PE12\r\n");
  ChassisIo_Say(port, "        ROT STEP/PD12 TIM4_CH1 DIR/PE13\r\n");
  ChassisIo_Say(port, "        LIFT STEP/PA6 TIM13_CH1 DIR/PE14\r\n");
  ChassisIo_Say(port, "        EN  PD3 wheels, PC8 rot, PC9 lift (low active)\r\n");
}

static void print_help(uint8_t port)
{
  ChassisIo_Say(port, "CMDS  ?|S  H|HELP  P  E 0|1  F [hz]  VER  HINT\r\n");
  ChassisIo_Say(port, "      M <a-d> <permille>   MALL <permille>\r\n");
  ChassisIo_Say(port, "      L|R <mm> [mm/s]      A <deg>        D <a-d> <0|1>  J <a-d> <steps> [hz]\r\n");
  ChassisIo_Say(port, "      TASK    STOP\r\n");
  ChassisIo_Say(port, "      STP 1..7  run one stp section (car at that start pose)\r\n");
  ChassisIo_Say(port, "      DRY       full route without vision (chassis + ring moves only)\r\n");
  ChassisIo_Say(port, "      ROT <deg> [hz]       LIFT <pulses>|HOME|ZERO [?] [hz]\r\n");
  ChassisIo_Say(port, "      ARM <deg>|OFF        PLT <ms>|OFF        CYC [n]\r\n");
  ChassisIo_Say(port, "      TRG [n]   (platform -> ground)\r\n");
  ChassisIo_Say(port, "      Y yaw   Z zero yaw   X 0|1 hold off/on\r\n");
  ChassisIo_Say(port, "      V vision snapshot   ALIGN <cls> [tol_mm]|STOP|?\r\n");
  ChassisIo_Say(port, "      DROP [1-9]  align that ring empty, then pick from platform and place\r\n");
  ChassisIo_Say(port, "      HINT <cls>     send $C to the Pi (tests the downlink only)\r\n");
  ChassisIo_Say(port, "      IN             read key/top/bottom/home inputs\r\n");
  ChassisIo_Say(port, "      TASK <code>    show 15-char task code on HMI (page 0)\r\n");
  ChassisIo_Say(port, "      STAT <g> <p>   show grab/place counts on HMI (page 1)\r\n");
  ChassisIo_Say(port, "      SCAN           tell the Pi to scan the QR at the scan area\r\n");
  ChassisIo_Say(port, "      TRACK <cls> [tol_mm]  follow moving target, STOP ends\r\n");
  ChassisIo_Say(port, "      GRAB <cls> [tol_mm] [HOLD]  track, lock, pick+place\r\n");
  ChassisIo_Say(port, "      PLACE   put the held part on the platform\r\n");
  ChassisIo_Say(port, "        cls 1-6 color / 10 tray / 11 ring\r\n");
}

/* ---- 任务码：4 组三位数，组间 '+'，共 15 字符；每位 1..6 ---- */
static uint8_t task_code_valid(const char *code)
{
  uint8_t i;

  if (code == 0 || strlen(code) != 15U) return 0U;
  for (i = 0U; i < 15U; i++) {
    if (i == 3U || i == 7U || i == 11U) {
      if (code[i] != '+') return 0U;
    } else if (code[i] < '1' || code[i] > '6') {
      return 0U;
    }
  }
  return 1U;
}

/* 任务码上屏：page 0 的 t0 = 前 7 字符，t1 = 后 8 字符（屏工程见 HMI 文档）。 */
static void hmi_show_task(const char *code)
{
  char line1[8];
  char line2[9];

  memcpy(line1, code, 7U);
  line1[7] = '\0';
  memcpy(line2, code + 7U, 8U);
  line2[8] = '\0';
  Hmi_SetPage(0U);
  Hmi_SetText("t0", line1);
  Hmi_SetText("t1", line2);
}

/* 统计上屏：page 1 的 t2 / t3。 */
static void hmi_show_stats(uint16_t grab_ok, uint16_t place_ok)
{
  char text[8];

  Hmi_SetPage(1U);
  (void)snprintf(text, sizeof(text), "%u", (unsigned)grab_ok);
  Hmi_SetText("t2", text);
  (void)snprintf(text, sizeof(text), "%u", (unsigned)place_ok);
  Hmi_SetText("t3", text);
}

static void handle_line(char *line, const Input *in)
{
  char *tok[4];
  uint8_t n;
  int32_t value, extra;
  int8_t axis;
  char buf[80];

  ChassisIo_SetReplyPort(in->port);
  to_upper(line);
  n = split_tokens(line, tok, 4U);
  if (n == 0U) return;

  /* CYC / TRG 在跑时拒绝机构手动命令，免得和序列打架；`OFF` 仍放行，方便随时断舵机。 */
  if ((Pick_CycleIsActive() || Pick_TrgIsActive() || Pick_GrabIsActive() || Pick_DropIsActive()) &&
      (strcmp(tok[0], "LIFT") == 0 || strcmp(tok[0], "ROT") == 0 ||
       strcmp(tok[0], "ARM") == 0 || strcmp(tok[0], "PLT") == 0)) {
    if (n < 2U || strcmp(tok[1], "OFF") != 0) {
      ChassisIo_Reply("ERR sequence running, send STOP\r\n");
      return;
    }
  }

  if (strcmp(tok[0], "?") == 0 || strcmp(tok[0], "S") == 0) {
    print_status(in->port);
  } else if (strcmp(tok[0], "H") == 0 || strcmp(tok[0], "HELP") == 0) {
    print_help(in->port);
  } else if (strcmp(tok[0], "VER") == 0 || strcmp(tok[0], "VERSION") == 0) {
    (void)snprintf(buf, sizeof(buf), "FW %s\r\n", VEHICLE_FW_TAG);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "IN") == 0) {
    /* 读输入口：key/top/bottom/home。用来确认上限位有没有接、有没有压到。 */
    (void)snprintf(buf, sizeof(buf), "IN key=%u top=%u bottom=%u home=%u\r\n",
                   (unsigned)Input_Active(INPUT_KEY),
                   (unsigned)Input_Active(INPUT_LIMIT_TOP),
                   (unsigned)Input_Active(INPUT_LIMIT_BOTTOM),
                   (unsigned)Input_Active(INPUT_HOME));
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "STAT") == 0) {
    int32_t grab_ok = 0;
    int32_t place_ok = 0;
    if (n < 3U || to_int(tok[1], &grab_ok) != 0U || to_int(tok[2], &place_ok) != 0U ||
        grab_ok < 0 || grab_ok > 999 || place_ok < 0 || place_ok > 999) {
      ChassisIo_Reply("ERR STAT <0-999> <0-999>\r\n");
      return;
    }
    hmi_show_stats((uint16_t)grab_ok, (uint16_t)place_ok);
    ChassisIo_Reply("OK STAT\r\n");
  } else if (strcmp(tok[0], "SCAN") == 0) {
    /* 通知树莓派"到扫码区了"：下行一行 @SCAN，树莓派收到后扫码。 */
    static const char scan_line[] = "@SCAN\n";
    (void)Serial_Write(SERIAL_RPI, (const uint8_t *)scan_line,
                       (uint16_t)(sizeof(scan_line) - 1U), 20U);
    ChassisIo_Reply("OK SCAN\r\n");
  } else if (strcmp(tok[0], "HINT") == 0) {
    /* 只把类别下发给树莓派（USART3 上的 `$C,<cls>`），不动任何电机。
     * 用来单独验证"STM32 → 树莓派"这条下行线，不用真去抓一次。 */
    if (n < 2U ||
        (class_from_text(tok[1], &value) == 0U &&
         (to_int(tok[1], &value) != 0U || value < 1 || value > 11))) {
      ChassisIo_Reply("ERR HINT <cls 1-6|10|11|BLUE...>\r\n");
      return;
    }
    Align_SendClassHint((uint8_t)value);
    (void)snprintf(buf, sizeof(buf), "OK HINT cls=%ld, sent $C\r\n", (long)value);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "P") == 0) {
    print_pinmap(in->port);
  } else if (strcmp(tok[0], "Y") == 0) {
    ImuSnapshot imu;
    (void)Imu_Read(&imu);
    (void)snprintf(buf, sizeof(buf), "YAW yaw=%ld mdeg rate=%ld mdps age=%lu ms\r\n",
                   (long)imu.yaw_mdeg, (long)imu.yaw_rate_mdps, (unsigned long)imu.age_ms);
    ChassisIo_Reply(buf);
    (void)snprintf(buf, sizeof(buf), "    frames=%lu bad=%lu %s\r\n",
                   (unsigned long)imu.frames, (unsigned long)imu.bad_frames,
                   (imu.fresh != 0U) ? "ok" : "no fresh frame");
    ChassisIo_Reply(buf);
    (void)snprintf(buf, sizeof(buf), "    raw=%lu drop=%lu baud=%lu\r\n",
                   (unsigned long)Imu_RawByteCount(),
                   (unsigned long)Serial_Dropped(SERIAL_IMU),
                   (unsigned long)BOARD_IMU_BAUD);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "Z") == 0) {
    if (Imu_HasFrame() == 0U) { ChassisIo_Reply("ERR no HWT101 frame yet\r\n"); return; }
    Imu_Zero();
    ChassisIo_Reply("OK yaw zeroed\r\n");
  } else if (strcmp(tok[0], "X") == 0) {
    /* 航向保持开关：现场做 A/B 对比用（不影响闭环转身）。 */
    if (n < 2U) {
      (void)snprintf(buf, sizeof(buf), "hold=%u\r\n",
                     (unsigned)ChassisMotion_GetHoldEnable());
      ChassisIo_Reply(buf);
      return;
    }
    if (to_int(tok[1], &value) != 0U || (value != 0 && value != 1)) {
      ChassisIo_Reply("ERR X <0|1>\r\n");
      return;
    }
    ChassisMotion_SetHoldEnable((uint8_t)value);
    (void)snprintf(buf, sizeof(buf), "OK hold=%ld\r\n", (long)value);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "E") == 0) {
    if (n < 2U || to_int(tok[1], &value) != 0U || (value != 0 && value != 1)) {
      ChassisIo_Reply("ERR E 0|1\r\n");
      return;
    }
    if (value == 0) {
      cancel_auto(1U);
      Pick_StopAll();
    }
    Step6_EnableAll((uint8_t)value);
    ChassisIo_Reply(value ? "OK enabled\r\n" : "OK disabled\r\n");
  } else if (strcmp(tok[0], "F") == 0) {
    if (n < 2U) {
      (void)snprintf(buf, sizeof(buf), "max=%lu Hz\r\n", (unsigned long)Step6_GetMaxHz());
      ChassisIo_Reply(buf);
      return;
    }
    if (to_int(tok[1], &value) != 0U || value < 100 || value > (int32_t)STEP6_HZ_HARD_LIMIT) {
      ChassisIo_Reply("ERR F 100..40000\r\n");
      return;
    }
    Step6_SetMaxHz((uint32_t)value);
    (void)snprintf(buf, sizeof(buf), "OK max=%lu Hz\r\n", (unsigned long)Step6_GetMaxHz());
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "MALL") == 0) {
    if (n < 2U || to_int(tok[1], &value) != 0U ||
        value < -CS_PERMILLE_LIMIT || value > CS_PERMILLE_LIMIT) {
      ChassisIo_Reply("ERR MALL -1000..1000\r\n");
      return;
    }
    if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return; }
    cancel_auto(1U);
    for (uint8_t i = 0U; i < 4U; i++) {
      (void)Step6_SetPermille((Step6Axis)i, (int16_t)value);
    }
    (void)snprintf(buf, sizeof(buf), "OK MALL %ld/1000\r\n", (long)value);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "L") == 0 || strcmp(tok[0], "R") == 0) {
    /* 左移/右移按**距离**走：第一个参数是毫米，第二个可选是速度 mm/s。
     * 方向由命令字母定（L = 车体左侧），走够自动停，有陀螺时自动保持航向。 */
    uint8_t left = (uint8_t)(tok[0][0] == 'L');
    uint32_t mm;
    uint32_t mm_per_s = CS_TASK_STRAFE_MM_PER_S;

    if (n < 2U || to_int(tok[1], &value) != 0U || value <= 0 || value > 5000) {
      ChassisIo_Reply("ERR L|R <mm> [mm/s]\r\n");
      return;
    }
    if (n >= 3U) {
      if (to_int(tok[2], &extra) != 0U || extra < 50 || extra > 1000) {
        ChassisIo_Reply("ERR L|R <mm> [mm/s]\r\n");
        return;
      }
      mm_per_s = (uint32_t)extra;
    }
    if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return; }
    cancel_auto(1U);
    mm = (uint32_t)value;
    if (ChassisMotion_Translate(left ? -1 : 1, left ? 1 : -1,
                                left ? 1 : -1, left ? -1 : 1, mm, mm_per_s) == 0U) {
      ChassisIo_Reply("ERR L|R start failed\r\n");
      return;
    }
    (void)snprintf(buf, sizeof(buf), "OK %s %lu mm @ %lu mm/s\r\n",
                   tok[0], (unsigned long)mm, (unsigned long)mm_per_s);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "M") == 0) {
    if (n < 3U) { ChassisIo_Reply("ERR M <a-d> <permille>\r\n"); return; }
    axis = axis_from_char(tok[1][0]);
    if (axis < 0 || tok[1][1] != '\0' ||
        to_int(tok[2], &value) != 0U ||
        value < -CS_PERMILLE_LIMIT || value > CS_PERMILLE_LIMIT) {
      ChassisIo_Reply("ERR M <a-d> -1000..1000\r\n");
      return;
    }
    if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return; }
    cancel_auto(1U);
    (void)Step6_SetPermille((Step6Axis)axis, (int16_t)value);
    (void)snprintf(buf, sizeof(buf), "OK M %s %ld/1000\r\n",
                   ChassisMotion_AxisName((uint8_t)axis), (long)value);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "D") == 0) {
    if (n < 3U) { ChassisIo_Reply("ERR D <a-d> <0|1>\r\n"); return; }
    axis = axis_from_char(tok[1][0]);
    if (axis < 0 || to_int(tok[2], &value) != 0U || (value != 0 && value != 1)) {
      ChassisIo_Reply("ERR D <a-d> <0|1>\r\n");
      return;
    }
    cancel_auto(1U);
    {
      uint32_t hz_now = Step6_GetHz((Step6Axis)axis);
      Step6_Stop((Step6Axis)axis);
      (void)Step6_SetHz((Step6Axis)axis, value ? (int32_t)hz_now : -(int32_t)hz_now);
    }
    (void)snprintf(buf, sizeof(buf), "OK D %s=%ld\r\n",
                   ChassisMotion_AxisName((uint8_t)axis), (long)value);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "J") == 0) {
    if (n < 3U) { ChassisIo_Reply("ERR J <a-d> <steps> [hz]\r\n"); return; }
    axis = axis_from_char(tok[1][0]);
    if (axis < 0 || to_int(tok[2], &value) != 0U) {
      ChassisIo_Reply("ERR J <a-d> <steps> [hz]\r\n");
      return;
    }
    extra = (int32_t)(((int32_t)Step6_GetMaxHz() * CS_JOG_DEF_PERMILLE) / 1000);
    if (n >= 4U) {
      if (to_int(tok[3], &extra) != 0U || extra < 1 ||
          extra > (int32_t)STEP6_HZ_HARD_LIMIT) {
        ChassisIo_Reply("ERR J hz 1..40000\r\n");
        return;
      }
    }
    cancel_auto(1U);
    ChassisMotion_JogStart((uint8_t)axis, value, (uint32_t)extra);
    (void)snprintf(buf, sizeof(buf), "J %s %ld steps @ %ld Hz\r\n",
                   ChassisMotion_AxisName((uint8_t)axis), (long)value, (long)extra);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "A") == 0) {
    if (n < 2U || to_int(tok[1], &value) != 0U || value < -360 || value > 360 || value == 0) {
      ChassisIo_Reply("ERR A -360..360, not 0\r\n");
      return;
    }
    if (!Step6_IsEnabled()) { ChassisIo_Reply("ERR not enabled\r\n"); return; }
    cancel_auto(1U);
    if (ChassisMotion_Spin((int16_t)value, ChassisTask_HeadingDeg()) == 0U) {
      ChassisIo_Reply("ERR spin start failed\r\n");
      return;
    }
    (void)snprintf(buf, sizeof(buf), "OK A %ld deg\r\n", (long)value);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "ROT") == 0) {
    if (n >= 2U && strcmp(tok[1], "?") == 0) {
      (void)snprintf(buf, sizeof(buf), "ROT %d deg (tracked)\r\n",
                     (int)Pick_RotGetDeg());
      ChassisIo_Reply(buf);
      return;
    }
    if (n < 2U || to_int(tok[1], &value) != 0U) {
      ChassisIo_Reply("ERR ROT <deg> [hz]\r\n");
      return;
    }
    extra = (int32_t)PK_ROT_HZ;
    if (n >= 3U) {
      if (to_int(tok[2], &extra) != 0U || extra < 100 || extra > (int32_t)STEP6_HZ_HARD_LIMIT) {
        ChassisIo_Reply("ERR ROT hz 100..40000\r\n");
        return;
      }
    }
    (void)Pick_RotDeg(value, (uint32_t)extra);
  } else if (strcmp(tok[0], "LIFT") == 0) {
    /* 子命令：HOME 回起点、ZERO 把当前位置设为原点、? 查询累计位置。 */
    extra = (int32_t)PK_LIFT_HZ;
    if (n >= 3U) {
      if (to_int(tok[2], &extra) != 0U || extra < 100 || extra > (int32_t)STEP6_HZ_HARD_LIMIT) {
        ChassisIo_Reply("ERR LIFT hz 100..40000\r\n");
        return;
      }
    }
    if (n >= 2U && strcmp(tok[1], "HOME") == 0) {
      (void)Pick_LiftGoHome((uint32_t)extra);
      return;
    }
    if (n >= 2U && strcmp(tok[1], "ZERO") == 0) {
      Pick_LiftZeroPos();
      return;
    }
    if (n >= 2U && strcmp(tok[1], "?") == 0) {
      (void)snprintf(buf, sizeof(buf), "LIFT pos=%ld pulses\r\n",
                     (long)Pick_LiftPosPulses());
      ChassisIo_Reply(buf);
      return;
    }
    /* 点动距离：第一个参数是脉冲数，不是速度；第二个参数（可选）才是走多快。 */
    if (n < 2U || to_int(tok[1], &value) != 0U) {
      ChassisIo_Reply("ERR LIFT <pulses>|HOME|ZERO|? [hz]\r\n");
      return;
    }
    (void)Pick_LiftPulses(value, (uint32_t)extra);
  } else if (strcmp(tok[0], "ARM") == 0) {
    if (n < 2U) { ChassisIo_Reply("ERR ARM <deg>|OFF\r\n"); return; }
    if (strcmp(tok[1], "OFF") == 0) { Pick_ArmOff(); return; }
    if (to_int(tok[1], &value) != 0U) { ChassisIo_Reply("ERR ARM <deg>|OFF\r\n"); return; }
    (void)Pick_ArmDeg(value);
  } else if (strcmp(tok[0], "PLT") == 0) {
    /* 只给时间：速度固定 PK_PLAT_JOG_PERMILLE（40%），负毫秒 = 反方向。 */
    if (n < 2U) { ChassisIo_Reply("ERR PLT <ms>|OFF\r\n"); return; }
    if (strcmp(tok[1], "OFF") == 0) { Pick_PlatOff(); return; }
    if (to_int(tok[1], &value) != 0U || value == 0 ||
        value < -(int32_t)PK_PLAT_MAX_MS || value > (int32_t)PK_PLAT_MAX_MS) {
      (void)snprintf(buf, sizeof(buf), "ERR PLT ms -%ld..%ld, not 0\r\n",
                     (long)PK_PLAT_MAX_MS, (long)PK_PLAT_MAX_MS);
      ChassisIo_Reply(buf);
      return;
    }
    (void)Pick_PlatMove((int16_t)((value > 0) ? (int32_t)PK_PLAT_JOG_PERMILLE
                                              : -(int32_t)PK_PLAT_JOG_PERMILLE),
                        (uint32_t)((value > 0) ? value : -value));
  } else if (strcmp(tok[0], "CYC") == 0) {
    /* 任务循环：参数是轮数，省略用 PK_CYC_ROUNDS_DEFAULT。 */
    value = (int32_t)PK_CYC_ROUNDS_DEFAULT;
    if (n >= 2U) {
      if (to_int(tok[1], &value) != 0U ||
          value < 1 || value > (int32_t)PK_CYC_MAX_ROUNDS) {
        (void)snprintf(buf, sizeof(buf), "ERR CYC 1..%ld rounds\r\n",
                       (long)PK_CYC_MAX_ROUNDS);
        ChassisIo_Reply(buf);
        return;
      }
    }
    Pick_CycleStart((uint8_t)value);
  } else if (strcmp(tok[0], "TRG") == 0) {
    /* 卸料：载物台夹起 → 降到地面 → 松开 → 升回。参数是轮数，省略用 PK_TRG_ROUNDS_DEFAULT。 */
    value = (int32_t)PK_TRG_ROUNDS_DEFAULT;
    if (n >= 2U) {
      if (to_int(tok[1], &value) != 0U ||
          value < 1 || value > (int32_t)PK_TRG_MAX_ROUNDS) {
        (void)snprintf(buf, sizeof(buf), "ERR TRG 1..%ld rounds\r\n",
                       (long)PK_TRG_MAX_ROUNDS);
        ChassisIo_Reply(buf);
        return;
      }
    }
    Pick_TrgStart((uint8_t)value);
  } else if (strcmp(tok[0], "TASK") == 0) {
    /* 带任务码：只上屏（page 0 的 t0/t1），不启动路线；不带参数：原来的路线启动。 */
    if (n >= 2U) {
      if (task_code_valid(tok[1]) == 0U) {
        ChassisIo_Reply("ERR TASK FORMAT\r\n");
        return;
      }
      if (ChassisTask_SetPlan(tok[1]) == 0U) {
        ChassisIo_Reply("ERR TASK RING 1..3\r\n");
        return;
      }
      {
        char plan_line[24];
        int plan_len = snprintf(plan_line, sizeof(plan_line), "@PLAN %s\n", tok[1]);
        if (plan_len > 0 && plan_len < (int)sizeof(plan_line)) {
          (void)Serial_Write(SERIAL_RPI, (const uint8_t *)plan_line,
                             (uint16_t)plan_len, 20U);
        }
      }
      hmi_show_task(tok[1]);
      ChassisIo_Reply("OK TASK\r\n");
      return;
    }
    if (grab_pending) ChassisIo_Reply("GRAB aborted\r\n");
    grab_pending = 0U;
    Align_Cancel();          /* 任务要抢四轮，对准不能留着 */
    ChassisTask_Start();
  } else if (strcmp(tok[0], "DRY") == 0) {
    /* 不接视觉：只跑底盘路线和换环位置，跳过扫码等待/对准/抓放。 */
    if (grab_pending) ChassisIo_Reply("GRAB aborted\r\n");
    grab_pending = 0U;
    Align_Cancel();
    ChassisTask_StartDry();
  } else if (strcmp(tok[0], "STP") == 0) {
    /* 单独跑 stp1..stp7 中某一段：车要手动摆到该段起点、朝向按 0 处理。 */
    if (n < 2U || to_int(tok[1], &value) != 0U || value < 1 || value > 7) {
      ChassisIo_Reply("ERR STP 1..7\r\n");
      return;
    }
    if (grab_pending) ChassisIo_Reply("GRAB aborted\r\n");
    grab_pending = 0U;
    Align_Cancel();
    (void)ChassisTask_StartStep((uint8_t)value);
  } else if (strcmp(tok[0], "V") == 0) {
    /* 视觉链路体检：先看统计能不能收到帧，再看最近一帧的内容。 */
    const VisionTarget *target = Vision_ReadTarget();
    VisionStats stats;

    Vision_ReadStats(&stats);
    (void)snprintf(buf, sizeof(buf), "VIS ok=%lu bad=%lu rep=%lu gap=%lu\r\n",
                   (unsigned long)stats.frames_ok, (unsigned long)stats.frames_bad,
                   (unsigned long)stats.frames_repeat, (unsigned long)stats.frames_gap);
    ChassisIo_Reply(buf);
    (void)snprintf(buf, sizeof(buf), "    ovf=%lu drp=%lu\r\n",
                   (unsigned long)stats.bytes_overflow,
                   (unsigned long)stats.bytes_dropped);
    ChassisIo_Reply(buf);
    (void)snprintf(buf, sizeof(buf), "TGT cls=%u valid=%u seq=%lu stale=%u\r\n",
                   (unsigned)target->class_code, (unsigned)target->valid,
                   (unsigned long)target->seq,
                   (unsigned)((Vision_TargetFresh(Board_NowMs(), VD_STALE_MS) == 0U) ? 1U : 0U));
    ChassisIo_Reply(buf);
    (void)snprintf(buf, sizeof(buf), "    x=%d mm y=%d mm yaw=%ld conf=%u age=%u\r\n",
                   (int)target->x_mm, (int)target->y_mm, (long)target->yaw_mdeg,
                   (unsigned)target->confidence_permille, (unsigned)target->age_ms);
    ChassisIo_Reply(buf);
  } else if (strcmp(tok[0], "ALIGN") == 0 || strcmp(tok[0], "GRAB") == 0 ||
             strcmp(tok[0], "TRACK") == 0) {
    /* ALIGN 一次性微调底盘；TRACK 连续跟随；GRAB = 追踪到锁定后下降夹取。
     * <cls> [tol_mm] [HOLD] 启动 / STOP 中止 / 无参数或 ? 查状态。
     * GRAB 默认抓完顺手放到载物台；加 HOLD 只夹不放（调试/分步用）。 */
    uint8_t grab = (uint8_t)(tok[0][0] == 'G');
    uint8_t hold = 0U;
    /* GRAB 也走追踪：物料在动时能等它停稳再夹；物料静止时锁定条件和原来的对准一样。 */
    uint8_t track = (uint8_t)((tok[0][0] == 'T') || (grab != 0U));

    if (n >= 2U && strcmp(tok[1], "STOP") == 0) {
      if (Pick_GrabIsActive() != 0U) {
        /* 夹取序列还在跑（对准已结束）：同一条命令也要能停它。 */
        ChassisIo_Reply("GRAB aborted: STOP\r\n");
        grab_pending = 0U;
        Align_Cancel();
        Pick_StopAll();
        Step6_StopAll();
        (void)snprintf(buf, sizeof(buf), "%s stopped\r\n", tok[0]);
        ChassisIo_Reply(buf);
        return;
      }
      if (Align_IsActive() == 0U && grab_pending == 0U) {
        (void)snprintf(buf, sizeof(buf), "%s not running\r\n", tok[0]);
        ChassisIo_Reply(buf);
        return;
      }
      if (grab_pending) ChassisIo_Reply("GRAB aborted\r\n");
      grab_pending = 0U;
      Align_Cancel();
      ChassisMotion_Cancel();
      Step6_StopAll();
      (void)snprintf(buf, sizeof(buf), "%s stopped\r\n", tok[0]);
      ChassisIo_Reply(buf);
      return;
    }
    if (n < 2U || strcmp(tok[1], "?") == 0) {
      Align_PrintStatus();
      if (grab_pending) ChassisIo_Reply("GRAB waiting for lock\r\n");
      return;
    }
    if (class_from_text(tok[1], &value) == 0U &&
        (to_int(tok[1], &value) != 0U || value < 1 || value > 11)) {
      (void)snprintf(buf, sizeof(buf),
                     "ERR %s <cls 1-6|10|11|RED|YELLOW|BLUE|GREEN|BLACK|LIGHTBLUE> [tol_mm]\r\n",
                     tok[0]);
      ChassisIo_Reply(buf);
      return;
    }
    extra = 0;
    if (n >= 3U) {
      if (strcmp(tok[2], "HOLD") == 0) {
        hold = 1U;
      } else if (to_int(tok[2], &extra) != 0U || extra < 0 || extra > 32767) {
        ChassisIo_Reply("ERR tol must be mm (or HOLD)\r\n");
        return;
      }
    }
    if (n >= 4U && strcmp(tok[3], "HOLD") == 0) hold = 1U;
    if (Pick_CycleIsActive() || Pick_TrgIsActive() || Pick_GrabIsActive() || Pick_DropIsActive()) {
      ChassisIo_Reply("ERR sequence running, send STOP\r\n");
      return;
    }
    /* 对准要花好几秒，位置不对就别等对准完才拒绝。 */
    if (grab && (Pick_LiftPosPulses() != 0 || Pick_RotIsMoving() != 0U)) {
      ChassisIo_Reply("ERR GRAB lift not home, send LIFT HOME\r\n");
      return;
    }
    cancel_auto(1U);              /* 先停掉 TASK / 残留运动，再开始对准 */
    grab_pending = grab;
    grab_with_place = (uint8_t)((grab != 0U && hold == 0U) ? 1U : 0U);
    if (((track != 0U) ? Align_StartEx((uint8_t)value, (int16_t)extra, 1U)
                       : Align_Start((uint8_t)value, (int16_t)extra)) == 0U) {
      grab_pending = 0U;
      grab_with_place = 0U;
    } else if (grab) {
      ChassisIo_Reply((hold != 0U)
                        ? "GRAB armed, will pick and hold\r\n"
                        : "GRAB armed, will pick and place when locked\r\n");
    }
  } else if (strcmp(tok[0], "DROP") == 0) {
    /* 圆环放置：先空爪对准第 n 个环心，再取料、下放；对准后底盘不再动。 */
    if (Pick_CycleIsActive() || Pick_TrgIsActive() || Pick_GrabIsActive() || Pick_DropIsActive()) {
      ChassisIo_Reply("ERR sequence running, send STOP\r\n");
      return;
    }
    if (n >= 2U && (to_int(tok[1], &value) != 0U || value < 1 || value > 9)) {
      ChassisIo_Reply("ERR DROP <1-9>\r\n");
      return;
    }
    cancel_auto(1U);
    Pick_DropStart((n >= 2U) ? (uint8_t)value : 1U);
  } else if (strcmp(tok[0], "PLACE") == 0) {
    /* 手里已经夹着料、停在最高处：只做放料那半段（GRAB ... HOLD 之后手动补一次）。 */
    cancel_auto(1U);
    Pick_PlaceStart();
  } else if (strcmp(tok[0], "STOP") == 0) {
    if (Pick_DropIsActive()) {
      ChassisIo_Reply("DROP aborted: STOP, lifting home\r\n");
      Pick_DropAbort();
    }
    if (Pick_CycleIsActive()) ChassisIo_Reply("CYC aborted: STOP\r\n");
    if (Pick_TrgIsActive()) ChassisIo_Reply("TRG aborted: STOP\r\n");
    if (Pick_GrabIsActive()) ChassisIo_Reply("GRAB aborted: STOP\r\n");
    Pick_StopAll();
    cancel_auto(1U);
    grab_with_place = 0U;
    Step6_StopAll();
    ChassisIo_Reply("OK stopped\r\n");
  } else {
    ChassisIo_Reply("ERR unknown, try H\r\n");
  }
}

/* 树莓派经 USART3 下发的非视觉行（TASK/STAT）。只放行这两类，
 * 避免 Pi 误发 E/M/LIFT 等命令直接驱动车辆。 */
static void handle_rpi_line(const char *text)
{
  char copy[CS_CMD_LEN];
  Input input;
  size_t length;

  if (text == 0) return;
  if (strncmp(text, "TASK ", 5U) != 0 && strncmp(text, "STAT ", 5U) != 0) return;
  length = strlen(text);
  if (length == 0U || length >= sizeof(copy)) return;
  memcpy(copy, text, length + 1U);
  memset(&input, 0, sizeof(input));
  input.port = (uint8_t)SERIAL_RPI;
  handle_line(copy, &input);
}

static void poll_inputs(void)
{
  int byte;
  uint8_t i;

  for (i = 0U; i < INPUT_COUNT; i++) {
    Input *in = &inputs[i];
    while ((byte = Serial_ReadByte((SerialPort)in->port)) >= 0) {
      char c = (char)byte;
      in->last_ms = Board_NowMs();
      if (c == '\r' || c == '\n') {
        if (in->overflow != 0U) {
          in->overflow = 0U;
          in->len = 0U;
        } else if (in->len > 0U) {
          in->line[in->len] = '\0';
          in->len = 0U;
          handle_line(in->line, in);
        }
      } else if (in->len < (CS_CMD_LEN - 1U)) {
        in->line[in->len++] = c;
      } else if (in->overflow == 0U) {
        in->overflow = 1U;
        ChassisIo_Say(in->port, "ERR command too long\r\n");
      }
    }

    if (in->idle_frame != 0U &&
        (uint32_t)(Board_NowMs() - in->last_ms) >= CS_LINE_IDLE_MS) {
      if (in->overflow != 0U) {
        in->overflow = 0U;
        in->len = 0U;
      } else if (in->len > 0U) {
        in->line[in->len] = '\0';
        in->len = 0U;
        handle_line(in->line, in);
      }
    }
  }
}

void VehicleApp_Begin(void)
{
  uint8_t i;

  for (i = 0U; i < INPUT_COUNT; i++) {
    inputs[i].len = 0U;
    inputs[i].overflow = 0U;
    inputs[i].last_ms = Board_NowMs();
  }
  inputs[0].port = (uint8_t)CMD_PORT;
  inputs[0].idle_frame = 0U;
  inputs[1].port = (uint8_t)BT_PORT;
  inputs[1].idle_frame = 1U;
  ChassisIo_SetReplyPort((uint8_t)CMD_PORT);
  led_ms = Board_NowMs();
  ChassisMotion_Init();
  ChassisTask_Init();
  Vision_Init();
  Vision_SetLineHandler(handle_rpi_line);
  Align_Init();
  Hmi_Init();

  Led_StatusInit();
  Step6_EnableAll(0U);
  Step6_StopAll();

  for (i = 0U; i < INPUT_COUNT; i++) {
    ChassisIo_Say(inputs[i].port,
                  "\r\nCHASSIS_STEP ready. 4 steppers, open loop, no encoder.\r\n");
    ChassisIo_Say(inputs[i].port, "Send E 1 to enable, then M/MALL/J. H for help.\r\n");
  }
}

void VehicleApp_Poll(void)
{
  ChassisMotionEvent event;

  poll_inputs();
  Hmi_Poll();
  Imu_Poll();                       /* 先把陀螺串口里的字节拼成帧，下面才知道 yaw */
  Vision_Poll();                    /* $V 目标流：只拼帧，不发任何东西 */
  ChassisMotion_PollJog();
  ChassisMotion_PollMove(ChassisTask_IsActive(), &event);
  if (event.timed_out && ChassisTask_IsActive()) ChassisTask_Abort("move timeout");
  else if (event.set_heading) ChassisTask_SetHeadingDeg(event.heading_deg);
  ChassisTask_Poll();
  Align_Poll();
  /* GRAB 挂起时：追踪已经锁定、且这一拍真的在容差内，就按"对准成功"收尾，
   * 下面那一段紧接着启动下降夹取。ALIGN 模式（track=0）这里永远返回 0。 */
  if (grab_pending != 0U) {
    (void)Align_TryCompleteForGrab();
  }
  /* GRAB = 对准成功才下降。失败、超时、被取消都不许夹。 */
  if (grab_pending != 0U && Align_IsActive() == 0U) {
    grab_pending = 0U;
    if (Align_LastReason() == ALIGN_REASON_DONE) {
      Pick_GrabStart(grab_with_place);
      grab_with_place = 0U;
    } else {
      grab_with_place = 0U;
      ChassisIo_Reply("GRAB aborted: align failed\r\n");
    }
  }
  Pick_Poll();

  if ((Board_NowMs() - led_ms) >= CS_LED_HEARTBEAT_MS) {
    led_ms = Board_NowMs();
    Led_StatusToggle();
  }
}
