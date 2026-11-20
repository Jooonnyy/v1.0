#include "hmi_smoke.h"
#include "board.h"
#include "board_hmi.h"
#include <stdio.h>
#include <string.h>

/*
 * 串口屏任务码显示联调（HMI Target）
 *
 * 链路：上位机 --USART1 或 USART3 / 115200--> STM32 --USART6/115200--> TJC 串口屏
 *
 * 命令来源由 HMI_COMMAND_PORT 决定：台架调试默认 USART1（省一个 USB-TTL），
 * 接树莓派时改成 SERIAL_RPI（USART3）。
 *
 * 树莓派扫二维码得到任务码（形如 156+123+516+231）后，通过命令串口发一行：
 *     TASK 156+123+516+231\n
 * 本程序校验格式，把 15 个字符拆成两行推到屏上：
 *     t0 = "156+123"
 *     t1 = "+516+231"
 * 比赛结束时树莓派发：
 *     STAT <抓取正确次数> <放置正确次数>\n
 * 屏上 t2 / t3 分别显示 "GRAB OK n" / "PLACE OK n"。
 *
 * 为什么拆两行：以 3.2 英寸 320x240 屏为例，12 mm 字高是 59 px，
 * 15 个字符一行约需 99 mm，屏宽只有 65 mm（详见 Docs/串口屏通讯.md）。
 * 工程里现成的 64 px ASCII 字库也能放下 8 个字符（约 282 px），可以直接用。
 *
 * 屏幕工程要求：三个页面，objname 必须完全一致：
 *   page 0：t0 / t1 —— 任务码两行（64 px 或 59 px ASCII 字库）
 *   page 1：t2 / t3 —— 统计两行（32 px ASCII 字库）
 *   page 2：b0      —— 启动页，只有一个 START 按钮
 * TJC 的串口指令只作用于**当前页**，所以页面由本程序切换：
 *   开机停在 page 2；按钮弹起事件在屏端写 printh 55 02 00 00 ff ff ff + page 0，
 *   固件收到 0x55 02 后同步到任务页；收到 TASK -> page 0，收到 STAT -> page 1。
 * 周期刷新只刷当前页，不切页。
 * ⚠ obj.x / y / w / h 是**只读属性**，串口改不了；sta 是字体渲染方式，不是对齐。
 * 所以控件位置、大小、对齐必须在上位机里设好（坐标见文档 4.3.1），固件只改文字。
 *
 * 注意：115200 波特率下每条指令仍是阻塞发送（约 87 us/字节），
 * 一页刷新约 0.02 秒。本 Target 只做显示联调，不驱动电机，所以可以接受；
 * 以后并进生产固件时仍要与步进脉冲错开（见 AGENTS.md 第 6 节）。
 */

/* ==================================================================== */
/* 参数区                                                                */
/* ==================================================================== */

/* 屏幕控件名。必须与 USART HMI 上位机里的 objname 完全一致（大小写敏感），
   名字不对时屏幕会回 0x1A，调试串口上能直接看到。 */
#define HMI_TASK_LINE1_OBJECT   "t0"
#define HMI_TASK_LINE2_OBJECT   "t1"
#define HMI_GRAB_OBJECT         "t2"
#define HMI_PLACE_OBJECT        "t3"

/* 页面分配：启动页在 page 2，任务码在 page 0，统计在 page 1。
   TJC 的串口指令只作用于**当前页**，所以切页必须由固件管理。 */
#define HMI_TASK_PAGE              0U
#define HMI_STATS_PAGE             1U
#define HMI_SPLASH_PAGE            2U

/* 启动页按钮的触摸事件码。屏端按钮的弹起事件里写：
     printh 55 02 00 00 ff ff ff
     page 0
   固件收到 55 02.. 后把内部页面状态切到任务页。 */
#define HMI_START_EVENT          0x02U

/* 布局必须在 USART HMI 里设好（固件改不了 x/y/w/h）：
     page 0：t0 (0, 52, 320, 64)、t1 (0, 124, 320, 64)，字库 64 px
     page 1：t2 (0, 76, 320, 40)、t3 (0, 124, 320, 40)，字库 32 px
   对齐在属性里设 xcen / ycen = 居中。 */

/* 主循环刷新周期。作用：屏比单片机后上电时能自己恢复当前页的显示；
   屏中途重启会回到第 0 页，下一次 TASK/STAT 命令会重新切页。 */
#define HMI_REFRESH_MS          5000U

/* 任务码：4 组三位数，组间用 '+'，共 15 个字符。 */
#define HMI_TASK_CODE_LEN         15U
#define HMI_TASK_LINE1_LEN         7U   /* "156+123" */
#define HMI_TASK_LINE2_LEN         8U   /* "+516+231" */

/* 命令来源串口。
   SERIAL_DEBUG（USART1）：台架联调用，同一个口也打印调试信息，省一个 USB-TTL；
   SERIAL_RPI（USART3）  ：接树莓派时改回这个。
   注意：两种接法的波特率都是 115200。 */
#define HMI_COMMAND_PORT          SERIAL_RPI

/* 命令缓冲：最长一条 "TASK 156+123+516+231" 是 20 字节。 */
#define COMMAND_CAPACITY          40U

/* 统计次数只做显示，限制在三位数以内，防止屏上文字超宽。 */
#define STAT_COUNT_MAX           999U

#if HMI_TASK_CODE_LEN != (HMI_TASK_LINE1_LEN + HMI_TASK_LINE2_LEN)
#error HMI task code split lengths must add up to the task code length
#endif
#if HMI_TASK_PAGE == HMI_STATS_PAGE
#error HMI task and statistics must be on different pages
#endif
#if HMI_SPLASH_PAGE == HMI_TASK_PAGE || HMI_SPLASH_PAGE == HMI_STATS_PAGE
#error HMI splash page must be different from task and statistics pages
#endif
#if HMI_REFRESH_MS < 1000U
#error HMI_REFRESH_MS is too small for a page refresh
#endif

/* ==================================================================== */
/* 内部状态                                                              */
/* ==================================================================== */

static char     task_code[HMI_TASK_CODE_LEN + 1U];
static uint8_t  task_valid;
static uint8_t  active_page;      /* 屏当前停在哪一页，0xFF = 未知 */
static uint16_t grab_ok_count;
static uint16_t place_ok_count;

static char     command_buffer[COMMAND_CAPACITY];
static uint8_t  command_length;
static uint8_t  command_overflow;

static uint32_t next_refresh_ms;

/* ==================================================================== */
/* 串口输出                                                              */
/* ==================================================================== */

static void send_debug(const char *message)
{
  (void)Serial_Write(SERIAL_DEBUG, (const uint8_t *)message,
                     (uint16_t)strlen(message), 100U);
}

/* 回给命令来源（USART1 或 USART3，见 HMI_COMMAND_PORT）。
   全部是 ASCII 单行，以 \r\n 结尾，与脚本按行读取的约定一致。 */
static void send_reply(const char *message)
{
  (void)Serial_Write(HMI_COMMAND_PORT, (const uint8_t *)message,
                     (uint16_t)strlen(message), 100U);
}

/* ==================================================================== */
/* 任务码校验与显示                                                       */
/* ==================================================================== */

/* 任务码只有两种合法字符：
     - 下标 3、7、11（第 4、8、12 个字符）必须是 '+'
     - 其余 12 位必须是 '1'..'6'（颜色 1..6，圆环号 1..6）
   长度必须正好 15 个字符。返回 1 表示合法。 */
static uint8_t task_code_valid(const char *code)
{
  uint8_t i;

  if (code == 0) return 0U;
  for (i = 0U; i < HMI_TASK_CODE_LEN; ++i) {
    if (i == 3U || i == 7U || i == 11U) {
      if (code[i] != '+') return 0U;
    } else if (code[i] < '1' || code[i] > '6') {
      return 0U;
    }
  }
  return (uint8_t)(code[HMI_TASK_CODE_LEN] == '\0');
}

/* 把任务码拆成两行推给屏。没有有效任务时显示 NO TASK，方便区分
   "还没收到任务" 和 "收到但格式不对"（后者不会覆盖已有画面）。 */
static void show_task_lines(void)
{
  char line1[HMI_TASK_LINE1_LEN + 1U];
  char line2[HMI_TASK_LINE2_LEN + 1U];

  if (!task_valid) {
    Hmi_SetText(HMI_TASK_LINE1_OBJECT, "NO TASK");
    Hmi_SetText(HMI_TASK_LINE2_OBJECT, "");
    return;
  }

  memcpy(line1, task_code, HMI_TASK_LINE1_LEN);
  line1[HMI_TASK_LINE1_LEN] = '\0';
  memcpy(line2, task_code + HMI_TASK_LINE1_LEN, HMI_TASK_LINE2_LEN);
  line2[HMI_TASK_LINE2_LEN] = '\0';

  Hmi_SetText(HMI_TASK_LINE1_OBJECT, line1);
  Hmi_SetText(HMI_TASK_LINE2_OBJECT, line2);
}

/* 抓取/放置正确次数。比赛结束时裁判要看这两个数。 */
static void show_statistics(void)
{
  char line[24];

  (void)snprintf(line, sizeof(line), "GRAB OK %u", (unsigned)grab_ok_count);
  Hmi_SetText(HMI_GRAB_OBJECT, line);
  (void)snprintf(line, sizeof(line), "PLACE OK %u", (unsigned)place_ok_count);
  Hmi_SetText(HMI_PLACE_OBJECT, line);
}

/* 注意：TJC 的 obj.x / y / w / h 是只读属性，串口写会回 0x1C（参数无效）；
   对齐要改 xcen / ycen，sta 是字体渲染方式（单色/灰度）。所以位置和大小
   必须在上位机里设好，固件只负责切页和改文字。 */

/* 启动页：静态页面，只有一个 START 按钮，固件只负责切过来。
   按钮的弹起事件在屏工程里写 printh 55 02 00 00 ff ff ff + page 0，
   屏自己先切到任务页，再通知固件同步状态。 */
static void show_splash_page(void)
{
  Hmi_SetPage(HMI_SPLASH_PAGE);
  active_page = HMI_SPLASH_PAGE;
}

/* 切到任务页并刷新文字。force=1 时即使记录为同一页也重发 page，
   用于命令触发的自愈（屏中途重启会自己回到第 0 页，固件并不知道）。 */
static void show_task_page(uint8_t force)
{
  if (force || active_page != HMI_TASK_PAGE) {
    Hmi_SetPage(HMI_TASK_PAGE);
    active_page = HMI_TASK_PAGE;
  }
  show_task_lines();
}

static void show_stats_page(uint8_t force)
{
  if (force || active_page != HMI_STATS_PAGE) {
    Hmi_SetPage(HMI_STATS_PAGE);
    active_page = HMI_STATS_PAGE;
  }
  show_statistics();
}

/* 周期刷新只刷当前页，**不切页**：每 5 秒切一次会闪，
   而且任务页在比赛运行期间必须一直可见。 */
static void refresh_active_page(void)
{
  if (active_page == HMI_SPLASH_PAGE) {
    return;                    /* 启动页是静态的，没有需要维护的控件 */
  }
  if (active_page == HMI_STATS_PAGE) {
    show_statistics();
  } else {
    show_task_lines();
  }
}

/* 屏上报自定义帧 0x55 p1 p2 p3（见 board_hmi.h）。
   启动按钮用 p1=0x02 通知固件：屏已经自己切到 page 0 了，
   这里把状态同步过去并重刷任务文字。 */
void Hmi_OnAppFrame(uint8_t param1, uint8_t param2, uint8_t param3)
{
  (void)param2;
  (void)param3;

  if (param1 == HMI_START_EVENT) {
    send_debug("START button pressed\r\n");
    show_task_page(1U);
  }
}

/* ==================================================================== */
/* 命令串口解析                                                           */
/* ==================================================================== */

static char *trim_spaces(char *text)
{
  char *end;

  while (*text == ' ' || *text == '\t') ++text;
  end = text + strlen(text);
  while (end > text && (end[-1] == ' ' || end[-1] == '\t')) --end;
  *end = '\0';
  return text;
}

/* 解析一个十进制无符号数（最多 5 位），返回 1 表示成功并前移 cursor。 */
static uint8_t parse_uint16(const char **cursor, uint16_t *value_out)
{
  const char *text = *cursor;
  uint32_t value = 0U;
  uint8_t digits = 0U;

  while (*text == ' ') ++text;
  while (*text >= '0' && *text <= '9') {
    if (digits >= 5U) return 0U;
    value = value * 10U + (uint32_t)(*text - '0');
    ++text;
    ++digits;
  }
  if (digits == 0U || value > 65535U) return 0U;

  *value_out = (uint16_t)value;
  *cursor = text;
  return 1U;
}

static void handle_task(char *code)
{
  char line[80];

  code = trim_spaces(code);
  if (!task_code_valid(code)) {
    (void)snprintf(line, sizeof(line), "TASK REJECTED: %s\r\n", code);
    send_debug(line);
    send_reply("ERR TASK FORMAT\r\n");
    return;
  }

  memcpy(task_code, code, HMI_TASK_CODE_LEN + 1U);
  task_valid = 1U;
  show_task_page(1U);

  (void)snprintf(line, sizeof(line), "TASK SET %s\r\n", task_code);
  send_debug(line);
  send_reply("OK TASK\r\n");
}

static void handle_stat(const char *parameters)
{
  const char *cursor = parameters;
  uint16_t grab_counts;
  uint16_t place_counts;
  char line[64];

  if (!parse_uint16(&cursor, &grab_counts) ||
      !parse_uint16(&cursor, &place_counts)) {
    send_reply("ERR STAT FORMAT\r\n");
    return;
  }
  while (*cursor == ' ') ++cursor;
  if (*cursor != '\0') {
    send_reply("ERR STAT FORMAT\r\n");
    return;
  }
  if (grab_counts > STAT_COUNT_MAX || place_counts > STAT_COUNT_MAX) {
    send_reply("ERR STAT RANGE\r\n");
    return;
  }

  grab_ok_count = grab_counts;
  place_ok_count = place_counts;
  show_stats_page(1U);

  (void)snprintf(line, sizeof(line), "STAT grab=%u place=%u\r\n",
                 (unsigned)grab_ok_count, (unsigned)place_ok_count);
  send_debug(line);
  send_reply("OK STAT\r\n");
}

/* 一条完整命令行（不含 \r\n）。命令不区分大小写；任务码只有数字和 '+'。 */
static void handle_line(char *line)
{
  size_t i;

  for (i = 0U; line[i] != '\0'; ++i) {
    if (line[i] >= 'a' && line[i] <= 'z')
      line[i] = (char)(line[i] - 'a' + 'A');
  }
  line = trim_spaces(line);
  if (line[0] == '\0') return;

  if (strncmp(line, "TASK", 4U) == 0 &&
      (line[4] == ' ' || line[4] == '\t' || line[4] == '\0')) {
    handle_task(line + 4U);
  } else if (strncmp(line, "STAT", 4U) == 0 &&
             (line[4] == ' ' || line[4] == '\t' || line[4] == '\0')) {
    handle_stat(line + 4U);
  } else if (strcmp(line, "PING") == 0) {
    send_reply("PONG\r\n");
    send_debug("command ping\r\n");
  } else {
    send_reply("ERR UNKNOWN\r\n");
    send_debug("unknown command\r\n");
  }
}

/* 从命令串口（HMI_COMMAND_PORT）的环形缓冲取字节拼行。
   拼帧在主循环里做，中断只入队。 */
static void poll_commands(void)
{
  int received;

  while ((received = Serial_ReadByte(HMI_COMMAND_PORT)) >= 0) {
    char byte = (char)received;

    if (byte == '\r' || byte == '\n') {
      if (command_overflow) {
        send_reply("ERR LINE TOO LONG\r\n");
        send_debug("command line too long\r\n");
      } else if (command_length > 0U) {
        command_buffer[command_length] = '\0';
        handle_line(command_buffer);
      }
      command_length = 0U;
      command_overflow = 0U;
    } else if (command_length < COMMAND_CAPACITY - 1U &&
               !command_overflow) {
      command_buffer[command_length++] = byte;
    } else {
      command_overflow = 1U;   /* 丢弃到下一个换行符 */
    }
  }
}

/* ==================================================================== */
/* 对外接口                                                              */
/* ==================================================================== */

void HmiSmoke_Begin(void)
{
  char line[112];
  const char *command_port_name =
      (HMI_COMMAND_PORT == SERIAL_DEBUG) ? "USART1" : "USART3";

  Hmi_Init();

  task_code[0] = '\0';
  task_valid = 0U;
  active_page = 0xFFU;
  grab_ok_count = 0U;
  place_ok_count = 0U;
  command_length = 0U;
  command_overflow = 0U;

  (void)snprintf(line, sizeof(line),
                 "TASK HMI ready: commands on %s 115200, screen USART6 %u\r\n",
                 command_port_name, (unsigned)BOARD_HMI_BAUD);
  send_debug(line);
  send_debug("TASK HMI: send TASK 156+123+516+231 / STAT 3 5 / PING on the command port\r\n");

  /* 告诉上位机本机已就绪（接树莓派时 Pi 脚本可用它确认链路）。 */
  send_reply("READY\r\n");

  /* 开机停在启动页，等用户按 START 进入任务页。统计值等收到 STAT 时再写。 */
  show_splash_page();
  send_debug("TASK HMI: waiting for START button on page 2\r\n");
  next_refresh_ms = HAL_GetTick() + HMI_REFRESH_MS;
}

void HmiSmoke_Poll(void)
{
  /* 先收屏的字节，避免 256 字节接收缓冲被冲掉；再收命令串口。 */
  Hmi_Poll();
  poll_commands();

  if ((int32_t)(HAL_GetTick() - next_refresh_ms) < 0) return;
  next_refresh_ms += HMI_REFRESH_MS;

  /* 只刷新当前页：切页会打断任务码显示。 */
  refresh_active_page();
}
