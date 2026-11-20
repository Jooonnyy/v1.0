#ifndef BOARD_HMI_H
#define BOARD_HMI_H

/*
 * 淘晶驰 (TJC) 串口屏通讯，跑在工程已有的 HMI 串口上：
 *   USART6 / PC6 = HMI_TX / PC7 = HMI_RX，波特率见 board.h 的 BOARD_HMI_BAUD
 *   （2026-10-17 起 115200）。引脚定义在 BSP/Inc/pins/hmi_uart.h。
 *
 * 协议要点（厂商 wiki http://wiki.tjc1688.com/ ，本模块按此实现）：
 *
 *   1. 单片机 -> 屏幕：ASCII 文本指令，**必须以 0xFF 0xFF 0xFF 结尾**。
 *        n0.val=123<FF FF FF>
 *        t0.txt="ABC"<FF FF FF>
 *        page 1<FF FF FF>
 *        click b0,1<FF FF FF>
 *
 *   2. 屏幕 -> 单片机：屏幕在触摸/事件里用 printh 自己发，格式由屏幕工程决定。
 *      常见两种：
 *        - 自定义帧：0x55 p1 p2 p3 <FF FF FF>（和参考工程
 *          "stm32与串口屏双向通讯" 的上位机约定一致）
 *        - 厂商触摸事件：0x65 page comp event <FF FF FF>
 *
 *   3. 屏幕每执行一条指令还会回一个状态码 <code><FF FF FF>，
 *      例如 0x01 = 成功，0x1A = 变量名无效，0x1C = 参数无效，0x1F = IO 操作失败。
 *      本模块自动收集：不会成功的码会转发到调试串口，避免刷屏。
 *
 * 实现说明：直接复用 board_comm.c 里 USART6 的中断接收环形缓冲
 * （Serial_ReadByte(SERIAL_HMI)），所以**不需要**再开一份接收缓冲，
 * 也不使用 HAL_UART_Receive_IT / HAL_UART_RxCpltCallback —— 这是和参考工程
 * 最大的区别，参考工程那套是给 F103 + HAL 阻塞式串口写的。
 */

#include <stdint.h>

/* ==================================================================== */
/* 屏幕 -> 单片机的帧格式                                                */
/* ==================================================================== */

#define HMI_APP_HEADER        0x55U  /* 自定义帧头（参考工程约定） */
#define HMI_APP_FRAME_LEN        4U  /* 55 p1 p2 p3 */
#define HMI_TOUCH_HEADER      0x65U  /* 厂商触摸事件帧头 */
#define HMI_TOUCH_FRAME_LEN      4U  /* 65 page comp event */

/* 一帧最多收多少字节（不含结尾的 FF FF FF）。自定义帧只有 4 字节，
   留 64 是为了以后接 get 之类的文本回读。 */
#define HMI_MAX_FRAME_LEN       64U

/* 发送缓冲：一条指令最多多少字节（不含结尾的 FF FF FF）。 */
#define HMI_TX_BUFFER_SIZE      96U

/* 串口屏一帧发不完时的等待上限。115200 波特率下每字节约 87 us，
   96+3 字节最坏约 8.6 ms，200 ms 留足了余量（换回低速波特率也不用改）。 */
#define HMI_TX_TIMEOUT_MS      200U

/* ==================================================================== */
/* 1. 初始化与轮询                                                        */
/* ==================================================================== */

/* 清空收发状态。必须在 Board_Init() 之后调用（HMI 串口已经配好）。 */
void Hmi_Init(void);

/* 主循环里反复调用：把 HMI 串口收到的字节拼成帧并分发。
   不需要在中断里调用 —— 收字节由 USART6 中断完成。 */
void Hmi_Poll(void);

/* ==================================================================== */
/* 2. 单片机 -> 屏幕                                                      */
/* ==================================================================== */

/* 原样发送一条指令，自动补 FF FF FF。command 不带结尾符。 */
void Hmi_SendRaw(const char *command);

/* t0.txt="text"；text 里不能出现双引号。 */
void Hmi_SetText(const char *object, const char *text);

/* n0.val=123；整数。带小数位的 x0 用屏幕的 vscope 决定小数点，
   所以这里传的是**放大后的整数**（vscope=2 时 1234 显示 12.34）。 */
void Hmi_SetValue(const char *object, int32_t value);

/* click b0,1 按下 / click b0,0 弹起。 */
void Hmi_Click(const char *object, uint8_t pressed);

/* vis b0,1 显示 / vis b0,0 隐藏。 */
void Hmi_SetVisible(const char *object, uint8_t visible);

/* page 1 切页。 */
void Hmi_SetPage(uint8_t page);

/* 注意：TJC 的 obj.x / obj.y / obj.w / obj.h 是**只读属性**，串口写入会回
   0x1C（参数无效）；文本对齐要用 xcen / ycen，而 sta 是字体渲染方式
   （单色/灰度），不是对齐。因此本模块不提供修改布局/对齐的接口，
   控件的位置、大小、对齐必须在 USART HMI 上位机里设定。详见 Docs/串口屏通讯.md。 */

/* 切换文本控件使用的字库编号。
   ⚠ **字号（字高）不能由单片机设置。** 字库是在 TJC 上位机（USART HMI）里
   生成并烧进屏幕工程的，STM32 只能用 font 属性在**屏幕工程里已经存在的**
   字库之间切换。font_id 不存在时屏幕会回 0x1A 之类的错误码，
   调试串口上就能看到。详见 Docs/串口屏通讯.md。 */
void Hmi_SetFont(const char *object, uint16_t font_id);

/* 格式化后发送，自动补 FF FF FF。超过缓冲会被截断并计入 Hmi_TxTruncated()。 */
void Hmi_Printf(const char *format, ...);

/* ==================================================================== */
/* 3. 诊断                                                                */
/* ==================================================================== */

uint32_t Hmi_Frames(void);              /* 收到的完整帧数（不含空帧） */
uint32_t Hmi_AppFrames(void);           /* 其中的 0x55 自定义帧数 */
uint32_t Hmi_TouchFrames(void);         /* 其中的 0x65 触摸事件数 */
uint32_t Hmi_BadFrames(void);           /* 长度/帧头不对或超长的帧数 */
uint32_t Hmi_ReturnCodes(void);         /* 收到的屏幕返回码个数 */
uint32_t Hmi_TxBytes(void);             /* 发给屏幕的字节数 */
uint32_t Hmi_TxTruncated(void);         /* 因为缓冲不够被截断的次数 */
uint8_t  Hmi_LastReturnCode(void);      /* 最近一次返回码 */
const char *Hmi_ReturnCodeText(uint8_t code);  /* 返回码文字说明 */

/* ==================================================================== */
/* 4. 应用钩子：在 Tests 或整车主程序里实现，覆盖这里的空实现             */
/* ==================================================================== */

/* 收到 0x55 p1 p2 p3 自定义帧。 */
void Hmi_OnAppFrame(uint8_t param1, uint8_t param2, uint8_t param3);

/* 收到 0x65 page comp event 触摸事件，event 通常是 1=按下 0=弹起。 */
void Hmi_OnTouch(uint8_t page, uint8_t component, uint8_t event);

#endif
