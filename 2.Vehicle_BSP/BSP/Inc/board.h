#ifndef BOARD_H
#define BOARD_H
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Board_Init() 完成后所有运动输出保持关闭，不会自动动作。 */
HAL_StatusTypeDef Board_Init(void);
void Board_DelayUs(uint32_t us);
/* 应用层通过板级包装取时基和保护短小快照，不直接调 HAL 或寄存器。 */
uint32_t Board_NowMs(void);
uint32_t Board_EnterCritical(void);
void Board_ExitCritical(uint32_t saved_mask);

/* 板载状态 LED（PC0，低电平点亮；引脚事实见 pins/led_status.h）。
   必须显式调用 Led_StatusInit()：生产固件不调用它，PC0 就保持复位后的默认状态，
   所以这个功能对生产固件和别的 Target 都是零影响。
   闪烁/心跳节奏由调用方自己用 Board_NowMs() 控制（例如 Tests/Src/rig_smoke.c）。 */
void Led_StatusInit(void);
void Led_StatusSet(uint8_t on);
void Led_StatusToggle(void);

typedef enum { MOTOR_A=0, MOTOR_B=1, MOTOR_C=2, MOTOR_D=3 } MotorId;
HAL_StatusTypeDef Motor_Set(MotorId id, int16_t permille);
void Motor_StopAll(void);
/* 主动刹车（动态制动）：IN1=1、IN2=1 把电机短接，比 Motor_Set(0) 的滑行停得快。
 * 只在低速短时间用；减速时电流会灌回电源，高速急刹容易让麦轮失去抓地。 */
HAL_StatusTypeDef Motor_Brake(MotorId id);
void Motor_BrakeAll(void);
/* AT8236 双路模块没有 STBY/nSLEEP 引脚；本函数现在是空实现，仅为兼容旧调用点。 */
void Motor_Standby(uint8_t driver, uint8_t awake);

typedef enum { STEPPER_1=0, STEPPER_2=1 } StepperId;
void Stepper_Enable(StepperId id, uint8_t enable);
void Stepper_Direction(StepperId id, uint8_t forward);
void Stepper_Pulse(StepperId id); /* 阻塞输出高、低电平各至少 2 us；调用方负责调度。 */
/* 同上，但高/低电平时长由调用方指定，用来调脉冲频率（单位 us）。
   实测脉宽用 DWT 周期计数器保证，是准确的。 */
void Stepper_PulseUs(StepperId id, uint16_t high_us, uint16_t low_us);
uint8_t Stepper_Fault(StepperId id);
/* 两路舵机初始化（PB14/TIM12_CH1、PB15/TIM12_CH2，50 Hz）。只配引脚和定时器，不启动通道。
   Board_Init() 与 Board_InitStep6() 都会调它：新底板生产入口也要能单独初始化舵机。 */
HAL_StatusTypeDef Servo_Init(void);
HAL_StatusTypeDef Servo_SetPulseUs(uint16_t pulse_us); /* 接受 500..2500 us，并启动 PWM。 */
void Servo_Disable(void);
/* 第二路舵机（U6 / PB15 / TIM12_CH2，载物台）。单舵机 API 保持不动，避免动已验证代码。 */
HAL_StatusTypeDef Servo2_SetPulseUs(uint16_t pulse_us);
void Servo2_Disable(void);

typedef enum { ENCODER_A=0, ENCODER_B=1, ENCODER_C=2, ENCODER_D=3 } EncoderId;
/* 累计计数，无副作用：任何上下文、调用多少次，返回值和状态都不变。
   计数只在 board_encoder.c 的 tick 里刷新（见 board_encoder.h）。 */
int32_t Encoder_ReadCounts(EncoderId id);
void Encoder_Zero(EncoderId id);

/* 顺序必须与 board_comm.c 的 ports[] 一一对应。
   新底板 V2：SERIAL_BT 是 UART5（PC12/PD2，HC-05），SERIAL_IMU 是 USART2（PA2/PA3，HWT101）。
   SERIAL_IMU 追加在末尾，前面的编号不变，已经按编号写死的数组不受影响。 */
typedef enum { SERIAL_DEBUG=0, SERIAL_RPI=1, SERIAL_HMI=2, SERIAL_BT=3, SERIAL_IMU=4 } SerialPort;
#define BOARD_DEBUG_BAUD 115200U
#define BOARD_RPI_BAUD 115200U
#define BOARD_HMI_BAUD 115200U   /* 串口屏：屏幕工程里的 bauds 必须与这里一致。 */
/* HC-05 默认透传波特率是 9600；这里跟 USART1 对齐用 115200，便于无线调参。
   改这里必须同时改模块的 AT+UART（见 Docs/HC05无线串口.md）。 */
#define BOARD_BT_BAUD 115200U
/* HWT101 陀螺出厂 100 Hz / 115200（维特上位机可改，改了两边一起改）。 */
#define BOARD_IMU_BAUD 115200U
int Serial_ReadByte(SerialPort port); /* 返回 -1 表示接收缓冲为空。 */
HAL_StatusTypeDef Serial_Write(SerialPort port, const uint8_t *data, uint16_t len, uint32_t timeout_ms);
uint32_t Serial_Dropped(SerialPort port);

/* 使用 7 位 I2C 地址；寄存器传输支持 8 位和 16 位寄存器地址。 */
HAL_StatusTypeDef Board_I2C_Read(uint8_t addr7, uint16_t reg, uint8_t reg_len, uint8_t *data, uint16_t len);
HAL_StatusTypeDef Board_I2C_Write(uint8_t addr7, uint16_t reg, uint8_t reg_len, const uint8_t *data, uint16_t len);
/* 诊断用：分别在"输入+上拉"和"开漏输出"下读 I2C 两条线的电平（正常都为 1）。
   两组结果的差异能区分"内部上拉在开漏模式下不生效"和"外部有东西摁住总线"。 */
void Board_I2C_LineLevels(uint8_t *scl_in_high, uint8_t *sda_in_high,
                          uint8_t *scl_od_high, uint8_t *sda_od_high);
/* 诊断用：扫描 7 位地址 0x08..0x77，返回应答个数，应答地址写进 ack_list。 */
uint8_t Board_I2C_Scan(uint8_t *ack_list, uint8_t capacity_counts);

typedef enum { INPUT_KEY=0, INPUT_LIMIT_TOP, INPUT_LIMIT_BOTTOM, INPUT_HOME,
               INPUT_MPU_IRQ, INPUT_TOF_IRQ } InputId;
uint8_t Input_Active(InputId id);
uint32_t Input_EventCount(InputId id); /* KEY 使用轮询，没有事件计数。 */

#endif
