# 引脚与电气状态

本页分两部分：

- **新底板 V2（2026-11 打样，当前唯一在用的板）** —— 引脚事实来源是
  `BSP/Inc/pins/stepper6.h` 与 `Docs/交接文档_新底板V2.md` §2（后者来自实际网表）。
- **老板（2026-09）接线** —— 只作历史对照。两张表有大量同名引脚含义不同，
  **冲突时以 V2 表为准**；老板的引脚头（`pins/tb6612_*`、`pins/drv8825_*`、`pins/encoder_*`、
  `pins/limit_*`、`pins/home.h` 等）仍留在 `BSP/Inc/pins/`，但**不能在 V2 板上用**。

> 老板表的依据：2026-09-19 保存的 EasyEDA 原理图快照
> `C:\Users\GSH\Desktop\0\EXE pro\_epro2_review\state3.txt`，并与工程内引脚头文件交叉核对。
> 旧表格 `原理图接线对照_建议版.xlsx` 里的 PA8/PA9/PA10 PWM 方案已废弃，只能作为历史建议。

---

## 1. 新底板 V2 as-built（当前）

### 1.1 步进六轴（张大头闭环伺服，STEP/DIR）

| 轴 | 网络名 | STEP | 定时器/通道 | DIR | 装车位（2026-11-03 台架） |
|---|---|---|---|---|---|
| A | STEP1 | **PE9** | TIM1_CH1 | **PE7** | 前左 |
| B | STEP2 | **PE6** | TIM9_CH2 | **PE8** | 前右 |
| C | STEP3 | **PA0** | TIM2_CH1 | **PE10** | 后左 |
| D | STEP4 | **PB0** | TIM3_CH3 | **PE12** | 后右 |
| 旋转 | STEP_turn | **PD12** | TIM4_CH1 | **PE13** | — |
| 升降 | STEP_up_down | **PA6** | TIM13_CH1 | **PE14** | — |

- **六个轴各占一个独立定时器**：同一定时器的所有通道共用 ARR，只能输出一种频率，
  而麦轮转向/横移时四轮速度各不相同。六个 DIR 全在 GPIOE，可一次 `GPIOE->ODR` 写完。
- 方向：`DIR` 高电平时 B/D 推车向前、A/C 向后（左右镜像安装的正常现象）。
  A/C 的取反在 `board_step6.c` 的 `STEP6_DIR_INVERT_*` 表里；旋转/升降的取反仍是 0，
  **还未单独 `J` 复核**。改方向改表，不要动接线。
- PB4（原方案里的 M4 STEP）**实际改成了 PB0**：PB4 是 JTAG 的 NJTRST，
  复位带上拉会让 D 轴蹭一步。

### 1.2 使能（nEN，低有效）

| 网络 | 引脚 | 管谁 |
|---|---|---|
| EN | **PD3** | A/B/C/D 四路并在一起，不能单独禁某一个轮子 |
| EN1 | **PC8** | 旋转 |
| EN2 | **PC9** | 升降 |

2026-11-09 起固件把**三根都配成推挽输出**，上电默认失能（高），
`E 0` / `E 1` 一次开关三根（`board_step6.c` 的 `en_pins_write()`）。
在此之前 PC8/PC9 完全没配，旋转/升降能不能动只取决于板上 10 k 上拉。

### 1.3 舵机（50 Hz）

| 网络 | 引脚 | 定时器 | 接谁 | 固件 |
|---|---|---|---|---|
| SERVO_J_PWM | **PB14** | TIM12_CH1 | 夹爪 / 机械臂（U5） | `pins/servo_u5.h` |
| SERVO_X_PWM | **PB15** | TIM12_CH2 | 载物台（U6） | `pins/servo_u6.h` |

- 两路共用 TIM12：同一个 PSC/ARR，天然同频。固件按 1 MHz 计数、ARR 19999 → 50 Hz；
  TIM12 挂 APB1，预分频按 `PCLK1×2` 实算（84 MHz → PSC=83）。
- **信号线上没有串阻、没有 TVS**（板子事实）。老板的 PE5 就是被舵机信号这一路打坏的
  （输出驱动失效、引脚恒为浮空 1.68 V），同样的失效模式还可能再来一次。
- 历史：老板上舵机是 PE5/TIM9_CH1，PE5 坏了以后飞线到 PA7/TIM14_CH1。
  V2 板把 PA7 还给 SPI1、把 PE6 改作 B 轴 STEP，所以那两套（含 `U5_SERVO_USE_PA7` 开关）
  已删除，**不要再加回来**。

### 1.4 串口

| 网络 | 引脚 | 外设 | 用途 |
|---|---|---|---|
| — | PA9 / PA10 | USART1 | 调试（核心板 USB），115200 |
| hw_TX / hw_RX | **PA2 / PA3** | USART2 | HWT101 陀螺（`SERIAL_IMU`，驱动 `board_imu.c`），**不是 5V 容忍** |
| RPI_RX / RPI_TX | PC10 / PC11 | USART3 | 树莓派 |
| HMI_TX / HMI_RX | PC6 / PC7 | USART6 | 串口屏 TJC |
| SPARE_TX / SPARE_RX | **PC12 / PD2** | UART5 | 蓝牙 HC-05（`SERIAL_BT`） |

蓝牙座是 SPARE 4P：1=PC12(TX)、2=PD2(RX)、3=GND、4=5V；接 HC-05 要交叉
（模块 TXD→PD2、RXD→PC12）。**PC12 在老板表里是"原点"输入，V2 上是蓝牙 TX**。

### 1.5 其它

| 功能 | 引脚 | 备注 |
|---|---|---|
| 按键 | **PB3** | 10 k 上拉到 3V3_IO + 100 nF 到地 |
| 板载 LED | **PC0** | 低电平点亮；`VehicleApp_Begin()` 起每 500 ms 翻转当心跳 |
| 悬空网络 | PC2 / PC3 / PD13 | 原理图上只有单端，别往上面接东西 |
| 限位建议位 | PD6 / PD7 / PD8 / PD9 | 空着，可做限位/原点（板子目前没画） |
| ADC 建议位 | PA4 / PA5 | ADC1_IN4/IN5，与 SPI1 的 NSS/SCK 二选一 |

空闲引脚清单见 `Docs/交接文档_新底板V2.md` §2.7。V2 板上**没有** I2C 器件位、
没有限位、没有编码器、没有 nFAULT（六路故障没接）。

---

## 2. 老板（2026-09）接线 —— 历史对照

⚠ 下表里 **PE9 / PE13 / PE14 在 V2 上是 A 轴 STEP 和旋转/升降 DIR；PA0、PD12 是 C/旋转的 STEP；
PC8、PC9 是旋转/升降 nEN；PC12 是蓝牙 TX；PD3 是四轮 nEN。** 所以 `Board_Init()`
在这块板上会把 STEP 引脚改成编码器/PWM —— 生产入口只允许 `Board_InitStep6()`。

| 功能 | MCU 引脚 | 外设或接口 | 说明 |
|---|---|---|---|
| AT8236 U1（A/B） | PE9 PWM1、PE11 PWM2；PE7 DIR-A、PE10 DIR-B | TIM1 CH1/2、GPIO | 2026-10 换 AT8236 双路模块：每通道 PWM+DIR；PE8/PE12（原 AIN2/BIN2）不再接模块 |
| AT8236 U3（C/D） | PE13 PWM3、PE14 PWM4；PB11 DIR-C、PB13 DIR-D | TIM1 CH3/4、GPIO | 同上；PB12/PB14 不再接模块 |
| 驱动待机 | PD2、PD3 | GPIO，默认低 | AT8236 没有 STBY/nSLEEP：PD2/PD3 不接模块，`Motor_Standby()` 为空实现 |
| DRV8825 U1 | PD0 STEP、PD1 DIR、PC8 nEN、PD14 nFAULT | GPIO | 快照中 nEN/nFAULT 悬空 |
| DRV8825 U4 | PD4 STEP、PD5 DIR、PC9 nEN、PD15 nFAULT | GPIO | 快照中 nEN/nFAULT 悬空 |
| ~~ULN2003 M2~~ | PE1～PE4 | ~~GPIO，默认低~~ | 网络存在但无驱动；2026-10-27 `pins/uln2003_m2.h` 与 `ULN2003_SetMask()` 已删除，PE1～PE4 上电浮空 |
| 舵机 U5（机械臂） | **PA7**（默认）/ PE5 | TIM14 CH1 / TIM9 CH1 | 2026-10-29 定案为可切换；**V2 板已改为 PB14/TIM12_CH1，开关已删除** |
| 舵机 U6（载物台） | PE6 | TIM9 CH2，50 Hz，初始不输出 | **V2 板已改为 PB15/TIM12_CH2** |
| 板载状态 LED | PC0 | GPIO 推挽输出，低电平点亮 | 核心板自带；V2 上仍是心跳灯 |
| 编码器 A | PA0/PA1 | TIM2 编码器模式 | V2 无编码器 |
| 编码器 B | PB4/PB5 | TIM3 编码器模式 | V2 无编码器 |
| 编码器 C | PD12/PD13 | TIM4 编码器模式 | V2 无编码器 |
| 编码器 D | PD8/PD9 | 双边沿 EXTI、正交表 | V2 无编码器 |
| MPU6050 | PB6/PB7 SCL/SDA、PC3 INT | 软件 I2C、EXTI | V2 板上没画 |
| TOF400C | PB6/PB7 SCL/SDA、PC2 INT | 软件 I2C、EXTI | V2 板上没画 |
| 调试串口 | PA9/PA10 | USART1，115200 | V2 不变 |
| 树莓派串口 | PC10/PC11 | USART3，115200 | V2 不变 |
| 串口屏 | PC6/PC7 | USART6，115200，TJC 协议 | V2 不变 |
| 按键 | PB3 | 上拉轮询输入 | V2 不变 |
| 上/下限位 | PD6/PD7 | 上拉 EXTI，低有效 | V2 板没画 |
| 原点 | PC12 | 上拉 EXTI，低有效 | V2 上是蓝牙 TX |
| 蓝牙串口 | PA2/PA3（USART2）→ 后改 PC12/PD2（UART5） | AF7 / AF8 | 现在是 UART5 |
| 保留功能 | PA11/PA12 USB、PA13/PA14 SWD | 不改动 | 核心板功能 |

老板上所有方向、线圈和 STEP 输出上电初始为低；DRV8825 nEN 初始为高；舵机 PWM 只有显式调用后才启动。
原理图快照未发现主板侧 I2C 上拉电阻，需确认传感器模块是否自带上拉。

---

## 验证状态

- **编译**：2026-11-09，`Vehicle_Integrated` 全量重建（`UV4.exe -r`）**0 Error / 3 Warning**，
  3 条均为 ST HAL `stm32f4xx_hal_flash_ex.c` 的 `unused parameter 'Banks'`；
  `Program Size: Code=17650 RO-data=1882 RW-data=1364 ZI-data=2676`。
  当日新改了舵机（PB14/PB15 + TIM12）与旋转/升降 nEN（PC8/PC9）。
- **实板**：2026-11-03 已确认四轴步进点动与方向（A/C 取反后 `J` 正数 = 四轮推车向前），
  引脚 PE9/PE6/PA0/PB0/PE7/PE8/PE10/PE12/PD3 由此算实板验证过。
  **旋转/升降的 STEP/DIR、PC8/PC9 使能、PB14/PB15 舵机脉冲、以及串口五路中的
  USART2 陀螺与 UART5 蓝牙，都还没有实板验证。** 舵机是本次新改的引脚，第一次上板
  先用示波器或 `Servo_SetPulseUs(1500)` 量 PB14/PB15 的 50 Hz 方波，再插舵机。
