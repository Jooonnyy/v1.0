# 工程结构与编码规范（Vehicle BSP / STM32F407VET6�?
**这是规范性文件�?* 人工�?AI 的所有改动都必须遵守；GPT / Codex 会自动读取本文件�?本文件没写到的地方，**保持与相邻已有代码一�?*，不要自创新风格�?本文件与任何其它文档冲突时，以本文件为准�?
�?11 节列�?*当前尚未达标、需要修正的偏差**，整合代码时按那里的优先级执行�?
---

## 1. 分层与依赖方向（硬性）

依赖只能单向流动，箭头表�?可以 include"�?
```
Tests ──�?App ──�?BSP ──�?Drivers(HAL/CMSIS)
Tests ──�?BSP ──�?Core
App   ──�?Core
```

- **禁止反向依赖**：`BSP/` 不许 include `App/` �?`Tests/`；`App/` 不许 include `Tests/`�?- **`Tests/` 是叶�?*：只有它能包含别人，任何生产代码都不许包�?`Tests/`�?- `Drivers/` 永远只读，任何自有代码都不许放进去�?
| 目录 | 放什�?| 不放什�?|
|---|---|---|
| `Core/` | 时钟、启动、中断向量、`main.c` 调度 | 业务逻辑、器件驱�?|
| `Drivers/` | ST HAL、CMSIS（厂商代码） | 任何自有代码 |
| `BSP/` | 引脚宏、GPIO/外设初始化�?*板级器件驱动**（DRV8825 / HMI �?/ 编码器…） | 整车策略、命令行、动作编�?|
| `App/` | 整车状态机、串口命令行、策略、时基调�?| 直接写寄存器或调 HAL |
| `Tests/` | 台架验证入口（上电跑一次、看现象�?| **可复用模�?* |

### Core 的改动规�?
**本工程不再用 CubeMX 重新生成**（`reference_407VE.ioc` 已废弃，见第 11 节）�?`Core/` 下只允许�?`USER CODE BEGIN/END` 标记区内手改；`Core/Src/main.c` 里的
`#if defined(...)` 测试分发链是**所有固件入�?*（见�?8 节）�?
### 引脚配置的单一事实来源

| 内容 | 唯一位置 |
|---|---|
| 端口/引脚宏（`XX_PORT` / `XX_PIN` / 复用号） | `BSP/Inc/pins/<器件>.h` |
| GPIO 初始化、上拉、EXTI、NVIC | `BSP/Src/board_gpio.c` |
| 文档化的引脚表与电气状�?| `Docs/PINMAP.md` |

**不要**在别处再写一遍引脚号。改引脚必须同时改这三处的相关项�?
> 需要第 4 个层（例如纯算法库）时，**先改本文�?*再加目录，不要临时新建�?>
> `Tools/` �?**PC 侧辅助工�?*（HTML / Python 脚本等），不属于固件分层�?> 也不进任�?Keil Target，不受上面的依赖规则约束。目前有
> `Tools/serial_scope.html`：零配置的串口波形上位机（Web Serial）�?
## 2. 新代码放哪里

按顺序问自己�?
1. 只是引脚宏、初始化、或直接读写寄存�?HAL�?�?**BSP**
2. �?某一个板载器�?的完整驱动（协议、换算、状态、参数）�?�?**BSP**
3. 是整车策略、命令解析、跨器件的动作编排？ �?**App**
4. 只是上电跑一次给人看的验证？ �?**Tests**
5. 以上都不�?�?先补本文件的分层，再动手�?
### BSP / App 的分界（有争议时按这条裁决）

- **单个器件自身的量纲换算和状�?*（编码器计数 �?轮子转角/里程、丝杆角�?�?高度�?  HMI 帧编解码）→ **BSP**。即使里面有数学、有状态机，也留在 BSP�?- **跨器件、带整车意图的决�?*�?看到限位就停�?�?收到上位机指令后按顺序走一套动�?�?  �?**App**�?- 一句话�?*BSP 回答"这个器件现在是什么状态、怎么驱动�?；App 回答"整车该做什�?�?*

### 两条硬判�?
- 一个文�?*会被 App 调用** �?它不属于 `Tests/`�?- 一个文�?*需�?include `App/`** �?它不属于 `BSP/`�?
## 3. 命名

| 对象 | 规则 | 例子 |
|---|---|---|
| BSP 源文�?| `board_<器件或功�?.c/.h` | `board_motion.c`、`board_hmi.c`、`board_lift.c` |
| 引脚�?| `BSP/Inc/pins/<器件>.h`，一个器件一�?| `BSP/Inc/pins/drv8825_u1.h` |
| App 源文�?| `<功能>_app.c/.h` | `vehicle_app.c` |
| 测试入口 | `<器件>_smoke.c` | `Tests/Src/hmi_smoke.c`、`Tests/Src/lift_smoke.c` |
| 配置�?| `<模块>_config.h`（见�?5 节） | `vehicle_app_config.h` |
| 类型 | 大驼�?| `LiftStatus`、`SerialPort` |
| 公开函数 | `<模块>_<动作><单位>()` | `Lift_MoveToUm()`、`Servo_SetPulseUs()` |
| 文件内静态函�?| 全小�?+ 下划线，不带模块前缀 | `hmi_write_frame()`、`div_round()` |
| �?| 全大�?+ 模块前缀 | `LIFT_STEP_PERIOD_US`、`HMI_RESEND_MS` |
| 全局/静态变量、结构体字段、函数参�?| 全小�?+ 下划�?| `motor_started_ms`、`permille` |

### 单位后缀（强制，按标识符种类决定写法�?
**任何表示物理量的标识符都必须带单位后缀�?* 不允许出�?`lead`、`period`�?`target`、`angle` 这种没有单位的名字�?
| 标识符种�?| 写法 | 例子 |
|---|---|---|
| �?| 全大�?+ `_单位` | `LIFT_SCREW_LEAD_UM`、`GRIP_OPEN_DEG` |
| 变量 / 字段 / 参数 | 全小�?+ `_单位` | `motor_started_ms`、`gripper_angle_deg`、`timeout_us` |
| 函数�?| 大驼峰，单位紧跟动作 | `Lift_MoveToUm()`、`Servo_SetPulseUs()` |

单位词表�?*只用这些**，需要新单位时先补本表）�?
| 后缀 | 含义 |
|---|---|
| `um` / `_UM` | 0.001 mm（微米） |
| `mm` / `_MM` | 毫米 |
| `mdeg` / `_MDEG` | 0.001 �?|
| `deg` / `_DEG` | �?|
| `us` / `_US` | 微秒 |
| `ms` / `_MS` | 毫秒 |
| `permille` / `_PERMILLE` | 千分比（�?000..1000�?|
| `hz` / `_HZ` | 赫兹 |
| `um_per_s` / `_UM_PER_S` | 微米每秒（速率一律用 `_PER_S` 形式，分子量级放前面�?|
| `px` / `_PX` | 像素（屏幕坐标、控件宽高） |
| `pulses` / `_PULSES` | STEP 脉冲个数 |
| `counts` / `_COUNTS` | 编码器计数、定时器计数 |
| `counts_per_s` / `_COUNTS_PER_S` | 计数每秒（轮速）。字段写 `speed_counts_per_s`；PID 统一用它，不用物理单�?|
| `kcps_per_spad` / `_KCPS_PER_SPAD` | 千计数每秒每 SPAD（VL53L1X 的信号率 / 环境光率�?|
| `spads` / `_SPADS` | SPAD 个数（VL53L1X �?ROI 尺寸�?|
| `mg` / `_MG` | �?g�? g = 1000 mg），加速度 |
| `mdps` / `_MDPS` | 0.001 °/s，角速度 |
| `mdeg_c` / `_MDEG_C` | 0.001 摄氏度，温度（与表内�?`mdeg` 角度区分�?|

**像素、脉冲、计数也是量，一样要带后缀�?* 所以控件几何参数写
`width_px` / `height_px`，脉冲数�?`pulses`、计数值写 `counts`�?
**唯一的豁�?*：直接透传 HAL 参数的内部包装函数（例如 `pin_init(..., uint32_t speed)`
里的 `speed` 就是 HAL �?`GPIO_InitTypeDef.Speed`），**沿用 HAL 的原�?*，不要造新单位�?
**不使用浮点�?* `0.001 mm` 这类量用整数放大表示（见 `Lift_*` 的换算实现）�?
## 4. 语言与编�?
- **注释一律用中文**，标识符一律用英文�?  - 含义�?*不要同一个文件里一部分注释英文、一部分中文**；也不要写中英对照的双语注释�?  - 例外：引用寄存器名、HAL 符号、错误信息原文时保留英文�?- **串口/调试输出一律用 ASCII 英文**（与注释相反）�?  串口助手默认�?GBK 解码，固件发 UTF-8 中文必定显示成乱码，排查问题时先被编码挡住；
  2026-10-02 已把 4 个台架测试（`tof_smoke.c` / `mpu6050_smoke.c` /
  `motors_all_smoke.c` / `stepper_zdt_smoke.c`）输出的中文全部改成英文�?  `⚠` `✓` `（` `）` `‰` 这类符号同样算非 ASCII�?*一律不许出现在输出字符串里**�?  一句话�?*注释用中文，打印给人看的字符串用英文�?*
- **文档�?`Docs/`，一个主题一�?`.md`，用中文写�?*
- 源文件编�?**UTF-8 �?BOM**�?*禁止 GBK** —�?参考工程是 GBK，工具链�?diff 都读不了�?- 顶层 `README.md` 只留简短摘�?+ 指向 `Docs/` 的索引，不要把细节复制到两处�?
## 5. 参数与编译期校验

1. **所有可调参数集中在文件顶部�?参数�?**，不许散落在函数体里�?2. **参数区后面紧�?`#if` / `#error` 校验**，非法值在**编译�?*就报错，不要等上板才发现�?3. **什么时候拆出独立配置头**（硬阈值，不要靠感觉）�?   **模块的可调参�?�?12 个，或该模块�?API 需要与参数分开给不同人看时**�?   �?`<模块>_config.h`，API 头只 `#include` 它。少�?12 个就放在 API 头文件顶部的参数区�?
参考实现：`App/Src/vehicle_app.c` �?7�?5 行、`BSP/Inc/board_lift.h` �?`LIFT_*` 校验块�?
## 6. 编码硬性规�?
- C99�?*不使用浮�?*。时间敏感路�?*禁止 `snprintf`/`printf`**（体积和耗时都不可控）�?- **栈只�?1 KB**（`startup_stm32f407xx.s` �?`Stack_Size EQU 0x400`）�?  **单个函数所有局部变量合�?�?192 B**（栈�?1/5，其余留给调用链和中断）�?  超过就用文件�?`static` 缓冲（参�?`board_hmi.c`）�?- **时间比较必须防回�?*�?  ```c
  if ((int32_t)(HAL_GetTick() - t0) >= 0) { ... }   // 不要�?if (HAL_GetTick() > t0)
  ```
- **阻塞�?`Serial_Write()` �?115200 下每字节�?87 us�?600 下约 1.04 ms�?  禁止在步进电机发脉冲的过程中调用** —�?会把脉冲间隔拉长、导致失步�?  发屏和发脉冲必须错开，或把发脉冲挪到定时器中断里�?
### 中断服务函数的边界（硬性）

ISR �?*只允�?*做这四件事：

1. �?写外设寄存器（数据寄存器、状态标志）�?2. 环形缓冲的入队与 head/tail 索引运算、溢出计数；
3. 置一�?`volatile` 标志或计数；
4. 清中断标志�?
**明确禁止**：字符串比较、帧�?帧尾判断�?*任何"这一帧收完了�?的判�?*�?调用 `Serial_Write` / `HAL_Delay` / `snprintf`、以及任何循环次数不确定的代码�?
拼帧和解析必须在 `Poll`（主循环）里做。现有分工就是这个模式：
`board_comm.c` �?ISR 只入队，`board_hmi.c` �?`Hmi_Poll()` 才拼帧�?
### 控制�?ISR（上�?ISR 边界的唯一例外�?
固定周期的控制环（例�?1 kHz 轮速环）允许在定时�?ISR 里做计算，但必须**同时**满足�?
1. **周期固定**，由定时器触发，不依�?`HAL_GetTick()` 的抖动；
2. **只算不发**：绝不调�?`Serial_Write` / `snprintf` / `HAL_Delay`�?3. **只写快照**：把这一拍的观测量（目标、实际、输出）写进一�?`volatile` 结构体，
   格式化与发送交给主循环�?4. **执行时间有界**：没有循环次数不确定的代码，没有 `while` 等待�?5. **输出先限幅再�?PWM**，且限幅值要明显低于 `Motor_Set` 允许�?1000�?   �?000�?= 100% 占空比，不应出现在闭环输出里）�?
串口 / EXTI �?ISR 仍然只搬字节 —�?这条例外**�?*针对控制环�?
### 编码器读�?
- 读原始计数必须是**无副作用**的原子快照（例如 `Encoder_ReadCounts(id)`）：
  谁调用都拿到同一个值，且不改变任何状态�?- **增量与速度只在唯一一处按固定周期计算**（控制环 ISR），结果放进快照�?- **禁止让中断和主循环各自调用一�?会累�?的读函数** —�?那会互相吃掉增量�?  两边读数都错，而且错得没有规律�?
### 遥测与上位机调参（VOFA+�?
- **协议�?FireWater（文本）**：每�?`v1,v2,...\n`，一行一条完整样本�?- **发送时不用浮点**：把量按定点整数放大后发（例�?0.001 mm 就发 µm 的整数）�?  单位�?VOFA+ 侧自己标注。整数不需�?`%f`，避免拖进庞大的浮点 printf 库�?  需要更高频率时再考虑 JustFloat（二进制�? 提高波特率；那时才需�?`float`�?  且必须先确认 FPU 已启用并把这条写进本文件�?- **调试文本不能和遥测抢同一个串�?*：`send_text("...")` 打出的非数字行会�?  VOFA+ 当成坏帧、波形出毛刺。要么把调试文本改到另一个串口（USART3 / USART6），
  要么在遥测开启期间关闭调试输出�?*同一�?UART 上，遥测流必须只有数字行�?*
- **发送在主循�?*，按固定节拍（`HAL_GetTick()` 时间戳）格式化后**整行一�?*发出�?  不要逐字段多�?`Serial_Write`�?- 先算带宽再定频率�?15200 下每字节�?87 µs，一�?20 字节样本最快约 570 Hz�?  9600 下每字节 1.04 ms。要更高就提�?460800 / 921600（USB-TTL 需支持）�?
### 其它

- 需要应用覆盖的钩子，用**弱符�?+ 默认空实�?*�?  ```c
  __attribute__((weak)) void Hmi_OnAppFrame(...) { }
  ```
  这样应用不覆盖也能链接通过（参�?`board_hmi.c`）�?- 模块对外只暴露必要接口，内部函数一�?`static`�?- BSP 内部跨文件接口放 `BSP/Inc/board_internal.h`，不对外暴露�?
## 7. 增删源文件的固定动作

Keil **不会**自动发现文件，`<Groups>` �?`IncludePath` 都是手工�?*�?Target** 维护的�?新增一�?`.c` 必须�?
1. 放进�?2 节判定的层目录；
2. �?*用到它的那些 Target** 的对�?`<Group>` 里加 `<File>`
   （`FileName` / `FileType`(1=C) / `FilePath`）；
   **只加到需要的 Target**，不要把测试专用文件塞进生产工程�?3. 该头文件所在目录是否已在该 Target �?`IncludePath` 里？没有就补
   （`IncludePath` �?**Target �?*配置，改的是那个 Target �?`VariousControls`）；
4. 编译�?**0 Error / 0 Warning**�?5. 2026-11-09 起没有活动台架工程。新 `.c` 只加�?`Vehicle_Integrated`。不要往 `_legacy/Bench_Tests.uvprojx` 里加�?
�?`.uvprojx` �?**LF 换行**�?XML。用脚本改时必须保持换行风格，否则整文件 diff 爆掉�?
### �?`.uvprojx` 的三条硬规矩�?026-10-01 踩过�?
1. **改之前必须先关掉 µVision（`UV4.exe`）�?* 它开着的时候会**回写** `.uvprojx`�?   本次它把 `Bench_Tests.uvprojx` �?222 KB 重写�?313 KB，并把新加的源文�?   **传播到所�?Target**、重�?IncludePath，脚本改动会被覆盖或污染�?   `.uvoptx`（Keil 本地状态，`.gitignore` 已覆盖）里也存着"文件是否参与�?Target 构建"
   的标志，行为诡异时删掉它再试�?2. **Keil 实际构建的是"所�?Target 文件的并�?，不是当�?Target �?`<Groups>`�?*
   表现�?每个台架 Target 都编译所有测试源"（加 TOF 之前就存在）�?   后果�?*从某�?Target �?`<Groups>` 里删文件对构建没有影�?*�?   只要并集里出现某�?`.c`，它需要的 `IncludePath` �?*必须在该工程的所�?Target 里都配上**�?   否则其余 Target 会因为在并集里编译它却找不到头文件而报错�?   本工程现状：`../Drivers/VL53L1X` 已加�?`Bench_Tests` 全部 **12 �?* Target�?026-10-02 �?15 个精简�?10 个，其后新增 `WHEEL`、`Motors_PowerOn` 并把 `Stepper_ZDT` 就地改名�?`RIG`，现�?12 个，见第 11 节）�?   未用到的代码靠链接期 section GC 丢弃，固件体积不受影响�?3. 脚本�?*不要�?`Write-Output` 在函数内输出进度** —�?PowerShell 会把它们并入函数返回值，
   一旦返回值被写回文件，就把提示文�?*注入�?XML �?*（本次真的发生了，已修复）�?   进度信息一律用 `Write-Host`�?
## 8. Keil 工程与固件入�?
| 工程 | 角色 | Define |
|---|---|---|
| `MDK-ARM/Vehicle_Integrated.uvprojx` | **唯一可构建工�?*�?026-11-09 起也是唯一入口�?| `VEHICLE_INTEGRATED_APP` |
| `MDK-ARM/_legacy/Bench_Tests.uvprojx` | 已归档。`HMI` / `WHEEL` / `RIG` / `CHASSIS_STEP` 不再烧录 | 不要再启�?|

- **不要再新�?`.uvprojx`，也不要把台�?Target 加回来�?* 新行为放�?`App/` 的对应模块，驱动�?`BSP/`�?- **入口只有一�?*：`Core/Src/main.c`。生产定义走 `Board_InitStep6()` + `VehicleApp_Begin()` / `VehicleApp_Poll()`�?  **测试代码不写 `main()`�?* `main.c` 里残留的 `HMI_TEST` / `WHEEL_TEST` / `RIG_TEST` / `CHASSIS_STEP_TEST` 分支只为归档工程留着，不要再往上加�?- 遗留工程�?`MDK-ARM/_legacy/`。从 `_legacy/` 打开时相对路径是错的，不要拿来编译�?
## 9. 文档�?验证状�?的诚�?
- 每个功能一�?`Docs/<主题>.md`�?*结尾必须有「验证状态」小�?*�?  �?*「编译」与「实板」分开�?*�?- **没上板就�?尚未上板验证"**。不要把"编译通过 0 �?0 警告"说成"能用"�?已验�?�?- `Docs/PINMAP.md` 是引脚的**文档化事实来�?*；改引脚必须同步它�?
## 10. 提交前检查表

- [ ] `Vehicle_Integrated` 编译 **0 Error / 0 Warning**
      （全量重�?`-r` 时只允许 ST HAL 自带�?3 �?`unused parameter 'Banks'`�?- [ ] 新增/修改的参数在参数区或 `*_config.h`，且 `#error` 覆盖非法�?- [ ] 新标识符都带单位后缀（宏大写、变量小写、函数驼峰）
- [ ] 引脚有变�?�?同步 `pins/*.h` + `board_gpio.c` + `Docs/PINMAP.md`
- [ ] 新功�?�?`Docs/` 有对应文�?+ 诚实的验证状�?- [ ] 单函数局部变量合�?�?192 B
- [ ] ISR 里没有解析、没有阻塞调�?- [ ] 没有在步进脉冲期间做阻塞串口发�?- [ ] 新增 `.c` 已加�?*需要它的那�?Target** �?`.uvprojx`

---

## 11. 待修正项

### 已核对完成（2026-10-01，逐项构建 + 静态核对通过）—�?**不要再重�?*

1. �?升降台模块搬�?`BSP/{Src,Inc}/board_lift.c|h`，`Tests/Src/lift_smoke.c` 为薄入口，原文件已删
2. �?工程收敛�?2 个：`Vehicle_Integrated` + `Bench_Tests`�?026-10-02 �?15 �?Target 精简�?10 个），旧工程移入 `_legacy/`
3. �?`.gitignore` 已覆盖全�?13 个输出目�?+ `uvoptx`/`uvguix`/`build*.log`/`_legacy`
4. �?废弃�?`reference_407VE.ioc` 已删�?5. �?新增文件均为中文注释
6. �?`README.md` �?`Docs/整合工程使用说明.md` 已重写，验证状态分「编译」「实板�?7. �?4 处缺单位后缀全部改对（`gripper_angle_deg` / `angle_deg` / `width_px`+`height_px` / `duty_counts`），旧名 0 残留
8. �?`MDK-ARM/` �?11 个旧工程的输出目录与过期 HEX 已删除（2026-10-02，原为下�?E 项）

**构建事实**�?026-10-02 已核对，当时�?10 �?Target）：`Vehicle_Integrated` 与全�?Target
全量重建�?**0 Error / 3 Warning**�? 条全部来�?ST `hal_flash_ex.c`�?1 �?HEX 已生成（HEX 数随 Target 删除相应失效）；define �?`main.c`
分发链逐一匹配；BSP 无反向依赖、App 不依�?Tests�?
**Target 精简**�?026-10-02）：`Bench_Tests` �?15 �?Target 减为 10 个。删�?`Motor_A`（A 路用途已�?`Motors_All` �?`ENCODER` 完全覆盖，且它是唯一已实板验证过的单项）�?`Stepper_28`、`Stepper_Phase`、`Stepper_Example10`、`Stepper_Example10_Slow`
�?8BYJ-48 的几种相�?速度组合实板只震动、无连续转动记录，留着只是重复同一个结论）�?同步清理：`Tests/{Src,Inc}` 8 个文件、`Core/Src/main.c` 分发链、两�?`.uvprojx`
（`Bench_Tests` �?5 �?`<Target>` 与每�?Target 里的 4 �?`<File>` 条目）�?5 个过期输出目录�?1 个旧工程输出目录；相关文档已标注「历史记录」�?被删�?4 个测试源所用的 `ULN2003_SetMask()` �?`pins/uln2003_m2.h` 当时保留�?BSP
�?026-10-02 决定：引脚初始化是板级事实；**该决定已�?2026-10-27 按用户要求推翻，见下�?*）�?
**死代码清�?*�?026-10-27）：删除孤儿�?`Tests/Src/stepper_zdt_smoke.c|h`
（`Stepper_ZDT` Target 改名 `RIG` 后遗留的无入口文件，单轴 ZDT 点动已被
`rig_smoke.c` �?LIFT/ROT 两轴命令取代），同步删除 `Bench_Tests.uvprojx` /
`.uvoptx` �?12 Target 的对�?`<File>` 条目、各 Target 输出目录的过�?`.d/.o`
和已不存在的 `Stepper_ZDT` 输出目录；删除零引用的墓碑头 `pins/adc_inputs.h`�?**2026-10-27 追加**：按用户要求删除 `ULN2003_SetMask()`（`board_motion.c` 函数 +
`board.h` 声明）与 `pins/uln2003_m2.h`（含 `board_gpio.c` �?PE1～PE4 的上电置�?初始化）——板面已�?ULN2003 用途（28BYJ-48 �?Target 早就删除，单�?ZDT 也被
RIG 取代）；副作用是 PE1～PE4 上电变为浮空（复位默认），不再被固件置低�?`Docs/PINMAP.md` / `Docs/整合工程使用说明.md` / `Docs/上板验证顺序.md` 已同步�?`Motor_Standby()` 空实现维持保留（AT8236 �?STBY，是调用语义占位）�?未重新编译验证，重建由人工完成（预期结论不变；同日另�?Target 精简，见下条）�?
**台架 Target 精简**�?026-10-27）：`Bench_Tests` �?12 �?Target 减为 **3 �?*
（`HMI` / `WHEEL` / `RIG`，对应屏幕、轮子、机械臂/载物�?电机联调，均已实板验证过核心功能）�?删除 9 个单�?Target 及其测试源：`Motor_C`、`Lift_U1`、`Stepper_42_U4`、`Servo`�?`ENCODER`、`Motors_All`、`TOF`、`MPU6050`、`Motors_PowerOn`。覆盖关系：四路电机开环与
单路命令 �?`RIG` �?`M/MR/MS/MALL`；舵�?�?`RIG` �?U5/U6 命令；编码器核对 �?`WHEEL`
�?`N` / `CMP` 遥测；TOF / MPU6050 单项验证取消（BSP 驱动保留，链接期 GC 丢弃）�?同步动作：`main.c` 分发链收窄为 4 分支（生�?HMI/WHEEL/RIG）；删除 `Motors_PowerOn`
专属�?`Board_MotorOnlyInit()`（`board.h` 声明 + `board_gpio.c` 实现）及其在
`stm32f4xx_it.c` �?`board_encoder.c` 里的 `MOTOR_POWERON_TEST` 条件编译�?两个工程文件里删�?9 �?`<Target>` 块与对应文件节点/条目（保�?LF，XML 已校验）�?删除 9 个测试输出目录；相关文档改为历史记录（开头有日期标注）�?
### 仍未解决

> **A / B / C 已按用户要求暂缓**（当前优先做底盘编码器闭环与麦轮调参），
> 等轮速环调完再回头处理，不要在调参期间顺手改升降�?
| # | 偏差 | 目标 |
|---|---|---|
| A | **「往返运动」功能丢�?*。旧 `stepper42_u1_lift.c` 里的 `LIFT_CYCLE_*`（上下限 / 端点暂停 / 往返次数）+ 非阻塞状态机 + SINGLE/CYCLE 双模式随文件删除一起没了（其唯一台架入口 `lift_smoke.c` / `Lift_U1` Target 也已�?2026-10-27 删除，BSP `board_lift.c` 保留但无调用入口�?| 按第 2 节它�?跨器件的动作编排" �?�?**App �?*（或恢复一�?`Lift_U1` Target）；暂停必须�?`HAL_GetTick()` 时间戳非阻塞等待，不�?`HAL_Delay` |
| B | ~~**生产固件动不了升�?*~~ —�?**2026-11-09 已解�?*（走新的 `pick_app` + `LIFT` 命令，不�?`board_lift.c`；参数在 `App/Inc/pick_config.h`）。仍缺：升降**没有限位/原点**，`LIFT` 按脉冲数点动、给多了直接顶死；`board_lift.c` 那套 `Lift_*` 仍无调用入口 | 要真正可用得先给 PD6/PD7 接限位，再把 `board_lift.c` 的限位停机逻辑接上 |
| C | **`LIFT_HOLD_AFTER_MOVE` 语义与实现矛�?*：`board_lift.c` �?`result != LIFT_RUN_COMPLETE` 时释放驱动器，其�?`LIFT_RUN_STOPPED`（中途停车）释放会让垂直丝杆平台因自重下滑，�?HOLD=1 的文档含义相�?| `LIMIT`/`FAULT` 释放、`STOPPED` 保持；或�?HOLD 的适用范围写清�?|
| D | **串口�?*还没进生产入口（升降 / 旋转 / 舵机 已于 2026-11-09 挂上 `ROT` / `LIFT` / `ARM` / `PLT` / `PICK`，见下方"机构命令"）�?026-11-09 �?`vehicle_app.c` 是新底板开环底盘命令，不再是电�?A + 夹爪 | 屏要�?`Board_InitStep6()` 和新板引脚（USART6 未变），不能把旧 `HMI` 命令原样并进�?|

### 工程收敛�?026-11-09�?
按用户要求，活动 Target 收成一个：`Vehicle_Integrated`。`WHEEL` 以及其余台架 Target（`HMI` / `RIG` / `CHASSIS_STEP`）不再作为烧录入口，工程文件移入 `_legacy/`。新底板命令�?`chassis_step_smoke.c` 拆进 App，脉冲驱动仍�?`board_step6`。旧直流电机命令（`A ON` / `G CLOSE`）没有并进来——它们依�?`Board_Init()`，会抢走 STEP 定时器�?
**收尾事实�?026-11-09，已完成，勿重做�?*：`Vehicle_Integrated.uvprojx`（LF、无 BOM、单 Target）的 BSP 组已�?`board_step6.c`，App 组已�?`chassis_io.c` / `chassis_motion.c` / `chassis_task.c` / `vehicle_app.c`�?`Bench_Tests.uvprojx` / `.uvoptx` / `.uvguix.GSH` / 两份 `.bak_*` 已移�?`MDK-ARM/_legacy/`�?`MDK-ARM/` 根下只剩这一份活动工程（其余 `build_*.log`、`add-chassis-target.ps1` 是历史脚本与日志，不是工程）�?全量重建 `-r` �?**0 Error / 3 Warning**�? 条均�?ST HAL `stm32f4xx_hal_flash_ex.c` �?`unused parameter 'Banks'`），日志 `MDK-ARM/Vehicle_Integrated/build.log`�?`Program Size: Code=17274 RO-data=1858 RW-data=1364 ZI-data=2676`�?*重构后的 HEX 未上板�?*

### 引脚补齐：舵机与旋转/升降使能�?026-11-09�?
按用户确认（舵机�?**PB14 / PB15**），�?BSP 里仍是老板接线的两处改成新底板 V2 as-built�?
- `pins/servo_u5.h` / `pins/servo_u6.h`：U5 夹爪 **PB14 / TIM12_CH1**、U6 载物�?**PB15 / TIM12_CH2**�?  两路共用 TIM12�? MHz 计数、ARR 19999 �?50 Hz）�?*删除�?`U5_SERVO_USE_PA7` 开�?*�?  TIM14/PE6 那一套（V2 �?PA7 还给 SPI1、PE6 �?B �?STEP，TIM9 �?B 轴占�?STEP，不可能再挂舵机）�?- `pins/stepper6.h` 新增 `ST6_EN_ROT_*`（PC8）、`ST6_EN_LIFT_*`（PC9）：
  之前只有注释写了"旋转/升降各有独立 nEN"，代码里没有宏、也没有初始化，PC8/PC9 一直是浮空输入�?  `board_step6.c` 现在把三�?nEN 都配成推挽输出、上电默认失能，`E 0`/`E 1` 一次开关三根�?- 舵机初始化抽�?`Servo_Init()`（`board.h` 声明），`Board_Init()` �?`Board_InitStep6()` 都调它：
  生产入口以前根本配不到舵机�?*两个通道初始化后都不启动**，要动必须显式发脉宽�?- `Tests/Src/rig_smoke.c`（归档台架）跟着删掉对已删宏的引用，回显改成 PB14/PB15 + TIM12�?
**验证**：`Vehicle_Integrated` 全量重建 `-r` **0 Error / 3 Warning**（仍只有 HAL `Banks`），
`Program Size: Code=17650 RO-data=1882 RW-data=1364 ZI-data=2676`�?**PB14/PB15 �?50 Hz 波形、PC8/PC9 的电平都还没上板量过**（命令在下一条补上）�?
### 机构命令：升�?/ 旋转 / 舵机 / PICK�?026-11-09�?
新底板第一次把这四样挂进生产入口，命令�?`vehicle_app.c`，编排在新模�?`App/Src/pick_app.c`，参数在新配置头 `App/Inc/pick_config.h`（含 `#error` 校验）�?**没有改动任何已有命令的行�?*，只加了新命令：

| 命令 | 作用 | 需�?`E 1` |
|---|---|---|
| `ROT <deg> [hz]` | 旋转台（PD12/TIM4）按**角度**点动，�?60 | �?|
| `LIFT <pulses> [hz]` | 升降台（PA6/TIM13）按**脉冲距离**点动，�?00000 脉冲，走够自�?| �?|
| `ARM <deg>` / `ARM OFF` | 夹爪/机械臂舵�?U5（PB14�?..180°（位置舵机，角度即距离） | �?|
| `PLT <permille> [ms]` / `PLT OFF` | 载物台舵�?U6（PB15）连续转 ±1000；带 `ms` 是定时点�?| �?|
| `PICK` | 一轮取放：�?�?�?�?�?�?�?�?�?�?�?�?回位�? 步，非阻塞） | �?|

**四条都是"点动距离"，不�?速度 × 时间"**：第一个参数是走多远（deg / 脉冲 / ms），
第二个可选参数才是一次走多快（Hz）。载物台是连续旋转舵机、没有位置反馈，距离只能折成时间�?
- `STOP` 现在会同时中�?`PICK`、停两轴、把载物台收回停止脉宽；`E 0` 也会中止 `PICK`�?- `S` 多了 `ROT` / `LIFT` / `SERVO` / `PICK` 四行状态�?- 参数与首次上板顺序写�?`Docs/机构指令_升降旋转舵机.md`�?- **全部未上�?*：方向、`PK_ROT_PULSES_PER_REV`、舵机角度映射、载物台停止点、`PICK` 行程
  都是估值或沿用�?RIG 台架的数，一个都没在 V2 上跑过。升降仍�?*没有限位/原点**�?- **速度**：`PICK` 的升降已�?2000 Hz 提到 **8000 Hz**�?4000 脉冲 7 s �?1.75 s），
  并给 `pick_app` 的两轴加�?*三段斜坡**（`PK_AXIS_RAMP_MS` = 50、`PK_AXIS_RAMP_PERMILLE` = 400）—�?  STEP/DIR 模式不加斜坡直接提速会跟丢或报过载。手�?`LIFT` 的第二个参数才是频率�?  第一个参数是脉冲距离。再往上调�?`Docs/机构指令_升降旋转舵机.md` §7�?
**验证**：`Vehicle_Integrated` 全量重建 `-r` **0 Error / 3 Warning**（仍只有 HAL `Banks`），
`Program Size: Code=22574 RO-data=2634 RW-data=1364 ZI-data=2732`�?`App/Src/pick_app.c` 已加�?`Vehicle_Integrated` �?App 组（`MDK-ARM/Vehicle_Integrated.uvprojx`）�?
### 陀螺辅助：HWT101 进生产入口（2026-11-09�?
HWT101 �?**USART2（PA2/PA3）@115200**，第一次进生产固件�?
- 新驱�?`BSP/Src/board_imu.c` + `BSP/Inc/board_imu.h`：维�?`0x55 0x53` 帧解析，
  **拼帧在主循环 `Imu_Poll()`**（串�?ISR 只入队），对�?`Imu_Read()` / `Imu_ReadYawMdeg()` /
  `Imu_ReadRateMdps()` / `Imu_IsFresh()` / `Imu_Zero()`。yaw 已减零点、已�?`IMU_YAW_SIGN` 换成
  "�?= 逆时�?，单�?0.001 度，�?wrap�?- `App/Src/chassis_motion.c`�?*只有拿到新鲜帧时才启�?*—�?  转身（`A` / `TASK` �?`TURN_ABS`）走**两段�?+ 过零判停**：`|误差| > CS_GYRO_TURN_SLOW_ZONE_MDEG`
  恒速，进减速区降到 `CS_GYRO_TURN_SLOW_HZ`�?*误差符号一变（刚扫过目标）就停**�?  `CS_GYRO_TURN_TOL_MDEG` 只是 0.3° 的兜底（**必须远小于一帧角�?*，否则会抢在过零前触发�?  转身系统性偏�?—�?2026-11-09 �?2.5° 时每�?`A 90` 都停�?88°）�?*整条转身只变两次�?*
  （每拍改频率会让 `Step6_SetHz()` 反复�?重配通道，手感就是一顿一顿）。步数与
  `CS_GYRO_TURN_TIMEOUT_MS` 只作兜底。平�?横移做航向保持（�?10 ms �?yaw 误差�?  B/D 加、A/C 减的差速）�?*没有陀螺帧时一条分支都不进，行为与改造前相同�?*
- `BSP/Src/board_imu.c` �?yaw �?*连续�?wrap** 的（模块�?±180 跳变，驱动按增量累积），
  转过 360° 就是 360000 mdeg；`Imu_Zero()` 以此为基准�?- `App/Src/chassis_task.c`：`TASK` 启动�?`RESET` �?`Imu_Zero()`；`TURN_ABS` 有陀螺时改走
  **`ChassisMotion_SpinAbs()`（绝�?yaw�?* —�?每站回到同一套绝对朝向，前几站的 0.3° 残差
  不会一站站累积；没有陀螺才退�?指令航向 + 相对�?的开环算法�?- `App/Src/vehicle_app.c`：主循环�?`Imu_Poll()`；新�?`Y`（yaw/帧数/帧龄/**raw 原始字节�?*）�?  `Z`（清零）；`S` 末尾多一�?`YAW`�?- 参数：`board_imu.h` �?`IMU_*`、`chassis_config.h` �?`CS_GYRO_*`，都�?`#error` 校验�?
**实板进展�?026-11-09�?*：HWT101 已连通（`frames` 增长、`bad=0`，模块出厂约 20 Hz�?�?100 Hz 的命令写�?`Docs/陀螺辅助底�?md` §2.1）。`A 90` 实测 yaw +89.6°�?`IMU_YAW_SIGN = 1` 正确。两版闭环转身的教训�?�?第一版把"角速度也要�?当停止条件，恒速下永远不成立，被步数上限截住；
�?第二版改成每拍比例调速，`Step6_SetHz()` 反复�?重配通道 �?手感一顿一顿�?现在定案�?*两段�?+ 过零即停**（恒�?�?减速区 600 Hz �?误差符号一变就停），整条转身只变两次速；
实板踩过"回传速率填成 0x06�?0 Hz�?，`age` 会到 95 ms、转身明显变�?（判据：10 Hz�?00 ms / 20 Hz�?0 ms / 100 Hz�?0 ms；模块每次更新发两帧�?所�?`raw` 增量 ÷ `frames` 增量恒为 22，不能当出数率用）�?
**验证**：全量重�?`-r` **0 Error / 3 Warning**（仍只有 HAL `Banks`），
`Program Size: Code=26040 RO-data=3440 RW-data=1364 ZI-data=2804`�?**A 90 已实板达标（残差 40�?00 mdeg、三次累�?+90.3°）；TASK 的绝�?yaw 转身�?航向保持 Kp/Kd、整条路线仍未验�?*；文档见 `Docs/陀螺辅助底�?md`�?
### 当前工作重点�?026-10-01 起，2026-11-09 改口径）：新底板开环底�?
编码器闭环那条线（`WHEEL`）已经归档，不要再往生产入口里接。现在要守的是新底板开环底盘：命令�?`vehicle_app.c`，运动在 `chassis_motion.c`，路线在 `chassis_task.c`，脉冲在 `board_step6.c`。下�?2026-10 的进度是旧车记录，不是当前入口�?
分阶段推进，**不要跳步**�?
> **进度�?026-10-28 六次更新�?*：阶�?1~4 的主体已�?`WHEEL` 台架实测可用 —�?> **直行（含航向保持）、原地转 `A ±deg`、倒车、`M/ML/MR` odom 判距�?> `TASK` 里程计化 + `TURN_ABS` 绝对角转�?*；比赛路�?**v0.3**�?7 步闭环）
> **已实板全程跑�?*（closure (�?8,�?8) mm �?10 cm）。据首跑日志做了四项精度
> 修正并修掉横移高频抖动（详见 `Docs/速度环调参与里程�?md` 追加八）�?> 满载（上底板+机械臂）横移/倒退驱动力严重不足且机械不可�?�?路线�?> v0.4（全前进试验，被当时三条硬约束否决）、v0.5（臂向必需横移 9 段，
> 均已编译未上板）演进�?**v0.6 全前进定�?*�?9 步：用户解除"工作区旁
> 禁转�?约束，除两条**几何必需**�?150 mm 短横移——发车位角距�?150 mm
> < 扫掠半径 212 mm 不可转身、前进又被墙堵死——外全部前进；到站朝向不�?> 就在站上转身�?WAIT；转�?17 个全部逆时针，终点车头朝北不能原样重跑�?> 已编译待上板，见追加九）。车外形定案 300×300。标度定�?CPR=810 /
> 轮径 75 mm / L=225 mm�?> 尚未收尾：v0.6 满载复跑（closure 期望 ±30 mm 内，超标则在扫码�?站点
> 编排 RESET）�?.5% 尺度标定（用户暂缓）、`L` 自转复测、`LAP`�?> **收尾完成前不要动升降（A/B/C 维持暂缓�?*�?> 这些过了才谈把底盘命令迁进生�?`vehicle_app.c`（第 11 �?D 项）�?
> **调试链路�?026-10-02 已就绪）**：PA2/PA3 改成 USART2 �?HC-05 蓝牙透传�?> （`SERIAL_BT` / `BOARD_BT_BAUD`，引脚见 `BSP/Inc/pins/bt_uart.h`），
> 遥测可以直接无线送到电脑上调参，不必拖着 USB-TTL 线跑车�?> 模块配置与接线见 `Docs/HC05无线串口.md`�?
1. **编码器可用化** —�?�?**编码完成、编译通过，待上板**�?026-10-01）：
   新增 `BSP/{Inc,Src}/board_encoder.*`（TIM7 100 Hz M 法测�?+ 低�?+ 换算 + 方向符号�?   �?`ENCODER` Target（`Tests/Src/encoder_smoke.c`�? 通道 FireWater 遥测 + 开环命令）�?   台架实测已确�?**TIM7 tick 在跑**（`t` 通道持续增长）；编码器计数仍�?0�?   原因�?*编码器未供电**（用户确认只�?STM32 供了电）�?   详见 `Docs/底盘编码器与麦轮调参.md`�?2. **单轮速度�?*（先�?A 路）：控制环 100 Hz，`duty = 前馈 + Kp·err + Ki·∫err`�?   限幅 + anti-windup，前馈用 Kv = 27.8 rpm/V。VOFA+ �?   `Tools/serial_scope.html` 同时�?目标 / 实际 / PWM 三条线调 Kp/Ki�?3. **复制�?4 �?*：确认每路符号与缩放一致，逐路验方向�?4. **麦轮逆运动学**�?vx, vy, ωz) �?四轮速度，先架空跑，再落地跑直线 / 横移 / 自转�?5. 需要更高频调参再上 JustFloat + 提高波特率（见第 6 节遥测规则）�?
已修掉（本会话核对通过，勿重复修）�?
- �?**`Encoder_Read()` 的副作用**：旧实现�?B/C 每次调用都会累加并更�?`last_*_counts`�?  控制中断与主循环遥测会互相吃掉增量。已拆成
  `Encoder_SampleCounts()`（tick 里唯一的更新点�? `Encoder_ReadCounts()`（无副作用）�?  `board.h` 里的 `Encoder_Read` 已改名，旧名无代码残留�?
仍需注意（尚未处理，用户已知情）�?
- **AT8236 没有 STBY/nSLEEP**：`Motor_Standby()` 现在是空实现，`Motor_StopAll()` 只把
  PWM �?DIR 清零（滑行）；STOP 之后直接 `Motor_Set()` 就能转，不需要再“唤醒”�?- **保持 `Motor_Set()` �?PWM+DIR 语义**：正�?= DIR 有效 + PWM 占空比，负数 = DIR 反向 +
  PWM 占空比；`MOTOR_DIR_FORWARD_LEVEL`（`board_motion.c`）是全局 DIR 极性�?- **闭环输出必须限幅**：`Motor_Set` 允许 1000‰（=100% 占空比）�?  闭环输出不应接近这个值（1.0 m/s 约需 76% 空载转速，所以建议夹�?600~700‰）�?- **AT8236 电流余量**：单通道峰值约 6 A，比 TB6612 �?3.2 A 宽裕，但仍没有可编程限流�?  堵转仍要�?`WHEEL_STALL_*` 软件检测和短时测试�?- **编码器供电建�?3.3V**：输出即 3.3V 电平，避�?5V 引脚问题�?- **PA2/PA3 已从「ADC1 预留」改成蓝牙串�?USART2**（`SERIAL_BT`，`pins/bt_uart.h`）：
  ADC 初始化与 `ADC_ReadChannel()` 已删除（本来就没有任何调用者，被链接器丢弃）�?  这两个脚**不能再做模拟输入**，将来要测电机电流必须换引脚；它们也**不是 5V 容忍**�?  只能�?3.3V 逻辑。改动理由与代价写在 `Docs/PINMAP.md` �?`Docs/HC05无线串口.md`�?