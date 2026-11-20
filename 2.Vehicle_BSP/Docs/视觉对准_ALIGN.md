# 视觉对准 ALIGN：树莓派报坐标，底盘四轮微调对准物料圆心

> 2026-11-11 新增；2026-11-16 改成连续速度环 + `TRACK` 连续追踪，
> 并把 `GRAB` 改成"追踪锁定后自动夹"，另加阶跃剔除/解锁滞回。
> `Vehicle_Integrated` 全量重建 `-r`：**0 Error / 3 Warning**（3 条都是 ST HAL
> `stm32f4xx_hal_flash_ex.c` 的 `unused parameter 'Banks'`，见 AGENTS.md 第 10 节）。
> `TRACK` 实车已验证收敛；自动夹、阶跃剔除、滞回**未上板复测**。

## 1. 要解决什么

夹爪圆心正对相机光轴，`$V` 帧里的 `x_mm / y_mm` 又以**车体中心**为原点，
所以"把物料圆心对到夹爪下面"就等价于把 `(x_mm, y_mm)` 收敛到 `(0, 0)`。
这段逻辑不猜位置、不依赖里程计，只看视觉给的两个数。

链路：

```
树莓派 vision_node.py --task color --want 3 --port /dev/serial0
        │  $V,<seq>,3,<x_mm>,<y_mm>,...  115200 USART3(PC10/PC11)
        ▼
STM32  board_vision.Vision_Poll()      ← 只拼帧，不发任何东西
        ▼
       Align_Poll()                     ← App/Src/align_app.c
        │  每 100 ms 按误差给四轮一个速度
        ▼
   Step6_SetHz(A/B/C/D)                 ← 频率没变不重发，避免打断 PWM
```

对准成功后若发的是 `GRAB` 而不是 `ALIGN`，主循环再调 `Pick_GrabStart()`：
张开 → 快降 → 慢降贴近 → 夹紧 → 慢抬离 → 升回最高处。对准失败不下降。

## 2. 命令

| 命令 | 作用 |
|---|---|
| `V` | 打印视觉链路统计 + 最近一帧（先确认能收到帧，再用 ALIGN） |
| `VER` / `VERSION` | 打印固件标识，例如 `FW 2026-11-16 cls-hint2`。**命令"发了没反应"时先看它** |
| `HINT <cls>` | 只把类别下发给树莓派（USART3 的 `$C,<cls>`），**不动任何电机**；用来单独测下行链路 |
| `ALIGN <cls> [tol_mm]` | 开始对准。cls：1-6 颜色、10 载物盘、11 色环；tol 省略用默认 8 mm |
| `ALIGN` / `ALIGN ?` | 打印当前状态（是否在跑、误差、速度、航向保持、上次结束原因） |
| `ALIGN STOP` | 中止（同时停车）。`GRAB` 已挂上但还没夹的话一起取消 |
| `TRACK <cls> [tol_mm]` | 连续追踪：目标动就一直跟，**不报 done、不自己停**；目标停稳且进容差报一次 `TRACK locked` |
| `TRACK` / `TRACK ?` | 打印状态（`mode=track`、误差、速度、前馈、`locked`） |
| `TRACK STOP` | 中止追踪（同时停车） |
| `GRAB <cls> [tol_mm] [HOLD]` | **追踪锁定后取料，默认顺手放到载物台**：升到放料高度 → 转 -132° → 松开 → 载物台走一格 → 转回 → 升回最高处。`HOLD` 只夹不放。升降累计位置必须是 0（最高处） |
| `PLACE` | 手里夹着料、停在最高处时，只做放料那半段（`GRAB ... HOLD` 之后用） |

`<cls>` 可以写数字，也可以写颜色名（大小写都行）：`RED`=1 `YELLOW`=2 `BLUE`=3
`GREEN`=4 `BLACK`=5 `LIGHTBLUE`=6 `TRAY`=10 `RING`=11。蓝牙上直接 `GRAB BLUE` 更省事。

`TRACK` 和 `GRAB` 用的是同一套跟随律，区别只有一个：`TRACK` 锁定后继续跟，`GRAB` 锁定后直接转去夹。
原来的"一次性 ALIGN 对准 + 夹"因为物料一动就白对，已经并进 `GRAB`。

回显示例：

```
 OK ALIGN cls=3 tol=8 mm ofs=(0,0)
 ALIGN e=(120,-30) mm  v=(80,-64) mm/s
 ALIGN e=(52,-11) mm  v=(80,-12) mm/s
 ALIGN done, e=(5,-6) mm

 OK TRACK cls=3 tol=8 mm ofs=(0,0) yaw=hold
 TRACK e=(120,-30) mm v=(75,-20) mm/s ff=(40,-5) locked=0
 TRACK e=(9,-4) mm v=(30,-2) mm/s ff=(28,0) locked=0
 TRACK locked, e=(4,-3) mm

 OK TRACK cls=3 tol=8 mm ofs=(0,0) yaw=hold      ← 注意：GRAB 也用追踪律
 TRACK locked, e=(3,-2) mm
 ALIGN done, e=(3,-2) mm                          ← 由 GRAB 触发
 GRAB start, pick then place (STOP aborts)       ← 取料 → 转 -132° → 松开 → 盘走一格 → 转回
 GRAB done, placed, held at top

 # 只夹不放（调试）：
 GRAB start, open-down-close-up (STOP aborts)
 GRAB done, held at top
 > PLACE
 PLACE start, turn-open-step-back (STOP aborts)
 PLACE done, held at top
```

`ff` 是两轴的目标速度估计（mm/s），`locked` 只在"目标停住且已在容差内"时为 1。

状态行每 0.5 s 打一条（`ALIGN_STATUS_EVERY`），`v` 是这一拍下发给四轮的合成速度。

方向约定照抄 `chassis_task.c` 里 TASK 的定义：
FWD `(+,+,+,+)`、BACK `(-,-,-,-)`、LEFT `(-,+,+,-)`、RIGHT `(+,-,-,+)`。

## 3. 参数（`App/Inc/align_config.h`）

| 宏 | 默认 | 含义 |
|---|---|---|
| `AL_TOL_DEFAULT_MM` | 8 | 默认容差（\|x\| 和 \|y\| 都进容差才算到位） |
| `AL_SETTLE_HITS` | 3 | ALIGN 报 done / TRACK 报 locked 需要连续几个不同 seq 的确认帧 |
| `AL_SERVO_PERIOD_MS` | 100 | 控制拍周期，与视觉 10 Hz 同频 |
| `AL_FRAME_HOLD_MS` | 150 | 同一帧最多再推这么久，到点就停，等下一帧 |
| `AL_NUDGE_MS` | 80 | 最低速会冲过零点时，只点一下的时长 |
| `AL_SERVO_GAIN_MILLI` | 2500 | 比例增益，2.5 (mm/s)/mm |
| `AL_SERVO_MAX_MM_PER_S` | 80 | 速度上限，也是斜向合成后单轮线速度上限 |
| `AL_SERVO_MIN_MM_PER_S` | 12 | 速度下限。已经在容差内的轴不会被抬到这个值 |
| `AL_HZ_UPDATE_MIN` | 12 | 频率变化小于它就不重发，避免每拍重配定时器 |
| `AL_HZ_PER_MM_PER_S_MILLI` | 12732 | 轮速→STEP 频率，换轮径/细分必须改 |
| `AL_YAW_HOLD_ENABLE` | 1 | 有新鲜陀螺帧时锁住启动瞬间的 yaw |
| `AL_TIMEOUT_MS` | 30000 | 总时长上限 |
| `AL_STALE_MS` | 300 | 本机收到帧之后超过它立即停车 |
| `AL_VISION_AGE_MS` | 500 | 视觉侧自己填的 age_ms 超过它，这帧不用（单应一帧要几百 ms） |
| `AL_CONF_MIN_PERMILLE` | 400 | 置信度低于它的帧不当目标 |
| `AL_LOST_MS` | 2000 | 连续这么久没可用目标才中止 |
| `AL_GRIPPER_OFFSET_X_MM` / `_Y_MM` | 0 | 夹爪相对光轴的机械偏差 |
| `AL_TRACK_FF_MILLI` | 700 | 目标速度前馈比例，千倍整数；0 = 关闭，退化成纯比例跟随 |
| `AL_TRACK_FF_MAX_MM_PER_S` | 300 | 目标速度估计上限，防止脏数据把车甩出去 |
| `AL_TRACK_FF_REJECT_MM_PER_S` | 120 | 超过它判为"跳变"（手碰/重捕），这一帧不更新前馈 |
| `AL_TRACK_UNLOCK_MARGIN_MM` | 5 | 解锁滞回：误差超过 tol+5 mm 才报 unlocked；夹取仍要求真正回到 tol 内 |
| `AL_TRACK_FF_MIN_MM_PER_S` | 8 | 前馈死区，低于它当成目标静止 |
| `AL_TRACK_FF_STILL_MM_PER_S` | 15 | 判"目标停住"的门限，锁定要求两轴速度都低于它 |
| `AL_TRACK_FF_HOLD_MM_PER_S` | 20 | 前馈超过它时这一帧按整帧持续走（否则靠近零点只点动） |
| `AL_TRACK_VEL_DT_MIN_MS` / `_MAX_MS` | 20 / 500 | 速度差分的有效时间窗，窗外的样本丢弃并清估计 |
| `AL_TRACK_TIMEOUT_MS` | 120000 | 追踪总时长上限，0 = 不限 |
| `AL_TRACK_LOST_MS` | 3000 | 追踪时丢目标多久才中止（比 ALIGN 宽松，手挡一下不退出） |

**本车的摄像头与夹爪在同一个 xy 坐标上，只是高度不同**（2026-11-11 确认：机械臂基座
在车体右侧，但夹爪末端正对相机光轴）。所以两个 offset 都保持 **0**，误差收敛到 0 时
夹爪就在物料正上方，只差一个下降动作。这两个宏留着只为吸收装配公差——
实测咬偏了几毫米就填几毫米（x 前正、y 左正）。

物料是**圆柱**，所以不用把物料自身的 `yaw_mdeg` 对到某个角度。
ALIGN 只做平移，但会用陀螺把**车体航向**锁在启动那一刻，避免麦轮斜走把车带转。
没有陀螺帧时这一项自动关掉。换成有方向的长方体物料时，才需要再加"先转到物料角度容差内"。

因为夹爪就在光轴上，物料最终停在画面**正中**，单应标定用画面中心那块区域即可，
这是误差最小的位置——不用像夹爪偏置时那样去标边缘。

两个轴同时走。已经在容差里的那一轴速度是 0，不会被最低速再推一把
（旧实现会把 1 mm 的轴也抬到 12 mm/s，车在目标附近画圈）。
斜向时 \|vx\|+\|vy\| 再限一次，单轮不会比直走更快。

## 4. 状态机

2026-11-11 改成**连续小幅度调整**（原来是"走一大步 → 停 → 再量"）：

每 `AL_SERVO_PERIOD_MS`（100 ms）读一次快照，按误差给四个轮子一个速度，
速度 = 误差 × `AL_SERVO_GAIN_MILLI`，限幅 `AL_SERVO_MAX_MM_PER_S`。
已经在容差内的轴输出 0。算出来低于 `AL_SERVO_MIN_MM_PER_S` 时，只有"保持一整帧也不会冲过零点"才抬到最低速，否则只按 `AL_NUDGE_MS` 点一下。

同一帧最多再推 `AL_FRAME_HOLD_MS`。时间到了立刻停，等下一个 seq，
不拿旧坐标继续盲走。`Step6_SetHz()` 每次都会停 PWM 再重配，所以频率变化小于
`AL_HZ_UPDATE_MIN` 不重发。

到位不是看单帧：车已经停住之后，连续 `AL_SETTLE_HITS` 个不同 seq 的帧都在容差内，才报 `ALIGN done`。
置信度低于 `AL_CONF_MIN_PERMILLE`、或视觉侧 `age_ms` 超过 `AL_VISION_AGE_MS`，这帧当没有目标。

有新鲜陀螺帧时，把启动瞬间的 yaw 锁住，平移过程中用左右差速拉回来（和底盘航向保持同一套符号）。
没有陀螺帧不拒绝对准，只是不修航向。停车确认期间不加差速，免得把车重新推出容差。

中止条件：超时、持续没可用目标（丢帧立刻停车，等 `AL_LOST_MS` 才中止）、
步进被 `E 0`、被手动命令或 `STOP` 取消、任务/点动/机构序列抢了四轮。

轮速合成（符号照抄 `chassis_task.c`）：`A = vx - vy`、`B = C = vx + vy`、`D = vx - vy`。
纯前走时四个都是 `vx`，纯左移时正好是 `(-vy, +vy, +vy, -vy)`。

手动命令（`M` / `J` / `L` / `R` / `A` / `TASK` …）都会先经过 `cancel_auto()`，
那里会调 `Align_Cancel()`，所以对准中随手发个命令就会退出对准，不会打架。

### 4.1 追踪模式（`TRACK`）——给"物料会动"准备的

`ALIGN` 是"走到位就收工"：进容差停稳就报 `done` 并停车，之后物料再动它也不管。
物料在转盘/传送带上会动时，需要的是**一直跟**，所以加了 `TRACK`。

控制律（和 ALIGN 的区别只有一处）：

```
ALIGN : v = 误差 × Kp
TRACK : v = 误差 × Kp + 目标速度 × AL_TRACK_FF_MILLI
```

**为什么必须加前馈**：视觉 10 Hz，一帧的延迟加帧周期约 0.2 s。纯比例跟随一个匀速移动的
目标时，会稳定差"目标速度 × 0.2 s"那么远——目标 100 mm/s 就永远差 20 mm，比 8 mm 容差还大。
把测到的目标速度直接加到轮速上，这个固定滞后就补掉了，比例项只负责修剩下的残差。

目标速度怎么来：相邻两帧的 `(x_mm, y_mm)` 差除以两帧的接受时刻差，限幅到
`AL_TRACK_FF_MAX_MM_PER_S`，再过 1/4 一阶低通。同 seq 的重发帧、时间间隔超出
`AL_TRACK_VEL_DT_MIN_MS..MAX_MS` 的样本都不参与（断帧后直接把估计清零，免得旧速度
在重新捕获目标时把车猛推一下）。

运行中的四个分支：

| 情况 | 输出 |
|---|---|
| 误差 > 容差 | `Kp·误差 + 前馈`，限幅 80 mm/s |
| 误差 ≤ 容差，目标还在动 | 只给前馈速度（跟着走，误差不再变大） |
| 误差 ≤ 容差，目标停住 | 停车，开始计 `settle_hits` |
| 容差内且连续 `AL_SETTLE_HITS` 帧都停住 | 报一次 `TRACK locked`，**继续跟踪** |

"目标停住"= 两轴目标速度估计都 ≤ `AL_TRACK_FF_STILL_MM_PER_S`。所以路过容差带的
移动目标不会被误判成锁定，只有真停下才 `locked`。

结束条件：只有 `TRACK STOP`、超时（`AL_TRACK_TIMEOUT_MS`）、连续丢目标
（`AL_TRACK_LOST_MS`）、`E 0`、被手动命令/任务取消。**它不会自己报 done。**

追踪完要夹取：直接 `GRAB <cls>`，它就是"跟到锁定 → 停 → 夹"一条命令。
如果已经用 `TRACK` 跟上了，也可以在锁定后 `TRACK STOP` 再 `GRAB <cls>`（会重新跟一次）。

### 4.2 车绕着物料转圈：相机安装角没算进去

2026-11-16 实机第一次跑 `TRACK 3` 的日志长这样（节选）：

```
TRACK e=(-21,36) mm v=(-31,48) mm/s ff=(0,0) locked=0
TRACK e=(-43,20) mm v=(-57,22) mm/s ff=(-40,-28) locked=0
TRACK e=(-49,-31) mm v=(-40,-40) mm/s ff=(-8,-88) locked=0
TRACK e=(-13,-62) mm v=(6,-73) mm/s ff=(57,-62) locked=0
...
```

特征很明确：**`|e|` 恒定（40~60 mm）、角度每 0.5 s 转 ~50°，`ff` 被带得很大**。
这不是"没收敛"，是误差向量在匀速旋转——指令方向和车实际走的方向差了 **90°**。
纯比例的闭环遇到 90° 的轴系错位，误差 dynamics 变成纯旋转（`ė = -Kp·R(90°)·e`），
半径不增不减，车就沿着以物料为圆心的圆一直走。
差 180° 会**发散**，差几度只是**收敛慢**，只有 90° 才会这么规矩地绕圈。

根因：相机相对车头装了 90°，而 `align_send.py` 默认按"画面正上方 = 车头"换算。
2026-11-16 由日志反推为 **+90°**（逆时针），已写成默认值
（`common.CAMERA_MOUNT_DEG`，`align_send.py` 的 `--rot-deg` 默认取它）。

判据（换相机/重新装过之后按这个复核）：

把物料放在**车头正前方 300 mm** 处，看终端打印的 x/y：

| 打印结果 | 说明 | 该填的 `--rot-deg` |
|---|---|---|
| x≈+300、y≈0 | 相机正装 | 0 |
| x≈0、y≈+300 | 相机逆时针歪 90°（物料显示在画面**左**边） | +90 |
| x≈0、y≈-300 | 相机顺时针歪 90°（显示在**右**边） | -90 |

改完再跑 `TRACK 3`：`e` 应该单调变小、`ff` 收到 ±20 mm/s 以内，最后出现
`TRACK locked`。还不对的话按现象分：

| 现象 | 原因 | 处理 |
|---|---|---|
| 半径不变、角度一直转（绕圈） | 安装角还差 90°（0/90 选错，或符号反了） | 换成 0 或 90 再试 |
| 半径越来越大（螺旋外扩） | 符号反了（+90 ↔ -90） | 取负 |

主机侧闭环仿真（同一个比例律、忽略航向）复现过：相机 +90°、发送端不补偿 → 半径恒定的
**逆时针**绕圈；补成 +90 → 收敛到 0；补成 -90 → 螺旋外扩。日志里的角度是**逆时针**，
所以是 +90，不是 -90。

### 4.3 阶跃剔除、解锁滞回、锁定后自动夹（2026-11-16 晚）

实机第一次跑通 `TRACK` 的日志里有这么一段：

```
TRACK e=(-2,-3) mm v=(0,0) mm/s ff=(-1,-3) locked=1
TRACK unlocked
TRACK e=(12,-1) mm v=(56,0) mm/s ff=(38,4) locked=0
TRACK e=(-12,-13) mm v=(-47,-32) mm/s ff=(-46,-16) locked=0
TRACK e=(10,4) mm v=(52,12) mm/s ff=(39,18) locked=0
```

两处小毛病，这次一起治了：

1. **阶跃被当成速度**：一帧里目标跳了 14 mm（手碰、遮挡后重捕），按 140 mm/s 算，
   前馈立刻给 38 mm/s，车冲过头，接着 ±12 mm 来回两三次。
   现在 `update_target_velocity()` 里，估计速度超过 `AL_TRACK_FF_REJECT_MM_PER_S`(120 mm/s)
   就**不更新前馈并清零**——车身只有 80 mm/s，更快的目标本来也追不上，丢掉前馈不影响跟随。
2. **容差边界抖状态**：原来一出 tol 就 `unlocked`。现在要超过 `tol + AL_TRACK_UNLOCK_MARGIN_MM`
   (5 mm) 才解锁，边界上不再反复刷屏。

**锁定后自动夹**：`GRAB <cls>` 现在走追踪律，`vehicle_app` 每轮 `Align_Poll()` 之后调
`Align_TryCompleteForGrab()`。它要求 `locked==1` **且** `|dx|<=tol && |dy|<=tol`
（锁定本身允许容差外 5 mm 的滞回，所以夹取另做一次真实误差检查），满足就按成功收尾、
停车、打印 `ALIGN done, e=(x,y) mm`，紧接着原有的 `Pick_GrabStart()` 下降。
超时、丢目标、被 `STOP`/手动命令取消，一律不下降。

## 5. 台架验证（不需要相机、不需要上车）


1. 串口通：USB-TTL 的 TX 接 PC11（USART3 RX），GND 共地，115200；
2. STM32：`E 1` → `V` 看 `VIS ok=` 在涨；
3. `ALIGN 3` / `TRACK 3`：看 `ALIGN e=` / `TRACK e=` 是否朝缩小方向走；
   `STOP` 必须立刻停；
4. `GRAB 3`：锁定 → 下降取料，中途 `STOP` 不许下降；
5. 只想看画面：`main.py --no-send`（或 `align_send.py --no-send`）。

## 6. 上车验证

相机固定、平面外参标好之后：

1. 树莓派 `align_send.py --cls <任务码颜色>`（或 `vision_node.py --task color --want …`）
2. 车摆在物料前约 300～500 mm，`E 1`
3. **先接好 HWT101**：`S` 里应看到 `YAW ok yaw=...`，`TRACK` 的启动行应是 `yaw=hold` 而不是 `yaw=off`。
   有陀螺帧时 TRACK/ALIGN 会锁住启动瞬间的航向，长距离跟随不会被麦轮斜走带偏；
   `yaw=off` 说明没收到帧（检查 USART2/PA2/PA3、模块供电、`IMU_POLL` 是否有 `frames` 增长）。
4. `GRAB <cls> 5`：追到锁定后取料，再自动放到载物台（`HOLD` 只取料，`PLACE` 单独放）
5. 看打印的 `ALIGN done, e=(x,y) mm`：**这就是夹爪圆心与物料圆心的残差**，再用卷尺核对一次咬合点。

**记 10 次锁定残差**（判夹爪是否真在光轴上）：连续跑 10 次 `GRAB <cls>`（每次先把物料挪到
不同位置），把每次的 `ALIGN done, e=(x,y) mm` 抄下来，算：

```bash
# 把 10 行 "e=(x,y)" 粘进 stdin，输出 RMS / 最大 / 平均
python -c "import sys,re,math; v=[tuple(map(float,m.groups())) for m in (re.search(r'e=\((-?\d+),(-?\d+)\)',l) for l in sys.stdin) if m]; print('n=%d rms=%.1f mm max=%.1f mm mean=(%.1f,%.1f)'%(len(v), math.sqrt(sum(x*x+y*y for x,y in v)/len(v)), max(math.hypot(x,y) for x,y in v), sum(x for x,y in v)/len(v), sum(y for x,y in v)/len(v)))"
```

判据：`rms ≤ 容差`，且 10 次的均值 `(mean_x, mean_y)` 就是**夹爪相对光轴的固定偏差**——
把它取负填进 `AL_GRIPPER_OFFSET_X_MM / _Y_MM`（x 前正、y 左正），下一位选手就不用再猜。
均值接近 0 说明夹爪确实在光轴上。< 5 mm 的离散度说明重复性好；离散度大先查单应抖动或颜色掩膜。

追踪上车（物料会动时）：

1. 把物料放在转盘上让它转/推着走，`TRACK <cls> 10`；
2. 看 `ff` 是否跟得上物料速度、`e` 是否稳定在容差附近不持续发散；
3. 物料停下后应出现 `TRACK locked`；
4. 跟丢（手挡、出画）时车立刻停，`AL_TRACK_LOST_MS` 后 `TRACK aborted: no target`。

判据：物料以 ≤ 50 mm/s 匀速移动时，`|e|` 稳态不超过 2 倍容差（前馈该补掉大部分滞后）；
超过说明 `AL_TRACK_FF_MILLI` 偏小或视觉延迟比预期大。**先别在 > 80 mm/s 下试**——
车身限速就 80 mm/s，追不上是物理限制，不是参数问题。

> **2026-11-17 起入口统一为 `main.py`**（双相机 + 串口 + `$C`/`@SCAN` 分发）；
> `align_send.py` 保留为检测引擎和单相机调试入口，下面的命令仍然可用。

## 7. 树莓派侧（`$V` 是谁发的）

对准用 **`Tools/vision/align_send.py`**：每帧检测全部颜色 → 按当前类别选一块 → 像素偏移
换算成毫米 → 发 `$V`。**不需要任何标定文件**，只要两个手工常数。

```bash
python3 align_send.py --cls 3                     # 自动找串口发帧 + 实时画面
python3 align_send.py --port COM7 --cls 3         # PC 上接 USB-TTL 时指定口
python3 align_send.py --port /dev/serial0 --cls 3 --px-per-mm 1.62 --rot-deg 90
python3 align_send.py --no-send                   # 只出画面，不发串口
python3 align_send.py --cls 11 --ring-px-per-mm 2.4   # 圆环模式（DROP 空爪对准）
```

`--rot-deg` 是**相机相对车头的安装角**（逆时针为正），本车默认 90，见 §4.2；
换相机、重新装过相机之后必须按 §4.2 的表复核一次，否则会绕圈。

**串口发送默认开**：先试 `/dev/serial0`（树莓派 GPIO 排针那道口），再按
CH340 / CP210x / FTDI / STM32-VCP 的优先级扫 USB 串口，打开时会打印实际用的口名。
自动挑错了就 `--port` 指定；不想发就 `--no-send`。

**帧率**（2026-11-16 实测，PC）：检测默认 `--detect-scale 0.5`（640×480 内部缩到 320×240，
坐标自动换算回原图，`PX_PER_MM` 不用改）：

| 模式 | 单帧检测耗时 | 说明 |
|---|---|---|
| `--detect all --detect-scale 1.0` | 16 ms | 六色全检，原分辨率 |
| `--detect all --detect-scale 0.5` | **4 ms** | 默认 |
| `--detect single --detect-scale 0.5` | **<1 ms** | 只算当前类别，切类靠 `$C`，最快 |

预览的 fps 上限由 `--hz` 决定（默认 10，和固件 100 ms 控制拍一致）。
若实测只有 5 fps，说明瓶颈不在检测（200 ms/帧），先按这个顺序排：

1. `--no-preview` 跑一遍，看终端 `TX` 行是不是 10 行/秒——是则预览/窗口是瓶颈；
2. 仍然慢就降相机分辨率：`--width 320 --height 240`（`PX_PER_MM` 要重新量）；
3. 相机曝光太长/后端不对也会拖慢采集，`v4l2-ctl --list-formats-ext` 确认格式。

**灰度图不会更快**：六色掩膜依赖 HSV 的 H/S，转灰度拿不到；黑色路径也只用 V/S，
灰度只是白转一次。

**环模式（`cls=11`）**：收到 `$C,11[,<n>[,<rot>[,<stage>]]]` 自动切到 `detect_rings()`——灰度 → CLAHE(5.0) →
高斯 → `HoughCircles(HOUGH_GRADIENT_ALT, param1=300, param2=0.85)` → 圆心聚簇，
只报**离夹爪最近的一个环**（车停在目标环前，不需要三个都入画）。
参数：`--ring-px-per-mm`（**在释放高度量**，底部精对，不能沿用颜色那套 1.5）、
`--ring-px-per-mm-top`（**在最高处量**，顶部粗对；0 = 用底部值）、`--ring-min-radius`、
`--ring-outer-diameter-mm`（**推荐**：填圆环外径，按每帧检出半径自动算 px/mm，顶部/底部通吃）、
`--ring-max-radius`、`--ring-param2`、`--ring-min-circles`（底部默认 3）、
`--ring-min-circles-top`（顶部默认 2）、`--ignore-top 0.10`（上部 10% 不参与检测）、
`--detect-always`（调试时忽略 `@DETECT` 门控）、
`--shape-fallback`（颜色完全没检出时先找圆形/圆柱轮廓再判色）、
`--shape-min-circularity 0.60` / `--shape-min-fill 0.55` / `--shape-min-area-px 600`；
`--smooth-alpha 0.5`（目标中心低通，越小越稳）、`--smooth-jump-px 80`（跳变重置）、
`--center-mode contour`（默认，颜色轮廓质心；`circle` 外接圆 / `blend` 平均 /
`distance` 最大内接圆圆心，抗残缺色块；circle 对残缺轮廓会跳，优先试 distance）。**只有 ≥N 个不同半径的同心圆才算环**，单圈/碎圆丢弃；
视觉**默认只用 `--rot-deg`（90）**，不叠加 `$C` 里的转台角；只有相机确实随转台转时，才加
`--ring-use-arm` 把转台角叠上去。
选中后会在帧间锁住同一个环，不会在两个环之间来回跳。`stage=0` 顶部粗对（不做数字识别），`stage=1` 底部精对。

**读数字**：裁环心 → OTSU → 居中最大轮廓 → 归一化 64×64 → 与 `digit_templates/1..3.png`
做四方向模板匹配（模板用 `make_digit_templates.py --image 图案照片` 从印刷图案生成，
换图案重跑一次即可）。`$C,11,<n>` 的 n 是**期望环号**（只在底部精对读）：读到数字不一致时
`--ring-digit-check warn`（默认，只提示）/ `strict`（不发目标，DROP 会因对准超时退出）/ `off`；
`--ring-digit-min-score` 默认 0.35。预览会画选中的环并显示 `ring 2 (0.98)`。
合成图测试：三环各自读到 1/2/3（score 1.0）；单环 `$C,11,2` 正常发目标；strict 不匹配不发。


## 7.1 类别可以运行时切换（蓝牙换色）

`align_send.py` 启动时用 `--cls` 定一个初始类别，但**之后每次 STM32 启动
`ALIGN` / `TRACK` / `GRAB <cls>`，都会往 USART3 回一行 `$C,<cls>`**，脚本收到就把
目标色切过去（日志里会打 `[STM32] 目标类别切到 1（red）`）。所以：

- 树莓派上先跑 `python3 align_send.py`（`--cls` 随便给，作为没收到指令时的默认）；
- 手机蓝牙连 HC-05（UART5 是第二个命令口），直接发 `GRAB 3` / `GRAB BLUE`；
- 固件把类别下发 → 视觉换色 → 开始跟 → 锁定 → 取放。

**前提**：USART3 是双向接的（PC10→Pi RX、PC11←Pi TX，见 `Docs/PINMAP.md`），
且树莓派侧开了串口（`--no-send` 就收不到下行）。没接成双向也不影响旧用法：
类别就固定在 `--cls` / `--want`。

> **排查下行**：`align_send.py --link-test` 只读不写，STM32 发 `HINT 3` 应看到 `RX b'$C,3\n'`；
> 没有就查 PC10（STM32 TX）→ 树莓派 RX、共地、两边 115200，以及 `VER` 是否当前固件。
>
> `align_send.py` 现在**每帧检测全部颜色**（复用 `vision_node.detect_blobs`，含黑色走
> `mode=dark` 的那条路径），`$C` 只决定"这一帧发哪一类"，所以 `GRAB BLACK` 也能用。
> 载物盘 10 / 色环 11 不是颜色块，`align_send` 里没有对应检测：收到 `$C,10` / `$C,11`
> 会打一行"没有这一类，忽略"并保持原类别，固件最终 `no target`——这两类要用
> `vision_node.py --task tray|ring`。

`vision_node.py` 同样支持 `$C` 切换（覆盖 `--want`），前提也是给了 `--port`。

| 常数 | 怎么定 |
|---|---|
| `PX_PER_MM` | 物料沿车头方向挪 100 mm，看像素 y 变化多少，除一下。默认 1.5 |
| `ROT_DEG` | 相机相对车头的安装角，逆时针为正。正装 0；**本车默认 90**（§4.2）。判据：物料放车头正前方应读到 x>0、y≈0 |

为什么可以不要标定：ALIGN 是闭环，每帧重新测误差。像素→毫米的尺度差 20%，车只是
每一步走多/走少，照样收敛到 0；安装角差几度只让收敛变慢。**先跑通闭环，再谈精度。**

`vision_node.py` 仍然保留（带 `$V` 协议、六色识别、存图、`--homography` 单应分支），
但**不能和 `align_send.py` 同时跑**——两者抢同一个 `/dev/serial0`。

只有需要**区分高度**（物料台面 120 mm 与地面 0 mm 两个平面）或要把视觉坐标与里程计
做比较时，才需要毫米坐标：跑一次 `calib_plane_h.py`（五点单应，不需要棋盘格/内参），
每个高度各标一张。内参 + solvePnP 那条完整标定路线已于 2026-11-16 删除。

PC 上相机默认 `0`，树莓派上换成 `/dev/cam_target`。

## 8. `GRAB`：追踪锁定 → 取料 → 放到载物台

```
GRAB 3            # 跟随 → 锁定 → 停车 → 取料 → 转 -132° → 松开 → 盘走一格 → 转回 → 升回最高处
GRAB 3 5          # 容差 5 mm
GRAB 3 8 HOLD     # 只取料，夹着料停在最高处（调试 / 分步）
PLACE             # 已经夹着料时，只做放料那半段
GRAB STOP         # 中止（同时停车，不下降）
```

和 `TRACK 3` 是同一条跟随链，差别只在锁定之后：`TRACK` 继续跟，`GRAB` 收尾去取放。
触发取料要**同时**满足：

1. `locked == 1`（目标停稳、连续 3 帧在容差内，容差外 5 mm 的滞回不算解锁）；
2. 这一拍 `|dx| ≤ tol` 且 `|dy| ≤ tol`（锁定允许滞回，夹取不允许带偏差）。

满足后 `Align_TryCompleteForGrab()` 打印 `ALIGN done, e=(x,y) mm`、停车、`reason=DONE`，
`vehicle_app` 紧接着调 `Pick_GrabStart(place)`。超时（120 s）、丢目标（3 s）、被 `STOP` / 手动命令 / `E 0`
取消，一律 `GRAB aborted`，**不下降**。

放料半段直接复用 `CYC` 每轮的后 4 步（转 `-PK_CYC_ROT_DEG` → 松 `PK_GRIP_OPEN_DEG` →
载物台走 `PK_CYC_PLAT_MS` → 转回），参数只有一份。升降净值仍为 0：
`+(DOWN-APP)+APP -LIFTOFF -(UP-LIFTOFF) -(DOWN-UP) = 0`，结束时回到最高处、累计位置归零，
可以直接接下一轮 `GRAB` / `CYC` / `TRG`。

抓取/放置本身在 `pick_app.c`，不在对准模块里。起点必须是丝杆最高处
（`LIFT` 累计位置为 0，否则 `ERR GRAB lift not home` / `ERR PLACE lift not home`）。
动作和参数见 `Docs/机构指令_升降旋转舵机.md` §3.3。

> 为什么 GRAB 从"一次性对准"改成"追踪后取放"：物料被手碰一下、或台面轻微移动，
> 一次性 ALIGN 在判定 done 之后就不再管，夹爪下降时已经偏了；追踪版会一直跟到
> 目标真正停稳且误差在容差内才下降，取起来后顺手把料放到载物台，省掉一次手动 `CYC`。
> 物料静止时行为与旧版一致（都是等 3 帧稳定）。

## 9. 命令发了没反应怎么查

2026-11-16 实机遇到"发了 GRAB 没动作"，日志长这样：

```
ALIGN active=0 mode=track cls=3 tol=8 mm hit=3 locked=0
OK ALIGN cls=3 tol=8 mm ofs=(0,0) yaw=hold      ← 关键：GRAB 不会回这行
ALIGN e=(-3,-35) mm v=(0,-80) mm/s c=0
ALIGN done, e=(-1,1) mm
```

三条证据说明**实际执行的是 `ALIGN`，不是 `GRAB`**：

1. 新固件里 `GRAB 3` 的第一行是 `OK TRACK ...`（GRAB 内部走追踪律），`ALIGN 3` 才是 `OK ALIGN ...`；
2. `GRAB` 一定会紧跟一行 `GRAB armed, will pick and place when locked`（旧固件是 `... after align`），日志里没有；
3. 结束状态是 `mode=align`（GRAB 会显示 `mode=track`）。

`ALIGN` 只做一次性对准，**永远不会启动夹爪**；会取放料的只有 `GRAB`。排查顺序：

| # | 检查 | 判据 |
|---|---|---|
| 1 | `VER` | 必须是当前那版，例如 `FW 2026-11-16 cls-hint2`；不对就是烧了旧 HEX |
| 2 | 烧写文件 | `MDK-ARM/Vehicle_Integrated/Vehicle_Integrated.hex`（16:51 那版，123441 字节） |
| 3 | 实际发出的字符串 | 串口助手/宏按钮/串口屏里存的可能还是 `ALIGN 3`，先看它到底发了什么 |
| 4 | `GRAB 3` 首行 | 应为 `OK TRACK ...`；若是 `OK ALIGN ...` 说明板上还是旧固件 |
| 5 | 锁定后 | 应依次出现 `TRACK locked` → `ALIGN done, e=...` → `GRAB start, pick then place`（或 `GRAB start, open-down-close-up` = 带了 `HOLD`） |
| 6 | **下行线** | 树莓派跑 `align_send.py --link-test`，STM32 发 `HINT 3`：树莓派应打印 `RX b'$C,3\n'`。一条都没有 = PC10→Pi RX 没接 / 接反 / 不是同一个串口 |

其它可能挡住夹取的回显（都在 `GRAB` 启动或收尾时出现，不会静默）：

- `ERR GRAB lift not home, send LIFT HOME`：丝杆不在最高处；
- `ERR not enabled`：没 `E 1`；
- `GRAB aborted: align failed`：追踪超时（120 s）或丢目标（3 s）没锁定；
- `ERR GRAB busy, send STOP` / `ERR sequence running`：机构序列还在跑。

## 验证状态

| 项 | 编码 | 编译 | 实板 |
|---|---|---|---|
| `App/Inc/align_config.h`、`App/Src/align_app.c` | 完成（2026-11-16 连续速度环 + `TRACK` 速度前馈/锁定 + 阶跃剔除/解锁滞回） | **0 Error / 3 Warning**（2026-11-16 全量重建 `-r`，3 条均为 ST HAL `Banks`） | 未上板 |
| `vehicle_app.c` 的 `V` / `ALIGN` / `GRAB` / `TRACK` / `PLACE`、`Vision_Poll()`、`Align_Poll()` | 完成（GRAB 改为"追踪锁定 → 取料 → 放到载物台"，带 `HOLD`） | **0 Error / 3 Warning**（同上） | 未上板 |
| `.uvprojx` 已加 `align_app.c`（App 组 1 处） | 完成 | — | — |
| `TRACK` 追踪（前馈 + 锁定） | 完成，新增参数见 §3 | **0 Error / 3 Warning**；`Program Size: Code=38016 RO=5804 RW=1368 ZI=3152` | **实车已验证收敛**（2026-11-16：从 33 mm 收敛到 1~3 mm、稳定 locked）；阶跃剔除/滞回**未上板复测** |
| 取料 + 放料（`GRAB` 默认放料、`HOLD`、`PLACE`） | 完成 | **0 Error / 3 Warning**（同上） | **未上板**：转台角、载物台走格、放料高度都没验证过 |
| 蓝牙换色（`$C,<cls>` 下行 + `GRAB BLUE` 名字） | 完成（`send_class_hint()` + `poll_class_hint()`） | **0 Error / 3 Warning**（同上） | **PC 单测通过**（假串口验证 cls 3→1→3 切换、`$C` 分帧解析）；实车未联调 |
| `Tools/vision/align_send.py`（检测引擎/单相机旧入口） | 完成 | — | **PC 实跑**；主入口已改为 `main.py` |
| 相机安装角补偿（`align_send --rot-deg` 默认 90°、`vision_node --mount-deg`） | 完成（2026-11-16 由 TRACK 绕圈日志 + 主机仿真定位） | — | **实车跟随已收敛**（2026-11-16 日志）；换相机后要按 §4.2 复核 |

**注意**：`Docs/交接_收成一个工程.md` 第 3 节写「谁把 `Vision_Poll()` 加回
`VehicleApp_Begin/Poll`，谁就把 STEP 引脚抢走」。经核对 `board_vision.c`，
`Vision_Poll()` 只调 `Serial_ReadByte(SERIAL_RPI)` / `Serial_Dropped()` / `Board_NowMs()`，
**不碰任何 GPIO 或定时器**，与 `board_step6` 无引脚冲突。那句警告对
`Board_Init()`（把 TIM2/3/4 配成编码器）和 `ChassisOdom_Update()` 成立，
对 `Vision_Poll()` 不成立。USART3 在 `Board_InitStep6()` → `Board_CommInit()` 里已经初始化。
