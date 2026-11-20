# 综合任务 stp1–stp7（drop16）

`TASK <code>` 先解析 15 位任务码，存两批**颜色/环号**；之后 `TASK` 跑全流程，
`STP 1..7` 只跑某一段。路线在 `App/Src/chassis_task.c`，环间距/动作参数在 `chassis_config.h` / `pick_config.h`。

## 场地约定

- 粗加工区、暂存区都是 **3 个环，车停在 2 号环前**，1/2/3 从左到右，间距 **150 mm**；
- 环号从 `TASK <code>` 的第 2/4 组来（1..3）；
- 颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6；
- 车到站后 `TURN_ABS` 的朝向：
  - 物料区 270°
  - 粗加工区 90°
  - 暂存区 0°

## 流程

| 段 | 动作 |
|---|---|
| stp1 | `RIGHT 150` → `FWD 1050` → `SCAN`（发 `@SCAN`，**等 `TASK <code>` 回来，最多 10 s**）→ `BACK 950` → `RIGHT 900` → `TURN_ABS 270` |
| stp2 | 先 `ROT_HOME` 回取料侧，再按颜色 `GRAB` ×3（没目标 0.5 s 重试，最多 20 s）；抓完 `ROT_CAROUSEL` 转到载物盘侧 |
| stp3 | `LEFT 1850` → `TURN_ABS 90`（机械臂在载物盘上方） |
| stp4 | 一个封装步 `TASK_STP4`：内部循环 3 个环：`ROT_HOME` → 移环 → `DROP` → `PLT` |
| stp5 | 粗加工区：第 1 批按环号 `GRAB_GND` ×3（地面 41000 对准→取到车上）→ `CENTER` 回 2 号环 |
| stp6 | `LEFT 900` → `BACK 900` → `TURN_ABS 0` → 第 1 批 `DROP` ×3（每次后 PLT）→ `CENTER` |
| stp7 | `LEFT 900` → `BACK 950` → `TURN_ABS 270` → 第 2 批 `GRAB` ×3 → `LEFT 1850` → `TURN_ABS 90` → 第 2 批 `DROP` ×3 → `GRAB_GND` ×3 → `CENTER` → `LEFT 900` → `BACK 900` → `TURN_ABS 0` → 第 2 批 `DROP` ×3 → `CENTER` → `LEFT 1950` → `BACK 1050` |

`value=0..2` 是第一批第 1/2/3 件，`3..5` 是第二批；具体颜色/环号由任务码决定，路线里不写死。

## 动作含义

| 步骤 | 实际动作 |
|---|---|
| `GRAB` | 视觉**跟踪** `$C,<颜色>`，锁定且进容差后 `Pick_GrabStart(1)`：台面 8000 取料 → 放载物盘 |
| `GRAB_GND` | 先平移到目标环 → 视觉对准颜色 → `Pick_GroundGrabStart(1)`：地面 41000 取料 → 放载物盘 |
| `DROP` | 先平移到目标环 → `Pick_DropStart(ring)`：空爪粗对/精对 → 载物盘取料 → 放进圆环 |
| `CENTER` | 平移回 2 号环前（±150 mm 的整数倍） |

移环用 `ChassisMotion_TranslateAbs` 横移，`STP_RING_STEP_MM=150`。

## stp1 → stp2 交接

1. stp1 发 `@SCAN` 给树莓派；
2. 树莓派扫码后发 `TASK <code>`；
3. 固件 `ChassisTask_SetPlan()` 解析两组颜色/环号，`plan_valid=1`，并回 `@PLAN <code>` 给树莓派；
4. `TASK_SCAN` 等到 `plan_valid` 才继续走位；10 s 没收到就 `TASK aborted: scan timeout`；
5. stp2 的 `GRAB 0/1/2` 按第 1 组颜色顺序发 `$C,<颜色>` + `@DETECT 1`，树莓派按这个颜色识别。

## 不接视觉干跑 `DRY`

```text
E 1
DRY
```

- 只跳过 `SCAN` 等待和视觉对准；**机械臂、转台、载物盘照常动作**；
- `GRAB`：照常降 8000 → 夹紧 → 转到载物盘放开 → PLT → 转回（抓空）；
- `GRAB_GND`：移到目标环 → 照常降 41000 → 夹紧 → 转到载物盘放开 → PLT → 转回（抓空）；
- `DROP`：从 0° 开始 → 粗对 → 降 41000 → 精对 → 回最高 → 转 -132° 取料 → 转回 0° 降 40000 放料 → 回最高；结束后停在 0°，stp4 直接移到下一个环重复；
- `PLT` 照常走一格；`ROT_HOME`、底盘走位、`CENTER` 全部保留；
- 跑之前确认台面/环位/升降下方没有东西，爪会真的下去；
- 回显 `TASK DRY step n/60`、`DROP DRY ring n`，最后一次 `TASK done`。

## 任务码

```text
TASK 156+123+516+231
```

- 第 1 组 `156`：第一批颜色红/黑/浅蓝；
- 第 2 组 `123`：第一批环号 1/2/3；
- 第 3 组 `516`：第二批颜色黑/红/浅蓝；
- 第 4 组 `231`：第二批环号 2/3/1。

只上屏、不启动路线；不扫码直接 `TASK` 会用默认 `1/2/3` 颜色和环号（调试用）。

## 测试

```text
E 1
TASK 156+123+516+231     # 先存任务码
TASK                     # 全流程
STP 4                    # 只跑粗加工放环（车先摆到 2 号环前，朝向按该段起点）
```

`STP` 会把当前车头当 0°，所以移动段（stp1/3/6/7）建议用完整 `TASK`；`STP 2/4/5` 这种原地/短移动作可以直接测。

## stp4 → stp5 交接

正常完整 `TASK` 里**不需要手动动旋转台**：

- `DROP` 最后一步是 `ROT -132°`，转回取料侧、升降回最高；
- `GRAB_GND` 从取料侧开始：张开 → 降 41000 → 夹紧 → 升到 2000 → 转放料侧放载物盘 → 转回取料侧；
- 所以 stp4 第三个 DROP 结束后，转台已经在取料侧，stp5 直接接着跑。

只有两种情况要手动处理：

1. **DROP 被 STOP/超时中止**：自动回位只保证升降回最高，转台可能停在放料侧；先 `ROT -132` 转回取料侧，再继续；
2. **分开跑 `STP 4` 再 `STP 5`**：`STP 5` 会把 `cur_ring` 重置成 2 号环，而 stp4 结束时车实际在最后一件的环位；要么先手动把车摆回 2 号环前，要么直接跑完整 `TASK`。

## stp2 初始转台位置

上电默认转台在载物盘侧（`-PK_CYC_ROT_DEG = -132°`），stp2 第一步 `ROT_HOME` 会把它转回取料侧 0°：
`ROT_HOME` 读 `Pick_RotGetDeg()`，发 `ROT -deg`，所以 -132° → 0°。
之后 `GRAB` 才能按“降 8000 取料 → 转 -132° 放载物盘 → 转回”的顺序动作。
用 `ROT ?` 可以看当前累计角度。

## 参数

- `STP_GRAB_TARGET_WAIT_MS = 20000`：抓取时等物料进视野的总超时，每次没目标停 0.5 s 重试；


- `STP_RING_STEP_MM = 150`：环间距；
- `PK_DROP_ALIGN_COARSE_MM=15` / `PK_DROP_ALIGN_TOL_MM=2`：DROP 粗/精对；
- `PK_GRIP_OPEN_DEG=30` / `PK_GRIP_CLOSE_DEG=180`；
- `CS_MAP_*`：各段距离（沿用旧路线）。

## 注意

- `GRAB_GND` 在**最高处**先用颜色对准，再下降 41000 取料；如果地面料在画面里太小/对不准，需要把对准改到 41000 高度做（视觉 px/mm 也要换）。
- `DROP` 的环号来自任务码；如果现场环号不是 1..3，`TASK <code>` 会回 `ERR TASK RING 1..3`。
