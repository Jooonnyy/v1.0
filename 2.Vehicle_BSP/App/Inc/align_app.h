#ifndef ALIGN_APP_H
#define ALIGN_APP_H
#include <stdint.h>

/* 视觉对准 / 追踪（App）：用 $V 目标流的 x_mm / y_mm 驱动底盘做四轮微调，
 * 把目标圆心对到夹爪正下方（= 相机光轴 = 车体原点）。
 *
 * 两种模式：
 *   ALIGN（Align_Start）：一次性对准，进容差并停稳后报 done、停车。
 *   TRACK（Align_StartEx(..., 1)）：连续追踪，目标动就一直跟，不报 done；
 *         目标停稳且进容差时报一次 `TRACK locked`，之后目标再动继续跟。
 *
 * 分层：读快照是 BSP（board_vision），脉冲是 board_step6。
 * 这里只回答"该往哪边挪、挪多快、什么时候算到位/锁定"。
 * 不走 chassis_motion 的"走一段再停"——那条路每步都要停稳再量，微调太慢，
 * 而且和对准抢同一组 STEP 定时器。
 *
 * 全部非阻塞：Align_Poll() 由主循环每轮调用，等待都用 Board_NowMs() 时间戳。
 *
 * 前提：
 *   - 底盘已使能（E 1，内部自查 Step6_IsEnabled()）；
 *   - 树莓派在往 USART3 发 $V 帧，且 cls 与请求的一致；
 *   - 夹爪圆心与相机光轴重合（机械前提；装配公差填 AL_GRIPPER_OFFSET_*）。
 *   - 对准期间不要同时跑 CYC / TRG / GRAB，下降时车必须是停的。 */

/* 中止原因，给回显和日志用。 */
typedef enum {
  ALIGN_REASON_NONE = 0,
  ALIGN_REASON_DONE,        /* ALIGN：连续多帧进容差；TRACK：不产生这个原因 */
  ALIGN_REASON_TIMEOUT,
  ALIGN_REASON_NO_TARGET,   /* ALIGN 连续 AL_LOST_MS / TRACK 连续 AL_TRACK_LOST_MS 没有合格帧 */
  ALIGN_REASON_ITERATIONS,  /* 保留：旧的"走满步数"原因，连续速度环不再使用 */
  ALIGN_REASON_MOVE_FAIL,   /* 保留：旧的 Translate 启动失败，连续速度环不再使用 */
  ALIGN_REASON_DISABLED,    /* 步进未使能或中途被 E 0 */
  ALIGN_REASON_CANCELLED    /* 被 ALIGN/TRACK STOP / 手动命令 / STOP / 其它运动取消 */
} AlignReason;

/* 清状态。第一次 Poll 之前必须调用。 */
void Align_Init(void);

/* 开始一次性对准。cls 与 $V 帧的 cls 一致：1..6 颜色、10 载物盘、11 色环。
 * tol_mm 传 0 用 AL_TOL_DEFAULT_MM。成功返回 1，并回一行 `OK ALIGN ...`。 */
uint8_t Align_Start(uint8_t cls, int16_t tol_mm);

/* 开始对准或追踪。track = 0 等价于 Align_Start；track = 1 是连续追踪，
 * 目标动就跟着走、不报 done。成功返回 1，回显 `OK ALIGN ...` / `OK TRACK ...`。 */
uint8_t Align_StartEx(uint8_t cls, int16_t tol_mm, uint8_t track);

/* 同 ALIGN，但允许在 TASK 内部跑（Align_Poll 不因任务 active 取消）。 */
uint8_t Align_StartExTask(uint8_t cls, int16_t tol_mm, uint8_t track);

/* 同上，额外带一个目标序号（圆环 1/2/3）：下行写成 `$C,<cls>,<aux>`。
 * DROP 用它告诉树莓派"对第几个环"；aux=0 时与 Align_StartEx 完全一样。 */
uint8_t Align_StartExAux(uint8_t cls, int16_t tol_mm, uint8_t track, uint8_t aux);

/* 同上，再带一个对准阶段：0 = 顶部粗对，1 = 底部精对（下行 `$C,cls,aux,rot,stage`）。 */
uint8_t Align_StartExAuxStage(uint8_t cls, int16_t tol_mm, uint8_t track,
                              uint8_t aux, uint8_t stage);
uint8_t Align_StartExAuxTask(uint8_t cls, int16_t tol_mm, uint8_t track,
                             uint8_t aux, uint8_t stage);

/* 中止（不发回显，由调用方决定说什么）。 */
void Align_Cancel(void);

uint8_t Align_IsActive(void);

/* 当前是不是追踪模式（1 = TRACK，0 = ALIGN / 未运行）。状态回显用。 */
uint8_t Align_IsTracking(void);

/* 最近一次结束的原因（Align_Cancel 不清它，方便事后看）。 */
AlignReason Align_LastReason(void);

/* 主循环轮询。每 AL_SERVO_PERIOD_MS 更新一次四轮速度。 */
void Align_Poll(void);

/* 打印当前状态（ALIGN ? 用）。输出走 ChassisIo_Reply。 */
void Align_PrintStatus(void);

/* 给 GRAB 用：追踪已锁定、且误差确实回到容差内时，按"对准成功"收尾并返回 1，
 * 让上层紧接着启动下降夹取；不满足条件返回 0，继续跟。
 * 收尾会停车、把 reason 置为 ALIGN_REASON_DONE，并回一行 `ALIGN done, e=...`。 */
uint8_t Align_TryCompleteForGrab(void);

/* 把类别下行给树莓派（`$C,<cls>`），不启动任何运动。`HINT <cls>` 命令用它单独测链路。 */
void Align_SendClassHint(uint8_t cls);

#endif
