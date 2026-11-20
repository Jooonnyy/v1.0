#ifndef ALIGN_CONFIG_H
#define ALIGN_CONFIG_H

/* 视觉对准（ALIGN）与追踪（TRACK）参数 —— 连续小幅度调整版。
 *
 * 干什么：$V 目标流给的 x_mm / y_mm 以夹爪正下方为原点（本车夹爪与相机光轴
 * 同轴），所以"对准"就是把这两个数压到容差内。
 *
 * 两种模式：
 *   ALIGN  一次性对准：进容差并停稳 AL_SETTLE_HITS 帧就报 done 并停车。
 *   TRACK  连续追踪：目标动就一直跟，永远不报 done；目标停稳并进容差才报
 *          `TRACK locked`。跟的是"目标速度前馈 + 误差比例"，见下面的 AL_TRACK_FF_*。
 *
 * 控制方式：每 AL_SERVO_PERIOD_MS 读一次快照，按误差给四个轮子一个速度
 * （vx 前后、vy 左右），速度与误差成正比并限幅。已经在容差内的那一轴速度为 0，
 * 不会被最低速再推出去。同一帧最多再推 AL_FRAME_HOLD_MS，超时就停，等下一帧，
 * 避免视觉卡一下车还按旧速度冲过容差。
 *
 * 这是闭环：步进电机本身没有位置反馈，位置全靠视觉每帧重测。
 * 像素→毫米的尺度差一点只影响每步多大，不影响最终收敛。
 *
 * 单位后缀按 AGENTS.md 第 3 节。改参数只改本文件。 */

/* ---- 判到位 ---- */
#define AL_TOL_DEFAULT_MM              8
#define AL_TOL_MIN_MM                  2
#define AL_TOL_MAX_MM                 50
/* 连续这么多个**不同 seq** 的帧都在容差内、且车已经停住，才报 done。
 * 单帧噪声不能算到位。 */
#define AL_SETTLE_HITS                 3U

/* ---- 调节节拍 ---- */
/* 每这么久读一次快照并更新一次四轮速度。视觉侧 10 Hz，这里同频。 */
#define AL_SERVO_PERIOD_MS           100U
/* 同一帧最多再按它推这么久。要比视觉周期长一点（相位对不齐时还能接上下一帧），
 * 必须短于 AL_STALE_MS，否则"帧还算新鲜"期间车已经盲走出去了。 */
#define AL_FRAME_HOLD_MS             150U
/* 误差已经小到最低速会冲过零点时，只按这个时长点一下，然后停下来等新帧。 */
#define AL_NUDGE_MS                   60U

/* ---- 速度环 ---- */
/* 比例增益，单位 (mm/s) / mm，用千倍整数表示：2500 = 2.5。
 * 误差 20 mm → 50 mm/s；误差 8 mm（刚好在容差外）→ 20 mm/s。 */
#define AL_SERVO_GAIN_MILLI         2500
/* 近目标（|e| ≤ AL_SERVO_NEAR_MM）用更小的增益，避免最小速度+延迟造成极限环。 */
#define AL_SERVO_NEAR_MM              20
#define AL_SERVO_GAIN_NEAR_MILLI     800
/* 速度上限，也是四轮合成后单轮线速度的上限（斜向不会比轴向更快）。 */
#define AL_SERVO_MAX_MM_PER_S         80
/* 速度下限：算出来的速度低于它、且这一拍不会冲过零点时，才抬到它。 */
#define AL_SERVO_MIN_MM_PER_S          6
/* 轮子线速度 → STEP 频率。CS_STEPS_PER_REV(3200) / 周长(251.328 mm) = 12.732，
 * 用千倍整数 12732 避免浮点。换轮径/细分时必须同步改这个数。 */
#define AL_HZ_PER_MM_PER_S_MILLI     12732
/* 频率变化小于这个值就不重发。Step6_SetHz() 每次都会停 PWM、重配 PSC/ARR，
 * 每拍都调会把脉冲打断，车一顿一顿。 */
#define AL_HZ_UPDATE_MIN              12

/* ---- 航向保持 ---- */
/* 麦轮斜向走会带一点自转。有新鲜陀螺帧时，把启动那一刻的 yaw 当作目标，
 * 用左右差速拉回来。没有帧就只做平移，不因此拒绝对准。
 * 符号与 chassis_motion 相同：修正量为正 = 想让车逆时针（B/D 加、A/C 减）。 */
#define AL_YAW_HOLD_ENABLE             1
#define AL_YAW_DEADBAND_MDEG         500    /* 0.5 度以内不修，避免左右抖 */
#define AL_YAW_KP_HZ_PER_DEG          60    /* 每 1 度误差加多少 Hz 差速 */
#define AL_YAW_KD_HZ_PER_DPS           6    /* 每 1 度/秒角速度加多少 Hz 阻尼 */
#define AL_YAW_MAX_HZ                120    /* 差速上限，不能盖过平移 */

/* ---- 追踪模式（TRACK）---- */

/* 目标速度前馈增益，千倍整数：700 = 把测到的目标速度按 70% 加进轮速。
 * 太小追不上移动目标（误差会一直涨），太大在视觉延迟下容易过冲。
 * 0 = 关闭前馈，退化成纯比例跟随。 */
#define AL_TRACK_FF_MILLI             700
/* 目标速度估计的上限，单位 mm/s。目标跑得比这还快时按上限算，避免脏数据把车甩出去。 */
#define AL_TRACK_FF_MAX_MM_PER_S      300
/* 目标速度超过它就判为"跳变"（手碰一下、遮挡后重新捕获），这一帧不更新前馈。
 * 否则一帧挪 14 mm 会被算成 140 mm/s，前馈把车甩出去一大步。
 * 车身本身只有 AL_SERVO_MAX_MM_PER_S（80 mm/s），更快的目标也追不上，丢前馈不影响跟随。 */
#define AL_TRACK_FF_REJECT_MM_PER_S   120
/* 解锁滞回，单位 mm：锁定后误差超过 tol + 这个值才报 unlocked，避免在容差边界抖。
 * 注意夹取触发仍然要求误差真正回到 tol 以内（见 Align_TryCompleteForGrab）。 */
#define AL_TRACK_UNLOCK_MARGIN_MM       5
/* 前馈死区，单位 mm/s。低于它当成静止，避免把视觉抖动放大成轮子抖。 */
#define AL_TRACK_FF_MIN_MM_PER_S        8
/* "目标停住了"的门限，单位 mm/s。锁定时要求两轴目标速度都低于它。 */
#define AL_TRACK_FF_STILL_MM_PER_S     15
/* 前馈超过它时，这一帧按完整 AL_FRAME_HOLD_MS 持续走；否则靠近零点只点动一下。 */
#define AL_TRACK_FF_HOLD_MM_PER_S      20
/* 速度差分的有效时间窗，单位 ms。太短（同帧抖动）丢掉，太长（中途断帧）清速度估计。 */
#define AL_TRACK_VEL_DT_MIN_MS         20U
#define AL_TRACK_VEL_DT_MAX_MS        500U

/* 追踪最长时间，单位 ms。0 = 不限。防止没人值守时车一直跟着跑。 */
#define AL_TRACK_TIMEOUT_MS        120000U
/* 追踪时连续这么久没有合格帧才中止。比 ALIGN 宽松：物料被手挡一下不该直接退出。 */
#define AL_TRACK_LOST_MS             3000U

/* ---- 超时与丢帧 ---- */
#define AL_TIMEOUT_MS              30000U   /* 一次对准最多允许多久 */
#define AL_STALE_MS                  300U   /* 帧龄超过它就算"当前没目标"，立即停车 */
#define AL_LOST_MS                  2000U   /* 连续这么久没目标才中止 */
/* 视觉侧自己填的 age_ms 超过它，说明拍照到发出已经太旧，不当成可用目标。
 * 单应分支一帧可能算 300~500 ms，所以默认放宽到 500；链路换快了可以调小。 */
#define AL_VISION_AGE_MS             500U
/* 置信度低于它的帧不当目标（仍算"当前没有可用目标"）。 */
#define AL_CONF_MIN_PERMILLE         400U

/* ---- 机械偏置 ---- */
/* 夹爪圆心相对相机光轴（= 车体原点）的偏差，单位 mm。
 *
 * 本车摄像头与夹爪在同一个 xy 坐标上，只是高度不同（2026-11-11 确认）。
 * 两个都应当是 0。留着只为吸收装配公差：实测咬偏几毫米就填几毫米
 * （x 前正、y 左正）。
 *
 * 算法：dx = x_mm - offset_x，dy = y_mm - offset_y。 */
#define AL_GRIPPER_OFFSET_X_MM         0
#define AL_GRIPPER_OFFSET_Y_MM         0

#if (AL_TOL_DEFAULT_MM < AL_TOL_MIN_MM || AL_TOL_DEFAULT_MM > AL_TOL_MAX_MM)
#error AL_TOL_DEFAULT_MM must be within AL_TOL_MIN_MM..AL_TOL_MAX_MM
#endif
#if (AL_TOL_MIN_MM < 1 || AL_TOL_MAX_MM > 200)
#error AL_TOL range must be 1..200
#endif
#if (AL_SETTLE_HITS < 2U || AL_SETTLE_HITS > 10U)
#error AL_SETTLE_HITS must be 2..10
#endif
#if (AL_SERVO_PERIOD_MS < 20U || AL_SERVO_PERIOD_MS > 500U)
#error AL_SERVO_PERIOD_MS must be 20..500
#endif
#if (AL_FRAME_HOLD_MS < AL_SERVO_PERIOD_MS || AL_FRAME_HOLD_MS > AL_STALE_MS)
#error AL_FRAME_HOLD_MS must be AL_SERVO_PERIOD_MS..AL_STALE_MS
#endif
#if (AL_NUDGE_MS < 40U || AL_NUDGE_MS > AL_FRAME_HOLD_MS)
#error AL_NUDGE_MS must be 40..AL_FRAME_HOLD_MS
#endif
#if (AL_SERVO_GAIN_MILLI < 100 || AL_SERVO_GAIN_MILLI > 20000)
#error AL_SERVO_GAIN_MILLI must be 100..20000 (0.1..20 (mm/s)/mm)
#endif
#if (AL_SERVO_GAIN_NEAR_MILLI < 100 || AL_SERVO_GAIN_NEAR_MILLI > AL_SERVO_GAIN_MILLI)
#error AL_SERVO_GAIN_NEAR_MILLI must be 100..AL_SERVO_GAIN_MILLI
#endif
#if (AL_SERVO_MAX_MM_PER_S < 10 || AL_SERVO_MAX_MM_PER_S > 400)
#error AL_SERVO_MAX_MM_PER_S must be 10..400
#endif
#if (AL_SERVO_MIN_MM_PER_S < 5 || AL_SERVO_MIN_MM_PER_S > AL_SERVO_MAX_MM_PER_S)
#error AL_SERVO_MIN_MM_PER_S must be 5..AL_SERVO_MAX_MM_PER_S
#endif
/* 点动那一下的位移必须小于最小容差，否则最低速会把车推出容差带。 */
#if ((AL_SERVO_MIN_MM_PER_S * AL_NUDGE_MS) > (AL_TOL_MIN_MM * 1000))
#error AL_NUDGE_MS too long for AL_SERVO_MIN_MM_PER_S vs AL_TOL_MIN_MM
#endif
#if (AL_HZ_PER_MM_PER_S_MILLI < 1000 || AL_HZ_PER_MM_PER_S_MILLI > 100000)
#error AL_HZ_PER_MM_PER_S_MILLI must be 1000..100000
#endif
#if (AL_HZ_UPDATE_MIN < 0 || AL_HZ_UPDATE_MIN > 80)
#error AL_HZ_UPDATE_MIN must be 0..80
#endif
#if (AL_YAW_HOLD_ENABLE != 0 && AL_YAW_HOLD_ENABLE != 1)
#error AL_YAW_HOLD_ENABLE must be 0 or 1
#endif
#if (AL_YAW_DEADBAND_MDEG < 0 || AL_YAW_DEADBAND_MDEG > 5000)
#error AL_YAW_DEADBAND_MDEG must be 0..5000
#endif
#if (AL_YAW_KP_HZ_PER_DEG < 0 || AL_YAW_KP_HZ_PER_DEG > 300)
#error AL_YAW_KP_HZ_PER_DEG must be 0..300
#endif
#if (AL_YAW_KD_HZ_PER_DPS < 0 || AL_YAW_KD_HZ_PER_DPS > 100)
#error AL_YAW_KD_HZ_PER_DPS must be 0..100
#endif
#if (AL_YAW_MAX_HZ < 0 || AL_YAW_MAX_HZ > 1000)
#error AL_YAW_MAX_HZ must be 0..1000
#endif
#if (AL_TIMEOUT_MS < 3000U || AL_TIMEOUT_MS > 120000U)
#error AL_TIMEOUT_MS must be 3000..120000
#endif
#if (AL_STALE_MS < 50U || AL_STALE_MS > 2000U)
#error AL_STALE_MS must be 50..2000
#endif
#if (AL_LOST_MS <= AL_STALE_MS)
#error AL_LOST_MS must exceed AL_STALE_MS
#endif
#if (AL_VISION_AGE_MS < 50U || AL_VISION_AGE_MS > 1000U)
#error AL_VISION_AGE_MS must be 50..1000
#endif
#if (AL_CONF_MIN_PERMILLE > 1000U)
#error AL_CONF_MIN_PERMILLE must be 0..1000
#endif
#if (AL_GRIPPER_OFFSET_X_MM < -200 || AL_GRIPPER_OFFSET_X_MM > 200)
#error AL_GRIPPER_OFFSET_X_MM must be -200..200
#endif
#if (AL_GRIPPER_OFFSET_Y_MM < -200 || AL_GRIPPER_OFFSET_Y_MM > 200)
#error AL_GRIPPER_OFFSET_Y_MM must be -200..200
#endif
#if (AL_TRACK_FF_MILLI < 0 || AL_TRACK_FF_MILLI > 1500)
#error AL_TRACK_FF_MILLI must be 0..1500
#endif
#if (AL_TRACK_FF_MAX_MM_PER_S < 0 || AL_TRACK_FF_MAX_MM_PER_S > 400)
#error AL_TRACK_FF_MAX_MM_PER_S must be 0..400
#endif
#if (AL_TRACK_FF_REJECT_MM_PER_S < AL_TRACK_FF_MIN_MM_PER_S || AL_TRACK_FF_REJECT_MM_PER_S > AL_TRACK_FF_MAX_MM_PER_S)
#error AL_TRACK_FF_REJECT_MM_PER_S must be AL_TRACK_FF_MIN..AL_TRACK_FF_MAX
#endif
#if (AL_TRACK_UNLOCK_MARGIN_MM < 0 || AL_TRACK_UNLOCK_MARGIN_MM > 50)
#error AL_TRACK_UNLOCK_MARGIN_MM must be 0..50
#endif
#if (AL_TRACK_FF_MIN_MM_PER_S < 0 || AL_TRACK_FF_MIN_MM_PER_S > AL_TRACK_FF_MAX_MM_PER_S)
#error AL_TRACK_FF_MIN_MM_PER_S must be 0..AL_TRACK_FF_MAX_MM_PER_S
#endif
#if (AL_TRACK_FF_STILL_MM_PER_S < AL_TRACK_FF_MIN_MM_PER_S || AL_TRACK_FF_STILL_MM_PER_S > AL_TRACK_FF_MAX_MM_PER_S)
#error AL_TRACK_FF_STILL_MM_PER_S must be AL_TRACK_FF_MIN..AL_TRACK_FF_MAX
#endif
#if (AL_TRACK_FF_HOLD_MM_PER_S < AL_TRACK_FF_MIN_MM_PER_S || AL_TRACK_FF_HOLD_MM_PER_S > AL_TRACK_FF_MAX_MM_PER_S)
#error AL_TRACK_FF_HOLD_MM_PER_S must be AL_TRACK_FF_MIN..AL_TRACK_FF_MAX
#endif
#if (AL_TRACK_VEL_DT_MIN_MS < 5U || AL_TRACK_VEL_DT_MIN_MS > 100U)
#error AL_TRACK_VEL_DT_MIN_MS must be 5..100
#endif
#if (AL_TRACK_VEL_DT_MAX_MS < AL_TRACK_VEL_DT_MIN_MS || AL_TRACK_VEL_DT_MAX_MS > 2000U)
#error AL_TRACK_VEL_DT_MAX_MS must be AL_TRACK_VEL_DT_MIN..2000
#endif
#if (AL_TRACK_TIMEOUT_MS != 0U && (AL_TRACK_TIMEOUT_MS < 5000U || AL_TRACK_TIMEOUT_MS > 600000U))
#error AL_TRACK_TIMEOUT_MS must be 0 or 5000..600000
#endif
#if (AL_TRACK_LOST_MS <= AL_STALE_MS || AL_TRACK_LOST_MS > 30000U)
#error AL_TRACK_LOST_MS must be AL_STALE_MS+1..30000
#endif

#endif
