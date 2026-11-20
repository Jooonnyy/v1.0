#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 新底板底盘开环参数。2026-11-09 从 Tests/Src/chassis_step_smoke.c 原样搬出，
 * 数值没改。速度只决定"跑多久"，不进入步数算式。 */

#define CS_CMD_LEN                  40U
#define CS_LED_HEARTBEAT_MS        500U
#define CS_LINE_IDLE_MS             50U   /* 蓝牙口空闲断帧 */
#define CS_JOG_TIMEOUT_MS        30000U
#define CS_JOG_DEF_PERMILLE        300
#define CS_PERMILLE_LIMIT         1000

/* 16 细分 × 200 步/转；**轮径 80 mm**（2026-11-11 换轮，π=3.1416 → 251.328 mm），
 * 周长用 π=3.1416。自旋半径 L=(lx+ly)，与轮径无关。
 * 2026-11-08 开环目测 A 90 ≈ 100°，225×90/100 后现场微调到 207。 */
#define CS_STEPS_PER_REV             3200U
#define CS_WHEEL_CIRC_UM           251328U
#define CS_SPIN_L_MM                  207U
#define CS_TASK_FWD_MM_PER_S          700U   /* 8914 Hz, 0.70 m/s */
#define CS_TASK_STRAFE_MM_PER_S       400U   /* 5093 Hz, 0.40 m/s（放慢换直线度） */
#define CS_TASK_SPIN_MM_PER_S         160U   /* 2037 Hz, 轮缘 0.16 m/s → 44.3 °/s */
/* 整边横移的判定阈值：>= 这个距离就不横移，改成"转 90° → 直行 → 转回来"。
 * 横移靠辊子侧向摩擦、又长又容易斜；直行是轮子纯滚动，几乎不打滑。
 * 小于这个值的"半个边"仍然直接横移。 */
#define CS_TASK_EDGE_MIN_MM          1200U

/* 平滑加减速：从 CS_MOVE_RAMP_PERMILLE 起，分 STEPS 档在 RAMP_UP_MS 内爬到全速，
 * 收尾再按同样档数对称降回来。一次跳变会被麦轮放大成"甩尾"，分档能明显减轻偏航。
 * 判停按每档实际频率积分，加减速不改变总步数。
 *
 * 2026-11-11 起步再放缓（用户实测"航向保持已经能走直线，只剩起步那一下甩角"）：
 * 原 400‰ / 200 ms / 8 档时，起步第一拍是 0 → 40% 巡航频率的**阶跃**
 * （横移 0→2173 Hz ≈ 0.16 m/s，直行 0→3802 Hz ≈ 0.28 m/s），这一下就是甩角的来源，
 * 后面的 S 曲线只决定 40%→100% 怎么爬，管不到它。改成 120‰ / 350 ms / 12 档后
 * 起步阶跃降到 ≈0.05 m/s。代价：每个动作的加减速各多 150 ms，整条约 +4~6 s。
 * 短段（< 约 300 mm）会按距离自动缩短斜坡，见 chassis_motion.c 的 ramp_ms。 */
#define CS_MOVE_RAMP_PERMILLE        120U
#define CS_MOVE_RAMP_UP_MS           350U
#define CS_MOVE_RAMP_UP_STEPS         12U
/* 短段的斜坡下限：150 mm 对齐这种段按距离缩短斜坡时，至少留这么久，
 * 再短就不是"限加加速度"而是一次跳变了。 */
#define CS_MOVE_RAMP_MIN_MS           80U
#define CS_MOVE_TIMEOUT_MS         120000U

// 2026-11-11 换 80 mm 轮后，把标度改对（CS_WHEEL_CIRC_UM 235620→251328），
// 下面两个数同时改回场地真实尺寸：之前被临时改成 1000 / 1750 是为了抵消
// "按 75 mm 算、实际 80 mm"带来的 6.7% 超走，标度修好后就不能再叠加这层补偿。
#define CS_MAP_ALIGN_MM          150
#define CS_MAP_QR_DROP_MM       1050
#define CS_MAP_SCAN_BACK_MM      950
#define CS_MAP_RAW_LEG_MM        900
#define CS_MAP_RAW_DROP_MM      1850
#define CS_MAP_LANE_CROSS_MM     900
#define CS_MAP_STORE_LEG_MM      900
#define CS_MAP_STORE_EXIT_MM     900
#define CS_MAP_RAW2_LEG_MM       950
#define CS_MAP_RETURN_EAST_MM   1950
#define CS_MAP_RETURN_NORTH_MM  1050
#define CS_MAP_SCAN_WAIT_MS    10000
/* 粗加工/暂存区：车停在 2 号环前，1/2/3 从左到右，间距 150 mm。 */
#define STP_RING_STEP_MM         150
/* 换环方向：1 = 环 1 在车的右侧（默认，按你现场“往远离圆环动”反过来了）；
 * 如果发现方向又反了，改成 0。 */
#define STP_RING_REVERSE         1U
/* 等物料盘转到位：每次没目标后 0.5 s 重试，总共最多等这么久。 */
#define STP_GRAB_TARGET_WAIT_MS 20000U
/* 工位等待改成"跑 N 轮 CYC"（2026-11-11）：到站后不再干等 3 s，而是把机构动作跑起来，
 * 跑满 N 轮才继续走位。轮数和动作表见 pick_config.h 的 PK_CYC_*。 */
#define CS_MAP_WORK_CYC_ROUNDS      3U

/* ---- HWT101 陀螺辅助（有新鲜帧才生效；没陀螺时行为与以前完全一样）----
 * 两个作用：
 *   1. 转身（A / TASK 的 TURN_ABS）走闭环：按陀螺 yaw 判到位，不再靠"步数 × 时间"估算；
 *   2. 直行 / 横移时做航向保持：按 yaw 误差给左右两侧差速，抵消走偏。
 * 正 yaw = 车体逆时针（`board_imu.h` 的 IMU_YAW_SIGN 已经统一）。 */
#define CS_GYRO_ENABLE                 1U   /* 1 = 自动使用陀螺，0 = 完全退回开环 */
#define CS_GYRO_TIMEOUT_MS           500U   /* 与 board_imu.h 的 IMU_TIMEOUT_MS 对齐 */
#define CS_GYRO_HOLD_UPDATE_MS        10U   /* 航向保持的修正周期 */
#define CS_GYRO_HOLD_DEADBAND_MDEG   300   /* 误差小于 0.3 度就不修，避免左右抖 */
#define CS_GYRO_HOLD_KP_HZ_PER_DEG   150   /* 每 1 度误差加多少 Hz 差速（1 Hz ≈ 0.02°/s 转向） */
#define CS_GYRO_HOLD_KI_HZ_PER_DEG_S 200   /* 积分：每 1 度误差每秒累积多少 Hz（消恒定扰动残差） */
/* 积分项自己的上限，**不能**跟 CS_GYRO_HOLD_MAX_PERMILLE 一样大。
 * 关系式：积分上限 / Kp 必须**大于**死区，否则积分饱和后的平衡偏角落在死区里，
 * 误差进不了"会累积"的区间，积分永远退不回来 → 车停在固定偏角上。见文件底部 #error。 */
#define CS_GYRO_HOLD_I_LIMIT_HZ      250   /* 250/150 ≈ 1.7 度，比死区 0.3 度大得多 */
#define CS_GYRO_HOLD_KD_HZ_PER_DPS    15   /* 每 1 度/秒角速度加多少 Hz 阻尼（0 = 关） */
#define CS_GYRO_HOLD_LAUNCH_MS       300U   /* 起步后这段时间内用更大的阻尼压甩尾 */
#define CS_GYRO_HOLD_KD_LAUNCH_HZ_PER_DPS 40  /* 起步阶段的阻尼增益（太大在 100 Hz 下会抖） */
#define CS_GYRO_HOLD_RATE_SHIFT        2U   /* 角速度低通：每拍向新值靠近 1/2^shift */
/* 横移"蟹行"校准偏置：麦轮横移时四轮侧向摩擦不均，车会斜着平移而车头不转（陀螺看不到）。
 * 给横移段的目标航向加一个固定偏置，让车主动"斜着走"，把斜掉的轨迹掰回直线。
 * 单位 mdeg：左移时 target += bias，右移时 target -= bias。0 = 关。 */
#define CS_GYRO_STRAFE_BIAS_MDEG       0
/* 每轴速度微调（1000 = 不变）：轮径/装配差异会让某个轮线速度差百分之几，
 * 横移就会斜。先用卷尺查机械，确实配不平时再动这四个数（±50 以内足够）。 */
#define CS_TRIM_A_PERMILLE           1000U
#define CS_TRIM_B_PERMILLE           1000U
#define CS_TRIM_C_PERMILLE           1000U
#define CS_TRIM_D_PERMILLE           1000U
#define CS_GYRO_HOLD_MAX_PERMILLE    400U   /* 修正量最多占基速的千分比 */
#define CS_GYRO_HOLD_MIN_STEP_HZ      20   /* 修正量变化小于这个值就不重发，减少定时器重配 */
#define CS_GYRO_TURN_TOL_MDEG        300   /* 兜底容差：只在"过零"迟迟不出现时停（正常由过零判停） */
#define CS_GYRO_TURN_TIMEOUT_MS    15000U   /* 陀螺闭环转身最长用时 */
/* 闭环转身的步数兜底倍率（1000 = 等于几何值）。deg_to_steps() 给出的就是"按模型
 * 正好转这么多度"的步数，余量为 0；转身时麦轮侧滑会让实际步数略超它，留 1.5 倍
 * 才不会在到位前被 TURN capped 截断（2026-11-11 就是这么坏的）。真实失败靠超时兜底。 */
#define CS_GYRO_TURN_CAP_PERMILLE   1500U
#define CS_GYRO_TURN_SLOW_ZONE_MDEG 8000   /* 进入减速区的误差：8 度 */
#define CS_GYRO_TURN_SLOW_HZ         600U   /* 减速区的转速；整条转身只在这里变一次速 */

#if (CS_STEPS_PER_REV < 200U || CS_STEPS_PER_REV > 51200U)
#error CS_STEPS_PER_REV out of range
#endif
#if (CS_WHEEL_CIRC_UM < 50000U || CS_WHEEL_CIRC_UM > 1000000U)
#error CS_WHEEL_CIRC_UM out of range
#endif
#if (CS_SPIN_L_MM < 50U || CS_SPIN_L_MM > 1000U)
#error CS_SPIN_L_MM out of range
#endif
#if (CS_TASK_FWD_MM_PER_S < 50U || CS_TASK_FWD_MM_PER_S > 2000U)
#error CS_TASK_FWD_MM_PER_S out of range
#endif
#if (CS_TASK_STRAFE_MM_PER_S < 50U || CS_TASK_STRAFE_MM_PER_S > 2000U)
#error CS_TASK_STRAFE_MM_PER_S out of range
#endif
#if (CS_TASK_SPIN_MM_PER_S < 20U || CS_TASK_SPIN_MM_PER_S > 1000U)
#error CS_TASK_SPIN_MM_PER_S out of range
#endif
#if (CS_TASK_EDGE_MIN_MM < 300U || CS_TASK_EDGE_MIN_MM > 5000U)
#error CS_TASK_EDGE_MIN_MM out of range
#endif
#if (CS_MOVE_RAMP_PERMILLE < 100U || CS_MOVE_RAMP_PERMILLE > 900U)
#error CS_MOVE_RAMP_PERMILLE out of range
#endif
#if (CS_MOVE_RAMP_UP_MS < 20U || CS_MOVE_RAMP_UP_MS > 2000U)
#error CS_MOVE_RAMP_UP_MS out of range
#endif
#if (CS_MOVE_RAMP_UP_STEPS < 1U || CS_MOVE_RAMP_UP_STEPS > 32U)
#error CS_MOVE_RAMP_UP_STEPS out of range
#endif
#if (CS_MOVE_RAMP_MIN_MS < 20U || CS_MOVE_RAMP_MIN_MS > CS_MOVE_RAMP_UP_MS)
#error CS_MOVE_RAMP_MIN_MS must be 20..CS_MOVE_RAMP_UP_MS
#endif
#if (CS_MOVE_RAMP_MIN_MS * 1U < CS_MOVE_RAMP_UP_STEPS * 4U)
#error CS_MOVE_RAMP_MIN_MS too small: one ramp step would be shorter than ~4 ms
#endif
#if (CS_MAP_ALIGN_MM <= 0 || CS_MAP_QR_DROP_MM <= 0 || CS_MAP_SCAN_BACK_MM <= 0 || \
     CS_MAP_RAW_LEG_MM <= 0 || CS_MAP_RAW_DROP_MM <= 0 || CS_MAP_LANE_CROSS_MM <= 0 || \
     CS_MAP_STORE_LEG_MM <= 0 || CS_MAP_STORE_EXIT_MM <= 0 || CS_MAP_RAW2_LEG_MM <= 0 || \
     CS_MAP_RETURN_EAST_MM <= 0 || CS_MAP_RETURN_NORTH_MM <= 0)
#error every route segment must be positive
#endif
#if (CS_MAP_SCAN_WAIT_MS <= 0 || CS_MAP_SCAN_WAIT_MS > 30000)
#error scan wait out of range
#endif
#if (CS_MAP_WORK_CYC_ROUNDS < 1U || CS_MAP_WORK_CYC_ROUNDS > 20U)
#error CS_MAP_WORK_CYC_ROUNDS must be 1..20 (PK_CYC_MAX_ROUNDS)
#endif
#if (CS_CMD_LEN < 8U || CS_CMD_LEN > 80U)
#error CS_CMD_LEN out of range
#endif
#if (CS_GYRO_ENABLE != 0U && CS_GYRO_ENABLE != 1U)
#error CS_GYRO_ENABLE must be 0 or 1
#endif
#if (CS_GYRO_TIMEOUT_MS < 50U || CS_GYRO_TIMEOUT_MS > 5000U)
#error CS_GYRO_TIMEOUT_MS must be 50..5000
#endif
#if (CS_GYRO_HOLD_UPDATE_MS < 2U || CS_GYRO_HOLD_UPDATE_MS > 100U)
#error CS_GYRO_HOLD_UPDATE_MS must be 2..100
#endif
#if (CS_GYRO_HOLD_DEADBAND_MDEG < 0 || CS_GYRO_HOLD_DEADBAND_MDEG > 20000)
#error CS_GYRO_HOLD_DEADBAND_MDEG must be 0..20000
#endif
#if (CS_GYRO_HOLD_KP_HZ_PER_DEG < 0 || CS_GYRO_HOLD_KP_HZ_PER_DEG > 500)
#error CS_GYRO_HOLD_KP_HZ_PER_DEG must be 0..500
#endif
#if (CS_GYRO_HOLD_KI_HZ_PER_DEG_S < 0 || CS_GYRO_HOLD_KI_HZ_PER_DEG_S > 2000)
#error CS_GYRO_HOLD_KI_HZ_PER_DEG_S must be 0..2000
#endif
#if (CS_GYRO_HOLD_I_LIMIT_HZ < 0 || CS_GYRO_HOLD_I_LIMIT_HZ > 2000)
#error CS_GYRO_HOLD_I_LIMIT_HZ must be 0..2000
#endif
/* 不变量：积分上限 / Kp 必须大于死区（单位统一到 mdeg 比较）。
 * 否则积分饱和后的平衡偏角 = I_LIMIT/Kp 落在死区内，误差不会再累积也退不回来，
 * 车会永久停在那个偏角上——2026-11-09 横移"修正不过来"就是这条被违反。 */
#if (CS_GYRO_HOLD_KP_HZ_PER_DEG > 0) && \
    ((CS_GYRO_HOLD_I_LIMIT_HZ * 1000) <= (CS_GYRO_HOLD_DEADBAND_MDEG * CS_GYRO_HOLD_KP_HZ_PER_DEG))
#error CS_GYRO_HOLD_I_LIMIT_HZ too small: I_LIMIT/Kp must exceed the deadband
#endif
#if (CS_GYRO_HOLD_KD_HZ_PER_DPS < 0 || CS_GYRO_HOLD_KD_HZ_PER_DPS > 500)
#error CS_GYRO_HOLD_KD_HZ_PER_DPS must be 0..500
#endif
#if (CS_GYRO_HOLD_LAUNCH_MS > 2000U)
#error CS_GYRO_HOLD_LAUNCH_MS must be 0..2000
#endif
#if (CS_GYRO_HOLD_KD_LAUNCH_HZ_PER_DPS < 0 || CS_GYRO_HOLD_KD_LAUNCH_HZ_PER_DPS > 2000)
#error CS_GYRO_HOLD_KD_LAUNCH_HZ_PER_DPS must be 0..2000
#endif
#if (CS_GYRO_HOLD_RATE_SHIFT < 0U || CS_GYRO_HOLD_RATE_SHIFT > 5U)
#error CS_GYRO_HOLD_RATE_SHIFT must be 0..5
#endif
#if (CS_GYRO_STRAFE_BIAS_MDEG < -30000 || CS_GYRO_STRAFE_BIAS_MDEG > 30000)
#error CS_GYRO_STRAFE_BIAS_MDEG must be -30000..30000
#endif
#if (CS_TRIM_A_PERMILLE < 800U || CS_TRIM_A_PERMILLE > 1200U || \
     CS_TRIM_B_PERMILLE < 800U || CS_TRIM_B_PERMILLE > 1200U || \
     CS_TRIM_C_PERMILLE < 800U || CS_TRIM_C_PERMILLE > 1200U || \
     CS_TRIM_D_PERMILLE < 800U || CS_TRIM_D_PERMILLE > 1200U)
#error CS_TRIM_x_PERMILLE must be 800..1200
#endif
#if (CS_GYRO_HOLD_MAX_PERMILLE < 50U || CS_GYRO_HOLD_MAX_PERMILLE > 900U)
#error CS_GYRO_HOLD_MAX_PERMILLE must be 50..900
#endif
#if (CS_GYRO_HOLD_MIN_STEP_HZ < 0 || CS_GYRO_HOLD_MIN_STEP_HZ > 200)
#error CS_GYRO_HOLD_MIN_STEP_HZ must be 0..200
#endif
#if (CS_GYRO_TURN_TOL_MDEG < 100 || CS_GYRO_TURN_TOL_MDEG > 20000)
#error CS_GYRO_TURN_TOL_MDEG must be 100..20000
#endif
#if (CS_GYRO_TURN_TIMEOUT_MS < 1000U || CS_GYRO_TURN_TIMEOUT_MS > 120000U)
#error CS_GYRO_TURN_TIMEOUT_MS must be 1000..120000
#endif
#if (CS_GYRO_TURN_CAP_PERMILLE < 1000U || CS_GYRO_TURN_CAP_PERMILLE > 4000U)
#error CS_GYRO_TURN_CAP_PERMILLE must be 1000..4000 (>=1000 keeps the cap above the geometric need)
#endif
#if (CS_GYRO_TURN_SLOW_ZONE_MDEG < 1000 || CS_GYRO_TURN_SLOW_ZONE_MDEG > 90000)
#error CS_GYRO_TURN_SLOW_ZONE_MDEG must be 1000..90000
#endif
#if (CS_GYRO_TURN_SLOW_HZ < 50U || CS_GYRO_TURN_SLOW_HZ > 5000U)
#error CS_GYRO_TURN_SLOW_HZ must be 50..5000
#endif
#if (CS_LINE_IDLE_MS < 10U || CS_LINE_IDLE_MS > 500U)
#error CS_LINE_IDLE_MS out of range
#endif

#endif
