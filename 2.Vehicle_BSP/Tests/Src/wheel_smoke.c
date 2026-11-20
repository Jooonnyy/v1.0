#include "wheel_smoke.h"
#include "board.h"
#include "board_encoder.h"
#include "board_wheel.h"
#include "board_mpu6050.h"
#include "chassis_odom.h"
#include <string.h>
#include <limits.h>

/* 四轮速度闭环台架：中断控制，主循环收命令、拼整行遥测。
 * 未实测的极性、角映射、前馈与增益保持 BSP 占位值，待标定。 */
/* ============================== 参数区 ============================== */
#define WHEEL_SMOKE_TX_HZ            50U
/* 2026-11-03 新底板 V2：蓝牙回到自己的 UART5（`SERIAL_BT`，PC12/PD2），
 * 陀螺 HWT101 单独占 USART2（`SERIAL_IMU`，PA2/PA3），两者不再抢同一个串口。
 * 老板上"临时把 HC-05 接到 PING/USART6"的做法已经不需要了，这里改回 SERIAL_BT；
 * 要看串口屏就把它改成 SERIAL_HMI。 */
#define WHEEL_SMOKE_TELEMETRY_PORT   SERIAL_BT
#define WHEEL_SMOKE_TEXT_PORT        SERIAL_BT
/* 0 = 单轮 target/actual/output；1 = 里程计位姿；2 = 四轮 target/actual/output（12 通道）；
 * 3 = 陀螺仪（yaw、yaw 角速度、帧龄、帧数、坏帧数）。运行时可用 TM <n> 切换。 */
#define WHEEL_SMOKE_TELEMETRY_MODE   2
#define WHEEL_SMOKE_CMD_LEN          48U
#define WHEEL_SMOKE_TARGET_COUNTS_PER_S 5000
#define WHEEL_SMOKE_BODY_UM_PER_S    1000000
#define WHEEL_SMOKE_SPIN_MDPS        360000
#define WHEEL_SMOKE_RX_BUDGET        64U

/* 单电机闭环模式：只测一路，不初始化 MPU6050 / 里程计，也不做上电自旋。
 * 四轮前后左右测试要把这里改成 0，并保持 WHEEL_SMOKE_USE_ODOM=0（新板没有 MPU）。 */
#define WHEEL_SMOKE_SINGLE_MOTOR     0
/* 是否使用 MPU6050 + ChassisOdom。新板已拆掉 MPU/TOF，四轮测试保持 0。 */
#define WHEEL_SMOKE_USE_ODOM         0

#if (WHEEL_SMOKE_SINGLE_MOTOR != 0 && WHEEL_SMOKE_SINGLE_MOTOR != 1)
#error WHEEL_SMOKE_SINGLE_MOTOR must be 0 or 1
#endif
#if (WHEEL_SMOKE_USE_ODOM != 0 && WHEEL_SMOKE_USE_ODOM != 1)
#error WHEEL_SMOKE_USE_ODOM must be 0 or 1
#endif
#if WHEEL_SMOKE_SINGLE_MOTOR && WHEEL_SMOKE_USE_ODOM
#error single-motor mode cannot use odometry
#endif
#if WHEEL_SMOKE_SINGLE_MOTOR && (WHEEL_SMOKE_TELEMETRY_MODE != 0)
#error single-motor mode requires WHEEL_SMOKE_TELEMETRY_MODE 0
#endif
#if !WHEEL_SMOKE_USE_ODOM && (WHEEL_SMOKE_TELEMETRY_MODE == 1)
#error telemetry mode 1 needs WHEEL_SMOKE_USE_ODOM=1
#endif

/* ---- X 阶跃测试：一条命令做完一次试验，只回一行摘要（不刷屏）---- */
#define WHEEL_SMOKE_STEP_MS_DEFAULT 1200U   /* 默认试验时长，单位 ms */
#define WHEEL_SMOKE_STEP_MS_MIN      200U
#define WHEEL_SMOKE_STEP_MS_MAX     5000U
#define WHEEL_SMOKE_STEP_SAMPLE_MS     2U   /* 采样周期，单位 ms（500 Hz 采实际值） */
#define WHEEL_SMOKE_STEP_TAIL         64U   /* 末尾窗口样本数，用来算稳态与振荡 */

/* D 开环调试命令的占空比上限，单位千分比。台架验证不需要满占空比。 */
#define WHEEL_SMOKE_DIRECT_PERMILLE  500

/* ---- 开环电机台架（ALL / CMP / N）----
 * 不经过闭环，直接 Motor_Set 给占空比：拆掉联轴器后比较四路电机本体，
 * 或者在闭环之前先把"电机 / 驱动 / 动力电"这一段确认掉。
 * 与 D 命令一致：正 = Motor_Set 的电气正方向，不一定等于闭环里的"前进"方向。 */
#define WHEEL_SMOKE_OL_PERMILLE_DEF   300     /* CMP 默认占空比 */
#define WHEEL_SMOKE_OL_CMP_MS_DEF    3000U    /* CMP 每路持续时间，单位 ms */
#define WHEEL_SMOKE_OL_CMP_MS_MIN    1000U    /* 每路至少 1 s：末尾 500 ms 窗口要装得下 */
#define WHEEL_SMOKE_OL_CMP_MS_MAX   10000U
#define WHEEL_SMOKE_OL_TAIL_MS        500U    /* 取每路末尾这段时间的平均值当稳态 */
#define WHEEL_SMOKE_OL_IDLE_STOP_MS 60000U    /* 开环输出非零且这么久没命令就自动停；0 = 关闭 */

/* ---- 里程计 + 点到点导航（R / GO）----
 * 只用编码器 + HWT101：编码器给四轮线速度，陀螺仪给航向（先用 10 Hz 也够）。
 * 导航时把航向稳定在 0，此时车体系 ≈ 场地系，直接 vx = Kp·ex、vy = Kp·ey 就能点到点
 * （麦轮可以横移）。全程不需要三角函数，也不引入浮点。 */
#define WHEEL_SMOKE_ODOM_PERIOD_MS        10U    /* 里程计积分周期，单位 ms */
#define WHEEL_SMOKE_GOTO_PERIOD_MS        20U    /* 导航控制周期，单位 ms */
#define WHEEL_SMOKE_GOTO_POS_GAIN          3     /* 每 1 mm 位置误差给多少 mm/s */
#define WHEEL_SMOKE_GOTO_SPEED_MAX_MM_S  300
#define WHEEL_SMOKE_GOTO_SPEED_SLOW_MM_S  80
#define WHEEL_SMOKE_GOTO_SLOW_DIST_MM    120     /* 离目标这么近就降到慢速 */
#define WHEEL_SMOKE_GOTO_TOL_MM           15     /* 判定到点 */
#define WHEEL_SMOKE_GOTO_HEAD_GAIN         4     /* 每 1 mdeg 航向误差给多少 mdps */
#define WHEEL_SMOKE_GOTO_WZ_LIMIT_MDPS 60000     /* 60 deg/s */
#define WHEEL_SMOKE_GOTO_TURN_FIRST_MDEG 30000   /* 航向差超过 30° 先原地转 */
#define WHEEL_SMOKE_GOTO_TIMEOUT_MS    30000U    /* 单段超时，单位 ms */

/* ---- A <±deg>：闭环原地转到相对角度（HWT101 航向环）----
 * 正 = 逆时针（左转），负 = 顺时针（右转）。到位保持后自动全停并回一行摘要。 */
/* 到位阈，单位 mdeg。2026-10-28 由 2000 收紧到 500：v0.3 首跑 6 个 TURN_ABS 里
 * 4 个带着 ±1.0~1.2° 残差收工（都在旧阈内"合格"），随后的 900~1950 mm 段被整体
 * 偏转，端点差 = 段长×sin(残差) ≈ 16~34 mm/段，是整程闭合差的大头之一。
 * 500 = 航向环死区（300）之上留一点余量，P 项能把末段收敛进来；代价是转身
 * 收尾略慢（每次多几百 ms）。 */
#define WHEEL_SMOKE_TURN_TOL_MDEG        500
#define WHEEL_SMOKE_TURN_RATE_TOL_MDPS 15000     /* 到位时角速度也要低于 15 deg/s */
#define WHEEL_SMOKE_TURN_SETTLE_MS      400U     /* 连续保持这么久才算到位 */
#define WHEEL_SMOKE_TURN_TIMEOUT_MS   12000U     /* 整段超时，单位 ms */
#define WHEEL_SMOKE_TURN_MIN_DEG          3      /* 绝对值下限，太小分不出到位 */
#define WHEEL_SMOKE_TURN_MAX_DEG        180

/* ---- LAP <side_mm>：正方形跑一圈（四个 GO 航点串联）----
 * 每段"先原地转正到段航向、再直行"（各段纯前进，不走横移），
 * 恒用现有 goto 状态机；段间结算靠 goto 的结果位。 */
#define WHEEL_SMOKE_LAP_MIN_MM           300
#define WHEEL_SMOKE_LAP_MAX_MM          3000

/* M <distance_mm> [speed_um_per_s]：编码器累计走一段距离后自动停。
 * 负距离 = 后退。默认 0.15 m/s；接近目标时降到 0.06 m/s 减少过冲。 */
#define WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S 150000
#define WHEEL_SMOKE_MOVE_MIN_MM                 10
#define WHEEL_SMOKE_MOVE_MAX_MM                 5000
#define WHEEL_SMOKE_MOVE_TIMEOUT_MS             60000U
#define WHEEL_SMOKE_MOVE_SLOW_MM                40
#define WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S    60000
/* 减速起点随速度缩放（2026-10-29 加）：40 mm 是按 0.4 m/s 标定的，把直行提到
 * 0.6 m/s 后同样 40 mm 根本来不及从 0.6 降到 0.06（要 0.54 m/s 的落差压进 40 mm），
 * 必然冲过头 —— 而走廊净宽 400、车 300，余量只有 ±50 mm。
 * 定加速度模型下刹车距离 ∝ v²，所以按 (v / 参考速度)² 放大：
 *   0.4 m/s -> 40 mm（原值，不变）  0.6 m/s -> 90 mm  0.84 m/s -> 176 mm
 * 参考速度及以下一律用 WHEEL_SMOKE_MOVE_SLOW_MM，所以 M 默认 0.15 m/s、横移 0.3 m/s
 * 的行为与改动前完全一致。封顶 WHEEL_SMOKE_MOVE_SLOW_MAX_MM 防离谱值把整段行程吃掉。 */
#define WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S     400000
#define WHEEL_SMOKE_MOVE_SLOW_MAX_MM            300
/* 到点后的闭环保持时长，单位 ms。0 = 到点立即断电（原来的行为）。
 * 为什么要有：Wheel_Stop 是"输出清零 + 短接刹车 150 ms + 滑行"，而横移方向
 * 车体是沿滚轴自由滑的，断电后溜出去的距离不进任何单段打印、只进整程闭合差
 * （v0.3 首跑各段侧偏账面合计约 35 mm，闭合差却到 98 mm，缺口主要是它）。
 * 改成到点后目标清零、闭环再保持一小段，让速度环把滑行 actively 顶住，
 * 窗口走完才真正 Wheel_Stop。每段多花这么长的停车时间。 */
#define WHEEL_SMOKE_MOVE_HOLD_MS                 300U
/* 横移横向位置环增益，单位 (um/s)/mm：横向偏移 1 mm 折算多少 um/s 的车体
 * 前后向修正速度。0 = 关闭。横移溜车方向正负都出现过（v0.3 首跑 10 段里
 * 9 段向后 1 段向前），固定补偿行不通，用 odom 每拍反馈闭环：
 * 2000 = 偏 1 mm 修 2 mm/s；漂移速率约 1.5%×横移速度（300 mm/s 时约
 * 4.5 mm/s），稳态残差 = 漂移速率 / 增益 ≈ 2~3 mm，远小于开环的 26~35 mm。 */
#define WHEEL_SMOKE_STRAFE_CROSS_GAIN_UM_PER_S_PER_MM  2000
/* 横向偏移参与计算的限幅，单位 mm：偏得再多也按这个数折算，防修正量爆算。 */
#define WHEEL_SMOKE_STRAFE_CROSS_MAX_MM                  50
/* 横向修正速度限幅，单位 um/s：修正量封顶，防止位置环抢过主运动。 */
#define WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S           60000
/* 轮周长，单位 um，供 M 定距离换算用。
 * 2026-10-27 修：原式 `WHEEL_DIAMETER_MM * 31416 / 10000` 少乘 1000，
 * 算出来是 235（um）而不是 235619 —— 每转只认 0.235 mm，于是 `M 300` 永远到不了
 * 300000 um，只会 60 s 超时。改成和 board_encoder.c 完全相同的 355/113 近似，
 * 两处口径一致，轮径改一处就够。 */
#define WHEEL_SMOKE_CIRCUMFERENCE_UM \
  ((int32_t)(((int64_t)WHEEL_DIAMETER_MM * 355000LL) / 113LL))

/* 航向保持（HWT101 yaw 外环）。误差单位 mdeg，输出 wz 单位 mdps。
 * 2026-10-26 实测：左侧 A/C 比右侧慢约 220 counts/s，折算自然偏航约 35 deg/s；
 * 纯 P 环 Kp=2 会留下十几度的稳态误差，所以加积分项把稳态误差压掉。
 * ⚠ 那条 220 counts/s 是 A/C 还跑在 AT8236 快衰减弱方向时测的，2026-10-26 换线后
 *   A/C 已与其他两路对称，"必须先有大积分"的前提已经不成立了。
 *
 * 2026-10-27 大角度转向过冲（实板实测 A 90 要 7~8.5 s）：
 *   积分限幅原来是 30000 mdps，90000 mdeg 的阶跃下 330 ms 就顶满，之后 P 项已经
 *   反向、积分还在往前推，车要冲过头把积分"顶回来"才肯停。退积分的速度是
 *   |error|/100 每 10 ms（Ki=1 时），误差 1° 时每拍只退 10 mdps，
 *   30000 要退到秒级 —— 这就是"能转到但微调很慢"的来源。
 *   两处修：① 限幅 30000 → 4000（最大只贡献 4 deg/s，约 70 counts/s 轮速）；
 *          ② 积分分离：|误差| > INT_ZONE 时不积分，大角度阶段完全交给 P，
 *             避免"边冲边积"的 windup。 */
#define WHEEL_SMOKE_HEADING_KP                4
/* 2026-10-27 实测把默认积分关掉：`J 0` 下 `A 180` 只用 2717 ms、终点误差 -417 mdeg，
 * 而带积分时 `A 90` 要 7249~8480 ms、误差还更大（-604~-1005）。
 * 也就是说 A/C 换线修好之后，已经不需要积分去补左右不对称了；末端这 0.4° 的残差
 * 由 P 项自己收敛得掉。台架上要重新加：在线发 `J 1` 即可（限幅与积分分离都在，不会 windup）。 */
#define WHEEL_SMOKE_HEADING_KI                0
#define WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS 4000
/* 积分分离阈值，单位 mdeg：只有 |误差| 小于它才累加积分。0 = 关闭（始终积分）。 */
#define WHEEL_SMOKE_HEADING_INT_ZONE_MDEG 15000
#define WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS  80000
/* 航向环输出的斜率限制，单位 mdps / 10 ms 控制拍。
 * 0 = 关闭（原来的行为，wz 一拍跳到限幅）。
 * 8000 = 100 ms 从 0 爬到 80 deg/s。
 * 为什么要这个：原地转向时四路同一拍收到 ±1408 counts/s 的阶跃目标，但左右静摩擦不同
 * （左侧起步要 ~370‰、右侧 ~110‰，2026-10-26 实测），慢的一侧滞后 100 ms 以上，
 * 车头就被另一侧"拽"着平移 —— 这是"原地转却跑位置"的瞬时主因。缓升能让速度环的
 * 积分有时间把四路拉平，把这段平移压掉。 */
#define WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS    8000
#define WHEEL_SMOKE_HEADING_DEADBAND_MDEG    300
#define WHEEL_SMOKE_HEADING_TIMEOUT_MS       500U
#define WHEEL_SMOKE_YAW_SIGN                   1
/* 横移段的 wz 限幅，单位 mdps。横移时航向环只负责压住滚轴摩擦不对称造成的
 * 小幅 yaw 扰动，用不到原地转调出来的 80 deg/s 权限；权限太大，起步/到点
 * 瞬间的打滑会被航向环猛修一下再甩回来，把扭转放大成肉眼可见的晃动。
 * 0 = 不单独限（沿用 WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS）。 */
#define WHEEL_SMOKE_STRAFE_WZ_LIMIT_MDPS  15000
/* ---- 倒车时的航向环增益 ----
 * 为什么倒车要单独一套：AT8236 的正反不对称不是"转不动"，而是"同样一条 wz 命令
 * 在倒车方向只转出约 43% 的角速度"（600 counts/s 正转 ~130‰、反转 ~300‰）。
 * 纯 P 环的稳态偏差 = 扰动 / 环路增益，增益掉一半多，倒车的稳态偏航就是正转的两倍多，
 * 表现为"正着开很直、倒着开会慢慢偏出去"。
 * 两个手段一起用：把比例增益按倍数补回来，并且倒车时打开一个小积分把残余零差吃掉。
 * 只在 heading_base_vx < 0（倒车）时生效；正转和 A 原地转完全不受影响。 */
#define WHEEL_SMOKE_HEADING_REV_GAIN_PERMILLE 2300   /* 反转比例增益倍数，1000 = 不补 */
#define WHEEL_SMOKE_HEADING_REV_KI               2   /* 反转专用积分系数，0 = 倒车也不积分 */

/* ---- 上电自检：四路同时低速正转（POWER-ON SPIN，仅台架）----
 * 目的：一上电就能看出哪几路电机/驱动通路是通的，不用发任何命令。
 * 走的是**开环直写**（不经过闭环），所以不依赖编码器，也不会误判堵转。
 * ⚠ 只在 WHEEL Target 里；生产固件不编译本文件，上电仍是四路输出 0。 */
#define WHEEL_SMOKE_AUTOSPIN              0     /* 1 = 上电自动转；0 = 关掉 */
#define WHEEL_SMOKE_AUTOSPIN_DELAY_MS  3000U    /* 上电后等多久再转，给人反应时间 */
#define WHEEL_SMOKE_AUTOSPIN_PERMILLE   250     /* 开环占空比，单位千分比 */
#define WHEEL_SMOKE_AUTOSPIN_MS        8000U    /* 转多久自动停，0 = 一直转到有命令 */

#if WHEEL_SMOKE_AUTOSPIN != 0 && WHEEL_SMOKE_AUTOSPIN != 1
#error WHEEL_SMOKE_AUTOSPIN must be 0 or 1
#endif
#if WHEEL_SMOKE_AUTOSPIN_DELAY_MS < 500U || WHEEL_SMOKE_AUTOSPIN_DELAY_MS > 30000U
#error WHEEL_SMOKE_AUTOSPIN_DELAY_MS must be within 500..30000
#endif
#if WHEEL_SMOKE_AUTOSPIN_PERMILLE < 150 || WHEEL_SMOKE_AUTOSPIN_PERMILLE > 500
#error WHEEL_SMOKE_AUTOSPIN_PERMILLE must be within 150..500 (below ~150 the 12V motor may not start)
#endif
#if WHEEL_SMOKE_AUTOSPIN_MS > 60000U
#error WHEEL_SMOKE_AUTOSPIN_MS must be <= 60000 (0 = run until a command arrives)
#endif

#if WHEEL_SMOKE_DIRECT_PERMILLE < 1 || WHEEL_SMOKE_DIRECT_PERMILLE > 1000
#error WHEEL_SMOKE_DIRECT_PERMILLE must be within 1..1000
#endif
#if WHEEL_SMOKE_OL_PERMILLE_DEF < 1 || WHEEL_SMOKE_OL_PERMILLE_DEF > WHEEL_SMOKE_DIRECT_PERMILLE
#error WHEEL_SMOKE_OL_PERMILLE_DEF must be within 1..WHEEL_SMOKE_DIRECT_PERMILLE
#endif
#if WHEEL_SMOKE_OL_CMP_MS_MIN < 200U || WHEEL_SMOKE_OL_CMP_MS_MAX > 30000U || \
    WHEEL_SMOKE_OL_CMP_MS_DEF < WHEEL_SMOKE_OL_CMP_MS_MIN || \
    WHEEL_SMOKE_OL_CMP_MS_DEF > WHEEL_SMOKE_OL_CMP_MS_MAX
#error WHEEL_SMOKE_OL_CMP_MS limits are invalid
#endif
#if WHEEL_SMOKE_OL_TAIL_MS < 100U || (WHEEL_SMOKE_OL_TAIL_MS * 2U) > WHEEL_SMOKE_OL_CMP_MS_MIN
#error WHEEL_SMOKE_OL_TAIL_MS must be >=100 and <= half of the minimum CMP duration
#endif
#if WHEEL_SMOKE_OL_IDLE_STOP_MS != 0U && WHEEL_SMOKE_OL_IDLE_STOP_MS < 1000U
#error WHEEL_SMOKE_OL_IDLE_STOP_MS must be 0 or at least 1000
#endif
#if WHEEL_SMOKE_GOTO_SPEED_SLOW_MM_S < 20 || WHEEL_SMOKE_GOTO_SPEED_MAX_MM_S > 600 || \
    WHEEL_SMOKE_GOTO_SPEED_SLOW_MM_S >= WHEEL_SMOKE_GOTO_SPEED_MAX_MM_S
#error goto speed limits are invalid
#endif
#if WHEEL_SMOKE_GOTO_TOL_MM < 5 || WHEEL_SMOKE_GOTO_TOL_MM > 100
#error WHEEL_SMOKE_GOTO_TOL_MM must be within 5..100 mm
#endif
#if WHEEL_SMOKE_GOTO_PERIOD_MS < 5U || WHEEL_SMOKE_ODOM_PERIOD_MS < 2U
#error goto/odometry periods are too short
#endif
#if WHEEL_SMOKE_TURN_TOL_MDEG < 200 || WHEEL_SMOKE_TURN_TOL_MDEG > WHEEL_SMOKE_GOTO_TURN_FIRST_MDEG
#error WHEEL_SMOKE_TURN_TOL_MDEG must stay within 200..GOTO_TURN_FIRST
#endif
#if WHEEL_SMOKE_STRAFE_WZ_LIMIT_MDPS < 0 || \
    WHEEL_SMOKE_STRAFE_WZ_LIMIT_MDPS > WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS
#error WHEEL_SMOKE_STRAFE_WZ_LIMIT_MDPS must be within 0..WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS
#endif
#if WHEEL_SMOKE_TURN_RATE_TOL_MDPS < 1000 || WHEEL_SMOKE_TURN_RATE_TOL_MDPS > 60000
#error WHEEL_SMOKE_TURN_RATE_TOL_MDPS must be within 1000..60000
#endif
#if WHEEL_SMOKE_TURN_SETTLE_MS < 100U || WHEEL_SMOKE_TURN_SETTLE_MS > WHEEL_SMOKE_TURN_TIMEOUT_MS
#error WHEEL_SMOKE_TURN_SETTLE_MS must be within 100..TURN_TIMEOUT
#endif
#if WHEEL_SMOKE_TURN_TIMEOUT_MS < 3000U || WHEEL_SMOKE_TURN_TIMEOUT_MS > 60000U
#error WHEEL_SMOKE_TURN_TIMEOUT_MS must be within 3000..60000
#endif
#if WHEEL_SMOKE_TURN_MIN_DEG < 1 || WHEEL_SMOKE_TURN_MAX_DEG < WHEEL_SMOKE_TURN_MIN_DEG
#error turn deg limits are invalid
#endif
#if (WHEEL_SMOKE_HEADING_INT_ZONE_MDEG < 0 || WHEEL_SMOKE_HEADING_INT_ZONE_MDEG > 180000)
#error WHEEL_SMOKE_HEADING_INT_ZONE_MDEG must be within 0..180000 (0 = integral always on)
#endif
#if (WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS < 0 || \
     WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS > WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS)
#error WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS must be within 0..WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS
#endif
#if (WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS < 0 || \
     WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS > WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS)
#error WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS must be within 0..WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS
#endif
#if (WHEEL_SMOKE_HEADING_REV_GAIN_PERMILLE < 1000 || \
     WHEEL_SMOKE_HEADING_REV_GAIN_PERMILLE > 5000)
#error WHEEL_SMOKE_HEADING_REV_GAIN_PERMILLE must be within 1000..5000 (1000 = no boost)
#endif
#if (WHEEL_SMOKE_HEADING_REV_KI < 0 || WHEEL_SMOKE_HEADING_REV_KI > 50)
#error WHEEL_SMOKE_HEADING_REV_KI must be within 0..50
#endif
#if WHEEL_SMOKE_LAP_MIN_MM < 100 || WHEEL_SMOKE_LAP_MAX_MM < WHEEL_SMOKE_LAP_MIN_MM || \
    WHEEL_SMOKE_LAP_MAX_MM > 10000
#error WHEEL_SMOKE_LAP_MIN_MM / MAX_MM are invalid
#endif
#if WHEEL_SMOKE_MOVE_MIN_MM < 1 || WHEEL_SMOKE_MOVE_MAX_MM < WHEEL_SMOKE_MOVE_MIN_MM
#error Invalid move distance limits
#endif
#if WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S < 10000 || \
    WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S > 500000
#error WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S must be within 10000..500000
#endif
#if WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S < 10000 || \
    WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S > WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S
#error WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S must be within 10000..default
#endif
#if WHEEL_SMOKE_MOVE_HOLD_MS > 2000U
#error WHEEL_SMOKE_MOVE_HOLD_MS must be within 0..2000
#endif
#if WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S < WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S
#error WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S must not be below the slow speed
#endif
#if WHEEL_SMOKE_MOVE_SLOW_MAX_MM < WHEEL_SMOKE_MOVE_SLOW_MM || \
    WHEEL_SMOKE_MOVE_SLOW_MAX_MM > 1000
#error WHEEL_SMOKE_MOVE_SLOW_MAX_MM must be within WHEEL_SMOKE_MOVE_SLOW_MM..1000
#endif
#if WHEEL_SMOKE_STRAFE_CROSS_GAIN_UM_PER_S_PER_MM < 0 || \
    WHEEL_SMOKE_STRAFE_CROSS_GAIN_UM_PER_S_PER_MM > 10000
#error WHEEL_SMOKE_STRAFE_CROSS_GAIN_UM_PER_S_PER_MM must be within 0..10000
#endif
#if WHEEL_SMOKE_STRAFE_CROSS_MAX_MM < 1 || WHEEL_SMOKE_STRAFE_CROSS_MAX_MM > 200
#error WHEEL_SMOKE_STRAFE_CROSS_MAX_MM must be within 1..200
#endif
#if WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S < 1000 || \
    WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S > 150000
#error WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S must be within 1000..150000
#endif

#if WHEEL_SMOKE_STEP_MS_DEFAULT < WHEEL_SMOKE_STEP_MS_MIN || \
    WHEEL_SMOKE_STEP_MS_DEFAULT > WHEEL_SMOKE_STEP_MS_MAX
#error WHEEL_SMOKE_STEP_MS_DEFAULT must be within MIN..MAX
#endif
#if WHEEL_SMOKE_STEP_SAMPLE_MS < 1U || WHEEL_SMOKE_STEP_SAMPLE_MS > 20U
#error WHEEL_SMOKE_STEP_SAMPLE_MS must be within 1..20
#endif
#if WHEEL_SMOKE_STEP_TAIL < 8U || WHEEL_SMOKE_STEP_TAIL > 128U
#error WHEEL_SMOKE_STEP_TAIL must be within 8..128
#endif
#if WHEEL_SMOKE_TX_HZ < 1U || WHEEL_SMOKE_TX_HZ > 200U
#error WHEEL_SMOKE_TX_HZ must be within 1..200
#endif
#if (WHEEL_SMOKE_TELEMETRY_MODE != 0 && WHEEL_SMOKE_TELEMETRY_MODE != 1 && \
     WHEEL_SMOKE_TELEMETRY_MODE != 2 && WHEEL_SMOKE_TELEMETRY_MODE != 3 && \
     WHEEL_SMOKE_TELEMETRY_MODE != 4)
#error WHEEL_SMOKE_TELEMETRY_MODE must be 0, 1, 2, 3 or 4
#endif
#if WHEEL_SMOKE_CMD_LEN < 32U || WHEEL_SMOKE_CMD_LEN > 128U
#error WHEEL_SMOKE_CMD_LEN must be within 32..128
#endif
#if WHEEL_SMOKE_TARGET_COUNTS_PER_S < 1 || WHEEL_SMOKE_TARGET_COUNTS_PER_S > 5000
#error WHEEL_SMOKE_TARGET_COUNTS_PER_S must be within 1..5000
#endif
#if WHEEL_SMOKE_BODY_UM_PER_S < 1 || WHEEL_SMOKE_BODY_UM_PER_S > 1000000
#error WHEEL_SMOKE_BODY_UM_PER_S must be within 1..1000000
#endif
#if WHEEL_SMOKE_SPIN_MDPS < 1 || WHEEL_SMOKE_SPIN_MDPS > 360000
#error WHEEL_SMOKE_SPIN_MDPS must be within 1..360000
#endif
#if WHEEL_SMOKE_RX_BUDGET < 1U || WHEEL_SMOKE_RX_BUDGET > 256U
#error WHEEL_SMOKE_RX_BUDGET must be within 1..256
#endif
/* 串口名是枚举而非预处理宏，使用 C99 类型长度完成编译期校验。 */
typedef char TelemetryPortValid[(WHEEL_SMOKE_TELEMETRY_PORT == SERIAL_DEBUG ||
                                WHEEL_SMOKE_TELEMETRY_PORT == SERIAL_BT ||
                                WHEEL_SMOKE_TELEMETRY_PORT == SERIAL_HMI) ? 1 : -1];
typedef char TextPortValid[(WHEEL_SMOKE_TEXT_PORT >= SERIAL_DEBUG &&
                          WHEEL_SMOKE_TEXT_PORT <= SERIAL_BT) ? 1 : -1];

typedef struct {
  char line[WHEEL_SMOKE_CMD_LEN];
  uint8_t length, overflow;
} CommandInput;
static CommandInput inputs[2]; /* 两个口各自拼帧，不能互相拼接半条命令。 */
static char output_line[768];
static uint16_t output_length;
static uint8_t telemetry_enabled;
/* 遥测格式：0 单轮 / 1 里程计 / 2 四轮 / 3 陀螺。编译期默认值可被 TM 命令在运行时改。 */
static uint8_t telemetry_mode = WHEEL_SMOKE_TELEMETRY_MODE;
static MotorId selected_wheel;
static uint32_t next_tx_ms;
static uint32_t reported_stall_events;

/* 开环台架状态：ol_permille[] 记录当前"指令占空比"，给 S 和遥测用；
 * ol_mode=1 表示闭环已关（Wheel_Enable(0)），此时 Wheel_OutputPermille() 恒为 0。 */
static int16_t  ol_permille[4];
static uint8_t  ol_mode;
static uint32_t ol_last_cmd_ms;
/* CMP：四路依次开环，每路取末尾窗口的平均转速，最后一行打印四路对比。 */
typedef struct {
  uint8_t  active;
  uint8_t  wheel;
  int16_t  permille;
  uint32_t duration_ms;
  uint32_t wheel_until_ms;
  int32_t  sum_counts_per_s;
  uint32_t samples;
  int32_t  result_counts_per_s[4];
} OpenLoopCompare;
static OpenLoopCompare ol_cmp;

/* ---- HWT101：USART2 / PA2/PA3（`SERIAL_IMU`），维特 0x55 帧 ----
 * 角度帧：55 53 RollL RollH PitchL PitchH YawL YawH VL VH SUM（11 字节）。
 * 只解析 0x53 角度帧；0x51/0x52 校验通过后丢弃。 */
#define HWT101_FRAME_LEN       11U
#define HWT101_HEADER          0x55U
#define HWT101_ANGLE_TYPE      0x53U
static uint8_t  hwt_frame[HWT101_FRAME_LEN];
static uint8_t  hwt_frame_len;
static int32_t  hwt_yaw_mdeg;
static int32_t  hwt_yaw_rate_mdps;   /* 相邻两帧差分的角速度，单位 0.001 deg/s */
static uint32_t hwt_frames;
static uint32_t hwt_bad_frames;
static uint32_t hwt_last_ms;
static uint32_t hwt_prev_ms;
static uint32_t hwt_period_ms;       /* 相邻两帧的间隔，用来确认模块出数率 */
static int32_t  hwt_prev_yaw_mdeg;
static uint8_t  hwt_have_prev;

/* ---- 里程计与点到点导航 ---- */
static int32_t  odom_x_um;              /* 场地坐标，单位 um，原点由 R 命令设定 */
static int32_t  odom_y_um;
static int32_t  odom_heading_mdeg;      /* 相对复位点的航向（车体逆时针为正） */
static int32_t  odom_yaw_offset_mdeg;   /* R 时的原始 yaw，用来把绝对角变成相对角 */
static uint8_t  odom_await_yaw;         /* 1 = 发 R 时还没有陀螺仪帧，等第一帧再定零点 */
static uint32_t odom_last_ms;
static uint8_t  odom_ready;
/* 定点旋转的两级余数：µm·ms 的 /1000 和旋转乘积的 /32768，都跨拍保留防截断。 */
static int64_t  odom_disp_rem_um_ms[2];
static int64_t  odom_field_rem_q15[2];
typedef struct {
  uint8_t  active;
  int8_t   result;         /* 段间结算用：1 = 到点，2 = 超时/中止；0 = 尚无结果 */
  int32_t  tx_um, ty_um;
  int32_t  heading_mdeg;
  uint32_t start_ms, next_ms;
} GotoState;
static GotoState goto_state;
static int32_t goto_dist_mm, goto_vx_mm_s, goto_vy_mm_s;

static void hwt_poll(void)
{
  int value;
  while ((value = Serial_ReadByte(SERIAL_IMU)) >= 0) {
    uint8_t byte = (uint8_t)value;

    if (hwt_frame_len == 0U) {
      if (byte == HWT101_HEADER) hwt_frame[hwt_frame_len++] = byte;
    } else if (hwt_frame_len == 1U) {
      if (byte == HWT101_ANGLE_TYPE || byte == 0x51U || byte == 0x52U) {
        hwt_frame[hwt_frame_len++] = byte;
      } else {
        hwt_frame_len = (byte == HWT101_HEADER) ? 1U : 0U;
      }
    } else {
      hwt_frame[hwt_frame_len++] = byte;
      if (hwt_frame_len == HWT101_FRAME_LEN) {
        uint8_t sum = 0U;
        uint8_t i;
        for (i = 0U; i < HWT101_FRAME_LEN - 1U; ++i) sum = (uint8_t)(sum + hwt_frame[i]);
        if (sum == hwt_frame[HWT101_FRAME_LEN - 1U]) {
          if (hwt_frame[1] == HWT101_ANGLE_TYPE) {
            int16_t yaw_raw = (int16_t)((uint16_t)hwt_frame[6] |
                                        ((uint16_t)hwt_frame[7] << 8));
            int32_t yaw = (int32_t)(((int64_t)yaw_raw * 180000LL) / 32768LL);
            uint32_t now_ms = HAL_GetTick();
            /* 角速度由相邻两帧差分得到：mdeg / ms 正好是 deg/s，再乘 1000 变 mdps。 */
            if (hwt_have_prev != 0U) {
              int32_t delta_mdeg = yaw - hwt_prev_yaw_mdeg;
              uint32_t delta_ms = now_ms - hwt_prev_ms;
              while (delta_mdeg > 180000) delta_mdeg -= 360000;
              while (delta_mdeg < -180000) delta_mdeg += 360000;
              if (delta_ms >= 1U) {
                hwt_period_ms = delta_ms;
                hwt_yaw_rate_mdps = (int32_t)(((int64_t)delta_mdeg * 1000) / (int64_t)delta_ms);
              }
            }
            hwt_prev_yaw_mdeg = yaw;
            hwt_prev_ms = now_ms;
            hwt_have_prev = 1U;
            hwt_yaw_mdeg = yaw;
            hwt_frames++;
            hwt_last_ms = now_ms;
          }
        } else {
          hwt_bad_frames++;
        }
        hwt_frame_len = 0U;
      }
    }
  }
}
/* 陀螺仪是否"活着"：有帧，且最后一帧的帧龄不超过 HEADING_TIMEOUT。
 * 航向相关的自动动作（O / A / GO / LAP）统一用这一条判据，别在每处各写一份，
 * 否则改一处超时就漏掉另一处。 */
static uint8_t hwt_fresh(void)
{
  if (hwt_frames == 0U) return 0U;
  return ((int32_t)(HAL_GetTick() - hwt_last_ms) <=
          (int32_t)WHEEL_SMOKE_HEADING_TIMEOUT_MS) ? 1U : 0U;
}
#if WHEEL_SMOKE_USE_ODOM
static Mpu6050Bias calibration;
#endif

/* X 阶跃测试的状态。全部 static，不进栈。 */
typedef struct {
  uint8_t  active;
  MotorId  wheel;
  int32_t  target;
  int32_t  tail[WHEEL_SMOKE_STEP_TAIL];
  uint8_t  tail_index;
  uint8_t  tail_count;
  int32_t  peak_magnitude;
  uint32_t start_ms;
  uint32_t duration_ms;
  uint32_t next_sample_ms;
  uint32_t rise10_ms;
  uint32_t rise90_ms;
} StepTest;
static StepTest step_test;

/* M 距离运动的状态：到点停车。距离源 odom（投影）/ counts（编码器累计）二选一。
 * 到点不立即断电：先进入 HOLD 阶段（目标清零、闭环保持 WHEEL_SMOKE_MOVE_HOLD_MS），
 * 让速度环把停车滑行顶住，窗口走完才走 move_finish 真正 Wheel_Stop。 */
typedef struct {
  uint8_t active;
  uint8_t axis;            /* WHEEL_SMOKE_MOVE_AXIS_FWD / _LEFT */
  uint8_t use_odom;        /* 1 = 距离按 odom 投影判定（发过 R 且陀螺仪有帧） */
  uint8_t phase;           /* WHEEL_SMOKE_MOVE_PHASE_RUN / _HOLD */
  uint8_t finish_timed_out; /* 进 HOLD 时记下的超时标志，move_finish 回显用 */
  int32_t target_um;
  int32_t speed_um_per_s;
  /* 本段进减速段的剩余距离，单位 mm。起步时按 speed_um_per_s 用 v² 定标算好
   * （见 move_start_axis 与 WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S），
   * 不能拿常量现算 —— 进减速段后 speed_um_per_s 已经被改成慢速值了。 */
  uint16_t slow_mm;
  /* 本段减速段的目标速度，单位 um/s。同样起步时定死：与 slow_mm 按同一个比例
   * （v/参考速度）放大，停车过程才是 0.4 m/s 那套的等比缩放。 */
  int32_t slow_speed_um_per_s;
  int32_t start_counts[4];
  /* odom 模式用：起点场地位姿（um）与投影方向 Q15 —— FWD = 起点航向，
   * LEFT = 航向 +90°（车体左侧）。方向在起步时刻定死，不走"朝向变了就跟着转"。 */
  int32_t start_x_um, start_y_um;
  int32_t axis_cos_q15, axis_sin_q15;
  uint32_t start_ms;
  uint32_t hold_until_ms;  /* HOLD 阶段结束时刻（HAL_GetTick 时基） */
} MoveState;
static MoveState move;
/* 前置声明：定义在下方 odom 区 / task 区，move_start_axis、move_poll 先要用
 * （与 turn_cancel 的先例一致）。 */
static int32_t odom_trig_q15(int32_t angle_mdeg, uint8_t sine);
static void task_abort(const char *reason);

/* TASK 调度状态（定义提前到这里：move_poll 的掉帧中止要读 task.active，
 * 而 move 区在文件前部）。 */
typedef struct {
  uint8_t  active;
  uint8_t  index;
  uint32_t wait_until_ms;
  /* 里程计校验（直线段）：每段发车时记下起点位姿、投影方向、有符号目标，
   * 段结束由 task_poll 用 odom 结算"应到 vs 实到"的沿线/侧向偏差。 */
  uint8_t  seg_check;
  int32_t  origin_x_um, origin_y_um;      /* TASK 发车点，整程闭合误差用 */
  int32_t  seg_start_x_um, seg_start_y_um;
  int32_t  seg_dir_cos_q15, seg_dir_sin_q15;
  int32_t  seg_target_um;
} TaskState;
static TaskState task;

/* 航向保持状态：O 1 时以当前 yaw 为目标，之后每 10 ms 修正 wz。 */
static uint8_t  heading_hold;
static int32_t  heading_target_mdeg;
static int32_t  heading_base_vx_um_per_s;
static int32_t  heading_base_vy_um_per_s;
/* 名义驱动方向：这段运动"本来要走"的车体前向速度，不含横向修正量。
 * 倒车增益切换只看它，不看 base_vx——横移的横向修正写在 base_vx 上、
 * 会在 0 附近正负摆动，拿 base_vx 判倒车会让 Kp/Ki 每拍在两套增益间来回
 * 跳，且积分在"倒车模式"里攒一截、切回来又冻着，棘轮式越偏越多
 * （2026-10-28 实板横移高频抖动+扭转的主因）。 */
static int32_t  heading_drive_vx_um_per_s;
/* wz 限幅的运行时值：横移段（含到点保持）调小，其余路径用宏默认值。
 * move_start_axis / move_enter_hold / move_finish / turn_begin_mdeg / B 命令
 * 负责维护它。 */
static int32_t  heading_wz_limit_mdps = WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS;
static uint32_t heading_next_ms;
static int16_t  heading_kp = WHEEL_SMOKE_HEADING_KP;
static int16_t  heading_ki = WHEEL_SMOKE_HEADING_KI;
static int32_t  heading_integral_mdps;
static int32_t  heading_wz_out;   /* 上一拍真正发出去的 wz，供斜率限制用 */

/* A 原地转的状态：航向环复用 heading_*，这里只管"目标与到位/超时"。 */
static uint8_t  turn_active;
static uint32_t turn_settle_ms;
static uint32_t turn_last_ms;
static uint32_t turn_start_ms;
static int32_t  turn_error_mdeg;
/* 转向起点的位姿，用来在 TURN done 里报"原地转到底跑了多少位置"。
 * 这是评价原地转向质量最直接的指标，比看角度误差更重要。 */
static int32_t  turn_start_x_um;
static int32_t  turn_start_y_um;

/* LAP 正方形跑一圈的状态：四个航点由 start 时算好，段间调 goto_start。 */
static uint8_t lap_active;
static uint8_t lap_index;                       /* 正在进行的段 0..3 */
static int32_t  lap_side_mm;
static int32_t  lap_tx_mm[4], lap_ty_mm[4];     /* 四个航点位置，mm，场地系 */
static const int32_t lap_th_mdeg[4] = {0, 90000, 180000, -90000}; /* 各段航向 */

/* 上电自检转动的状态：0 = 已武装等延迟，1 = 正在转，2 = 结束/已取消。 */
static uint8_t autospin_phase;
#if WHEEL_SMOKE_AUTOSPIN
static uint32_t autospin_start_ms;
#endif

/* 定长缓冲拼接整数，周期路径不引入 printf 家族。 */
static void append_text(const char *text)
{
  while (*text != '\0' && output_length < sizeof(output_line) - 1U)
    output_line[output_length++] = *text++;
}
static void append_unsigned(uint32_t value)
{
  char digits[10];
  uint8_t length = 0U;
  do { digits[length++] = (char)('0' + value % 10U); value /= 10U; } while (value != 0U);
  while (length != 0U && output_length < sizeof(output_line) - 1U)
    output_line[output_length++] = digits[--length];
}
static void append_integer(int32_t value)
{
  if (value < 0) append_text("-");
  append_unsigned(value < 0 ? 0U - (uint32_t)value : (uint32_t)value);
}
static void append_char(char value)
{
  if (output_length < sizeof(output_line) - 1U) output_line[output_length++] = (char)value;
}
/* 十六进制输出，用于 H 命令的寄存器回读。 */
static void append_hex(uint32_t value)
{
  static const char digits[] = "0123456789ABCDEF";
  char buffer[8];
  uint8_t length = 0U;
  do { buffer[length++] = digits[value & 0xFU]; value >>= 4; } while (value != 0U);
  while (length != 0U) append_char(buffer[--length]);
}
static void send_text(const char *message)
{
  uint8_t resume_telemetry = telemetry_enabled;
  /* 用户确认：同口文本先暂停遥测，普通回显结束后恢复。
   * STALL 与 C 调用前已关流，因此不会被这里自动重新开启。 */
  if (WHEEL_SMOKE_TEXT_PORT == WHEEL_SMOKE_TELEMETRY_PORT) telemetry_enabled = 0U;
  (void)Serial_Write(WHEEL_SMOKE_TEXT_PORT, (const uint8_t *)message,
                     (uint16_t)strlen(message), 100U);
  telemetry_enabled = resume_telemetry;
}
static void send_built_text(void)
{
  output_line[output_length] = '\0';
  send_text(output_line);
}
static void append_status_gains(void)
{
  int16_t kp_q10, ki_q10, ff_per_1000cps;
  Wheel_GetGains(&kp_q10, &ki_q10, &ff_per_1000cps);
  append_text(" kp_q10="); append_integer(kp_q10);
  append_text(" ki_q10="); append_integer(ki_q10);
  append_text(" ff="); append_integer(ff_per_1000cps);
}
/* 自动调参脚本的固定文本契约，与人读状态中的标签分开。 */
static void print_gains(void)
{
  int16_t kp_q10, ki_q10, ff_per_1000cps;
  Wheel_GetGains(&kp_q10, &ki_q10, &ff_per_1000cps);
  output_length = 0U;
  append_text("GAINS kp="); append_integer(kp_q10);
  append_text(" ki="); append_integer(ki_q10);
  append_text(" ff="); append_integer(ff_per_1000cps);
  append_text("\r\n"); send_built_text();
}
static void print_status(void)
{
  output_length = 0U;
  for (uint32_t i = 0U; i < 4U; ++i) {
    append_text(" wheel="); append_integer((int32_t)i);
    append_text(" target/actual/output=");
    append_integer(Wheel_TargetCountsPerSec((MotorId)i)); append_text("/");
    append_integer(Encoder_SpeedCountsPerSec((EncoderId)i)); append_text("/");
    append_integer(Wheel_OutputPermille((MotorId)i));
  }
  append_text(" state="); append_integer((int32_t)Wheel_State());
  append_text(" autospin="); append_integer((int32_t)autospin_phase);
  append_text(" stalls="); append_unsigned(Wheel_StallEvents());
  append_text(" hwt_yaw_mdeg="); append_integer(hwt_yaw_mdeg);
  append_text(" hwt_rate_mdps="); append_integer(hwt_yaw_rate_mdps);
  append_text(" hwt_frames="); append_unsigned(hwt_frames);
  append_text(" hwt_bad="); append_unsigned(hwt_bad_frames);
  append_text(" yaw_hold="); append_integer((int32_t)heading_hold);
  append_text(" yaw_target_mdeg="); append_integer(heading_target_mdeg);
  append_text(" yaw_kp="); append_integer(heading_kp);
  append_text(" yaw_ki="); append_integer(heading_ki);
  append_text(" yaw_i_mdps="); append_integer(heading_integral_mdps);
  append_text(" pose_mm="); append_integer(odom_x_um / 1000); append_text("/");
  append_integer(odom_y_um / 1000); append_text("/"); append_integer(odom_heading_mdeg);
  append_text(" goto="); append_integer((int32_t)goto_state.active);
  append_text(" dist_mm="); append_integer(goto_dist_mm);
  append_text(" turn="); append_integer((int32_t)turn_active);
  append_text(" err_mdeg="); append_integer(turn_error_mdeg);
  append_text(" lap="); append_integer((int32_t)lap_active);
  append_text(" leg="); append_integer((int32_t)lap_index + 1);
  append_status_gains();
  append_text(" ol_mode="); append_integer(ol_mode);
  append_text(" ol_permille="); append_integer(ol_permille[0]); append_text("/");
  append_integer(ol_permille[1]); append_text("/");
  append_integer(ol_permille[2]); append_text("/");
  append_integer(ol_permille[3]);
#if WHEEL_SMOKE_USE_ODOM
  {
    ChassisPose pose;
    ChassisOdom_GetPose(&pose);
    append_text(" pose_um/mdeg="); append_integer(pose.x_um); append_text("/");
    append_integer(pose.y_um); append_text("/"); append_integer(pose.heading_mdeg);
    append_text(" velocity_um/s/mdps="); append_integer(pose.vx_um_per_s); append_text("/");
    append_integer(pose.vy_um_per_s); append_text("/"); append_integer(pose.wz_mdps);
  }
  {
    ChassisOdomStatus status;
    ChassisOdom_GetStatus(&status);
    append_text(" gyro_ready="); append_integer(status.gyro_ready);
    append_text(" zupt="); append_integer(status.stationary);
    append_text(" pp_mdps="); append_integer(status.gyro_pp_mdps);
    append_text(" gyro_errors="); append_unsigned(status.gyro_read_errors);
    append_text(" resync="); append_unsigned(status.resync_events);
  }
#endif
  append_text("\r\n"); send_built_text();
}
/* H：把电机输出相关的寄存器原样打出来。
 * 用来证明"固件到底有没有把 PWM 发到管脚上"，把固件问题和硬件问题分开：
 *   CEN=1 计数器在跑、CCER 对应位=1 通道输出使能、MOE=1 高级定时器主输出使能、
 *   CCR1..4 = 当前比较值（8400 = 100%）。三个都是 1 才说明 PWM 真出去了。
 *   GPIOx_ODR 是方向脚/STBY 的最后写入值（复用脚看不出实际电平）。 */
static void print_hardware(void)
{
  output_length = 0U;
  append_text("H TIM1 CEN="); append_hex((TIM1->CR1 & TIM_CR1_CEN) ? 1U : 0U);
  append_text(" CCER=0x"); append_hex(TIM1->CCER);
  append_text(" MOE="); append_hex((TIM1->BDTR & TIM_BDTR_MOE) ? 1U : 0U);
  append_text(" CCR1..4=");
  append_hex(TIM1->CCR1); append_char(','); append_hex(TIM1->CCR2); append_char(',');
  append_hex(TIM1->CCR3); append_char(','); append_hex(TIM1->CCR4);
  append_text(" | ODR E=0x"); append_hex(GPIOE->ODR);
  append_text(" D=0x"); append_hex(GPIOD->ODR);
  append_text(" B=0x"); append_hex(GPIOB->ODR);
  append_text(" C=0x"); append_hex(GPIOC->ODR);
  append_text("\r\n");
  send_built_text();
}

static void send_telemetry(void)
{
  uint32_t i;
  output_length = 0U;
  if (telemetry_mode == 3U) {
    /* 陀螺仪调试：yaw、yaw 角速度、帧龄、累计帧数、校验错帧数（5 通道，无浮点）。 */
    append_integer(hwt_yaw_mdeg); append_text(",");
    append_integer(hwt_yaw_rate_mdps); append_text(",");
    append_unsigned((hwt_frames != 0U) ? (HAL_GetTick() - hwt_last_ms) : 99999U);
    append_text(","); append_unsigned(hwt_frames);
    append_text(","); append_unsigned(hwt_bad_frames);
  } else if (telemetry_mode == 4U) {
    /* 里程计导航：x_mm, y_mm, heading_mdeg, goto, dist_mm, vx_mm_s, vy_mm_s。 */
    append_integer(odom_x_um / 1000); append_text(",");
    append_integer(odom_y_um / 1000); append_text(",");
    append_integer(odom_heading_mdeg); append_text(",");
    append_integer((int32_t)goto_state.active); append_text(",");
    append_integer(goto_dist_mm); append_text(",");
    append_integer(goto_vx_mm_s); append_text(",");
    append_integer(goto_vy_mm_s);
  } else if (telemetry_mode == 0U) {
    append_integer(Wheel_TargetCountsPerSec(selected_wheel)); append_text(",");
    append_integer(Encoder_SpeedCountsPerSec((EncoderId)selected_wheel)); append_text(",");
    append_integer(Wheel_OutputPermille(selected_wheel));
  } else if (telemetry_mode == 1U) {
#if WHEEL_SMOKE_USE_ODOM
    ChassisPose pose;
    ChassisOdom_GetPose(&pose);
    append_integer(pose.x_um); append_text(","); append_integer(pose.y_um); append_text(",");
    append_integer(pose.heading_mdeg); append_text(","); append_integer(pose.vx_um_per_s); append_text(",");
    append_integer(pose.vy_um_per_s); append_text(","); append_integer(pose.wz_mdps); append_text(",");
    append_unsigned(pose.updated_ms);
#else
    /* 里程计没编进来：回退成四轮格式，别发空行。TM 命令也会拒绝选 1。 */
    append_integer(0); append_text(",0,0,0,0,0,0");
#endif
  } else {
    /* 默认：四轮 目标 / 实际 / 输出。 */
    for (i = 0U; i < 4U; ++i) {
      append_integer(Wheel_TargetCountsPerSec((MotorId)i)); append_text(",");
      append_integer(Encoder_SpeedCountsPerSec((EncoderId)i)); append_text(",");
      /* 开环时 Wheel_OutputPermille() 恒为 0，改发指令占空比，否则看波形会以为没输出。 */
      append_integer(ol_mode ? (int32_t)ol_permille[i] : Wheel_OutputPermille((MotorId)i));
      if (i < 3U) append_text(",");
    }
  }
  append_text("\n");
  (void)Serial_Write(WHEEL_SMOKE_TELEMETRY_PORT, (const uint8_t *)output_line,
                     output_length, 50U);
}
static char *skip_space(char *text)
{
  while (*text == ' ' || *text == '\t') ++text;
  return text;
}
/* 每个整数完整消费，超范围或多余字符都拒绝；负数绝对值用无符号处理。 */
static uint8_t parse_integer(char **cursor, int32_t *value)
{
  char *text = skip_space(*cursor);
  uint32_t magnitude = 0U, limit;
  uint8_t negative = 0U, digits = 0U;
  if (*text == '-' || *text == '+') { negative = (*text == '-'); ++text; }
  limit = negative ? 2147483648U : 2147483647U;
  while (*text >= '0' && *text <= '9') {
    uint32_t digit = (uint32_t)(*text++ - '0');
    if (magnitude > (limit - digit) / 10U) return 0U;
    magnitude = magnitude * 10U + digit; digits = 1U;
  }
  if (!digits || (*text != '\0' && *text != ' ' && *text != '\t')) return 0U;
  *value = negative ? (int32_t)(0U - magnitude) : (int32_t)magnitude;
  *cursor = skip_space(text);
  return 1U;
}
static uint8_t within(int32_t value, int32_t limit)
{
  return (value >= -limit && value <= limit) ? 1U : 0U;
}
#if WHEEL_SMOKE_USE_ODOM
static void recalibrate(void)
{
  uint8_t calibration_valid = 0U;
  Wheel_Stop();
  /* 同口时先暂停遥测，保证“必须静止”提示可见；校准后由 T 1 恢复。 */
  telemetry_enabled = 0U;
  send_text("Calibration: MUST remain STILL for about 2 s; wheels stopped.\r\n");
  if (MPU6050_Configure() != HAL_OK ||
      MPU6050_CalibrateBias(&calibration, MPU6050_CALIB_SAMPLES) != HAL_OK ||
      !calibration.calibrated) send_text("ERR gyro calibration; heading unavailable\r\n");
  else {
    calibration_valid = 1U;
    send_text("Gyro calibrated; use T 1 to resume telemetry\r\n");
  }
  MPU6050_ResetFilter();
  {
    ChassisPose pose;
    ChassisOdom_GetPose(&pose);
    /* 丢弃阻塞校准期间窗口，保留当前位置与朝向，不积分两秒旧数据。 */
    ChassisOdom_SetPose(pose.x_um, pose.y_um, pose.heading_mdeg);
    ChassisOdom_SetGyroCalibrationValid(calibration_valid);
  }
  next_tx_ms = HAL_GetTick();
}
#endif
static void print_help(void)
{
#if WHEEL_SMOKE_SINGLE_MOTOR
  send_text("SINGLE-MOTOR closed loop: E 1 | V <a|b|c|d> <counts/s> | T 1 stream\r\n"
            "S status | G [kp ki ff] gains | P/I/F <value> single gain (silent)\r\n"
            "W <wheel> <counts/s> silent target | M <distance_mm> [speed] move\r\n"
            "D <wheel> <permille> open loop | X <wheel> <counts/s> [ms] step\r\n"
            "G [kp ki ff] gains | P/I/F <v> single gain | S status | Y HWT101 yaw\r\n"
            "O 0|1 yaw hold | K/J <v> yaw Kp/Ki | Z zero | STOP | ? help\r\n"
            "Wheel A is the default. Only the selected wheel gets a nonzero target.\r\n");
#else
  send_text("FOUR-WHEEL: E 1 | B <vx_um/s> <vy_um/s> <wz_mdps> body velocity\r\n"
            "  forward B 300000 0 0 | back B -300000 0 0 | left B 0 300000 0\r\n"
            "  right B 0 -300000 0 | spin B 0 0 300000 | stop B 0 0 0\r\n"
            "A <deg> spin in place to relative heading (+ = CCW left / - = CW right)\r\n"
            "LAP [side_mm] square lap, 4 GO legs (send R first; default 1000)\r\n"
            "V <wheel> <counts/s> target | W <wheel> <counts/s> silent target\r\n"
            "M <distance_mm> [speed_um/s] move distance (negative = back)\r\n"
            "ML <mm> [speed] strafe LEFT | MR <mm> [speed] strafe RIGHT\r\n"
            "TASK run built-in path (edit task_steps[]; needs R + HWT101 first)\r\n"
            "D <wheel> <permille> open loop | X <wheel> <counts/s> [ms] step\r\n"
            "G [kp ki ff] gains | P/I/F <v> single gain | S status | Y HWT101 yaw\r\n"
            "O 0|1 yaw hold (B + HWT101) | K/J <v> yaw Kp/Ki | T 0|1 stream\r\n"
            "TM 0|1|2|3|4 telemetry: wheel / odom / four-wheel / gyro / pose | Y gyro detail\r\n"
            "R zero pose | GO <x_mm> <y_mm> [heading_deg]  point-to-point (heading held 0)\r\n"
            "OPEN LOOP: ALL [permille] all four (default 300) | D <wheel> <permille>\r\n"
            "  CMP [permille] [ms] sequential compare | N cumulative counts\r\n");
#if WHEEL_SMOKE_USE_ODOM
  send_text("C gyro zero-bias cal (MUST be STILL)\r\n");
#endif
#endif
}

/* ------------------------- X 阶跃测试（不刷屏） -------------------------
 * 一条命令做完一次试验：给目标 → 按 2 ms 采样实际转速 → 到时停那一路 →
 * 只在文本口回一行摘要。指标就四个：超调、上升时间、稳态值、稳态振荡峰峰值。
 * 非阻塞状态机，STOP 可以中途叫停。 */
static int32_t step_abs(int32_t value) { return value < 0 ? -value : value; }
static char step_wheel_letter(MotorId id) { return (char)('a' + (int32_t)id); }

static void step_test_finish(void)
{
  int32_t sum = 0, low = 0, high = 0, mean, osc_pp = 0, over_pct = 0, duty;
  uint32_t count = step_test.tail_count;
  uint32_t rise_ms = 0U;

  duty = Wheel_OutputPermille(step_test.wheel);
  Wheel_SetTargetCountsPerSec(step_test.wheel, 0);   /* 做完就停，别让它一直转 */
  step_test.active = 0U;

  for (uint32_t i = 0U; i < count; ++i) {
    int32_t value = step_test.tail[i];
    sum += value;
    if (i == 0U || value < low) low = value;
    if (i == 0U || value > high) high = value;
  }
  mean = (count != 0U) ? (sum / (int32_t)count) : 0;
  if (count > 1U) osc_pp = high - low;
  if (step_test.target > 0) {
    over_pct = (step_test.peak_magnitude - step_test.target) * 100 / step_test.target;
    if (over_pct < 0) over_pct = 0;
  }
  if (step_test.rise10_ms != 0U && step_test.rise90_ms >= step_test.rise10_ms)
    rise_ms = step_test.rise90_ms - step_test.rise10_ms;

  output_length = 0U;
  append_text("X "); append_char(step_wheel_letter(step_test.wheel));
  append_char(' '); append_integer(step_test.target);
  append_text(": peak="); append_integer(step_test.peak_magnitude);
  append_text(" over="); append_integer(over_pct); append_char('%');
  append_text(" rise="); append_unsigned(rise_ms); append_text("ms");
  append_text(" steady="); append_integer(mean);
  append_text(" err="); append_integer(mean - step_test.target);
  append_text(" osc_pp="); append_integer(osc_pp);
  append_text(" duty="); append_integer(duty);
  append_text(" tail="); append_unsigned(count);
  append_text("\r\n");
  send_built_text();
}

static void step_test_sample(uint32_t now_ms)
{
  int32_t actual = Encoder_SpeedCountsPerSec((EncoderId)step_test.wheel);
  int32_t magnitude = step_abs(actual);
  uint32_t elapsed_ms = now_ms - step_test.start_ms;

  if (magnitude > step_test.peak_magnitude) step_test.peak_magnitude = magnitude;
  if (step_test.rise10_ms == 0U && magnitude * 10 >= step_test.target)
    step_test.rise10_ms = (elapsed_ms == 0U) ? 1U : elapsed_ms;
  if (step_test.rise90_ms == 0U && magnitude * 10 >= step_test.target * 9)
    step_test.rise90_ms = (elapsed_ms == 0U) ? 1U : elapsed_ms;

  step_test.tail[step_test.tail_index] = actual;
  step_test.tail_index = (uint8_t)((step_test.tail_index + 1U) % WHEEL_SMOKE_STEP_TAIL);
  if (step_test.tail_count < (uint8_t)WHEEL_SMOKE_STEP_TAIL) step_test.tail_count++;
}

static void step_test_poll(uint32_t now_ms)
{
  if (!step_test.active) return;
  if ((int32_t)(now_ms - step_test.next_sample_ms) >= 0) {
    step_test.next_sample_ms = now_ms + (uint32_t)WHEEL_SMOKE_STEP_SAMPLE_MS;
    step_test_sample(now_ms);
  }
  if ((int32_t)(now_ms - step_test.start_ms) >= (int32_t)step_test.duration_ms) step_test_finish();
}

static void step_test_start(MotorId wheel, int32_t target, uint32_t duration_ms)
{
  if (!Wheel_IsEnabled()) {
    send_text("ERR enable first: E 1, then X <a|b|c|d> <counts/s>\r\n");
    return;
  }
  if (target <= 0) {
    send_text("ERR X needs positive counts/s; use V for reverse\r\n");
    return;
  }
  memset(&step_test, 0, sizeof(step_test));
  step_test.wheel = wheel;
  step_test.target = target;
  step_test.duration_ms = duration_ms;
  step_test.start_ms = HAL_GetTick();
  step_test.next_sample_ms = step_test.start_ms;
  step_test.active = 1U;
  selected_wheel = wheel;
  Wheel_SetTargetCountsPerSec(wheel, target);
  output_length = 0U;
  append_text("X "); append_char(step_wheel_letter(wheel)); append_char(' ');
  append_integer(target); append_text(" running "); append_unsigned(duration_ms);
  append_text("ms; STOP aborts\r\n");
  send_built_text();
}
/* ------------------------- M 定距离（里程计投影 / 编码器里程） -------------------------
 * M <distance_mm> [speed_um_per_s]：前进/后退指定距离后自动停。
 * 距离判定二选一，起手时定死、发车回显 mode=：
 *   odom（默认优先）：发过 R 且陀螺仪有帧——把 odom 位移投影到"该轴方向的起点朝向"上，
 *     到投影距离即停。打滑、拧着走都不会误停，还在回显里报告侧向偏移 offset_mm。
 *   counts（回退）：没发 R / 陀螺仪没数 —— 四路编码器累计平均（横移则侧向反解）。
 * 中途陀螺仪掉帧直接中止，不带着停更的位姿继续判距离（与 GO 同款契约）。 */
/* 定距离运动的轴向。FWD 用 vx（前后），LEFT 用 vy（左右，正 = 车体左侧）。 */
#define WHEEL_SMOKE_MOVE_AXIS_FWD   0U
#define WHEEL_SMOKE_MOVE_AXIS_LEFT  1U
/* 运行阶段：RUN = 按距离判停；HOLD = 已到点，闭环保持窗口，结束后才真正断电。 */
#define WHEEL_SMOKE_MOVE_PHASE_RUN   0U
#define WHEEL_SMOKE_MOVE_PHASE_HOLD  1U

static int32_t move_distance_um(void)
{
  uint32_t i;
  int64_t distance_counts;

  if (move.use_odom != 0U) {
    /* 里程计模式：把起点以来的场地位移投影到该轴方向（D = (Δx,Δy)·(cos,sin)）。
     * 投影天然在"拧着走/打滑"时仍按沿线距离计，并保留侧向偏移不掺进判停。 */
    int64_t delta_x = (int64_t)(odom_x_um - move.start_x_um);
    int64_t delta_y = (int64_t)(odom_y_um - move.start_y_um);
    return (int32_t)((delta_x * move.axis_cos_q15 + delta_y * move.axis_sin_q15) / 32768);
  }

  if (move.axis == WHEEL_SMOKE_MOVE_AXIS_LEFT) {
    /* （counts 回退）横移时四轮是两两反向转，平均值为 0，不能用。按车身侧向速度反解：
     *   vy = (-FL + FR + RL - RR) / 4  ->  (-dA + dB + dC - dD) / 4
     * 与 Wheel_SetBodyVelocity() 的 X 型解算严格互逆。 */
    distance_counts = (-(int64_t)(Encoder_ReadCounts(ENCODER_A) - move.start_counts[0]) +
                        (int64_t)(Encoder_ReadCounts(ENCODER_B) - move.start_counts[1]) +
                        (int64_t)(Encoder_ReadCounts(ENCODER_C) - move.start_counts[2]) -
                        (int64_t)(Encoder_ReadCounts(ENCODER_D) - move.start_counts[3])) / 4;
  } else {
    int64_t sum_counts = 0;
    for (i = 0U; i < 4U; ++i)
      sum_counts += (int64_t)(Encoder_ReadCounts((EncoderId)i) - move.start_counts[i]);
    distance_counts = sum_counts / 4;
  }
  return (int32_t)(distance_counts * (int64_t)WHEEL_SMOKE_CIRCUMFERENCE_UM /
                   (int64_t)WHEEL_COUNTS_PER_REV);
}

/* odom 模式专用：起点以来垂直于该轴的偏移，正 = 偏轴左侧。只做回显观测。 */
static int32_t move_cross_offset_um(void)
{
  int64_t delta_x = (int64_t)(odom_x_um - move.start_x_um);
  int64_t delta_y = (int64_t)(odom_y_um - move.start_y_um);
  /* 沿轴单位向量是 (cos,sin)，其左侧垂直向量是 (−sin,cos)。 */
  return (int32_t)((delta_x * -(int64_t)move.axis_sin_q15 +
                    delta_y * move.axis_cos_q15) / 32768);
}

static void move_finish(int32_t distance_um, uint8_t timed_out)
{
  uint32_t elapsed_ms = HAL_GetTick() - move.start_ms;
  move.active = 0U;
  Wheel_Stop();
  /* ML/MR 的航向保持到此收工：不关的话 heading_poll 会拿旧 base_vy 再把车推走。 */
  heading_hold = 0U;
  heading_integral_mdps = 0;
  heading_wz_out = 0;
  heading_drive_vx_um_per_s = 0;
  heading_wz_limit_mdps = WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS;
  output_length = 0U;
  append_text("MOVE done dist_um="); append_integer(distance_um);
  append_text(" target_um="); append_integer(move.target_um);
  append_text(" ms="); append_unsigned(elapsed_ms);
  if (move.use_odom != 0U) {
    /* 里程计模式顺带报侧向偏移：评价这一段走没走斜，比查询位姿直观。 */
    append_text(" offset_mm="); append_integer(move_cross_offset_um() / 1000);
  }
  if (timed_out) append_text(" TIMEOUT");
  append_text("\r\n");
  send_built_text();
}

static void move_start_axis(int32_t distance_mm, int32_t speed_um_per_s, uint8_t axis)
{
  uint32_t i;
  int32_t speed = speed_um_per_s;
  int32_t slow_mm;
  int32_t slow_speed;
  if (speed < 0) speed = -speed;
  if (speed < 10000) speed = WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S;

  /* 减速起点按 v² 定标：参考速度（0.4 m/s）及以下用 WHEEL_SMOKE_MOVE_SLOW_MM 原值，
   * 更快就按 (v/参考速度)² 放大（0.6 m/s -> 90 mm）。用的是起步时的目标速度，
   * 之后 speed_um_per_s 被改成慢速值也不受影响。 */
  slow_mm = WHEEL_SMOKE_MOVE_SLOW_MM;
  if (speed > WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S) {
    int32_t scaled_mm = (int32_t)(((int64_t)WHEEL_SMOKE_MOVE_SLOW_MM * (int64_t)speed *
                                   (int64_t)speed) /
                                  ((int64_t)WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S *
                                   (int64_t)WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S));
    if (scaled_mm > (int32_t)WHEEL_SMOKE_MOVE_SLOW_MAX_MM) {
      scaled_mm = (int32_t)WHEEL_SMOKE_MOVE_SLOW_MAX_MM;
    }
    slow_mm = scaled_mm;
  }
  move.slow_mm = (uint16_t)slow_mm;

  /* 减速段的进场速度也跟着按 v/参考速度 放大（0.06 -> 0.09 m/s）：
   * 距离按 v²、速度按 v 缩放 ⇒ 整个停车过程就是 0.4 m/s 那套的等比放大，
   * 毫米级裕量不变。只放大距离不放大速度的话，末段要爬 90 mm / 0.06 m/s = 1.5 s，
   * 提速省下来的时间会被这段慢爬全吃掉（13 段直线合计约 20 s，比提速省的还多）。 */
  slow_speed = WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S;
  if (speed > WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S) {
    slow_speed = (int32_t)(((int64_t)WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S * (int64_t)speed) /
                           (int64_t)WHEEL_SMOKE_MOVE_SLOW_REF_UM_PER_S);
    /* 进场速度不该超过巡航速度的一半，也不能比原来的慢速还低（防止除出怪值）。 */
    if (slow_speed > speed / 2) slow_speed = speed / 2;
    if (slow_speed < (int32_t)WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S) {
      slow_speed = (int32_t)WHEEL_SMOKE_MOVE_SLOW_SPEED_UM_PER_S;
    }
  }
  move.slow_speed_um_per_s = slow_speed;

  if (!Wheel_IsEnabled()) Wheel_Enable(1U);
  for (i = 0U; i < 4U; ++i) move.start_counts[i] = Encoder_ReadCounts((EncoderId)i);
  /* 发过 R 且陀螺仪有帧 → 距离用 odom 投影判；否则回退编码器累计。起步时定死。 */
  move.use_odom = (odom_ready != 0U && hwt_fresh() != 0U) ? 1U : 0U;
  move.start_x_um = odom_x_um;
  move.start_y_um = odom_y_um;
  {
    /* 投影方向在起步时刻定死：FWD 沿起点航向，LEFT 沿"左侧"（航向 +90°）。
     * 途中车头被航向环修回，投影方向不动，这样判的是"沿线走了多远"。 */
    int32_t dir_mdeg = odom_heading_mdeg;
    if (axis == WHEEL_SMOKE_MOVE_AXIS_LEFT) dir_mdeg += 90000;
    move.axis_cos_q15 = odom_trig_q15(dir_mdeg, 0U);
    move.axis_sin_q15 = odom_trig_q15(dir_mdeg, 1U);
  }
  move.axis = axis;
  move.phase = WHEEL_SMOKE_MOVE_PHASE_RUN;
  move.target_um = distance_mm * 1000;
  move.speed_um_per_s = (distance_mm < 0) ? -speed : speed;
  move.start_ms = HAL_GetTick();
  move.active = 1U;
  /* 航向保持开着时，heading_poll() 每 10 ms 会用 heading_base_vx/vy 重写四轮目标，
   * 这里不同步更新的话，直行会被它按旧速度覆盖掉（甚至直接停住）。 */
  if (axis == WHEEL_SMOKE_MOVE_AXIS_LEFT) {
    heading_base_vx_um_per_s = 0;
    heading_base_vy_um_per_s = move.speed_um_per_s;
    heading_drive_vx_um_per_s = 0;                     /* 横移没有"名义前向" */
    heading_wz_limit_mdps = WHEEL_SMOKE_STRAFE_WZ_LIMIT_MDPS;
    Wheel_SetBodyVelocity(0, move.speed_um_per_s, 0);
  } else {
    heading_base_vx_um_per_s = move.speed_um_per_s;
    heading_base_vy_um_per_s = 0;
    heading_drive_vx_um_per_s = move.speed_um_per_s;
    heading_wz_limit_mdps = WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS;
    Wheel_SetBodyVelocity(move.speed_um_per_s, 0, 0);
  }

  output_length = 0U;
  append_text("MOVE start axis=");
  append_char((axis == WHEEL_SMOKE_MOVE_AXIS_LEFT) ? 'y' : 'x');
  append_text(" target_um="); append_integer(move.target_um);
  append_text(" speed_um_per_s="); append_integer(move.speed_um_per_s);
  /* 新增字段（追加在末尾，前面几项格式不变）：本段的减速距离与进场速度，
   * 用了 v² 缩放后每段都不一样，回显出来才好对着串口判断减速是不是够早。 */
  append_text(" slow_mm="); append_unsigned((uint32_t)move.slow_mm);
  append_text(" slow_um_per_s="); append_integer(move.slow_speed_um_per_s);
  append_text(" mode="); append_text(move.use_odom ? "odom" : "counts");
  append_text("\r\n");
  send_built_text();
}

static void move_start(int32_t distance_mm, int32_t speed_um_per_s)
{
  move_start_axis(distance_mm, speed_um_per_s, WHEEL_SMOKE_MOVE_AXIS_FWD);
}

/* 到点收尾第一步：目标清零但闭环继续开 WHEEL_SMOKE_MOVE_HOLD_MS —— 让速度环
 * 把停车滑行 actively 顶住（横移方向车体沿滚轴自由滑，立即断电溜得最远），
 * 窗口走完由 move_poll 调 move_finish 做真正的 Wheel_Stop 与结算打印。
 * 保持期间航向保持也继续开着（base 已清零，只剩 wz 在收敛），陀螺仪掉线时
 * heading_poll 的超时回退会写 (0,0,0)，行为一致。 */
static void move_enter_hold(uint32_t now_ms, uint8_t timed_out)
{
  move.phase = WHEEL_SMOKE_MOVE_PHASE_HOLD;
  move.finish_timed_out = timed_out;
  move.hold_until_ms = now_ms + (uint32_t)WHEEL_SMOKE_MOVE_HOLD_MS;
  heading_base_vx_um_per_s = 0;
  heading_base_vy_um_per_s = 0;
  heading_drive_vx_um_per_s = 0;   /* 保持期间名义驱动为 0，增益切换不误触发 */
  (void)Wheel_SetBodyVelocity(0, 0, 0);
}

static void move_poll(uint32_t now_ms)
{
  int32_t distance_um, abs_target_um, remaining_um, slow_speed_um_per_s;
  int32_t corr_vx_um_per_s;
  if (!move.active) return;

  /* HOLD 阶段：到点后的闭环保持窗口。目标已清零、环路继续开着，速度环把
   * 停车滑行顶住；窗口走完才算真正收尾（这期间的位移会进 odom，所以收尾
   * 打印和 TASK 结算用的都是保持结束后的最终位姿）。 */
  if (move.phase == WHEEL_SMOKE_MOVE_PHASE_HOLD) {
    if ((int32_t)(now_ms - move.hold_until_ms) >= 0) {
      move.phase = WHEEL_SMOKE_MOVE_PHASE_RUN;   /* 先复位再收尾，防残留状态 */
      move_finish(move_distance_um(), move.finish_timed_out);
    }
    return;
  }

  /* odom 模式的距离完全来自位姿积分：陀螺仪掉帧就中止，
   * 不带着停更的位姿继续判距离（与 GO 的掉线中止同款契约）。
   * TASK 在跑时升级成整条中止——否则下一拍 task_poll 会拿冻结位姿"结算"，
   * 然后把下一段静默降级成 counts 判距接着跑。 */
  if (move.use_odom != 0U && hwt_fresh() == 0U) {
    move.active = 0U;
    Wheel_Stop();
    heading_hold = 0U;
    heading_integral_mdps = 0;
    heading_wz_out = 0;
    if (task.active != 0U) {
      task_abort("gyro frames lost (odom)");
    } else {
      output_length = 0U;
      append_text("MOVE aborted: gyro frames lost (odom)\r\n");
      send_built_text();
    }
    return;
  }

  distance_um = move_distance_um();
  /* 带符号比较：倒车必须投影真的反向到达才算到；方向走反只欠着，
   * 让 60 s 超时兜住并回显实际跑反了多少。 */
  if ((move.target_um >= 0)
      ? (distance_um >= move.target_um)
      : (distance_um <= move.target_um)) {
    move_enter_hold(now_ms, 0U);
    return;
  }
  if ((int32_t)(now_ms - move.start_ms) >= (int32_t)WHEEL_SMOKE_MOVE_TIMEOUT_MS) {
    move_enter_hold(now_ms, 1U);
    return;
  }

  abs_target_um = (move.target_um < 0) ? -move.target_um : move.target_um;
  remaining_um = abs_target_um - ((distance_um < 0) ? -distance_um : distance_um);
  slow_speed_um_per_s = move.slow_speed_um_per_s;
  if (remaining_um <= (int32_t)move.slow_mm * 1000 &&
      (move.speed_um_per_s > slow_speed_um_per_s ||
       move.speed_um_per_s < -slow_speed_um_per_s)) {
    move.speed_um_per_s = (move.target_um < 0) ? -slow_speed_um_per_s : slow_speed_um_per_s;
    if (move.axis == WHEEL_SMOKE_MOVE_AXIS_LEFT) {
      /* 横移慢速段：base_vx（横向修正）与四轮目标由下面的横向位置环每拍统一处理 */
      heading_base_vy_um_per_s = move.speed_um_per_s;
    } else {
      heading_base_vx_um_per_s = move.speed_um_per_s;
      heading_drive_vx_um_per_s = move.speed_um_per_s;
      heading_base_vy_um_per_s = 0;
      Wheel_SetBodyVelocity(move.speed_um_per_s, 0, 0);
    }
  }

  /* 横移横向位置环：把垂直于横移轴的偏移按比例折算成车体前向修正速度。
   * cross 正 = 偏到运动轴左侧；横移轴是"航向 +90°"，它的左侧垂直向量指向
   * 车体后方，所以 cross 正 = 车体后向漂移 → 给正的车体前向速度拉回线上。
   * 每拍刷新：航向保持开着时由 heading_poll 统一下发（它每 10 ms 用
   * base_vx/vy/wz 重写四轮目标），没开就直接下发，否则没人重写四轮目标。 */
  corr_vx_um_per_s = 0;
  if (move.axis == WHEEL_SMOKE_MOVE_AXIS_LEFT && move.use_odom != 0U) {
    int32_t cross_um = move_cross_offset_um();
    if (cross_um > (int32_t)WHEEL_SMOKE_STRAFE_CROSS_MAX_MM * 1000)
      cross_um = (int32_t)WHEEL_SMOKE_STRAFE_CROSS_MAX_MM * 1000;
    if (cross_um < -(int32_t)WHEEL_SMOKE_STRAFE_CROSS_MAX_MM * 1000)
      cross_um = -(int32_t)WHEEL_SMOKE_STRAFE_CROSS_MAX_MM * 1000;
    corr_vx_um_per_s = (cross_um *
                        (int32_t)WHEEL_SMOKE_STRAFE_CROSS_GAIN_UM_PER_S_PER_MM) / 1000;
    if (corr_vx_um_per_s > (int32_t)WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S)
      corr_vx_um_per_s = (int32_t)WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S;
    if (corr_vx_um_per_s < -(int32_t)WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S)
      corr_vx_um_per_s = -(int32_t)WHEEL_SMOKE_STRAFE_CORR_MAX_UM_PER_S;
    heading_base_vx_um_per_s = corr_vx_um_per_s;
    if (heading_hold == 0U)
      (void)Wheel_SetBodyVelocity(corr_vx_um_per_s, move.speed_um_per_s, 0);
  }
}

static void heading_poll(uint32_t now_ms)
{
  int32_t error, wz, kp_eff, ki_eff;
  uint8_t reversing;
  if (!heading_hold) return;
  if ((int32_t)(now_ms - heading_next_ms) < 0) return;
  heading_next_ms = now_ms + 10U;

  if ((int32_t)(now_ms - hwt_last_ms) > (int32_t)WHEEL_SMOKE_HEADING_TIMEOUT_MS) {
    heading_hold = 0U;
    heading_wz_out = 0;
    Wheel_SetBodyVelocity(heading_base_vx_um_per_s, heading_base_vy_um_per_s, 0);
    return;
  }

  error = heading_target_mdeg - hwt_yaw_mdeg;
  while (error > 180000) error -= 360000;
  while (error < -180000) error += 360000;
  if (error > -(int32_t)WHEEL_SMOKE_HEADING_DEADBAND_MDEG &&
      error < (int32_t)WHEEL_SMOKE_HEADING_DEADBAND_MDEG) error = 0;

  /* 倒车方向单独一套增益：快衰减下同样 wz 只转出约 43% 的角速度，
   * 环路增益不够就会留下比正转大一倍多的稳态偏航。正转 / A 原地转不受影响。
   * 判据用名义驱动方向 drive_vx：base_vx 里含横移的横向修正（0 附近摆动），
   * 拿它判会把修正量误判成"倒车"，增益来回跳 + 积分棘轮（见变量注释）。 */
  reversing = (heading_drive_vx_um_per_s < 0) ? 1U : 0U;
  kp_eff = heading_kp;
  ki_eff = heading_ki;
  if (reversing != 0U) {
    kp_eff = (kp_eff * (int32_t)WHEEL_SMOKE_HEADING_REV_GAIN_PERMILLE) / 1000;
    ki_eff = (int32_t)WHEEL_SMOKE_HEADING_REV_KI;
  }

  if (ki_eff != 0 && error != 0 &&
      (WHEEL_SMOKE_HEADING_INT_ZONE_MDEG == 0 ||
       (error < (int32_t)WHEEL_SMOKE_HEADING_INT_ZONE_MDEG &&
        error > -(int32_t)WHEEL_SMOKE_HEADING_INT_ZONE_MDEG))) {
    heading_integral_mdps += (error * ki_eff) / 100;
    if (heading_integral_mdps > (int32_t)WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS)
      heading_integral_mdps = (int32_t)WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS;
    if (heading_integral_mdps < -(int32_t)WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS)
      heading_integral_mdps = -(int32_t)WHEEL_SMOKE_HEADING_INT_LIMIT_MDPS;
  }

  wz = error * kp_eff * (int32_t)WHEEL_SMOKE_YAW_SIGN + heading_integral_mdps;
  /* 限幅用运行时值：横移段被 move_start_axis 调小，别用 80 deg/s 的权限去修
   * 横移中的小扰动（会把打滑扭动放大成可见扭转）。 */
  if (wz > heading_wz_limit_mdps)
    wz = heading_wz_limit_mdps;
  if (wz < -heading_wz_limit_mdps)
    wz = -heading_wz_limit_mdps;

  /* 输出斜率限制：别让 wz 一拍从 0 跳到 80 deg/s。左右静摩擦不同，阶跃起步时慢的
   * 一侧会滞后，车头被另一侧拽着走 —— 原地转向跑位置主要就是这一段。 */
  if (WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS != 0) {
    if (wz > heading_wz_out + (int32_t)WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS)
      wz = heading_wz_out + (int32_t)WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS;
    else if (wz < heading_wz_out - (int32_t)WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS)
      wz = heading_wz_out - (int32_t)WHEEL_SMOKE_HEADING_WZ_SLEW_MDPS;
  }
  heading_wz_out = wz;

  Wheel_SetBodyVelocity(heading_base_vx_um_per_s, heading_base_vy_um_per_s, wz);
}

/* ---- 上电自检转动：开环直写四路，不碰闭环 ---- */
static void autospin_write(uint8_t on)
{
  if (on != 0U) { Motor_Standby(0U, 1U); Motor_Standby(1U, 1U); }
  Wheel_BrakeCancel();   /* 自检是开环直写，不能被刹车计时器清掉 */
  for (uint8_t i = 0U; i < 4U; ++i) {
    (void)Motor_Set((MotorId)i, (on != 0U) ? (int16_t)WHEEL_SMOKE_AUTOSPIN_PERMILLE : 0);
  }
  if (on == 0U) { Motor_Standby(0U, 0U); Motor_Standby(1U, 0U); }
}
static void autospin_cancel(void)
{
  if (autospin_phase == 1U) {
    autospin_write(0U);
    send_text("AUTOSPIN stopped\r\n");
  }
  autospin_phase = 2U;
}
static void autospin_poll(uint32_t now_ms)
{
#if WHEEL_SMOKE_AUTOSPIN
  if (autospin_phase == 0U) {
    if ((int32_t)(now_ms - autospin_start_ms) >= (int32_t)WHEEL_SMOKE_AUTOSPIN_DELAY_MS) {
      autospin_write(1U);
      autospin_phase = 1U;
      output_length = 0U;
      append_text("AUTOSPIN: 4 wheels FORWARD, OPEN LOOP ");
      append_integer((int32_t)WHEEL_SMOKE_AUTOSPIN_PERMILLE);
      append_text(" permille. Any command stops it.\r\n");
      send_built_text();
    }
  } else if (autospin_phase == 1U) {
#if WHEEL_SMOKE_AUTOSPIN_MS > 0
    if ((int32_t)(now_ms - autospin_start_ms) >=
        (int32_t)(WHEEL_SMOKE_AUTOSPIN_DELAY_MS + WHEEL_SMOKE_AUTOSPIN_MS)) {
      autospin_write(0U);
      autospin_phase = 2U;
      send_text("AUTOSPIN done (timeout)\r\n");
    }
#endif
  }
#else
  (void)now_ms;
#endif
}

/* ------------------------- 开环电机台架（ALL / CMP / N） -------------------------
 * 这组命令不经过 Wheel_Update：先 Wheel_Enable(0) 关掉闭环（顺带把输出清零），
 * 再直接 Motor_Set()。所以读数只反映电机 / 驱动 / 动力电这一段，不会被 PI 和积分遮住。 */
static HAL_StatusTypeDef ol_output_one(uint8_t wheel, int16_t permille)
{
  HAL_StatusTypeDef status;
  uint8_t i;
  /* 开环接管：先撤掉可能还在跑的停机刹车，并把另外三路放开，
   * 否则刹车计时器到点会把这里设的占空比一起清掉。 */
  if (Wheel_IsBraking() != 0U) { Wheel_BrakeCancel(); Motor_StopAll(); }
  status = Motor_Set((MotorId)wheel, permille);
  for (i = 0U; i < 4U; ++i) ol_permille[i] = (i == wheel) ? permille : 0;
  if (status == HAL_OK) { ol_last_cmd_ms = HAL_GetTick(); ol_mode = 1U; }
  return status;
}

static void ol_output_all(int16_t permille)
{
  uint8_t i;
  Wheel_BrakeCancel();   /* 同为开环接管，见 ol_output_one 的注释 */
  for (i = 0U; i < 4U; ++i) {
    (void)Motor_Set((MotorId)i, permille);
    ol_permille[i] = permille;
  }
  ol_last_cmd_ms = HAL_GetTick();
  ol_mode = 1U;
}

static void ol_off(void)
{
  uint8_t i;
  /* 闭环层刚做了停机刹车的话别再 Motor_StopAll()：那会把电机的短接制动变回滑行，
   * 刹车窗口就白设了。等 Wheel_Update() 的计时器到点自己松。 */
  if (Wheel_IsBraking() == 0U) Motor_StopAll();
  for (i = 0U; i < 4U; ++i) ol_permille[i] = 0;
  ol_mode = 0U;
  ol_cmp.active = 0U;
}

/* N：四路累计计数。手转减速箱输出轴一整圈，正常应接近 WHEEL_COUNTS_PER_REV（1320）。 */
static void ol_print_counts(void)
{
  uint8_t i;
  output_length = 0U;
  append_text("COUNTS");
  for (i = 0U; i < 4U; ++i) {
    append_char(' '); append_char((char)('A' + i)); append_char('=');
    append_integer(Encoder_ReadCounts((EncoderId)i));
  }
  append_text("\r\n");
  send_built_text();
}

static void ol_cmp_start(int16_t permille, uint32_t duration_ms)
{
  Wheel_Enable(0U);                 /* 关掉闭环，否则下一拍 10 ms 就把开环输出覆盖了 */
  memset(&ol_cmp, 0, sizeof(ol_cmp));
  ol_cmp.permille = permille;
  ol_cmp.duration_ms = duration_ms;
  ol_cmp.wheel_until_ms = HAL_GetTick() + duration_ms;
  ol_cmp.active = 1U;
  ol_output_all(0);                 /* 先四路全 0，再从 A 开始一路一路来 */
  (void)ol_output_one(0U, permille);
  output_length = 0U;
  append_text("CMP "); append_integer(permille); append_text(" permille, ");
  append_unsigned(duration_ms);
  append_text(" ms each; ANY command aborts, E 1 restores loop\r\n");
  send_built_text();
}

static void ol_poll(uint32_t now_ms)
{
  if (ol_cmp.active != 0U) {
    int32_t speed = Encoder_SpeedCountsPerSec((EncoderId)ol_cmp.wheel);
    if ((int32_t)(now_ms - ol_cmp.wheel_until_ms) < 0) {
      /* 只累计末尾窗口，避开启动段。 */
      if ((int32_t)(now_ms - (ol_cmp.wheel_until_ms - (uint32_t)WHEEL_SMOKE_OL_TAIL_MS)) >= 0) {
        ol_cmp.sum_counts_per_s += speed;
        ol_cmp.samples++;
      }
      return;
    }
    ol_cmp.result_counts_per_s[ol_cmp.wheel] =
        (ol_cmp.samples != 0U) ? (ol_cmp.sum_counts_per_s / (int32_t)ol_cmp.samples) : 0;
    (void)Motor_Set((MotorId)ol_cmp.wheel, 0);
    ol_permille[ol_cmp.wheel] = 0;
    ol_cmp.wheel++;
    if (ol_cmp.wheel >= 4U) {
      int16_t done_permille = ol_cmp.permille;
      uint8_t i;
      ol_off();
      output_length = 0U;
      append_text("CMP "); append_integer(done_permille); append_text(" permille: ");
      for (i = 0U; i < 4U; ++i) {
        append_char((char)('A' + i)); append_char('=');
        append_integer(ol_cmp.result_counts_per_s[i]);
        if (i < 3U) append_char(' ');
      }
      append_text(" counts/s\r\n");
      send_built_text();
      return;
    }
    ol_cmp.sum_counts_per_s = 0;
    ol_cmp.samples = 0U;
    ol_cmp.wheel_until_ms = now_ms + ol_cmp.duration_ms;
    (void)ol_output_one(ol_cmp.wheel, ol_cmp.permille);
    return;
  }
  /* 开环输出非零时的呆滞保护：太久没命令就自动停，防止忘记关。 */
  if (WHEEL_SMOKE_OL_IDLE_STOP_MS != 0U && ol_mode != 0U) {
    if ((int32_t)(now_ms - ol_last_cmd_ms) >= (int32_t)WHEEL_SMOKE_OL_IDLE_STOP_MS) {
      ol_off();
      send_text("OL idle timeout, all outputs off\r\n");
    }
  }
}

/* ------------------------- 里程计 + 点到点导航（R / GO） -------------------------
 * 航向由 HWT101 给（先用 10 Hz 也能跑），位移由四路编码器的线速度积分。
 * 逆运动学反解（与 Wheel_SetBodyVelocity 的 X 型解算严格互逆）：
 *   vx = (FL + FR + RL + RR) / 4
 *   vy = (-FL + FR + RL - RR) / 4
 * 位置积分按"上一拍航向"把车体位移旋转进场地系（2026-10-27 补，
 * 之前只在航向恒 0 时成立，跑一圈必然出错）；每段控制时把航向先稳定到
 * 段航向，所以控制端仍直接给 vx/vy，不需要旋转矩阵。 */
static int32_t wrap_mdeg(int32_t value)
{
  while (value > 180000) value -= 360000;
  while (value < -180000) value += 360000;
  return value;
}

/* 整数开方（逐位法），避免浮点。 */
static uint32_t isqrt_u32(uint32_t value)
{
  uint32_t root = 0U;
  uint32_t bit = 1UL << 30;
  while (bit > value) bit >>= 2;
  while (bit != 0U) {
    if (value >= root + bit) {
      value -= root + bit;
      root = (root >> 1) + bit;
    } else {
      root >>= 1;
    }
    bit >>= 2;
  }
  return root;
}

/* 0..90° 的 sin 值，Q15（32767×sinθ），1° 步距 + 线性插值。
 * 这里是台架临时实现；正式量产位姿积分应走 App 的 chassis_odom（257 项 Q8 插值）。 */
static const int16_t odom_sin_table_q15[91] = {
    0,  572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126, 5690, 6252, 6813,
 7371, 7927, 8481, 9032, 9580,10126,10668,11207,11743,12275,12803,13328,13848,
14364,14876,15383,15886,16383,16876,17364,17846,18323,18794,19260,19720,20173,
20621,21062,21497,21925,22347,22762,23170,23571,23964,24351,24730,25101,25465,
25821,26169,26509,26841,27165,27481,27788,28087,28377,28659,28932,29196,29451,
29697,29934,30162,30381,30591,30791,30982,31163,31335,31498,31650,31794,31927,
32051,32165,32269,32364,32448,32523,32587,32642,32687,32722,32747,32762,32767,
};
static int32_t odom_trig_q15(int32_t angle_mdeg, uint8_t sine)
{
  int32_t angle = wrap_mdeg(angle_mdeg);      /* -180000..179999 */
  int32_t sign = 1;
  if (sine == 0U) angle = wrap_mdeg(angle + 90000);   /* cos = sin(angle+90) */
  if (angle < 0) { angle = -angle; sign = -1; }       /* 负角镜像，sin 为奇函数 */
  if (angle > 90000) angle = 180000 - angle;          /* 90..180 折回 0..90，符号恒为正 */
  if (angle >= 90000) return sign * 32767;            /* 正好在端点，避免读越界 */
  /* 0..90000 mdeg → Q8 索引：256 步对应 1000 mdeg，Q8 小数线性插值。 */
  {
    int32_t index_q8 = (int32_t)(((int64_t)angle * 256) / 1000);
    int32_t base = odom_sin_table_q15[index_q8 >> 8];
    return sign * (base +
        (int32_t)(((int64_t)(odom_sin_table_q15[(index_q8 >> 8) + 1] - base) *
                   (int64_t)(index_q8 & 255)) / 256));
  }
}
/* 车体系位移旋转进场地系：x = dx·cosH − dy·sinH，y = dx·sinH + dy·cosH（Q15）。 */
static void odom_rotate_integrate(int32_t dx_local_um, int32_t dy_local_um)
{
  int32_t sin_q15 = odom_trig_q15(odom_heading_mdeg, 1U);
  int32_t cos_q15 = odom_trig_q15(odom_heading_mdeg, 0U);
  odom_field_rem_q15[0] += (int64_t)dx_local_um * cos_q15 -
                           (int64_t)dy_local_um * sin_q15;
  odom_field_rem_q15[1] += (int64_t)dx_local_um * sin_q15 +
                           (int64_t)dy_local_um * cos_q15;
  odom_x_um += (int32_t)(odom_field_rem_q15[0] / 32768);
  odom_y_um += (int32_t)(odom_field_rem_q15[1] / 32768);
  odom_field_rem_q15[0] %= 32768;
  odom_field_rem_q15[1] %= 32768;
}

/* R：把当前位置记成原点、当前航向记成 0。上电或换场地后必须先发一次。 */
static void odom_reset(void)
{
  odom_x_um = 0;
  odom_y_um = 0;
  odom_heading_mdeg = 0;
  /* 此刻还没有任何陀螺仪帧时，hwt_yaw_mdeg 是未初始化的 0，不能拿它当零点：
   * 一旦第一帧到达，odom_heading_mdeg 就会从 0 突跳到当时的绝对 yaw。
   * 这种情况把零点推迟到第一帧（odom_await_yaw），期间 odom_poll 不积分位置。 */
  if (hwt_frames == 0U) {
    odom_await_yaw = 1U;
    odom_yaw_offset_mdeg = 0;
  } else {
    odom_await_yaw = 0U;
    odom_yaw_offset_mdeg = hwt_yaw_mdeg;
  }
  odom_last_ms = HAL_GetTick();
  odom_disp_rem_um_ms[0] = 0;
  odom_disp_rem_um_ms[1] = 0;
  odom_field_rem_q15[0] = 0;
  odom_field_rem_q15[1] = 0;
  odom_ready = 1U;
  goto_state.active = 0U;
  goto_state.result = 0;
  lap_active = 0U;
  goto_dist_mm = 0;
  goto_vx_mm_s = 0;
  goto_vy_mm_s = 0;
}

static void odom_poll(uint32_t now_ms)
{
  uint32_t dt_ms;
  int32_t fl, fr, rl, rr, vx_um_s, vy_um_s;
  if (odom_ready == 0U) return;
  dt_ms = now_ms - odom_last_ms;
  if (dt_ms < WHEEL_SMOKE_ODOM_PERIOD_MS) return;
  odom_last_ms = now_ms;

  /* 发 R 时陀螺仪还没出数：等第一帧到了再锁存航向零点。等帧期间不积分位置，
   * 因为此时航向未知，按任意航向积出来的位姿都是错的。 */
  if (odom_await_yaw != 0U) {
    if (hwt_frames == 0U) return;
    odom_yaw_offset_mdeg = hwt_yaw_mdeg;
    odom_await_yaw = 0U;
  }

  fl = Encoder_SpeedUmPerSec(ENCODER_A);
  fr = Encoder_SpeedUmPerSec(ENCODER_B);
  rl = Encoder_SpeedUmPerSec(ENCODER_C);
  rr = Encoder_SpeedUmPerSec(ENCODER_D);
  vx_um_s = (fl + fr + rl + rr) / 4;
  vy_um_s = (-fl + fr + rl - rr) / 4;
  /* 车体位移 ×dt，保留 /1000 余数后旋转进场地系（防止低速每拍截断成 0）。 */
  odom_disp_rem_um_ms[0] += (int64_t)vx_um_s * (int64_t)dt_ms;
  odom_disp_rem_um_ms[1] += (int64_t)vy_um_s * (int64_t)dt_ms;
  /* 取旋转前的航向（本拍新航向在下一行才更新），即窗口起点姿态。 */
  odom_rotate_integrate((int32_t)(odom_disp_rem_um_ms[0] / 1000),
                        (int32_t)(odom_disp_rem_um_ms[1] / 1000));
  odom_disp_rem_um_ms[0] %= 1000;
  odom_disp_rem_um_ms[1] %= 1000;
  /* 陀螺仪装反就把 WHEEL_SMOKE_YAW_SIGN 改 -1；这里统一换算成"车体逆时针为正"。 */
  odom_heading_mdeg = wrap_mdeg((hwt_yaw_mdeg - odom_yaw_offset_mdeg) *
                                (int32_t)WHEEL_SMOKE_YAW_SIGN);
}

static void goto_stop_with(const char *message)
{
  (void)Wheel_SetBodyVelocity(0, 0, 0);
  goto_state.active = 0U;
  goto_state.result = 2;          /* 超时或被中止：LAP 据此放弃后续段 */
  goto_vx_mm_s = 0;
  goto_vy_mm_s = 0;
  if (message != NULL) send_text(message);
}

static void goto_start(int32_t tx_mm, int32_t ty_mm, int32_t heading_mdeg)
{
  if (odom_ready == 0U) { send_text("ERR send R first (zero the pose)\r\n"); return; }
  if (hwt_fresh() == 0U) { send_text("ERR no fresh HWT101 frames\r\n"); return; }
  if (Wheel_IsEnabled() == 0U) Wheel_Enable(1U);
  /* 导航和别的自动动作互斥：航向保持、定距离、阶跃测试都先停掉。 */
  heading_hold = 0U;
  move.active = 0U;
  step_test.active = 0U;
  goto_state.tx_um = tx_mm * 1000;
  goto_state.ty_um = ty_mm * 1000;
  goto_state.heading_mdeg = heading_mdeg;
  goto_state.start_ms = HAL_GetTick();
  goto_state.next_ms = goto_state.start_ms;
  goto_state.active = 1U;
  goto_state.result = 0;
  goto_dist_mm = 0;
  output_length = 0U;
  append_text("GO x_mm="); append_integer(tx_mm);
  append_text(" y_mm="); append_integer(ty_mm);
  append_text(" h_mdeg="); append_integer(heading_mdeg);
  append_text(" vmax_mm_s="); append_unsigned(WHEEL_SMOKE_GOTO_SPEED_MAX_MM_S);
  append_text("\r\n");
  send_built_text();
}

static void goto_poll(uint32_t now_ms)
{
  int32_t ex_mm, ey_mm, dist_mm, eh_mdeg, vx_mm_s, vy_mm_s, wz_mdps;
  int32_t speed_mm_s, vmax_mm_s;
  if (goto_state.active == 0U) return;

  /* 陀螺仪掉线：odom_heading_mdeg 会冻在最后一个值上，闭环仍然"自认为"航向正确，
   * 继续走就是把车往错方向带。跑到一半掉线必须立刻停，不能等 30 s 段超时。
   * goto_stop_with 会置 result=2，LAP 据此把整圈作废。 */
  if (hwt_fresh() == 0U) {
    goto_stop_with("GOAL aborted: gyro frames lost\r\n");
    return;
  }

  if ((int32_t)(now_ms - goto_state.start_ms) >= (int32_t)WHEEL_SMOKE_GOTO_TIMEOUT_MS) {
    goto_stop_with("GOAL timeout, stopped\r\n");
    return;
  }
  if ((int32_t)(now_ms - goto_state.next_ms) < 0) return;
  goto_state.next_ms = now_ms + WHEEL_SMOKE_GOTO_PERIOD_MS;

  ex_mm = (goto_state.tx_um - odom_x_um) / 1000;
  ey_mm = (goto_state.ty_um - odom_y_um) / 1000;
  dist_mm = (int32_t)isqrt_u32((uint32_t)(ex_mm * ex_mm + ey_mm * ey_mm));
  goto_dist_mm = dist_mm;

  if (dist_mm <= (int32_t)WHEEL_SMOKE_GOTO_TOL_MM) {
    (void)Wheel_SetBodyVelocity(0, 0, 0);
    goto_state.active = 0U;
    goto_state.result = 1;        /* 到点：单发 GO 也一样，只是没人消费 */
    goto_vx_mm_s = 0;
    goto_vy_mm_s = 0;
    output_length = 0U;
    append_text("GOAL x_mm="); append_integer(odom_x_um / 1000);
    append_text(" y_mm="); append_integer(odom_y_um / 1000);
    append_text(" heading_mdeg="); append_integer(odom_heading_mdeg);
    append_text("\r\n");
    send_built_text();
    return;
  }

  eh_mdeg = wrap_mdeg(goto_state.heading_mdeg - odom_heading_mdeg);
  if (eh_mdeg > (int32_t)WHEEL_SMOKE_GOTO_TURN_FIRST_MDEG ||
      eh_mdeg < -(int32_t)WHEEL_SMOKE_GOTO_TURN_FIRST_MDEG) {
    /* 航向差太大：先原地转正，别一边走一边甩。 */
    vx_mm_s = 0;
    vy_mm_s = 0;
  } else {
    vx_mm_s = ex_mm * WHEEL_SMOKE_GOTO_POS_GAIN;
    vy_mm_s = ey_mm * WHEEL_SMOKE_GOTO_POS_GAIN;
    speed_mm_s = (int32_t)isqrt_u32((uint32_t)(vx_mm_s * vx_mm_s + vy_mm_s * vy_mm_s));
    vmax_mm_s = (dist_mm < (int32_t)WHEEL_SMOKE_GOTO_SLOW_DIST_MM)
                  ? (int32_t)WHEEL_SMOKE_GOTO_SPEED_SLOW_MM_S
                  : (int32_t)WHEEL_SMOKE_GOTO_SPEED_MAX_MM_S;
    if (speed_mm_s > vmax_mm_s && speed_mm_s != 0) {
      vx_mm_s = vx_mm_s * vmax_mm_s / speed_mm_s;
      vy_mm_s = vy_mm_s * vmax_mm_s / speed_mm_s;
    }
  }

  wz_mdps = eh_mdeg * WHEEL_SMOKE_GOTO_HEAD_GAIN;
  if (wz_mdps > (int32_t)WHEEL_SMOKE_GOTO_WZ_LIMIT_MDPS)
    wz_mdps = (int32_t)WHEEL_SMOKE_GOTO_WZ_LIMIT_MDPS;
  if (wz_mdps < -(int32_t)WHEEL_SMOKE_GOTO_WZ_LIMIT_MDPS)
    wz_mdps = -(int32_t)WHEEL_SMOKE_GOTO_WZ_LIMIT_MDPS;

  goto_vx_mm_s = vx_mm_s;
  goto_vy_mm_s = vy_mm_s;
  (void)Wheel_SetBodyVelocity(vx_mm_s * 1000, vy_mm_s * 1000, wz_mdps);
}

/* ------------------------- LAP 正方形跑一圈 -------------------------
 * 起点即当前位置，场地轴 = 发 R 时静止的车体轴。四段分别纯前进：
 *   (S,0,0°) → (S,S,90°) → (0,S,180°) → (0,0,−90°)，每段"先原地转正、再直行"，
 * 不走横移（横移另行排查）。段间结算靠 goto_state.result。 */
static void turn_cancel(void);   /* turn 模块在后面；LAP 先用到，前置声明 */

static void lap_cancel(void)
{
  lap_active = 0U;
  goto_state.active = 0U;     /* 掐掉正在走的段，否则 goto_poll 会一直改写轮速 */
  goto_vx_mm_s = 0;
  goto_vy_mm_s = 0;
}

static void lap_start(int32_t side_mm)
{
  int32_t x_mm, y_mm;
  if (odom_ready == 0U) { send_text("ERR send R first (zero the pose)\r\n"); return; }
  if (hwt_fresh() == 0U) { send_text("ERR no fresh HWT101 frames\r\n"); return; }
  /* 与别的自动动作互斥：M / X / A / 航向保持都让位给 LAP（goto 自带航向控制）。 */
  move.active = 0U; step_test.active = 0U; turn_cancel();
  if (!Wheel_IsEnabled()) Wheel_Enable(1U);
  x_mm = odom_x_um / 1000;
  y_mm = odom_y_um / 1000;
  lap_side_mm = side_mm;
  lap_tx_mm[0] = x_mm + side_mm; lap_ty_mm[0] = y_mm;
  lap_tx_mm[1] = x_mm + side_mm; lap_ty_mm[1] = y_mm + side_mm;
  lap_tx_mm[2] = x_mm;           lap_ty_mm[2] = y_mm + side_mm;
  lap_tx_mm[3] = x_mm;           lap_ty_mm[3] = y_mm;
  lap_index = 0U;
  lap_active = 1U;
  goto_start(lap_tx_mm[0], lap_ty_mm[0], lap_th_mdeg[0]);
  output_length = 0U;
  append_text("LAP side_mm="); append_integer(side_mm);
  append_text(" leg headings deg: 0 90 180 270 (CCW)\r\n");
  send_built_text();
}

static void lap_poll(void)
{
  if (lap_active == 0U) return;
  if (goto_state.active != 0U) return;            /* 当前段还在走 */
  if (goto_state.result != 1) {                   /* 超时/被中止：整圈作废 */
    lap_active = 0U;
    (void)Wheel_SetBodyVelocity(0, 0, 0);
    output_length = 0U;
    append_text("LAP aborted at leg "); append_integer((int32_t)lap_index + 1);
    append_text(" pose_mm="); append_integer(odom_x_um / 1000);
    append_text("/"); append_integer(odom_y_um / 1000);
    append_text("\r\n"); send_built_text();
    return;
  }
  lap_index++;
  if (lap_index >= 4U) {
    lap_active = 0U;
    output_length = 0U;
    append_text("LAP done side_mm="); append_integer(lap_side_mm);
    append_text(" final_mm="); append_integer(odom_x_um / 1000);
    append_text("/"); append_integer(odom_y_um / 1000);
    append_text(" heading_mdeg="); append_integer(odom_heading_mdeg);
    append_text("\r\n"); send_built_text();
    return;
  }
  goto_start(lap_tx_mm[lap_index], lap_ty_mm[lap_index], lap_th_mdeg[lap_index]);
  output_length = 0U;
  append_text("LAP leg "); append_integer((int32_t)lap_index + 1);
  append_text("/4\r\n"); send_built_text();
}

/* ------------------------- A 原地转到相对角度 -------------------------
 *   A +90 左转、A -90 右转（车体逆时针为正）。航向环就是 heading_poll 的
 *   P+I+死区 限幅，这里只负责：设目标、盯到位、超时/丢陀螺仪时收工。
 * 完成语义：连续 TURN_SETTLE 毫秒内 |误差| <= 2° 且角速度 <= 15 deg/s，然后全停。 */
static void turn_cancel(void)
{
  turn_active = 0U;
  heading_hold = 0U;          /* A 的航向目标随之失效；O 命令可重新接管 */
  turn_settle_ms = 0U;
  turn_error_mdeg = 0;
  heading_wz_out = 0;
}

/* 转向起步核心：增量给 mdeg（车体逆时针为正）。deg 版与绝对角版都走这里。 */
static void turn_begin_mdeg(int32_t delta_mdeg)
{
  if (hwt_fresh() == 0U) {
    send_text("ERR no fresh HWT101 frames\r\n");
    return;
  }
  /* 与别的自动动作互斥：M / X / LAP 让位给 A。 */
  move.active = 0U; step_test.active = 0U; lap_cancel();
  if (!Wheel_IsEnabled()) Wheel_Enable(1U);
  heading_base_vx_um_per_s = 0;
  heading_base_vy_um_per_s = 0;
  heading_drive_vx_um_per_s = 0;
  heading_wz_limit_mdps = WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS;
  heading_integral_mdps = 0;
  heading_wz_out = 0;
  turn_start_x_um = odom_x_um;
  turn_start_y_um = odom_y_um;
  /* 目标是 HWT101 的原始 yaw：车体增量 × YAW_SIGN（与 odom_reset 的换算同向）。 */
  heading_target_mdeg = hwt_yaw_mdeg + delta_mdeg * (int32_t)WHEEL_SMOKE_YAW_SIGN;
  heading_hold = 1U;
  heading_next_ms = HAL_GetTick();
  turn_start_ms = heading_next_ms;
  turn_last_ms = heading_next_ms;
  turn_settle_ms = 0U;
  turn_active = 1U;
}

static void turn_start(int32_t delta_deg)
{
  turn_begin_mdeg(delta_deg * 1000);
  output_length = 0U;
  append_text("TURN deg="); append_integer(delta_deg);
  append_text(" target_yaw_mdeg="); append_integer(heading_target_mdeg);
  append_text("\r\n"); send_built_text();
}

/* TASK 用：转到"当前里程计坐标系"的绝对航向（0 = 最近一次 R/RESET 时刻的车头）。
 * 相对转角会把上一次没转准的残差累加进下一次——实板首跑 8 个 90° 净攒出约 3.6°
 * 的地图旋转，1850 长边横向弓出约 100 mm；绝对角每次都把残差修回来，不跨段累积。 */
static void turn_start_abs(int32_t abs_deg)
{
  int32_t delta_mdeg = wrap_mdeg(abs_deg * 1000 - odom_heading_mdeg);
  turn_begin_mdeg(delta_mdeg);
  output_length = 0U;
  append_text("TURN abs_deg="); append_integer(abs_deg);
  append_text(" delta_mdeg="); append_integer(delta_mdeg);
  append_text(" target_yaw_mdeg="); append_integer(heading_target_mdeg);
  append_text("\r\n"); send_built_text();
}

static void turn_poll(uint32_t now_ms)
{
  uint32_t delta_ms;
  int32_t error_mdeg;
  if (turn_active == 0U) return;
  if (heading_hold == 0U) {          /* 航向环被别人关掉了（陀螺仪掉线等） */
    turn_cancel();
    (void)Wheel_SetBodyVelocity(0, 0, 0);
    output_length = 0U;
    append_text("TURN aborted: yaw hold off\r\n"); send_built_text();
    return;
  }
  error_mdeg = wrap_mdeg(heading_target_mdeg - hwt_yaw_mdeg);
  turn_error_mdeg = error_mdeg;
  delta_ms = now_ms - turn_last_ms;
  if (delta_ms > 20U) delta_ms = 20U;   /* 主循环偶尔卡住也不虚增计数 */
  turn_last_ms = now_ms;
  if (error_mdeg < (int32_t)WHEEL_SMOKE_TURN_TOL_MDEG &&
      error_mdeg > -(int32_t)WHEEL_SMOKE_TURN_TOL_MDEG &&
      hwt_yaw_rate_mdps < (int32_t)WHEEL_SMOKE_TURN_RATE_TOL_MDPS &&
      hwt_yaw_rate_mdps > -(int32_t)WHEEL_SMOKE_TURN_RATE_TOL_MDPS) {
    turn_settle_ms += delta_ms;
  } else {
    turn_settle_ms = 0U;
  }
  if (turn_settle_ms >= WHEEL_SMOKE_TURN_SETTLE_MS) {
    turn_active = 0U;
    heading_hold = 0U;
    heading_integral_mdps = 0;
    heading_wz_out = 0;
    (void)Wheel_SetBodyVelocity(0, 0, 0);
    output_length = 0U;
    append_text("TURN done err_mdeg="); append_integer(turn_error_mdeg);
    append_text(" ms="); append_unsigned(now_ms - turn_start_ms);
    /* 原地转的位移漂移：起点到现在的 odom 差。这个数比角度误差更能说明转向质量。 */
    append_text(" drift_mm="); append_integer((odom_x_um - turn_start_x_um) / 1000);
    append_char(','); append_integer((odom_y_um - turn_start_y_um) / 1000);
    append_text("\r\n"); send_built_text();
    return;
  }
  if ((int32_t)(now_ms - turn_start_ms) >= (int32_t)WHEEL_SMOKE_TURN_TIMEOUT_MS) {
    turn_active = 0U;
    heading_hold = 0U;
    heading_wz_out = 0;
    (void)Wheel_SetBodyVelocity(0, 0, 0);
    output_length = 0U;
    append_text("TURN timeout err_mdeg="); append_integer(turn_error_mdeg);
    append_text(" drift_mm="); append_integer((odom_x_um - turn_start_x_um) / 1000);
    append_char(','); append_integer((odom_y_um - turn_start_y_um) / 1000);
    append_text("\r\n"); send_built_text();
  }
}

/* ------------------------- TASK：步骤表驱动的顺序执行 -------------------------
 * 一条命令跑完整条路径。每一步只做一件事：直线（前/后/左/右）、原地转、等待、
 * 位姿重置（RESET，见 WHEEL_SMOKE_TASK_RESET 注释）。段间靠 move.active /
 * turn_active 结算；直线段的判停与偏差结算全部走 odom（开工需先发 R）。
 * ⚠ 路径写死在这张表里，改路线就是改表 + 重编译；**没有循环步骤**——
 * 要跑 K 圈就把整圈步骤手工展开 K 份（每份结尾放 RESET 就是"每圈清误差"）。 */
#define WHEEL_SMOKE_TASK_FWD    0U
#define WHEEL_SMOKE_TASK_BACK   1U
#define WHEEL_SMOKE_TASK_LEFT   2U
#define WHEEL_SMOKE_TASK_RIGHT  3U
#define WHEEL_SMOKE_TASK_TURN   4U
#define WHEEL_SMOKE_TASK_WAIT   5U
/* RESET：在可信锚点（扫码/地面二维码确认"车已到位"）后调用，等价于步骤内置的 R：
 * odom 位姿清零、航向基准重取，此后 seg 结算照常，closure 从最近一次 RESET 起算。
 * 用途：把几圈之间累积的航位推算误差清掉——每圈过一次锚点，误差就不出这一圈。
 * ⚠ 只有确认车真的停在锚点上才允许编排这一步；错位重置等于把锚点焊在错的地方。 */
#define WHEEL_SMOKE_TASK_RESET  6U
/* TURN_ABS：绝对角转身，value = 目标航向（deg，当前里程计坐标系，0 = 最近一次
 * R/RESET 时刻的车头方向，正 = 逆时针）。与相对 TURN 的区别见 turn_start_abs()。 */
#define WHEEL_SMOKE_TASK_TURN_ABS 7U

typedef struct {
  uint8_t kind;
  int16_t value;      /* FWD/BACK/LEFT/RIGHT: mm；TURN: deg；WAIT: ms */
} TaskStep;

/* 直线段的车体速度，单位 um/s。对应手动发 `B 600000 0 0`（0.6 m/s）。
 * 600000 um/s 折合每轮约 2063 counts/s（810 计数/转、轮径 75 mm，满速约 4495）。
 * 按 board_wheel.h 的前馈口径 125‰/1000cps + 摩擦项 70‰ 估，稳态占空比约 330‰，
 * 弱的那一路（C 所在 U3 的 A 半边）约 380~420‰ —— 离 WHEEL_DUTY_LIMIT_PERMILLE
 * (600) 还有余量，但已不是"随便加"的区间：再加就要先看四路 output 的实测数。
 * 横移另给一个更小的值：麦轮横移的效率只有正向的一半左右，同样占空比走不出同样的速度，
 * 用 0.6 m/s 很容易四路一起顶到 600‰ 限幅。 */
/* 跑图速度：直行 / 横移，单位 um/s（2026-10-28 横移由 200000 提到 300000，
 * 用户要求加快左右运动；2026-10-29 直行由 400000 提到 600000，用户要求加快前进）。
 * 转身步不用速度（航向环自定）；WAIT 步用 WHEEL_SMOKE_MAP_*_WAIT_MS。
 * ⚠ 提速的代价是过点停车与走廊余量：走廊净宽 400、车 300，余量只有 ±50 mm，
 *   减速段已按 v² 自动放大（见 move_start_axis），但首跑要用 TASK done 的
 *   段末偏差与 closure_mm 核对，冲过头就先把本宏降到 500000。 */
#define WHEEL_SMOKE_TASK_SPEED_UM_PER_S   600000
#define WHEEL_SMOKE_TASK_STRAFE_UM_PER_S  300000
/* 上限从 500000 放到 800000 —— 600000 已经越过原来的闸。留到这里是因为再高
 * 就不是调参而是换驱动了（2063 counts/s 已用掉约 46% 的满轮速）。 */
#if (WHEEL_SMOKE_TASK_SPEED_UM_PER_S < 10000 || WHEEL_SMOKE_TASK_SPEED_UM_PER_S > 800000)
#error WHEEL_SMOKE_TASK_SPEED_UM_PER_S must be within 10000..800000
#endif
/* ⚠ 本宏 WHEEL_SMOKE_TASK_BACK 也在用。倒车是 AT8236 的快衰减弱方向
 * （见 board_wheel.h 的 WHEEL_FF_REVERSE_BOOST_PERMILLE），同转速要多给 1.7 倍占空比，
 * 0.6 m/s 倒车很可能四路一起顶到 600‰ 限幅、闭环失效。当前路线表里没有 BACK 步
 * （所有倒着走的地方都改成了 FWD + TURN_ABS 180），所以现在不受影响；
 * 以后要加 BACK 步，先把本宏降到 400000，或者继续照现在的写法改成转身前进。 */
#if (WHEEL_SMOKE_TASK_STRAFE_UM_PER_S < 10000 || WHEEL_SMOKE_TASK_STRAFE_UM_PER_S > 500000)
#error WHEEL_SMOKE_TASK_STRAFE_UM_PER_S must be within 10000..500000
#endif

/* ------------------------- 比赛路线 v0.3（2026-10-28 用户修订，逐段复核通过） -------------------------
 * v0.1 图上推算作废（发车位 y 推错 300 mm）；v0.2 按用户首版清单实现。
 * v0.3 = 用户修订清单：第 8 步保留"顺时针180"（转完朝东），去粗加工的
 *   1850 改成横移（面东右=南）——v0.2 的两处修正随之取消：不再需要改
 *   逆时针90；第 23 步转回朝南后 24/26/27 无转身，终点车头=朝南=发车
 *   朝向，收尾转身免了（27 步闭环，整表可原样重跑 = 循环）。
 * 用户清单两处笔误（此时车头均朝南，与旧清单"面西"语境不同）：
 *   第 26 步"向后1950（向东）"→ 向后=北会出界，改 向左1950（向东）；
 *   第 27 步"向右1050（向北）"→ 向右=西回不到发车位，改 后退1050（向北）。
 * 实测几何：发车位 (2250,2250) 朝南，车道中心距墙约 300；扫码点 (2100,1200)、
 * 原料区 (1200,2150)、粗加工区 (1200,300)、暂存区 (300,1200)。
 * 车外形 300×300（用户 2026-10-28 确认）：中央 400 间隙走廊余量 ±50；
 * 本版 9 处穿越走廊全是横移/倒车（车长横在走廊里；300×300 两个方向余量相同），
 * 但横移的前向溜车正好顶向走廊壁——上板前必须先做 MR 900 溜车探测与
 * ML 左向核对。如擦碰把 WHEEL_SMOKE_TASK_SPEED_UM_PER_S（直行）/
 * WHEEL_SMOKE_TASK_STRAFE_UM_PER_S（横移）调低（全表生效）。
 * ⚠ 本表无 RESET（按用户清单）；closure_mm 即整程闭合误差，期望 (0,0)。 */
/* 各段长度与等待（2026-10-28 用户实测定案；单位 mm / ms，改路线只改这些数字）。
 * 宏名与用户清单逐条对应（括号内 = 车体动作与世界方向）：
 *   ALIGN        向右(西) 150：出发对中，x → 车道中心 2100
 *   QR_DROP      向前(南) 1050：发车位 → 扫码点 (2100,1200)
 *   SCAN_BACK    后退(北) 950：扫码点倒回原料纬度 y=2150
 *   RAW_LEG      向前(西) 900：→ 原料区 (1200,2150)
 *   RAW_DROP     向右(南) 1850：原料 → 粗加工 (1200,300)，横穿中央纵向走廊
 *   LANE_CROSS   向左(北) 900：粗加工 → 中央交点 (1200,1200)
 *   STORE_LEG    向右(西) 900：中央交点 → 暂存区 (300,1200)，横穿中央横向走廊
 *   STORE_EXIT   向左(东) 900：暂存 → 中央交点 (1200,1200)
 *   RAW2_LEG     向右(北) 950：中央交点 → 原料区，横穿中央纵向走廊
 *   RETURN_EAST  向左(东) 1950：暂存东行回右车道 x=2250，横穿中央横向走廊
 *   RETURN_NORTH 后退(北) 1050：右车道倒车回发车位 (2250,2250) */
#define WHEEL_SMOKE_MAP_FIELD_MM        2400    /* 场地边长（仅作校验基准） */
#define WHEEL_SMOKE_MAP_ALIGN_MM        150
#define WHEEL_SMOKE_MAP_QR_DROP_MM      1050
#define WHEEL_SMOKE_MAP_SCAN_BACK_MM    950
#define WHEEL_SMOKE_MAP_RAW_LEG_MM      900
#define WHEEL_SMOKE_MAP_RAW_DROP_MM     1850
#define WHEEL_SMOKE_MAP_LANE_CROSS_MM   900
#define WHEEL_SMOKE_MAP_STORE_LEG_MM    900
#define WHEEL_SMOKE_MAP_STORE_EXIT_MM   900
#define WHEEL_SMOKE_MAP_RAW2_LEG_MM     950
#define WHEEL_SMOKE_MAP_RETURN_EAST_MM  1950
#define WHEEL_SMOKE_MAP_RETURN_NORTH_MM 1050
#define WHEEL_SMOKE_MAP_SCAN_WAIT_MS    1000    /* 扫码点停 1 s */
#define WHEEL_SMOKE_MAP_WORK_WAIT_MS    3000    /* 各工作区停 3 s */
#if (WHEEL_SMOKE_MAP_FIELD_MM < 1000 || WHEEL_SMOKE_MAP_FIELD_MM > 10000)
#error WHEEL_SMOKE_MAP_FIELD_MM out of sane range
#endif
#if (WHEEL_SMOKE_MAP_ALIGN_MM <= 0 || WHEEL_SMOKE_MAP_QR_DROP_MM <= 0 || \
     WHEEL_SMOKE_MAP_SCAN_BACK_MM <= 0 || WHEEL_SMOKE_MAP_RAW_LEG_MM <= 0 || \
     WHEEL_SMOKE_MAP_RAW_DROP_MM <= 0 || WHEEL_SMOKE_MAP_LANE_CROSS_MM <= 0 || \
     WHEEL_SMOKE_MAP_STORE_LEG_MM <= 0 || WHEEL_SMOKE_MAP_STORE_EXIT_MM <= 0 || \
     WHEEL_SMOKE_MAP_RAW2_LEG_MM <= 0 || WHEEL_SMOKE_MAP_RETURN_EAST_MM <= 0 || \
     WHEEL_SMOKE_MAP_RETURN_NORTH_MM <= 0)
#error every route segment must be positive
#endif
#if (WHEEL_SMOKE_MAP_SCAN_WAIT_MS <= 0 || WHEEL_SMOKE_MAP_WORK_WAIT_MS <= 0)
#error wait times must be positive
#endif

/* 路线表：发车 (2250,2250) 朝南 → 对中 → 扫码 (2100,1200) → 原料 (1200,2150)
 * → 粗加工 (1200,300) → 暂存 (300,1200) → 原料 → 粗加工 → 暂存 → 回发车位。
 * 转身全部 TURN_ABS（0° = R 时刻车头 = 朝南，90=东 180=北 270=西），
 * 本版转身序列 270/90/0/270/90/0。 */
static const TaskStep task_steps[] = {
  /* —— 出发对中：右移 150，x → 车道中心 2100 —— */
  {WHEEL_SMOKE_TASK_RIGHT, WHEEL_SMOKE_MAP_ALIGN_MM},
  /* —— 南行 1050 到扫码点 (2100,1200) —— */
  {WHEEL_SMOKE_TASK_FWD,   WHEEL_SMOKE_MAP_QR_DROP_MM},
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_SCAN_WAIT_MS},    /* 扫码窗口 */
  /* —— 倒车 950 退出扫码位，回到原料纬度 y=2150 —— */
	{WHEEL_SMOKE_TASK_TURN_ABS, 180},
  {WHEEL_SMOKE_TASK_FWD,  WHEEL_SMOKE_MAP_SCAN_BACK_MM},
  {WHEEL_SMOKE_TASK_TURN_ABS, 270},                  /* 朝南 → 朝西（顺时针90） */
  /* —— 西行 900 到原料区 (1200,2150) —— */
  {WHEEL_SMOKE_TASK_FWD,   WHEEL_SMOKE_MAP_RAW_LEG_MM},
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_WORK_WAIT_MS},    /* 原料加工 */
	
  {WHEEL_SMOKE_TASK_TURN_ABS,  0},                  /* 朝西 → 朝东（顺时针180） */
  /* —— 右移 1850 南行到粗加工区 (1200,300)，车长横穿中央纵向走廊 —— */
  {WHEEL_SMOKE_TASK_FWD, WHEEL_SMOKE_MAP_RAW_DROP_MM},
	{WHEEL_SMOKE_TASK_TURN_ABS,  90},
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_WORK_WAIT_MS},    /* 粗加工 */
  /* —— 左移 900 北行到中央交点 (1200,1200) —— */
	{WHEEL_SMOKE_TASK_TURN_ABS,  180},
  {WHEEL_SMOKE_TASK_FWD,  WHEEL_SMOKE_MAP_LANE_CROSS_MM},
	{WHEEL_SMOKE_TASK_TURN_ABS,  270},
	{WHEEL_SMOKE_TASK_FWD, WHEEL_SMOKE_MAP_STORE_LEG_MM},
  {WHEEL_SMOKE_TASK_TURN_ABS,   0},                  /* 朝东 → 朝南（顺时针90） */
  /* —— 右移 900 西行到暂存区 (300,1200)，车长横穿中央横向走廊 —— */
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_WORK_WAIT_MS},    /* 暂存 */
  /* —— 左移 900 东行回中央交点 (1200,1200) —— */
	{WHEEL_SMOKE_TASK_TURN_ABS, 90},      
  {WHEEL_SMOKE_TASK_FWD,  WHEEL_SMOKE_MAP_STORE_EXIT_MM},
  {WHEEL_SMOKE_TASK_TURN_ABS, 180},                  /* 朝南 → 朝西（顺时针90） */
  /* —— 右移 950 北行二进原料区 (1200,2150)，车长横穿中央纵向走廊 —— */
  {WHEEL_SMOKE_TASK_FWD, WHEEL_SMOKE_MAP_RAW2_LEG_MM},
	{WHEEL_SMOKE_TASK_TURN_ABS, 270},     
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_WORK_WAIT_MS},    /* 原料加工 */
	{WHEEL_SMOKE_TASK_TURN_ABS,  0},
	{WHEEL_SMOKE_TASK_FWD, WHEEL_SMOKE_MAP_RAW_DROP_MM},
  {WHEEL_SMOKE_TASK_TURN_ABS,  90},                  /* 朝西 → 朝东（顺时针180） */
  /* —— 右移 1850 南行二进粗加工区 (1200,300) —— */
  
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_WORK_WAIT_MS},    /* 粗加工 */
	
	
  {WHEEL_SMOKE_TASK_TURN_ABS,  180},
  {WHEEL_SMOKE_TASK_FWD,  WHEEL_SMOKE_MAP_LANE_CROSS_MM},
	{WHEEL_SMOKE_TASK_TURN_ABS,  270},
	{WHEEL_SMOKE_TASK_FWD, WHEEL_SMOKE_MAP_STORE_LEG_MM},
  {WHEEL_SMOKE_TASK_TURN_ABS,   0},                  /* 朝东 → 朝南（顺时针90） */
  /* —— 右移 900 西行到暂存区 (300,1200)，车长横穿中央横向走廊 —— */
  {WHEEL_SMOKE_TASK_WAIT,  WHEEL_SMOKE_MAP_WORK_WAIT_MS},    /* 暂存 */
	{WHEEL_SMOKE_TASK_TURN_ABS, 90},  
	
  /* —— 左移 1950 东行回右车道 x=2250，车长横穿中央横向走廊 —— */
  {WHEEL_SMOKE_TASK_FWD,  WHEEL_SMOKE_MAP_RETURN_EAST_MM},
	{WHEEL_SMOKE_TASK_TURN_ABS, 180},  
  /* —— 倒车 1050 北行回发车位 (2250,2250)，车头保持朝南 —— */
  {WHEEL_SMOKE_TASK_FWD,  WHEEL_SMOKE_MAP_RETURN_NORTH_MM},
	
};
#define WHEEL_SMOKE_TASK_COUNT (sizeof(task_steps) / sizeof(task_steps[0]))

/* 开始一段直线前先把航向保持打开，目标取当前 yaw。
 * 每次 turn 结束 heading_hold 会被清掉，不重新打开的话下一段直线就会漂。 */
static void task_hold_heading(void)
{
  if (hwt_fresh() == 0U) return;
  heading_hold = 1U;
  heading_target_mdeg = hwt_yaw_mdeg;
  heading_integral_mdps = 0;
  heading_wz_out = 0;
  heading_next_ms = HAL_GetTick();
}

static void task_abort(const char *reason)
{
  task.active = 0U;
  task.seg_check = 0U;
  heading_hold = 0U;
  move.active = 0U;
  turn_cancel();
  (void)Wheel_SetBodyVelocity(0, 0, 0);
  output_length = 0U;
  append_text("TASK aborted at step "); append_integer((int32_t)task.index + 1);
  append_text(": "); append_text(reason); append_text("\r\n");
  send_built_text();
}

/* 每段直线发车成功后把"应到终点"与投影方向记进 task：段结束由 task_poll 结算。
 * move.use_odom==0（理论上不会出现——task_start 已挡）时不校验。 */
static void task_record_seg(void)
{
  if (move.active == 0U || move.use_odom == 0U) return;
  task.seg_check = 1U;
  task.seg_start_x_um = move.start_x_um;
  task.seg_start_y_um = move.start_y_um;
  task.seg_dir_cos_q15 = move.axis_cos_q15;
  task.seg_dir_sin_q15 = move.axis_sin_q15;
  task.seg_target_um = move.target_um;
}

static uint8_t task_run_step(void)
{
  const TaskStep *step = &task_steps[task.index];

  switch (step->kind) {
    case WHEEL_SMOKE_TASK_FWD:
    case WHEEL_SMOKE_TASK_BACK:
      task_hold_heading();
      if (step->kind == WHEEL_SMOKE_TASK_FWD)
        move_start(step->value, (int32_t)WHEEL_SMOKE_TASK_SPEED_UM_PER_S);
      else
        move_start(-(int32_t)step->value, (int32_t)WHEEL_SMOKE_TASK_SPEED_UM_PER_S);
      task_record_seg();
      return (move.active != 0U) ? 1U : 0U;

    case WHEEL_SMOKE_TASK_LEFT:
    case WHEEL_SMOKE_TASK_RIGHT:
      task_hold_heading();
      move_start_axis((step->kind == WHEEL_SMOKE_TASK_LEFT) ? (int32_t)step->value
                                                            : -(int32_t)step->value,
                      (int32_t)WHEEL_SMOKE_TASK_STRAFE_UM_PER_S,
                      WHEEL_SMOKE_MOVE_AXIS_LEFT);
      task_record_seg();
      return (move.active != 0U) ? 1U : 0U;

    case WHEEL_SMOKE_TASK_TURN:
      turn_start(step->value);
      return (turn_active != 0U) ? 1U : 0U;

    case WHEEL_SMOKE_TASK_TURN_ABS:
      turn_start_abs(step->value);
      return (turn_active != 0U) ? 1U : 0U;

    case WHEEL_SMOKE_TASK_RESET:
      /* 位姿重置锚点：扫码确认到位后清掉累积误差。odom_reset 本身非阻塞。 */
      odom_reset();
      task.origin_x_um = 0;
      task.origin_y_um = 0;
      output_length = 0U;
      append_text("TASK reset (pose zeroed at step ");
      append_integer((int32_t)task.index + 1);
      append_text("); closure counts from here\r\n");
      send_built_text();
      return 1U;

    case WHEEL_SMOKE_TASK_WAIT:
      task.wait_until_ms = HAL_GetTick() + (uint32_t)step->value;
      return 1U;

    default:
      return 0U;
  }
}

static void task_start(void)
{
  /* 跑图全程用 odom 判距（每段起动前的 mode=odom 由 move_start_axis 定）：
   * 没发 R 或陀螺仪没帧就不开工，避免"前几段 counts、后几段 odom"的混合语义。 */
  if (odom_ready == 0U) { send_text("ERR TASK needs R first (odometry)\r\n"); return; }
  if (hwt_fresh() == 0U) { send_text("ERR TASK needs live HWT101 frames\r\n"); return; }
  if (Wheel_IsEnabled() == 0U) Wheel_Enable(1U);
  task.origin_x_um = odom_x_um;
  task.origin_y_um = odom_y_um;
  task.index = 0U;
  task.active = 1U;
  task.wait_until_ms = 0U;
  task.seg_check = 0U;
  output_length = 0U;
  append_text("TASK start steps="); append_unsigned((uint32_t)WHEEL_SMOKE_TASK_COUNT);
  append_text("\r\n"); send_built_text();
  if (task_run_step() == 0U) task_abort("step start failed");
}

static void task_poll(uint32_t now_ms)
{
  const TaskStep *step;

  if (task.active == 0U) return;
  /* 当前段还在跑就走开。直线和转向是两套独立的状态机，谁在跑就等谁。 */
  if (move.active != 0U || turn_active != 0U) return;

  /* 刚完成的直线段：用 odom 结算"应到 vs 实到"的沿线/侧向偏差。
   * err_along 是终点偏差在段方向上的分量（段没停准/超冲都体现在这里），
   * side 是终点偏离段直线的距离（走斜了体现在这里）。两级都留给下一级的修正。 */
  if (task.seg_check != 0U) {
    int64_t delta_x = (int64_t)(odom_x_um - task.seg_start_x_um);
    int64_t delta_y = (int64_t)(odom_y_um - task.seg_start_y_um);
    int32_t along = (int32_t)((delta_x * task.seg_dir_cos_q15 +
                              delta_y * task.seg_dir_sin_q15) / 32768);
    int32_t side  = (int32_t)((delta_x * -(int64_t)task.seg_dir_sin_q15 +
                               delta_y * task.seg_dir_cos_q15) / 32768);
    output_length = 0U;
    append_text("TASK seg "); append_integer((int32_t)task.index + 1);
    append_text("/"); append_unsigned((uint32_t)WHEEL_SMOKE_TASK_COUNT);
    append_text(" err_along_mm="); append_integer((along - task.seg_target_um) / 1000);
    append_text(" side_mm="); append_integer(side / 1000);
    append_text("\r\n"); send_built_text();
    task.seg_check = 0U;
  }

  step = &task_steps[task.index];
  if (step->kind == WHEEL_SMOKE_TASK_WAIT &&
      (int32_t)(now_ms - task.wait_until_ms) < 0) return;

  task.index++;
  if (task.index >= (uint8_t)WHEEL_SMOKE_TASK_COUNT) {
    task.active = 0U;
    output_length = 0U;
    append_text("TASK done steps="); append_unsigned((uint32_t)WHEEL_SMOKE_TASK_COUNT);
    append_text(" pose_mm="); append_integer(odom_x_um / 1000);
    append_text("/"); append_integer(odom_y_um / 1000);
    /* 整程闭合误差：终点相对 TASK 发车点的偏差。理想（正方形回到起点）= 0,0。 */
    append_text(" closure_mm="); append_integer((odom_x_um - task.origin_x_um) / 1000);
    append_char(','); append_integer((odom_y_um - task.origin_y_um) / 1000);
    append_text("\r\n"); send_built_text();
    return;
  }

  output_length = 0U;
  append_text("TASK step "); append_integer((int32_t)task.index + 1);
  append_text("/"); append_unsigned((uint32_t)WHEEL_SMOKE_TASK_COUNT);
  append_text("\r\n"); send_built_text();

  if (task_run_step() == 0U) task_abort("step start failed");
}

static void run_command(CommandInput *input)
{
  char *text = skip_space(input->line), *cursor;
  int32_t first, second, third;
  size_t length;
  if (input->overflow) { send_text("ERR command too long\r\n"); return; }
  length = strlen(text);
  while (length && (text[length-1U] == ' ' || text[length-1U] == '\t')) text[--length] = '\0';
  for (size_t i = 0U; i < length; ++i)
    if (text[i] >= 'a' && text[i] <= 'z') text[i] = (char)(text[i] - 'a' + 'A');
  if (!length) return;
  /* 任何命令都取消上电自检转动（含 S/? 这类只读命令），行为可预测。 */
  autospin_cancel();
  /* 同理：任何命令都中止 CMP 并清掉开环残留输出；CMP 自己会在下面重新开始。 */
  if (ol_cmp.active) { ol_cmp.active = 0U; ol_off(); }
  /* 开环期间任何命令都算"有人还在操作"，刷新呆滞计时。 */
  if (ol_mode) ol_last_cmd_ms = HAL_GetTick();
  /* A / LAP 是自动动作：发任何运动/初始化命令都中止（S/?/G/Y/H/TM 等只读命令除外）。
   * A 里也会做互斥清理，这里只是在它生效之前把上一轮 cleanly 摘掉。 */
  if ((turn_active || lap_active || task.active) &&
      (text[0] == 'B' || text[0] == 'V' || text[0] == 'W' || text[0] == 'D' ||
       text[0] == 'E' || text[0] == 'X' || text[0] == 'M' || text[0] == 'Z' ||
       text[0] == 'R' || text[0] == 'A' || text[0] == 'L' ||
       (text[0] == 'G' && text[1] == 'O') || !strcmp(text, "STOP"))) {
    turn_cancel();
    lap_cancel();
    task.active = 0U;   /* TASK 是这些自动动作的调度者，被它们打断时一起收工 */
  }
  if (move.active && (text[0] == 'B' || text[0] == 'V' || text[0] == 'W' ||
                      text[0] == 'D' || text[0] == 'E' || text[0] == 'X' ||
                      text[0] == 'Z')) move.active = 0U;
  /* E 1 不取消航向保持：它是"先使能、再保持"的常规步骤；只有 E 0（失能）才取消，见下面 E 分支。
   * M 的匹配必须排除 ML/MR：横移分支自己会重开航向保持，不能在这里一并清掉。 */
  if (heading_hold &&
      (text[0] == 'V' || text[0] == 'W' || text[0] == 'D' || text[0] == 'X' ||
       (text[0] == 'M' && text[1] != 'L' && text[1] != 'R') ||
       !strcmp(text, "STOP")))
    heading_hold = 0U;
  if (!strcmp(text, "STOP")) {
    move.active = 0U; step_test.active = 0U; Wheel_Stop(); ol_off();
    goto_state.active = 0U; goto_vx_mm_s = 0; goto_vy_mm_s = 0;
    send_text("Stopped\r\n"); return;
  }
  if (!strcmp(text, "?")) { print_help(); return; }
  if (!strcmp(text, "G")) { print_gains(); return; }
  if (!strcmp(text, "S")) { print_status(); return; }
  if (!strcmp(text, "Y")) {
    output_length = 0U;
    append_text("HWT yaw_mdeg="); append_integer(hwt_yaw_mdeg);
    append_text(" rate_mdps="); append_integer(hwt_yaw_rate_mdps);
    append_text(" frames="); append_unsigned(hwt_frames);
    append_text(" bad="); append_unsigned(hwt_bad_frames);
    append_text(" age_ms=");
    append_unsigned((hwt_frames != 0U) ? (HAL_GetTick() - hwt_last_ms) : 99999U);
    append_text(" period_ms="); append_unsigned(hwt_period_ms);
    append_text("\r\n"); send_built_text();
    return;
  }
  if (!strcmp(text, "H")) { print_hardware(); return; }
  /* TM <0|1|2|3>：运行时切换遥测格式（不必为了看陀螺仪重编一次）。 */
  if (text[0] == 'T' && text[1] == 'M' &&
      (text[2] == '\0' || text[2] == ' ' || text[2] == '\t')) {
    char *arg = skip_space(text + 2);
    if (*arg == '\0' || !parse_integer(&arg, &first) || *arg != '\0' ||
        first < 0 || first > 4) goto invalid;
#if !WHEEL_SMOKE_USE_ODOM
    if (first == 1) { send_text("ERR telemetry mode 1 needs odometry; compile-time off\r\n"); return; }
#endif
    telemetry_mode = (uint8_t)first;
    telemetry_enabled = 1U;          /* 切格式时顺手开流，省得再发 T 1 */
    next_tx_ms = HAL_GetTick();
    output_length = 0U;
    append_text("TM "); append_integer(first);
    append_text(" telemetry on (0 wheel / 1 odom / 2 four-wheel / 3 gyro / 4 pose)\r\n");
    send_built_text();
    return;
  }
  /* GO <x_mm> <y_mm> [heading_deg]：点到点导航，航向默认保持 0。 */
  if (text[0] == 'G' && text[1] == 'O' &&
      (text[2] == '\0' || text[2] == ' ' || text[2] == '\t')) {
    char *arg = skip_space(text + 2);
    int32_t tx = 0, ty = 0, th = 0;
    if (!parse_integer(&arg, &tx) || !parse_integer(&arg, &ty)) goto invalid;
    if (*arg != '\0' && !parse_integer(&arg, &th)) goto invalid;
    if (*arg != '\0') goto invalid;
    if (tx < -3000 || tx > 3000 || ty < -3000 || ty > 3000 || th < -180 || th > 180) goto invalid;
    goto_start(tx, ty, th * 1000);
    return;
  }
  /* LAP <side_mm> 正方形跑一圈：四个 GO 航点串联，需先 R、有陀螺仪帧。
   * 省略参数 = 1000 mm。正方形方向固定逆时针（左转外圈）。 */
  if (text[0] == 'L' && text[1] == 'A' && text[2] == 'P' &&
      (text[3] == '\0' || text[3] == ' ' || text[3] == '\t')) {
    char *arg = skip_space(text + 3);
    int32_t side = 1000;
    if (*arg != '\0') {
      if (!parse_integer(&arg, &side) || *arg != '\0') goto invalid;
    }
    if (side < (int32_t)WHEEL_SMOKE_LAP_MIN_MM || side > (int32_t)WHEEL_SMOKE_LAP_MAX_MM) goto invalid;
    lap_start(side);
    return;
  }
  /* ML <mm> [speed] / MR <mm> [speed]：定距离横移。ML 正值 = 左移，MR 正值 = 右移。 */
  if ((text[0] == 'M' && (text[1] == 'L' || text[1] == 'R')) &&
      (text[2] == '\0' || text[2] == ' ' || text[2] == '\t')) {
    char *arg = skip_space(text + 2);
    int32_t distance = 0;
    int32_t speed = (int32_t)WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S;
    if (!parse_integer(&arg, &distance)) goto invalid;
    if (*arg != '\0' && !parse_integer(&arg, &speed)) goto invalid;
    if (*arg != '\0') goto invalid;
    if (distance < (int32_t)WHEEL_SMOKE_MOVE_MIN_MM ||
        distance > (int32_t)WHEEL_SMOKE_MOVE_MAX_MM) goto invalid;
    if (speed < 10000 || speed > 500000) goto invalid;
    /* 与 TASK 的横移段同款（task_run_step 的 LEFT/RIGHT 分支）：开航向保持，
     * 横移时只走位不转角。hwt_fresh() 不通过就什么都不做，行为与 TASK 一致。 */
    task_hold_heading();
    move_start_axis((text[1] == 'R') ? -distance : distance, speed,
                    WHEEL_SMOKE_MOVE_AXIS_LEFT);
    return;
  }
  /* TASK：按 task_steps[] 里写死的路径顺序跑一遍。 */
  if (!strcmp(text, "TASK")) {
    if (task.active != 0U) { send_text("ERR task already running; STOP first\r\n"); return; }
    task_start();
    return;
  }
  if (!strcmp(text, "N")) { ol_print_counts(); return; }
  /* ALL [permille]：四路一起开环；省略参数用默认占空比，0 = 全停。 */
  if (!strncmp(text, "ALL", 3U) && (text[3] == '\0' || text[3] == ' ' || text[3] == '\t')) {
    char *arg = skip_space(text + 3);
    first = (int32_t)WHEEL_SMOKE_OL_PERMILLE_DEF;
    if (*arg != '\0') {
      if (!parse_integer(&arg, &first) || *arg != '\0') goto invalid;
    }
    if (!within(first, WHEEL_SMOKE_DIRECT_PERMILLE)) goto invalid;
    Wheel_Enable(0U);
    if (first == 0) { ol_off(); send_text("OL all stopped\r\n"); return; }
    ol_output_all((int16_t)first);
    output_length = 0U;
    append_text("OL all "); append_integer(first);
    append_text(" permille; closed loop OFF, E 1 to restore\r\n");
    send_built_text();
    return;
  }
  /* CMP [permille] [ms]：四路依次开环，结束时回一行四路稳态转速对比。 */
  if (!strncmp(text, "CMP", 3U) && (text[3] == '\0' || text[3] == ' ' || text[3] == '\t')) {
    int32_t permille = WHEEL_SMOKE_OL_PERMILLE_DEF;
    int32_t duration_ms = (int32_t)WHEEL_SMOKE_OL_CMP_MS_DEF;
    char *arg = skip_space(text + 3);
    if (*arg != '\0') {
      if (!parse_integer(&arg, &permille)) goto invalid;
      if (*arg != '\0' && !parse_integer(&arg, &duration_ms)) goto invalid;
      if (*arg != '\0') goto invalid;
    }
    if (permille < 0 || !within(permille, WHEEL_SMOKE_DIRECT_PERMILLE) ||
        duration_ms < (int32_t)WHEEL_SMOKE_OL_CMP_MS_MIN ||
        duration_ms > (int32_t)WHEEL_SMOKE_OL_CMP_MS_MAX) goto invalid;
    ol_cmp_start((int16_t)permille, (uint32_t)duration_ms);
    return;
  }
#if WHEEL_SMOKE_USE_ODOM
  if (!strcmp(text, "C")) { recalibrate(); return; }
#else
  if (!strcmp(text, "C")) { send_text("C not available: odometry disabled\r\n"); return; }
#endif
  if (!strcmp(text, "R")) {
    /* 里程计复位：当前位置记成原点，当前航向记成 0。
     * 没有陀螺仪帧时 odom_reset() 会把航向零点推迟到第一帧，这里要如实告诉用户，
     * 否则他会以为"已经回零"，实际上位姿要等帧到了才开始积分。 */
    odom_reset();
    if (odom_await_yaw != 0U)
      send_text("pose zeroed; WARN no gyro frame yet, heading offset latches on first frame\r\n");
    else
      send_text("pose zeroed (x=0 y=0 heading=0)\r\n");
    return;
  }
  if (!strcmp(text, "Z")) {
    Encoder_ResetAll();
#if WHEEL_SMOKE_USE_ODOM
    ChassisOdom_Reset();
#endif
    send_text("Zeroed\r\n"); return;
  }
  if (text[1] != ' ' && text[1] != '\t') goto invalid;
  cursor = skip_space(text + 1);
  /* A <±deg>：闭环原地转到相对角度。+ = 左转（车体逆时针），- = 右转。
   * 用 HWT101 航向环（P+I，K/J 调增益），到位保持后自动停。 */
  if (text[0] == 'A') {
    if (!parse_integer(&cursor, &first) || *cursor != '\0') goto invalid;
    if (first > (int32_t)WHEEL_SMOKE_TURN_MAX_DEG ||
        first < -(int32_t)WHEEL_SMOKE_TURN_MAX_DEG ||
        (first > -(int32_t)WHEEL_SMOKE_TURN_MIN_DEG &&
         first < (int32_t)WHEEL_SMOKE_TURN_MIN_DEG)) goto invalid;
    turn_start(first);
    return;
  }
  /* M <distance_mm> [speed_um_per_s]：走一段距离后自动停；负距离 = 后退。 */
  if (text[0] == 'M') {
    second = (int32_t)WHEEL_SMOKE_MOVE_DEFAULT_SPEED_UM_PER_S;
    if (!parse_integer(&cursor, &first)) goto invalid;
    if (*cursor != '\0' && !parse_integer(&cursor, &second)) goto invalid;
    if (*cursor != '\0') goto invalid;
    if (first > (int32_t)WHEEL_SMOKE_MOVE_MAX_MM ||
        first < -(int32_t)WHEEL_SMOKE_MOVE_MAX_MM ||
        (first > -(int32_t)WHEEL_SMOKE_MOVE_MIN_MM &&
         first < (int32_t)WHEEL_SMOKE_MOVE_MIN_MM)) goto invalid;
    if (second < 10000 || second > 500000) goto invalid;
    move_start(first, second);
    return;
  }
  /* O 0|1：航向保持开关。O 1 以当前 HWT101 yaw 为目标，配合最近一次 B 的 vx/vy。 */
  if (text[0] == 'O') {
    if (!parse_integer(&cursor, &first) || *cursor || (first != 0 && first != 1)) goto invalid;
    if (first != 0) {
      if (hwt_fresh() == 0U) {
        send_text("ERR no fresh HWT101 frames\r\n");
        return;
      }
      heading_hold = 1U;
      heading_target_mdeg = hwt_yaw_mdeg;
      heading_integral_mdps = 0;
      heading_wz_out = 0;
      heading_next_ms = HAL_GetTick();
      output_length = 0U;
      append_text("YAW hold ON target_mdeg="); append_integer(heading_target_mdeg);
      append_text("\r\n"); send_built_text();
    } else {
      heading_hold = 0U;
      heading_integral_mdps = 0;
      heading_wz_out = 0;
      Wheel_SetBodyVelocity(heading_base_vx_um_per_s, heading_base_vy_um_per_s, 0);
      send_text("YAW hold OFF\r\n");
    }
    return;
  }
  /* K <kp> / J <ki>：航向环在线调参，静默，给 VOFA+ 滑块用。 */
  if (text[0] == 'K' || text[0] == 'J') {
    if (!parse_integer(&cursor, &first) || *cursor || first < 0 || first > 200) goto invalid;
    if (text[0] == 'K') heading_kp = (int16_t)first;
    else { heading_ki = (int16_t)first; heading_integral_mdps = 0; }
    return;
  }
  /* P/I/F：单参数改增益，给 VOFA+ 滑块用，不回声，避免打断遥测。
   * P <kp_q10> | I <ki_q10> | F <ff_per_1000cps> */
  if (text[0] == 'P' || text[0] == 'I' || text[0] == 'F') {
    int16_t kp_q10, ki_q10, ff_per_1000cps;
    if (!parse_integer(&cursor, &first) || *cursor ||
        first < 0 || first > INT16_MAX) goto invalid;
    Wheel_GetGains(&kp_q10, &ki_q10, &ff_per_1000cps);
    if (text[0] == 'P') kp_q10 = (int16_t)first;
    else if (text[0] == 'I') ki_q10 = (int16_t)first;
    else ff_per_1000cps = (int16_t)first;
    Wheel_SetGains(kp_q10, ki_q10, ff_per_1000cps);
    return;   /* 静默：滑块连续拖动时不刷文本 */
  }
  /* W <wheel> <counts/s>：静默版 V，给 VOFA+ 目标滑块用，不回 OK。 */
  if (text[0] == 'W') {
    char wheel = *cursor++;
    if (wheel < 'A' || wheel > 'D' || (*cursor != ' ' && *cursor != '\t')) goto invalid;
    if (!parse_integer(&cursor, &first) || *cursor ||
        !within(first, WHEEL_SMOKE_TARGET_COUNTS_PER_S)) goto invalid;
    selected_wheel = (MotorId)(wheel - 'A');
    Wheel_SetTargetCountsPerSec(selected_wheel, first);
    return;
  }
  if (text[0] == 'X') {
    /* X <a|b|c|d> <counts/s> [ms]：一条命令做完一次阶跃试验，只回一行摘要。 */
    char wheel = *cursor++;
    if (wheel < 'A' || wheel > 'D' || (*cursor != ' ' && *cursor != '\t')) goto invalid;
    if (!parse_integer(&cursor, &first) || first <= 0 ||
        !within(first, WHEEL_SMOKE_TARGET_COUNTS_PER_S)) goto invalid;
    third = (int32_t)WHEEL_SMOKE_STEP_MS_DEFAULT;
    if (*cursor != '\0' && !parse_integer(&cursor, &third)) goto invalid;
    if (*cursor != '\0' || third < (int32_t)WHEEL_SMOKE_STEP_MS_MIN ||
        third > (int32_t)WHEEL_SMOKE_STEP_MS_MAX) goto invalid;
    step_test_start((MotorId)(wheel - 'A'), first, (uint32_t)third);
    return;
  }
  if (text[0] == 'D') {
    /* D <a|b|c|d> <permille>：开环直接给占空比，不看编码器，用来分清
     * "电机/驱动/动力电" 还是 "编码器反馈" 的问题。
     * 会先关掉闭环 —— 否则下一拍(10 ms)中断就把这个占空比覆盖了。 */
    char wheel = *cursor++;
    uint8_t driver;
    if (wheel < 'A' || wheel > 'D' || (*cursor != ' ' && *cursor != '\t')) goto invalid;
    if (!parse_integer(&cursor, &first) || *cursor != '\0' ||
        !within(first, WHEEL_SMOKE_DIRECT_PERMILLE)) goto invalid;
    driver = ((wheel - 'A') >= 2) ? 1U : 0U;   /* A/B 在 U2，C/D 在 U3 */
    Wheel_Enable(0U);
    step_test.active = 0U;
    Motor_Standby(driver, 1U);   /* AT8236 没有 STBY；保留调用，空操作 */
    if (ol_output_one((uint8_t)(wheel - 'A'), (int16_t)first) != HAL_OK) goto invalid;
    output_length = 0U;
    append_text("D "); append_char((char)('a' + (wheel - 'A')));
    append_char(' '); append_integer(first);
    append_text(" open-loop permille; closed loop OFF, E 1 to restore\r\n");
    send_built_text();
    return;
  }
  if (text[0] == 'V') {
    char wheel = *cursor++;
    if (wheel < 'A' || wheel > 'D' || (*cursor != ' ' && *cursor != '\t')) goto invalid;
    if (!parse_integer(&cursor, &first) || *cursor ||
        !within(first, WHEEL_SMOKE_TARGET_COUNTS_PER_S)) goto invalid;
    selected_wheel = (MotorId)(wheel - 'A');
    Wheel_SetTargetCountsPerSec(selected_wheel, first);
  } else if (text[0] == 'B' || text[0] == 'G') {
    if (!parse_integer(&cursor, &first) || !parse_integer(&cursor, &second) ||
        !parse_integer(&cursor, &third) || *cursor) goto invalid;
    if (text[0] == 'B') {
      if (!within(first, WHEEL_SMOKE_BODY_UM_PER_S) ||
          !within(second, WHEEL_SMOKE_BODY_UM_PER_S) || !within(third, WHEEL_SMOKE_SPIN_MDPS)) goto invalid;
      /* 换向（前进 <-> 倒车）时清掉航向积分：前进方向攒下的积分在倒车方向是错的，
       * 不清的话车会先被它顶偏一段再慢慢回正，看起来就是"倒车起步甩一下"。 */
      if ((first < 0) != (heading_base_vx_um_per_s < 0)) {
        heading_integral_mdps = 0;
        heading_wz_out = 0;
      }
      heading_base_vx_um_per_s = first;
      heading_base_vy_um_per_s = second;
      heading_drive_vx_um_per_s = first;
      heading_wz_limit_mdps = WHEEL_SMOKE_HEADING_WZ_LIMIT_MDPS;
      Wheel_SetBodyVelocity(first, second, third);
    } else {
      if (first < 0 || second < 0 || third < 0 ||
          first > INT16_MAX || second > INT16_MAX || third > INT16_MAX) goto invalid;
      Wheel_SetGains((int16_t)first, (int16_t)second, (int16_t)third);
      print_gains(); return;
    }
  } else if (text[0] == 'E' || text[0] == 'T') {
    if (!parse_integer(&cursor, &first) || *cursor || (first != 0 && first != 1)) goto invalid;
    if (text[0] == 'E') {
      Wheel_Enable((uint8_t)first);
      ol_off();
      /* 只有失能才取消航向保持；E 1 是保持之前的常规步骤，不能把它误杀。 */
      if (first == 0) { heading_hold = 0U; heading_integral_mdps = 0; heading_wz_out = 0; }
    }
    else { telemetry_enabled = (uint8_t)first; next_tx_ms = HAL_GetTick(); }
  } else goto invalid;
  send_text("OK\r\n"); return;
invalid:
  send_text("ERR invalid command or range; type ?\r\n");
}
static void poll_commands(SerialPort port, CommandInput *input)
{
  for (uint32_t i = 0U; i < WHEEL_SMOKE_RX_BUDGET; ++i) {
    int received = Serial_ReadByte(port);
    if (received < 0) break;
    if (received == '\r' || received == '\n') {
      if (input->length || input->overflow) {
        input->line[input->length] = '\0'; run_command(input);
      }
      input->length = 0U; input->overflow = 0U;
    } else if (received < 32 && received != '\t') input->overflow = 1U;
    else if (received > 126) input->overflow = 1U;
    else if (!input->overflow && input->length < WHEEL_SMOKE_CMD_LEN - 1U)
      input->line[input->length++] = (char)received;
    else input->overflow = 1U;
  }
}
void WheelSmoke_Begin(void)
{
  Wheel_Stop(); Encoder_ResetAll();
  memset(inputs, 0, sizeof(inputs)); selected_wheel = MOTOR_A;
  reported_stall_events = Wheel_StallEvents();
  telemetry_enabled = 0U;
#if WHEEL_SMOKE_SINGLE_MOTOR
  send_text("SINGLE-MOTOR ready; outputs OFF, stream OFF.\r\n");
  print_help();
  autospin_phase = 2U;
#else
  send_text("FOUR-WHEEL ready; outputs OFF, stream OFF. E 1 then B 300000 0 0.\r\n");
  print_help();
#if WHEEL_SMOKE_USE_ODOM
  send_text("Startup calibration: MUST remain STILL for about 2 s.\r\n");
  ChassisOdom_Init();
  if (!MPU6050_GetBias()->calibrated) send_text("ERR gyro calibration; heading unavailable\r\n");
#endif
#if WHEEL_SMOKE_AUTOSPIN
  /* 延迟从 odom 初始化结束开始算，保证"3 秒"是给人反应的时间。 */
  autospin_start_ms = HAL_GetTick();
  autospin_phase = 0U;
  output_length = 0U;
  append_text("AUTOSPIN armed: 4 wheels FORWARD, open loop, in ");
  append_unsigned(WHEEL_SMOKE_AUTOSPIN_DELAY_MS / 1000U);
  append_text(" s. Any command cancels.\r\n");
  send_built_text();
#else
  autospin_phase = 2U;
#endif
#endif
  telemetry_enabled = 0U; next_tx_ms = HAL_GetTick();
}
void WheelSmoke_Poll(void)
{
  uint32_t now_ms;
  uint32_t stall_events = Wheel_StallEvents();
  /* ISR 已完成急停，主循环才输出提示；按累计事件捕获，避免 STOP 清状态后漏报。 */
  if (stall_events != reported_stall_events) {
    reported_stall_events = stall_events;
    move.active = 0U;
    telemetry_enabled = 0U;
    turn_cancel();
    lap_cancel();
    task.active = 0U;   /* 堵转后不许 TASK 自动往下一段走 */
    send_text("STALL: wheels stopped; explicit E 1 required\r\n");
  }
  poll_commands(WHEEL_SMOKE_TELEMETRY_PORT, &inputs[0]);
  if (WHEEL_SMOKE_TEXT_PORT != WHEEL_SMOKE_TELEMETRY_PORT)
    poll_commands(WHEEL_SMOKE_TEXT_PORT, &inputs[1]);
  hwt_poll();
#if WHEEL_SMOKE_USE_ODOM
  ChassisOdom_Update();
#endif
  now_ms = HAL_GetTick();
  odom_poll(now_ms);
  goto_poll(now_ms);
  lap_poll();
  step_test_poll(now_ms);
  move_poll(now_ms);
  heading_poll(now_ms);
  turn_poll(now_ms);
  task_poll(now_ms);
  autospin_poll(now_ms);
  ol_poll(now_ms);
  if ((int32_t)(now_ms - next_tx_ms) < 0) return;
  /* 不补发积压帧，长命令后从当前时刻重新起拍。 */
  next_tx_ms = now_ms + 1000U / WHEEL_SMOKE_TX_HZ;
  if (telemetry_enabled) send_telemetry();
}
