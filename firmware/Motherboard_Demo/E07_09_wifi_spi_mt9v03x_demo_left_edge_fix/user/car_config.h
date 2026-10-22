/*********************************************************************************************************************
* 整车闭环控制 —— 硬件配置与可调参数
*
* 这个文件只放"配置"，不放逻辑：改参数、改引脚都在这里，不用动其它文件
*
* 硬件构成（本次并进来的 E02 内容）：
*   后轮：2 个直流电机 + DRV8701E 双路驱动板（每路 1 根 DIR + 1 根 PWM）  <- E02_02
*   前轮：1 个转向舵机（PWM 50Hz，占空比换算与 E02_03 完全一致）           <- E02_03
*   测速：2 个增量编码器（左右各一），用于速度闭环
*
* 引脚占用现状（E07_09 视觉工程本身已占用的，这里一个都不能碰）：
*   摄像头数据 D0~D7 = P20 ~ P27   摄像头 IIC = PB6 / PB7
*   WiFi SPI = P81 / P82 / P83      WiFi CS = P64   WiFi INT = P15   WiFi RST = P47
* 本文件用到的：
*   电机 = P50/P51(左) P52/P53(右)   舵机 = PA0   编码器 = P60/P62 + P40/P42
*   注意 P52 同时是主板上的用户 LED，做右电机 DIR 时 LED 会跟着方向亮灭，属正常现象
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入整车闭环控制（电机+舵机+编码器）
********************************************************************************************************************/

#ifndef __CAR_CONFIG_H
#define __CAR_CONFIG_H

#include "zf_common_headfile.h"
#include "vision_geometry.h"

/*====================================================================================================================
 * 一、上车总开关 —— 第一次上车请严格按"先舵机、再电机、最后速度环"的顺序放开
 *===================================================================================================================*/
#define CAR_ENABLE_SERVO        (1)         // 1: 允许舵机输出   0: 舵机不动
#define CAR_ENABLE_MOTOR        (1)         // 1: 允许电机输出   0: 电机完全不转（调舵机时先设 0！）
/* Disabled by default: fw72-noenc restored motor output in the bench test.
 * Keep monitoring off until the initialization/readout regression is isolated. */
#define CAR_ENCODER_MONITOR_ENABLE (0)
#define CAR_ENCODER_FEEDBACK_ENABLE (CAR_ENABLE_ENCODER || CAR_ENCODER_MONITOR_ENABLE)
#define CAR_ENABLE_ENCODER      (0)         // 1: 速度闭环(PI)   0: 速度开环(目标速度直接换算成占空比)；第一圈先开环，少一个变量

/* 上电时先把电机输出"硬关"（PWM 直接 0，PI 不参与），要发无线串口命令 o 才出力。
 *   为什么：这样一次烧录就能做两件事 —— 上电后车绝对不动，先用手推着车沿赛道走，
 *   看前轮是不是"往中线方向打"（打角方向反了车会直接冲出赛道，必须先确认）；
 *   确认没问题再发 o 让电机出力，车才开始跑。
 *   注意：台架自检(CAR_BRINGUP_TEST=1)要自己转轮子，那种配置下这个开关自动失效 */
#define CAR_BOOT_OUTPUT_OFF     (1)         // 1 = 上电输出硬关（推荐）；0 = 老行为（上电就允许出力）

/* 打角整体取反：万一"车偏左、前轮却往左打"（视觉左右边界反了），把这个置 1。
 *   也可以不重新烧录：无线串口发 n 在线切换（发 ? 能看到 steerinv=0/1） */
#define CAR_STEER_INVERT        (0)
                                            //    编码器还没接好/没标定时，先设 0
/* 固件版本标记：无线串口上电横幅和 ? 状态里会打印出来，
 * 用来确认板子里烧的到底是不是最新那版（改了代码我会把它 +1） */
#define CAR_FW_TAG              "fw103-preview42"

/* 把宏的值变成字符串（打印用）：115200 这种数超过 int16，直接用 %d 打印会变成负数 */
#define CAR_STR2(x)             #x
#define CAR_STR(x)              CAR_STR2(x)


/*====================================================================================================================
 * 二、后轮双电机：DRV8701E（DIR + PWM）
 *   引脚与 E02_02 例程的"电机1 / 电机2"完全一致，直接对应主板电机信号接口的第 1、2 路
 *===================================================================================================================*/
#define CAR_MOTOR_PWM_FREQ      (17000)             // 电机 PWM 频率(Hz)，与 E02_02 一致

/* 左右对调开关（电机）：台架自检里"打印 LEFT wheel 时，动的却是车右侧那个轮子" -> 置 1
 *   它管的是"哪一侧"；单个轮子"前进/后退"反了是另一件事，要改下面的 CAR_MOTOR_x_INVERT。
 *   置 1 只是把左右两路的引脚整体换过来，下面两个 INVERT 的含义仍然是"左轮 / 右轮"。 */
#define CAR_SWAP_MOTOR_LR       (1)

#if (0 == CAR_SWAP_MOTOR_LR)
#define CAR_MOTOR_L_DIR         (IO_P50)            // 左电机 DIR（GPIO）
#define CAR_MOTOR_L_PWM         (PWMD_CH2_P51)      // 左电机 PWM
#define CAR_MOTOR_R_DIR         (IO_P52)            // 右电机 DIR（GPIO，同时是板上 LED）
#define CAR_MOTOR_R_PWM         (PWMD_CH4_P53)      // 右电机 PWM
#else
#define CAR_MOTOR_L_DIR         (IO_P52)            // 左电机 DIR（实际接在主板电机口第 2 路）
#define CAR_MOTOR_L_PWM         (PWMD_CH4_P53)      // 左电机 PWM
#define CAR_MOTOR_R_DIR         (IO_P50)            // 右电机 DIR（实际接在主板电机口第 1 路）
#define CAR_MOTOR_R_PWM         (PWMD_CH2_P51)      // 右电机 PWM
#endif

#define CAR_MOTOR_L_INVERT      (0)                 // 左轮方向取反：往前转反了就把这个 0/1 互换（"正反转"问题，与上面的 SWAP 无关）
#define CAR_MOTOR_R_INVERT      (0)                 // 右轮方向取反：往前转反了就把这个 0/1 互换

#define CAR_MOTOR_DUTY_MAX      (5000)              // 占空比上限(万分比, 10000 = 100%)
                                                    // 先按例程的 50%（E02_01 / E02_02 里就是 MAX_DUTY 50），
                                                    // 这是保护电机/齿轮箱、同时限制最高车速的硬上限；
                                                    // 跑顺了嫌慢再 5000 -> 6000 -> 7000 一点点加
#define CAR_MOTOR_DEADZONE      (0)                 // 起动死区补偿(万分比)：目标不为 0 但小于该值时，直接抬到该值
                                                    // 静摩擦导致小占空比不转时填，常见 300~800，实测后填

#define CAR_MOTOR_RAMP_STEP     (200)               // 每个速度环周期占空比最大变化量(万分比)
                                                    // 默认 10ms 一个周期 => 5000/200 = 25 个周期 = 250ms 从 0 到满

/*====================================================================================================================
 * 三、前轮转向舵机
 *   占空比换算与 E02_03 一致：duty = PWM_DUTY_MAX / (1000/频率) * (0.5 + 角度/90)
 *   即 50Hz 下：角度 90 度 -> 1.5ms -> 7.5% 占空比
 *
 *   【必改】CAR_SERVO_PWM：主板从上往下的 4 个舵机口（与 E02_03 例程完全一致）：
 *           PWM1 = PWME_CH1P_PA0
 *           PWM2 = PWME_CH2P_PA2
 *           PWM3 = PWME_CH3P_PA4
 *           PWM4 = PWME_CH4P_PA6     <- 当前默认值 = 你说的"从上往下第 4 个"
 *
 *   【注意】千万不要用 P22！本工程里 P22 是摄像头的数据位 D2（MT9V03X_D2_PIN）：
 *      往 P22 输出 PWM 就是和摄像头抢同一根线 —— 舵机会被图像数据抽得乱抖，
 *      图像也会因为 D2 被拉坏而出现条纹/噪点。
 *   不确定插在哪个口：先用 E02_03 舵机例程（它一次驱动 4 个口，舵机插在哪个口就在哪个口来回摆），
 *      确认之后把这里改成对应宏即可。
 *===================================================================================================================*/
#define CAR_SERVO_PWM           (PWME_CH4P_PA6)     // 已确认：舵机在主板 PWM4 口（从上往下第 4 个）= PA6
#define CAR_SERVO_FREQ          (50)                // 舵机 PWM 频率，允许范围 50 ~ 300

#define CAR_SERVO_CENTER_DEG    (90.0)              // 中位角度：车走直线时舵机停在这个角度
#define CAR_SERVO_LEFT_DEG      (55.0)              // 绝对角度硬极限(最外层保险)：只防"配置写错把舵机顶死"，正常跑车碰不到
#define CAR_SERVO_RIGHT_DEG     (125.0)             // 同上。★注意：真正的左右打角极限看下面的 CAR_SERVO_HARD_LIMIT_DEG
                                                    // 上面两个先给 ±20 度这种保守值：例程只敢跑 80~100 度，
                                                    // 就是怕装车位置不对、舵机顶死堵转烧掉。
                                                    // 确认前轮到 ±20 度时既不顶死也不嗡嗡响之后，
                                                    // 再逐步改成实际"打死"位置（常见 62 / 118）
#define CAR_SERVO_TRIM_DEG      (5.0)               // 机械中位微调（单位=舵机角度）：前轮归中时偏右就填正数、偏左就填负数，边试边改
/* ★ 打角硬极限（相对"前轮正中"对称，左右各这么多度）：  *   为什么不能用 LEFT/RIGHT_DEG 那两个数当极限 —— 它们是相对 90° 写的，  *   而"前轮正中"还叠着 TRIM + 直行微调：本车零点在 97.5°，原来 [70,110] 实际只给了  *   "往左 12.5° / 往右 27.5°"，左转打角被偷偷砍掉一大截（弯道过不去就是这么来的），  *   而且按 . 加大最大打角对左转完全无效（110° 先卡住了）。  *   现在改成绕零点对称：左右各 CAR_SERVO_HARD_LIMIT_DEG 度，跟 TRIM/直行微调无关。  *   ★先别发 o，按 . 把最大打角加到 18，看前轮打到头有没有顶住/异响；有就按 , 收回来。 */ #define CAR_SERVO_HARD_LIMIT_DEG (18.0)              // 相对前轮正中，左右各能打多少度
#define CAR_SERVO_INVERT        (1)                 // 打角方向反了（该往左打却往右打）-> 改成 1
/* ★ 舵机响应速度上限：原来写的是"每个图像帧最多转 3 度"，而 servo_set_steer() 是每处理完一帧图像才调用一次 ——
 *   帧率一低（发图阻塞时只有 10~20fps），"每帧 3 度"就只剩 30~60 度/秒，入弯时舵机明显反应不过来。
 *   现在改成按【时间】算：每 10ms 最多转 CAR_SERVO_SLEW_DEG_PER_10MS 度，跟帧率无关。
 *   2.0 度/10ms = 200 度/秒（一般舵机 400~600 度/秒，留了余量保护齿轮）。
 *   ★ 可以不用重新烧录：无线串口发 f = 慢一点、g = 快一点，当前值看 ? 里的 slew= */
#define CAR_SERVO_SLEW_DEG_PER_10MS (4.0)            // 每 10ms 最多转多少度（fw43：2.0 → 4.0 = 400 度/秒。舵机本体约 400~500 度/秒，再大没意义）
#define CAR_SERVO_SLEW_STEP_MAX (36.0)              // Full +/-18deg span; timed slew and final angle limits still apply.
#define CAR_SERVO_SLEW_STEP_DEG (0.25)              // ★在线调 f / g 每次 -/+ 这么多（度/10ms）
#define CAR_SERVO_OFF_MAX_DEG   (8.0)               // 直行微调上限(度)：防止误按把中位调飞
#define CAR_SERVO_OFF_COARSE_DEG (2.0)              // 直行微调粗调步长(度)：无线串口 { = 往左、} = 往右（偏得比较多时先用它）
#define CAR_SERVO_OFF_STEP_DEG  (0.5)               // 直行微调细调步长(度)：无线串口 ; = 往左、' = 往右，每次动这么多
#define CAR_SERVO_OFF_DEG       (-2.5)              // 直行微调初值(度)：> 0 = 前轮往右，< 0 = 前轮往左。★2026-09-22 现场调出来的值：直道往右偏，往左修 2.5°

/*====================================================================================================================
 * 四、编码器（速度闭环用）
 *
 *   【必改】下面 4 个引脚：我给的是 SDK 例程里推荐、且没有被摄像头/WiFi/电机/舵机占用的一组引脚。
 *          你的车实际把编码器接在哪几根线上，请按实际接线改这里！接错了速度环会反着调（车会冲出去）。
 *
 *   CAR_ENCODER_MODE: 0 = 正交编码器 A/B 两相（此时 CAR_ENC_x_DIR 就是 B 相）★本车已确认是这种
 *                     1 = 带方向增量编码器（1 根 PULSE + 1 根 DIR）
 *   注意：STC32G144K 只有 PWM 接口支持正交解码，定时器接口不支持，所以正交模式必须用 PWMA~PWMF
 *===================================================================================================================*/
#define CAR_ENCODER_MODE        (0)                     // 0 = 正交(A/B)   1 = 带方向(PULSE + DIR)
                                                        // ★ 已确认本车两个编码器都是 A/B 正交型（2026-09-20）：
                                                        //   带方向模式下 DIR(其实是 B 相) 是 90 度方波，符号会
                                                        //   随它的瞬时电平乱翻 -> 2 秒 net≈0 / abs 很大 / flips 一堆。
                                                        //   正交模式下硬件自己做 4 倍频计数，方向和计数都是硬件给的，干净。

/* 左右对调开关（编码器）：自检时"只转左轮，动的却是 encR" -> 置 1
 *   电机和编码器一般是同一侧接反，所以默认和 CAR_SWAP_MOTOR_LR 保持一致；
 *   如果两边实际情况不同（例如电机换了、编码器没换），就单独改这个。
 *   和电机一样：它管"哪一侧"，计数正负号是下面的 CAR_ENC_x_INVERT 管。 */
#define CAR_SWAP_ENCODER_LR     (1)

#if (0 == CAR_SWAP_ENCODER_LR)
#define CAR_ENC_L_INDEX         (PWMA_ENCODER)          // 左编码器使用的硬件接口
#define CAR_ENC_L_PULSE         (PWMA_ENCODER_CH1P_P60) // 【必改】左编码器 PULSE（正交模式 = A 相）
#define CAR_ENC_L_DIR           (PWMA_ENCODER_CH2P_P62) // 【必改】左编码器 DIR  （正交模式 = B 相）
#define CAR_ENC_R_INDEX         (PWMC_ENCODER)          // 右编码器使用的硬件接口
#define CAR_ENC_R_PULSE         (PWMC_ENCODER_CH1P_P40) // 【必改】右编码器 PULSE（正交模式 = A 相）
#define CAR_ENC_R_DIR           (PWMC_ENCODER_CH2P_P42) // 【必改】右编码器 DIR  （正交模式 = B 相）
#else
#define CAR_ENC_L_INDEX         (PWMC_ENCODER)          // 左编码器使用的硬件接口（跟着电机一起换过来了）
#define CAR_ENC_L_PULSE         (PWMC_ENCODER_CH1P_P40) // 左编码器 PULSE
#define CAR_ENC_L_DIR           (PWMC_ENCODER_CH2P_P42) // 左编码器 DIR
#define CAR_ENC_R_INDEX         (PWMA_ENCODER)          // 右编码器使用的硬件接口
#define CAR_ENC_R_PULSE         (PWMA_ENCODER_CH1P_P60) // 右编码器 PULSE
#define CAR_ENC_R_DIR           (PWMA_ENCODER_CH2P_P62) // 右编码器 DIR
#endif

/* ★ 2026-09-20 推车实测（fw14）：把车往前推，rawL 和 rawR 都是正数（net L=+1574 R=+1501，
 *   abs≈net、flips=1，读数干净），所以两边 INVERT 都保持 0。
 *   之前 R 写 1 是"左右镜像"的猜测，实测证明那个假设不成立 —— 两个编码器同型号、装法一致，不用镜像。 */
#define CAR_ENC_L_INVERT        (0)                     // 0 = 不取反（推车实测：前进读正数）
#define CAR_ENC_R_INVERT        (0)                     // 0 = 不取反（推车实测：前进读正数）

#define CAR_SPEED_PIT           (TIM1_PIT)              // 速度环使用的周期中断（与 E03_01 例程用的同一个）
#define CAR_SPEED_PERIOD_MS     (10)                    // 速度环周期(ms)：10ms => 100Hz
                                                        // 例程里用 100ms 只是"看计数"，速度环要快得多，
                                                        // 10ms 是常用值，最小不要低于 5ms

#define CAR_ENC_COUNTS_PER_METER (15740)               // 粗标定：推车 5 秒 net≈3445，没尺子按两次推车估的；只影响 cm/s 这个单位准不准
                                                        // 用 CAR_DEBUG_PRINT 打印的 encL/encR 累加自己标定

/* 速度环 PI + 前馈：输出是占空比(万分比)，误差单位是"计数/周期"
 * duty = 前馈*目标 + Kp*误差 + Ki*误差累加
 * 调参顺序：先只留前馈(把 Kp、Ki 设 0)让车能跑起来，再加 Kp 消稳态误差，最后加一点点 Ki  */
#define CAR_SPEED_FF_NUM        (10)                    // 前馈：每 1 个"计数/周期"对应多少占空比
#define CAR_SPEED_KP_NUM        (6)                     // Kp：每 1 个"计数/周期"的误差对应多少占空比
#define CAR_SPEED_KI_NUM        (1)                     // Ki：误差累加项系数
#define CAR_SPEED_I_MAX         (2500)                  // 积分限幅(抗饱和)

/* 编码器 / 堵转保护（强烈建议保留）
 *
 * 为什么必须有：编码器没接好、引脚填错、模式选错时，速度环读到的速度永远是 0，
 * 它会以为"车一直不动"，于是一路把占空比加到上限 —— 车会直接窜出去，你还不知道为什么。
 * 有了这道闸：给着 > 15% 的占空比却连续 500ms 一个计数都读不到 => 判定编码器或电机有问题，
 * 立刻停车锁死并在串口打印 FAULT（复位或重新 motor_run(1) 才能再跑）。
 * 台架自检时会自动临时关掉这条保护（自检本来就是要试接线对不对）。
 * 注意：它同时也是一个"轮子卡住/堵转"保护。 */
#define CAR_ENC_FAIL_ENABLE     (1)                     // 1 = 打开该保护
#define CAR_ENC_FAIL_DUTY       (1500)                  // 占空比超过该值(万分比)才开始判定
#define CAR_ENC_FAIL_TICKS      (50)                    // 50 个周期 × 10ms = 500ms

/* 开环模式(CAR_ENABLE_ENCODER = 0)下，目标速度(cm/s) -> 占空比的换算系数 */
#define CAR_OPENLOOP_DUTY_PER_CM_S (40)                 // 每 1cm/s 对应多少占空比(万分比)：
                                                        // 120cm/s * 40 = 4800 => 48%

/*====================================================================================================================
 * 五、转向控制：视觉已经给好 x1(左线) / x2(中线) / x3(右线)，这里只做"让车贴着中线走"
 *   行号约定：0 = 图像最上方(最远)，119 = 最下方(最近)
 *   用近 / 中 / 远三行加权出一个"瞄点"：远处权重大 = 提前打角（预瞄），这是过弯稳的关键
 *===================================================================================================================*/
#define CAR_ROW_NEAR            (VISION_PX(84))   // 近处预览行
#define CAR_ROW_MID             (VISION_PX(60))    // 中间预览行
#define CAR_ROW_FAR             (VISION_PX(34))    // 远处预览行
#define CAR_W_NEAR              (20)    // 三行权重：近处小、远处大
#define CAR_W_MID               (30)
#define CAR_W_FAR               (60)

#define CAR_STEER_KP            (0.30)  // 【fw52】比例系数：0.40 -> 0.30（用户要求直接降 kp 治直道画龙）。★在线 ( / ) 每次 ±0.05
#define CAR_STEER_KD            (0.55)  // 【fw47e】微分系数：0.50 -> 0.55（加阻尼压画龙）。★在线 < / > 每次 ±0.05
/* 微分项一阶低通系数（0~1）：越小越平滑、越大越灵敏。
 *   视觉的 aim 每帧都有几个像素的噪声，微分项会把噪声放大成"舵机来回抖"（直道画龙）。
 *   0.35 = 新样本占 35%；还画龙就往下调（0.2），觉得反应迟钝就往上调（0.5）。0 = 关掉滤波 */
#define CAR_STEER_D_FILTER      (0.30)  // 【fw47e】微分项一阶低通：0.35 -> 0.30（滤得更狠一点，微分别把视觉噪声放大）
#define CAR_STEER_KI_ENABLE     (0)     // 打角积分总开关：0 = 完全不用积分（推荐先这样）
#define CAR_STEER_KI            (0.00)  // 积分系数：弯道里会积累、容易画龙，确认需要再加
#define CAR_STEER_MAX_DEG       (18.0)  // 打角限幅（相对中位）。★★ 可以不用重新烧录：无线串口发 , / . 每次 -1 / +1 度
                                        // 先和上面舵机 ±20 度对齐；舵机极限放宽后，这个也跟着放大
#define CAR_STEER_ERR_MAX       (70)    // 瞄点误差限幅(像素)：防止某帧边线异常把舵机打飞
#define CAR_STEER_KP_SOFT       (0.20)  // 【fw60】双增益里的"软"增益：|aim| 小于过渡起点时用（直道安静）。★在线 9 / x 每次 -/+0.02
#define CAR_STEER_KP_BLEND_PX   (8)     // 【fw60】双增益过渡起点(像素)：误差小于它 = 全程用软增益
#define CAR_STEER_KP_BLEND_SPAN (12)    // 【fw60】双增益过渡长度(像素)：到 8+12 = 20px 就完全用硬增益（线性过渡，不是折点）
#define CAR_STEER_DEAD_DEG      (0.4)   // 【fw60】舵机输出死区(度)：算出来的动态打角比它还小就归零，压掉视觉微抖
#define CAR_RING_GATE_LOST_MAX  (VISION_PX(100))   // 【fw60】闸门③：入口侧丢线行数超过它 = 整条边线都没了（丢线/出画），不是环口
#define CAR_RING_GATE_WIDE_PX   (VISION_PX(110))   // 【fw60】闸门④：30~60 行里赛道最大宽度 >= 它才算"环口变宽"（画面宽 188）

/*====================================================================================================================
 * 六、速度规划：直道快、弯道慢、十字稳、丢线爬
 *===================================================================================================================*/
 // 【fw43】本行以下三个"速度/缓变"值 2026-09-23 按用户要求调整过：MIN 30→38、CURVE_K 0.8→0.6、SLEW 2.0→4.0
#define CAR_SPEED_FIXED_ENABLE  (1)     // 【fw56】1 = 固定速度：弯道 / 直道 同速（用户要求）；0 = 老的"弯道减速 + 提前减速"
#define CAR_SPEED_FIXED_CM_S    (38)    // 【fw56】固定速度(cm/s)：38 x 40 = 15.2% 占空比。★在线 u / i 每次 -5 / +5
#define CAR_SPEED_FIXED_MIN_CM_S (20)   // 【fw56】固定速度的下限（在线 u 最多降到这里）
#define CAR_START_BOOST_CM_S    (14)    // 【fw56】起步助推：从静止起步的头若干帧额外加这么多 cm/s（台架：<18% 占空比静止带不动）
#define CAR_START_BOOST_FRAMES  (40)    // 【fw56】助推持续多少帧（40 帧 ≈ 0.8 秒 @50fps），之后回到固定速度
#define CAR_SPEED_BASE_CM_S     (58)    // 直道目标速度(cm/s)：开环 58 x 40 = 23.2% 占空比（fw27：18% 从静止起步拉不动车，抬回 23%）
#define CAR_SPEED_MIN_CM_S      (34)     // 【fw47e】最低速度(弯道速度)：38 -> 34（过弯再慢一档）；直道速度由下面 MAX 决定
#define CAR_SPEED_MAX_CM_S      (58)    // 最高速度(cm/s)：70 x 40 = 28%
#define CAR_SPEED_STEP_CM_S     (5)     // ★在线调速步长(cm/s)：无线串口 u = 减速、i = 加速，每次这么多

#define CAR_SPEED_CURVE_K       (1.00)  // 【fw47e】弯道减速系数：0.6 -> 1.0（过弯要更慢）。★在线 5 / 6 每次 -/+0.1
#define CAR_CURVE_EARLY_K       (0.25)  // 【fw49】"看到弯道就提前减速"的系数：speed -= 它 x (|远端偏移| - 死区)
                                        //   ★在线 7 / 8 每次 -0.05 / +0.05（范围 0~0.6，0 = 关掉提前减速）
#define CAR_CURVE_EARLY_DEAD    (6)     // 【fw49】提前减速的死区(像素)：远端偏移小于它不动（挡直道上的几像素抖动）

#define CAR_SPEED_CURVE_MAX     (80)    // 弯道程度限幅(像素)
#define CAR_SPEED_CROSS_CM_S    (30)    // 十字区域的速度上限(cm/s)：50 = 20%，和下限一致（再低可能从静止带不动）
#define CAR_SPEED_LOST_CM_S     (50)    // 丢线时的爬行速度(cm/s)：同上，不要低于 50

#define CAR_DIFF_K              (0.4)   // Restored code; retain requested differential gain 0.4; 3/4 adjusts -/+0.2
/* Differential diagnosis: keep the original defaults for one-change-at-a-time tests.
 * '=' switches normalized/legacy mixing; '%' switches the corner cap. */
#define CAR_DIFF_NORMALIZED_DEFAULT (0)
#define CAR_DIFF_REFERENCE_CM_S     (38.0f)
#define CAR_DIFF_RATIO_MAX          (0.45f)
#define CAR_TURN_LIMIT_DEFAULT      (0)
#define CAR_TURN_SPEED_CAP_CM_S     (38)
#define CAR_TURN_STEER_DEG          (8.0f)
#define CAR_TURN_STEER_FULL_DEG     (15.0f)
#define CAR_TURN_CURVE_PX           (12)
#define CAR_TURN_CURVE_FULL_PX      (24)
#define CAR_LOST_STEER_HOLD     (0.70)  // 丢线时保留多少上一帧的打角（0 = 立刻回正，1 = 完全保持）。★注意：只有"左右两条线同时丢"才算丢线，见 car_control.c
/* ===== 【fw47】圆环行驶策略（视觉组只做"识别 + 补线"，怎么走、什么时候进环是我们的活） =====
 *   ▲ fw45 的老策略已经作废：它是"视觉组一报进环就打角"（IN 附加 4 度、INSIDE 固定角绕），
 *     而实车要求是【先直行通过第一个路口，走到第二个路口才打角】—— 见下面的 v2。
 *     这里保留的老参数：ENABLE / STEER_INSIDE / SPEED_INSIDE / OUT_HOLD / DIR_SIGN / 两个在线步长 / SPEED_MIN
 *     （老参数里的"进环预打角 CAR_RING_STEER_IN"不再使用：进环改成整段固定打角接管）
 *
 *   ▼ fw47 v2：圆环入口那一侧有【两处张开】——
 *     开口的"近端拐点"是第一个路口，"远端拐点"才是要进去的第二个路口
 *     （视觉组那边正好就是 ring_open_row1 = 近端 / ring_open_row2 = 远端）。
 *     "看到张开就进" = 在第一个路口就扎进去了；而且视觉组的 ring_fix_side 在状态 1/2 会把
 *     入口侧补成【朝环内倾斜】的斜线，中线也会被带进环里 —— 所以时序必须由这一层自己排，
 *     只从视觉组读两个量：
 *       ring_flag / ring_type         识别结果（哪一侧开口）
 *       track_lost[] 入口侧的丢线行    开口在画面里走到哪儿了（car_control.c 0b 段扫描）
 *   阶段（car_control.c 的 s_rp_phase）：
 *     1 直行：瞄点换成"识别前锁下来的值"，沿直道直着走，一路走过第一个路口
 *     2 进环：开口已经到车头（= 第二个路口）-> 固定打角打进去，持续 CAR_RING_ENTER_FRAMES 帧
 *     3 环内：打角线性交还给跟线；序列继续压着十字/丢线处理，回到直道（弯道程度很小）就结束
 *   在线可调：y = 开关  m/M = 进环固定打角 -/+1 度  l/L = 环内速度 -/+5 cm/s  z = 方向取反
 *             j/J = 进环打角持续帧数 -/+5（转不够 / 转过头）  p/P = 进环时机 提前/滞后（行号 -/+4） */
#define CAR_RING_ENABLE          (0)        // 1 = 启用圆环策略；0 = 完全按中线跟（= 旧行为）
#define CAR_RING_STEER_INSIDE    (6.0)      // 进环/环内固定打角(度)（在线 m / M 调）。【fw55】9 -> 6：用户要求进环打角改 6 度
                                         // 【fw50】实车"在环中打角太大"：12 -> 9（小一点），要再小按 m
#define CAR_RING_SPEED_INSIDE    (38)       // 鐜唴鐩爣閫熷害(cm/s)銆傘�恌w56銆?4 -> 38锛氳窡鍥哄畾閫熷害涓�鑷达紙寮亾鐩撮亾鍚岄�燂級
#define CAR_RING_OUT_HOLD        (8)        // 进环固定打角交还给正常跟线的过渡帧数
#define CAR_RING_DIR_SIGN        (1)        // 方向：+1 = 用 ring_type 推出来的方向；-1 = 取反（在线按 z）
#define CAR_RING_STEER_STEP_DEG  (1.0)      // 在线 m / M 每次 减 / 加 这么多度
#define CAR_RING_SPEED_STEP_CM_S (5)        // 在线 l / L 每次 减 / 加 这么多 cm/s
#define CAR_RING_SPEED_MIN_CM_S  (20)       // 环内速度的下限（台架验证过：低于 20 起步带不动）

#define CAR_RING_OPEN_LOST_MIN   (VISION_PX(18))    // 入口侧丢线行数 >= 本值 = "已经看到开口"（第一个路口进了画面）
#define CAR_RING_OPEN_GONE_LOST  (VISION_PX(6))     // 入口侧丢线行数 <= 本值 = 开口已经从画面里过去（车已开过第二个路口）
#define CAR_RING_ENTER_ROW       (VISION_PX(76))    // 开口远端 >= 本行 = 到第二个路口了（p/P 在线 ±4）
                                         // 【fw57】按实车（用户按 p 调出来的值）同步：88 -> 76
                                         // 【fw50】实车"进环滞后、反应慢"：100 -> 88，提前约 12 行（约 10cm）打角
#define CAR_RING_ENTER_CONFIRM   (2)     // 上面这条连续几帧成立才算数（挡单帧噪声）
#define CAR_RING_ENTER_FRAMES    (60)    // 进环固定打角持续多少帧（j/J 在线 ±5）。【fw55】40 -> 60：打角 9->6 度后按时长补回来（9x40 = 6x60）
                                         // 【fw50】打角 12->9 之后按时长补回来：12x30 ≈ 9x40，总转角基本不变
#define CAR_RING_STRAIGHT_MAX    (150)   // 直行阶段最长帧数：超了就当没识别到，继续直行不乱打角
/* 【fw47b】出环判据全删了（用户要求"进环之后让他继续跑就行"）：进环打角交还完就【放手】，
 *   后面让车自己跟线继续跑 —— 不做出环判定，也不长期压着十字/丢线逻辑。
 *   原来的 CAR_RING_LOOP_MIN_FRAMES / CAR_RING_SEQ_MAX_FRAMES /
 *   CAR_RING_EXIT_CURVE_MAX / CAR_RING_EXIT_CONFIRM 四个宏全部作废。 */
/* 圆环序列的阶段编号（car_control.c 的 s_rp_phase 用） */
#define CAR_RP_IDLE              (0)     // 没进圆环序列
#define CAR_RP_STRAIGHT          (1)     // 直行阶段：走过第一个路口
#define CAR_RP_ENTER             (2)     // 进环阶段：到第二个路口，固定打角打进去
#define CAR_RP_HANDOFF           (3)     // 交还阶段：把进环打角还给跟线，给完就放手（出环不归我们管）

/* ---------- 【fw47b】圆环"可信度闸门"：实车反馈"看到什么都觉得是圆环" ----------
 *   视觉组的识别判据（code\ring_road.c）是"入口侧连续丢线 >= 25 行 且 对面那条边是直道"，
 *   而【弯道里外侧边线跑出画面】恰好也是这个特征 —— 所以误报会到处都是。
 *   闸门不过 = 当没看见：不锁瞄点、不打角（视觉组的补线我们管不了，但至少不跟着它冲）。
 *   在线按 s 循环切档：0 = 关（老行为：报了就信）  1 = 直道接近 + 开口有界  2 = 再加"环内特征"（默认最严） */
#define CAR_RING_GATE_LEVEL        (2)   // 默认档位
#define CAR_RING_GATE_LEVEL_STRICT (2)   // 这一档还要求"视觉组自己看到环内特征"
#define CAR_RING_GATE_CURVE_MAX    (12)  // 【fw47c】20 -> 12：弯道程度的"快衰减峰值"上限（实车误报主要在弯道里）
                                         // 圆环都是从直道进的（直道 0~10）；调小 = 更不容易在弯里误进环
#define CAR_RING_SIDE_MARGIN       (VISION_PX(6))   // 【fw47c】判"环在哪一侧"的余量：一侧的最长丢线段比另一侧多这么多行才算
                                         // （左环实测左 43~53 / 右 4~9，差得很开；两边差不多时还是用 vision 的 ring_type）
#define CAR_RING_LABEL_RUN_MIN     (VISION_PX(25))  // 【fw47d】入口侧标签纠错：我们这一侧连续丢线 >= 这么多行
#define CAR_RING_LABEL_OPPO_RUN_MAX (VISION_PX(12)) //                     且 另一侧 <= 这么多行（真环 43~56 对 4~9；十字两边都丢，不满足）
#define CAR_RING_MOD_CURVE_MAX     (24)  // 【fw47e】弯道里"不叫圆环模块"的阈值（比接受阈值 12 宽）：太严会漏掉紧跟弯道的真环
#define CAR_RING_GATE_MIN_ABOVE    (VISION_PX(8))   // 开口远端还得能看到 >= 这么多行边线 = "第二个路口看得见"
                                         // （外侧线跑出画面时丢线会一直顶到画面最上面，这条就过不了）
#define CAR_RING_GATE_MIN_OPEN     (VISION_PX(20))  // 入口侧丢线行数至少这么多才算"路真的开了"（实测真环 43~56 行）
#define CAR_RING_ABORT_CURVE       (45)  // 直行阶段里弯道程度连续超这个值 -> 判定是误判，立刻放开瞄点
#define CAR_RING_ABORT_N           (6)   // 连续几帧才认
#define CAR_LOST_STOP_FRAMES    (80)    // 连续丢线超过这么多帧就停车（安全兜底，约 1~3 秒）

/* 视觉链路（code/ 里那四个模块，视觉组给的那份）的输出，我们只读：
 *   track_left[row]   左边界列坐标
 *   track_right[row]  右边界列坐标
 *   track_center[row] 中线列坐标      ← ★我们直接用它，不再自己 (左+右)/2
 *   track_lost[row]   这一行哪条线是丢的：bit0 = 左线丢  bit1 = 右线丢
 *   row 0 = 最远（画面顶部），VISION_H-1 = 最近（画面底部）
 * 十字/圆环补线、丢线插值、中值滤波、跑出画面贴边，他们那边都已经做完了 ——
 * 所以"这一行可不可信"不再靠宽度猜，而是看两条：
 *   ① 宽度要在 [CAR_TRACK_W_MIN, CAR_TRACK_W_OPEN] 之间（兜底，挡明显异常的）
 *   ② 边线不能是贴到画面边上的（那种是"跑出画面"，中线是猜的）
 * 三行都不可信 或 三行都有丢线标志 -> 按十字(短暂)/丢线(持续)处理 */
#define CAR_TRACK_W_MIN         (VISION_PX(10))    // 小于这个宽度认为该行不可信(像素)
#define CAR_TRACK_W_OPEN        (VISION_PX(268))   // 大于这个宽度认为该行不可信(十字/兜底，像素)
#define CAR_TRACK_PIN_EDGE      (VISION_PX(2))     // 边线贴到画面边（左线 <= 2 列 / 右线 >= 185 列）= 这条线跑出画面了，这一行的中线不可信
#define CAR_OPEN_AS_LOST_FRAMES (25)    // 连续这么多帧"三行都不可信"才判定为丢线(约 0.5s @50fps)

/*====================================================================================================================
 * 七、起步与安全
 *===================================================================================================================*/
#define CAR_USE_START_BUTTON    (0)     // 0: 上电延时 CAR_START_DELAY_MS 后自动起步
                                        // 1: 等按下主板 KEY1(PB2) 再起步（推荐：先把车摆好再按键）
#define CAR_START_BUTTON_PIN    (IO_PB2)    // 主板 KEY1（按下为低电平）
#define CAR_START_DELAY_MS      (5000)      // 自动起步延时(ms)
#define CAR_FRAME_TIMEOUT_MS    (500)       // 超过这么久没有新图像（视觉卡住/死机）-> 立刻停车并锁死

#define CAR_TELEMETRY_EVERY     (1)         // 每 N 帧上传一次图像+边线到逐飞助手：fw27 从 1 改成 3。
#define CAR_TELEMETRY_SLOW_MS  (300)       // ★链路自查：单次发图超过这么多 ms 就算"链路卡住"（正常一张图只要几 ms）
                                           //   fw42 起判据改成"最近 CAR_LINK_WIN_SENDS 张里有 CAR_LINK_WIN_MAX 张超时"就停发图像+示波器，
                                           //   并且每 CAR_LINK_PROBE_MS 毫秒"试发一张"，顺利就自动恢复（旧版要求连续 3 次，偶发卡顿抓不住）
#define CAR_LINK_WIN_SENDS      (6)         // 【fw42】链路自查窗口：最近这么多张图里……
#define CAR_LINK_WIN_MAX        (2)         //   ……有这么多张发图超时 -> 停发图像 + 示波器（旧版是"连续 3 次"，偶发卡顿抓不住）
#define CAR_LINK_PROBE_MS       (5000)      // 【fw42】停发之后每隔这么多毫秒"试发一张"，顺利就自动恢复
                                                        // 上传会占主循环时间，占太久"视觉心跳看门狗"会误判成视觉卡死把车锁死

/* 是否使用 WiFi + 逐飞助手（图像上传）：
 *   1 = **默认**：照原来的样子连 WiFi + TCP，把图像和边线传给逐飞助手看图。
 *       编码器 / 速度这些数字不用它 —— 那些走 USB 串口（串口助手）看 printf
 *   0 = 不连 WiFi、不连 TCP、也不上传图像：没热点、没开上位机也能正常启动跑车（台架自检、纯串口调试用）
 * 注意：设 0 不影响摄像头和视觉算法，只影响"要不要连无线、要不要把图像发出去" */
#define CAR_USE_WIFI_LINK       (1)     // 【fw49】1 = 连 WiFi + 上传图像（用户要分析图像时打开）；跑车按无线串口 ! 停发图像
                                        //   打开后：PC 先连热点 YOUR_WIFI_SSID / YOUR_WIFI_PASSWORD，本机 IP 要是 YOUR_PC_IP，再开逐飞助手当 TCP 服务端(8086)
                                        //   ★图像上传是阻塞的 -> 帧率会掉；跑车时按无线串口 ! 就能临时停发图像（不用重新烧录）
                                        // 【fw42】跑车时改成 0：完全不连 WiFi / 不连 TCP / 不发图，主循环不被链路拖累；
                                        //   要在助手看图像调试时再改回 1（此时链路一卡会自动停发、通了自动恢复）

/* 开机要不要主动去连上位机的 TCP（逐飞助手当 TCP Server、端口 8086）：
 *   1 = 连。**只有连上了图像才能传到逐飞助手**；IP/端口/上位机没配好时，这一步最多会卡 50 秒。
 *   0 = 不连。开机不会被它拖住，数字上报和跑车全都正常，只是看不到图像。
 * 把电脑 IP 填进 zf_device_wifi_spi.h 的 WIFI_SPI_TARGET_IP、逐飞助手开好 TCP Server 之后，再改回 1。 */
#define CAR_WIFI_TCP_CONNECT    (1)     // ★ 1 = 主动连上位机 TCP（图像才能传到逐飞助手）。IP/上位机没配好时开机最多卡 50 秒
/* 视觉链路（WiFi -> TCP -> 摄像头）每一步最多重试多少次，超了就跳过视觉、直接进主循环。
 *   逐飞例程里这三步都是 while(失败){等100ms} 的**死循环**：没插 WiFi 模块 / 没开热点 /
 *   没开逐飞助手 / 没接摄像头，程序就永远卡在那里 -> 舵机、电机、编码器、无线串口一个都不初始化，
 *   串口助手上什么数据都没有（看着就像"编码器连不上串口助手"，其实是被卡在 WiFi 那一步了）。
 *   给 60 次、每次等 100ms（约 6 秒）：正常连 WiFi 远用不了这么久，连不上就别死等。
 *   0 = 保持原例程的"无限重试"（不推荐：一失败就完全没有串口输出，没法排查）。
 *   注意：跳过视觉只影响"要不要跑图像"，不影响摄像头和视觉算法本身。 */
#define CAR_VISION_TRY_MAX      (2)     // 每步最多重试 2 次。★库里超时很大：连热点一次 10s、连 TCP 一次 50s，别设大！

/* 连上位机 TCP 那一步最多试几次：库里 SOCKET_CONNECT_TIME_OUT = 50 秒/次，所以**只能给 1**。
 * 试 2 次就是开机等 100 秒，没人受得了。IP/上位机没配好时它会失败一次然后继续跑。 */
#define CAR_TCP_TRY_MAX         (1)

/* 用"无线转串口模块"把编码器等数据发到电脑的串口助手（车上不用插 USB 线）
 *   模块插在主板的无线串口插座上（逐飞主板 = UART8：模块 TX→P92、模块 RX→P93），
 *   电脑端插配套的 USB 接收头，会多出一个 COM 口，串口助手选它、波特率 115200
 * 下面 4 个参数是为了"万一你主板插座接线不同 / 模块波特率不是 115200"能直接改，
 * 改完重新编译即可，不用动其它代码 */
#define CAR_WIRELESS_UART_ENABLE (1)        // 0 = 关掉无线串口上报
#define CAR_WL_UART              (UART_8)       // 用哪个串口
#define CAR_WL_BAUD              (115200)       // 模块波特率（不是 115200 就改这里，常见 9600）
#define CAR_WL_TX_PIN            (UART8_TX_P93) // 单片机 TX -> 模块 RX
#define CAR_WL_RX_PIN            (UART8_RX_P92) // 单片机 RX <- 模块 TX
#define CAR_WL_MAX_LINE         (200)   // 【fw61】无线串口单条消息的硬上限(字节)：超过就截断，保证串口助手那边一行不超过它

/*====================================================================================================================
 * 八、调试与台架自检
 *===================================================================================================================*/
#define CAR_BRINGUP_TEST        (0)     // 1: 上电先做一次台架自检（会自己转轮子）；上赛道要视觉，必须 0
                                        //    最后分两步（只转左轮 / 只转右轮）打印编码器计数，用来第一次确认接线和方向。
                                        //    当前恢复正常循迹，默认 0；已验证的台架固件单独保留为 fw72-noenc。
                                        //    改成 1 时跳过 WiFi / 逐飞助手 / 摄像头，上电自动转轮自检。
                                        //    编码器监测当前关闭，计数阶段及实测轮速暂不更新。
                                        //    ★ 跑赛道之前务必改回 0：自检会自己转轮子，而且置 1 时车不会自己跑赛道。

/* 台架自检时电机给的固定占空比(万分比)：2000 = 20%
 *   自检这一段是**纯开环**（不看编码器、不跑速度环），因为自检时编码器接线还没验证，
 *   闭环会因为读数乱而把电机一会儿正一会儿反，分不清是接线问题还是闭环在打架。
 *   转不动就加大（比如 3000），上限是 CAR_MOTOR_DUTY_MAX */
#define CAR_TEST_DUTY           (2000)

/* 台架自检时跳过整条视觉链路（WiFi / 逐飞助手 / 摄像头）：
 *   1 = 跳过。main.c 的顺序是"先连 WiFi -> 再连逐飞助手 -> 再初始化摄像头"，任一步失败就一直在那里重试，
 *       而台架自检（舵机 / 电机 / 编码器）根本用不到视觉。置 1 之后只插一根 USB 线就能看串口自检输出，
 *       不用开手机热点、不用开逐飞助手、不用接摄像头。
 *   只有在 CAR_BRINGUP_TEST = 1 时它才起作用；CAR_BRINGUP_TEST = 0（正式跑车）时它自动失效。 */
#define CAR_BRINGUP_SKIP_VISION (1)

#if ((1 == CAR_BRINGUP_TEST) && (1 == CAR_BRINGUP_SKIP_VISION))
    #define CAR_SKIP_VISION     (1)
#else
    #define CAR_SKIP_VISION     (0)
#endif

/* Diagnostic network traffic shares the blocking image link. */
#define CAR_OSC_EVERY_FRAMES (3)

#endif
