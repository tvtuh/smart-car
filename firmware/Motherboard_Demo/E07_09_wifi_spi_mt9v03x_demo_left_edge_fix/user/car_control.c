/*********************************************************************************************************************
* 整车闭环控制：视觉中线 -> 打角 / 速度
*
* 分工：
*   视觉算法输出 track_left / track_center / track_right / track_lost（row 0 = 最远）。
*   本模块负责"盯着中线开车"：
*     1. 近/中/远三行中线加权 -> 瞄点误差 -> PD -> 舵机打角（远处权重大 = 预瞄，过弯才稳）
*     2. 弯道程度（远端偏移 - 近端偏移）-> 自动减速；十字压速；丢线爬行、久丢停车
*     3. 打角 -> 左右轮差速：内侧轮减速、外侧轮加速
*     4. 起步门（延时或按 KEY1）、视觉心跳看门狗、故障锁死
*
* 重要：本模块只"读"视觉的结果，不改动视觉算法一个字
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入
********************************************************************************************************************/

#include "car_config.h"
#include "car_control.h"
#include "car_motor.h"
#include "car_servo.h"
#include "car_drive.h"

/* 打印用：把浮点按 x10 / x100 四舍五入成整数。
 * 原来直接用 (int16) 截断，kp = 0.45 会打印成 44、steer = 0.45 打印成 4，
 * 看着像“参数被改了”，很容易误判。这里全部改成四舍五入 */
#define CAR_P10(x)   ((int16)(((x) >= 0.0) ? ((x) * 10.0  + 0.5) : ((x) * 10.0  - 0.5)))
#define CAR_P100(x)  ((int16)(((x) >= 0.0) ? ((x) * 100.0 + 0.5) : ((x) * 100.0 - 0.5)))

/*-------------------------------------------------------------------------------------------------------------------
 * 视觉链路的输出（code/ 里那四个模块，视觉组给的那份），这里只读引用
 *   code/track_edge.h 里声明了这几个数组，直接 include 它，免得类型/名字对不上：
 *     track_left[row]   左边界列坐标      track_right[row] 右边界列坐标
 *     track_center[row] 中线列坐标        ← ★直接用它，不再自己 (左+右)/2
 *     track_lost[row]   这一行哪条线是丢的（bit0 左 / bit1 右）
 *   row 0 = 最远（画面顶部），VISION_H-1 = 最近（画面底部）
 *   十字/圆环补线、丢线插值、中值滤波、跑出画面贴边，他们那边都做完了 ——
 *   我们只读最终结果，视觉算法一个字都不改
 *-----------------------------------------------------------------------------------------------------------------*/
#include "track_edge.h"
#include "ring_road.h"                 /* 【fw45】圆环策略要用它给的 ring_flag / ring_type / ring_state */

/* 图像水平中心：中线误差以它为基准，误差为正表示中线在画面右侧（车偏左了，要往右打） */
#define CAR_IMAGE_CENTER        (VISION_W / 2)

/* 车状态 */
static uint8   s_started       = 0;             /* 1 = 已起步，允许电机输出 */
static uint8   s_fault_shown   = 0;             /* 故障提示只打印一次 */
static uint16  s_lost_frames   = 0;             /* 连续丢线帧数 */
static uint16  s_open_frames   = 0;             /* 连续"三行都不可信"的帧数：用来分十字(短暂)和丢线(持续) */
static uint16  s_frame_cnt    = 0;              /* 控制循环处理过的图像帧数：不涨 = 视觉一帧都没进来 */
static int16   s_w_mid_last   = 0;              /* 最近一帧"中间行"量到的赛道宽度：0 = 这一行不可信（没找到线） */
static int16   s_w_near_last   = 0;             /* 最近一帧“近处行”量到的宽度：0 = 这一行不可信 */
static int16   s_w_far_last    = 0;             /* 最近一帧“远处行”量到的宽度：0 = 这一行不可信 */

/* 近/中/远三行各自的"半赛道宽"(像素)：只在"两边边线都在画面里"的行上慢速更新。
 * 弯道里外侧边线会跑出画面（视觉组把它贴到 0 / 187），这时用它 + 内侧那条可靠边线把中线估出来 ——
 * 否则那几行只能丢掉，而弯道里偏偏就是那几行的偏移最大，丢掉等于"看不见弯"。
 * 初值按实测的 w=128/108/83 填。 */
static int16   s_hw_near       = VISION_PX(64);
static int16   s_hw_mid        = VISION_PX(54);
static int16   s_hw_far        = VISION_PX(41);
/* Live preview rows; p/P moves mid and far together within +/-10 rows. */
static uint8 s_row_mid = CAR_ROW_MID;
static uint8 s_row_far = CAR_ROW_FAR;
static uint8 s_preview_changed = 0;
static int16   s_err_near_last = 0;             /* 最近一帧近/中/远三行的中线偏移(像素)：诊断"视觉中线是不是偏的" */
static int16   s_err_mid_last  = 0;
static int16   s_err_far_last  = 0;
static int16   s_aim_last      = 0;             /* 上一帧瞄点误差，算微分用 */
static float   s_steer_last    = 0.0;           /* 上一帧打角量，丢线保持用 */
static float   s_steer_i       = 0.0;           /* 打角积分累加（默认不启用） */
static uint8   s_steer_inv     = CAR_STEER_INVERT;  /* 打角整体取反（n 命令在线切）：赛道上一试就知道方向对不对 */
static float   s_steer_max     = CAR_STEER_MAX_DEG; /* 打角限幅(度)：可在线调（, / .） */
static float   s_steer_kp      = CAR_STEER_KP;        /* 比例系数：可在线调（( / )） */
static float   s_steer_kd      = CAR_STEER_KD;        /* 微分系数：可在线调（< / >） */
static float   s_steer_kp_soft = CAR_STEER_KP_SOFT;   /* 【fw60】双增益的"软"增益：小误差时用（在线 9 / x 调） */
static float   s_d_aim_filt    = 0.0;                 /* 微分项的滤波值（一阶低通，抗视觉噪声） */
/* 【fw45】圆环行驶策略的运行时状态（在线可调，见 car_config.h 的 CAR_RING_*） */
static uint8   s_ring_on       = CAR_RING_ENABLE; /* Legacy ring fallback enable from CAR_RING_ENABLE; no UART toggle. */
static float   s_ring_steer    = CAR_RING_STEER_INSIDE; /* Legacy fixed steering from CAR_RING_STEER_INSIDE; no UART adjustment. */
static int16   s_ring_speed    = CAR_RING_SPEED_INSIDE;     /* 环内速度(cm/s)（在线按 l / L 调） */
static int8    s_ring_dir_sign = CAR_RING_DIR_SIGN; /* Legacy fixed-steering direction from configuration. */
static uint8   s_ring_hold     = 0;                         /* 出环"交还打角"剩余帧数 */
static uint8   s_ring_active   = 0;                         /* 1 = 本帧圆环策略接管（跳过十字/丢线处理） */

/* 【fw47】圆环行驶 v2 的运行时状态：识别之后先直行过第一个路口，到第二个路口才打角（见 car_config.h） */
static uint8   s_rp_phase      = CAR_RP_IDLE;   /* 序列阶段：0 没进 1 直行 2 进环 3 交还 */
static uint16  s_rp_frames     = 0;             /* 当前阶段已经过了几帧 */
static uint16  s_rp_total      = 0;             /* 整个圆环序列已经过了几帧（兜底超时用） */
static uint8   s_rp_seen       = 0;             /* 1 = 已经看到"开口"（入口侧大量丢线） */
static uint8   s_rp_ok         = 0;             /* "开口已经到车头"连续成立的帧数 */
static uint8   s_rp_mask       = 0;             /* 入口侧在 track_lost 里的位（1 = 左 2 = 右），识别那一帧锁下来 */
static uint8   s_rp_enter_frm  = CAR_RING_ENTER_FRAMES; /* Legacy entry duration from configuration. */
static int16   s_rp_enter_row  = CAR_RING_ENTER_ROW; /* Legacy entry row from configuration; p/P now changes preview. */
static int16   s_rp_aim_hold   = 0;             /* 识别前锁下来的瞄点：直行阶段用它，抵消视觉组"朝环内补斜线" */
static int16   s_rp_lost       = 0;             /* 本帧入口侧丢线行数（诊断上报用） */
static int16   s_rp_first      = -1;            /* 本帧入口侧最靠前(最远)的丢线行，-1 = 这一侧没丢线（诊断用） */
static int16   s_rp_first_l    = -1;            /* 左侧最靠前的丢线行（每帧都算：进环时机看的就是它） */
static int16   s_rp_first_r    = -1;            /* 右侧最靠前的丢线行 */
static uint8   s_rp_gate_on    = CAR_RING_GATE_LEVEL; /* Legacy fallback gate from configuration. */
static int16   s_rp_run_l     = 0;              /* 【fw47c】左侧最长连续丢线段（行）：判"环在哪一侧"用它 */
static int16   s_rp_run_r     = 0;              /* 右侧最长连续丢线段（行） */
static int16   s_rp_wide       = 0;              /* 【fw60】30~60 行里左右都有效时的最大赛道宽度(px)：环口一定比正常赛道宽 */
static uint8   s_rp_near_l     = 0;              /* 【fw60】左侧在近底部(100/106/112 行)还有边线的行数(0~3) */
static uint8   s_rp_near_r     = 0;              /* 【fw60】右侧同上 */
static uint8   s_rp_near       = 0;              /* 【fw60】入口侧在近底部还有边线的行数（上报用） */
static uint8   s_rp_oside     = 0;              /* 我们自己判出来的入口侧（1 = 左 2 = 右），上报用 */
static int16   s_curve_peak   = 0;              /* 弯道程度的"快衰减峰值"：【fw47c】闸门用它 —— 弯道里偶尔一帧干净也骗不过去 */
static float   s_curve_k      = CAR_SPEED_CURVE_K; /* Auto-speed coefficient from configuration; fixed-speed mode is active by default. */
static float   s_curve_early_k = CAR_CURVE_EARLY_K; /* Auto-speed early-brake coefficient from configuration. */
static uint8 s_drive_normalized = CAR_DIFF_NORMALIZED_DEFAULT;
static uint8 s_turn_limit = CAR_TURN_LIMIT_DEFAULT;
static car_drive_output_t s_drive_output;
static float s_pwm_steer = 0.0f;
static float   s_diff_k       = CAR_DIFF_K;        /* 【fw53】差速系数（在线按 3 / 4 调）：内轮减速/外轮加速 = 它 x 打角(度) */
static uint8   s_img_on       = 1;              /* 【fw48】图像上传的运行时开关（在线按 ! 切换）：跑车时停发图像，连接保持 */
static uint16  s_mod_skip      = 0;             /* 【fw47e】一共跳过多少次"圆环识别"（弯道里不叫它，诊断用） */
static uint8   s_rp_aim_lock  = 0;              /* 【fw47e】识别到了但闸门没过 = 那个"圆环"不可信：瞄点也锁成直行基准，
                                                 *   否则视觉组的补线会把中线往一边推 -> 直道画龙 */
static uint16  s_rp_label_fix  = 0;             /* 【fw47d】改过几次视觉组的 ring_type 标签（诊断） */
static uint8   s_rp_gate_ok    = 0;             /* 这一轮识别有没有过闸门（诊断） */
static uint16  s_rp_gate_fail  = 0;             /* 闸门一共挡掉了几次误报（诊断，? 里看） */
static uint8   s_rp_vconf      = 0;             /* 视觉组自己有没有看到"环内特征"（严格档进环的必要条件） */
static uint8   s_rp_abort      = 0;             /* 直行阶段"弯道很大"连续帧数（误判兜底） */
static float   s_rp_aim_pre    = 0.0;           /* 还没识别到圆环时的"干净"瞄点 = 直行阶段锁的基准 */
static int16   s_curve_last    = 0;             /* 最近一帧的弯道程度（上报用：CT 的 cv、? 的 cv） */
static float   s_aim_slow      = 0.0;           /* 瞄点的慢速平均（识别那一刻取上一帧的值当直行基准） */
static int16   s_speed_base     = CAR_SPEED_BASE_CM_S;  /* 直道目标速度(cm/s)：★可在线调（u = 减速、i = 加速） */
static int16   s_speed_fixed    = CAR_SPEED_FIXED_CM_S;  /* 【fw56】固定速度(cm/s)（在线仍可用 u / i 调）：弯道直道同速 */
static uint16  s_run_frames     = 0;            /* 【fw56】连续"速度 > 0"的帧数：起步助推用（静止起步那一下） */
static int16   s_speed_last     = 0;                    /* 最近一帧实际下发的目标速度：给无线串口看 */
static uint16  s_last_tick     = 0;                    /* 上一帧的 10ms tick：算控制周期 dt 用 */
static uint16  s_dt_ms         = 0;                    /* 两帧之间过了多少毫秒：上报里看主循环到底多快 */


/* 舵机自测（无线串口发 k）：用来把"舵机/供电坏了"和"视觉没给出打角"分开。
 * 自测期间由 s_servo_hold_deg 接管舵机，否则会被每帧的视觉打角覆盖掉 */
static uint8   s_servo_test_on  = 0;            /* 1 = 正在做舵机自测 */
static uint8   s_servo_step     = 0;            /* 自测走到第几步 */
static uint16  s_servo_seen     = 0;            /* 上一步的 tick */
static uint8   s_servo_hold     = 0;            /* 1 = 舵机由 s_servo_hold_deg 接管 */
static float   s_servo_hold_deg = 0.0;

#if CAR_WIRELESS_UART_ENABLE
static void car_wireless_init(void);            /* 定义在文件后半部分，这里先声明（C 要求先声明后使用） */
#endif

#if CAR_BRINGUP_TEST
/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     自检专用延时：边等边喂心跳，防止自检中途被看门狗停车
 *-----------------------------------------------------------------------------------------------------------------*/
static void car_test_delay(uint16 ms)
{
    uint16 i;

    for(i = 0; i < (uint16)(ms / 10); i++)
    {
        motor_heartbeat();
        car_wireless_poll();                /* 自检期间也把数据发出去，免得看起来像"串口不通" */
        system_delay_ms(10);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     自检专用：把舵机缓慢打到指定打角量
 * 备注信息     舵机有 CAR_SERVO_SLEW_DEG 的缓变限制，所以要反复下发 + 等待
 *-----------------------------------------------------------------------------------------------------------------*/
static void car_servo_move(float steer_deg)
{
    uint16 i;

    for(i = 0; i < 40; i++)                     /* 40 * 10ms = 400ms */
    {
        servo_set_steer(steer_deg);
        motor_heartbeat();
        car_wireless_poll();                /* 自检期间也把数据发出去（和图像帧无关） */
        system_delay_ms(10);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     台架自检：确认舵机方向、左右电机方向、编码器计数方向
 * 备注信息     只在 CAR_BRINGUP_TEST = 1 时编译进来，跑赛道前必须改回 0
 *-----------------------------------------------------------------------------------------------------------------*/
static void car_bringup_test(void)
{
    uint16 i;
    int16  sum_l = 0;
    int16  sum_r = 0;

    motor_run(1);                               /* 自检要能驱动电机 */
    motor_test_mode(1);                         /* 自检期间关掉编码器保护，否则接错线会被提前锁死、看不出问题 */

    car_servo_move(-20.0);
    car_servo_move(0.0);
    car_servo_move(20.0);
    car_servo_move(0.0);

    /* 下面两段是**纯开环**固定占空比：只看"哪一侧 + 往前还是往后"。
     * 不用闭环，是因为自检时编码器接线还没验证，闭环会跟着乱读数把电机一会儿正一会儿反，
     * 那样就分不清"接反了"还是"闭环在打架"。 */
    motor_test_duty(CAR_TEST_DUTY, 0);
    car_test_delay(1500);
    motor_test_duty(0, 0);
    car_test_delay(800);

    motor_test_duty(0, CAR_TEST_DUTY);
    car_test_delay(1500);
    motor_test_duty(0, 0);
    car_test_delay(800);

    sum_l = 0;
    sum_r = 0;
    for(i = 0; i < 200; i++)
    {
        sum_l = (int16)(sum_l + motor_enc_left());
        sum_r = (int16)(sum_r + motor_enc_right());
        motor_heartbeat();
        system_delay_ms(10);
    }

    sum_l = 0;
    sum_r = 0;
    for(i = 0; i < 200; i++)
    {
        sum_l = (int16)(sum_l + motor_enc_left());
        sum_r = (int16)(sum_r + motor_enc_right());
        motor_heartbeat();
        system_delay_ms(10);
    }

    motor_test_mode(0);                         /* 恢复编码器保护 */
    motor_run(0);
}
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     整车控制初始化
 * 参数说明     void
 * 返回参数     void
 * 备注信息     调用顺序：必须在 mt9v03x_init() 之后（时钟已经配好），在进入主循环之前
 *              上电后舵机立刻回中位，电机保持静止，直到 car_control_wait_start() 才允许输出
 *-----------------------------------------------------------------------------------------------------------------*/
void car_control_init(void)
{
    servo_init();
    servo_set_soft_limit(s_steer_max);             /* 让舵机模块的软限幅和打角限幅保持一致 */
    motor_init();

#if CAR_WIRELESS_UART_ENABLE
    car_wireless_init();                            /* 无线转串口模块：P93=TX / P92=RX，115200 */
#endif
    s_started     = 0;
    
    
    
    s_run_frames  = 0;
    s_fault_shown = 0;
    s_lost_frames = 0;
    s_aim_last    = 0;
    s_steer_last  = 0.0;
    s_steer_i     = 0.0;


#if CAR_BRINGUP_TEST
    car_bringup_test();
#endif

    /* 上电先把电机输出硬关：上电后车绝对不动，可以先手推车确认打角方向，发 o 才出力。
     * 台架自检要自己转轮子，所以那种配置下这个开关不生效（见 car_config.h） */
#if ((1 == CAR_BOOT_OUTPUT_OFF) && (1 != CAR_BRINGUP_TEST))
    motor_output_off(1);
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     起步门
 * 参数说明     void
 * 返回参数     void
 * 备注信息     CAR_USE_START_BUTTON = 0：上电延时 CAR_START_DELAY_MS 后自动起步（台架调试用）
 *              CAR_USE_START_BUTTON = 1：等按下主板 KEY1(PB2) 再起步，按下后还要等松手，
 *                                        避免一按就冲出去（推荐比赛/正式跑车用）
 *-----------------------------------------------------------------------------------------------------------------*/
void car_control_wait_start(void)
{
    uint16 i;

#if CAR_USE_START_BUTTON
    gpio_init(CAR_START_BUTTON_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    while(gpio_get_level(CAR_START_BUTTON_PIN))
    {
        system_delay_ms(10);
    }
    while(!gpio_get_level(CAR_START_BUTTON_PIN))        /* 等松手 */
    {
        system_delay_ms(10);
    }
#else
    for(i = 0; i < (uint16)(CAR_START_DELAY_MS / 100); i++)
    {
        motor_heartbeat();                          /* ★ 起步延时这一段没有任何图像帧：
                                                     * 只要电机被使能过（发过 o / t），"视觉心跳看门狗"
                                                     * （500ms）就会把车锁死。这里一直喂着，保证起步是干净的 */
        if(0 == (uint16)(i % 5))                    /* 每 500ms 报一次进度 */
        {
        }
        system_delay_ms(100);
    }
#endif

    s_started     = 1;
    
    
    
    s_fault_shown = 0;
    s_lost_frames = 0;
    s_steer_i     = 0.0;
    s_steer_last  = 0.0;
    s_aim_last    = 0;

    motor_run(1);                                       /* 放开电机 */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     停车
 * 参数说明     void
 * 返回参数     void
 * 备注信息     退回未起步状态，需要重新调用 car_control_wait_start() 才会再跑
 *-----------------------------------------------------------------------------------------------------------------*/
void car_control_stop(void)
{
    
    s_started = 0;
    motor_run(0);
    motor_stop();
    servo_center();
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     由第 row 行的左/右边界算"中线相对画面中心的偏移"
 * 参数说明     row         行号（0 = 最远）
 *               w           输出：这一行量到的赛道宽度(像素)，0 表示这一行不可信
 * 返回参数     中线偏移(像素，正 = 中线偏右；这一行不可信时返回 0)
 * 备注信息     宽度不合理（太窄 or 几乎铺满整幅）就认为这一行没有可信赛道。
 *              视觉组的算法找不到线时会用种子值兜底（左右都贴到画面边缘、宽度约等于整幅），
 *              所以"宽度异常大"正是他们那边"这行没东西"的表现
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 row_center_err(uint8 row, int16 *w)
{
    int16 l;
    int16 r;
    int16 width;
    int16 centre;
    int16 hw;

    l = (int16)track_left[row];
    r = (int16)track_right[row];
    width = (int16)(r - l);
    hw = (row == CAR_ROW_NEAR) ? s_hw_near : ((row == s_row_mid) ? s_hw_mid : s_hw_far);

    *w = 0;
    if((l >= VISION_W) || (r >= VISION_W) ||
       (track_center[row] >= VISION_W) || (r <= l)) { return 0; }

    /* A confirmed virtual entrance lane uses the drawn centre, even though one edge is at the image border. */
    if(0 != ring_guide_row_valid(row))
    {
        *w = width;
        return (int16)VISION_ERROR_REF((int16)track_center[row] - (int16)CAR_IMAGE_CENTER);
    }

    /* Interpolation alone is not a valid route when both physical edges are lost. */
    if(0x03 == (track_lost[row] & 0x03)) { return 0; }
    if((width < CAR_TRACK_W_MIN) || (width > CAR_TRACK_W_OPEN))
    {
        return 0;                                   /* 宽度不合理：这一行什么都没有 */
    }

    if((l > CAR_TRACK_PIN_EDGE) && (r < (int16)(VISION_W - 1 - CAR_TRACK_PIN_EDGE)))
    {
        /* 两条边线都在画面里：完全可信。顺手把"半赛道宽"慢慢更新（慢滤波，单帧异常带不偏） */
        if(row == CAR_ROW_NEAR)     { s_hw_near = (int16)((s_hw_near * 7 + width / 2) / 8); }
        else if(row == s_row_mid) { s_hw_mid  = (int16)((s_hw_mid  * 7 + width / 2) / 8); }
        else                        { s_hw_far  = (int16)((s_hw_far  * 7 + width / 2) / 8); }

        *w = width;
        return (int16)VISION_ERROR_REF((int16)track_center[row] - (int16)CAR_IMAGE_CENTER);
    }

    /* 一边跑出画面（视觉组把那条线贴到了 0 / 187）：
     *   绝不能拿贴边的值直接算中线 —— 那会把中线往"出画面那一侧"拽，而弯道里正好是反方向，
     *   等于让车往弯外打方向（这是"弯道打角不够 / 方向怪"的元凶之一）。
     *   正确做法：用"还在画面里的那条可靠边线 ± 半赛道宽"把中线估出来。 */
    if((l > CAR_TRACK_PIN_EDGE) && (r >= (int16)(VISION_W - 1 - CAR_TRACK_PIN_EDGE)))
    {
        centre = (int16)(l + hw);                   /* 右线出画面 -> 用左线推 */
    }
    else if((r < (int16)(VISION_W - 1 - CAR_TRACK_PIN_EDGE)) && (l <= CAR_TRACK_PIN_EDGE))
    {
        centre = (int16)(r - hw);                   /* 左线出画面 -> 用右线推 */
    }
    else
    {
        return 0;                                   /* 两条都出画面：这一行真没辙 */
    }

    if(centre < 0)                      { centre = 0; }
    if(centre > (int16)(VISION_W - 1)) { centre = (int16)(VISION_W - 1); }

    *w = width;                                     /* 报个非 0 = 这一行可用 */
    return (int16)VISION_ERROR_REF(centre - (int16)CAR_IMAGE_CENTER);
}


#if CAR_WIRELESS_UART_ENABLE
static uint8  s_wl_ready     = 0;
static uint8  s_wl_motor_on = 0;                    /* 电机输出开关（发 o 切换），调编码器时先关掉 */
/* Bounded UART transmit buffer. */
static char s_wl_line[CAR_WL_MAX_LINE + 2];
static void car_wl_send(const char *s)
{
    uint16 len;
    uint16 max_len;

    if(0 == s_wl_ready)
    {
        return;
    }

    max_len = CAR_WL_MAX_LINE;
    len = (uint16)strlen(s);
    if(len > max_len)
    {
        len = (uint16)(max_len - 2);
        memcpy(s_wl_line, s, (uint32)len);
        s_wl_line[len]     = '\r';
        s_wl_line[len + 1] = '\n';
        s_wl_line[len + 2] = '\0';
        uart_write_string(CAR_WL_UART, s_wl_line);
        return;
    }

    uart_write_string(CAR_WL_UART, s);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     点亮无线串口模块（可以重复调用；已经点亮就直接返回）
 * 备注信息     ★ 为什么必须能在"连 WiFi / 摄像头之前"调用 ★
 *   视觉链路三步出错时会重试，而库里的超时很大：连热点一次 10 秒、连 TCP 一次 50 秒。
 *   原来无线串口是在 car_control_init() 里才初始化的，而 car_control_init() 排在这三步后面，
 *   结果就是"开机后串口助手一片空白、要等几分钟才出数据"，看着像模块坏了。
 *   现在 main.c 在 debug_init() 之后立刻调用本函数，1 秒内串口助手里就有东西。
 *-----------------------------------------------------------------------------------------------------------------*/
void car_wireless_link_init(void)
{
    char buf[80];

    if(0 != s_wl_ready)
    {
        return;
    }

    uart_init(CAR_WL_UART, CAR_WL_BAUD, CAR_WL_TX_PIN, CAR_WL_RX_PIN);
    s_wl_ready = 1;

    sprintf(buf, "BOOT %s\r\n", CAR_FW_TAG);
    car_wl_send(buf);
}

/* 往外发一行进度（开机阶段用；没点亮就什么都不做） */
void car_wireless_progress(const char *s)
{
    car_wl_send(s);
}

/* 开机阶段轮询：用户在无线串口里发过 x / X 就返回 1 */
uint8 car_wireless_skip_requested(void)
{
    uint8 c;

    if(0 == s_wl_ready)
    {
        return 0;
    }
    while(uart_query_byte(CAR_WL_UART, &c))
    {
        if(('x' == c) || ('X' == c))
        {
            return 1;
        }
    }
    return 0;
}

static void car_wireless_init(void)
{
    car_wireless_link_init();                   /* main 开头可能已经点过了；没点过就在这里点 */
}
static void car_pwm_report(void)
{
    char buf[100];
    motor_pwm_debug_t state;
    motor_pwm_debug_get(&state);
    sprintf(buf, "DRV HW " CAR_FW_TAG " en=%d enc=%d arr=%u/%u cmp=%u/%u bad=%d dir=%d\r\n",
            (int16)state.enabled, (int16)CAR_ENCODER_FEEDBACK_ENABLE,
            state.period_l, state.period_r, state.compare_l, state.compare_r,
            (int16)state.bad, (int16)state.dir);
    car_wl_send(buf);
}
/* Reply to requested commands during camera idle time. */
static uint8 s_tune_reply = 0;
static uint8 s_help_reply = 0;
static uint8 s_hw_reply = 0;
static void car_tune_report(void)
{
    char buf[120];
    if(1 == s_tune_reply)
    {
        sprintf(buf, "DRV TUNE row=%d/%d/%d slew100=%d",
                (int16)CAR_ROW_NEAR, (int16)s_row_mid, (int16)s_row_far,
                CAR_P100(servo_get_slew()));
        sprintf(buf + strlen(buf), " kp100=%d/%d kd100=%d lim10=%d\r\n",
                CAR_P100(s_steer_kp), CAR_P100(s_steer_kp_soft),
                CAR_P100(s_steer_kd), CAR_P10(s_steer_max));
        s_tune_reply = 2;
    }
    else
    {
        sprintf(buf, "DRV SET v=%d diff100=%d mode=%d cap=%d img=%d",
                (int16)((0 != CAR_SPEED_FIXED_ENABLE) ? s_speed_fixed : s_speed_base),
                CAR_P100(s_diff_k), (int16)s_drive_normalized,
                (int16)s_turn_limit, (int16)s_img_on);
        sprintf(buf + strlen(buf), " out=%d trim10=%d inv=%d ringv=%d k=%d\r\n",
                (int16)(0 == motor_output_off_get()), CAR_P10(servo_get_offset()),
                (int16)s_steer_inv, s_ring_speed, (int16)s_servo_hold);
        s_tune_reply = 0;
    }
    car_wl_send(buf);
}

void car_wireless_poll(void)
{
    uint8 c;
    uint8 need_state = 0;
    uint8 budget = 16;
    if(0 == s_wl_ready) { return; }
    /* Single-byte commands; bounded RX work keeps camera control responsive. */
    while((budget > 0) && uart_query_byte(CAR_WL_UART, &c))
    {
        budget--;
        switch(c)
        {
            case 'p':
                if(s_row_far > (CAR_ROW_FAR - 10))
                { s_row_mid--; s_row_far--; s_preview_changed = 1; }
                need_state = 1; break;
            case 'P':
                if(s_row_mid < (CAR_ROW_MID + 10))
                { s_row_mid++; s_row_far++; s_preview_changed = 1; }
                need_state = 1; break;
            case 'r': case 'R':
                s_row_mid = CAR_ROW_MID; s_row_far = CAR_ROW_FAR;
                s_preview_changed = 1; need_state = 1; break;
            case 'f': case 'F':
                servo_set_slew(servo_get_slew() - CAR_SERVO_SLEW_STEP_DEG);
                need_state = 1; break;
            case 'g': case 'G':
                servo_set_slew(servo_get_slew() + CAR_SERVO_SLEW_STEP_DEG);
                need_state = 1; break;
            case '(':
                s_steer_kp -= 0.05f; if(s_steer_kp < 0.10f) { s_steer_kp = 0.10f; }
                need_state = 1; break;
            case ')':
                s_steer_kp += 0.05f; if(s_steer_kp > 1.50f) { s_steer_kp = 1.50f; }
                need_state = 1; break;
            case '9': case 'x':
                s_steer_kp_soft -= 0.02f; if(s_steer_kp_soft < 0) { s_steer_kp_soft = 0; }
                need_state = 1; break;
            case '/':
                s_steer_kp_soft += 0.02f; if(s_steer_kp_soft > 2) { s_steer_kp_soft = 2; }
                need_state = 1; break;
            case '<':
                s_steer_kd -= 0.05f; if(s_steer_kd < 0) { s_steer_kd = 0; }
                need_state = 1; break;
            case '>':
                s_steer_kd += 0.05f; if(s_steer_kd > 2) { s_steer_kd = 2; }
                need_state = 1; break;
            case ',':
                if(s_steer_max > 4) { s_steer_max -= 1; }
                servo_set_soft_limit(s_steer_max); need_state = 1; break;
            case '.':
                if(s_steer_max < CAR_SERVO_HARD_LIMIT_DEG) { s_steer_max += 1; }
                servo_set_soft_limit(s_steer_max); need_state = 1; break;
            case 'n': case 'N':
                s_steer_inv = (uint8)(0 == s_steer_inv); need_state = 1; break;
            case ';':
                servo_nudge(-CAR_SERVO_OFF_STEP_DEG); need_state = 1; break;
            case '\'':
                servo_nudge(CAR_SERVO_OFF_STEP_DEG); need_state = 1; break;
            case '{':
                servo_nudge(-CAR_SERVO_OFF_COARSE_DEG); need_state = 1; break;
            case '}':
                servo_nudge(CAR_SERVO_OFF_COARSE_DEG); need_state = 1; break;
            case '0':
                servo_set_offset(0); need_state = 1; break;
            case 'u': case 'U':
#if(1 == CAR_SPEED_FIXED_ENABLE)
                s_speed_fixed -= CAR_SPEED_STEP_CM_S;
                if(s_speed_fixed < CAR_SPEED_FIXED_MIN_CM_S) { s_speed_fixed = CAR_SPEED_FIXED_MIN_CM_S; }
#else
                s_speed_base -= CAR_SPEED_STEP_CM_S;
                if(s_speed_base < CAR_SPEED_MIN_CM_S) { s_speed_base = CAR_SPEED_MIN_CM_S; }
#endif
                need_state = 1; break;
            case 'i': case 'I':
#if(1 == CAR_SPEED_FIXED_ENABLE)
                s_speed_fixed += CAR_SPEED_STEP_CM_S;
                if(s_speed_fixed > CAR_SPEED_MAX_CM_S) { s_speed_fixed = CAR_SPEED_MAX_CM_S; }
#else
                s_speed_base += CAR_SPEED_STEP_CM_S;
                if(s_speed_base > CAR_SPEED_MAX_CM_S) { s_speed_base = CAR_SPEED_MAX_CM_S; }
#endif
                need_state = 1; break;
            case '3':
                s_diff_k -= 0.2f; if(s_diff_k < 0) { s_diff_k = 0; }
                need_state = 1; break;
            case '4':
                s_diff_k += 0.2f; if(s_diff_k > 4) { s_diff_k = 4; }
                need_state = 1; break;
            case 'l':
                s_ring_speed -= CAR_RING_SPEED_STEP_CM_S;
                if(s_ring_speed < CAR_RING_SPEED_MIN_CM_S) { s_ring_speed = CAR_RING_SPEED_MIN_CM_S; }
                need_state = 1; break;
            case 'L':
                s_ring_speed += CAR_RING_SPEED_STEP_CM_S;
                if(s_ring_speed > CAR_SPEED_MAX_CM_S) { s_ring_speed = CAR_SPEED_MAX_CM_S; }
                need_state = 1; break;
            case 'o': case 'O':
                s_wl_motor_on = (uint8)(0 == s_wl_motor_on);
                s_run_frames = 0;
                if(0 != s_wl_motor_on)
                {
                      
                    s_open_frames = 0; s_lost_frames = 0;
                    motor_output_off(0); motor_run(1);
                }
                else { motor_output_off(1); }
                need_state = 1; break;
            case '!': s_img_on = (uint8)(0 == s_img_on); need_state = 1; break;
            case '=': s_drive_normalized = (uint8)(0 == s_drive_normalized); need_state = 1; break;
            case '%': s_turn_limit = (uint8)(0 == s_turn_limit); need_state = 1; break;
            case '$': s_hw_reply = 1; break;
            case '?': need_state = 1; break;
            case 'h': case 'H': s_help_reply = 1; break;
            case 'k': case 'K':
                s_servo_test_on = 1; s_servo_step = 0;
                s_servo_seen = motor_tick_get(); s_servo_hold = 1; s_servo_hold_deg = 0;
                need_state = 1; break;
            default: break; /* Retired/unknown commands, CR and LF are ignored. */
        }
    }
    /* Timed state maintenance must run even while TX/camera work is pending. */
    if(0 != s_servo_test_on)
    {
        uint16 now_s = motor_tick_get();
        if((uint16)(now_s - s_servo_seen) >= 40)
        {
            s_servo_seen = now_s;
            switch(s_servo_step)
            {
                case 0: s_servo_hold_deg = 15.0f; break;
                case 1: s_servo_hold_deg = 0.0f; break;
                case 2: s_servo_hold_deg = -15.0f; break;
                case 3: s_servo_hold_deg = 0.0f; break;
                default: s_servo_test_on = 0; s_servo_hold = 0; break;
            }
            s_servo_step++;
            need_state = 1;
        }
    }
    if(0 != need_state) { s_tune_reply = 1; }
    if(mt9v03x_finish_flag) { return; }
    if(0 != s_help_reply)
    {
        s_help_reply = 0;
        car_wl_send("DRV CMD p/P preview; r reset; ? tune; f/g slew; (/) Kp; </> Kd; u/i speed; o motor\r\n");
        return;
    }
    if(0 != s_hw_reply) { s_hw_reply = 0; car_pwm_report(); return; }
    if(0 != s_tune_reply) { car_tune_report(); return; }

}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     主循环空转期间用的"限速轮询"判断：每 10ms 返回一次 1
 * 返回参数     1 = 现在该轮询一次无线串口了    0 = 还不到时候，别浪费 CPU
 * 备注信息     ★为什么要它：主循环在等摄像头出帧时是【全速空转】的，每秒要转几十万圈。
 *              如果每一圈都调 car_wireless_poll()，光这一件事就能把 CPU 吃光
 *              （实测帧率从 18fps 掉到 5.5fps）。
 *              但也不能"只在有图像帧时才轮询"：万一图像停了（摄像头掉线/视觉卡死），
 *              串口就收不到命令、也不上报了，你就没法用 o / ? 救车。
 *              折中：空转期间每 10ms 轮询一次（100Hz，收命令完全够用），有帧时那一帧再轮询一次。
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 car_wireless_poll_due(void)
{
    static uint16 last_tick = 0;
    uint16 now = motor_tick_get();

    if(now != last_tick)
    {
        last_tick = now;
        return 1;
    }
    return 0;
}
#else
/* 无线串口关掉时：这三个接口留空实现，main.c 照常调用不会出错 */
void car_wireless_link_init(void) { }
void car_wireless_progress(const char *s) { (void)s; }
uint8 car_wireless_skip_requested(void) { return 0; }

void car_wireless_poll(void)
{
}

uint8 car_wireless_poll_due(void) { return 0; }
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     把打角下发给舵机（自测期间改用测试角度，保证 k 命令一定看得见效果）
 *-----------------------------------------------------------------------------------------------------------------*/
static void car_servo_apply(float steer)
{
    if(0 != s_servo_hold)
    {
        servo_set_steer(s_servo_hold_deg);
    }
    else
    {
        servo_set_steer(steer);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     一帧控制：每处理完一帧图像调用一次
 * 参数说明     void
 * 返回参数     void
 * 备注信息     调用位置在 main.c 主循环里，紧跟在视觉边线更新（image_update_yellow_boundary）之后、
 *              上传图像到逐飞助手之前 —— 先算控制量再传图，控制延迟最小
 *-----------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------
 * 【fw47d】问一句：这一帧要不要把"圆环识别"这个模块叫起来？（main.c 在 ring_road_process() 之前调用）
 *   为什么需要：视觉组的判据是"入口侧连续丢线 >= 25 行 且 对面那条边是直道"，
 *   而【弯道里外侧边线跑出画面】恰好也是这个特征 —— 弯道里叫它，它就报圆环，
 *   而且它的 ring_fix_side 会把中线往【弯外】推（车冲出弯道，他们文档里也记过这个事故）。
 *   圆环都是在直道上进的，所以弯道里【跳过它】没有副作用：等出了弯（峰值衰减到阈值以下）再叫。
 *   闸门档位按 s 切：0 档永远返回 1（= 完全不干预，老行为）。
 * 参数说明     void
 * 返回参数     1 = 照常调用 ring_road_process()   0 = 这一帧跳过（main.c 会顺手把它的状态清零）
 *-----------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------
 * 【fw48】图像上传的运行时开关（main.c 每次要发图之前问一句）
 *   为什么要有它：发图是【阻塞】的（一次最坏等 1 秒），跑车时会把主循环拖慢、甚至把速度环心跳拖到锁死。
 *   调图像的时候要发，跑车的时候不想发 —— 有了它就不用来回改 CAR_USE_WIFI_LINK 重新烧录了：
 *   无线串口发 ! 就开/关（连接和实时数字完全不受影响）。
 * 参数说明     void
 * 返回参数     1 = 可以发图   0 = 这一帧不发（连接保持，示波器照发）
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 car_img_send_ok (void)
{
    return s_img_on;
}
uint8 car_ring_module_ok (void)
{
#if(1 == RING_FIX_ENABLE)
    return 1;                       /* P 点引导有自己的跨帧/十字闸门，不受旧固定打角开关影响。 */
#else
    if(0 == s_ring_on)             { return 1; }        /* 圆环策略关了：不干预 */
    if(0 == s_rp_gate_on)          { return 1; }        /* 闸门 0 档：不干预 */
    if(CAR_RP_IDLE != s_rp_phase)  { return 1; }        /* 我们自己的圆环序列在跑：让它正常更新 */

    /* 【fw47e】这里用的阈值（CAR_RING_MOD_CURVE_MAX 24）比"接受它"的阈值（12）宽：
     *   原来都按 12 跳过，结果"紧跟在弯道后面"的真圆环也一起不认了（实车：有时候识别不上）。
     *   宽出来的 12~24 这一段里就算它误报，我们也不会进环（闸门挡着），
     *   而且瞄点锁在直行基准上（s_rp_aim_lock），它的补线也带不动车。 */
    if(s_curve_peak > CAR_RING_MOD_CURVE_MAX)
    {
        s_mod_skip++;                                   /* 诊断：? 里的 mskp */
        return 0;
    }
    return 1;
#endif
}



void car_control_update(void)
{
    int16 err_near;
    int16 err_mid;
    int16 err_far;
    int16 w_near;
    int16 w_mid;
    int16 w_far;
    uint8 valid_rows;
    uint8 lost_rows;
    int32 aim_sum;
    int32 aim_w;
    int16 aim;
    int16 curve;
#if(0 == CAR_SPEED_FIXED_ENABLE)
    int16 curve_early;
#endif
    int16 speed;



    float steer;

    s_frame_cnt++;                                      /* 视觉每给一帧就 +1，无线串口里看它涨不涨 */
    {
        uint16 now_t = motor_tick_get();
        s_dt_ms = (uint16)((uint16)(now_t - s_last_tick) * (uint16)CAR_SPEED_PERIOD_MS);
        s_last_tick = now_t;
    }

    /* ---------- 0. 喂心跳：告诉速度环"视觉还活着" ---------- */
    motor_heartbeat();





    if(0 == s_started)                                  /* 还没起步：舵机回正、电机不动 */
    {
        car_servo_apply(0.0);
        return;
    }

    if(0 != motor_is_fault())                           /* 已锁死：停车，并且只提示一次 */
    {
        car_servo_apply(0.0);
        if(0 == s_fault_shown)
        {
            s_fault_shown = 1;
            if(CAR_MOTOR_FAULT_ENCODER == motor_fault_reason())
            {
                car_wireless_progress("FAULT encoder/stall; motor OFF; o clears\r\n");
            }
            else
            {
                car_wireless_progress("FAULT camera timeout; motor OFF; o clears\r\n");
            }
        }
        return;
    }

    /* ---------- 1. 近/中/远三行：左/右边界 -> 中线 -> 加权成"瞄点误差" ----------
     * 正 = 中线在画面右侧（车偏左了，要往右打）。
     * 某一行不可信（宽度异常）时该行偏移按 0 处理 —— 等价于"这一行就按画面中心走" */
    /* ---------- 0b. 【fw47】左右两侧"最靠前(最远)的丢线行"：圆环进环时机就看它 ----------
     *   row 0 = 最远，VISION_H-1 = 最近。某一侧"最靠前的丢线行"越大，说明那一侧的开口
     *   离车头越近；等它 >= CAR_RING_ENTER_ROW 时，入口侧那个开口已经到车头了
     *   （开口的两个拐点都过去了）= 到第二个路口。顺便上报（CT 行的 lf / rf），方便看阈值该放多少。 */
    s_rp_first_l = -1;
    s_rp_first_r = -1;
    {
        int16 yy;

        for(yy = 0; yy < (int16)VISION_H; yy++)
        {
            if((s_rp_first_l < 0) && (0 != (track_lost[yy] & 0x01))) { s_rp_first_l = yy; }
            if((s_rp_first_r < 0) && (0 != (track_lost[yy] & 0x02))) { s_rp_first_r = yy; }
            if((s_rp_first_l >= 0) && (s_rp_first_r >= 0)) { break; }
        }
    }

    /* 【fw47c】同一趟再数一遍"每一侧最长的连续丢线段"：
     *   右环 = 右边有一段很长的连续丢线（实测 43~56 行），左环 = 左边。
     *   实车反馈"左环也往右打"—— 不再只信 ring_type 那个标签，用这个原始数据自己判环在哪一侧。 */
    s_rp_run_l = 0;
    s_rp_run_r = 0;
    {
        int16 run_l = 0;
        int16 run_r = 0;
        int16 yy;

        for(yy = 0; yy < (int16)VISION_H; yy++)
        {
            if(0 != (track_lost[yy] & 0x01))
            {
                run_l++;
                if(run_l > s_rp_run_l) { s_rp_run_l = run_l; }
            }
            else
            {
                run_l = 0;
            }

            if(0 != (track_lost[yy] & 0x02))
            {
                run_r++;
                if(run_r > s_rp_run_r) { s_rp_run_r = run_r; }
            }
            else
            {
                run_r = 0;
            }
        }
    }
    /* 【fw60】闸门用的两个新观测量（每帧实时算，参考 smart-car-origin 的 ring.c）：
     *   ① s_rp_wide：画面 30~60 行里"左右边线都有效"时的最大宽度
     *                —— 环口一定比正常赛道宽（对应他们的 Ring_First_Upper_Width_Ready，ring.c:145-159）
     *   ② s_rp_near_*：两侧在近底部（100/106/112 行）还有边线的行数
     *                —— 用来区分"真开口"和"整条边线丢光"（对应他们的 Ring_*_Side_Ready，ring.c:161-169） */
    s_rp_wide   = 0;
    s_rp_near_l = 0;
    s_rp_near_r = 0;
    {
        uint8 rr;

        for(rr = VISION_PX(30); rr <= VISION_PX(60); rr += VISION_PX(6))
        {
            if(0 == (track_lost[rr] & 0x03))
            {
                int16 ww = (int16)track_right[rr] - (int16)track_left[rr];

                if(ww > s_rp_wide) { s_rp_wide = ww; }
            }
        }

        for(rr = VISION_PX(100); rr <= VISION_PX(112); rr += VISION_PX(6))
        {
            if(0 == (track_lost[rr] & 0x01)) { s_rp_near_l++; }
            if(0 == (track_lost[rr] & 0x02)) { s_rp_near_r++; }
        }
    }
    err_near = row_center_err(CAR_ROW_NEAR, &w_near);
    err_mid  = row_center_err(s_row_mid,  &w_mid );
    err_far  = row_center_err(s_row_far,  &w_far );
    s_w_near_last = w_near;                            /* 三行宽度都给无线串口看：判断某个预览行是不是被丢掉了 */
    s_w_mid_last  = w_mid;
    s_w_far_last  = w_far;
    s_err_near_last = err_near;                        /* 诊断用：三行各自的偏移，从无线串口看 */
    s_err_mid_last  = err_mid;
    s_err_far_last  = err_far;

    valid_rows = 0;
    if(0 != w_near) { valid_rows++; }
    if(0 != w_mid ) { valid_rows++; }
    if(0 != w_far ) { valid_rows++; }
    /* Original frame-count loss fallback below remains active. */

    /* 视觉组还给了"这一行哪条线是补出来的"（track_lost）—— 用它判断这一行还有没有赛道 */
    /* ★只统计"左右两条线同时丢"的行（track_lost == 0x03）。
     *   为什么不能"丢一条就算"：弯道里外侧那条线跑出画面是常态（视觉组的说明里写了），
     *   以前按"有一条丢就算"，结果一入弯三行全被算成丢线 -> 车以为看不见赛道 ->
     *   打角被砍到 70%、速度压到最低 —— 表现就是"明明看见弯了，舵机却不反应"。 */
    lost_rows = 0;
    if(0x03 == (track_lost[CAR_ROW_NEAR] & 0x03)) { lost_rows++; }
    if(0x03 == (track_lost[s_row_mid ] & 0x03)) { lost_rows++; }
    if(0x03 == (track_lost[s_row_far ] & 0x03)) { lost_rows++; }

    /* ★ 只把"可信的行"加权平均。原来某行不可信时是按偏移 0 参与平均的，
     *   等于硬把 aim 往画面中心拉 —— 现在不可信的行直接不参与，权重重新归一化 */
    aim_sum = 0;
    aim_w   = 0;
    if(0 != w_near) { aim_sum += (int32)err_near * CAR_W_NEAR; aim_w += CAR_W_NEAR; }
    if(0 != w_mid ) { aim_sum += (int32)err_mid  * CAR_W_MID;  aim_w += CAR_W_MID;  }
    if(0 != w_far ) { aim_sum += (int32)err_far  * CAR_W_FAR;  aim_w += CAR_W_FAR;  }

    if(0 != aim_w)
    {
        aim = (int16)(aim_sum / aim_w);
    }
    else
    {
        /* 三行都用不了。分两种情况，别一律清成 0：
         *   三行都"左右同时丢"（lost_rows=3）= 十字 / 丢线 -> 直行（同时压速），这是对的；
         *   只是"边线跑出画面"导致用不了 -> 弯道里绝不能让方向自己回正，保持上一帧的打角 */
        aim = (0 != lost_rows) ? 0 : s_aim_last;
    }

    if(aim > CAR_STEER_ERR_MAX)  { aim = CAR_STEER_ERR_MAX;  }
    if(aim < -CAR_STEER_ERR_MAX) { aim = -CAR_STEER_ERR_MAX; }

    /* 仅 RING_FIX_ENABLE=0 的旧固定打角策略会锁瞄点；新 P 点引导直接跟虚拟红线。
     * 【fw47】圆环【直行阶段】：瞄点换成"识别前锁下来的值"，不用本帧中线 ----------
     *   识别到圆环之后，视觉组会把入口侧那条边线补成"朝环内倾斜的斜线"（ring_fix_side），
     *   中线跟着往环里摆 —— 跟它走就等于在第一个路口提前扎进去了。
     *   这里在 PD 之前把瞄点换掉：车按识别前那条直道的偏移直着走，
     *   走到"第二个路口"（开口已经到车头）才由下面的状态机接管打角。
     *   ★基准值 s_rp_aim_pre 只在"还没识别到圆环"的帧更新 —— 误报持续很多帧也不会被补线污染。 */
    s_aim_slow += 0.25f * ((float)aim - s_aim_slow);
    if(RING_TYPE_NONE == ring_type)
    {
        s_rp_aim_pre = s_aim_slow;
    }
#if(0 == RING_FIX_ENABLE)
    if((0 != s_ring_on) && ((CAR_RP_STRAIGHT == s_rp_phase) || (0 != s_rp_aim_lock)))
    {
        aim = s_rp_aim_hold;
    }
#endif

    /* ---------- 2. 弯道程度 = (远端偏移 - 中间偏移) x 2（直道时约等于 0，弯道里很大） ----------
     * fw27 换成"中/远两行"：近处行(row 100)离车最近、画面里赛道最宽，边线容易贴到画面边缘，
     * 实测它就经常比中/远行偏十几个像素 —— 用它算出来的弯道程度在直道上也很大，速度会被一路压到下限。
     * 中/远两行的行距只有近/远的一半，所以乘 2 保持原来的灵敏度（CAR_SPEED_CURVE_K 也相应减半） */
    curve = (int16)((err_far - err_mid) * 2);          /* fw27：改用中/远两行算弯道程度（见上面的说明） */
    if(curve < 0)                   { curve = (int16)(-curve); }
    if(curve > CAR_SPEED_CURVE_MAX) { curve = CAR_SPEED_CURVE_MAX; }
    s_curve_last = curve;                               /* 【fw47b】上报用：CT 的 cv、? 的 cv 都看它 */
    /* 【fw47c】弯道程度的"快衰减峰值"：闸门看它，不看单帧 ----------
     *   实车反馈圆环误报主要在弯道里；弯道中偶尔会有一帧看着挺直，只看单帧就漏过去了。
     *   峰值每帧最多衰减 3，所以"刚出弯"的一小段时间里仍然算大弯道 —— 圆环都是直道上进的，
     *   这一小段等待换来的是"弯道里绝不误进环"。 */
    if(curve > s_curve_peak)
    {
        s_curve_peak = curve;
    }
    else if(s_curve_peak > 3)
    {
        s_curve_peak -= 3;
    }
    else
    {
        s_curve_peak = 0;
    }

    /* ---------- 3. 打角：PD（+ 可选积分） ---------- */
    /* Changing preview rows must not create a one-frame derivative impulse. */
    if(0 != s_preview_changed)
    {
        s_aim_last = aim;
        s_d_aim_filt = 0.0f;
        s_preview_changed = 0;
    }
    s_d_aim_filt += (float)CAR_STEER_D_FILTER * ((float)(aim - s_aim_last) - s_d_aim_filt);   /* 微分项一阶低通：压掉视觉噪声 */
    /* 【fw60】A. 双增益比例项：误差小的时候用"软"增益，误差大了线性过渡到"硬"增益。
     *   为什么：直道画龙的本质是"误差很小时增益还是那么硬" —— 视觉噪声几行就被放大成摆动；
     *   而弯道里误差大、需要硬增益才拐得动。参考 smart-car-origin 的 pid.c:25-36
     *   （他们是 8px 处直接折到 3.3 倍，我们改成线性过渡 —— 折点自己会变成极限环源）。
     *   软增益 s_steer_kp_soft（在线 9 / x 每次 -/+0.02）
     *   硬增益 s_steer_kp     （在线 ( / ) 每次 +/-0.05，就是原来那个 kp） */
    {
        float abs_aim = (aim >= 0) ? (float)aim : (float)(-(float)aim);
        float kp_now;

        if(abs_aim <= (float)CAR_STEER_KP_BLEND_PX)
        {
            kp_now = s_steer_kp_soft;
        }
        else if(abs_aim >= (float)(CAR_STEER_KP_BLEND_PX + CAR_STEER_KP_BLEND_SPAN))
        {
            kp_now = s_steer_kp;
        }
        else
        {
            kp_now = s_steer_kp_soft
                   + (s_steer_kp - s_steer_kp_soft)
                   * ((abs_aim - (float)CAR_STEER_KP_BLEND_PX) / (float)CAR_STEER_KP_BLEND_SPAN);
        }

        steer = kp_now * (float)aim + s_steer_kd * s_d_aim_filt;
    }

#if CAR_STEER_KI_ENABLE
    s_steer_i += (float)aim;
    if(s_steer_i >  200.0) { s_steer_i =  200.0; }
    if(s_steer_i < -200.0) { s_steer_i = -200.0; }
    steer += CAR_STEER_KI * s_steer_i;
#endif

    if(0 != s_steer_inv) { steer = -steer; }           /* n 命令：打角整体取反（方向反了不用重新烧录） */
    /* 【fw60】F. 输出死区：动态打角小到一定程度就直接归零 —— 视觉微抖不再让舵机来回蹭
     *   （参考 smart-car-origin pid.c:125-128 的 |out| < 5 归零）。
     *   注意：机械中位偏差走的是 CAR_SERVO_OFF_DEG 那一路，不经过这里，所以直行微调不受影响。 */
    if((steer < CAR_STEER_DEAD_DEG) && (steer > -CAR_STEER_DEAD_DEG)) { steer = 0.0f; }

    /* ---------- 4. 赛道状态：三行全部不可信 = 十字(短暂) 或 丢线(持续) ----------
     * 视觉组那份算法没有 track_type 这种标志（它总是靠种子值把线补出来），
     * 所以这里用"三行宽度都不合理"来判断：
     *   短时间这样 -> 十字（中线取画面中心 => aim 约等于 0，直行，同时压速）
     *   持续这样   -> 认为真看不见赛道（丢线）：保留一部分打角、爬行，久丢停车 */
#if(1 == CAR_SPEED_FIXED_ENABLE)
    /* 【fw56】固定速度：弯道 / 直道 / 十字 全都用这一个值（用户要求"同速度、尽量慢"）。
     *   不走"弯道减速 + 提前减速"那两条 —— 它们靠视觉的 curve / err_far 算出来，
     *   会让同一个弯的速度忽高忽低。想回到变速：把 car_config.h 的 CAR_SPEED_FIXED_ENABLE 改成 0。 */
    speed = s_speed_fixed;
#else
    speed = (int16)((float)s_speed_base - s_curve_k * (float)curve);   /* 【fw47e】弯道减速系数改成在线可调（5 / 6） */
    /* ---------- 4.7 【fw49】"看到弯道就开始减速"（提前量）----------
     *   原来只有 curve = |远端偏移 - 中间偏移| x 2 这一项：它要等弯道进到【中/远行之间】才涨起来，
     *   也就是车头快进弯了才开始减速 —— 实车感觉就是"看见弯了但来不及减速"。
     *   远端行(row 58)看的是最前面那一段：前面一有弯，【远端行自己的偏移】先变大（此时中间行还是 0），
     *   所以拿 |err_far| 当提前量：只要远端偏出死区就开始减速，弯越近偏得越多、减得越多。
     *   直道上 PID 把 aim 压在 0 附近，远端偏移只有几像素 -> 死区挡掉，不会平白无故慢下来。 */
    curve_early = err_far;
    if(curve_early < 0) { curve_early = (int16)(-curve_early); }
    if(curve_early > CAR_CURVE_EARLY_DEAD)
    {
        speed = (int16)((float)speed - s_curve_early_k * (float)(curve_early - CAR_CURVE_EARLY_DEAD));
    }

#endif   /* CAR_SPEED_FIXED_ENABLE */


    /* 新 P 点引导由 ring_guide_row_valid 提供瞄点；以下旧固定打角序列仅在关闭新补线时编译。
     * ---------- 3.6 【fw47】圆环行驶：先直行过"第一个路口"，到"第二个路口"再打角 ------------------
     *   视觉组的状态机（code\ring_road.c）只负责【识别】和【补线】，它把"看到开口"当成进环时机；
     *   实车要的是：入口那一侧有【两处张开】—— 第一个路口不进，走到第二个路口才打角。
     *   阶段（s_rp_phase）：
     *     1 直行：瞄点锁在"识别前那条直道"上，一路走过第一个路口
     *     2 进环：开口到车头（= 第二个路口）-> 固定打角打进去，持续 CAR_RING_ENTER_FRAMES 帧
     *     3 交还：CAR_RING_OUT_HOLD 帧内把打角还给跟线 -> 给完【彻底放手】
     *   ★出环【不归我们管】（用户 2026-09-25 明确要求）：进环打角交还完就直接放手，
     *     后面让车自己跟线继续跑 —— 不做"出环判定"，也不长期压着十字/丢线那套逻辑。
     *
     *   【fw47b】实车反馈"看到什么都觉得是圆环" —— 他们的判据是"一侧连续丢线 >= 25 行 且 对面直"，
     *   而弯道里外侧边线跑出画面恰好也是这个特征，所以误报到处都是。这里加两道闸：
     *     ① 可信度闸门（s_rp_gate_on，在线按 s 切 0/1/2）：
     *          0 = 关（报了就信）  1 = 直道接近 + 开口有界  2 = 再加"视觉组看到环内特征"（默认）
     *        过不了 = 当没看见：不锁瞄点、不打角；
     *     ② 直行阶段兜底：弯道程度连续超 CAR_RING_ABORT_CURVE 就判定是误判，立刻放开瞄点。
     *   ★车开过第二个路口时，视觉组自己的状态机就会判"出环"（ring_lost <= 12），ring_flag 随时可能
     *     变 0 —— 所以序列一旦开始就靠 s_rp_phase 自己撑住，不再依赖 ring_type。 */
    s_ring_active = 0;

#if(0 == RING_FIX_ENABLE)
    if((0 != s_ring_on) && ((RING_TYPE_NONE != ring_type) || (CAR_RP_IDLE != s_rp_phase)))
    {
        int8  rdir;
        int16 r_lost;
        int16 r_first;
        uint8 r_mask;

        if(CAR_RP_IDLE != s_rp_phase)
        {
            r_mask = s_rp_mask;                                 /* 序列已经在跑：用当初锁下来的那一侧 */
        }
        else if(s_rp_run_l > (s_rp_run_r + CAR_RING_SIDE_MARGIN))
        {
            r_mask = 0x01;                                      /* 【fw47c】左侧那段连续丢线更长 -> 左环 */
        }
        else if(s_rp_run_r > (s_rp_run_l + CAR_RING_SIDE_MARGIN))
        {
            r_mask = 0x02;                                      /* 右侧那段连续丢线更长 -> 右环 */
        }
        else
        {
            r_mask = (uint8)((RING_TYPE_RIGHT == ring_type) ? 0x02 : 0x01);   /* 两边差不多：还是信他们的标签 */
        }
        s_rp_oside = r_mask;
        /* 【fw47d】入口侧"标签纠错"：我们按原始丢线判出来的侧和视觉组的 ring_type 不一致时，
         *   直接把 ring_type 改成对的 —— 否则他们的 ring_fix_side 会往【错的那一侧】补线
         *   （实车现象：左环却往右打），他们的状态机也会盯着错的那一侧的丢线数去等。
         *   只在"特征非常明确"时才改：我们这一侧连续丢线 >= CAR_RING_LABEL_RUN_MIN(25) 行，
         *   另一侧 <= CAR_RING_LABEL_OPPO_RUN_MAX(12) 行（真环实测 43~56 对 4~9；
         *   十字那种两边都丢得很多的情况不满足，不会被误改）。闸门 0 档不改（= 完全不干预）。 */
        if((0 != s_ring_on) && (0 != s_rp_gate_on) && (CAR_RP_IDLE == s_rp_phase) && (RING_TYPE_NONE != ring_type))
        {
            uint8 v_side = (uint8)((RING_TYPE_RIGHT == ring_type) ? 0x02 : 0x01);

            if(v_side != r_mask)
            {
                int16 my_run = (int16)((0x01 == r_mask) ? s_rp_run_l : s_rp_run_r);
                int16 op_run = (int16)((0x01 == r_mask) ? s_rp_run_r : s_rp_run_l);

                if((my_run >= CAR_RING_LABEL_RUN_MIN) && (op_run <= CAR_RING_LABEL_OPPO_RUN_MAX))
                {
                    ring_type = (uint8)((0x01 == r_mask) ? RING_TYPE_LEFT : RING_TYPE_RIGHT);
                    s_rp_label_fix++;
                }
            }
        }
        if(0x02 == r_mask)                                  /* 入口侧 = 右边界 */
        {
            r_lost  = (int16)track_right_lost_count;
            r_first = s_rp_first_r;
        }
        else                                                /* 入口侧 = 左边界 */
        {
            r_lost  = (int16)track_left_lost_count;
            r_first = s_rp_first_l;
        }
        s_rp_lost  = r_lost;
        s_rp_first = r_first;

        if(CAR_RP_IDLE == s_rp_phase)                       /* 识别那一帧：先过可信度闸门 */
        {
            uint8 gate = 1;

            if(0 != s_rp_gate_on)
            {
                /* 【fw47e】弯道里报的圆环：只认"视觉组自己已经走到环内(INSIDE/OUT)"这一种强证据。
                 *   原来是"弯道里一律不信"，结果紧跟在弯道后面的真圆环也被否掉（实车反馈"有时候识别不上"）。 */
                if(s_curve_peak > CAR_RING_GATE_CURVE_MAX)
                {
                    if((RING_STATE_INSIDE != ring_state) && (RING_STATE_OUT != ring_state)) { gate = 0; }
                }
                if(r_first < CAR_RING_GATE_MIN_ABOVE)   { gate = 0; }   /* 开口远端看不到边线 = 不是"两处张开" */
                if(r_lost < CAR_RING_GATE_MIN_OPEN)     { gate = 0; }   /* 开口太短，不像环 */
            s_rp_near = (uint8)((0x02 == r_mask) ? s_rp_near_r : s_rp_near_l);   /* 上报用：入口侧近底部还有边线的行数 */

            /* 【fw60】③ 入口侧"整条边线几乎都没了"= 丢线/出画，不是环口。
             *   实车被这个骗过一次：rlost=108/120（整条左边全没了）闸门却放行。
             *   真环实测丢 42~90 行，所以上限 100（CAR_RING_GATE_LOST_MAX）留了余量。 */
            if(r_lost > CAR_RING_GATE_LOST_MAX)     { gate = 0; }

            /* 【fw60】④ 上方赛道要变宽：环口一定比正常赛道宽（参考 smart-car-origin
             *   ring.c:145-159，他们要求 25~65 行里任一行宽 > 110px；我们的量在 ? 里显示 wide=）。
             *   缺这个证据时只认"视觉组自己已经走到环内(INSIDE/OUT)"这一种强证据 ——
             *   和上面弯道那条同一个思路，避免小 S 弯 / 急弯被当成环。 */
            if(s_rp_wide < CAR_RING_GATE_WIDE_PX)
            {
                if((RING_STATE_INSIDE != ring_state) && (RING_STATE_OUT != ring_state)) { gate = 0; }
            }
            }

            if(0 != gate)
            {
                s_rp_phase    = CAR_RP_STRAIGHT;
                s_rp_frames   = 0;
                s_rp_total    = 0;
                s_rp_seen     = 0;
                s_rp_ok       = 0;
                s_rp_vconf    = 0;
                s_rp_abort    = 0;
                s_ring_hold   = 0;
                s_rp_mask     = r_mask;
                s_rp_aim_hold = (int16)s_rp_aim_pre;            /* 识别前那个干净的瞄点 = 直行基准 */
                s_rp_gate_ok  = 1;
            }
            else
            {
                s_rp_gate_ok = 0;
                s_rp_gate_fail++;                               /* 诊断：闸门挡掉一次 */

                /* 【fw47e】闸门没过 = 这个"圆环"不可信：我们的瞄点也锁成"识别前的直行基准"。
                 *   为什么必须锁：视觉组的补线是按它自己的判断推中线的（最多 30 个像素），
                 *   不锁的话车就跟着那条被推歪的中线走 —— 实车现象就是"直道画龙"。
                 *   只在确实在直道上时才锁（弯道里锁瞄点 = 不让车转弯，那更危险）。 */
                if(s_curve_peak <= CAR_RING_GATE_CURVE_MAX)
                {
                    s_rp_aim_lock = 1;
                    s_rp_aim_hold = (int16)s_rp_aim_pre;
                }
            }
        }

        if(CAR_RP_IDLE != s_rp_phase)                       /* 过了闸门（或序列已经在跑）才继续 */
        {
            rdir = (int8)(((0x02 == s_rp_mask) ? 1 : -1) * s_ring_dir_sign);   /* 环在右边就往右打 */

            s_rp_frames++;
            s_rp_total++;

            /* 视觉组自己有没有看到"环内特征"（全白行 / 黑白跳变行）—— 严格档进环的必要条件 */
            /* 【fw47e】"环内特征"的判据从严：
             *   ① 视觉组自己的状态机走到 INSIDE(3)/OUT(4)（= 他们的 FIND->IN->INSIDE 一路确认过）；或者
             *   ② 全白行【和】黑白跳变行同时找到（他们方法二的两条判据都成立）。
             *   原来只要有全白行就算数 —— 直道上地板上有一块白斑也能凑出全白行，就会误进环（画龙）。
             *   嫌太严、真环进不去：按 s 把闸门降到 1 档（1 档不要这条）。 */
            if((RING_STATE_INSIDE == ring_state) || (RING_STATE_OUT == ring_state)
               || ((0 != ring_white_row) && (0 != ring_trans_row)))
            {
                s_rp_vconf = 1;
            }

            if(CAR_RP_STRAIGHT == s_rp_phase)
            {
                /* 【阶段 1】直行：瞄点已经在上面 1 段里换成直行基准了，这里只管等开口过去 */
                s_ring_active = 1;
                if(speed > s_ring_speed) { speed = s_ring_speed; }

                if(r_lost >= CAR_RING_OPEN_LOST_MIN) { s_rp_seen = 1; }

                if((0 != s_rp_seen) && ((r_first >= s_rp_enter_row) || (r_lost <= CAR_RING_OPEN_GONE_LOST)))
                {
                    s_rp_ok++;                                  /* 开口最靠前那行已经贴到车头 / 开口已经过去 */
                }
                else
                {
                    s_rp_ok = 0;
                }

                if(curve > CAR_RING_ABORT_CURVE) { s_rp_abort++; } else { s_rp_abort = 0; }

                if(s_rp_abort >= CAR_RING_ABORT_N)
                {
                    s_rp_phase  = CAR_RP_IDLE;                  /* 误判兜底：直行阶段却一直是大弯道 -> 放开瞄点 */
                    s_ring_hold = 0;
                }
                else if((0 != s_rp_seen) && (s_rp_ok >= CAR_RING_ENTER_CONFIRM)
                        && ((s_rp_gate_on < CAR_RING_GATE_LEVEL_STRICT) || (0 != s_rp_vconf)))
                {
                    s_rp_phase  = CAR_RP_ENTER;                 /* 第二个路口到了：打角进环 */
                    s_rp_frames = 0;
                    s_ring_hold = CAR_RING_OUT_HOLD;
                }
                else if(s_rp_frames > (uint16)CAR_RING_STRAIGHT_MAX)
                {
                    s_rp_phase  = CAR_RP_IDLE;                  /* 一直等不到就放弃：继续直行，不乱打角 */
                    s_ring_hold = 0;
                }
            }
            else if(CAR_RP_ENTER == s_rp_phase)
            {
                /* 【阶段 2】进环：固定打角打进去（角度 m/M、帧数 j/J 都在线可调） */
                steer = (float)rdir * s_ring_steer;
                speed = s_ring_speed;
                s_ring_active = 1;

                if(s_rp_frames >= (uint16)s_rp_enter_frm)
                {
                    s_rp_phase  = CAR_RP_HANDOFF;               /* 打角打完 -> 交还给跟线 */
                    s_rp_frames = 0;
                    s_ring_hold = CAR_RING_OUT_HOLD;
                }
            }
            else
            {
                /* 【阶段 3】交还：把进环打角还给跟线；给完就放手（出环不归我们管，车自己继续跑） */
                s_ring_active = 1;
                if(speed > s_ring_speed) { speed = s_ring_speed; }

                if(0 != s_ring_hold)
                {
                    float t = (float)s_ring_hold / (float)CAR_RING_OUT_HOLD;   /* 1 -> 0 */

                    steer = (float)rdir * s_ring_steer * t + steer * (1.0f - t);
                    s_ring_hold--;
                }
                else
                {
                    s_rp_phase = CAR_RP_IDLE;                   /* 交还完成：放手，后面让车自己跟线跑 */
                }
            }
        }
    }
    else
    {
        s_ring_hold  = 0;                                   /* 没识别到圆环（或 y 关掉了策略） */
        s_rp_phase   = CAR_RP_IDLE;                         /* ★序列必须一起清：否则直行阶段的"锁瞄点"会一直留着 */
        s_rp_seen    = 0;
        s_rp_ok      = 0;
        s_rp_abort   = 0;
        s_rp_vconf   = 0;
        s_rp_lost    = 0;
        s_rp_first   = -1;
        s_rp_aim_lock = 0;                                  /* 【fw47e】没有圆环了：把"锁直行"放开 */
    }
#endif

    if((0 == s_ring_active) && ((0 == valid_rows) ||
       ((3 == lost_rows) && !ring_guide_row_valid(CAR_ROW_NEAR) &&
        !ring_guide_row_valid(s_row_mid) && !ring_guide_row_valid(s_row_far))))     /* 【fw45】环内不判十字/丢线（环里丢线多是正常的）*/
    {
        s_open_frames++;

        if(s_open_frames > CAR_OPEN_AS_LOST_FRAMES)
        {
            s_lost_frames++;
            steer = s_steer_last * CAR_LOST_STEER_HOLD; /* 别立刻回正，否则弯道丢线就冲出去 */
#if(0 == CAR_SPEED_FIXED_ENABLE)
            speed = CAR_SPEED_LOST_CM_S;
#else
            speed = s_speed_fixed;                  /* 【fw56】固定速度模式下爬行也用同一个值（原来的 50 反而更快） */
#endif
            if(s_lost_frames > CAR_LOST_STOP_FRAMES)
            {
                speed = 0;                              /* 丢线太久：认输，停车 */
            }
        }
        else
        {
            s_lost_frames = 0;
#if(0 == CAR_SPEED_FIXED_ENABLE)
            if(speed > CAR_SPEED_CROSS_CM_S) { speed = CAR_SPEED_CROSS_CM_S; }
#endif
        }
    }
    else
    {
        s_open_frames = 0;
        s_lost_frames = 0;
    }

    /* ---------- 5. 打角限幅并下发舵机 ---------- */
    if(steer > s_steer_max)  { steer = s_steer_max;  }
    if(steer < -s_steer_max) { steer = -s_steer_max; }
    car_servo_apply(steer);

    /* ---------- 6. 速度限幅 + 差速 + 下发电机 ---------- */
    if(speed > 0)
    {
        /* 【fw56】起步助推：固定速度定得低的时候（台架记录：静止起步 <18% 占空比带不动），
         *   从静止起步的头 CAR_START_BOOST_FRAMES 帧把速度顶上去 CAR_START_BOOST_CM_S，
         *   之后自动回到固定速度 —— 只在"刚起步"那一下，正常行驶不受影响。 */
        speed = car_drive_start_boost(speed,
                (uint8)((0 == motor_output_off_get()) && (0 == motor_is_fault())),
                &s_run_frames);

        if((0 != ring_guide_row_valid(s_row_mid)) && (speed > s_ring_speed))
        {
            speed = s_ring_speed;
        }

        if(speed > CAR_SPEED_MAX_CM_S) { speed = CAR_SPEED_MAX_CM_S; }
#if(0 == CAR_SPEED_FIXED_ENABLE)
        if((0 == s_ring_active) && (speed < CAR_SPEED_MIN_CM_S)) { speed = CAR_SPEED_MIN_CM_S; }   /* 【fw45】环内允许低于最低速 */
#endif
    }

    if(speed <= 0) { s_run_frames = 0; }
    /* Apply after startup boost so the cap cannot be bypassed by that boost. */
    speed = car_drive_corner_limit(speed, curve, steer, s_turn_limit);
    s_pwm_steer = servo_get_steer();
    car_drive_mix(speed, (0 != s_drive_normalized) ? s_pwm_steer : steer,
                  s_diff_k, s_drive_normalized, &s_drive_output);
    motor_set_target(s_drive_output.left, s_drive_output.right);

    /* ---------- 7. 记录状态 ---------- */
    s_steer_last = steer;
    s_aim_last   = aim;
    s_speed_last = speed;

    

}
