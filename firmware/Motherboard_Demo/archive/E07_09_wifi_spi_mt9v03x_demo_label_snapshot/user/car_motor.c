/*********************************************************************************************************************
* 后轮双电机驱动 + 速度闭环（DRV8701E：每路 1 根 DIR + 1 根 PWM）
*
* 本模块负责：
*   1. 把"带符号的占空比"翻译成 DRV8701E 的 DIR + PWM（正负号 = 正反转）
*   2. 10ms 周期中断里跑速度环（编码器 PI + 前馈），电机 PWM 只在这一个地方被改写
*   3. 输出斜坡限制：任何目标都不会瞬间给到电机，保护齿轮箱，车也不会一窜一窜
*   4. 视觉心跳看门狗：图像超过 CAR_FRAME_TIMEOUT_MS 没更新就立刻停车并锁死
*
* 单位约定：
*   对外接口 motor_set_target() 的目标速度单位是 cm/s（厘米每秒）
*   内部速度环的目标是"每 10ms 周期的编码器计数"，两者用 CAR_ENC_COUNTS_PER_METER 换算
*   占空比一律用"万分比"：10000 = 100%，上限由 CAR_MOTOR_DUTY_MAX 卡住
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入
********************************************************************************************************************/

#include "car_config.h"
#include "car_motor.h"

/* 编码器计数 <-> 速度(cm/s) 换算
 * 计数是"每个速度环周期(10ms)"的计数，所以 1 秒的计数 = cnt * (1000 / CAR_SPEED_PERIOD_MS)
 * cm/s = cnt * (1000/周期ms) * 100 / 每米计数 = cnt * (100000/周期ms) / 每米计数
 * 默认 10ms 周期时就是 cnt * 10000 / 每米计数 */
#define CAR_ENC_TO_CM_S(cnt)    ((int16)(((int32)(cnt) * (100000L / CAR_SPEED_PERIOD_MS)) / CAR_ENC_COUNTS_PER_METER))
#define CAR_CM_S_TO_ENC(v)      ((int16)(((int32)(v) * CAR_ENC_COUNTS_PER_METER) / (100000L / CAR_SPEED_PERIOD_MS)))

/* 视觉心跳：每 CAR_SPEED_PERIOD_MS 减 1，减到 0 说明图像停了 */
#define CAR_HEARTBEAT_TICKS     ((uint16)(CAR_FRAME_TIMEOUT_MS / CAR_SPEED_PERIOD_MS))

/* 速度环内部状态（中断与主循环共享，加 volatile） */
static volatile int16 motor_target_l = 0;       /* 目标速度(cm/s) */
static volatile int16 motor_target_r = 0;
static volatile int16 motor_duty_l   = 0;       /* 当前实际输出的占空比(万分比, 带符号) */
static volatile int16 motor_duty_r   = 0;
static volatile int16 motor_speed_l  = 0;       /* 实测速度(cm/s)，开环模式下恒为 0 */
static volatile int16 motor_speed_r  = 0;
static volatile int16 motor_enc_l    = 0;       /* 最近一个周期的原始计数 */
static volatile int16 motor_enc_r    = 0;
static volatile uint8 motor_enable   = 0;       /* 1 = 允许电机输出 */
static volatile uint8 motor_fault    = 0;       /* 0 正常 / 1 视觉超时 / 2 编码器故障，锁死后要复位 */
static volatile uint16 motor_beat    = 0;       /* 心跳计数 */
static volatile uint8 motor_no_enc_check = 0;   /* 1 = 临时关掉编码器保护（台架自检用） */
static volatile uint16 motor_tick_cnt = 0;      /* 10ms 计数（速度环每进一次加 1），主循环定时用 */
static volatile int16 motor_enc_raw_l = 0;      /* 没取反的原始计数（调试对照用） */
static volatile int16 motor_enc_raw_r = 0;
static volatile uint8 motor_enc_inv_l = CAR_ENC_L_INVERT;   /* 运行时可变，初值取配置里的 CAR_ENC_x_INVERT */
static volatile uint8 motor_enc_inv_r = CAR_ENC_R_INVERT;
static volatile uint8 motor_enc_mode_now = CAR_ENCODER_MODE;    /* 运行时可变：0 = 正交(A/B)  1 = 带方向(PULSE+DIR) */
/* 编码器接口与引脚（运行时可变，方便试主板上的 4 个编码器插座，见 motor_enc_socket_set） */
static volatile uint8  motor_enc_socket  = 0;
static volatile uint16 enc_idx_l   = CAR_ENC_L_INDEX;
static volatile uint16 enc_pulse_l = CAR_ENC_L_PULSE;
static volatile uint16 enc_dir_l   = CAR_ENC_L_DIR;
static volatile uint16 enc_idx_r   = CAR_ENC_R_INDEX;
static volatile uint16 enc_pulse_r = CAR_ENC_R_PULSE;
static volatile uint16 enc_dir_r   = CAR_ENC_R_DIR;
static volatile uint8 motor_out_off = 0;        /* 1 = 硬关输出：占空比直接 0，PI 不参与（台架调编码器用） */
static volatile uint8 motor_test_duty_on = 0;   /* 1 = 自检固定占空比模式（完全不看编码器） */
static volatile int16 motor_test_duty_l  = 0;   /* 自检固定占空比(万分比, 带符号) */
static volatile int16 motor_test_duty_r  = 0;

static uint16 motor_enc_fail_l = 0;             /* "给着占空比却读不到计数"的连续周期数 */
static uint16 motor_enc_fail_r = 0;

static int32 motor_i_l = 0;                     /* 速度环积分累加 */
static int32 motor_i_r = 0;

#if CAR_ENABLE_ENCODER                          /* 编码器不参与控制（开环）时这两个函数用不到，不编译进来 */
/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     读一路 PWM 编码器的计数（**故意不用库里的 encoder_get_count**，原因见备注）
 * 参数说明     idx       编码器索引：PWMA~PWMF = 0~5 走本函数；TIM 索引交给库里
 *               dir_io    带方向模式时的方向引脚（IO_ 编号，取低 8 位）
 *               quad      1 = 正交(A/B) 接口   0 = 带方向(PULSE + DIR) 接口
 * 返回参数     本周期计数（带符号）
 * 备注信息     ★ 为什么要自己读 ★
 *   库里用一张 static 的 encoder_dir_pin[] 表判断"这一路是正交还是带方向"：
 *     表里是 0xFF -> 当正交处理，多做一次 ret >> 2（4 分频）
 *     表里是引脚 -> 当带方向处理，用那个引脚的**瞬时电平**翻正负
 *   而 encoder_quad_init() **不会**把这张表清回 0xFF。于是"先按带方向初始化、再切成正交"时，
 *   表里还留着上一次写进去的引脚，库里就把正交计数当成带方向处理，结果：
 *     (1) 符号被 B 相（A/B 编码器的 B 就是 90 度方波）的瞬时电平随机翻转
 *         -> 读数忽正忽负、2 秒净和约等于 0、abs 很大、flips 一堆
 *     (2) 少做一次 >>2 -> 数值偏大 4 倍
 *   这就是"换成 mode=0 正交以后 abs 很大、net 约等于 0"的真正原因：不是编码器坏，也不是接线错。
 *   本函数直接读计数器寄存器，按**我们自己的模式变量**处理，和初始化顺序、切换历史完全无关。
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 motor_enc_read_pwm(uint16 idx, uint16 dir_io, uint8 quad)
{
    vuint8 far *p;
    uint8 h1;
    uint8 l;
    uint8 h2;
    int16 v;

    p = (vuint8 far *)(PWMX_CNTR_ADDR[idx]);        /* [idx] = 高 8 位地址，+1 = 低 8 位 */

    /* 高/低字节连着读两遍核对，避免正好碰上进位/借位时读到中间值 */
    do
    {
        h1 = *p;
        l  = *(p + 1);
        h2 = *p;
    } while(h1 != h2);

    v = (int16)((((uint16)h1) << 8) | (uint16)l);

    if(0 == quad)
    {
        /* 带方向接口：计数器只数 PULSE 的上升沿，方向由 DIR 引脚电平给出 */
        if(0 == gpio_get_level((gpio_pin_enum)dir_io))
        {
            v = (int16)(-v);
        }
    }
    /* 正交接口：硬件是 4 倍频（A/B 的上升沿和下降沿都数），这里**故意不做 >>2**，保留 4 倍分辨率
     *（低速时量化误差更小）。所以"正交模式的计数 = 脉冲数的 4 倍"，标定 CAR_ENC_COUNTS_PER_METER
     * 时会自动吸收这个系数；用 a / d 换过模式之后，记得重新标定一次。 */

    return v;
}

/* 读一路编码器的计数：PWM 口自己读，TIM 口交给库里（TIM 那张表是 init 时刚写进去的，没有上面的历史问题） */
static int16 motor_enc_read_one(uint16 idx, uint16 dir_io, uint8 quad)
{
    if(idx >= TIM0_ENCODER)
    {
        return encoder_get_count((encoder_index_enum)idx);
    }
    return motor_enc_read_pwm(idx, dir_io, quad);
}
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     占空比限幅 + 起动死区补偿
 * 参数说明     duty        带符号占空比(万分比)
 * 返回参数     处理后的占空比
 * 备注信息     CAR_MOTOR_DEADZONE 用来对付静摩擦：目标很小的时候电机根本不转，
 *              干脆抬到能转起来的最小值。填 0 表示不做补偿
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 motor_limit_duty(int16 duty)
{
    if(duty > CAR_MOTOR_DUTY_MAX)  { duty = CAR_MOTOR_DUTY_MAX; }
    if(duty < -CAR_MOTOR_DUTY_MAX) { duty = -CAR_MOTOR_DUTY_MAX; }

    if(0 != CAR_MOTOR_DEADZONE)
    {
        if((duty > 0) && (duty < CAR_MOTOR_DEADZONE))  { duty = CAR_MOTOR_DEADZONE; }
        if((duty < 0) && (duty > -CAR_MOTOR_DEADZONE)) { duty = -CAR_MOTOR_DEADZONE; }
    }

    return duty;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     输出斜坡：每次最多变化 CAR_MOTOR_RAMP_STEP
 * 参数说明     now         当前值
 * 参数说明     target      目标值
 * 返回参数     本周期应该输出的值
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 motor_ramp(int16 now, int16 target)
{
    int16 diff = (int16)(target - now);

    if(diff > CAR_MOTOR_RAMP_STEP)  { return (int16)(now + CAR_MOTOR_RAMP_STEP); }
    if(diff < -CAR_MOTOR_RAMP_STEP) { return (int16)(now - CAR_MOTOR_RAMP_STEP); }

    return target;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     把带符号占空比写到 DRV8701E（DIR 决定方向，PWM 决定快慢）
 * 参数说明     left        左电机占空比(万分比, 带符号)
 * 参数说明     right       右电机占空比(万分比, 带符号)
 * 返回参数     void
 * 备注信息     DIR 高 = 正转（与 E02_02 例程一致）
 *              同一时间只输出一个方向的 PWM，另一路是 DIR 电平，不存在 H 桥直通的问题
 *-----------------------------------------------------------------------------------------------------------------*/
static void motor_write(int16 left, int16 right)
{
    uint32 duty;
    int16  out;

    /* ---------------- 左电机 ---------------- */
    out = left;
    if(0 != CAR_MOTOR_L_INVERT) { out = (int16)(-out); }

    if(out >= 0)
    {
        gpio_set_level(CAR_MOTOR_L_DIR, GPIO_HIGH);
        duty = (uint32)out;
    }
    else
    {
        gpio_set_level(CAR_MOTOR_L_DIR, GPIO_LOW);
        duty = (uint32)(-out);
    }
    pwm_set_duty(CAR_MOTOR_L_PWM, duty);

    /* ---------------- 右电机 ---------------- */
    out = right;
    if(0 != CAR_MOTOR_R_INVERT) { out = (int16)(-out); }

    if(out >= 0)
    {
        gpio_set_level(CAR_MOTOR_R_DIR, GPIO_HIGH);
        duty = (uint32)out;
    }
    else
    {
        gpio_set_level(CAR_MOTOR_R_DIR, GPIO_LOW);
        duty = (uint32)(-out);
    }
    pwm_set_duty(CAR_MOTOR_R_PWM, duty);
}

#if CAR_ENABLE_ENCODER                          /* 开环(CAR_ENABLE_ENCODER=0)时速度环用不到，不编译进来，避免"未引用"警告 */
/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     速度环：前馈 + PI
 * 参数说明     target      目标计数(每周期)
 * 参数说明     actual      实际计数(每周期)
 * 参数说明     i_acc       积分累加器
 * 返回参数     占空比(万分比, 带符号)
 * 备注信息     前馈让电机先动起来（输出大致等于目标所需），PI 只负责修剩下的误差，
 *              这样比纯 PI 好调很多。Kp、Ki 设 0 就退化成纯前馈开环
 *-----------------------------------------------------------------------------------------------------------------*/
static int16 motor_speed_pi(int16 target, int16 actual, int32 *i_acc)
{
    int16 err = (int16)(target - actual);
    int32 out;

    *i_acc += (int32)err * CAR_SPEED_KI_NUM;
    if(*i_acc > CAR_SPEED_I_MAX)  { *i_acc = CAR_SPEED_I_MAX; }
    if(*i_acc < -CAR_SPEED_I_MAX) { *i_acc = -CAR_SPEED_I_MAX; }

    out = (int32)target * CAR_SPEED_FF_NUM
        + (int32)err * CAR_SPEED_KP_NUM
        + *i_acc;

    return motor_limit_duty((int16)out);
}
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     速度环周期中断（10ms）—— 电机 PWM 唯一被改写的地方
 * 参数说明     void
 * 返回参数     void
 * 备注信息     这里不要放 printf 之类的耗时操作，中断要尽快返回
 *-----------------------------------------------------------------------------------------------------------------*/
static void motor_speed_pit(void)
{
    int16 cnt_l = 0;
    int16 cnt_r = 0;
    int16 tgt_l;
    int16 tgt_r;
    int16 duty_l;
    int16 duty_r;

    motor_tick_cnt++;                           /* 10ms 计数（无线串口定时用） */

    /* ---------- 1. 读编码器（读一次清零一次，10ms 内不会溢出 16 位计数器） ---------- */
#if CAR_ENABLE_ENCODER
    cnt_l = motor_enc_read_one(enc_idx_l, (uint16)(enc_dir_l & 0xFF), (uint8)((0 == motor_enc_mode_now) ? 1 : 0));
    cnt_r = motor_enc_read_one(enc_idx_r, (uint16)(enc_dir_r & 0xFF), (uint8)((0 == motor_enc_mode_now) ? 1 : 0));
    encoder_clear_count((encoder_index_enum)enc_idx_l);     /* 清零仍用库函数：它只写计数器，和那张方向表无关 */
    encoder_clear_count((encoder_index_enum)enc_idx_r);

    motor_enc_raw_l = cnt_l;                    /* 先记下原始值：调试时能看出"取反到底有没有生效" */
    motor_enc_raw_r = cnt_r;

    if(0 != motor_enc_inv_l) { cnt_l = (int16)(-cnt_l); }   /* 方向标志运行时可改（默认 = CAR_ENC_x_INVERT） */
    if(0 != motor_enc_inv_r) { cnt_r = (int16)(-cnt_r); }
#endif
    motor_enc_l = cnt_l;
    motor_enc_r = cnt_r;

    /* ---------- 2. 视觉心跳看门狗 ---------- */
    if(motor_beat > 0)
    {
        motor_beat--;
    }
    else if((0 != motor_enable) && (0 == motor_fault))
    {
        motor_fault = CAR_MOTOR_FAULT_VISION;   /* 图像停了：锁死停车，必须复位才能再跑 */
    }

    /* ---------- 2b. 编码器 / 堵转保护 ----------
     * 编码器没接好、引脚填错、模式选错时，速度环永远读到 0，会一路把占空比加到上限，
     * 车会直接窜出去。这里判定：给着 > CAR_ENC_FAIL_DUTY 的占空比，
     * 却连续 CAR_ENC_FAIL_TICKS 个周期一个计数都没有 => 锁死停车。 */
#if CAR_ENABLE_ENCODER && CAR_ENC_FAIL_ENABLE
    if((0 != motor_enable) && (0 == motor_fault) && (0 == motor_no_enc_check))
    {
        if((motor_duty_l > CAR_ENC_FAIL_DUTY) && (0 == cnt_l)) { motor_enc_fail_l++; }
        else                                                   { motor_enc_fail_l = 0; }

        if((motor_duty_r > CAR_ENC_FAIL_DUTY) && (0 == cnt_r)) { motor_enc_fail_r++; }
        else                                                   { motor_enc_fail_r = 0; }

        if((motor_enc_fail_l > CAR_ENC_FAIL_TICKS) || (motor_enc_fail_r > CAR_ENC_FAIL_TICKS))
        {
            motor_fault = CAR_MOTOR_FAULT_ENCODER;
        }
    }
#endif

    /* ---------- 3. 决定本周期的目标速度 ---------- */
    if((0 != motor_enable) && (0 == motor_fault))
    {
        tgt_l = motor_target_l;
        tgt_r = motor_target_r;
    }
    else
    {
        tgt_l = 0;
        tgt_r = 0;
    }

    /* ---------- 4. 硬关输出 / 自检固定占空比 / 闭环 / 开环 ---------- */
    if((0 != motor_out_off) || (0 != motor_fault))
    {
        /* 硬关：占空比直接 0，PI 不参与。
         * 为什么需要它：原来"目标速度 = 0"时 PI 还是会反向发力去"顶住"轮子
         * （你手推轮子时电机就会跟你抢）；故障锁死时也不该再驱动电机 */
        duty_l = 0;
        duty_r = 0;
        motor_i_l = 0;
        motor_i_r = 0;
        motor_enc_fail_l = 0;
        motor_enc_fail_r = 0;
    }
    else if((0 != motor_test_duty_on) && (0 != motor_enable))
    {
        /* 台架自检：直接给固定占空比，完全不看编码器、不跑速度环。
         * 原因：自检阶段编码器接线还没验证，闭环会因为编码器读数乱而把电机反复正反转，
         *       那样就分不清"线接反了"还是"闭环在跟乱读数打架"。要看方向必须纯开环。 */
        duty_l = motor_test_duty_l;
        duty_r = motor_test_duty_r;
        motor_i_l = 0;
        motor_i_r = 0;
        motor_enc_fail_l = 0;
        motor_enc_fail_r = 0;
        motor_speed_l = CAR_ENC_TO_CM_S(cnt_l);
        motor_speed_r = CAR_ENC_TO_CM_S(cnt_r);
    }
    else
    {
    /* ---------- 4b. 正常闭环 / 开环 ---------- */
#if CAR_ENABLE_ENCODER
    duty_l = motor_speed_pi(CAR_CM_S_TO_ENC(tgt_l), cnt_l, &motor_i_l);
    duty_r = motor_speed_pi(CAR_CM_S_TO_ENC(tgt_r), cnt_r, &motor_i_r);
    motor_speed_l = CAR_ENC_TO_CM_S(cnt_l);
    motor_speed_r = CAR_ENC_TO_CM_S(cnt_r);
#else
    duty_l = (int16)((int32)tgt_l * CAR_OPENLOOP_DUTY_PER_CM_S);
    duty_r = (int16)((int32)tgt_r * CAR_OPENLOOP_DUTY_PER_CM_S);
    duty_l = motor_limit_duty(duty_l);
    duty_r = motor_limit_duty(duty_r);
    motor_i_l = 0;                              /* 开环模式下把积分清掉，切回闭环时不会突然一窜 */
    motor_i_r = 0;
#endif
    }

    /* ---------- 5. 斜坡 + 输出 ---------- */
    motor_duty_l = motor_ramp(motor_duty_l, duty_l);
    motor_duty_r = motor_ramp(motor_duty_r, duty_r);

#if CAR_ENABLE_MOTOR
    motor_write(motor_duty_l, motor_duty_r);
#else
    motor_duty_l = 0;                           /* CAR_ENABLE_MOTOR = 0：只算不输出，方便单独调舵机 */
    motor_duty_r = 0;
    motor_write(0, 0);
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     按"当前模式"初始化两路编码器（带方向 / 正交）
 * 备注信息     单独拎出来是为了能在运行时切换模式（用无线串口发个字符就行，不用重新编译烧录）。
 *              编码器类型和 CAR_ENCODER_MODE 不匹配时，读数会忽正忽负、像噪声 —— 这是最常见的坑
 *              ★ 还有库里那个坑：encoder_get_count() 靠一张 static 表判断"正交还是带方向"，
 *                而 encoder_quad_init() 不会把表清回去，所以"先 dir 再 quad"会读到乱符号。
 *                本文件的 motor_enc_read_one() 自己读寄存器，就是为了绕开它，a/d 随便切都没事。
 *-----------------------------------------------------------------------------------------------------------------*/
#if CAR_ENABLE_ENCODER
static void motor_enc_hw_init(void)
{
    if(0 == motor_enc_mode_now)
    {
        encoder_quad_init((encoder_index_enum)enc_idx_l, (encoder_channel_enum)enc_pulse_l, (encoder_channel_enum)enc_dir_l);   /* 正交 A/B */
        encoder_quad_init((encoder_index_enum)enc_idx_r, (encoder_channel_enum)enc_pulse_r, (encoder_channel_enum)enc_dir_r);
    }
    else
    {
        encoder_dir_init(enc_idx_l, enc_pulse_l, enc_dir_l);                    /* 带方向 PULSE + DIR */
        encoder_dir_init(enc_idx_r, enc_pulse_r, enc_dir_r);
    }
    encoder_clear_count((encoder_index_enum)enc_idx_l);
    encoder_clear_count((encoder_index_enum)enc_idx_r);
}
#endif

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     电机与速度环初始化
 * 参数说明     void
 * 返回参数     void
 * 备注信息     DIR 先拉到"正转"再开 PWM 且占空比为 0，所以上电瞬间电机不会动
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_init(void)
{
    /* DIR 引脚：GPIO 推挽输出 */
    gpio_init(CAR_MOTOR_L_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(CAR_MOTOR_R_DIR, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    /* PWM：17kHz，占空比 0 */
    pwm_init(CAR_MOTOR_L_PWM, CAR_MOTOR_PWM_FREQ, 0);
    pwm_init(CAR_MOTOR_R_PWM, CAR_MOTOR_PWM_FREQ, 0);

    motor_target_l = 0;
    motor_target_r = 0;
    motor_duty_l   = 0;
    motor_duty_r   = 0;
    motor_speed_l  = 0;
    motor_speed_r  = 0;
    motor_i_l      = 0;
    motor_i_r      = 0;
    motor_enable   = 0;
    motor_fault    = 0;
    motor_beat     = 0;
    motor_enc_fail_l = 0;
    motor_enc_fail_r = 0;

    motor_write(0, 0);                          /* 确认静止 */

    /* 编码器：按"当前模式"初始化（模式可在运行时切换，见 motor_enc_mode_set） */
#if CAR_ENABLE_ENCODER
    motor_enc_hw_init();
#endif

    /* 速度环周期中断：速度环与斜坡都在这里面跑 */
    pit_ms_init(CAR_SPEED_PIT, CAR_SPEED_PERIOD_MS, motor_speed_pit);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     设置左右轮目标速度（cm/s）
 * 参数说明     left_cm_s   左轮目标速度，正值 = 前进
 * 参数说明     right_cm_s  右轮目标速度
 * 返回参数     void
 * 备注信息     本函数只是把目标记下来，真正的输出在 10ms 中断里执行；
 *              电机没使能（还没起步 / 已经锁死）时目标会被忽略
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_set_target(int16 left_cm_s, int16 right_cm_s)
{
    if(left_cm_s  < 0) { left_cm_s  = 0; }
    if(right_cm_s < 0) { right_cm_s = 0; }

    motor_target_l = left_cm_s;
    motor_target_r = right_cm_s;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     允许 / 禁止电机输出（起步门、停车都用它）
 * 参数说明     enable      1 = 允许跑   0 = 目标强制清零（靠斜坡慢慢停）
 * 返回参数     void
 * 备注信息     enable = 1 会同时清掉锁死标志、清积分、把心跳喂满
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_run(uint8 enable)
{
    if(0 != enable)
    {
        motor_i_l    = 0;
        motor_i_r    = 0;
        motor_fault  = 0;
        motor_beat   = CAR_HEARTBEAT_TICKS;
        motor_enc_fail_l = 0;
        motor_enc_fail_r = 0;
        motor_enable = 1;
    }
    else
    {
        motor_enable  = 0;
        motor_target_l = 0;
        motor_target_r = 0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     急停：立刻把占空比打到 0，不走斜坡
 * 参数说明     void
 * 返回参数     void
 * 备注信息     正常情况下用 motor_run(0) 让它平滑停；这里用于故障急停
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_stop(void)
{
    motor_enable   = 0;
    motor_target_l = 0;
    motor_target_r = 0;
    motor_duty_l   = 0;
    motor_duty_r   = 0;
    motor_write(0, 0);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     喂心跳：每处理完一帧图像调用一次
 * 参数说明     void
 * 返回参数     void
 * 备注信息     图像长时间不来 => 速度环判定视觉卡死 => 停车锁死
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_heartbeat(void)
{
    motor_beat = CAR_HEARTBEAT_TICKS;
}
/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     发图（阻塞操作）之前给看门狗"续一条长命"（fw42）
 * 备注信息     库里的 wifi_spi_wait_idle 一次最多等 1 秒，整张图最坏阻塞 ~1.2 秒；
 *              而心跳平时每帧才喂一次（上限 CAR_FRAME_TIMEOUT_MS = 500ms），
 *              链路抖一下就会被判成"视觉停了"把车锁死。这里一次性续到 3 倍覆盖最坏情况；
 *              发图返回后主循环照常每帧喂，所以"视觉真的死了"仍然会照常锁死。
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_heartbeat_link_guard (void)
{
    motor_beat = (uint16)(CAR_HEARTBEAT_TICKS * 3);
}


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     查询是否已经锁死
 * 参数说明     void
 * 返回参数     0 = 正常，非 0 = 已锁死
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 motor_is_fault(void)
{
    return motor_fault;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     查询故障码
 * 参数说明     void
 * 返回参数     0 正常 / CAR_MOTOR_FAULT_VISION 视觉超时 / CAR_MOTOR_FAULT_ENCODER 编码器故障
 *-----------------------------------------------------------------------------------------------------------------*/
uint8 motor_fault_reason(void)
{
    return motor_fault;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     台架自检开关：临时关掉"编码器读不到就锁死"的保护
 * 参数说明     on          1 = 关掉保护（自检用）  0 = 恢复保护
 * 返回参数     void
 * 备注信息     自检的目的就是试出接线对不对。如果保护还开着，编码器没接好的那一侧
 *              会在 500ms 后被锁死，反而看不出真正的问题
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_test_mode(uint8 on)
{
    if(0 != on)
    {
        motor_no_enc_check = 1;
    }
    else
    {
        motor_no_enc_check = 0;
        motor_enc_fail_l = 0;
        motor_enc_fail_r = 0;
        motor_test_duty_on = 0;                 /* 恢复时顺便退出"固定占空比"模式 */
        motor_test_duty_l  = 0;
        motor_test_duty_r  = 0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     台架自检专用：固定占空比开环输出（不看编码器、不跑速度环）
 * 参数说明     left_duty / right_duty   目标占空比(万分比, 带符号, 10000 = 100%)
 * 返回参数     void
 * 备注信息     为什么要它：自检阶段编码器接线还没验证。如果这时候用闭环，
 *              编码器读数一乱，速度环就会把电机一会儿正一会儿反，
 *              于是"轮子时前时后"——根本分不清是接线问题还是闭环在打架。
 *              这里只验"哪一侧 + 往前还是往后"，所以必须纯开环。
 *              传 (0, 0) 就是停；实际输出仍受限幅 CAR_MOTOR_DUTY_MAX、
 *              总开关 CAR_ENABLE_MOTOR 和斜坡 CAR_MOTOR_RAMP_STEP 的约束
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_test_duty(int16 left_duty, int16 right_duty)
{
    motor_test_duty_l  = motor_limit_duty(left_duty);
    motor_test_duty_r  = motor_limit_duty(right_duty);
    motor_test_duty_on = 1;
    motor_i_l = 0;                              /* 退出这个模式时不会带着积分一窜 */
    motor_i_r = 0;
    motor_enc_fail_l = 0;
    motor_enc_fail_r = 0;
}

/* 编码器方向可以在运行时翻转（用无线串口发命令，不必重新编译烧录） */
int16 motor_enc_raw_left   (void) { return motor_enc_raw_l; }
int16 motor_enc_raw_right  (void) { return motor_enc_raw_r; }
uint8 motor_enc_inv_left   (void) { return motor_enc_inv_l; }
uint8 motor_enc_inv_right  (void) { return motor_enc_inv_r; }
uint8 motor_output_off_get (void) { return motor_out_off; }

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     硬关 / 恢复电机输出（台架调编码器时用）
 * 参数说明     off         1 = 硬关（PWM 直接 0，PI 完全不参与）  0 = 恢复
 * 返回参数     void
 * 备注信息     和 motor_run(0) 的区别：motor_run(0) 只是把"目标速度"设为 0，
 *              PI 仍会反向发力去顶住轮子（手推轮子时电机会跟你抢）；这个是真的不给输出
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_output_off(uint8 off)
{
    motor_out_off = (0 != off) ? 1 : 0;

    if(0 != motor_out_off)
    {
        motor_enable   = 0;
        motor_target_l = 0;
        motor_target_r = 0;
        motor_i_l      = 0;
        motor_i_r      = 0;
        motor_test_duty_on = 0;
    }
}

uint8 motor_enc_mode_get   (void) { return motor_enc_mode_now; }
uint8 motor_enc_socket_get (void) { return motor_enc_socket; }

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     单独指定"左轮编码器"用哪个插座（右轮不动）—— 万一只是一路插座/走线有问题，可以换一路用
 * 参数说明     which   0 = 编码器1（P40/P42, PWMC）   1 = 编码器2（P60/P62, PWMA）
 *                      2 = 编码器3（P80/P44, TIM17）   3 = 编码器4（P90/P46, TIM18）
 * 备注信息     插座3/4 走定时器接口，不支持正交解码，会自动切成"带方向"模式
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_enc_left_socket_set(uint8 which)
{
    switch(which & 0x03)
    {
        case 0:     /* 编码器1：P40/P42（PWMC） */
            enc_idx_l = PWMC_ENCODER; enc_pulse_l = PWMC_ENCODER_CH1P_P40; enc_dir_l = PWMC_ENCODER_CH2P_P42;
        break;

        case 1:     /* 编码器2：P60/P62（PWMA） */
            enc_idx_l = PWMA_ENCODER; enc_pulse_l = PWMA_ENCODER_CH1P_P60; enc_dir_l = PWMA_ENCODER_CH2P_P62;
        break;

        case 2:     /* 编码器3：P80/P44（TIM17） */
            enc_idx_l = TIM17_ENCODER; enc_pulse_l = TIM17_ENCODER_CH1_P80; enc_dir_l = IO_P44;
            motor_enc_mode_now = 1;
        break;

        default:    /* 编码器4：P90/P46（TIM18） */
            enc_idx_l = TIM18_ENCODER; enc_pulse_l = TIM18_ENCODER_CH1_P90; enc_dir_l = IO_P46;
            motor_enc_mode_now = 1;
        break;
    }

#if CAR_ENABLE_ENCODER
    motor_enc_hw_init();
#endif
    motor_enc_fail_l = 0;
    motor_enc_fail_r = 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     选择"车上的编码器插在主板哪两个插座上"（主板丝印：编码器1/2/3/4）
 * 参数说明     which   0 = 插座1当左、插座2当右（P40/P42 + P60/P62）   ← 默认
 *                      1 = 插座2当左、插座1当右（P60/P62 + P40/P42）
 *                      2 = 插座3当左、插座4当右（TIM17: P80/P44 + TIM18: P90/P46）
 *                      3 = 插座4当左、插座3当右
 * 返回参数     void
 * 备注信息     原理图上：编码器1 = P4.0/P4.2、编码器2 = P6.0/P6.2、编码器3 = P8.0/P4.4、编码器4 = P9.0/P4.6
 *              插座3/4 走定时器接口，**不支持正交解码**，所以会自动切成"带方向"模式
 *-----------------------------------------------------------------------------------------------------------------*/
void motor_enc_socket_set(uint8 which)
{
    motor_enc_socket = (uint8)(which & 0x03);

    switch(motor_enc_socket)
    {
        case 0:     /* 插座1 = 左(P40/P42, PWMC)，插座2 = 右(P60/P62, PWMA) */
            enc_idx_l = PWMC_ENCODER; enc_pulse_l = PWMC_ENCODER_CH1P_P40; enc_dir_l = PWMC_ENCODER_CH2P_P42;
            enc_idx_r = PWMA_ENCODER; enc_pulse_r = PWMA_ENCODER_CH1P_P60; enc_dir_r = PWMA_ENCODER_CH2P_P62;
        break;

        case 1:     /* 插座2 = 左，插座1 = 右 */
            enc_idx_l = PWMA_ENCODER; enc_pulse_l = PWMA_ENCODER_CH1P_P60; enc_dir_l = PWMA_ENCODER_CH2P_P62;
            enc_idx_r = PWMC_ENCODER; enc_pulse_r = PWMC_ENCODER_CH1P_P40; enc_dir_r = PWMC_ENCODER_CH2P_P42;
        break;

        default:    /* 插座3/4 = 定时器接口（只能"带方向"） */
            if(2 == motor_enc_socket)
            {
                enc_idx_l = TIM17_ENCODER; enc_pulse_l = TIM17_ENCODER_CH1_P80; enc_dir_l = IO_P44;
                enc_idx_r = TIM18_ENCODER; enc_pulse_r = TIM18_ENCODER_CH1_P90; enc_dir_r = IO_P46;
            }
            else
            {
                enc_idx_l = TIM18_ENCODER; enc_pulse_l = TIM18_ENCODER_CH1_P90; enc_dir_l = IO_P46;
                enc_idx_r = TIM17_ENCODER; enc_pulse_r = TIM17_ENCODER_CH1_P80; enc_dir_r = IO_P44;
            }
            motor_enc_mode_now = 1;             /* 定时器接口不支持正交解码 */
        break;
    }

#if CAR_ENABLE_ENCODER
    motor_enc_hw_init();
#endif
    motor_enc_fail_l = 0;
    motor_enc_fail_r = 0;
}
void  motor_enc_mode_set   (uint8 mode)
{
    motor_enc_mode_now = (0 != mode) ? 1 : 0;
#if CAR_ENABLE_ENCODER
    motor_enc_hw_init();                        /* 重新按新模式配置编码器接口 */
#endif
    motor_enc_fail_l = 0;
    motor_enc_fail_r = 0;
}
void  motor_enc_inv_set_left (uint8 on)  { motor_enc_inv_l = (0 != on) ? 1 : 0; }
void  motor_enc_inv_set_right(uint8 on)  { motor_enc_inv_r = (0 != on) ? 1 : 0; }

/* 以下为调试/显示用的只读接口 */
uint16 motor_tick_get  (void) { return motor_tick_cnt; }
int16 motor_speed_left (void) { return motor_speed_l; }
int16 motor_speed_right(void) { return motor_speed_r; }
int16 motor_enc_left   (void) { return motor_enc_l;   }
int16 motor_enc_right  (void) { return motor_enc_r;   }
int16 motor_duty_left  (void) { return motor_duty_l;  }
int16 motor_duty_right (void) { return motor_duty_r;  }
