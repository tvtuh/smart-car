/*********************************************************************************************************************
* 前轮转向舵机驱动
*
* 占空比换算与 E02_03 例程完全一致：
*   duty = PWM_DUTY_MAX / (1000 / 频率) * (0.5 + 角度 / 90)
*   50Hz 下：角度 90 度 -> 1.5ms 高电平 -> 7.5% 占空比
*
* 本模块做四件事：角度限幅、中位微调、缓变限速（防止舵机齿轮被打坏）、打角方向取反
* 打角方向：steer > 0 = 往右打，steer < 0 = 往左打（反了就把 CAR_SERVO_INVERT 改成 1）
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入
********************************************************************************************************************/

#include "car_config.h"
#include "car_servo.h"
#include "car_motor.h"                     /* 只为拿 10ms 心跳 tick：缓变限速要按时间算 */

static float servo_now_deg = CAR_SERVO_CENTER_DEG;      /* 舵机当前角度(度)，缓变限速用 */
static float servo_soft_limit = CAR_STEER_MAX_DEG;      /* 运行时软限幅(度)：中位 ± 这个值 */
static float servo_off_deg = CAR_SERVO_OFF_DEG;        /* 直行微调(度)：> 0 = 前轮往右一点点，叠加在每个打角量上 */
static float servo_zero_deg = CAR_SERVO_CENTER_DEG + CAR_SERVO_TRIM_DEG;  /* 打角量 = 0（前轮正中）对应的舵机角度：软限幅绕它对称 */
static float servo_slew_rate = CAR_SERVO_SLEW_DEG_PER_10MS;  /* 舵机响应速度上限(度/10ms)：f / g 在线调 */
static float servo_hard_limit = CAR_SERVO_HARD_LIMIT_DEG; /* 打角硬极限(度)：绕"打角零点"对称，左右各这么多 */
static uint16 servo_slew_tick = 0;                          /* 上次调舵机时的 10ms tick，用来算"过了多少时间" */

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     重算"打角零点"：打角量 = 0（前轮正中）时舵机应该停的角度
 * 备注信息     它 = CAR_SERVO_CENTER_DEG + CAR_SERVO_TRIM_DEG + 直行微调（取反开关会影响加还是减）。
 *              软限幅以它为中心，所以"往左能打多少度"和"往右能打多少度"永远一样大 ——
 *              否则直行微调一加，一边的可用打角就被吃掉一截。
 *              每次 servo_init() 和直行微调变化时都要调一次
 *-----------------------------------------------------------------------------------------------------------------*/
static void servo_update_zero(void)
{
    if(0 != CAR_SERVO_INVERT)
    {
        servo_zero_deg = CAR_SERVO_CENTER_DEG - servo_off_deg + CAR_SERVO_TRIM_DEG;
    }
    else
    {
        servo_zero_deg = CAR_SERVO_CENTER_DEG + servo_off_deg + CAR_SERVO_TRIM_DEG;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     角度 -> 舵机 PWM 占空比
 * 参数说明     deg         机械角度(度)，0 ~ 180
 * 返回参数     PWM 占空比(0 ~ PWM_DUTY_MAX)
 *-----------------------------------------------------------------------------------------------------------------*/
static uint32 servo_angle_to_duty(float deg)
{
    float duty;

    duty = (float)PWM_DUTY_MAX / (1000.0 / (float)CAR_SERVO_FREQ) * (0.5 + deg / 90.0);

    if(duty < 1.0)                 { duty = 1.0; }
    if(duty > (float)PWM_DUTY_MAX) { duty = (float)PWM_DUTY_MAX; }

    return (uint32)duty;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     输出一个机械角度（内部用：先限幅，再算占空比）
 * 参数说明     deg         机械角度(度)
 * 返回参数     void
 * 备注信息     CAR_ENABLE_SERVO = 0 时只更新内部状态、不写 PWM 寄存器
 *-----------------------------------------------------------------------------------------------------------------*/
static void servo_write(float deg)
{
    if(servo_soft_limit > 0.0)                          /* 运行时软限幅：只收紧，不放宽 */
    {
        /* 注意：限幅要绕"真正的直行位置"（打角零点）对称 —— 它既含 CAR_SERVO_TRIM_DEG 的机械微调，
         * 也含你在无线串口里调出来的直行微调；如果绕 CAR_SERVO_CENTER_DEG 对称，左右打角就会一边大一边小 */
        float mid = servo_zero_deg;

        if(deg > (mid + servo_soft_limit)) { deg = mid + servo_soft_limit; }
        if(deg < (mid - servo_soft_limit)) { deg = mid - servo_soft_limit; }
    }
    /* ★ 对称硬限幅：绕"打角零点"（前轮正中）左右各 servo_hard_limit 度。
     *   没有它的话，绝对限幅（LEFT/RIGHT_DEG）会把零点偏移算进去，变成一边大一边小 ——
     *   本车零点 97.5°，原来 [70,110] 只给了"左 12.5° / 右 27.5°"，左转直接过不了弯。 */
    if(servo_hard_limit > 0.0)
    {
        if(deg > (servo_zero_deg + servo_hard_limit)) { deg = servo_zero_deg + servo_hard_limit; }
        if(deg < (servo_zero_deg - servo_hard_limit)) { deg = servo_zero_deg - servo_hard_limit; }
    }


    if(deg < CAR_SERVO_LEFT_DEG)  { deg = CAR_SERVO_LEFT_DEG;  }
    if(deg > CAR_SERVO_RIGHT_DEG) { deg = CAR_SERVO_RIGHT_DEG; }

    servo_now_deg = deg;

#if CAR_ENABLE_SERVO
    pwm_set_duty(CAR_SERVO_PWM, servo_angle_to_duty(deg));
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     舵机初始化：PWM 50Hz，并回到中位
 * 参数说明     void
 * 返回参数     void
 * 备注信息     CAR_ENABLE_SERVO = 0 时不初始化 PWM（舵机是松的，仅用于单独调电机）
 *-----------------------------------------------------------------------------------------------------------------*/
void servo_init(void)
{
#if CAR_ENABLE_SERVO
    pwm_init(CAR_SERVO_PWM, CAR_SERVO_FREQ, 0);
#endif
    servo_now_deg = CAR_SERVO_CENTER_DEG;
    servo_off_deg = CAR_SERVO_OFF_DEG;                  /* 直行微调恢复配置初值 */
    servo_update_zero();                                /* 打角零点也要跟着初始化 */
    servo_write(servo_zero_deg);                        /* 回到"前轮正中"（含直行微调） */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     设置打角量（相对中位）
 * 参数说明     steer_deg   打角量(度)：> 0 往右，< 0 往左
 * 返回参数     void
 * 备注信息     每次调用最多变化 servo_slew_rate * 经过的10ms 度（默认 2 度/10ms = 200 度/秒），
 *              正常跑车时这个限制不影响跟随（只削掉突变），但会让舵机齿轮活得久一点
 *-----------------------------------------------------------------------------------------------------------------*/
void servo_set_steer(float steer_deg)
{
    float target;
    steer_deg += servo_off_deg;                         /* 直行微调：先叠加，再走下面的取反和中位换算 */

    if(0 != CAR_SERVO_INVERT)
    {
        target = CAR_SERVO_CENTER_DEG - steer_deg;
    }
    else
    {
        target = CAR_SERVO_CENTER_DEG + steer_deg;
    }
    target += CAR_SERVO_TRIM_DEG;

    /* 缓变限速：按【时间】算，不是按"调用了几次"。
     * 为什么必须这样：本函数是每处理完一帧图像调用一次的，帧率一低（发图阻塞时只有 10~20fps），
     * "每帧最多 3 度"就只剩 30~60 度/秒 —— 入弯时舵机肉眼可见地反应不过来。
     * 现在：每过 10ms 最多转 servo_slew_rate 度，和帧率无关 */
    {
        uint16 now   = motor_tick_get();
        uint16 dt    = (uint16)(now - servo_slew_tick);
        float  allow = servo_slew_rate * (float)dt;

        if(allow > CAR_SERVO_SLEW_STEP_MAX) { allow = CAR_SERVO_SLEW_STEP_MAX; }   /* 单次最多跳这么多，保护齿轮 */
        if(dt > 0) { servo_slew_tick = now; }                                     /* 同一个 10ms 内多次调用不重复加速 */

        if(target > (servo_now_deg + allow)) { target = servo_now_deg + allow; }
        if(target < (servo_now_deg - allow)) { target = servo_now_deg - allow; }
    }

    servo_write(target);
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     回到中位
 * 参数说明     void
 * 返回参数     void
 *-----------------------------------------------------------------------------------------------------------------*/
void servo_center(void)
{
    servo_write(servo_zero_deg);                        /* 回正 = 回到打角零点（含直行微调） */
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     读取当前舵机角度（调试用）
 * 参数说明     void
 * 返回参数     当前角度(度)
 *-----------------------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     运行时软限幅（中位 ± deg 度）：deg <= 0 表示关掉，只用配置里的硬极限
 *-----------------------------------------------------------------------------------------------------------------*/
void servo_set_soft_limit(float deg)
{
    servo_soft_limit = (deg > 0.0) ? deg : 0.0;
}

float servo_get_soft_limit(void)
{
    return servo_soft_limit;
}


float servo_get_angle(void)
{
    return servo_now_deg;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     直行微调(度)：> 0 = 前轮往右一点点（修"直道上车往左跑"），< 0 = 往左
 * 参数说明     deg         微调量(度)，超过 CAR_SERVO_OFF_MAX_DEG 会被限幅
 * 返回参数     void
 * 备注信息     它改的是"打角量的零点"，所以 servo_set_steer(0)（走直道 / 还没起步回正）时前轮也跟着挪。
 *              微调量一变，立刻按同样的方向把前轮挪一点点：这样静止（没有图像帧）时也能看着前轮调中位。
 *              这一步不经过缓变限速 —— 它一次只有按键的幅度（0.5 / 2 度），而且 servo_write 里还有硬极限兜着
 *-----------------------------------------------------------------------------------------------------------------*/
void servo_set_offset(float deg)
{
    float old_zero = servo_zero_deg;                    /* 打角零点原来的角度 */

    if(deg >  CAR_SERVO_OFF_MAX_DEG) { deg =  CAR_SERVO_OFF_MAX_DEG; }
    if(deg < -CAR_SERVO_OFF_MAX_DEG) { deg = -CAR_SERVO_OFF_MAX_DEG; }

    servo_off_deg = deg;
    servo_update_zero();                                /* 打角零点跟着微调一起挪（取反规则已经含在里面） */

    /* 立刻把前轮按同样的方向挪一点点：静止（没有图像帧）时也能看着前轮调中位。
     * 这一步不经过缓变限速 —— 它一次只有按键的幅度（0.5 / 2 度），而且 servo_write 里还有硬极限兜着 */
    servo_write(servo_now_deg + (servo_zero_deg - old_zero));
}

/* 相对当前值再挪一点点（无线串口每按一次键调一次） */
void servo_nudge(float delta_deg)
{
    servo_set_offset(servo_off_deg + delta_deg);
}

float servo_get_offset(void)
{
    return servo_off_deg;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     舵机响应速度上限(度/10ms)：无线串口 f / g 在线调
 * 参数说明     deg_per_10ms   每 10ms 最多转多少度（0.1 ~ 20）
 * 备注信息     ★ 入弯反应慢就调大；舵机抖/齿轮响就调小
 *-----------------------------------------------------------------------------------------------------------------*/
void servo_set_slew(float deg_per_10ms)
{
    if(deg_per_10ms <  0.1) { deg_per_10ms =  0.1; }
    if(deg_per_10ms > 20.0) { deg_per_10ms = 20.0; }
    servo_slew_rate = deg_per_10ms;
}

float servo_get_slew(void)
{
    return servo_slew_rate;
}

/* Positive is RIGHT in the vehicle frame, independent of servo mounting. */
float servo_get_steer(void)
{
    return (0 != CAR_SERVO_INVERT) ? (servo_zero_deg - servo_now_deg)
                                  : (servo_now_deg - servo_zero_deg);
}
