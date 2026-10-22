/*********************************************************************************************************************
* 前轮转向舵机驱动 对外接口
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入
********************************************************************************************************************/

#ifndef __CAR_SERVO_H
#define __CAR_SERVO_H

#include "zf_common_headfile.h"

/* 初始化舵机 PWM 并回到中位 */
void    servo_init        (void);

/* 设置打角量(度)：> 0 往右，< 0 往左。内部带左右极限限幅与缓变限速 */
void    servo_set_steer   (float steer_deg);

/* 回中位 */
void    servo_center      (void);

/* 读取当前角度(度)，调试用 */
float   servo_get_angle   (void);

/* 运行时软限幅：舵机角度对称限制在 中位 ± deg 度以内（deg <= 0 表示关掉软限幅，只用配置里的 LEFT/RIGHT）。
 * 用途：赛道上一试就想把打角改小时，用无线串口 , / . 在线调，不用重新编译烧录。
 * 注意：它只会在配置里 CAR_SERVO_LEFT_DEG / RIGHT_DEG 的基础上**再收紧**，绝不会放宽。 */
void    servo_set_soft_limit (float deg);
float   servo_get_soft_limit (void);

/* 直行微调(度)：> 0 = 前轮往右一点点（用来修掉"直道上车往左跑"，或者视觉中线本身有偏差）。
 *   ★ 可以不用重新烧录：无线串口 ; = 往左一步、' = 往右一步、{ } = 粗调、0 = 清零
 *   它调的是"打角量的零点"：所以 steer = 0（走直道、或者还没起步回正）时前轮也跟着挪，
 *   方向和在赛道上看到的一致 —— 车往左跑，就按撇号（往右修） */
void    servo_set_offset  (float deg);
void    servo_nudge       (float delta_deg);
float   servo_get_offset  (void);

/* 舵机响应速度上限(度/10ms)：运行时用无线串口 f / g 在线调 */
void    servo_set_slew      (float deg_per_10ms);
float   servo_get_slew      (void);

#endif
