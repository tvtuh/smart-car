/*********************************************************************************************************************
* 后轮双电机驱动 + 速度闭环（DRV8701E）对外接口
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入
********************************************************************************************************************/

#ifndef __CAR_MOTOR_H
#define __CAR_MOTOR_H

#include "zf_common_headfile.h"

/* 故障码：motor_is_fault() 非 0 时，用 motor_fault_reason() 看具体是哪一种 */
#define CAR_MOTOR_FAULT_NONE        (0)
#define CAR_MOTOR_FAULT_VISION      (1)         /* 视觉心跳超时：图像停了 */
#define CAR_MOTOR_FAULT_ENCODER     (2)         /* 编码器读不到计数 / 轮子堵转 */

/* 初始化：DIR 引脚、电机 PWM、编码器、10ms 速度环中断。上电后电机保持静止 */
void    motor_init          (void);

/* 设置左右轮目标速度(cm/s)，正值 = 前进。实际输出由 10ms 速度环执行 */
void    motor_set_target    (int16 left_cm_s, int16 right_cm_s);

/* 允许(1) / 禁止(0) 电机输出。0 时目标清零，靠斜坡平滑停下 */
void    motor_run           (uint8 enable);

/* 急停：占空比立刻归零，不走斜坡 */
void    motor_stop          (void);

/* 每帧图像处理完喂一次心跳，超时未喂则自动停车锁死 */
void    motor_heartbeat     (void);
void    motor_heartbeat_link_guard (void);      /* 【fw42】发图（阻塞）之前把心跳续长，避免链路卡顿被误判成"视觉停了" */

/* 1 = 已锁死（视觉超时或编码器故障），需要复位或重新 motor_run(1) */
uint8   motor_is_fault      (void);

/* 故障码，取值见上面的 CAR_MOTOR_FAULT_xxx */
uint8   motor_fault_reason  (void);

/* 台架自检专用：1 = 临时关掉"编码器读不到就锁死"的保护（自检本来就是要试接线对不对） */
void    motor_test_mode     (uint8 on);

/* 台架自检专用：不管编码器、不跑速度环，把左右轮占空比直接锁成给定值(万分比, 带符号)
 * 自检时编码器接线还没验证，闭环会因为读数乱而把电机反复正反转；看"哪一侧 / 往前还是往后"必须纯开环。
 * 传 (0, 0) 或调用 motor_test_mode(0) 都能退出这个模式 */
void    motor_test_duty     (int16 left_duty, int16 right_duty);

/* 10ms 计数（速度环周期数），主循环里做定时用 */
uint16  motor_tick_get      (void);

/* 硬关 / 恢复电机输出（1 = 硬关，PWM 直接 0，PI 不参与）；台架调编码器时用 */
void    motor_output_off    (uint8 off);
uint8   motor_output_off_get(void);

/* 编码器原始计数（没取反的）和"当前方向标志"：调试 + 无线串口里在线翻转方向用 */
int16   motor_enc_raw_left  (void);
int16   motor_enc_raw_right (void);
uint8   motor_enc_inv_left  (void);
uint8   motor_enc_inv_right (void);
void    motor_enc_inv_set_left  (uint8 on);
void    motor_enc_inv_set_right (uint8 on);

/* 编码器模式（0 = 正交 A/B，1 = 带方向 PULSE+DIR）：也可以运行时切换，方便一试就知道接的是哪种 */
uint8   motor_enc_mode_get  (void);
void    motor_enc_mode_set  (uint8 mode);

/* 编码器插在主板哪两个插座上（0..3，见 car_motor.c 里 motor_enc_socket_set 的说明） */
uint8   motor_enc_socket_get(void);
void    motor_enc_socket_set(uint8 which);

/* 只换"左轮编码器"用的插座（0=编码器1 P40/P42，1=编码器2 P60/P62，2=编码器3 P80/P44，3=编码器4 P90/P46） */
void    motor_enc_left_socket_set(uint8 which);

/* 调试用只读接口 */
int16   motor_speed_left    (void);         /* 实测左轮速度(cm/s)，开环模式恒为 0 */
int16   motor_speed_right   (void);
int16   motor_enc_left      (void);         /* 最近一个速度环周期的原始计数 */
int16   motor_enc_right     (void);
int16   motor_duty_left     (void);         /* 当前输出占空比(万分比, 带符号) */
int16   motor_duty_right    (void);

#endif
