/*********************************************************************************************************************
* 整车闭环控制：视觉中线 -> 打角 / 速度
*
* 分工：
*   视觉算法（main.c 里的 track_image_process 那一套）负责给出 x1/x2/x3 三条边线与 track_type
*   本模块负责"看着中线开车"：
*     1. 近/中/远三行中线加权 -> 瞄点误差 -> PD -> 舵机打角（远处权重大 = 预瞄，过弯才稳）
*     2. 弯道程度（远端偏移 - 近端偏移）-> 自动减速；十字压速；丢线爬行、久丢停车
*     3. 打角 -> 左右轮差速，内侧轮减速、外侧轮加速
*     4. 起步门（延时或按 KEY1）、视觉心跳看门狗、故障锁死
*
* 重要：本模块只"读"视觉的结果，不改动视觉算法一个字
*
* 修改记录
* 日期              作者           备注
* 2026-09-16        -              首次加入
********************************************************************************************************************/

#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include "zf_common_headfile.h"

/* 下面三个取值必须与 main.c 里视觉算法给出的 track_type 一致（只做引用，不参与计算） */
#define CAR_TRACK_NORMAL        (0)         /* main.c: TRACK_TYPE_NORMAL */
#define CAR_TRACK_CROSS         (1)         /* main.c: TRACK_TYPE_CROSS  */
#define CAR_TRACK_LOST          (2)         /* main.c: TRACK_TYPE_LOST   */

/* 初始化舵机、电机、编码器、速度环与调试串口信息。上电后车保持静止 */
void    car_control_init        (void);

/* 起步门：按配置延时或等按键，返回后电机才允许输出。放在主循环之前调用 */
void    car_control_wait_start  (void);

/* 每处理完一帧图像调用一次：算打角、算速度、下发执行器 */
void    car_control_update      (void);
/* 【fw47d】main.c 问它：这一帧要不要调用 ring_road_process()（弯道里跳过，理由见 car_control.c） */
uint8   car_ring_module_ok       (void);
/* 【fw48】main.c 问它：这一帧要不要上传图像（无线串口 ! 切换；0 = 只保持连接不发图） */
uint8   car_img_send_ok          (void);

/* Service commands; send replies only when requested and no camera frame is pending. */
void    car_wireless_poll   (void);
uint8   car_wireless_poll_due   (void);

/* ★ 开机就把无线串口点亮（只做 uart_init + 发横幅，可以重复调用，幂等）。
 *   为什么要有它：视觉链路（连热点 / 连 TCP / 摄像头）出错时会重试很久，
 *   而 car_control_init() 原来排在它们后面 —— 那段时间串口助手上什么都没有，像模块坏了。
 *   main.c 在 debug_init() 之后立刻调用它，1 秒内就能看到东西。 */
void    car_wireless_link_init   (void);

/* 往外发一行进度信息（开机阶段用；无线串口没点亮就什么都不做） */
void    car_wireless_progress    (const char *s);

/* 开机阶段轮询：用户在无线串口里发过 x / X 就返回 1（用来"跳过视觉链路，别等了"） */
uint8   car_wireless_skip_requested(void);

/* 停车：电机停、舵机回正、并退回"未起步"状态 */
void    car_control_stop        (void);

#endif
