/*********************************************************************************************************************
* STC32G144K Opensourec Library 即（STC32G144K 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G144K 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-11-20        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// *************************** 例程硬件连接说明 ***************************
// 接入舵机 主板上对应有舵机的接口 务必注意不要插反 <红色电源> <黑色信号> <黄色/橙色/棕色/白色...其它彩色的那根是信号>
//      模块管脚            单片机管脚
//      PWM1                A0
//      PWM2                A2
//      PWM3                A4
//      PWM4                A6
//      GND                 舵机电源 GND 联通 核心板电源地 GND
//      VCC                 舵机电源输出


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程
//
// 2.在断电情况下 完成核心板插到主板的核心板插座上 确认核心板与主板插座没有明显缝隙
//
// 3.然后将舵机与主板正确连接 请务必注意不要插反 然后使用电池给主板供电打开开关
//
// 4.正常情况下舵机会来回转动 最好在舵机没有装在车上固定连接转向连杆时测试 防止安装位置不对造成堵转烧舵机
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************
#define CHANNEL_NUMBER          (4)
#define SERVO_PWM1              (PWME_CH1P_PA0)                         // 定义主板上舵机对应引脚
#define SERVO_PWM2              (PWME_CH2P_PA2)                         // 定义主板上舵机对应引脚
#define SERVO_PWM3              (PWME_CH3P_PA4)                         // 定义主板上舵机对应引脚
#define SERVO_PWM4              (PWME_CH4P_PA6)                         // 定义主板上舵机对应引脚

#define SERVO_FREQ              (50 )                                           // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_L_MAX             (80 )                                           // 定义主板上舵机活动范围 角度
#define SERVO_R_MAX             (100)                                           // 定义主板上舵机活动范围 角度

// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#define SERVO_DUTY(x)         ((float)PWM_DUTY_MAX / (1000.0 / (float)SERVO_FREQ) * (0.5 + (float)(x) / 90.0))

#if ((SERVO_FREQ < 50) || (SERVO_FREQ > 300))
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif

#define LED                 (IO_P52)

float servo_motor_duty = 90.0;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

int16 duty = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {SERVO_PWM1, SERVO_PWM2, SERVO_PWM3, SERVO_PWM4};


void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

	// 此处编写用户代码 例如外设初始化代码等
    for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++)
    {
        pwm_init(channel_list[channel_index], SERVO_FREQ, 0);
    }
    gpio_init(LED, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        gpio_toggle_level(LED); 
        // 此处编写需要循环执行的代码
        
        for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index++)
        {
            pwm_set_duty(channel_list[channel_index], SERVO_DUTY(servo_motor_duty));
            if(servo_motor_dir)
            {
                servo_motor_duty ++;
                if(servo_motor_duty >= SERVO_R_MAX)
                {
                    servo_motor_dir = 0x00;
                }
            }
            else
            {
                servo_motor_duty --;
                if(servo_motor_duty <= SERVO_L_MAX)
                {
                    servo_motor_dir = 0x01;
                }
            }
            system_delay_ms(100);    
        }
                                            // 延时
        // 此处编写需要循环执行的代码
    }
}
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
//
// 问题1：舵机不动
//      检查舵机供电是否正常 至少5V供电 不可以用杜邦线供电
//      检查PWM信号是否正常 是否连通


