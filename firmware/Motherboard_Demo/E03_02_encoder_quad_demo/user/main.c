/*********************************************************************************************************************
* STC32G Opensourec Library 即（STC32G 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2025 SEEKFREE 逐飞科技
*
* 本文件是STC32G144K开源库的一部分
*
* STC32G 开源库 是免费软件
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
* 2024-08-01        大W            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"

// *************************** 例程硬件连接说明 ***************************
// 使用 type-c 连接
//      直接将type-c插入核心板，即可使用USB-CDC
//
// 接入方向编码器连接
//      模块管脚            单片机管脚
//      LSB                 ENCODER_DIR_PULSE_1 宏定义的引脚 	默认 P60
//      DIR                 ENCODER_DIR_DIR_1   宏定义的引脚 	默认 P62
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//
//      LSB                 ENCODER_DIR_PULSE_2 宏定义的引脚 	默认 P40
//      DIR                 ENCODER_DIR_DIR_2   宏定义的引脚 	默认 P42
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//
//      LSB                 ENCODER_DIR_PULSE_3 宏定义的引脚 	默认 P80
//      DIR                 ENCODER_DIR_DIR_3   宏定义的引脚 	默认 P45
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//
//      LSB                 ENCODER_DIR_PULSE_4 宏定义的引脚 	默认 P90
//      DIR                 ENCODER_DIR_DIR_4   宏定义的引脚 	默认 P46
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
//
// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，使用 type-c接口，并连接好编码器，在断电情况下完成连接
//
// 2.将 type-c的USB-CDC接口连接 电脑，完成上电
//
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
//
// 4.可以在串口助手上看到如下串口信息：
//      encoder_data_dir_1 counter     x .
//      encoder_data_dir_2 counter     x .
//      encoder_data_dir_3 counter     x .
//      encoder_data_dir_4 counter     x .
//
// 5.转动编码器就会看到数值变化
//
// STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。
// STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。
// STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。


// **************************** 代码区域 ****************************
#define PIT_CH                          (TIM1_PIT )                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
//#define PIT_PRIORITY                    (TIM1_IRQn)               TIM1的中断优先级默认最低，不可修改，具体看手册。

#define ENCODER_QUAD_1                 	(PWMA_ENCODER)              // 带方向编码器对应使用的编码器接口 
#define ENCODER_QUAD_1_CHA            	(PWMA_ENCODER_CH1P_P60)     // PULSE 对应的引脚
#define ENCODER_QUAD_1_CHB              (PWMA_ENCODER_CH2P_P62)     // DIR 对应的引脚

#define ENCODER_QUAD_2                 	(PWMC_ENCODER)              // 带方向编码器对应使用的编码器接口
#define ENCODER_QUAD_2_CHA   		    (PWMC_ENCODER_CH1P_P40)     // PULSE 对应的引脚
#define ENCODER_QUAD_2_CHB       	    (PWMC_ENCODER_CH2P_P42)     // DIR 对应的引脚

int16 encoder_data_dir_1 = 0;
int16 encoder_data_dir_2 = 0;


void pit_handler (void);

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    // STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。
    // STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。
    // STC32G144K只有PWM接口支持正交解码编码器.定时器接口不支持正交编码器。
	// 如果，看了此段话再来问，我会再次截图给你这段话。
    encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式
    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB);   // 初始化编码器模块与引脚 正交解码编码器模式

    
	// 设置100ms的周期定时器
    pit_ms_init(PIT_CH, 100, pit_handler);                                      // 初始化 PIT 为周期中断 100ms 周期

//    interrupt_set_priority(PIT_PRIORITY, 0);                                  // TIM1的中断优先级默认最低，不可修改，具体看手册。
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        printf("encoder_data_dir_1 counter %d .\r\n", encoder_data_dir_1);      // 输出编码器计数信息
        printf("encoder_data_dir_2 counter %d .\r\n", encoder_data_dir_2);      // 输出编码器计数信息
        system_delay_ms(500);
        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的周期中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    encoder_data_dir_1 = encoder_get_count(ENCODER_QUAD_1);                  // 获取编码器计数
    encoder_data_dir_2 = encoder_get_count(ENCODER_QUAD_2);                  // 获取编码器计数

    encoder_clear_count(ENCODER_QUAD_1);                                		// 清空编码器计数
    encoder_clear_count(ENCODER_QUAD_2);                                		// 清空编码器计数
}
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
//
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口，检查打开的 COM 口是否对应的type-c的USB_CDC
//      如果是的type-c的USB_CDC连接，那么检查下载器线是否松动
//
// 问题2：串口数据乱码
//      查看串口助手设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
//
// 问题3：数值不随编码转动变化
//      如果使用主板测试，主板必须要用电池供电
//      检查编码器是否是正常的，线是否松动，编码器是否发热烧了，是否接错线

