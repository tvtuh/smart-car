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
// 使用 type-c 连接
//      直接将type-c插入核心板，即可使用USB-CDC
//

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，使用 type-c接口，在断电情况下完成连接
//
// 2.将 type-c的USB-CDC接口连接 电脑，完成上电
//
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
//
// 4.可以在串口助手上看到如下串口信息：
//      ADC channel 1 convert data is x.
//      ...
//      ADC channel 1 mean filter convert data is x.
//      ...
//
// 5.将 ADC_CHANNELx 宏定义对应的引脚分别接到 3V3/GND 再对应的信息会看到数据变化
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#define CHANNEL_NUMBER          ( 8 )

#define ADC_CHANNEL1            ( ADC2_CH0_P00 )
#define ADC_CHANNEL2            ( ADC2_CH1_P01 )
#define ADC_CHANNEL3            ( ADC2_CH2_P02 )
#define ADC_CHANNEL4            ( ADC2_CH3_P03 )
#define ADC_CHANNEL5            ( ADC2_CH6_P06 )
#define ADC_CHANNEL6            ( ADC2_CH7_P07 )
#define ADC_CHANNEL7            ( ADC1_CH1_P11 )
#define ADC_CHANNEL8            ( ADC1_CH4_P14 )

uint8 channel_index = 0;
adc_channel_enum channel_list[CHANNEL_NUMBER] =
{
    ADC_CHANNEL1, 
    ADC_CHANNEL2, 
    ADC_CHANNEL3, 
    ADC_CHANNEL4,
    ADC_CHANNEL5,
    ADC_CHANNEL6,
    ADC_CHANNEL7,
    ADC_CHANNEL8,
};


void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
	
    // 此处编写用户代码 例如外设初始化代码等
    adc_init(ADC_CHANNEL1, ADC_12BIT);          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL2, ADC_12BIT);          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL3, ADC_10BIT);          // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL4, ADC_8BIT);           // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL5, ADC_8BIT);           // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL6, ADC_8BIT);           // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL7, ADC_8BIT);           // 初始化对应 ADC 通道为对应精度
    adc_init(ADC_CHANNEL8, ADC_8BIT);           // 初始化对应 ADC 通道为对应精度

                                                // 这里你会发现输出全是 8bit 精度
                                                // 因为这都是同一个 ADC 模块的引脚
                                                // 所以会以最后一个初始化精度为准
    // 此处编写用户代码 例如外设初始化代码等

    
    while(1)
    {
        // 此处编写需要循环执行的代码
        for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index ++)
        {
            printf(
                "ADC channel %d convert data is %d.\r\n",
                channel_index + 1,
                adc_convert(channel_list[channel_index]));                      // 循环输出单次转换结果
        }
        system_delay_ms(500);

        for(channel_index = 0; channel_index < CHANNEL_NUMBER; channel_index ++)
        {
            printf(
                "ADC channel %d mean filter convert data is %d.\r\n",
                channel_index + 1,
                adc_mean_filter_convert(channel_list[channel_index], 10));      // 循环输出 10 次均值滤波转换结果
        }
        system_delay_ms(500);
        // 此处编写需要循环执行的代码
    }
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
