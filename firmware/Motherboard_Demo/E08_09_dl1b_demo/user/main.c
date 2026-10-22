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
//
// 接入 DL1A 模块
//      模块管脚            单片机管脚
//      SCL                 查看 zf_device_dl1b.h 中 DL1B_SCL_PIN  宏定义
//      SDA                 查看 zf_device_dl1b.h 中 DL1B_SDA_PIN  宏定义
//      XS                  查看 zf_device_dl1b.h 中 DL1B_XS_PIN   宏定义
//      VCC                 5V 电源
//      GND                 电源地


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，使用 type-c接口，并连接好 DL1B 模块,在断电情况下完成连接
//      ( 如果使用配套的学习主板测试，则直接将 DL1B 模块接到对应的接口位置 )
//
// 3.使用 Type-C 给核心板供电
//      ( 如果使用配套的学习主板测试，则需要直接使用电池供电，2S锂电池/蓝电池电压不应低于7.8V，3S锂电池电压不应低于11.3V )
//
// 4.可以在串口助手上看到如下串口信息：
//      DL1B distance data: ...
//      DL1B distance data: ...
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查



// **************************** 代码区域 ****************************
#define LED1                        (IO_P90 )

#define PIT                         (TIM6_PIT )                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用

vuint8  pit_ms_flag          = 0;
uint16  pit_ms_count         = 0;
uint16  dl1b_refresh_freq    = 0;
void pit_handler (void);

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);  // 初始化 LED1 输出 默认高电平 推挽输出模式

    while(1)
    {
        if(dl1b_init())
        {
            printf("\r\n DL1B init error. \r\n");    // DL1B 初始化失败
        }
        else
        {
            break;
        }
        gpio_toggle_level(LED1);                     // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
    pit_ms_init(PIT, 5, pit_handler);
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        if(pit_ms_flag)
        {
            printf("\r\n DL1B distance data: %5d \r\n", dl1b_distance_mm);
            printf("\r\n DL1B refresh freq: %5d \r\n", dl1b_refresh_freq);
            gpio_toggle_level(LED1);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
            dl1b_refresh_freq = 0;
            pit_ms_flag = 0;
        }
        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_handler();
//-------------------------------------------------------------------------------------------------------------------
void pit_handler (void)
{
    // 如果开启了 INT 的使能 那么会自动触发外部中断进行数据采集 就不需要自行采集了
#if (!DL1B_INT_ENABLE)
    if(0 == pit_ms_count % 10)                                                  // 每 10ms 获取一次测距信息 周期 10 ms 频率 100Hz
    {
        dl1b_get_distance();                                                    // 测距调用频率不应高于 100Hz 周期不应低于 10ms
    }
#endif
    // 通过判断 dl1a_finsh_flag 来确定是否更新了数据 记得获取了数据后就清空标志位
    if(dl1b_finsh_flag)
    {
        dl1b_refresh_freq ++;
        dl1b_finsh_flag = 0;
    }
    pit_ms_count = (pit_ms_count == 995) ? (0) : (pit_ms_count + 5);            // 1000ms 周期计数
    if(0 == pit_ms_count % 1000)
    {
        pit_ms_flag = 1;
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
//
// 问题3：串口输出 DL1B init error.
//      检查 DL1B 的接线是否正确是否松动
//      检查 DL1B 的模块是不是坏了
//      给信号线加上拉看看
//
// 问题4：DL1B 数值异常 显示 819x 或者一个固定数据
//      测距有效范围为 1400mm 超过这个距离会显示 819x 的无效值
//      如果数据显示正常，突然变为一个固定值不再变化，那么尝试断电重启，如果重启后正常数据，证明是 XS 信号上受到干扰导致模块丢失数据

