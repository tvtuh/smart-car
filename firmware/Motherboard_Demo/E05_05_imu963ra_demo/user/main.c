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
// 接入 IMU963RA
//      模块管脚            单片机管脚
//      SCL/SPC             查看 zf_device_imu963ra.h 中 IMU963RA_SPC_PIN 宏定义
//      SDA/DSI             查看 zf_device_imu963ra.h 中 IMU963RA_SDI_PIN 宏定义
//      SA0/SDO             查看 zf_device_imu963ra.h 中 IMU963RA_SDO_PIN 宏定义
//      CS                  查看 zf_device_imu963ra.h 中 IMU963RA_CS_PIN  宏定义
//      GND                 电源地 GND
//      3V3                 电源 3V3



// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，使用 type-c接口，在断电情况下完成连接
//
// 2.将 type-c的USB-CDC接口连接 电脑，完成上电
//
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
//
// 4.可以在串口助手上看到如下串口信息：
//      IMU963RA acc data: x-..., y-..., z-...
//      IMU963RA gyro data: x-..., y-..., z-...
//
// 5.移动旋转 IMU963RA 就会看到数值变化
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************
#define LED1                        (IO_P90)

#define PIT                         (TIM6_PIT )                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用
void pit_hanlder (void);

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
    // 此处编写用户代码 例如外设初始化代码等
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式

    while(1)
    {
        if(imu963ra_init())
            printf("\r\nIMU963RA init error.");     // IMU963RA 初始化失败
        else
            break;
        gpio_toggle_level(LED1);                    // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
    }
    pit_ms_init(PIT, 5, pit_hanlder);
    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
        // 此处编写需要循环执行的代码
        printf("\r\nIMU963RA acc data: x=%5d, y=%5d, z=%5d\r\n", imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z);
        printf("\r\nIMU963RA gyro data:  x=%5d, y=%5d, z=%5d\r\n", imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z);
        printf("\r\nIMU963RA mag data: x=%5d, y=%5d, z=%5d\r\n", imu963ra_mag_x, imu963ra_mag_y, imu963ra_mag_z);
        gpio_toggle_level(LED1);                                                // 翻转 LED 引脚输出电平 控制 LED 亮灭
        system_delay_ms(300);
        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PIT 的中断处理函数 这个函数将在 PIT 对应的定时器中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     pit_hanlder();
//-------------------------------------------------------------------------------------------------------------------
void pit_hanlder (void)
{
    imu963ra_get_acc();                                                         // 获取 IMU963RA 的加速度测量数值
    imu963ra_get_gyro();                                                        // 获取 IMU963RA 的角速度测量数值
    imu963ra_get_mag();                                                         // 获取 IMU963RA 的地磁计测量数值
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
// 问题3：串口输出 IMU963RA init error.
//      检查 IMU963RA 的接线是否正确
//      检查 IMU963RA 的模块是不是坏了
//      给信号线加上拉看看
//
// 问题4：IMU963RA 数值异常
//      看看是不是线松了 或者信号线被短路了
//      可能模块部分受损

