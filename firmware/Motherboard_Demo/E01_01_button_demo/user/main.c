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

// *************************** 例程测试说明 ***************************
// 1.核心板插在主板上 主板使用电池供电 下载本例程
//
// 2.复位核心板 对应的屏幕就会显示按键值
//
// 3.任意按一个按键，屏幕上面显示的值就会加一
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

#include "zf_common_headfile.h"

// 定义按键引脚
#define KEY1_PIN        IO_PB2
#define KEY2_PIN        IO_PB3
#define KEY3_PIN        IO_PB4
#define KEY4_PIN        IO_P32

#define SWITCH1_PIN     IO_PB0
#define SWITCH2_PIN     IO_PB1

// 开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

// 上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

// 开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

uint8 test1=0,test2=0,test3=0,test4=0;

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

	gpio_init(KEY1_PIN, GPI, 1, GPI_PULL_UP);
	gpio_init(KEY2_PIN, GPI, 1, GPI_PULL_UP);
	gpio_init(KEY3_PIN, GPI, 1, GPI_PULL_UP);
	gpio_init(KEY4_PIN, GPI, 1, GPI_PULL_UP);

    gpio_init(SWITCH1_PIN, GPI, 1, GPI_PULL_UP);
    gpio_init(SWITCH2_PIN, GPI, 1, GPI_PULL_UP);
    
	
//	oled_init();
//	tft180_init();
//  ips114_init();
	ips200_init();

    // 此处编写用户代码 例如外设初始化代码等

    while(1)
    {
		// 此处编写需要循环执行的代码

        // 使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
        // 保存按键状态
        key1_last_status = key1_status;
        key2_last_status = key2_status;
        key3_last_status = key3_status;
        key4_last_status = key4_status;

		// 读取当前按键状态
        key1_status = gpio_get_level(KEY1_PIN);
        key2_status = gpio_get_level(KEY2_PIN);
        key3_status = gpio_get_level(KEY3_PIN);
        key4_status = gpio_get_level(KEY4_PIN);
        
        //检测到按键按下之后  并放开置位标志位
        if(key1_status && !key1_last_status)    key1_flag = 1;
        if(key2_status && !key2_last_status)    key2_flag = 1;
        if(key3_status && !key3_last_status)    key3_flag = 1;
        if(key4_status && !key4_last_status)    key4_flag = 1;
        
        //标志位置位之后，可以使用标志位执行自己想要做的事件
        if(key1_flag)   
        {
            key1_flag = 0;//使用按键之后，应该清除标志位
            test1++;
        }
        
        if(key2_flag)   
        {
            key2_flag = 0;//使用按键之后，应该清除标志位
            test2++;
        }
        
        if(key3_flag)   
        {
            key3_flag = 0;//使用按键之后，应该清除标志位
            test3++;
        }
        
        if(key4_flag)   
        {
            key4_flag = 0;//使用按键之后，应该清除标志位
            test4++;
        }
        
//        // 在OLED屏幕上显示测试变量，需要初始化OLED屏幕，才能使用。
//        oled_show_string(0 , 0, "KEY1 TEST:");   oled_show_int(11*8, 0, test1, 3);
//        oled_show_string(0 , 1, "KEY2 TEST:");   oled_show_int(11*8, 1, test2, 3);
//        oled_show_string(0 , 2, "KEY3 TEST:");   oled_show_int(11*8, 2, test3, 3);
//        oled_show_string(0 , 3, "KEY4 TEST:");   oled_show_int(11*8, 3, test4, 3);



//        // 在1.8寸TFT屏幕上显示测试变量，需要初始化1.8寸TFT屏幕，才能使用。
//        tft180_show_string(0 , 0*16, "KEY1 TEST:");   tft180_show_uint8(11*8, 0*16, test1);
//        tft180_show_string(0 , 1*16, "KEY2 TEST:");   tft180_show_uint8(11*8, 1*16, test2);
//        tft180_show_string(0 , 2*16, "KEY3 TEST:");   tft180_show_uint8(11*8, 2*16, test3);
//        tft180_show_string(0 , 3*16, "KEY4 TEST:");   tft180_show_uint8(11*8, 3*16, test4);

		
		
//      // 在1.14寸IPS屏幕上显示测试变量，需要初始化1.14寸IPS屏幕，才能使用。
//      ips114_show_string(0 , 0*16, "KEY1 TEST:");   ips114_show_uint8(11*8, 0*16, test1);
//      ips114_show_string(0 , 1*16, "KEY2 TEST:");   ips114_show_uint8(11*8, 1*16, test2);
//      ips114_show_string(0 , 2*16, "KEY3 TEST:");   ips114_show_uint8(11*8, 2*16, test3);
//      ips114_show_string(0 , 3*16, "KEY4 TEST:");   ips114_show_uint8(11*8, 3*16, test4);
		
		
       // 在2寸IPS屏幕上显示测试变量，需要初始化2寸IPS屏幕，才能使用。
       ips200_show_string(0 , 0*16, "KEY1 TEST:");   ips200_show_uint8(11*8, 0*16, test1);
       ips200_show_string(0 , 1*16, "KEY2 TEST:");   ips200_show_uint8(11*8, 1*16, test2);
       ips200_show_string(0 , 2*16, "KEY3 TEST:");   ips200_show_uint8(11*8, 2*16, test3);
       ips200_show_string(0 , 3*16, "KEY4 TEST:");   ips200_show_uint8(11*8, 3*16, test4);

       ips200_show_string(0 , 4*16, "SWITCH1 TEST:");   ips200_show_uint8(11*10, 4*16, gpio_get_level(SWITCH1_PIN));
       ips200_show_string(0 , 5*16, "SWITCH2 TEST:");   ips200_show_uint8(11*10, 5*16, gpio_get_level(SWITCH2_PIN));
              
        // 此处编写需要循环执行的代码
    }
}

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
//
// 问题1：S1-S4 按下 屏幕数值不增加
//      如果使用主板测试，主板必须要用电池供电
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
//      万用表检查对应 S1-S4 引脚电压是否正常变化，是否跟接入信号不符，引脚是否接错
//
// 问题2：屏幕不亮或者屏幕显示有问题
//		查看屏幕初始化是否正确
// 		查看屏幕是否插好
