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

// *************************** 例程硬件连接说明 ***************************
// 接入两寸ips200pro屏幕
//      模块管脚            单片机管脚
//      CLK                 查看 zf_device_ips200pro.h 中 IPS200PRO_CLK_PIN  宏定义
//      MOSI                查看 zf_device_ips200pro.h 中 IPS200PRO_MOSI_PIN 宏定义
//      MISO                查看 zf_device_ips200pro.h 中 IPS200PRO_MISO_PIN 宏定义
//      RST                 查看 zf_device_ips200pro.h 中 IPS200PRO_RST_PIN  宏定义
//      INT                 查看 zf_device_ips200pro.h 中 IPS200PRO_INT_PIN  宏定义
//      CS                  查看 zf_device_ips200pro.h 中 IPS200PRO_CS_PIN   宏定义
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

// *************************** 例程测试说明 ***************************
// 1.该例程为IPS200PRO显示进度条控件控制示例
// 2.核心板烧录本例程 插在主板上
//   2寸IPSPRO 显示模块插在主板的屏幕接口排座上
//   请注意引脚对应 不要插错
// 2.电池供电 上电后 2寸IPS 屏幕亮起 并且底部显示“Progress Bar测试”
// 3.主页面将显示五个不同方式的进度条控件，并且展示进度条变化效果
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

#include "zf_common_headfile.h"

// 保存屏幕的栈大小，
uint32 ips200pro_stack_size;

// 保存创建的页面ID，当需要对页面进行操作的时候则需要填写此ID
uint16 page_id[3];

uint16 progress_bar_id[5];
        
uint8 i;

void main(void)
{
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化
	
    // 此处编写用户代码 例如外设初始化代码等

    
    // 初始化屏幕并创建一个页面
    page_id[0] = ips200pro_init("Progress Bar测试", IPS200PRO_TITLE_BOTTOM, 30);

    // 获取屏幕信息，根据个人需求调用
    ips200pro_get_information(&ips200pro_information);
    ips200pro_get_free_stack_size(&ips200pro_stack_size);
    ips200pro_get_time(&ips200pro_time);

    // 设置全局字体为16号
    ips200pro_set_default_font(FONT_SIZE_16);

    // 屏幕设置为竖屏
    ips200pro_set_direction(IPS200PRO_PORTRAIT);

    // 设置编码格式
    ips200pro_set_format(IPS200PRO_FORMAT_GBK);

    // 设置选中页面的标题背景颜色，填写任意一个创建完成的页面ID即可
    ips200pro_set_color(page_id[0], COLOR_PAGE_SELECTED_BG, IPS200PRO_RGB888_TO_RGB565(0xFF, 0xE3, 0xE3));

    // 设置第一个页面的背景色
    ips200pro_set_color(page_id[0], COLOR_BACKGROUND, IPS200PRO_RGB888_TO_RGB565(0x43, 0x4A, 0x61));

    // 设置页面标题文字颜色，填写任意一个创建完成的页面ID即可
    ips200pro_set_color(page_id[0], COLOR_FOREGROUND, RGB565_BLACK);

    // 设置页面选中后的标题文字颜色，填写任意一个创建完成的页面ID即可
    ips200pro_set_color(page_id[0], COLOR_PAGE_SELECTED_TEXT, RGB565_WHITE);

    // 设置屏幕背光亮度，可以设置范围0-255
    ips200pro_set_backlight(255);

    // 切换页面，切换到第一个页面
    ips200pro_page_switch(page_id[0] ,PAGE_ANIM_ON);

    // 创建一个长条形进度条
    progress_bar_id[0] = ips200pro_progress_bar_create(20, 10, 200, 20);

    progress_bar_id[1] = ips200pro_progress_bar_create(20, 50, 200, 20);

    // 创建一个圆形进度条，当宽度与高度相等时进度条为圆形
    progress_bar_id[2] = ips200pro_progress_bar_create(30, 80, 60, 60);

    progress_bar_id[3] = ips200pro_progress_bar_create(160, 80, 60, 60);

    // 创建一个纵向进度条
    progress_bar_id[4] = ips200pro_progress_bar_create(110, 80, 20, 200);

    // 修改进度条颜色
    ips200pro_set_color(progress_bar_id[0], COLOR_FOREGROUND, IPS200PRO_RGB888_TO_RGB565(255, 150, 78));
    // 设置进度条背景颜色
    ips200pro_set_color(progress_bar_id[0], COLOR_BACKGROUND, RGB565_WHITE);

    // 修改进度条颜色
    ips200pro_set_color(progress_bar_id[1], COLOR_FOREGROUND, IPS200PRO_RGB888_TO_RGB565(255, 150, 78));
    // 设置进度条背景颜色
    ips200pro_set_color(progress_bar_id[1], COLOR_BACKGROUND, RGB565_WHITE);

    // 修改进度条颜色
    ips200pro_set_color(progress_bar_id[2], COLOR_FOREGROUND, IPS200PRO_RGB888_TO_RGB565(255, 150, 78));
    // 设置进度条背景颜色
    ips200pro_set_color(progress_bar_id[2], COLOR_BACKGROUND, RGB565_WHITE);

    // 修改进度条颜色
    ips200pro_set_color(progress_bar_id[3], COLOR_FOREGROUND, IPS200PRO_RGB888_TO_RGB565(255, 150, 78));
    // 设置进度条背景颜色
    ips200pro_set_color(progress_bar_id[3], COLOR_BACKGROUND, RGB565_WHITE);

    // 修改进度条颜色
    ips200pro_set_color(progress_bar_id[4], COLOR_FOREGROUND, IPS200PRO_RGB888_TO_RGB565(255, 150, 78));
    // 设置进度条背景颜色
    ips200pro_set_color(progress_bar_id[4], COLOR_BACKGROUND, RGB565_WHITE);
    
    
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        // 此处编写需要循环执行的代码

        for(i=0; 90 > i; i++)
        {
            // 起始值固定为0仅修改结束值
            ips200pro_progress_bar_set_value(progress_bar_id[0], 0, i);
            ips200pro_progress_bar_set_value(progress_bar_id[2], 0, i);
            ips200pro_progress_bar_set_value(progress_bar_id[4], 0, i);
            // 同时修改起始值与结束值，从而实现类似水平仪的效果
            ips200pro_progress_bar_set_value(progress_bar_id[1], i, i + 10);
            ips200pro_progress_bar_set_value(progress_bar_id[3], i, i + 10);
            system_delay_ms(20);
        }

        // 反向
        for(i=100; 10 <= i; i--)
        {
            // 进度条数值设置
            ips200pro_progress_bar_set_value(progress_bar_id[0], i, 100);
            ips200pro_progress_bar_set_value(progress_bar_id[2], i, 100);
            ips200pro_progress_bar_set_value(progress_bar_id[4], i, 100);

            ips200pro_progress_bar_set_value(progress_bar_id[1], i - 10, i);
            ips200pro_progress_bar_set_value(progress_bar_id[3], i - 10, i);

            system_delay_ms(20);
        }
      
      
        // 此处编写需要循环执行的代码
    }
}

// **************************** 代码区域 ****************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：屏幕不显示
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕型号是否与例程所使用的型号对应
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控