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

#ifndef _zf_common_debug_h_
#define _zf_common_debug_h_

#include "zf_common_typedef.h"
#include "zf_common_fifo.h"

#define PRINTF_ENABLE               (1)                                             // 使能printf
#define USER_USB_CDC                (1)

// 默认使用USB_CDC进行通信
#if(USER_USB_CDC == 0)
    #define DEBUG_UART_INDEX            (UART_1)                               	    // 指定 debug uart 所使用的的串口
    #define DEBUG_UART_BAUDRATE         (115200)                				    // 指定 debug uart 所使用的的串口波特率
    #define DEBUG_UART_TX_PIN           (UART1_TX_P37 )                        	    // 指定 debug uart 所使用的的串口引脚
    #define DEBUG_UART_RX_PIN           (UART1_RX_P36 )                        	    // 指定 debug uart 所使用的的串口引脚
#endif

#define DEBUG_UART_USE_INTERRUPT    (1)                                             // 是否启用 debug uart 接收中断

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     断言
// 参数说明     x           判断是否触发断言 0-触发断言 1-不触发断言
// 返回参数     void
// 使用示例     zf_assert(0);
// 备注信息     一般用于参数判断 zf_assert(0) 就断言报错
//              默认情况下会在 Debug UART 输出
//              但如果使用开源库内屏幕接口初始化了屏幕 则会在屏幕上显示
//-------------------------------------------------------------------------------------------------------------------
#define zf_assert(x)                (debug_assert_handler((x), __FILE__, __LINE__))

uint32      debug_send_buffer(const uint8 *buff, uint32 len);

#if DEBUG_UART_USE_INTERRUPT                        // 如果启用 debug uart 接收中断
    #define     DEBUG_RING_BUFFER_LEN   (128)        // 定义环形缓冲区大小 默认 128byte
#endif

extern fifo_struct  debug_uart_fifo;

uint32      debug_read_buffer(uint8 *buff, uint32 len);
void        debug_assert_handler        (uint8 pass, char *file, int line);
void        debug_init                  (void);

#endif