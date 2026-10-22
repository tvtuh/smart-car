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
#include "image_process.h"                  // 图像二值化处理（见 code 文件夹）
#include "track_edge.h"                     // 种子生长法寻线（见 code 文件夹）
#include "cross_road.h"                     // 十字路口识别与补线（见 code 文件夹）
#include "ring_road.h"                      // 圆环识别（见 code 文件夹）

/* ================= 整车闭环控制挂钩（视觉算法一个字没动，只加下面这几处） ================= */
#include "car_config.h"                 /* 整车参数：引脚 / 速度 / 打角 / 自检开关 */
#include "car_control.h"                /* 整车闭环：读视觉的中线 -> 打角 + 速度 */
#include "car_motor.h"                 /* 只是为了拿 motor_tick_get()：链路自查要计时 */

// *************************** 例程硬件连接说明 ***************************
// 使用 type-c 连接核心板
//      直接将下载器正确连接在核心板的调试下载接口即可
//
// 接入 高速Wifi SPI 模块
//      模块管脚            单片机管脚
//      RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
//      INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
//      CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
//      MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
//      SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
//      MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
//      5V                  5V 电源
//      GND                 电源地
//
// 接入 总钻风摄像头
//      模块管脚            单片机管脚
//      TXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX        宏定义
//      RXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX        宏定义
//      D0                  查看 zf_device_mt9v03x.h 中 MT9V03X_D0_PIN             宏定义
//      D1                  查看 zf_device_mt9v03x.h 中 MT9V03X_D1_PIN             宏定义
//      D2                  查看 zf_device_mt9v03x.h 中 MT9V03X_D2_PIN             宏定义
//      D3                  查看 zf_device_mt9v03x.h 中 MT9V03X_D3_PIN             宏定义
//      D4                  查看 zf_device_mt9v03x.h 中 MT9V03X_D4_PIN             宏定义
//      D5                  查看 zf_device_mt9v03x.h 中 MT9V03X_D5_PIN             宏定义
//      D6                  查看 zf_device_mt9v03x.h 中 MT9V03X_D6_PIN             宏定义
//      D7                  查看 zf_device_mt9v03x.h 中 MT9V03X_D7_PIN             宏定义
//      PCLK                查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN           宏定义
//      VSYNC               查看 zf_device_mt9v03x.h 中 MT9V03X_VSY_PIN            宏定义
//      HSYNC               查看 zf_device_mt9v03x.h 中 MT9V03X_HERF_PIN           宏定义
//  ------------------------------------
//
// *************************** 例程使用步骤说明 ***************************
// 1.根据硬件连接说明连接好模块，使用电源供电(下载器供电会导致模块电压不足)
//
// 2.查看电脑所连接的wifi，记录wifi名称，密码，IP地址
//
// 3.在下方的代码区域中修改宏定义，WIFI_SSID_TEST为wifi名称，WIFI_PASSWORD_TEST为wifi密码
//
// 4.打开zf_device_wifi_spi.h，修改WIFI_SPI_TARGET_IP宏定义，设置为电脑wifi的IP地址
//
// 5.下载例程到单片机中，打开串口助手，打开下载器的串口
//
// 6.打开逐飞科技的逐飞助手软件，选择图像传输功能
//
// 7.选择网络，设置为TCP Server，本机地址中选择WIFI网络然后点击链接


// *************************** 例程测试说明 ***************************
// 1.本例程会通过 Debug 串口输出测试信息 请务必接好调试串口以便获取测试信息
//
// 2.连接好模块和核心板后（尽量使用配套主板测试以避免供电不足的问题） 烧录本例程 按下复位后程序开始运行
//
// 3.如果模块未能正常初始化 会通过 DEBUG 串口输出未能成功初始化的原因 随后程序会尝试重新初始化 一般情况下重试会成功
//
// 4.如果一直在 Debug 串口输出报错 就需要检查报错内容 并查看本文件下方的常见问题列表进行排查
//
// 5.程序默认不开启 WIFI_SPI_AUTO_CONNECT 宏定义 通过 main 函数中的接口建立网络链接 如果需要固定自行建立链接 可以开启该宏定义
//
// 6.当模块初始化完成后会通过 DEBUG 串口输出当前模块的主要信息：固件版本、IP信息、MAC信息、PORT信息
//
// 7.本例程是 TCP Client 例程 模块会被配置为 TCP Client 需要连接到局域网内的 TCP Server 才能进行通信
//   目标连接的 TCP Server 的 IP 与端口默认使用 zf_device_wifi_spi.h 中 WIFI_SPI_TARGET_IP 与 WIFI_SPI_TARGET_PORT 定义
//   实际测试需要根据现场 TCP Server 的实际 IP 地址与端口设置
//
// 8.当本机设备主动连接到 TCP Server （例如电脑使用逐飞助手上位机进入 TCP Server 模式 然后本机连接到电脑的 IP 与端口）
//   本例程会采集总钻风图像并发送到逐飞助手上位机
//
// 9.默认情况下逐飞助手显示摄像头的图像帧率可以达到50帧，如果无线网络比较复杂例如附近有较多的WIFI热点，可能会导致显示帧率较低
//
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************

//0：不包含边界信息
//1：包含三条边线信息，边线信息只包含横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点
//2：包含三条边线信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点，一般来说很少有这样的使用需求
//3：包含三条边线信息，边界信息含有横纵轴坐标，意味着你可以指定每个点的横纵坐标，边线的数量也可以大于或者小于图像的高度，通常来说边线数量大于图像的高度，一般是搜线算法能找出回弯的情况
//4：没有图像信息，仅包含三条边线信息，边线信息只包含横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点，这样的方式可以极大的降低传输的数据量
//-------------------------------------------------------------------------------------------------------------------
// 调试开关：要不要把图像发给上位机
//
//   1 = 正常发图像（当前）
//   0 = 【只发三条边线，不发图像】
//
// 为什么要这个开关：
//   发送图像是【阻塞】的 —— 22560 字节要花掉大半帧的时间，整个主循环都停在那里。
//   关掉图像之后，主循环就不再被发送卡住，**示波器曲线的推进速度 = 真实的循环频率**。
//   于是可以一次性判断瓶颈在哪：
//
//     把这里改成 0，看示波器曲线推进得快不快（x 轴数字涨得快不快）
//       - 涨得飞快（几十 fps） -> 瓶颈是"发图像"这个阻塞操作
//       - 还是 5~7 fps       -> 瓶颈在摄像头出帧 或 主循环处理，跟发送无关
//
//   注意：关掉之后上位机就不显示图像了，只剩三条边线，调完记得改回 1
//-------------------------------------------------------------------------------------------------------------------
#define CAMERA_SEND_IMAGE       1

//-------------------------------------------------------------------------------------------------------------------
// 调试开关：跳过全部"图像算法"，只保留 二值化 + 发图
//
//   0 = 正常运行（当前）
//   1 = 跳过 寻线 / 十字 / 圆环 / 后处理 / 贴边
//
// 用途：测"天花板帧率"。跳掉算法之后，主循环里只剩下
//         摄像头出帧 + 大津法 + 二值化 + 发图
//       这时候逐飞助手显示的 FPS 就是这套硬件的上限。
//
//   如果天花板很高(40+)，说明帧率被"算法"吃掉了 -> 继续优化算法
//   如果天花板也只有 17~18，说明卡在"摄像头出帧"或"无线链路" -> 跟算法无关，别再改算法
//
// 注意：跳掉算法之后画面上那三条线是上一帧的残留(或者乱线)，这是正常的，只看 FPS 就行
//-------------------------------------------------------------------------------------------------------------------
#define DEBUG_SKIP_ALGORITHM    0

//-------------------------------------------------------------------------------------------------------------------
// 调试开关：连"二值化"也一起跳过（要把 DEBUG_SKIP_ALGORITHM 也设成 1 才有意义）
//
//   0 = 正常
//   1 = 跳过 求阈值 + 二值化
//
// 用途：测"二值化到底占多少时间"。
//       DEBUG_SKIP_ALGORITHM=1 时已经跳掉了寻线/十字/圆环，剩下的是
//         摄像头出帧 + 大津法 + 二值化 + 发图
//       再把这里也设成 1，就只剩
//         摄像头出帧 + 发图
//       两次的 FPS 差就是"大津法 + 二值化"的耗时。
//
//   注意：跳掉二值化之后 image_copy 里是上一帧的旧数据（甚至是花的），这是正常的，只看 FPS
//-------------------------------------------------------------------------------------------------------------------
#define DEBUG_SKIP_BINARIZE     0

//-------------------------------------------------------------------------------------------------------------------
// 核心开关：算法直接用【灰度图】，省掉"二值化写一遍全图"那一趟
//
//   1 = 灰度模式（当前）
//       采完帧之后做一次 memcpy 快照，算法和上位机都用这张灰度图；
//       算法里判断"是不是白色"是拿像素和 image_threshold 比（等价于二值化，但不写那一趟）
//       PC 上显示的是【灰度图】（协议里 MT9V03X 和 GRAY 是同一个值，都是 1 字节/像素）
//
//   0 = 二值模式
//       采完帧之后跑 image_binarize 把灰度图二值化进快照，算法和上位机都用二值图
//
// 两种模式都用 image_copy 这张【静态快照】，区别只在"快照里存的是灰度还是黑白"。
//-------------------------------------------------------------------------------------------------------------------
#define IMAGE_USE_GRAY          1

//-------------------------------------------------------------------------------------------------------------------
// 摄像头降帧：解决"灰度直读"模式下的图像撕裂
//
//   1 = 初始化后把摄像头帧率降到 CAMERA_LOW_FPS_VALUE
//   0 = 保持库里的默认值(100fps)
//
// 为什么要降：
//   mt9v03x_image 是 DMA 直接写入的缓冲区，摄像头每出一帧就把它整体覆盖一遍。
//   我们处理+发送一张图大约 40ms。
//     100fps(10ms 一帧)：这 40ms 里被覆盖 4 次 -> 算法和上位机读到的都是
//                        "好几帧拼起来的图"，画面是横条噪声、寻线也找不到线
//     30fps(33ms 一帧)：基本只跨一帧，图像就是一致的
//   反正主循环也只跑 26 帧，降帧率不损失任何东西。
//
//   用的 MT9V03X_FPS 命令码(=3) 和库里初始化配置表用的是同一个，
//   所以不需要改库文件。
//
// 如果画面上还有轻微撕裂，把 CAMERA_LOW_FPS_VALUE 再降到 20。
//-------------------------------------------------------------------------------------------------------------------
#define CAMERA_LOW_FPS_ENABLE   0
#define CAMERA_LOW_FPS_VALUE    30

// 算法和发送统一都用 image_copy（它是"静态快照"）
//
// 【关键】不能直接读 mt9v03x_image！
//   mt9v03x_image 是 DMA 直接写入的，摄像头每出一帧就把它整体覆盖一遍。
//   我们处理+发送要 40ms，而摄像头 100fps(10ms 一帧) ->
//   直接读它的话，算法和上位机拿到的都是"好几帧拼起来的图"（画面是噪声、寻线找不到线）。
//   image_copy 是采完帧之后立刻做的一张【静态快照】，之后 DMA 再怎么覆盖都不影响它。
#define PROC_IMAGE          image_copy[0]

#define INCLUDE_BOUNDARY_TYPE   1                   // 1：发送二值化图像 + 种子生长法寻出的三条边线


// **************************** 图像二值化配置 ****************************
// 1：把二值化(黑白)图像发送到上位机显示   0：发送原始灰度图像
#define IMAGE_BINARIZE_ENABLE       (1)

// 阈值(二值化)算法选择
//  IMAGE_THRESHOLD_MODE_OTSU       大津法(OTSU)  全局阈值  每帧自动求阈值 抗光照变化
//  IMAGE_THRESHOLD_MODE_ITERATION  迭代法        全局阈值  每帧自动求阈值
//  IMAGE_THRESHOLD_MODE_MANUAL     手动固定阈值  固定值在 image_process.h 的 IMAGE_THRESHOLD_MANUAL（当前 150）
// 二值化阈值模式已经挪到 code\image_process.h 里定义（IMG_WHITE_LEVEL 需要在那之前就能看到它）

// 换场地/换光线需要重新标定阈值时，把 image_process.h 里的 IMAGE_MANUAL_COMPARE_AUTO 改回 1，
// 串口会同时打印「手动阈值」和「大津法自动算出的阈值」，照着 auto 的值改 IMAGE_THRESHOLD_MANUAL 即可
// 想微调自动阈值时改 image_process.h 中的 IMAGE_THRESHOLD_OFFSET（正数图像偏黑 负数图像偏白）

// 每隔多少帧通过 Debug 串口打印一次状态，方便观察（0 = 不打印）
#define IMAGE_THRESHOLD_PRINT_FRAME (0)                     // 【fw37】原来是 50：这是视觉组的调试打印，走 USB CDC，
                                                            //   而 USB 打印是【阻塞】的 —— 实测每 50 帧卡一次、一次 300ms+
                                                            //   （fw36 的 PF2 里 img 段就是这么被拖长的）。0 = 关掉。
                                                            //   要诊断阈值就把这里改回 50（帧率会掉几帧，属正常）

// 1：开启种子生长法寻线 0：不寻线（只发图像）
#define TRACK_EDGE_ENABLE           (1)

// 1：开启十字路口识别与补线 0：不判十字
// 注意：十字补线要在寻线之后做，所以 TRACK_EDGE_ENABLE 必须同时为 1
#define CROSS_ROAD_ENABLE           (1)

// 1：开启圆环识别（左右环都判） 0：不判圆环
#define RING_ROAD_ENABLE            (0)

// 1：把寻线/十字的诊断数据通过 WiFi 送到逐飞助手的虚拟示波器 0：不发送
// 和图像走同一条 WiFi 链路，每帧只多 36 字节，对帧率几乎没有影响
// 8 个通道的分配见 assistant_oscilloscope_send() 里的注释
#define ASSISTANT_OSC_ENABLE        (1)


#define WIFI_SSID_TEST          "YOUR_WIFI_SSID"
#define WIFI_PASSWORD_TEST      "YOUR_WIFI_PASSWORD" // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL


// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 3 / 2)

// 只有X边界
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];

// 只有Y边界
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y边界都是单独指定的
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];

// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 far image_copy[MT9V03X_H][MT9V03X_W];





#if(1 == ASSISTANT_OSC_ENABLE)
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把寻线/十字的诊断数据送到逐飞助手的虚拟示波器
// 返回参数     void
// 使用示例     assistant_oscilloscope_send();
// 备注信息     逐飞助手里打开"虚拟示波器"就能看到这 8 条曲线，和图像走同一条 WiFi 链路
//              8 个通道：
//                ch0 左线丢线行数        ch1 右线丢线行数
//                ch2 大津法自动阈值(和固定的手动阈值对比用)
//                ch3 十字状态位(0不在 1进入 2十字内 3出十字)
//                ch4 圆环 —— 识别到：类型*10+状态（11~14=左环状态1~4，21~24=右环状态1~4）
//                          没识别到：负数是失败原因，0=没跑；-2=入口侧道路没张开；
//                                    -3=对面也有长丢线段或对面也丢线(更像十字)；
//                                    -1x=对面不算连续直道(-11丢线多 -12点太少 -13拟合退化 -14不够直)
//                ch5/ch6 是共用的，看当前处在十字还是圆环：
//                  十字状态(ch3!=0)时：ch5 = 左上角点所在行，ch6 = 左下角点所在行（0 表示没找到）
//                                       —— 补线的行范围就是这两个行号之间，能直接看出补了哪一段
//                  否则：              ch5 = 圆环入口拐点行，ch6 = 圆环入口拐点列
//                                      （圆环没识别到时这两个通道改报圆环的诊断量，见上）
//                ch7 被丢线插值补上的行数
//              想换成别的量（例如边线坐标、角点坐标）直接改下面 8 行赋值即可
//-------------------------------------------------------------------------------------------------------------------
static void assistant_oscilloscope_send (void)
{
    seekfree_assistant_oscilloscope_data.dat[0] = (float)track_left_lost_count;
    seekfree_assistant_oscilloscope_data.dat[1] = (float)track_right_lost_count;
    seekfree_assistant_oscilloscope_data.dat[2] = (float)image_threshold_auto;
    seekfree_assistant_oscilloscope_data.dat[3] = (float)cross_state;
    // 识别到圆环就报 类型*10+状态；没识别到就报失败原因(负数)，方便定位是哪一条判据没过
    seekfree_assistant_oscilloscope_data.dat[4] = (0 != ring_state) ? (float)(ring_type * 10 + ring_state)
                                                                  : (float)ring_fail;

#if(1 == CROSS_ROAD_ENABLE)
    if(0 != cross_state)
    {
        // 十字激活时 ch5/ch6 报十字的角点行号（补线的行范围），优先看十字
        seekfree_assistant_oscilloscope_data.dat[5] = (float)cross_dbg_up_l;
        seekfree_assistant_oscilloscope_data.dat[6] = (float)cross_dbg_down_l;
    }
    else
#endif
    {
        // 没识别到时 ch5/ch6 改报诊断量，用来定阈值
        seekfree_assistant_oscilloscope_data.dat[5] = (0 != ring_state) ? (float)ring_corner_row
                                                                       : (float)ring_rep_dev;
        seekfree_assistant_oscilloscope_data.dat[6] = (0 != ring_state) ? (float)ring_corner_col
                                                                       : (float)ring_rep_lost;
    }

    seekfree_assistant_oscilloscope_data.dat[7] = (float)track_interp_count;

    seekfree_assistant_oscilloscope_data.channel_num = 8;                       // 发送 8 个通道

    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
#endif


/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     视觉链路重试计数：超过上限就返回 1（放弃这一步），让程序继续往主循环走
 * 参数说明     n       这一步的重试次数（调用方各自维护一个）
 *               what    这一步的名字，放弃时打印出来
 * 返回参数     0 = 继续重试    1 = 放弃这一步（调用方据此 break 并置 vision_ok = 0）
 * 备注信息     逐飞例程里 WiFi / TCP / 摄像头三步都是 while(失败){等100ms} 的死循环。
 *              只要其中任一步永远失败，程序就永远进不了主循环：
 *              舵机、电机、编码器、无线串口全都不会初始化 -> 串口助手上一片空白。
 *              这里改成最多重试 CAR_VISION_TRY_MAX 次，超了就跳过视觉链路，控制部分照常跑。
 *              这是**唯一**为此改动的三处循环，视觉算法（image_update_yellow_boundary）一个字没动。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 vision_retry_give_up(uint16 *n, const char *what, uint16 limit)
{
#if (CAR_VISION_TRY_MAX > 0)
    char msg[88];

    (*n)++;

    if(0 != car_wireless_skip_requested())          /* 开机等太久：串口助手里发个 x 立刻跳过视觉 */
    {
        car_wireless_progress("[CAR] vision skipped by user (x) -- control + wireless uart still run\r\n");
        return 1;
    }

    sprintf(msg, "[CAR] %s failed, retry %d/%d  (send x to skip vision now)\r\n",
            what, (int16)(*n), (int16)limit);
    car_wireless_progress(msg);

    if(*n >= limit)
    {
        printf("\r\n [CAR] %s keeps failing after %d tries -> skip vision chain, go to control loop", what, (int16)limit);
        car_wireless_progress("[CAR] vision give-up -> control loop + wireless uart still work\r\n");
        return 1;
    }
#else
    (void)n;
    (void)what;
#endif
    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     发一张图给上位机，并顺便做链路判定（fw42）
 * 返回参数     1 = 这次发得顺利；0 = 这次超时（>= CAR_TELEMETRY_SLOW_MS）
 * 备注信息     ★为什么要在发图【之前】续心跳：seekfree_assistant_camera_send() 是【阻塞】的，
 *              库里的 wifi_spi_wait_idle 一次最多等 1 秒，整张图最坏会阻塞 ~1.2 秒；
 *              而视觉心跳平时每帧才喂一次（上限 CAR_FRAME_TIMEOUT_MS = 500ms）——
 *              链路抖一下就会被看门狗判成"图像停了"把车锁死（实车现象：WiFi 一卡就 flt=1 不动了）。
 *              这里发图前把心跳一次性续到 3 倍，覆盖最坏阻塞；
 *              发图返回后主循环照常每帧喂，所以"视觉真的死了"仍然会在 500ms 后照常锁死。
 *-----------------------------------------------------------------------------------------------------------------*/
static uint8 car_send_image_once (void)
{
    uint16 t0 = motor_tick_get();

    motor_heartbeat_link_guard();               /* 【fw42】先续心跳，再发图（发图是阻塞的） */
    seekfree_assistant_camera_send();
    motor_heartbeat_link_guard();               /* 【fw42】先续心跳，再发图（发图是阻塞的） */
    seekfree_assistant_camera_send();

    {   /* 【fw49】第一张图发出去时在 USB 口喊一声：证明"链路通了 + 图真的在传" */
        static uint8 first_img_done = 0;

        if(0 == first_img_done)
        {
            first_img_done = 1;
            printf("\r\n [CAR] first image sent -> upload is running (assistant should show the picture)\r\n");
        }
    }


    if((uint16)(motor_tick_get() - t0) < (uint16)(CAR_TELEMETRY_SLOW_MS / 10))
    {
        return 1;
    }
    return 0;
}

void main (void)
{
#if(2 == INCLUDE_BOUNDARY_TYPE || 3 == INCLUDE_BOUNDARY_TYPE || 4 == INCLUDE_BOUNDARY_TYPE)
    int32 i = 0;                                                                // 类型 1 的边线来自种子生长法 不需要 i
#endif
#if(3 == INCLUDE_BOUNDARY_TYPE)
    int32 j = 0;
#endif
#if((1 == IMAGE_BINARIZE_ENABLE) && (0 != IMAGE_THRESHOLD_PRINT_FRAME))
    uint16 threshold_print_count = 0;                                           // 阈值打印计数器
#endif
    uint8  vision_ok  = 1;          /* 0 = 视觉链路没起来（WiFi/TCP/摄像头重试超限）：跳过它，照常进主循环 */
    uint16 vision_try = 0;          /* 重试计数：每一步各用一个 */
    uint16 telemetry_cnt = 0;       /* 图像上传抽帧用：每 CAR_TELEMETRY_EVERY 帧传一次 */
    uint8  link_slow_giveup = 0;    /* 1 = 判定链路卡住：不再每帧发图/示波器（数字照常走无线串口） */
    /* 【fw42】链路自查改成"窗口计数 + 自动试探恢复"（旧版要求【连续 3 次】超时，偶发卡顿永远抓不住） */
    uint8  link_win_n       = 0;    /* 最近发了几张图 */
    uint8  link_win_slow    = 0;    /* 其中发图超时的张数 */
    uint16 link_probe_tick  = 0;    /* 停发之后下一次"试发一张"的 tick */
    /* 【fw35 性能分解】帧时间统计用的三个变量：在 main() 里声明，主循环里一直有效 */
    uint16 perf_last_tick   = 0;    /* 上一帧"开始处理"那一刻的 tick */
    uint16 perf_last_inside = 0;    /* 上一帧"处理+发送"一共用了几个 tick */
    uint8  perf_first       = 1;    /* 1 = 还没统计过（第一帧没有上一帧，跳过） */
    clock_init(SYSTEM_CLOCK_96M); 				// 时钟配置及系统初始化<务必保留>
    debug_init();                       		// 调试串口信息初始化

    /* ★ 先把无线串口点亮（1 秒内的事）：下面连 WiFi / 连 TCP / 初始化摄像头这几步
     *   每一步失败都会重试，而库里超时很大（连热点 10s/次、连 TCP 50s/次）。
     *   如果等到 car_control_init() 才初始化无线串口，这段时间串口助手上会一片空白，
     *   看起来就像"模块坏了 / 程序没跑" —— 所以这里提前点亮，进度也一并报出去。
     *   开机阶段在串口助手里发 x 可以立刻跳过整条视觉链路。 */
    car_wireless_link_init();

    /* 台架自检时可以跳过整条视觉链路（WiFi + 逐飞助手 + 摄像头）：
     * 只插一根 USB 线就能看自检串口输出。见 car_config.h 的 CAR_BRINGUP_SKIP_VISION，
     * 它只在 CAR_BRINGUP_TEST = 1 时生效；正式跑车（=0）时这个 if 永远为假，视觉链路照常 */
    if(0 == CAR_SKIP_VISION)
    {
    /* 不用逐飞助手时（CAR_USE_WIFI_LINK = 0）不连 WiFi、不连 TCP：
     * 这样没开热点/没开上位机也能正常启动跑车，数据全看 USB 串口 */
    if(0 != CAR_USE_WIFI_LINK)
    {
    car_wireless_progress("[CAR] boot: joining wifi hot-spot (up to 10s per try) ...\r\n");
    /* 【fw49】同一句话也打一份到 USB 调试口：无线串口那对 dongle 一旦单向坏了，
     *   这几行就成了唯一能看到"开机走到哪一步"的地方（串口助手开 STC 的 USB CDC 口，115200）。 */
    printf("\r\n [CAR] %s boot: joining wifi hot-spot (%s) ...\r\n", CAR_FW_TAG, WIFI_SSID_TEST);
    vision_try = 0;
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
        if(0 != vision_retry_give_up(&vision_try, "wifi_spi_init", (uint16)CAR_VISION_TRY_MAX)) { vision_ok = 0; break; }
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接、创建 TCP 服务器等操作
    if(0 == CAR_WIFI_TCP_CONNECT)
    {
        car_wireless_progress("[CAR] TCP connect skipped (CAR_WIFI_TCP_CONNECT=0): numbers + control only, no image\r\n");
    }
            // ★只有"WiFi 链路开着 + 连上了 + 链路没卡死"才发。
            //   库里的 wifi_spi_wait_idle(OTHER_TIME_OUT = 1000ms) 一次最多等 1 秒：
            //   TCP 半死不活（上位机没开 / 网断了但连接还在）时，每次发送都要等满超时 ——
            //   那才是"帧率只有个位数、怎么改参数都不动"的真凶。
            if((0 != CAR_USE_WIFI_LINK) && (0 != vision_ok) && (0 == link_slow_giveup))
    {
        car_wireless_progress("[CAR] connecting TCP ... if IP/upper-computer is wrong this can take up to 50s\r\n");
        printf("\r\n [CAR] connecting TCP -> %s : %s ...\r\n", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT);
    }
    if((0 != vision_ok) && (0 != CAR_WIFI_TCP_CONNECT) && (1 != WIFI_SPI_AUTO_CONNECT))   // 没开自动连接就手动连目标 IP（WiFi 没连上、或没配好上位机就不试）
    {
        uint16 tcp_try = 0;
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            WIFI_SPI_TARGET_IP,                                                 // 指定远端的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,                                               // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))                                               // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
            if(0 != vision_retry_give_up(&tcp_try, "wifi_spi_socket_connect", (uint16)CAR_TCP_TRY_MAX)) { vision_ok = 0; break; }
        }
    }
        if(0 != vision_ok)
        {
            printf("\r\n [CAR] TCP connected OK -> %s : %s  (image upload starts)\r\n", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT);
        }
        else
        {
            printf("\r\n [CAR] TCP FAILED -> check: PC IPv4 == %s ? assistant listening on %s ? same hot-spot ?\r\n", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT);
        }

    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）
    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）
    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）


    }   /* ===== WiFi / TCP 结束（CAR_USE_WIFI_LINK） ===== */

    // 推荐先初始化摄像头，后初始化逐飞助手
    car_wireless_progress("[CAR] boot: init camera (mt9v03x) ...\r\n");
    vision_try = 0;
    while(mt9v03x_init())
    {
        system_delay_ms(100); 
		printf("\r\n mt9v03x_init error, try again.");
        if(0 != vision_retry_give_up(&vision_try, "mt9v03x_init", (uint16)CAR_VISION_TRY_MAX)) { vision_ok = 0; break; }
    }

#if(1 == CAMERA_LOW_FPS_ENABLE)
    // 【降低摄像头帧率】必须在 init 之后调用（init 里已经按库里的默认值配过一次了）
    //   MT9V03X_FPS 是模块的命令码(=3)，和初始化配置表里用的是同一个，
    //   所以这里等于把"帧率"那一项重新写一遍，不需要改库文件。
    mt9v03x_set_reg(MT9V03X_FPS, CAMERA_LOW_FPS_VALUE);
#endif

    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）
    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）
    // 使用高速 WIFI SPI模块时无法使用屏幕（因为引脚有共用）

    // 逐飞助手初始化 数据传输使用高速WIFI SPI
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);

    // 如果要发送图像信息，则务必调用seekfree_assistant_camera_information_config函数进行必要的参数设置
    // 如果需要发送边线则还需调用seekfree_assistant_camera_boundary_config函数设置边线的信息

#if(0 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(仅包含原始图像信息)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, PROC_IMAGE, MT9V03X_W, MT9V03X_H);


#elif(1 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 三条边线直接使用种子生长法寻线的结果：左边界 / 中线 / 右边界
    // track_left、track_center、track_right 由 track_edge_process() 每帧更新，
    // 这里只需要把数组地址交给上位机协议，发送时自动读取最新内容，不需要每帧重新配置
#if(0 != CAMERA_SEND_IMAGE)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, PROC_IMAGE, MT9V03X_W, MT9V03X_H);
#else
    // 图像地址传 NULL：协议层就不会发图像，只发下面的三条边线（120×3 = 360 字节）
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, MT9V03X_W, MT9V03X_H);
#endif
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, track_left, track_center, track_right, NULL, NULL ,NULL);


#elif(2 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点)
    // 通常很少有这样的使用需求
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_W; i++)
    {
        y1_boundary[i] = i * MT9V03X_H / MT9V03X_W;
        y2_boundary[i] = MT9V03X_H / 2;
        y3_boundary[i] = (MT9V03X_W - i) * MT9V03X_H / MT9V03X_W;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, PROC_IMAGE, MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, MT9V03X_W, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);


#elif(3 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息含有横纵轴坐标)
    // 这样的方式可以实现对于有回弯的边界显示
    j = 0;
    for(i = MT9V03X_H - 1; i >= MT9V03X_H / 2; i--)
    {
        // 直线部分
        xy_x1_boundary[j] = 34;
        xy_y1_boundary[j] = i;

        xy_x2_boundary[j] = 47;
        xy_y2_boundary[j] = i;

        xy_x3_boundary[j] = 60;
        xy_y3_boundary[j] = i;
        j++;
    }

    for(i = MT9V03X_H / 2 - 1; i >= 0; i--)
    {
        // 直线连接弯道部分
        xy_x1_boundary[j] = 34 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 34) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = i;

        xy_x2_boundary[j] = 47 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 47) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;

        xy_x3_boundary[j] = 60 + (MT9V03X_H / 2 - i) * (MT9V03X_W / 2 - 60) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }

    for(i = 0; i < MT9V03X_H / 2; i++)
    {
        // 回弯部分
        xy_x1_boundary[j] = MT9V03X_W / 2 + i * (138 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y1_boundary[j] = i;

        xy_x2_boundary[j] = MT9V03X_W / 2 + i * (133 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;

        xy_x3_boundary[j] = MT9V03X_W / 2 + i * (128 - MT9V03X_W / 2) / (MT9V03X_H / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, PROC_IMAGE, MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);


#elif(4 == INCLUDE_BOUNDARY_TYPE)
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 对边界数组写入数据
    for(i = 0; i < MT9V03X_H; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / MT9V03X_H;
        x2_boundary[i] = MT9V03X_W / 2;
        x3_boundary[i] = 118 + (168 - 118) * i / MT9V03X_H;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);


#endif



    }   /* ===== 视觉链路结束（CAR_SKIP_VISION） ===== */
    else
    {
        printf("\r\n step2~4 SKIPPED: CAR_BRINGUP_SKIP_VISION=1 (台架自检不需要视觉)");
        printf("\r\n [CAR] vision chain OFF (bringup): control + wireless uart still run");
    }

    /* ================= 整车控制初始化（必须在进入主循环之前） ================= */
    car_control_init();             /* 舵机回中位、电机保持静止、编码器与速度环就绪 */
    car_control_wait_start();       /* 起步门：按配置延时或等主板 KEY1，返回后才允许电机输出 */
    // 此处编写用户代码 例如外设初始化代码等
    while(1)
    {
        // 【性能修复】car_wireless_poll() 原来放在这里（if 外面）——
        //   主循环在等摄像头出帧的时候是"全速空转"的，每秒要转几十万圈，
        //   每一圈都调用一次这个函数（它里面有 char buf[192] 局部数组 + 排空串口），
        //   空转期间就把 CPU 吃光了 -> 实测帧率从 18fps 掉到 5.5fps。
        //   挪到下面"每帧一次"的区块里：18fps 时每秒仍被轮询 18 次，收命令完全够用。
        /* 无线串口：空转期间【限速】轮询（每 10ms 一次）。
         *   不能每圈都调：主循环等摄像头出帧时每秒空转几十万圈，每圈都调会把 CPU 吃光
         *     （实测帧率 18fps -> 5.5fps）；
         *   也不能完全不调：图像一停（掉线/卡死）串口就收不到命令也不上报了，没法用 o / ? 救车。
         *   所以折中：空转期间 100Hz 轮询，有帧时下面那一帧里再轮询一次。 */
        if(0 != car_wireless_poll_due()) { car_wireless_poll(); }

        if(mt9v03x_finish_flag)
        {
            /* 【fw35 性能分解】这一帧的时间花在哪：取 6 个时间点，帧尾交给 car_control.c 累加，
             *   无线串口的 PF 行里打印各段平均值。算法一个字没动，只是"顺手记个时间"。
             *   先全部赋成 t_f0：万一某一段被 #if 关掉了，它的耗时就是 0，不会读到没初始化的值 */
            uint16 t_f0;
            uint16 t_poll;
            uint16 t_alg;
            uint16 t_cp;
            uint16 t_vs;
            uint16 t_osc;
            uint16 t_img;
            uint16 t_end;
            uint16 dt_ticks;
            uint16 inside_ticks;
            uint16 wait_ticks;

            t_f0   = motor_tick_get();
            t_poll = t_f0;
            t_alg  = t_f0;
            t_cp   = t_f0;
            t_vs   = t_f0;
            t_osc  = t_f0;
            t_img  = t_f0;
            mt9v03x_finish_flag = 0;

            car_wireless_poll();        // 【性能修复】挪到这里：每帧只调用一次
            t_poll = motor_tick_get();      /* 【fw35】无线串口轮询 + 定时上报到这里结束 */

#if(1 == IMAGE_BINARIZE_ENABLE)
#if(0 == DEBUG_SKIP_BINARIZE)                                       // ==1 时连二值化也跳过（只为测耗时）
            // 1.求赛道与背景之间的全局跳变阈值（迭代法 / 大津法）
            image_threshold_calc(IMAGE_THRESHOLD_MODE);

            // 2.按阈值把当前帧二值化后写入发送缓冲区
            //   这里同时完成了原来的图像备份工作，不需要额外申请图像数组，也就不会多占用内存
#if(1 == IMAGE_USE_GRAY)
            // 【灰度模式】只做一次快照，不做二值化。
            //   memcpy 是"读一次写一次"，二值化是"读一次+比较+写一次"，
            //   所以灰度模式比二值化模式还略快一点，而且上位机看到的是灰度图。
            //   必须紧跟在"刚采完一帧"之后做，这时候 mt9v03x_image 才是有效数据。
            image_gray_copy(image_copy[0]);
#else
            image_binarize(image_copy[0]);                          // 二值模式：把灰度图二值化后写进快照
#endif
#endif                                                              // DEBUG_SKIP_BINARIZE

#if(0 == DEBUG_SKIP_ALGORITHM)                                      // ==1 时跳过下面全部算法，只留二值化+发图
#if(1 == TRACK_EDGE_ENABLE)
            // 3.种子生长法寻线：在二值化图像上提取左右边线和中线
            //   结果放在 track_left / track_center / track_right 中，随图像一起发到上位机显示
            t_cp = motor_tick_get();        /* 【fw36】灰度快照（含阈值计算）到这里结束 */
            track_edge_process(PROC_IMAGE);

#if(1 == CROSS_ROAD_ENABLE)
            // 4.十字路口识别与补线：识别到十字会把断掉的边线补上 并刷新中线
            cross_road_process(PROC_IMAGE);
#endif

#if(1 == RING_ROAD_ENABLE)
            // 4.5 圆环识别（左右环都判）：识别结果放在 ring_flag / ring_type / ring_state
            /* 【fw47d】弯道里【不叫】它 —— 见 car_control.c 的 car_ring_module_ok()：
             *   它的判据"一侧连续丢线 >= 25 行 + 对面直"在弯道里（外侧线跑出画面）也会成立，
             *   一叫它就误报圆环，而且它的 ring_fix_side 会把中线往弯外推（车冲出弯道）。
             *   跳过时顺手把它的状态清零，免得出了弯还用弯里那次的判断去补线。 */
            if(0 != car_ring_module_ok())
            {
                ring_road_process(PROC_IMAGE);
            }
            else
            {
                ring_flag  = 0;
                ring_type  = RING_TYPE_NONE;
                ring_state = RING_STATE_NONE;
            }
#else
            ring_flag       = 0;
            ring_type       = RING_TYPE_NONE;
            ring_state      = RING_STATE_NONE;
            ring_corner_row = 0;
            ring_corner_col = 0;
#endif

            // 5.边线后处理：把丢线的行用两端有效值插值补上 + 去掉单行毛刺 + 重算中线
            //   必须在十字补线之后（十字补线靠原始丢线标志决定补哪些行）
            track_line_smooth();

            // 6.丢线贴边：连续丢线很多行、而且已经贴近画面边缘的，说明边线跑出画面了，
            //   整段贴到画面边上，而不是让它飘在画面中间（放在平滑之后做，覆盖插值结果）
            track_pin_lost_edges();

            /* ★ 整车闭环挂钩：一帧控制（读 track_center / track_lost -> 打角 + 速度 -> 舵机 + 电机）
             *   位置很关键：在所有补线/平滑/贴边之后（拿到的中线才是最终结果），
             *   并且在发图之前（控制延迟最小）。白区约束在控制计算前执行。 */
#if(1 == TRACK_TEMPORAL_ENABLE)
            /* 【fw44】视觉组 v7 新增：帧间稳定（左右边线"3 帧时间中值"）
             *   治"线本身平滑、但位置一帧一个样"的整体左右晃；顺带把孤立一帧的坏数据（例如整帧 0/187）扔掉。
             *   必须在所有寻线 / 补线 / 贴边之后调用 —— 他们 v7 的 main.c 里就是这么放的。
             *   开关在 code\track_edge.h 的 TRACK_TEMPORAL_ENABLE（他们默认 0 = 关） */
            track_temporal_filter();
#endif
            track_white_lane_guard(PROC_IMAGE);
            t_vs = motor_tick_get();        /* 【fw36】视觉算法（寻线/十字/圆环/平滑/贴边）到这里结束 */
            car_control_update();
#endif
#endif                                                              // DEBUG_SKIP_ALGORITHM
            t_alg = motor_tick_get();       /* 【fw36】整车控制到这里结束（alg = cp + vs + ct） */

#if(1 == ASSISTANT_OSC_ENABLE)
            // 6.把诊断数据送到逐飞助手的虚拟示波器（和图像同一条 WiFi 链路）
            // 只有"图像链路开着 + 视觉链路真的起来了 + 链路没被判卡死"才发：
            //   ★fw35 修复：原来这里只判断了 vision_ok，所以你把 CAR_USE_WIFI_LINK 改成 0 之后
            //     示波器这几十个字节还在每帧往外发 —— 你想"关掉链路测真实帧率"时它照样占时间。
            //     现在把链路开关也一起判断，CAR_USE_WIFI_LINK = 0 就是真的一点都不发。
            if((0 != CAR_USE_WIFI_LINK) && (0 != vision_ok) && (0 == link_slow_giveup))
            {
                assistant_oscilloscope_send();
            }
#endif
            t_osc = motor_tick_get();       /* 【fw35】示波器发送到这里结束 */

#if(0 != IMAGE_THRESHOLD_PRINT_FRAME)
            // 每隔 IMAGE_THRESHOLD_PRINT_FRAME 帧打印一次状态 方便观察
            if(0 == (threshold_print_count++ % IMAGE_THRESHOLD_PRINT_FRAME))
            {
#if(1 == IMAGE_MANUAL_COMPARE_AUTO)
                // 手动固定阈值模式：同时打印手动值和大津法自动算出的值，方便对比
                printf("\r\n th:%d auto:%d ref:%d near:%d lost L:%d R:%d",
                        image_threshold, image_threshold_auto,
                        image_near_ref, image_threshold_near,
                        track_left_lost_count, track_right_lost_count);
#else
                // 打印状态：阈值 / 起始行左右边界 / 丢线行数 / 十字标志与状态 / 四个L角点
                printf("\r\n th:%d L:%d R:%d lost:%d,%d cross:%d,%d C:%d,%d",
                        image_threshold,
                        track_left[TRACK_START_ROW], track_right[TRACK_START_ROW],
                        track_left_lost_count, track_right_lost_count,
                        cross_flag, cross_state,
                        cross_dbg_up_l, cross_dbg_down_l);
#endif
            }
#endif
#else
            // 在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
            image_gray_copy(image_copy[0]);
#endif

#if(1 == TRACK_DEBUG_MARK)
            // 7.调试用：把丢线的行在图像上画一段黑横条，一眼就能看出寻线丢在哪几行
            //   它是在所有寻线/补线处理之后才画的，不影响任何计算结果
            //   跑车时把 track_edge.h 里的 TRACK_DEBUG_MARK 改成 0 关掉
            track_lost_mark(image_copy[0]);
#endif

            /* 只有"WiFi + TCP 都连上了"（vision_ok = 1）才上传图像：
             * 没连上时发送会去等模块空闲，而库里的 wifi_spi_wait_idle(OTHER_TIME_OUT) 一次最多等 1 秒，
             * 会把主循环拖到几秒一帧；而且主循环一慢，"视觉心跳看门狗"（500ms）还会把电机锁死。
             * CAR_TELEMETRY_EVERY 控制每几帧上传一次（默认 3）—— 发图是阻塞的，抽帧后控制频率就不会被拖垮。 */
            if((0 != CAR_USE_WIFI_LINK) && (0 != vision_ok) && (0 != car_img_send_ok()))   /* 【fw48】! 可以临时停发图像 */
            {
                if(0 == link_slow_giveup)
                {
                    telemetry_cnt++;
                    if(telemetry_cnt >= (uint16)CAR_TELEMETRY_EVERY)
                    {
                        telemetry_cnt = 0;
                        if(0 == car_send_image_once())              /* 这一次发图超时了 */
                        {
                            if(link_win_slow < 250) { link_win_slow++; }
                        }
                        link_win_n++;

                        /* 【fw42】最近 CAR_LINK_WIN_SENDS 张图里有 CAR_LINK_WIN_MAX 张超时 -> 判定链路卡住：
                         *   立刻停发图像和示波器（数字照常走无线串口），之后每 CAR_LINK_PROBE_MS 毫秒试发一张，
                         *   顺利就自动恢复。这样"链路偶发抖动"不会被"卡着继续发"放大成 1 秒级卡顿。 */
                        if(link_win_n >= (uint8)CAR_LINK_WIN_SENDS)
                        {
                            if(link_win_slow >= (uint8)CAR_LINK_WIN_MAX)
                            {
                                link_slow_giveup = 1;
                                link_probe_tick  = motor_tick_get();
                                car_wireless_progress("WV WARN: wifi link stalled -> image + oscilloscope STOPPED (auto-retry every 5s), numbers still run\r\n");
                            }
                            link_win_n    = 0;
                            link_win_slow = 0;
                        }
                    }
                }
                else
                {
                    /* 停发期间：每 CAR_LINK_PROBE_MS 毫秒【试发一张】，顺利就自动恢复发图 */
                    if((uint16)(motor_tick_get() - link_probe_tick) >= (uint16)(CAR_LINK_PROBE_MS / 10))
                    {
                        link_probe_tick = motor_tick_get();
                        if(0 != car_send_image_once())
                        {
                            link_slow_giveup = 0;
                            link_win_n       = 0;
                            link_win_slow    = 0;
                            car_wireless_progress("WV: wifi link recovered -> image + oscilloscope RESUMED\r\n");
                        }
                    }
                }
            }
            // 如果使用UDP协议传输数据则推荐在数据全部发送到模块之后立即调用wifi_spi_udp_send_now()函数，以告知模块立即将收到的数据发送到网络上
            // 如果没有立即调用则模块会在持续2毫秒未收到数据后，将数据发送到网络上
            // 调用wifi_spi_udp_send_now()前传输给模块的数据数量建议不要超过40960字节
            // wifi_spi_udp_send_now();
            /* 【fw36 性能分解】帧尾结算：算出"等摄像头 / 轮询上报 / 快照 cp / 视觉算法 vs / 控制 ct / 示波器 / 发图"各花了多少。
             *   全部用 16 位无符号差值（10ms 计数器），回绕也是对的：
             *     dt   = 本帧开始 - 上帧开始 = 上帧的处理时间 + 中间空转等摄像头的时间
             *     wait = dt - 上帧的处理时间  <- 它大 = 帧率被摄像头限制；它小 = 帧率被我们的代码限制
             *   第一帧没有上一帧，只记录不统计 */
            t_img = motor_tick_get();       /* 【fw35】图像上传到这里结束（没上传的帧这俩 tick 相等） */
            t_end        = motor_tick_get();
            inside_ticks = (uint16)(t_end - t_f0);              /* 这一帧"处理+发送"一共用掉的 tick */
            dt_ticks     = (uint16)(t_f0 - perf_last_tick);     /* 和上一帧开始的间隔 = inside + 空转等待 */
            wait_ticks   = (uint16)(dt_ticks - perf_last_inside);
            if(0 == perf_first)                                 /* 第一帧没有"上一帧"，只记录不统计 */
            {
                car_perf_frame(dt_ticks, wait_ticks,
                               (uint16)(t_poll - t_f0), (uint16)(t_cp - t_poll),
                               (uint16)(t_vs - t_cp), (uint16)(t_alg - t_vs),
                               (uint16)(t_osc - t_alg), (uint16)(t_img - t_osc));
            }
            else
            {
                perf_first = 0;
            }
            perf_last_tick   = t_f0;
            perf_last_inside = inside_ticks;
        }
        // 此处编写需要循环执行的代码
    }
}
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 问题1：串口没有数据
//      查看串口助手打开的是否是正确的串口 检查打开的 COM 口是否对应的是 Debug UART(type-c的USB-CDC) 的 COM 口
//
// 问题2：串口数据乱码
//      查看串口助手使用的波特率是否是 zf_common_debug.h 中 DEBUG_UART_BAUDRATE 宏定义的一致 默认是 115200
//
// 问题3：上位机收不到数据或者一直显示Connect TCP Servers error, try again.
//      检查默认使用的IP 地址（zf_device_wifi_spi.h 中 WIFI_SPI_TARGET_IP）是否与上位机的本机地址一致
//      检查默认使用的通信端口（zf_device_wifi_spi.h 中 WIFI_SPI_TARGET_PORT）是否与上位机的端口号一致
//
// 问题4：一直显示 connect wifi failed
//      将 WIFI_SSID_TEST 宏定义修改为你测试使用的 Wi-Fi 名称
//      将 WIFI_PASSWORD_TEST 宏定义修改为你测试使用的 Wi-Fi 密码


