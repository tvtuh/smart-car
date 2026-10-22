/*********************************************************************************************************************
* 文件名称          image_process.c
* 公司名称          成都逐飞科技有限公司
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
*
* 功能简介          赛道图像二值化处理
*                  算法参考：智能车图像处理(一)阈值处理
*                  https://blog.csdn.net/weixin_50567482/article/details/119322251
*
* 修改记录
* 日期              作者           备注
* 2026-xx-xx        user           first version
********************************************************************************************************************/

#include "image_process.h"


// 当前使用的二值化阈值 求阈值函数会更新本变量
uint8 image_threshold = IMAGE_ITERATION_INIT;

// 手动模式下顺便算出的大津法阈值 仅用于和手动阈值对比观察 不参与二值化
uint8 image_threshold_auto = 0;
uint8 image_threshold_near = 0;                                    // 近端参考白点算出来的阈值
uint8 image_near_ref       = 0;                                    // 近端参考白点本身(取样区灰度均值)
static uint16 image_auto_tick = 0;                                  // 大津法隔帧计数(性能优化用)


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     迭代法求全局阈值
// 对应文章     智能车图像处理(一)阈值处理 —— (一)迭代法求阈值
// 基本思路     1.为全局阈值 T 选择一个初始估计值 BlackThres
//              2.用 T 分割该图像 这将产生两组像素 G0 由灰度值小于 T 的所有像素组成 G1 由灰度值大于等于 T 的像素组成
//              3.对 G0 和 G1 的像素分别计算平均灰度值(均值) m0 和 m1
//              4.计算一个新的阈值 T = 1/2 (m0 + m1)
//              5.重复步骤 2 到步骤 4 直到连续迭代中的 T 值间的差小于一个预定义的参数 ΔT 为止
// 返回参数     uint8               得到的全局阈值
// 备注信息     本函数与参考文章代码有三处差异 都是为了实际跑车时更稳定：
//              1.参考文章中 S0/S1/N0/N1 只在函数开头清了一次零 每轮迭代会把上一轮的数据继续累加进去
//                这里改成每轮迭代开始前重新统计
//              2.增加了 N0 或者 N1 为 0 时的除零保护(整幅图像全黑或者全白时会出现)
//              3.增加了最大迭代次数限制 避免极端图像在 while 中出不来
//              4.初始阈值改用整幅图像的平均灰度 文章中也写到"图像的平均灰度对于 T 来说是较好的初始选择"
//                因为固定的初值(例如 58)在整幅图像都偏亮时会使得 G0 一个像素都没有 阈值直接退化不再迭代
//-------------------------------------------------------------------------------------------------------------------
uint8 image_threshold_iteration (void)
{
    uint8  *row = NULL;                                             // 行首地址 避免每次访问像素都做二维乘法
    uint32 sum0 = 0, sum1 = 0;                                      // G0、G1 两组像素的灰度值之和
    uint16 num0 = 0, num1 = 0;                                      // G0、G1 两组像素的个数
    float  m0 = 0, m1 = 0;                                          // G0、G1 两组像素的平均灰度值
    float  t_old = 0, t_new = 0;                                    // 上一轮阈值、本轮新阈值
    uint8  t_cmp = 0;                                               // 参与像素比较的整型阈值(浮点比较太慢)
    uint16 i = 0, j = 0;
    uint8  loop = 0;
    uint32 sum_all = 0;                                             // 整幅图像的灰度值总和

    // 初始阈值 T2 = 图像的平均灰度 初始阈值必须大于图像中的最小灰度级而小于最大灰度级
    for(i = 0; i < MT9V03X_H; i++)
    {
        row = mt9v03x_image[i];
        for(j = 0; j < MT9V03X_W; j++)
        {
            sum_all += row[j];
        }
    }
    t_old = (float)sum_all / (float)(MT9V03X_H * MT9V03X_W);

    if((t_old < 1) || (t_old > 254))                                // 整幅图像全黑或者全白 用宏定义的初值兜底
    {
        t_old = (float)IMAGE_ITERATION_INIT;
    }

    do
    {
        sum0 = 0;   sum1 = 0;                                       // 每轮迭代重新统计 不累加上一轮的数据
        num0 = 0;   num1 = 0;

        t_cmp = (uint8)(t_old + 0.5f);                              // 阈值四舍五入成整数再与像素比较

        for(i = 0; i < MT9V03X_H; i++)
        {
            row = mt9v03x_image[i];                                 // 取该行行首地址
            for(j = 0; j < MT9V03X_W; j++)
            {
                if(row[j] < t_cmp)                                  // 灰度值小于阈值 归入 G0
                {
                    sum0 += row[j];
                    num0++;
                }
                else                                                // 灰度值大于等于阈值 归入 G1
                {
                    sum1 += row[j];
                    num1++;
                }
            }
        }

        if((0 == num0) || (0 == num1))                              // 整幅图像全黑或者全白 已经无法再分 直接结束
        {
            break;
        }

        m0    = (float)sum0 / (float)num0;                          // G0 的平均灰度
        m1    = (float)sum1 / (float)num1;                          // G1 的平均灰度
        t_new = (m0 + m1) * 0.5f;                                   // 新阈值 T = 1/2 (m0 + m1)

        if(((t_new > t_old) ? (t_new - t_old) : (t_old - t_new)) <= (float)IMAGE_ITERATION_ERROR)
        {
            t_old = t_new;                                          // 两轮阈值之差小于 ΔT 认为已经收敛
            break;
        }

        t_old = t_new;                                              // 用新阈值进入下一轮迭代
    }while(++loop < IMAGE_ITERATION_MAX_LOOP);

    image_threshold = (uint8)(t_old + 0.5f);                        // 四舍五入取整 对应文章中的 BlackThres

    return image_threshold;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     大津法(OTSU)求全局阈值
// 对应文章     智能车图像处理(一)阈值处理 —— (二)基于Otus的最佳全局阈值处理
// 基本思路     1.计算灰度级中每个像素在整幅图像中的个数    -> 灰度直方图
//              2.计算每个像素值的点在整幅图像中的概率
//              3.计算全局灰度均值
//              4.计算类间方差  delta = w0 * (u0 - u)^2 + w1 * (u1 - u)^2
//              5.得到类间方差最大时的灰度值即为 Otus 阈值
// 返回参数     uint8               得到的全局阈值
// 备注信息     1.参考文章中 gray_sum/pixelSum 使用了整型除法 会丢掉小数 这里改成浮点除法
//              2.参考文章用 pow(x,2) 求平方 这里直接写成 x*x 结果相同但是省掉了库函数调用
//              3.参考文章为了加速 直方图统计时隔行隔列取样 这里保持一致(IMAGE_OTSU_STEP = 2)
//-------------------------------------------------------------------------------------------------------------------
uint8 image_threshold_otsu (void)
{
    static uint16 pixel_count[256];                                 // 灰度直方图 每个灰度级的像素个数
                                                                    // 使用 static 避免占用栈空间
    uint32 gray_sum  = 0;                                           // 参与统计的像素灰度值总和
    uint32 num_total = 0;                                           // 参与统计的像素总个数
    uint32 sum_back  = 0;                                           // 背景(灰度 0~t)部分的灰度值之和
    uint32 num_back  = 0;                                           // 背景(灰度 0~t)部分的像素个数
    float  gray_mean = 0;                                           // 全局平均灰度 u
    float  w0 = 0, w1 = 0;                                          // 背景、前景像素所占比例
    float  u0 = 0, u1 = 0;                                          // 背景、前景的平均灰度
    float  delta = 0, delta_max = 0;                                // 类间方差、最大类间方差
    uint8  *row = NULL;                                             // 行首地址
    uint8  threshold = image_threshold;                             // 求不出有效阈值时保持原有阈值
    uint16 i = 0, j = 0, t = 0;

    for(i = 0; i < 256; i++)
    {
        pixel_count[i] = 0;                                         // 清空直方图
    }

    // 1.统计灰度级中每个像素在整幅图像中的个数(隔行隔列抽样 与参考文章一致)
    for(i = 0; i < MT9V03X_H; i += IMAGE_OTSU_STEP)
    {
        row = mt9v03x_image[i];
        for(j = 0; j < MT9V03X_W; j += IMAGE_OTSU_STEP)
        {
            pixel_count[row[j]]++;                                  // 将当前的点的像素值作为计数数组的下标
            gray_sum += row[j];                                     // 灰度值总和
            num_total++;
        }
    }

    if(0 == num_total)                                              // 异常保护
    {
        return image_threshold;
    }

    gray_mean = (float)gray_sum / (float)num_total;                  // 全局平均灰度

    // 2.遍历灰度级[0,255] 找到类间方差最大的那个灰度值
    for(t = 0; t < 256; t++)
    {
        num_back += pixel_count[t];                                 // 背景部分每个灰度值的像素点个数之和
        sum_back += (uint32)t * pixel_count[t];                     // 背景部分 每个灰度值的点的个数 * 灰度值

        if((0 == num_back) || (num_total == num_back))               // 全是前景或者全是背景 无法计算 跳过
        {
            continue;
        }

        w0 = (float)num_back / (float)num_total;                    // 背景像素所占比例
        w1 = 1.0f - w0;                                             // 前景像素所占比例
        u0 = (float)sum_back / (float)num_back;                     // 背景平均灰度
        u1 = (float)(gray_sum - sum_back) / (float)(num_total - num_back);   // 前景平均灰度

        delta = w0 * (u0 - gray_mean) * (u0 - gray_mean)             // 类间方差
              + w1 * (u1 - gray_mean) * (u1 - gray_mean);

        if(delta > delta_max)
        {
            delta_max = delta;
            threshold = (uint8)t;                                   // 记录当前的最佳阈值
        }
#if (1 == IMAGE_OTSU_EARLY_BREAK)
        else if(delta < delta_max)
        {
            break;                                                  // 类间方差曲线是单峰的 开始减小说明已经越过峰值
        }
#endif
    }

    image_threshold = threshold;

    return image_threshold;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     阈值限幅 把阈值限制在 1~254 之间
// 参数说明     value               待限幅的阈值
// 返回参数     uint8               限幅后的阈值
// 备注信息     阈值取 0 会让整幅图像全白 取 255 会让整幅图像全黑 所以限幅到 1~254
//-------------------------------------------------------------------------------------------------------------------
static uint8 image_threshold_limit (int16 value)
{
    if(value < 1)
    {
        return 1;
    }

    if(value > 254)
    {
        return 254;
    }

    return (uint8)value;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据指定算法求全局阈值
// 参数说明     mode                IMAGE_THRESHOLD_MODE_OTSU / IMAGE_THRESHOLD_MODE_ITERATION / IMAGE_THRESHOLD_MODE_MANUAL
// 返回参数     uint8               最终使用的全局阈值(同时保存在 image_threshold 中)
// 使用示例     image_threshold_calc(IMAGE_THRESHOLD_MODE_OTSU);
// 备注信息     自动模式下 求出的阈值还会再加上 IMAGE_THRESHOLD_OFFSET 偏移 用来微调赛道和背景的分界
//-------------------------------------------------------------------------------------------------------------------
uint8 image_threshold_calc (uint8 mode)
{
    if(IMAGE_THRESHOLD_MODE_MANUAL == mode)                         // 手动模式 不自动求阈值 直接用宏定义的固定值
    {
        image_threshold = image_threshold_limit(IMAGE_THRESHOLD_MANUAL);

#if(1 == IMAGE_MANUAL_COMPARE_AUTO)
        {
            uint8 save_threshold = image_threshold;

            // 顺便用大津法算一遍 只写入 image_threshold_auto 给串口打印做对比
            // 算完把 image_threshold 还原 二值化依然使用手动阈值
            //
            // 【性能优化】大津法只是为了在示波器 ch2 上观察光线有没有整体变化，不需要每帧都算。
            //   它要采样 11280 个像素再跑一遍 256 级直方图，每帧都算是纯浪费。
            //   改成每隔 IMAGE_AUTO_THRESHOLD_INTERVAL 帧算一次，ch2 曲线照样能看出趋势。
            if(0 == (image_auto_tick++ % IMAGE_AUTO_THRESHOLD_INTERVAL))
            {
                image_threshold_auto = image_threshold_otsu();
            }
            image_threshold = save_threshold;
        }
#endif

        return image_threshold;
    }

    if(IMAGE_THRESHOLD_MODE_NEAR_REF == mode)                       // 近端参考白点：最靠近车身那几行一定是赛道
    {
        uint8 *row_p;
        uint32 sum = 0;
        uint16 cnt = 0;
        uint16 x;
        uint16 y;
        uint8  ref;
        int16  th;

        for(y = IMAGE_NEAR_REF_ROW1; y <= IMAGE_NEAR_REF_ROW2; y++)
        {
            row_p = mt9v03x_image[y];
            for(x = IMAGE_NEAR_REF_COL1; x <= IMAGE_NEAR_REF_COL2; x++)
            {
                sum += row_p[x];
                cnt++;
            }
        }

        ref = (uint8)(sum / cnt);
        image_near_ref = ref;

        if(ref >= IMAGE_NEAR_REF_MIN)                               // 这块确实像赛道 才更新
        {
            th = (int16)((float)ref * IMAGE_NEAR_REF_RATIO + 0.5f);

#if(1 == IMAGE_NEAR_REF_SMOOTH)
            {
                int16 old = (int16)image_threshold;

                if(0 == old)                                        // 第一次没有旧值 直接用新值
                {
                    old = th;
                }
                th = (int16)(((int32)old * 3 + (int32)th) / 4);      // 一阶平滑 3:1
            }
#endif

            image_threshold_near = image_threshold_limit(th);
        }
        // 参考白点太低（摄像头被挡/全黑/车冲出赛道）-> 保留上一次的阈值 不让它乱跳

        image_threshold = image_threshold_near;

        return image_threshold;
    }


    if(IMAGE_THRESHOLD_MODE_ITERATION == mode)                      // 迭代法
    {
        image_threshold_iteration();
    }
    else                                                            // 大津法(默认)
    {
        image_threshold_otsu();
    }

    image_threshold = image_threshold_limit((int16)image_threshold + IMAGE_THRESHOLD_OFFSET);

    return image_threshold;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按当前阈值 image_threshold 对灰度图像做二值化
// 参数说明     *dest               二值化图像存放地址 大小必须 >= MT9V03X_IMAGE_SIZE 个字节
// 返回参数     void
// 使用示例     image_binarize(image_copy[0]);
// 备注信息     对应参考文章中的 Binarization 函数
//              灰度值大于阈值 置为全白  灰度值小于等于阈值 置为全黑
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     灰度快照：把摄像头灰度图原样拷进目标缓冲
// 参数说明     *dest               目标缓冲(通常 image_copy[0])
// 返回参数     void
// 备注信息     这一趟是【必须】的：
//               mt9v03x_image 是 DMA 直接写入的缓冲区，摄像头每出一帧就把它整体覆盖一遍。
//               必须"刚采完一帧"时立刻把它拷成一张静态快照，后面的算法和发送都用快照，
//               否则读到的是好几帧拼起来的图（画面是噪声、寻线找不到线）。
//
//             为什么不用标准库的 memcpy：
//               C251 标准库的 memcpy 处理 far 指针时性能很差，实车帧率会掉到个位数。
//               这里用和 image_binarize() 完全相同的循环结构
//               （行指针 + 每行只算一次行偏移 + 自增指针），那种写法实测是能跑满帧的。
//-------------------------------------------------------------------------------------------------------------------
void image_gray_copy (uint8 *dest)
{
    uint8  *src_row;
    uint8  *dst_row;
    uint8  *s;
    uint8  *d;
    uint16 i;
    uint16 j;

    for(i = 0; i < MT9V03X_H; i++)
    {
        src_row = mt9v03x_image[i];
        dst_row = dest + (uint32)i * MT9V03X_W;

        s = src_row;
        d = dst_row;

        for(j = 0; j < MT9V03X_W; j++)
        {
            *d++ = *s++;
        }
    }
}
void image_binarize (uint8 *dest)
{
    uint8  *src_row = NULL;                                         // 源图像行首地址(库中的灰度图)
    uint8  *dst_row = NULL;                                         // 目标图像行首地址(二值化结果)
    uint8  *s;                                                      // 自增读指针
    uint8  *d;                                                      // 自增写指针
    uint16 i = 0, j = 0;
    uint8  th;                                                      // 阈值取到局部变量

    // 【性能优化】这一趟要跑 120×188 = 22560 个像素，是整个流程里最贵的一步（实测约 19ms）。
    //   改了两点：
    //   1. 用"自增指针"代替 src_row[j] / dst_row[j] 下标访问
    //      —— C251 上每次下标访问都要重算一遍地址，自增指针只要加一
    //   2. 把全局变量 image_threshold 提到局部 th
    //      —— 否则内层循环每轮都要重新从内存读一次这个全局量
    th = image_threshold;

    for(i = 0; i < MT9V03X_H; i++)
    {
        src_row = mt9v03x_image[i];
        dst_row = dest + (uint32)i * MT9V03X_W;                     // 按行偏移 避免每个像素都做二维乘法

        s = src_row;
        d = dst_row;

        for(j = 0; j < MT9V03X_W; j++)
        {
            *d++ = (*s++ > th) ? IMAGE_WHITE : IMAGE_BLACK;
        }
    }
}
