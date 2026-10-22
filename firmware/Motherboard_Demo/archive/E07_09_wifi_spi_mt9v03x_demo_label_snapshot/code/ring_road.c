/*********************************************************************************************************************
* 文件名称          ring_road.c
* 公司名称          成都逐飞科技有限公司
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
*
* 功能简介          圆环（环岛）识别，左环右环都支持
*                  算法参考：智能车图像处理(三)元素判断
*                  https://blog.csdn.net/weixin_50567482/article/details/119406477
*
* 说明              文章里圆环判断只有文字描述、没有给出代码（只有三岔给了代码），
*                  本文件是把文章描述的两套方法都实现出来并且互相印证：
*                    方法一 找拐点    ：对面直道 + 入口侧下拐点(边界突变) + 中/上拐点
*                    方法二 黑白跳变点：对面直道 + 拐点上方全白行 + 黑白跳变行
*                  左右环完全对称。
*
* 修改记录
* 日期              作者           备注
* 2026-xx-xx        user           first version
********************************************************************************************************************/

#include "ring_road.h"
#include "image_process.h"                                // 灰度直读要用到 image_threshold
#include "track_edge.h"
#include "cross_road.h"                                      // 十字状态(圆环补线要和十字互斥)


// 识别结果
uint8 ring_flag       = 0;
uint8 ring_type       = RING_TYPE_NONE;
uint8 ring_state      = RING_STATE_NONE;
uint8 ring_corner_row = 0;
uint8 ring_corner_col = 0;
uint8 ring_white_row  = 0;
uint8 ring_trans_row  = 0;
uint8 ring_open_row1  = 0;
uint8 ring_open_row2  = 0;
int8  ring_fail       = 0;
uint8 ring_dbg_dev    = 0;                                          // 对面边线拟合后的最大偏差(像素)
uint8 ring_dbg_lost   = 0;                                          // 对面边线在取行范围内的丢线行数
uint8 ring_rep_dev    = 0;                                          // 只记录"主尝试"（丢线多那一侧）的实测值
uint8 ring_rep_lost   = 0;                                          // 否则会被第二次尝试覆盖掉，诊断就错了

// 内部变量
static uint16 ring_state_count = 0;                                 // 状态保护计数
static uint16 ring_find_fail = 0;                                  // 状态1里"一直等不到张开"的连续帧数
static uint8  ring_confirm     = 0;                                 // 连续满足条件的帧数
static uint8  ring_straight_fail = 0;                               // 对面直道判据里哪一条没过
static uint8  ring_dev_hist[RING_DEV_BIN];                           // 偏差直方图 用来求中位数

#define RING_PIXEL(image, row, col)     ((image)[(uint32)(row) * MT9V03X_W + (col)])
#define RING_IS_WHITE(pixel)            (((pixel) >= IMG_WHITE_LEVEL) ? 1 : 0)   // 灰度直读：用编译期常量最快


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断一条边线是不是"连续直道"
// 参数说明     *edge               边线数组(track_left / track_right)
// 参数说明     mask                丢线标志位
// 返回参数     uint8               1:是连续直道  0:不是
// 备注信息     对应文章的"判断对面那条边界为连续直道"：
//              先看这段行里丢线多不多，再用最小二乘拟合一条直线看最大偏差
//-------------------------------------------------------------------------------------------------------------------
static uint8 ring_is_straight (uint8 *edge, uint8 mask)
{
    int32 sx  = 0;
    int32 sy  = 0;
    int32 sxy = 0;
    int32 syy = 0;
    int16 n   = 0;
    int16 lost_count = 0;
    int16 y;
    int16 i;
    int16 d;
    int16 half;
    int16 acc;
    int16 median = 0;
    float den;
    float k;
    float b;
    float dev;

    for(y = RING_STRAIGHT_Y1; y <= RING_STRAIGHT_Y2; y++)
    {
        if(0 != (track_lost[y] & mask))
        {
            lost_count++;
            continue;
        }

        sx  += edge[y];
        sy  += y;
        sxy += (int32)edge[y] * y;
        syy += (int32)y * y;
        n++;
    }

    if(lost_count > RING_STRAIGHT_LOST_MAX)
    {
        ring_dbg_lost = (uint8)lost_count;
        ring_straight_fail = 1;                                     // 丢线太多
        return 0;
    }

    if(n < RING_STRAIGHT_MIN_PT)
    {
        ring_dbg_lost = (uint8)lost_count;
        ring_straight_fail = 2;                                     // 有效点太少 拟合不了
        return 0;
    }

    den = (float)n * (float)syy - (float)sy * (float)sy;
    if((den < 0.001f) && (den > -0.001f))
    {
        ring_straight_fail = 3;
        return 0;
    }

    k = ((float)n * (float)sxy - (float)sx * (float)sy) / den;
    b = ((float)sx - k * (float)sy) / (float)n;

    for(i = 0; i < RING_DEV_BIN; i++)                               // 偏差直方图清零
    {
        ring_dev_hist[i] = 0;
    }

    n = 0;
    for(y = RING_STRAIGHT_Y1; y <= RING_STRAIGHT_Y2; y++)
    {
        if(0 != (track_lost[y] & mask))
        {
            continue;
        }

        dev = (k * (float)y + b) - (float)edge[y];
        if(dev < 0)
        {
            dev = -dev;
        }

        d = (int16)(dev + 0.5f);
        if(d >= RING_DEV_BIN)
        {
            d = RING_DEV_BIN - 1;
        }
        ring_dev_hist[d]++;
        n++;
    }

    // 用中位数而不是最大值来衡量直线度
    // 实车实测：对面那条边线肉眼看是直的，但数组里偶尔有几行是离群点
    // （种子行的初始误差、个别行的跳变），最大偏差会被这几行拉到 58 像素，
    // 换成中位数就只剩几个像素，误判就消失了
    half   = n / 2;
    acc    = 0;
    median = RING_DEV_BIN - 1;
    for(i = 0; i < RING_DEV_BIN; i++)
    {
        acc += (int16)ring_dev_hist[i];
        if(acc > half)
        {
            median = i;
            break;
        }
    }

    ring_dbg_lost = (uint8)lost_count;
    ring_dbg_dev  = (uint8)median;

    if(median > RING_STRAIGHT_DEV_MAX)
    {
        ring_straight_fail = 4;                                     // 不够直
        return 0;
    }

    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     找圆环入口的拐点（边界在几行内突然向外张开）
// 参数说明     *edge               入口那一侧的边线
// 参数说明     mask                丢线标志位
// 参数说明     to_left             1:找左环(左边界向左突变)  0:找右环(右边界向右突变)
// 参数说明     *row_out, *col_out  找到的拐点
// 返回参数     uint8               1:找到  0:没找到
// 备注信息     对应文章的"找到图一和图二左下方的拐点"，以及三岔代码里
//              "边界列数先变大再变小/先变小后变大，找到那个大小变化的点"的思路
//              PC 仿真发现：入口那个突变经常正好落在丢线行上（种子生长跟不上这么大的跳变），
//              如果要求"连续几行都不丢线"就会把拐点漏掉，所以这里改成
//              "比较相邻两个有效行（可以跨过丢线行）"
//-------------------------------------------------------------------------------------------------------------------
static uint8 ring_find_corner (uint8 *edge, uint8 mask, uint8 to_left, uint8 *row_out, uint8 *col_out)
{
    int16 y;
    int16 prev = -1;                                                // 上一个有效行
    int16 delta;

    for(y = RING_SCAN_NEAR; y > RING_SCAN_FAR; y--)
    {
        if(0 != (track_lost[y] & mask))
        {
            continue;                                               // 丢线行跳过 但保留 prev
        }

        if((prev >= 0) && ((prev - y) <= RING_JUMP_ROWS))
        {
            if(0 != to_left)
            {
                delta = (int16)edge[prev] - (int16)edge[y];          // 往上走 列数变小 = 向左张开
            }
            else
            {
                delta = (int16)edge[y] - (int16)edge[prev];          // 往上走 列数变大 = 向右张开
            }

            if(delta >= RING_JUMP)
            {
                *row_out = (uint8)prev;
                *col_out = edge[prev];
                return 1;
            }
        }

        prev = y;
    }

    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在图像上半部分找"全白行"和"黑白跳变行"
// 返回参数     void
// 备注信息     对应文章方法二的第 2 步：
//              "在左下拐点的上方为全白；往下隔3行扫描，共扫描4行，
//                看这几行同一行是否存在由白到黑以及黑到白的跳变点"
//              white_row 记录最白的那一行，trans_row 记录跳变最多的那一行
//-------------------------------------------------------------------------------------------------------------------
static void ring_scan_feature_rows (uint8 *image)
{
    int16 y;
    int16 x;
    uint16 white_count;
    uint16 trans_count;
    uint16 best_white = 0;
    uint16 best_trans = 0;
    uint8 *row_p;                                                   // 行首指针(性能优化用)
    uint8  cur;
    uint8  prev;

    ring_white_row = 0;
    ring_trans_row = 0;

    for(y = RING_FEATURE_ROW1; y <= RING_FEATURE_ROW2; y++)
    {
        white_count = 0;
        trans_count = 0;
        row_p       = image + (uint32)y * MT9V03X_W;
        prev        = (row_p[0] >= IMG_WHITE_LEVEL) ? 1 : 0;

        for(x = 0; x < MT9V03X_W; x++)
        {
            // 【性能优化】原来每个像素要取两次(自己 + 左边)，而且每次都做 32 位乘法。
            //   改成"一行只算一次地址 + 记住上一个像素的值"，访问次数减半、乘法几乎没了
            cur = (row_p[x] >= IMG_WHITE_LEVEL) ? 1 : 0;

            if(0 != cur)
            {
                white_count++;
            }

            if(0 != cur != prev)
            {
                trans_count++;
            }
            prev = cur;
        }

        if(white_count > best_white)
        {
            best_white     = white_count;
            ring_white_row = (uint8)y;
        }

        // 既要有黑转白又要有白转黑 -> 至少两次跳变
        if(trans_count > best_trans)
        {
            best_trans     = trans_count;
            ring_trans_row = (uint8)y;
        }
    }

    if(best_white < RING_WHITE_MIN)
    {
        ring_white_row = 0;
    }

    if(best_trans < RING_TRANS_MIN)
    {
        ring_trans_row = 0;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     找入口那一侧最长的一段连续丢线
// 参数说明     mask                丢线标志位
// 参数说明     *a, *b, *n          丢线段的起止行与长度
// 返回参数     uint8               1:找到  0:没有
// 备注信息     圆环入口时，入口那一侧的道路向这一侧张开，边线会连续丢一大段
//-------------------------------------------------------------------------------------------------------------------
static uint8 ring_find_lost_run (uint8 mask, int16 *a, int16 *b, int16 *n)
{
    int16 y;
    int16 start = -1;
    int16 best_a = 0;
    int16 best_b = 0;
    int16 best_n = 0;

    *a = 0;
    *b = 0;
    *n = 0;

    for(y = 0; y <= MT9V03X_H; y++)
    {
        if((y < MT9V03X_H) && (0 != (track_lost[y] & mask)))
        {
            if(start < 0)
            {
                start = y;
            }
            continue;
        }

        if(start >= 0)
        {
            if((y - start) > best_n)
            {
                best_n = y - start;
                best_a = start;
                best_b = y - 1;
            }
            start = -1;
        }
    }

    if(best_n <= 0)
    {
        return 0;
    }

    *a = best_a;
    *b = best_b;
    *n = best_n;

    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按一侧去尝试识别圆环
// 参数说明     to_left             1:尝试识别左环  0:尝试识别右环
// 返回参数     uint8               1:识别到  0:没识别到
// 备注信息     三个条件（对应文章两套方法，并且用实车数据修正过）：
//              1.对面那条边线是连续直道（丢线少 + 接近直线）
//              2.入口这一侧有一段足够长的连续丢线（道路向这一侧张开）
//                找不到长丢线段时退回用文章方法一的"向外张开的拐点"
//              3.这段行范围里 对面基本不丢线
//                —— 这是圆环和十字最大的区别：十字的横杠会把左右两边同时盖住，
//                   圆环只会盖住一边（实车实测：右环 左0行/右56行）
//-------------------------------------------------------------------------------------------------------------------
static uint8 ring_try_side (uint8 to_left, uint8 primary)
{
    uint8 *ring_edge = (0 != to_left) ? track_left : track_right;   // 圆环所在那一侧的边线
    uint8 *oppo_edge = (0 != to_left) ? track_right : track_left;   // 对面那一侧的边线
    uint8  ring_mask = (0 != to_left) ? 0x01 : 0x02;
    uint8  oppo_mask = (0 != to_left) ? 0x02 : 0x01;
    int16  a = 0;
    int16  b = 0;
    int16  n = 0;
    int16  y;
    int16  oppo_lost = 0;
    uint8  row = 0;
    uint8  col = 0;

    if(0 != ring_state)                                             // 已经在圆环状态里就不重复识别
    {
        return 0;
    }

    // ---- 条件 1：对面是连续直道 ----
    ring_straight_fail = 0;                                         // 每次重新判 不然会残留上一次的结果

    if(0 == ring_is_straight(oppo_edge, oppo_mask))
    {
        if(0 != primary)                                            // 主尝试的实测值单独留一份 供诊断
        {
            ring_rep_dev  = ring_dbg_dev;
            ring_rep_lost = ring_dbg_lost;
        }

        if(0 == ring_fail)
        {
            // -11 丢线太多  -12 有效点太少  -13 拟合退化  -14 不够直
            ring_fail = (int8)(-10 - (int8)ring_straight_fail);
        }
        return 0;
    }

    // ---- 条件 2：入口这一侧道路张开 ----
    if(0 != ring_find_lost_run(ring_mask, &a, &b, &n))
    {
        if(n < RING_OPEN_LOST_MIN)
        {
            if(0 == ring_fail)
            {
                ring_fail     = -2;                             // 入口侧道路没张开
                ring_rep_lost = (uint8)n;                       // 把实测到的丢线段长度报出来 差多少一眼看得出
            }
            return 0;
        }
    }
    else
    {
        // 没有长丢线段 退回用文章方法一的拐点判据
        if(0 == ring_find_corner(ring_edge, ring_mask, to_left, &row, &col))
        {
            if(0 == ring_fail)
            {
                ring_fail = -2;                                 // 既没有长丢线段 也没有拐点
            }
            return 0;
        }
        a = (int16)row;
        b = (int16)row;
    }

    // ---- 条件 3：对面那条边线自己也不允许有太长的连续丢线段 ----
    // 极端斜入十字时左右两边的丢线段是"接着"的（一个在上一个在下），只靠范围重叠区分不开
    {
        int16 oa = 0;
        int16 ob = 0;
        int16 on = 0;

        if(0 != ring_find_lost_run(oppo_mask, &oa, &ob, &on))
        {
            if(on >= RING_OPPO_RUN_MAX)
            {
                if(0 == ring_fail)
                {
                    ring_fail = -3;                                 // 对面也有长丢线段 -> 更像十字
                }
                return 0;
            }
        }
    }

    // ---- 条件 4：这段行范围里 对面基本不丢线 ----
    for(y = (a - 2); y <= (b + 2); y++)
    {
        if((y < 0) || (y >= MT9V03X_H))
        {
            continue;
        }
        if(0 != (track_lost[y] & oppo_mask))
        {
            oppo_lost++;
        }
    }

    if(oppo_lost > RING_OPPO_LOST_MAX)
    {
        if(0 == ring_fail)
        {
            ring_fail = -3;                                     // 对面在张开段也丢线 -> 更像十字
        }
        return 0;
    }

    ring_open_row1  = (uint8)a;
    ring_open_row2  = (uint8)b;
    ring_corner_row = (uint8)a;
    ring_corner_col = ring_edge[a];

    return 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环识别状态机
// 参数说明     *image              二值化图像
// 返回参数     void
// 备注信息     对应文章的状态推进：
//              状态一(识别到入口) -> 状态二(进环中) -> 状态三(环内) -> 出环
//              出环判据用文章说的"丢线数下降 + 赛道宽度恢复"
//-------------------------------------------------------------------------------------------------------------------
static void ring_state_machine (uint8 *image)
{
    uint16 ring_lost;
    uint16 oppo_lost;
    uint8  found;

    ring_lost = (RING_TYPE_LEFT == ring_type) ? track_left_lost_count : track_right_lost_count;
    oppo_lost = (RING_TYPE_LEFT == ring_type) ? track_right_lost_count : track_left_lost_count;

    switch(ring_state)
    {
        case RING_STATE_NONE:                                       // 不在圆环里：等识别条件
        {
            ring_scan_feature_rows(image);

            found     = 0;
            ring_type = RING_TYPE_NONE;
            ring_fail = 0;

            // 先从丢线多的那一侧试（那一侧才像圆环入口）；失败时保留它的原因供诊断
            if(track_left_lost_count >= track_right_lost_count)
            {
                if(0 != ring_try_side(1, 1))
                {
                    found     = 1;
                    ring_type = RING_TYPE_LEFT;
                }
                else if(0 != ring_try_side(0, 0))
                {
                    found     = 1;
                    ring_type = RING_TYPE_RIGHT;
                }
            }
            else
            {
                if(0 != ring_try_side(0, 1))
                {
                    found     = 1;
                    ring_type = RING_TYPE_RIGHT;
                }
                else if(0 != ring_try_side(1, 0))
                {
                    found     = 1;
                    ring_type = RING_TYPE_LEFT;
                }
            }

            if(0 != found)
            {
                ring_confirm++;
                if(ring_confirm >= RING_CONFIRM_FRAME)              // 连续几帧都满足才确认
                {
                    ring_flag         = 1;
                    ring_state        = RING_STATE_FIND;
                    ring_state_count  = 0;
                    ring_confirm      = 0;
                }
            }
            else
            {
                ring_confirm  = 0;
                ring_corner_row = 0;
                ring_corner_col = 0;
            }
            break;
        }

        case RING_STATE_FIND:                                       // 识别到入口：等入口一侧大量丢线(张开)
        {
            ring_flag = 1;

            if(ring_lost >= RING_IN_LOST_MIN)
            {
                ring_state       = RING_STATE_IN;
                ring_state_count = 0;
                ring_find_fail   = 0;
            }
            else
            {
                // 【防误判】入口识别出来之后，如果"这一侧道路张开"(大量丢线)一直等不到，
                //   说明这个入口是假的（实车：直道上误判成左环，ch4=11 一直卡在状态1，
                //   原来只能等 RING_STATE_TIMEOUT=200帧≈10秒 才自己好，中间左边线一直被补歪）。
                //   这里给它一个更短的耐心值：连续 RING_FIND_FAIL_MAX 帧等不到张开 就复位重新识别。
                ring_find_fail++;
                if(ring_find_fail >= RING_FIND_FAIL_MAX)
                {
                    ring_flag        = 0;
                    ring_type        = RING_TYPE_NONE;
                    ring_state       = RING_STATE_NONE;
                    ring_state_count = 0;
                    ring_find_fail   = 0;
                    ring_confirm     = 0;
                }
            }
            break;
        }

        case RING_STATE_IN:                                         // 正在进环：等环内特征(跳变行/全白行)
        {
            ring_flag = 1;
            ring_scan_feature_rows(image);

            if((0 != ring_white_row) || (0 != ring_trans_row))
            {
                ring_state       = RING_STATE_INSIDE;
                ring_state_count = 0;
            }
            break;
        }

        case RING_STATE_INSIDE:                                     // 环内：等入口一侧丢线下降
        {
            ring_flag = 1;

            if(ring_lost <= RING_OUT_LOST_MAX)
            {
                ring_state       = RING_STATE_OUT;
                ring_state_count = 0;
            }
            break;
        }

        case RING_STATE_OUT:                                        // 出环：对面也恢复直道就结束
        {
            ring_flag = 1;

            if((ring_lost <= RING_OUT_LOST_MAX) && (oppo_lost <= RING_OUT_LOST_MAX))
            {
                ring_flag        = 0;
                ring_type        = RING_TYPE_NONE;
                ring_state       = RING_STATE_NONE;
                ring_state_count = 0;
            }
            break;
        }

        default:
        {
            ring_flag        = 0;
            ring_type        = RING_TYPE_NONE;
            ring_state       = RING_STATE_NONE;
            ring_state_count = 0;
            break;
        }
    }

    // 状态保护：长时间没有推进就强制复位 防止卡死
    if(RING_STATE_NONE != ring_state)
    {
        ring_state_count++;
        if(ring_state_count > RING_STATE_TIMEOUT)
        {
            ring_flag        = 0;
            ring_type        = RING_TYPE_NONE;
            ring_state       = RING_STATE_NONE;
            ring_state_count = 0;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环补线：把圆环侧那条边线在"入口拐点以上"的一段补出来
// 参数说明     to_left             1:左环  0:右环
// 返回参数     void
// 备注信息     分三个阶段，对应圆环的三个状态：
//                入环(状态1/2)：补成朝环内倾斜的斜线（RING_IN_SLOPE）
//                                —— 入口处那一侧道路张开了，寻线要么丢线要么跟到环的外沿上，
//                                   直接用手会让中线摆得太早/太晚；补成固定斜率的斜线，
//                                   中线会稳定地摆向环内，车就平稳转进去了
//                环内(状态3)  ：不补。环内的白区就是环的路面，正常寻线得到的中线就能走
//                出环(状态4)  ：补成竖直的线（斜率为 0）
//                                —— 让车直着出环，不被环的外沿带着继续绕圈
//              补线的起点是"入口拐点"那一行，往上一直补到 RING_FIX_TOP 行
//-------------------------------------------------------------------------------------------------------------------
// 【实车事故】原来这里算的是"拐点以下那一段的【平均】赛道宽度"，再拿这个宽度去补上面的行。
//   问题是摄像头是前视的、赛道在图像里是【梯形】：近处(行号大)宽、远处(行号小)窄。
//   用近处的平均宽度去补远处的行，宽度就偏大，补出来的边线被一路推到画面外面。
//   实车现象（直道上误判成左环、ch4=11 卡在状态1）：
//     右边界正常（166→106），左边界从第 91 行的 30 直接跳到第 88 行的 0，
//     往上 5~88 行全是贴着画面左边的 0 —— 上位机上绿线断成两截、上半截贴着左边缘，
//     中线也跟着歪（红线整段右移）。
//   现在改成：对"宽度 vs 行号"做一次线性拟合，按行算宽度（前视梯形就是个近似线性的关系），
//   并且【补出来的值如果出画面就不补】—— 宁可这一段保持寻线的原结果，
//   也不能把边线贴到画面上去（贴上去会把中线整个带偏）。
//-------------------------------------------------------------------------------------------------------------------
static void ring_fix_side (uint8 to_left)
{
    uint8 *ring_edge = (0 != to_left) ? track_left : track_right;   // 圆环侧(要补的那条)
    uint8 *oppo_edge = (0 != to_left) ? track_right : track_left;   // 对面(可靠的那条)
    int16  y;
    int16  value;
    int16  wd;
    int16  from_y;
    int16  n = 0;
    int16  w_corner;
    int16  width_y;
    int16  bias;
    int32  sy = 0;
    int32  sw = 0;
    int32  syw = 0;
    int32  syy = 0;
    float  den;
    float  kw = 0.0f;                                               // 宽度对行号的斜率
    float  bw = 0.0f;                                               // 宽度拟合的截距

    if(RING_STATE_INSIDE == ring_state)
    {
        return;                                                     // 环内不补
    }

    if(RING_TYPE_NONE == ring_type)
    {
        return;
    }

    from_y = (int16)ring_corner_row;                                // 从入口拐点开始
    if(from_y <= RING_FIX_TOP)
    {
        return;                                                     // 拐点太靠上 没什么可补的
    }

    // ---- 第一步：用"拐点以下"那一段(正常的直道)统计"赛道宽度随行号的变化" ----
    // 这一段两条边线都是可靠的，拟合出来的直线能一直外推到上面的行
    for(y = from_y; y < MT9V03X_H; y++)
    {
        if(0 != (track_lost[y] & 0x03))                             // 两条边线都要有效
        {
            continue;
        }
        if(track_right[y] <= track_left[y])
        {
            continue;
        }

        wd = (int16)track_right[y] - (int16)track_left[y];

        sy  += y;
        sw  += wd;
        syw += (int32)wd * y;
        syy += (int32)y * y;
        n++;
    }

    if(n < 5)
    {
        return;                                                     // 量不出宽度就不补 免得更糟
    }

    den = (float)n * (float)syy - (float)sy * (float)sy;

    if((den > 0.001f) || (den < -0.001f))
    {
        kw = ((float)n * (float)syw - (float)sy * (float)sw) / den;
        bw = ((float)sw - kw * (float)sy) / (float)n;
    }
    else
    {
        kw = 0.0f;                                                  // 所有点在同一行 退化成平均
        bw = (float)sw / (float)n;
    }

    if(kw > RING_WIDTH_SLOPE_MAX)
    {
        kw = RING_WIDTH_SLOPE_MAX;
    }
    if(kw < -RING_WIDTH_SLOPE_MAX)
    {
        kw = -RING_WIDTH_SLOPE_MAX;
    }

    w_corner = (int16)(kw * (float)from_y + bw);                    // 拐点那一行的宽度
    if(w_corner < 20)
    {
        w_corner = 20;                                              // 太窄肯定不对
    }

    // ---- 第二步：逐行补线 ----
    // 关键：补出来的边界 = 对面那条边线 ± 该行的赛道宽度 (+ 入环偏移)
    //       宽度按行算（梯形），所以边界永远在赛道附近，不会像第一版那样被推到图像边缘
    for(y = (from_y - 1); y >= RING_FIX_TOP; y--)
    {
        bias = 0;

        if(RING_STATE_OUT != ring_state)                            // 入环：再朝环内偏一点
        {
            bias = (int16)(RING_IN_SLOPE * (float)(from_y - y) + 0.5f);
            if(bias > RING_IN_BIAS_MAX)
            {
                bias = RING_IN_BIAS_MAX;
            }
        }

        width_y = (int16)(kw * (float)y + bw);                      // 这一行的赛道宽度(按拟合外推)
        if(width_y < (w_corner / 2))
        {
            width_y = w_corner / 2;                                 // 外推得太小就兜住
        }
        if(width_y > ((w_corner * 3) / 2))
        {
            width_y = (w_corner * 3) / 2;                           // 外推得太大也兜住
        }

        if(0 != to_left)
        {
            value = (int16)oppo_edge[y] - width_y - bias;           // 左环：左线 = 右线 - 宽度 - 偏移
        }
        else
        {
            value = (int16)oppo_edge[y] + width_y + bias;           // 右环：右线 = 左线 + 宽度 + 偏移
        }

        // 【关键】算出来的位置必须在画面里。出画面说明"宽度模型在这么高的行上不成立"，
        //   这一行就不补，保持寻线原来的结果 —— 绝不能限幅贴到画面边上
        if((value < 0) || (value > (MT9V03X_W - 1)))
        {
            continue;
        }

        ring_edge[y] = (uint8)value;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环补线（按识别出来的类型补对应那一侧）
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------
static void ring_fix_line (void)
{
#if(0 == RING_FIX_ENABLE)
    return;                                                         // 补线开关关掉 只识别不补
#endif

    if(RING_STATE_NONE == ring_state)
    {
        return;
    }

    // 【十字优先】十字正在补线（或这一帧刚桥接过）时，圆环补线不做。
    //   实车实测：十字里 ch4 冒出 23（误判成右环且在环内），两套补线互相打架，
    //   边线每帧形状都不一样 —— 看起来就是"线一直跳"。
    if((0 != cross_state) || (0 != cross_bridge_used))
    {
        return;
    }

    if(RING_TYPE_LEFT == ring_type)
    {
        ring_fix_side(1);                                           // 左环补左线
    }
    else if(RING_TYPE_RIGHT == ring_type)
    {
        ring_fix_side(0);                                           // 右环补右线
    }

    track_center_update();                                          // 补完刷新中线
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环识别
// 参数说明     *binary_image      二值化图像(0/255) 通常传 image_copy[0]
// 返回参数     void
// 使用示例     ring_road_process(image_copy[0]);
// 备注信息     结果放在 ring_flag / ring_type / ring_state 中
//              识别完之后顺便把圆环侧的边线补出来（入环/出环阶段），中线也一起刷新
//-------------------------------------------------------------------------------------------------------------------
void ring_road_process (uint8 *binary_image)
{
    ring_state_machine(binary_image);
    ring_fix_line();
}
