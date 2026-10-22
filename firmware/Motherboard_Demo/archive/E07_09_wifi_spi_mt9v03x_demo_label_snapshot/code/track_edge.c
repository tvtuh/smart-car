/*********************************************************************************************************************
* 文件名称          track_edge.c
* 公司名称          成都逐飞科技有限公司
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
*
* 功能简介          种子生长法寻线（提取赛道左右边线和中线）
*                  算法参考：智能车图像处理(二)基础寻线
*                  https://blog.csdn.net/weixin_50567482/article/details/119353098
*
* 说明              参考文章里"种子生长法"只有文字描述没有给出代码(作者原文："第三种巡线方式
*                  之后如果能够写出来，可能会添上")，本文件是按文章的描述实现的：
*                  "以寻到的两个点为基础，框出一个3*3的矩形框，分别对左右两个点所在的矩阵框内
*                    进行顺时针和逆时针扫描，找到跳变点，依次往下扫描"
*
* 修改记录
* 日期              作者           备注
* 2026-xx-xx        user           first version
********************************************************************************************************************/

#include "track_edge.h"
#include "image_process.h"                                // 灰度直读要用到 image_threshold
#include "cross_road.h"                                   // 贴边要知道当前在不在十字里
#include "ring_road.h"                                    // 贴边要知道当前在不在圆环里


// 寻线结果
uint8 track_left[MT9V03X_H];
uint8 track_right[MT9V03X_H];
uint8 track_center[MT9V03X_H];
uint8 track_lost[MT9V03X_H];
uint8 track_left_lost_count  = 0;
uint8 track_right_lost_count = 0;
uint8 track_interp_count    = 0;                                   // 被丢线插值补上的行数(诊断用)

// 左右两侧连续丢失的行数 用于决定什么时候做全局重捕获
static uint16 track_lost_run[2] = { 0, 0 };
static uint8 track_left_top_pin_rows = 0;

#define TRACK_LEFT_TOP_PIN_COL (2)
#define TRACK_RIGHT_TREND_ROWS (12)
#define TRACK_RIGHT_TREND_MIN_ROWS (8)
#define TRACK_RIGHT_PREDICT_MAX_GAP (60)
#define TRACK_RIGHT_PREDICT_MAX_SHIFT (40)
#define TRACK_RIGHT_REACQUIRE_TOL (8)
#define TRACK_RIGHT_CROSS_END_TOL (12)
#define TRACK_RIGHT_CROSS_JOIN_TOL (8)
#define TRACK_RIGHT_CROSS_MAX_SPAN (70)
#define TRACK_WHITE_RUN_MIN (2)
#define TRACK_WHITE_EDGE_STEP_MAX (8)

static void track_lock_left_top_rows (void)
{
    uint16 row;

    for(row = 0; row < track_left_top_pin_rows; row++)
    {
        track_left[row] = 0;
    }
}


// 取像素 用一维偏移访问 避免每行重复做二维乘法
#define TRACK_PIXEL(image, row, col)    ((image)[(uint32)(row) * MT9V03X_W + (col)])


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某个像素是不是赛道(白)
// 参数说明     pixel               像素灰度值
// 返回参数     uint8               1:赛道(白)  0:背景(黑)
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_is_white (uint8 pixel)
{
    // 【灰度直读】原来是和固定值 TRACK_WHITE_LEVEL(128) 比，
    //   现在统一改成和 image_threshold 比 —— 这样无论传进来的是
    //   二值图(0/255) 还是灰度图，判断结果都一样，两种模式都兼容
    return (pixel >= IMG_WHITE_LEVEL) ? 1 : 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     起始行扫描：在最靠近车身的一行找到左右边界，作为种子点
// 对应文章     第二种寻线方式的 first_scan 函数
// 基本思路     1.从画面中心向左扫描 找 "黑黑白" 跳变点 该点右边第一个白点就是赛道左边界
//              2.从画面中心向右扫描 找 "白黑黑" 跳变点 该点左边第一个白点就是赛道右边界
// 参数说明     *image              二值化图像
// 参数说明     row                 起始行
// 返回参数     uint8               1:找到种子点  0:这一行一个跳变点都没有(整行全黑/全白)
//-------------------------------------------------------------------------------------------------------------------
// 【实车教训 1】原来本函数第一句是"画面中心那一个像素不是白 就 return 0 放弃整帧"。
//   实车截图证实：第 119 行(最靠近车身那行)中心处正好压着一小块暗斑(约 6 像素宽 灰度 128~135，
//   低于阈值 150)，于是整帧左右边线都保持初值 0 / 187、中线 94 ——
//   上位机画面上就是【三条笔直贴在画面边上的线】，寻线等于完全失效；
//   而且丢线标志只落在第 119 行（图像左下角、右下角各一个小黑横条，就是 TRACK_DEBUG_MARK 画的）。
//   现在改成：中心是黑的也照样往两边扫，三像素跳变判据自己就能跳过这一小段暗斑。
//
// 【实车教训 2】中心是黑的时候，紧挨着中心的那两个像素很容易凑出一个假跳变点
//   （暗斑的左沿正好满足 "白-黑-黑"），右边界会一下子停在中心附近(例如 93)，
//   这一帧后面的右边界就全错了。所以中心是黑的时，跳过"离中心 TRACK_SEED_SKIP 以内"的跳变点。
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_first_scan (uint8 *image, uint16 row)
{
    uint8 *row_p = image + (uint32)row * MT9V03X_W;                 // 【性能优化】行首指针 每行只算一次偏移
    int16 left  = -1;
    int16 right = -1;
    int16 c     = 0;
    uint8 seed_dark = 0;

    // 中心那一个像素是不是黑的（黑说明中心压着暗斑/黑点，不是"中心不在赛道上"）
    if(0 == track_is_white(row_p[TRACK_SEED_COL]))
    {
        seed_dark = 1;
    }

    // 从中心向左找 黑黑白 跳变点
    for(c = (int16)TRACK_SEED_COL; c >= 1; c--)
    {
        if((0 == track_is_white(row_p[c - 1])) &&
           (0 == track_is_white(row_p[c]))     &&
           (1 == track_is_white(row_p[c + 1])))
        {
            if((0 != seed_dark) &&
               (c >= ((int16)TRACK_SEED_COL - TRACK_SEED_SKIP)) &&
               (c <= ((int16)TRACK_SEED_COL + TRACK_SEED_SKIP)))
            {
                continue;                                           // 中心附近的跳变点是暗斑自己的沿 不是赛道边线
            }

            left = c + 1;                                           // 跳变点右边第一个白点就是左边界
            break;
        }
    }

    // 从中心向右找 白黑黑 跳变点
    for(c = (int16)TRACK_SEED_COL; c <= (MT9V03X_W - 2); c++)
    {
        if((1 == track_is_white(row_p[c - 1])) &&
           (0 == track_is_white(row_p[c]))     &&
           (0 == track_is_white(row_p[c + 1])))
        {
            if((0 != seed_dark) &&
               (c >= ((int16)TRACK_SEED_COL - TRACK_SEED_SKIP)) &&
               (c <= ((int16)TRACK_SEED_COL + TRACK_SEED_SKIP)))
            {
                continue;                                           // 同上
            }

            right = c - 1;                                          // 跳变点左边第一个白点就是右边界
            break;
        }
    }

    // 整行左右都没找到跳变点 -> 这一行根本没有赛道
    //   注意：只有"一个跳变点都没有"才算失败，不再因为"中心一个像素是黑的"就放弃整帧
    if((left < 0) && (right < 0))
    {
        return 0;
    }

    if(left  < 0)                                                   // 只丢一侧 就以图像边缘作为边界
    {
        left = 0;
    }
    if(right < 0)
    {
        right = MT9V03X_W - 1;
    }

    track_left[row]   = (uint8)left;
    track_right[row]  = (uint8)right;
    track_center[row] = (uint8)((left + right) / 2);

    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某个列是不是左边界(跳变点：左边是背景 当前是赛道)
// 参数说明     *image              二值化图像
// 参数说明     row                 行号
// 参数说明     c                   待判断的列号
// 返回参数     uint8               1:是左边界  0:不是
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_hit_left (uint8 *row_p, int16 c)
{
    if((c < 1) || (c > (MT9V03X_W - 2)))                            // 越界不算
    {
        return 0;
    }

    // 用文章的三像素判据 黑-黑-白：单像素的白噪点凑不出这个形状 抗散粒噪声更好
    // PC 仿真：散粒噪声 0.5%~1% 时 左线误差能减半(1.89->1.21 / 2.38->1.22 像素)
    //
    // 【性能优化】参数从 (image, row) 改成行首指针 row_p：
    //   原来每调用一次要做 3 次 (uint32)row*188+col 的 32 位乘法，
    //   而本函数一帧最多被调用 11×2×120 = 2640 次，合计约 8000 次乘法。
    //   改成指针之后乘法只剩"每行一次"
    return ((0 == track_is_white(row_p[c - 1])) &&
            (0 == track_is_white(row_p[c]))     &&
            (1 == track_is_white(row_p[c + 1]))) ? 1 : 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某个列是不是右边界(跳变点：当前是赛道 右边是背景)
// 参数说明     *image              二值化图像
// 参数说明     row                 行号
// 参数说明     c                   待判断的列号
// 返回参数     uint8               1:是右边界  0:不是
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_hit_right (uint8 *row_p, int16 c)
{
    if((c < 1) || (c > (MT9V03X_W - 2)))                            // 越界不算
    {
        return 0;
    }

    // 同左边界 用文章的三像素判据 白-白-黑
    // 【性能优化】同 track_hit_left，改成收行首指针，省掉每次调用的 3 次 32 位乘法
    return ((1 == track_is_white(row_p[c - 1])) &&
            (1 == track_is_white(row_p[c]))     &&
            (0 == track_is_white(row_p[c + 1]))) ? 1 : 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     全局扫描重新捕获左边界（对应文章的第一种寻线：由中线往左边扫黑白跳变点）
// 参数说明     *image              二值化图像
// 参数说明     row                 行号
// 参数说明     *out                找到的左边界
// 返回参数     uint8               1:找到  0:这一行没有可利用的跳变点（例如十字横杠整行全白）
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_global_left (uint8 *image, uint16 row, int16 *out)
{
    int16 c;
    uint8 *row_p = image + (uint32)row * MT9V03X_W;                 // 【性能优化】行首指针

    for(c = (MT9V03X_W / 2); c >= 2; c--)
    {
        if((0 == track_is_white(row_p[c - 1])) &&
           (0 == track_is_white(row_p[c]))     &&
           (1 == track_is_white(row_p[c + 1])))
        {
            *out = c + 1;
            return 1;
        }
    }

    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     全局扫描重新捕获右边界（左弯时允许从画面左半幅找回）
// 参数说明     *image              二值化图像
// 参数说明     row                 行号
// 参数说明     left_hint/right_hint 上一行的左右边界，用来决定左弯时的重捕获起点
// 参数说明     *out                找到的右边界
// 返回参数     uint8               1:找到  0:这一行没有可利用的跳变点
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_global_right (uint8 *image, uint16 row, int16 left_hint, int16 right_hint, int16 *out)
{
    int16 c;
    int16 start = MT9V03X_W / 2;
    uint8 left_bend = 0;
    uint8 *row_p = image + (uint32)row * MT9V03X_W;                 // 【性能优化】行首指针

    // 右边界已经弯过画面中点时，从左边界右侧重新找，不能再只扫右半幅。
    if((right_hint < (MT9V03X_W / 2)) && (left_hint < right_hint))
    {
        left_bend = 1;
        start = left_hint + 13;                                     // 给远处赛道至少留 12 像素宽
        if(start < 5) { start = 5; }
    }

    for(c = start; c <= (MT9V03X_W - 3); c++)
    {
        if((1 == track_is_white(row_p[c - 1])) &&
           (0 == track_is_white(row_p[c]))     &&
           (0 == track_is_white(row_p[c + 1])))
        {
            // 左半幅的黑白跳变点很多；候选边界内侧至少要有连续白色路面。
            if((0 != left_bend) &&
               ((0 == track_is_white(row_p[c - 2])) ||
                (0 == track_is_white(row_p[c - 3])) ||
                (0 == track_is_white(row_p[c - 4])) ||
                ((c - 1) < (right_hint - 36)) ||
                ((c - 1) > (right_hint + 8))))
            {
                continue;
            }
            *out = c - 1;
            return 1;
        }
    }

    return 0;
}

// Fit the last visible right-edge segment in Q8 so slopes below one pixel/row survive.
static uint8 track_right_predict (int16 row, int16 *predicted, int16 *trend_q)
{
    int16 first = -1;
    int16 y;
    int16 t;
    int16 value;
    int16 count = 0;
    int32 sum_t = 0;
    int32 sum_x = 0;
    int32 sum_tt = 0;
    int32 sum_tx = 0;
    int32 denominator;
    int32 slope_q;
    int32 base_q;
    int32 prediction_q;
    int32 error_q;

    for(y = row + 1; (y < MT9V03X_H) && (count < TRACK_RIGHT_TREND_ROWS); y++)
    {
        if(0 != (track_lost[y] & 0x02))
        {
            if(first >= 0) { break; }
            continue;
        }

        value = (int16)track_right[y];
        if((value <= 1) || (value >= (MT9V03X_W - 2)))
        {
            if(first >= 0) { break; }
            continue;
        }

        if(first < 0)
        {
            first = y;
            if((first - row) > TRACK_RIGHT_PREDICT_MAX_GAP) { return 0; }
        }

        t = y - first;
        sum_t += t;
        sum_x += value;
        sum_tt += (int32)t * t;
        sum_tx += (int32)t * value;
        count++;
    }

    if(count < TRACK_RIGHT_TREND_MIN_ROWS) { return 0; }
    denominator = (int32)count * sum_tt - sum_t * sum_t;
    if(denominator == 0) { return 0; }
    slope_q = (((int32)count * sum_tx - sum_t * sum_x) * 256) / denominator;
    if((slope_q > (TRACK_EDGE_SLOPE_MAX * 256L)) ||
       (slope_q < -(TRACK_EDGE_SLOPE_MAX * 256L))) { return 0; }

    base_q = (sum_x * 256 - slope_q * sum_t) / count;
    for(y = first; y < (first + count); y++)
    {
        error_q = (int32)track_right[y] * 256 -
                  (base_q + slope_q * (y - first));
        if((error_q > (3 * 256L)) || (error_q < -(3 * 256L))) { return 0; }
    }

    prediction_q = base_q + slope_q * (row - first);
    value = (int16)((prediction_q + 128) / 256);
    if((value < 2) || (value >= (MT9V03X_W - 2)) ||
       ((value - (int16)track_right[first]) > TRACK_RIGHT_PREDICT_MAX_SHIFT) ||
       (((int16)track_right[first] - value) > TRACK_RIGHT_PREDICT_MAX_SHIFT)) { return 0; }

    *predicted = value;
    if(0 != trend_q) { *trend_q = (int16)slope_q; }
    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     以种子点为基础生长出下一行的左右边界
// 对应文章     "以寻到的两个点为基础，框出一个3*3的矩形框，分别对左右两个点所在的矩阵框内
//              进行顺时针和逆时针扫描，找到跳变点，依次往下扫描"
// 基本思路     1.左边界：以上一行的左边界为种子 先在半径 0(原位)试 再逐圈扩大到 TRACK_SEED_BOX
//                      每一圈先试外侧列再试内侧列 找到 "左黑右白" 跳变点作为本行左边界
//              2.右边界：以上一行的右边界为种子 同样逐圈扩大 找到 "左白右黑" 跳变点
//              3.两边都丢失时保留上一行的值(画出来的线保持连续) 并记录丢失标志
// 参数说明     *image              二值化图像
// 参数说明     row                 本次要生长的行
// 参数说明     last_row            上一行(种子所在行)
// 返回参数     void
// 备注信息     1.参考文章代码里 last_left_line / last_right_line 在 Follow_scan 中没有更新，
//                会导致每一行都在同一个窗口里扫描(只能寻出一条竖线)，这里改成每行都更新种子
//              2.文章只给了 3*3 的文字描述(TRACK_SEED_BOX = 1)。实测二值化边界只要有一两个像素
//                的抖动 3*3 就会大面积跟丢，所以这里改成"先在 3*3 里找 找不到再逐圈扩大"，
//                既保留种子生长的思路 又能在急弯和噪声下跟住线
//-------------------------------------------------------------------------------------------------------------------
static void track_follow_scan (uint8 *image, uint16 row, uint16 last_row)
{
    uint8 *row_p     = image + (uint32)row * MT9V03X_W;             // 【性能优化】本行首指针，一行只算一次乘法
    int16 seed_left  = (int16)track_left[last_row];                 // 上一行左边界 作为本行种子
    int16 seed_right = (int16)track_right[last_row];                // 上一行右边界 作为本行种子
    int16 left       = seed_left;
    int16 right      = seed_right;
    int16 k          = 0;                                           // 当前搜索半径
    int16 predicted_right = 0;
    uint8 found_left  = 0;
    uint8 found_right = 0;
    uint8 has_prediction = 0;

    if((0 == ring_state) && (0 != (track_lost[last_row] & 0x02)))
    {
        has_prediction = track_right_predict((int16)row, &predicted_right, 0);
        if((0 != has_prediction) && (predicted_right > seed_left + 8))
        {
            seed_right = predicted_right;
            right = seed_right;
        }
        else
        {
            has_prediction = 0;
        }
    }

    //------------------------------ 左边界：半径由小到大逐圈扩大(逆时针) ------------------------------
    for(k = 0; k <= TRACK_SEED_BOX; k++)
    {
        if(0 == k)                                                  // 半径 0：边界没有移动 直接命中
        {
            if(0 != track_hit_left(row_p, seed_left))
            {
                left       = seed_left;
                found_left = 1;
                break;
            }
        }
        else                                                        // 先试外侧 再试内侧
        {
            if(0 != track_hit_left(row_p, seed_left - k))
            {
                left       = seed_left - k;
                found_left = 1;
                break;
            }

            if(0 != track_hit_left(row_p, seed_left + k))
            {
                left       = seed_left + k;
                found_left = 1;
                break;
            }
        }
    }

    //------------------------------ 右边界：半径由小到大逐圈扩大(顺时针) ------------------------------
    for(k = 0; k <= TRACK_SEED_BOX; k++)
    {
        if(0 == k)                                                  // 半径 0：边界没有移动 直接命中
        {
            if(0 != track_hit_right(row_p, seed_right))
            {
                right       = seed_right;
                found_right = 1;
                break;
            }
        }
        else                                                        // 先试外侧 再试内侧
        {
            if(0 != track_hit_right(row_p, seed_right + k))
            {
                right       = seed_right + k;
                found_right = 1;
                break;
            }

            if(0 != track_hit_right(row_p, seed_right - k))
            {
                right       = seed_right - k;
                found_right = 1;
                break;
            }
        }
    }

    //------------------------------ 异常与丢失处理 ------------------------------
    if((0 == found_right) && (0 == has_prediction) && (0 == ring_state))
    {
        has_prediction = track_right_predict((int16)row, &predicted_right, 0);
        if((0 == has_prediction) || (predicted_right <= left + 8))
        {
            has_prediction = 0;
        }
        else
        {
            seed_right = predicted_right;
        }
    }

    if((0 == found_left) || (0 == found_right) || (left >= right))
    {
        // 左右边界交叉说明两边都跟丢了 这一行保留上一行的值 保证画出来的线不断
        if((left >= right) || ((0 == found_left) && (0 == found_right)))
        {
            left  = seed_left;
            right = seed_right;
        }
        else if(0 == found_left)
        {
            left = seed_left;
        }
        else
        {
            right = seed_right;
        }
    }

    // ---- 局部生长失败时 连续丢失若干行就做一次全局扫描 重新捕获边线 ----
    // 对应文章的第一种寻线（由中线分别往左右两边扫 寻找黑白跳变点）
    // 没有重捕获的话 种子一旦跟丢（例如经过十字的横杠）就再也回不来 远处整片都会丢线
    if(0 == found_left)
    {
        track_lost_run[0]++;
        if(track_lost_run[0] >= TRACK_RECOVER_LOST)
        {
            int16 rec = 0;
            if(0 != track_global_left(image, row, &rec))
            {
                left            = rec;
                found_left      = 1;
                track_lost_run[0] = 0;
            }
        }
    }
    else
    {
        track_lost_run[0] = 0;
    }

    if(0 == found_right)
    {
        track_lost_run[1]++;
        if(track_lost_run[1] >= TRACK_RECOVER_LOST)
        {
            int16 rec = 0;
            if((0 != track_global_right(image, row, seed_left, seed_right, &rec)) &&
               ((0 == has_prediction) ||
                ((rec - predicted_right <= TRACK_RIGHT_REACQUIRE_TOL) &&
                 (predicted_right - rec <= TRACK_RIGHT_REACQUIRE_TOL))))
            {
                right             = rec;
                found_right       = 1;
                track_lost_run[1] = 0;
            }
        }
    }
    else
    {
        track_lost_run[1] = 0;
    }

    if(0 == found_left)
    {
        track_lost[row] |= 0x01;                                    // 记录本行左线丢失
        track_left_lost_count++;
    }

    if(0 == found_right)
    {
        track_lost[row] |= 0x02;                                    // 记录本行右线丢失
        track_right_lost_count++;
    }

    track_left[row]   = (uint8)left;
    track_right[row]  = (uint8)right;
    track_center[row] = (uint8)((left + right) / 2);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     种子生长法寻线
// 参数说明     *binary_image      二值化后的图像(0/255) 通常传 image_copy[0]
// 返回参数     void
// 使用示例     track_edge_process(image_copy[0]);
// 备注信息     结果保存在 track_left / track_right / track_center 中
//              下标是行号 存放的是该行的列坐标 可以直接交给逐飞助手显示边线
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     文章《基础寻线》第二种方法的一行扫描
// 参数说明     *image              二值化/灰度图
// 参数说明     row                 本行
// 参数说明     last_row            上一行(种子所在行)
// 返回参数     void
// 备注信息     和文章 Follow_scan 一致的地方：
//               1. 判据是"黑-白-白"（左）/ "白-白-黑"（右）
//               2. 丢线只做标记，【没有全局重捕获】
//               3. 搜索范围很宽（向外最多 TRACK_FOLLOW_OUT）
//
//             和文章不同的地方（都是实车暴露出来的问题）：
//               a. 【搜索顺序】文章是从窗口最左/最右端往中间扫，取最外端那个匹配；
//                  改成"从上一行的位置开始，先向外、再向内"，取【最近】的匹配。
//                  （实车现象：直道上绿线被旁边的深色物体抢走）
//               b. 【向内范围】文章向内允许找 3w（60像素），太大 ——
//                  赛道内部的地面接缝/阴影/反光同样能凑出判据，会抓到假边线。
//                  （实车现象：右边界向内跳 60 像素，黄线飞到画面左边）
//                  现在向内只给 TRACK_FOLLOW_IN（8像素）。
//
//             和文章原码的两处不同（都是原码的笔误/省略，照抄会出问题）：
//               a. 文章里 last_left_line / last_right_line 在 Follow_scan 里没有更新
//                  —— 那样每一行都在同一个窗口里扫，会寻出一条竖线。这里每行都更新种子。
//               b. 文章丢线时"不做赋值"，数组里会留下 0 或者上一帧的值（画面上一段竖线）。
//                  这里改成"保持上一行的值"，配合后面的丢线插值，线才是连续的。
//
//             另外文章只扫到第 40 行（远景区不扫），这里扫到第 0 行 —— 因为十字/圆环
//             模块要用到远处那些行。
//-------------------------------------------------------------------------------------------------------------------
static void track_follow_scan_article (uint8 *image, uint16 row, uint16 last_row)
{
    uint8 *row_p     = image + (uint32)row * MT9V03X_W;
    int16  last_left  = (int16)track_left[last_row];
    int16  last_right = (int16)track_right[last_row];
    int16  left       = last_left;
    int16  right      = last_right;
    int16  c;
    int16  k;
    uint8  found_left  = 0;
    uint8  found_right = 0;


    // 【搜索顺序】内、外两个方向【交替】按半径由小到大找，取最近的匹配
    //
    //   不能写成"先把外侧找完再找内侧" —— 那样如果真边线在内侧几像素处、
    //   而外侧几十像素处恰好有个干扰点，就会先抓到干扰点
    //   （实车现象：边线明显离干扰点更近，还是被干扰了）
    //
    //   范围仍然是不对称的：外侧给 TRACK_FOLLOW_OUT(60)，内侧只给 TRACK_FOLLOW_IN(8)
    //   因为边线可能突然向外张开（赛道变宽、过横杠），但不可能突然跳进赛道里面
    for(k = 0; k <= TRACK_FOLLOW_OUT; k++)
    {
        c = last_left - k;                                          // 外侧（往左）
        if((c >= 0) && (c <= (MT9V03X_W - 3)) &&
           (row_p[c] < IMG_WHITE_LEVEL) &&
           (row_p[c + 1] >= IMG_WHITE_LEVEL) &&
           (row_p[c + 2] >= IMG_WHITE_LEVEL))
        {
            left       = c;
            found_left = 1;
            break;
        }

        if(0 == k)
        {
            continue;                                               // k=0 时内、外是同一个位置，不重复试
        }

        if(k <= TRACK_FOLLOW_IN)                                    // 内侧（往右），范围小
        {
            c = last_left + k;
            if((c >= 0) && (c <= (MT9V03X_W - 3)) &&
               (row_p[c] < IMG_WHITE_LEVEL) &&
               (row_p[c + 1] >= IMG_WHITE_LEVEL) &&
               (row_p[c + 2] >= IMG_WHITE_LEVEL))
            {
                left       = c;
                found_left = 1;
                break;
            }
        }
    }


    // 【搜索顺序】同左边界：内、外交替，按半径由小到大取最近的匹配
    for(k = 0; k <= TRACK_FOLLOW_OUT; k++)
    {
        c = last_right + k;                                         // 外侧（往右）
        if((c >= 2) && (c <= (MT9V03X_W - 2)) &&                      // 上限收到 W-2：最后一列(187)恒为黑，会假命中
           (row_p[c - 2] >= IMG_WHITE_LEVEL) &&
           (row_p[c - 1] >= IMG_WHITE_LEVEL) &&
           (row_p[c]     <  IMG_WHITE_LEVEL))
        {
            right       = c;
            found_right = 1;
            break;
        }

        if(0 == k)
        {
            continue;
        }

        if(k <= TRACK_FOLLOW_IN)                                    // 内侧（往左），范围小
        {
            c = last_right - k;
            if((c >= 2) && (c <= (MT9V03X_W - 2)) &&                      // 上限收到 W-2：最后一列(187)恒为黑，会假命中
               (row_p[c - 2] >= IMG_WHITE_LEVEL) &&
               (row_p[c - 1] >= IMG_WHITE_LEVEL) &&
               (row_p[c]     <  IMG_WHITE_LEVEL))
            {
                right       = c;
                found_right = 1;
                break;
            }
        }
    }

    //------------------------------ 向外限速 ------------------------------
    // 【实车教训】上面两个搜索循环里"向外"给了 TRACK_FOLLOW_OUT(60) 像素的窗口，
    //   而"向内"只给 TRACK_FOLLOW_IN(8) —— 向外几乎不设限。
    //   后果：十字里横杠白区一张开，边线一步就窜到画面边缘
    //   （实车逐行量到：那一行两边界相距 166 像素 = 几乎整幅画面），
    //   画面上绿线贴左、黄线贴右、中线一根竖线，看起来就像"没补线"。
    //   物理上边线随行号只变化 0.5~1 像素/行，这里限到 TRACK_FOLLOW_OUT_STEP(3)。
    if(0 != found_left)
    {
        if(left < (last_left - TRACK_FOLLOW_OUT_STEP))
        {
            left = last_left - TRACK_FOLLOW_OUT_STEP;
        }
    }

    if(0 != found_right)
    {
        if(right > (last_right + TRACK_FOLLOW_OUT_STEP))
        {
            right = last_right + TRACK_FOLLOW_OUT_STEP;
        }
    }


    //------------------------------ 贴边时的全局重捕获 ------------------------------
    // 【实车教训】十字横杠把近处边线整段盖住时，起种只能把这一侧退到画面边缘(0 或 187)，
    //   而跟踪窗口"向内只给 TRACK_FOLLOW_IN(8) 像素"，横杠过去以后真正的边线在 40~60 列，
    //   永远追不上 —— 整段边线就一直贴在画面边上，中线跟着歪。
    //   所以：这一侧已经贴边时，允许"从画面中心整行重扫"把它抓回来。
    //   抓回来的结果必须能和对面那条边线形成合理宽度才认（防抓到画面里的深色物体）。
    if(0 == found_left)
    {
        if(last_left <= TRACK_ACQUIRE_NEAR)
        {
            int16 acq = -1;

            if(0 != track_global_left(image, row, &acq))
            {
                if(((int16)track_right[last_row] - acq) >= TRACK_ACQUIRE_MIN_WIDTH)
                {
                    // 限速：一次最多往右挪 TRACK_ACQUIRE_STEP 像素
                    //   不限速的话，边线会一步从 0 跳到 60 —— 画面上就是一个折角
                    if(acq > (last_left + TRACK_ACQUIRE_STEP))
                    {
                        acq = last_left + TRACK_ACQUIRE_STEP;
                    }

                    left       = acq;
                    found_left = 1;
                }
            }
        }
    }

    if(0 == found_right)
    {
        if(last_right >= (MT9V03X_W - 1 - TRACK_ACQUIRE_NEAR))
        {
            int16 acq = -1;

            if(0 != track_global_right(image, row, last_left, last_right, &acq))
            {
                if((acq - (int16)track_left[last_row]) >= TRACK_ACQUIRE_MIN_WIDTH)
                {
                    if(acq < (last_right - TRACK_ACQUIRE_STEP))             // 同样限速
                    {
                        acq = last_right - TRACK_ACQUIRE_STEP;
                    }

                    right       = acq;
                    found_right = 1;
                }
            }
        }
    }


    //------------------------------------------------------------------------------------------
    // 【实车教训 · MT9V03X 最后一列恒为黑】
    //   实测（原始帧逐像素量过）：第 176~186 列是正常图像内容，只有【第 187 列恒为 0(纯黑)】。
    //   于是"白-白-黑"判据在 c = 187 处永远命中（185、186 是白、187 是黑）：
    //     右边界每一行都能"找到"一条边(186)，永远不丢线；
    //     后果 1：右边线不跟真实边线，一直贴着画面右边；
    //     后果 2：十字预识别要求"左右同时丢线"，右边永远不丢 -> 十字永远进不去。
    //   所以右边界搜索的上限收到 MT9V03X_W - 2，不让最后一列参与判据。
    //------------------------------------------------------------------------------------------


    //------------------------------ 丢线处理：只标记，值保持上一行的 ------------------------------
    if(0 == found_left)
    {
        track_lost[row] |= 0x01;
        track_left_lost_count++;
        left = last_left;                                           // 保持上一行的值，画出来才连续
    }

    if(0 == found_right)
    {
        track_lost[row] |= 0x02;
        track_right_lost_count++;
        right = last_right;
    }

    if(left >= right)                                               // 左右交叉：两边都跟丢了
    {
        left  = last_left;
        right = last_right;
    }

    track_left[row]   = (uint8)left;
    track_right[row]  = (uint8)right;
    track_center[row] = (uint8)((left + right) / 2);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     隔行搜索的补行：没搜过的行取上下两行的平均值
// 返回参数     void
// 备注信息     和 TRACK_SEARCH_STEP 配套。
//              直道上边线位置是行号的线性函数，"上下平均"就是精确值；
//              曲线上跨度内的非线性会带来一点点插值误差，但两行的跨度很小。
//              丢线标志取上下两行的"或"：只要有一边丢了就算丢了，宁可保守。
//-------------------------------------------------------------------------------------------------------------------
static void track_fill_search_gap (void)
{
    uint16 row;

    if(TRACK_SEARCH_STEP <= 1)
    {
        return;                                                     // 逐行搜索时不需要补
    }

    for(row = 1; row < (MT9V03X_H - 1); row++)
    {
        // 判断这一行"有没有被搜过"：和起始行同余的才算搜过
        if(0 == (((uint16)TRACK_START_ROW - row) % (uint16)TRACK_SEARCH_STEP))
        {
            continue;                                               // 搜过的行，不动
        }

        track_left[row]   = (uint8)(((uint16)track_left[row - 1]  + (uint16)track_left[row + 1])  / 2);
        track_right[row]  = (uint8)(((uint16)track_right[row - 1] + (uint16)track_right[row + 1]) / 2);
        track_center[row] = (uint8)(((uint16)track_left[row] + (uint16)track_right[row]) / 2);
        track_lost[row]   = (uint8)(track_lost[row - 1] | track_lost[row + 1]);
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某一行是不是"整行全白"（一个暗点都没有）
// 参数说明     *image              图像
// 参数说明     row                 行号
// 返回参数     uint8               1:整行全白  0:还有暗点
// 备注信息     整行全白 = 这一行上赛道比视野还宽，左右两条边线都在画面外了 ——
//              这种情况两边都该贴到画面边上（0 / 187），而不是死等种子。
//              实车现象：车头前方一大片纯白时，最下面好几行一个跳变点都找不到，
//              原来会因为"起种失败"把【整帧】都置成初值（三条线全是贴边的直线）。
//-------------------------------------------------------------------------------------------------------------------
static uint8 track_row_all_white (uint8 *image, uint16 row)
{
    uint8 *row_p = image + (uint32)row * MT9V03X_W;
    int16  x;

    for(x = 0; x < MT9V03X_W; x++)
    {
        if(row_p[x] < IMG_WHITE_LEVEL)
        {
            return 0;                                               // 找到暗点 不是全白
        }
    }

    return 1;
}



void track_edge_process (uint8 *binary_image)
{
    uint16 row      = 0;
    uint16 last_row = 0;

    // 先全部清成默认值 防止上一帧的数据残留在没扫到的行里
    for(row = 0; row < MT9V03X_H; row++)
    {
        track_left[row]   = 0;
        track_right[row]  = MT9V03X_W - 1;
        track_center[row] = MT9V03X_W / 2;
        track_lost[row]   = 0;
    }
    track_lost_run[0] = 0;
    track_lost_run[1] = 0;
    track_left_lost_count  = 0;
    track_right_lost_count = 0;

    // 第 1 步：在最靠近车身的一行找种子点
    //   【实车教训】起始行本身找不到赛道时（例如第 119 行中心压着一小块暗斑、
    //   或者车身/阴影刚从画面最下面进来一条），不要立刻放弃整帧 ——
    //   往远一点的方向再试 TRACK_SEED_RETRY 行，找到之后把起始行到起种行之间那几行
    //   直接沿用起种行的结果：近处几行的赛道宽度变化极慢，沿用不会有肉眼可见的误差，
    //   不沿用的话那几行会停在初值(左 0 / 右 187)，中线在图像最下面会突然跳一下。
    {
        uint16 seed_row = (uint16)TRACK_START_ROW;
        uint16 try_row;
        uint8  seed_ok = 0;
        int16  seed_k;

        for(seed_k = 0; seed_k <= TRACK_SEED_RETRY; seed_k++)
        {
            try_row = (uint16)((int16)TRACK_START_ROW + (int16)TRACK_ROW_STEP * seed_k);

            if(try_row >= MT9V03X_H)                                // 越界(反装时才会走到)
            {
                break;
            }

            if(0 != track_first_scan(binary_image, try_row))
            {
                if(0 == seed_ok)                                    // 第一个能起种的行先记住 当兜底
                {
                    seed_row = try_row;
                    seed_ok  = 1;
                }

                // 【实车教训】图像最下面那一行中心偶尔压着一小块暗斑（灰度 118~135）。
                //   暗斑【不在中心正下方、而是在旁边一两列】时，"白-黑-黑"判据会在暗斑边沿命中，
                //   右边界一下子停在中心附近（例如 95），这一行量出来的"赛道宽度"就只有 70~80 像素；
                //   而近处这一行的赛道本来是最宽的（实车实测 123~152 像素）。
                //   所以起种时用宽度做体检：
                //     宽度够   -> 就用这一行
                //     宽度不够 -> 说明边界被暗斑骗了，继续往上试一行（上面的行不受这块暗斑影响）
                //   注意：万一往上试完都没有"够宽"的行，仍然用第一个能起种的行 ——
                //         绝不能因为这个体检把整帧废掉。
                if(((int16)track_right[try_row] - (int16)track_left[try_row]) >= TRACK_SEED_MIN_WIDTH)
                {
                    seed_row = try_row;
                    break;
                }
            }
        }

        if(0 == seed_ok)                                            // 这几行都没有赛道 这一帧放弃
        {
            track_lost[TRACK_START_ROW] = 0x03;
            track_left_lost_count       = 1;
            track_right_lost_count      = 1;
            return;
        }

        for(row = TRACK_START_ROW; row != seed_row; row = (uint16)((int16)row + (int16)TRACK_ROW_STEP))
        {
            if(0 != track_row_all_white(binary_image, row))         // 整行全白：赛道比视野还宽 两边都出画面了
            {
                track_left[row]   = 0;                              // 先给个"贴边"的初值
                track_right[row]  = MT9V03X_W - 1;
                track_center[row] = MT9V03X_W / 2;
                track_lost[row]  |= 0x03;                           // 【重要】标成丢线
                                                                    // 这样它会和别的丢线段一样【被统一处理】
                                                                    // （桥接或插值），而不是每帧一个形状
                continue;
            }

            // 不是全白（例如只是中心压了一小块暗斑、或者暗斑把跳变判据挡住了）：
            //   沿用起种行的结果。近处几行赛道宽度变化极慢，沿用不会有肉眼可见的误差。
            track_left[row]   = track_left[seed_row];
            track_right[row]  = track_right[seed_row];
            track_center[row] = track_center[seed_row];
            track_lost[row]   = 0;
        }
    }

    // 第 2 步：以种子点为基础往远处生长，每次走 TRACK_SEARCH_STEP 行（隔行搜索）
    last_row = TRACK_START_ROW;
    row      = TRACK_START_ROW;

    for(;;)
    {
        int16 next;

        next = (int16)row + (int16)TRACK_ROW_STEP * (int16)TRACK_SEARCH_STEP;

        if(next < (int16)TRACK_END_ROW)                             // 夹到终点行，保证最后一行一定被搜到
        {
            next = (int16)TRACK_END_ROW;
        }

        if(next == (int16)row)
        {
            break;
        }

        row = (uint16)next;
        if(2 == TRACK_METHOD)
        {
            track_follow_scan_article(binary_image, row, last_row); // 文章第二种方法（宽窗口 + 无重捕获）
        }
        else
        {
            track_follow_scan(binary_image, row, last_row);         // 我们原来的方法（窄窗口 + 全局重捕获）
        }
        last_row = row;

        if(row == TRACK_END_ROW)
        {
            break;
        }
    }

    // 第 3 步：没搜过的行用上下两行平均补上（隔行搜索时才有活干）
    track_fill_search_gap();
}


// 中值滤波用的临时数组
static uint8 track_temp[MT9V03X_H];


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把一段连续丢线的行 用上下两端的有效边线值线性插值补上
// 参数说明     *edge               边线数组(track_left 或 track_right)
// 参数说明     mask                丢线标志位 BIT0:左线 BIT1:右线
// 返回参数     void
// 备注信息     寻线在丢线时是"保持上一行的值" 画出来就是一截竖线 而且上下两端还接不上 看着是歪的
//              这里把每一段连续丢线的行用它两端(远端/近端)的有效值线性插值补上 边线就连续了
//              必须在十字补线之后调用：十字补线是靠原始丢线标志决定补哪些行的
//-------------------------------------------------------------------------------------------------------------------
static void track_fill_lost_side (uint8 *edge, uint8 mask)
{
    int16 row;
    int16 run_start = -1;
    int16 a;
    int16 b;
    int16 y;
    int16 xa;
    int16 xb;
    int16 value;
    int16 k;                                                        // 趋势(每行变多少像素)
    int16 right_trend_q;
    int16 predicted_right;
    int16 upper_y;
    int16 projected_upper;
    int32 slope_q;
    uint8 use_right_trend;

    for(row = 0; row <= MT9V03X_H; row++)
    {
        if((row < MT9V03X_H) && (0 != (track_lost[row] & mask)))
        {
            if(run_start < 0)
            {
                run_start = row;                                    // 一段连续丢线开始
            }
            continue;
        }

        if(run_start < 0)
        {
            continue;
        }

        a = run_start;
        b = row - 1;                                                // 丢线区间 [a, b]
        xa = (a > 0) ? (int16)edge[a - 1] : -1;                     // 远端那一端的有效值
        xb = (b < (MT9V03X_H - 1)) ? (int16)edge[b + 1] : -1;       // 近端那一端的有效值
        use_right_trend = 0;
        if((0x02 == mask) && (xb >= 0) && (0 == ring_state) &&
           ((xa < 0) || (0 != cross_bridge_used)))
        {
            use_right_trend = track_right_predict(a, &predicted_right, &right_trend_q);
            // Replace a bridged endpoint only if the real edge above the bar agrees.
            if((xa >= 0) && (0 != use_right_trend) &&
               (0 != cross_bridge_used) && (cross_bridge_top > 0) &&
               (cross_bridge_bottom == (a - 1)) &&
               (((xa - predicted_right) > TRACK_RIGHT_CROSS_END_TOL) ||
                ((predicted_right - xa) > TRACK_RIGHT_CROSS_END_TOL)))
            {
                upper_y = cross_bridge_top - 1;
                projected_upper = xb + (int16)(((int32)right_trend_q *
                                      (upper_y - (b + 1))) / 256);
                if(((b + 1 - upper_y) <= TRACK_RIGHT_CROSS_MAX_SPAN) &&
                   (0 == (track_lost[upper_y] & 0x02)) &&
                   (((int16)edge[upper_y] - projected_upper) <= TRACK_RIGHT_CROSS_JOIN_TOL) &&
                   ((projected_upper - (int16)edge[upper_y]) <= TRACK_RIGHT_CROSS_JOIN_TOL))
                {
                    for(y = cross_bridge_top; y < a; y++)
                    {
                        value = xb + (int16)(((int32)right_trend_q *
                                             (y - (b + 1))) / 256);
                        if(value < 0) { value = 0; }
                        if(value > (MT9V03X_W - 1)) { value = MT9V03X_W - 1; }
                        edge[y] = (uint8)value;
                    }
                    xa = -1;
                }
            }
        }

        // 左边线到达最左侧后若向上丢线，整段沿图像边界竖直补齐。
        // 只在普通弯道使用；十字和圆环仍由各自的补线逻辑处理。
        if((0x01 == mask) && (0 == a) &&
           (xb >= 0) && (xb <= TRACK_LEFT_TOP_PIN_COL) &&
           (0 == cross_state) && (0 == ring_state))
        {
            track_left_top_pin_rows = (uint8)(b + 1);
            for(y = a; y <= b; y++)
            {
                edge[y] = 0;
            }
            run_start = -1;
            continue;
        }

        track_interp_count += (uint8)(b - a + 1);                   // 统计被补上的行数

        if((xa >= 0) && (xb >= 0))
        {
            for(y = a; y <= b; y++)                                 // 两端都有效 线性插值
            {
                value = xa + (int16)(((int32)(xb - xa) * (y - (a - 1))) / ((b + 1) - (a - 1)));

                if(value < 0)
                {
                    value = 0;
                }
                if(value > (MT9V03X_W - 1))
                {
                    value = MT9V03X_W - 1;
                }

                edge[y] = (uint8)value;
            }
        }
        else if(xb >= 0)                                            // 上端不可靠或丢到图像顶端：按趋势延伸
        {
            // 【实车教训】原来是 edge[y] = xb（保持不变），画出来是一根竖直线 + 上下两个折角
            //   （实测：十字里一张帧第 0~9 行数值完全一样）。改成用近端那几行的趋势延伸。
            slope_q = 0;
            if(0 != use_right_trend)
            {
                slope_q = right_trend_q;
            }
            else
            {
                k = 0;
                for(y = (b + 2); (y < MT9V03X_H) && (y <= (b + 6)); y++) // 跳过 xb 自己，避免除以 0
                {
                    if(0 != (track_lost[y] & mask))
                    {
                        continue;
                    }

                    k = ((int16)edge[y] - (int16)xb) / (int16)(y - (b + 1));
                    break;
                }

                if(k > TRACK_EDGE_SLOPE_MAX)  { k = TRACK_EDGE_SLOPE_MAX; }
                if(k < -TRACK_EDGE_SLOPE_MAX) { k = -TRACK_EDGE_SLOPE_MAX; }
                slope_q = (int32)k * 256;
            }

            for(y = a; y <= b; y++)
            {
                value = (int16)xb + (int16)((slope_q * (y - (b + 1))) / 256);

                if(value < 0)
                {
                    value = 0;
                }
                if(value > (MT9V03X_W - 1))
                {
                    value = MT9V03X_W - 1;
                }

                edge[y] = (uint8)value;
            }
        }
        else if(xa >= 0)                                            // 一直丢到画面最下面：同样按趋势【延伸】
        {
            k = 0;
            for(y = (a - 2); (y >= 0) && (y >= (a - 6)); y--)        // 跳过 xa 自己，避免除以 0
            {
                if(0 != (track_lost[y] & mask))
                {
                    continue;
                }

                k = ((int16)xa - (int16)edge[y]) / (int16)((a - 1) - y);
                break;
            }

            if(k > TRACK_EDGE_SLOPE_MAX)  { k = TRACK_EDGE_SLOPE_MAX; }
            if(k < -TRACK_EDGE_SLOPE_MAX) { k = -TRACK_EDGE_SLOPE_MAX; }

            for(y = a; y <= b; y++)
            {
                value = (int16)xa + k * (int16)(y - (a - 1));

                if(value < 0)
                {
                    value = 0;
                }
                if(value > (MT9V03X_W - 1))
                {
                    value = MT9V03X_W - 1;
                }

                edge[y] = (uint8)value;
            }
        }

        run_start = -1;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     边线 3 点中值滤波 去掉单行的毛刺
// 参数说明     *edge               边线数组
// 返回参数     void
// 备注信息     文章里也提到过"图像锯齿"的问题 单行的毛刺会让中线看起来一跳一跳的
//-------------------------------------------------------------------------------------------------------------------
static void track_median_side (uint8 *edge)
{
    int16 row;
    uint8 a;
    uint8 b;
    uint8 c;
    uint8 mid;

    for(row = 0; row < MT9V03X_H; row++)
    {
        a = edge[(row > 0) ? (row - 1) : row];
        b = edge[row];
        c = edge[(row < (MT9V03X_H - 1)) ? (row + 1) : row];

        if(((a <= b) && (b <= c)) || ((c <= b) && (b <= a)))
        {
            mid = b;                                                // b 在中间
        }
        else if(((b <= a) && (a <= c)) || ((c <= a) && (a <= b)))
        {
            mid = a;                                                // a 在中间
        }
        else
        {
            mid = c;                                                // c 在中间
        }

        track_temp[row] = mid;
    }

    for(row = 0; row < MT9V03X_H; row++)
    {
        edge[row] = track_temp[row];
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     边线后处理：丢线行插值 + 去毛刺 + 重算中线
// 返回参数     void
// 使用示例     track_edge_process(image_copy[0]);
//              cross_road_process(image_copy[0]);
//              track_line_smooth();                  // 放在十字补线之后
// 备注信息     调用之后 track_left / track_right / track_center 就是最终结果
//-------------------------------------------------------------------------------------------------------------------
void track_line_smooth (void)
{
    track_interp_count = 0;
    track_left_top_pin_rows = 0;

    track_fill_lost_side(track_left,  0x01);                        // 左线丢线行插值
    track_fill_lost_side(track_right, 0x02);                        // 右线丢线行插值

#if(1 == TRACK_MEDIAN_ENABLE)
    track_median_side(track_left);                                  // 去掉单行毛刺
    track_median_side(track_right);
#endif

    track_lock_left_top_rows();
    track_center_update();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把"丢线"的行在图像上标出来（调试用）
// 参数说明     *binary_image      二值化图像(就是要发给上位机的那一幅)
// 返回参数     void
// 使用示例     track_lost_mark(image_copy[0]);
// 备注信息     左线丢线的行 在左边界处画一小段黑横条；右线丢线的行同理
//              上位机图像上看到黑横条的位置 就是寻线没找到边线的那些行
//              它是在所有寻线/补线处理之后才画的 不影响任何计算结果 只是给人看的
//-------------------------------------------------------------------------------------------------------------------
void track_lost_mark (uint8 *binary_image)
{
    uint16 row;
    int16  i;
    int16  x;

    for(row = 0; row < MT9V03X_H; row++)
    {
        if(0 != (track_lost[row] & 0x01))                           // 左线丢线的行
        {
            x = (int16)track_left[row];
            for(i = 0; i < TRACK_MARK_LEN; i++)
            {
                if((x + i) < MT9V03X_W)
                {
                    TRACK_PIXEL(binary_image, row, x + i) = 0;
                }
            }
        }

        if(0 != (track_lost[row] & 0x02))                           // 右线丢线的行
        {
            x = (int16)track_right[row];
            for(i = 0; i < TRACK_MARK_LEN; i++)
            {
                if((x - i) >= 0)
                {
                    TRACK_PIXEL(binary_image, row, x - i) = 0;
                }
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     连续丢线很多行、而且已经贴近画面边缘的边线，整段贴到画面边上
// 参数说明     *edge               边线数组
// 参数说明     mask                丢线标志位
// 参数说明     pin_value           贴边贴到哪一列（左线贴 0，右线贴 187）
// 返回参数     void
// 备注信息     连续丢线很多行、并且丢线前那一段已经贴在画面边缘 -> 说明这条边线已经跑出画面了
//              这种情况应该把整段直接贴到画面边上，而不是保持上一行的值
//              （保持的话边线会停在画面中间"乱飘"，看起来就是歪的）
//              判断"贴近边缘"是为了保护十字/圆环：那两种情况丢线段的端点在画面中间，
//              不能贴边
//-------------------------------------------------------------------------------------------------------------------
static void track_pin_lost_side (uint8 *edge, uint8 mask, uint8 pin_value)
{
    int16 row;
    int16 start = -1;
    int16 n;
    int16 y;
    uint8 near_edge = 0;

    for(row = 0; row <= MT9V03X_H; row++)
    {
        if((row < MT9V03X_H) && (0 != (track_lost[row] & mask)))
        {
            if(start < 0)
            {
                start = row;
            }
            continue;
        }

        if(start < 0)
        {
            continue;
        }

        n = row - start;                                            // 这一段丢了 n 行

        // 【修复】判断"贴不贴边"要看丢线段【两端】的有效值，只要有一端贴近画面边缘就算贴边
        //
        //   原来是只看 edge[start-1]，也就是丢线段【上方】那一端 —— 那是错的：
        //   边线跑出画面通常是从【靠近车身的那一端】开始的，
        //   结果本该贴边的段被判成"不贴边"，后处理把它插值成一条斜线
        //   （实车现象：第 119 行左边线没找到，绿线却不是贴在 x=0 上，而是一条斜线）
        near_edge = 0;

        if(start > 0)                                               // 上端那个有效值
        {
            if(0 != pin_value)                                      // 右线：贴近右边缘？
            {
                if((int16)edge[start - 1] >= (MT9V03X_W - 1 - TRACK_EDGE_PIN_NEAR))
                {
                    near_edge = 1;
                }
            }
            else                                                    // 左线：贴近左边缘？
            {
                if((int16)edge[start - 1] <= TRACK_EDGE_PIN_NEAR)
                {
                    near_edge = 1;
                }
            }
        }

        if(row < MT9V03X_H)                                         // 下端那个有效值（row 是这一段丢线下面的第一个有效行）
        {
            if(0 != pin_value)                                      // 右线
            {
                if((int16)edge[row] >= (MT9V03X_W - 1 - TRACK_EDGE_PIN_NEAR))
                {
                    near_edge = 1;
                }
            }
            else                                                    // 左线
            {
                if((int16)edge[row] <= TRACK_EDGE_PIN_NEAR)
                {
                    near_edge = 1;
                }
            }
        }

        // 【触发条件①】丢线段必须一直连到画面最下面一行（第 119 行，最靠近车身那行）
        //   物理含义：边线"跑出画面"一定是从靠近车身那端就开始看不到线的，
        //             所以丢线段必然连到最下面一行。
        //   反过来，如果丢线段是在画面中间断的（下面还有有效边线），那只是【暂时丢线】，
        //   应该交给十字/圆环补线或者插值去处理，不能贴边 ——
        //   否则会把十字上半段的左边界整段贴到 x=0，十字补线就没法做了。
        //   （实车现象：进十字时左丢线 48~59 行，左边界被整段贴到 x=0，只剩一条竖线）
        if(row < MT9V03X_H)
        {
            near_edge = 0;
        }

        if((n >= TRACK_EDGE_PIN_ROWS) && (0 != near_edge))
        {
            for(y = start; y < row; y++)
            {
                edge[y] = pin_value;                                // 整段贴到画面边上
            }
        }

        start = -1;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把"跑出画面"的边线整段贴到画面边上（左右线都处理）
// 返回参数     void
// 使用示例     track_pin_lost_edges();
// 备注信息     放在 track_line_smooth() 之后调用：先让插值/滤波把边线弄平滑，
//              再把那些确实跑出画面的长丢线段贴到边上去
//-------------------------------------------------------------------------------------------------------------------
void track_pin_lost_edges (void)
{
    // 【触发条件②】十字 / 圆环状态下【完全不贴边】
    //   那两种情况下的长丢线段是"道路被横杠盖住"或者"道路向一侧张开"，
    //   要靠十字/圆环模块自己的补线去处理，贴边会把它们的成果冲掉。
    if((0 != cross_state) || (0 != ring_state))
    {
        return;
    }

    track_pin_lost_side(track_left,  0x01, 0);                      // 左线贴到第 0 列
    track_pin_lost_side(track_right, 0x02, MT9V03X_W - 1);          // 右线贴到最后一列

    track_center_update();                                          // 贴完刷新中线
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     帧间稳定：左右边线 3 帧"时间中值"
// 返回参数     void
// 备注信息     只平滑左右边线，中线最后重算 —— 保证 中线 = (左+右)/2 永远成立。
//              前两帧只是存历史（还没凑够 3 帧，输出保持原样）。
//-------------------------------------------------------------------------------------------------------------------
#if(1 == TRACK_TEMPORAL_ENABLE)
static uint8 track_left_hist[3][MT9V03X_H];
static uint8 track_right_hist[3][MT9V03X_H];
static uint8 track_hist_idx = 0;
static uint8 track_hist_cnt = 0;
#endif

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     最终空间平滑：一条边线做滑动平均
// 参数说明     *edge               边线数组
// 返回参数     void
// 备注信息     治"台阶"：5~10 像素的横移被摊成几行的小斜坡。
//              只在这一条边线上做，中线最后重算（保证 中线 = (左+右)/2）。
//-------------------------------------------------------------------------------------------------------------------
#if(TRACK_FINAL_SMOOTH_WIN > 1)
static uint8 track_smooth_tmp[MT9V03X_H];

static void track_smooth_side_final (uint8 *edge)
{
    int16 row;
    int16 k;
    int16 y;
    int16 sum;
    int16 n;
    int16 half = (int16)(TRACK_FINAL_SMOOTH_WIN / 2);

    for(row = 0; row < MT9V03X_H; row++)
    {
        sum = 0;
        n   = 0;

        for(k = -half; k <= half; k++)
        {
            y = (int16)row + k;

            if((y < 0) || (y >= MT9V03X_H))
            {
                continue;                                           // 越界的点不参与（不然边上会被拉向 0）
            }

            sum += (int16)edge[y];
            n++;
        }

        track_smooth_tmp[row] = (uint8)(sum / n);
    }

    for(row = 0; row < MT9V03X_H; row++)
    {
        edge[row] = track_smooth_tmp[row];
    }
}
#endif


void track_temporal_filter (void)
{
#if(1 == TRACK_TEMPORAL_ENABLE)
    uint16 row;
    uint8  i0;
    uint8  i1;
    uint8  i2;
    uint8  a;
    uint8  b;
    uint8  c;
    uint8  tmp;

    i0 = track_hist_idx;
    i1 = (uint8)(i0 + 1);
    if(i1 >= 3) { i1 = 0; }
    i2 = (uint8)(i1 + 1);
    if(i2 >= 3) { i2 = 0; }

    for(row = 0; row < MT9V03X_H; row++)                            // 先把这一帧存进历史
    {
        track_left_hist[i0][row]  = track_left[row];
        track_right_hist[i0][row] = track_right[row];
    }

    if(track_hist_cnt >= 3)                                         // 凑够 3 帧才开始输出中值
    {
        for(row = 0; row < MT9V03X_H; row++)
        {
            a = track_left_hist[i0][row];                           // 左边界 3 帧取中值
            b = track_left_hist[i1][row];
            c = track_left_hist[i2][row];

            if(a > b) { tmp = a; a = b; b = tmp; }
            if(b > c) { tmp = b; b = c; c = tmp; }
            if(a > b) { tmp = a; a = b; b = tmp; }

            track_left[row] = b;

            a = track_right_hist[i0][row];                          // 右边界同样
            b = track_right_hist[i1][row];
            c = track_right_hist[i2][row];

            if(a > b) { tmp = a; a = b; b = tmp; }
            if(b > c) { tmp = b; b = c; c = tmp; }
            if(a > b) { tmp = a; a = b; b = tmp; }

            track_right[row] = b;
        }

        track_center_update();
    }
    else
    {
        track_hist_cnt++;
    }

    track_hist_idx = i1;
#endif

#if(TRACK_FINAL_SMOOTH_WIN > 1)
    // 最后再摊一次台阶（时间中值之后做，因为它自己也可能拼出台阶）
    track_smooth_side_final(track_left);
    track_smooth_side_final(track_right);
    track_center_update();
#endif

    if(0 != track_left_top_pin_rows)
    {
        track_lock_left_top_rows();
        track_center_update();
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从图像底端跟踪连通的白色赛道，最终三条轨迹不允许落在黑色像素上
// 备注信息     从画面中心所在白区起步；仅当前车道左边界碰到左端时才保持 x=0。
//              左端变黑后沿同一白区的左缘右转，十字全白行保持原车道边界。
//              若白区断开，远处行标为无效(255)，不跨黑色区域补线。
//-------------------------------------------------------------------------------------------------------------------
void track_white_lane_guard (uint8 *image)
{
    int16 row;
    int16 x;
    int16 start;
    int16 end;
    int16 best_l;
    int16 best_r;
    int16 best_dist;
    int16 best_overlap;
    int16 distance;
    int16 overlap;
    int16 reference;
    int16 guide_l;
    int16 guide_r;
    int16 prev_l = -1;
    int16 prev_r = -1;
    int16 prev_c = TRACK_SEED_COL;
    uint8 pin_left = 0;
    uint8 left_white;
    uint8 *row_p;

    for(row = MT9V03X_H - 1; row >= 0; row--)
    {
        row_p = image + (uint32)row * MT9V03X_W;
        left_white = track_is_white(row_p[0]);
        reference = (prev_l < 0) ? TRACK_SEED_COL : prev_c;
        if((reference < 0) || (reference >= MT9V03X_W)) { reference = TRACK_SEED_COL; }
        start = -1;
        best_l = -1;
        best_r = -1;
        best_dist = MT9V03X_W + 1;
        best_overlap = -1;

        for(x = 0; x <= MT9V03X_W; x++)
        {
            if((x < MT9V03X_W) && (0 != track_is_white(row_p[x])))
            {
                if(start < 0) { start = x; }
                continue;
            }
            if(start < 0) { continue; }
            end = x - 1;
            if((end - start + 1) < TRACK_WHITE_RUN_MIN) { start = -1; continue; }
            if((prev_l >= 0) && ((end < (prev_l - 1)) || (start > (prev_r + 1))))
            {
                start = -1;
                continue;
            }

            // 左端已连到赛道时，白就继续竖直走；刚转黑就选紧挨左端的白段。
            if((0 != pin_left) && (0 != left_white))
            {
                if(0 == start) { best_l = start; best_r = end; break; }
                start = -1;
                continue;
            }
            if((0 != pin_left) && (0 == left_white))
            {
                if((best_l < 0) || (start < best_l)) { best_l = start; best_r = end; }
                start = -1;
                continue;
            }

            distance = 0;
            if(reference < start) { distance = start - reference; }
            else if(reference > end) { distance = reference - end; }
            overlap = (prev_l < 0) ? (end - start + 1) :
                      (((end < prev_r) ? end : prev_r) - ((start > prev_l) ? start : prev_l) + 1);
            if(overlap < 0) { overlap = 0; }
            if((best_l < 0) || (distance < best_dist) ||
               ((distance == best_dist) && (overlap > best_overlap)))
            {
                best_l = start;
                best_r = end;
                best_dist = distance;
                best_overlap = overlap;
            }
            start = -1;
        }

        if(best_l < 0)
        {
            for(; row >= 0; row--)
            {
                track_left[row] = 255;
                track_right[row] = 255;
                track_center[row] = 255;
                track_lost[row] |= 0x03;
            }
            break;
        }

        guide_l = track_left[row];
        guide_r = track_right[row];
        // 只有双端确认的十字桥接才沿用插值斜率，且三条线必须仍在同一白区。
        if((0 != cross_flag) && (0 != cross_bridge_used) && (0 != cross_bridge_two_end) &&
           (row >= cross_bridge_top - 1) && (row <= cross_bridge_bottom + 1) &&
           (guide_l >= best_l) && (guide_l < guide_r) && (guide_r <= best_r) &&
           (guide_r < MT9V03X_W) &&
           (0 != track_is_white(row_p[guide_l])) &&
           (0 != track_is_white(row_p[(guide_l + guide_r) / 2])) &&
           (0 != track_is_white(row_p[guide_r])) &&
           ((prev_l < 0) ||
            (((guide_l - prev_l) <= TRACK_WHITE_EDGE_STEP_MAX) &&
             ((prev_l - guide_l) <= TRACK_WHITE_EDGE_STEP_MAX) &&
             ((guide_r - prev_r) <= TRACK_WHITE_EDGE_STEP_MAX) &&
             ((prev_r - guide_r) <= TRACK_WHITE_EDGE_STEP_MAX))))
        {
            best_l = guide_l;
            best_r = guide_r;
        }
        // 十字的整行白色会把左右边界接成一片，不能因此跳到图像边框。
        else if(prev_l >= 0)
        {
            if((0 == best_l) && (prev_l > TRACK_WHITE_EDGE_STEP_MAX) &&
               (0 != track_is_white(row_p[prev_l])))
            {
                best_l = prev_l;
            }
            if((MT9V03X_W - 1 == best_r) &&
               (prev_r < MT9V03X_W - 1 - TRACK_WHITE_EDGE_STEP_MAX) &&
               (0 != track_is_white(row_p[prev_r])))
            {
                best_r = prev_r;
            }
        }

        track_left[row] = (uint8)best_l;
        track_right[row] = (uint8)best_r;
        track_center[row] = (uint8)((best_l + best_r) / 2);
        prev_l = best_l;
        prev_r = best_r;
        prev_c = (best_l + best_r) / 2;
        pin_left = (0 == best_l) ? 1 : 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     根据 track_left / track_right 重新计算 track_center
// 返回参数     void
// 使用示例     track_center_update();
// 备注信息     十字补线改写了左右边线之后需要调用它刷新中线
//-------------------------------------------------------------------------------------------------------------------
void track_center_update (void)
{
    uint16 row;

    for(row = 0; row < MT9V03X_H; row++)
    {
        track_center[row] = (uint8)(((uint16)track_left[row] + (uint16)track_right[row]) / 2);
    }
}
