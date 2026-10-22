/*********************************************************************************************************************
* 文件名称          cross_road.c
* 功能简介          十字路口（横杠）识别与补线 —— 极简版（设计说明见 cross_road.h）
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "cross_road.h"
#include "track_edge.h"
#include "image_process.h"

//------------------------------------------------------------------------------------------------------------------
// 对外变量
//------------------------------------------------------------------------------------------------------------------
uint8 cross_flag        = 0;
uint8 cross_state       = 0;
uint8 cross_bridge_used = 0;
uint8 cross_bridge_two_end = 0;
int16 cross_bridge_top = -1;
int16 cross_bridge_bottom = -1;
uint8 cross_dbg_up_l    = 0;
uint8 cross_dbg_down_l  = 0;

static uint16 cross_state_count = 0;
static uint8  cross_row_white[MT9V03X_H];                           // 每行"是不是横杠行"的缓存(每帧算一次)


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某一行是不是"横杠行"（几乎整行都是白）
// 参数说明     *image              图像
// 参数说明     row                 行号
// 返回参数     uint8               1:是横杠行
// 备注信息     抽样统计这一行的白点比例。非白行会提前退出（非白点够多就直接返回 0），所以很便宜。
//-------------------------------------------------------------------------------------------------------------------
static uint8 cross_row_is_white (uint8 *image, uint16 row)
{
    uint8  *row_p = image + (uint32)row * MT9V03X_W;
    uint16  col;
    uint16  white = 0;
    uint16  total = 0;
    uint16  allow;

    allow = (uint16)((MT9V03X_W + CROSS_WHITE_STEP - 1) / CROSS_WHITE_STEP);
    allow = (uint16)(allow - (allow * CROSS_WHITE_PERCENT) / 100);   // 允许出现多少个非白点

    for(col = 0; col < MT9V03X_W; col += CROSS_WHITE_STEP)
    {
        if(row_p[col] >= IMG_WHITE_LEVEL)
        {
            white++;
        }
        else if((total - white) > allow)
        {
            return 0;                                               // 非白点已经够多 直接判定不是横杠行
        }

        total++;
    }

    return ((white * 100) >= (total * CROSS_WHITE_PERCENT)) ? 1 : 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     找最长的一段"不可信行"
// 参数说明     only_white          1:只认"整行几乎全白"(横杠)  0:整行全白 或 两侧都丢线
// 参数说明     *top, *bottom       找到的段 [top, bottom]（top 行号更小 = 更远）
// 返回参数     uint8               1:找到
//-------------------------------------------------------------------------------------------------------------------
static uint8 cross_find_bar (uint8 only_white, int16 *top, int16 *bottom)
{
    int16 y;
    int16 a = -1;
    int16 n = 0;
    int16 best_a = -1;
    int16 best_b = -1;
    int16 best_n = 0;
    uint8 need;

    for(y = 0; y <= MT9V03X_H; y++)
    {
        need = 0;

        if(y < MT9V03X_H)
        {
            if(0 != cross_row_white[y])
            {
                need = 1;
            }
            else if((0 == only_white) && (0x03 == (track_lost[y] & 0x03)))
            {
                need = 1;                                           // 两侧都丢线的行 也算"不可信"
            }
        }

        if(0 != need)
        {
            if(a < 0)
            {
                a = y;
            }
            n++;
            continue;
        }

        if(a >= 0)
        {
            if(n > best_n)
            {
                best_n = n;
                best_a = a;
                best_b = y - 1;
            }
            a = -1;
            n = 0;
        }
    }

    if(best_n < CROSS_WHITE_MIN_ROWS)
    {
        return 0;
    }

    *top    = best_a;
    *bottom = best_b;

    return 1;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     取某一行可靠边线的"中线"和"半宽(左右边界之差)"
//-------------------------------------------------------------------------------------------------------------------
static void cross_row_cw (int16 y, int16 *c, int16 *w)
{
    *c = ((int16)track_left[y] + (int16)track_right[y]) / 2;
    *w = (int16)track_right[y] - (int16)track_left[y];
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     估趋势：从 from 出发 往 dir 方向找最多 CROSS_LOOK_BACK 行里的较远可靠行
// 参数说明     from                起点行(这一行本身必须可靠)
// 参数说明     dir                 +1:往行号大的方向找  -1:往行号小的方向找
// 参数说明     *kc, *kw            Q8 格式的每行中线/宽度变化量
// 返回参数     uint8               1:估到了
// 备注信息     【方向很重要】丢线段贴着画面【最下面】时，可靠的一侧在【上面】(dir=-1)；
//              贴着【最上面】时相反(dir=+1)。写反了不会报错，只会静默退化成"保持原值"，
//              画面上就是一根竖直线加两个折角（实车踩过这个坑）。
//-------------------------------------------------------------------------------------------------------------------
static uint8 cross_est_trend (int16 from, int16 dir, int16 *kc, int16 *kw)
{
    int16 i;
    int16 c0;
    int16 w0;
    int16 c1;
    int16 w1;
    int16 sample_y = -1;

    cross_row_cw(from, &c0, &w0);

    for(i = (from + dir); (i >= 0) && (i < MT9V03X_H) && (i <= (from + CROSS_LOOK_BACK)) && (i >= (from - CROSS_LOOK_BACK)); i += dir)
    {
        if(0 != (track_lost[i] & 0x03))
        {
            continue;                                               // 不可靠的行不用来估趋势
        }

        cross_row_cw(i, &c1, &w1);
        sample_y = i;
    }

    if(sample_y >= 0)
    {
        *kc = (int16)(((int32)(c1 - c0) * 256) / (sample_y - from));
        *kw = (int16)(((int32)(w1 - w0) * 256) / (sample_y - from));

        if(*kc > (CROSS_BRIDGE_SLOPE_MAX * 256))  { *kc = CROSS_BRIDGE_SLOPE_MAX * 256; }
        if(*kc < -(CROSS_BRIDGE_SLOPE_MAX * 256)) { *kc = -(CROSS_BRIDGE_SLOPE_MAX * 256); }
        if(*kw > (CROSS_BRIDGE_SLOPE_MAX * 256))  { *kw = CROSS_BRIDGE_SLOPE_MAX * 256; }
        if(*kw < -(CROSS_BRIDGE_SLOPE_MAX * 256)) { *kw = -(CROSS_BRIDGE_SLOPE_MAX * 256); }

        return 1;
    }

    *kc = 0;
    *kw = 0;

    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把一段"不可信"的行 用两端可靠的中线/半宽接过去
// 参数说明     top, bottom         不可信段 [top, bottom]
// 返回参数     void
// 备注信息     两端都有 -> 线性插值；只有一端 -> 按趋势直线延伸
//-------------------------------------------------------------------------------------------------------------------
static void cross_bridge_range (int16 top, int16 bottom)
{
    int16 y;
    int16 near_y;
    int16 far_y;
    int16 c_near = 0;
    int16 c_far  = 0;
    int16 w_near = 0;
    int16 w_far  = 0;
    int16 c_ref  = 0;
    int16 w_ref  = 0;
    int16 kc     = 0;
    int16 kw     = 0;
    int16 c;
    int16 wd;
    int16 l;
    int16 r;
    uint8 two_end = 0;
    uint8 near_valid;
    uint8 far_valid;

    near_y = top - 1;
    far_y  = bottom + 1;
    if((0 != cross_flag) && (top <= (int16)cross_dbg_down_l) && (bottom >= (int16)cross_dbg_up_l))
    {
        // 横杠两端紧邻的行也可能单侧丢线，向外找最近的双边可靠行。
        for(; (near_y >= 0) && (near_y >= top - CROSS_LOOK_BACK); near_y--)
        {
            if((0 == (track_lost[near_y] & 0x03)) && (0 == cross_row_white[near_y])) { break; }
        }
        for(; (far_y < MT9V03X_H) && (far_y <= bottom + CROSS_LOOK_BACK); far_y++)
        {
            if((0 == (track_lost[far_y] & 0x03)) && (0 == cross_row_white[far_y])) { break; }
        }
        near_valid = ((near_y >= 0) && (near_y >= top - CROSS_LOOK_BACK)) ? 1 : 0;
        far_valid  = ((far_y < MT9V03X_H) && (far_y <= bottom + CROSS_LOOK_BACK)) ? 1 : 0;
    }
    else
    {
        near_valid = ((near_y >= 0) && (0 == (track_lost[near_y] & 0x03))) ? 1 : 0;
        far_valid  = ((far_y < MT9V03X_H) && (0 == (track_lost[far_y] & 0x03))) ? 1 : 0;
    }

    if((0 != near_valid) && (0 != far_valid))
    {
        two_end = 1;
        cross_row_cw(near_y, &c_near, &w_near);
        cross_row_cw(far_y,  &c_far,  &w_far);
    }
    else if(0 != near_valid)                                         // 只有上面那一端可靠 -> 往上估趋势
    {
        cross_row_cw(near_y, &c_ref, &w_ref);
        cross_est_trend(near_y, -1, &kc, &kw);
    }
    else if(0 != far_valid)                                          // 只有下面那一端可靠 -> 往下估趋势
    {
        cross_row_cw(far_y, &c_ref, &w_ref);
        cross_est_trend(far_y, 1, &kc, &kw);
    }
    else
    {
        return;                                                     // 整幅图都不可信 -> 不动
    }

    cross_bridge_used = 1;
    cross_bridge_two_end = (0 != two_end) &&
                           (track_left[near_y] > CROSS_BRIDGE_ANCHOR_MARGIN) &&
                           (track_right[near_y] < MT9V03X_W - 1 - CROSS_BRIDGE_ANCHOR_MARGIN) &&
                           (track_left[far_y] > CROSS_BRIDGE_ANCHOR_MARGIN) &&
                           (track_right[far_y] < MT9V03X_W - 1 - CROSS_BRIDGE_ANCHOR_MARGIN);
    cross_bridge_top = top;
    cross_bridge_bottom = bottom;

    for(y = top; y <= bottom; y++)
    {
        if(0 != two_end)
        {
            c  = c_near + (int16)(((int32)(c_far - c_near) * (y - near_y)) / (far_y - near_y));
            wd = w_near + (int16)(((int32)(w_far - w_near) * (y - near_y)) / (far_y - near_y));
        }
        else
        {
            int16 ref_y = (0 != near_valid) ? near_y : far_y;

            c  = c_ref + (int16)(((int32)kc * (y - ref_y)) / 256);
            wd = w_ref + (int16)(((int32)kw * (y - ref_y)) / 256);

            // 【封顶】延伸出来的中线/半宽 相对锚点那一行最多偏 CROSS_BRIDGE_DRIFT_MAX 像素
            //   实车：横杠占 58 行时，按每行 1 像素外推 58 行，宽度从 90 涨到 173（几乎整幅画面），
            //         画面上就是三条贴边的竖线，看起来跟没补一样。
            if(c > (c_ref + CROSS_BRIDGE_DRIFT_MAX))  { c = c_ref + CROSS_BRIDGE_DRIFT_MAX; }
            if(c < (c_ref - CROSS_BRIDGE_DRIFT_MAX))  { c = c_ref - CROSS_BRIDGE_DRIFT_MAX; }
            if(wd > (w_ref + CROSS_BRIDGE_DRIFT_MAX)) { wd = w_ref + CROSS_BRIDGE_DRIFT_MAX; }
            if(wd < (w_ref - CROSS_BRIDGE_DRIFT_MAX)) { wd = w_ref - CROSS_BRIDGE_DRIFT_MAX; }
        }

        if(c < 10) { c = 10; }
        if(c > (MT9V03X_W - 11)) { c = (MT9V03X_W - 11); }
        if(wd < 10) { wd = 10; }
        if(wd > (MT9V03X_W - 2)) { wd = (MT9V03X_W - 2); }

        l = c - (wd / 2);
        r = c + (wd / 2);
        if(l < 0) { l = 0; }
        if(r > (MT9V03X_W - 1)) { r = (MT9V03X_W - 1); }

        track_left[y]   = (uint8)l;
        track_right[y]  = (uint8)r;
        track_center[y] = (uint8)((l + r) / 2);
        track_lost[y]   = (uint8)(track_lost[y] & 0xFC);            // 清掉两侧丢线标志 防止被后面的插值/贴边覆盖
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     十字路口识别与补线
// 参数说明     *binary_image      图像
// 返回参数     void
// 使用示例     track_edge_process(image_copy[0]);
//              cross_road_process(image_copy[0]);
//-------------------------------------------------------------------------------------------------------------------
void cross_road_process (uint8 *binary_image)
{
    int16 y;
    int16 top      = 0;
    int16 bottom   = 0;
    int16 b_top    = 0;
    int16 b_bottom = 0;

    cross_bridge_used = 0;
    cross_bridge_two_end = 0;
    cross_bridge_top  = -1;
    cross_bridge_bottom = -1;
    cross_dbg_up_l    = 0;
    cross_dbg_down_l  = 0;

    // 每帧先把"哪几行是横杠行"算一遍
    for(y = 0; y < MT9V03X_H; y++)
    {
        cross_row_white[y] = cross_row_is_white(binary_image, (uint16)y);
    }

    // ---- 状态：只看横杠在哪（只对外汇报 不驱动补线）----
    if(0 != cross_find_bar(1, &top, &bottom))
    {
        cross_dbg_up_l    = (uint8)top;
        cross_dbg_down_l  = (uint8)bottom;
        cross_flag        = 1;
        cross_state       = (bottom >= (MT9V03X_H - 2)) ? CROSS_STATE_INSIDE : CROSS_STATE_IN;
        cross_state_count = 0;
    }
    else
    {
        cross_flag = 0;

        if((CROSS_STATE_IN == cross_state) || (CROSS_STATE_INSIDE == cross_state))
        {
            cross_state       = CROSS_STATE_OUT;                    // 横杠刚消失 -> 告诉控制侧"刚出十字"
            cross_state_count = 0;
        }
        else if(CROSS_STATE_OUT == cross_state)
        {
            cross_state_count++;
            if(cross_state_count > CROSS_OUT_HOLD)
            {
                cross_state       = CROSS_STATE_NONE;
                cross_state_count = 0;
            }
        }
        else
        {
            cross_state = CROSS_STATE_NONE;
        }
    }

    // ---- 补线：把"不可信的那一段"接过去（横杠 或 两侧都丢线）----
    //   注意：不依赖 cross_state。图像里有这么一段就补，状态怎么抖都不影响线的形状。
    if(0 != cross_find_bar(0, &b_top, &b_bottom))
    {
        cross_bridge_range(b_top, b_bottom);
    }

    track_center_update();
}
