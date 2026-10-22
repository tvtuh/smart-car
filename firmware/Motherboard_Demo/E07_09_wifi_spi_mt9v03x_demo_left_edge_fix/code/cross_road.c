/*********************************************************************************************************************
* 文件名称          cross_road.c
* 功能简介          双侧重叠开口的分侧图像补线；保留90%横杠和原桥接作为后备
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "cross_road.h"
#include "track_edge.h"
#include "image_process.h"

//------------------------------------------------------------------------------------------------------------------
// 对外变量
//------------------------------------------------------------------------------------------------------------------
uint8 cross_image_status = CROSS_IMAGE_STATUS_IDLE;
int16 cross_image_fail_row = -1;
uint8 cross_flag        = 0;
uint8 cross_state       = 0;
uint8 cross_bridge_used = 0;
uint8 cross_bridge_two_end = 0;
int16 cross_bridge_top = -1;
int16 cross_bridge_bottom = -1;
uint8 cross_dbg_up_l    = 0;
uint8 cross_dbg_down_l  = 0;

// Cached image runs and guides are independent of the filtered track arrays.
static uint8 cross_image_row_state[VISION_H];
static uint8 cross_image_left[VISION_H];
static uint8 cross_image_right[VISION_H];
static uint8 cross_guide_left[VISION_H];
static uint8 cross_guide_right[VISION_H];
static uint8 cross_guide_mask[VISION_H];
static uint8 cross_guide_active = 0;
static uint8 cross_last_left[VISION_H];
static uint8 cross_last_right[VISION_H];
static uint8 cross_last_valid = 0;
static uint8 cross_hold_age = 0;
static uint8 cross_hold_frame = 0;

static uint16 cross_state_count = 0;
static uint8  cross_row_white[VISION_H];                           // 每行"是不是横杠行"的缓存(每帧算一次)


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断某一行是不是"横杠行"（几乎整行都是白）
// 参数说明     *image              图像
// 参数说明     row                 行号
// 返回参数     uint8               1:是横杠行
// 备注信息     抽样统计这一行的白点比例。非白行会提前退出（非白点够多就直接返回 0），所以很便宜。
//-------------------------------------------------------------------------------------------------------------------
static uint8 cross_row_is_white (uint8 *image, uint16 row)
{
    uint8  *row_p = image + (uint32)row * VISION_W;
    uint16  col;
    uint16  white = 0;
    uint16  total = 0;
    uint16  allow;

    allow = (uint16)((VISION_W + CROSS_WHITE_STEP - 1) / CROSS_WHITE_STEP);
    allow = (uint16)(allow - (allow * CROSS_WHITE_PERCENT) / 100);   // 允许出现多少个非白点

    for(col = 0; col < VISION_W; col += CROSS_WHITE_STEP)
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

    for(y = 0; y <= VISION_VALID_H; y++)
    {
        need = 0;

        if(y < VISION_VALID_H)
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

    for(i = (from + dir); (i >= 0) && (i < VISION_VALID_H) && (i <= (from + CROSS_LOOK_BACK)) && (i >= (from - CROSS_LOOK_BACK)); i += dir)
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
        for(; (far_y < VISION_VALID_H) && (far_y <= bottom + CROSS_LOOK_BACK); far_y++)
        {
            if((0 == (track_lost[far_y] & 0x03)) && (0 == cross_row_white[far_y])) { break; }
        }
        near_valid = ((near_y >= 0) && (near_y >= top - CROSS_LOOK_BACK)) ? 1 : 0;
        far_valid  = ((far_y < VISION_VALID_H) && (far_y <= bottom + CROSS_LOOK_BACK)) ? 1 : 0;
    }
    else
    {
        near_valid = ((near_y >= 0) && (0 == (track_lost[near_y] & 0x03))) ? 1 : 0;
        far_valid  = ((far_y < VISION_VALID_H) && (0 == (track_lost[far_y] & 0x03))) ? 1 : 0;
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
                           (track_right[near_y] < VISION_W - 1 - CROSS_BRIDGE_ANCHOR_MARGIN) &&
                           (track_left[far_y] > CROSS_BRIDGE_ANCHOR_MARGIN) &&
                           (track_right[far_y] < VISION_W - 1 - CROSS_BRIDGE_ANCHOR_MARGIN);
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

        if(c < VISION_PX(10)) { c = VISION_PX(10); }
        if(c > (VISION_W - VISION_PX(11))) { c = (VISION_W - VISION_PX(11)); }
        if(wd < VISION_PX(10)) { wd = VISION_PX(10); }
        if(wd > (VISION_W - 2)) { wd = (VISION_W - 2); }

        l = c - (wd / 2);
        r = c + (wd / 2);
        if(l < 0) { l = 0; }
        if(r > (VISION_W - 1)) { r = (VISION_W - 1); }

        track_left[y]   = (uint8)l;
        track_right[y]  = (uint8)r;
        track_center[y] = (uint8)((l + r) / 2);
        track_lost[y]   = (uint8)(track_lost[y] & 0xFC);            // 清掉两侧丢线标志 防止被后面的插值/贴边覆盖
    }
}


// Row-state bits: 1 = scanned, 2 = valid run, 4/8 = left/right opening.
static uint8 cross_image_run (uint8 *image, int16 row)
{
    uint8 *row_p;
    int16 reference;
    int16 radius;
    int16 left;
    int16 right;

    if(0 != (cross_image_row_state[row] & 0x01))
    {
        return (uint8)(cross_image_row_state[row] & 0x02);
    }
    cross_image_row_state[row] = 0x01;
    row_p = image + (uint32)row * VISION_W;
    reference = (track_center[row] < VISION_W) ? track_center[row] : TRACK_SEED_COL;
    if(row_p[reference] < IMG_WHITE_LEVEL)
    {
        for(radius = 1; radius <= CROSS_IMAGE_WHITE_SEARCH; radius++)
        {
            if((reference - radius >= 0) &&
               (row_p[reference - radius] >= IMG_WHITE_LEVEL))
            {
                reference -= radius;
                break;
            }
            if((reference + radius < VISION_W) &&
               (row_p[reference + radius] >= IMG_WHITE_LEVEL))
            {
                reference += radius;
                break;
            }
        }
        if(radius > CROSS_IMAGE_WHITE_SEARCH) { return 0; }
    }
    left = reference;
    right = reference;
    while((left > 0) && (row_p[left - 1] >= IMG_WHITE_LEVEL)) { left--; }
    while((right < VISION_W - 1) && (row_p[right + 1] >= IMG_WHITE_LEVEL)) { right++; }
    if(right - left < CROSS_IMAGE_WIDTH_MIN) { return 0; }
    cross_image_left[row] = (uint8)left;
    cross_image_right[row] = (uint8)right;
    cross_image_row_state[row] |= 0x02;
    if(left <= CROSS_IMAGE_OPEN_MARGIN) { cross_image_row_state[row] |= 0x04; }
    if(right >= VISION_W - 1 - CROSS_IMAGE_OPEN_MARGIN) { cross_image_row_state[row] |= 0x08; }
    return 1;
}


// An anchor and its confirming rows must lie outside the opening.
static int16 cross_image_anchor (uint8 *image, int16 from, int16 direction, uint8 side)
{
    int16 distance, sample, y, sample_y, edge, previous;
    int16 first, last, count;
    int32 residual;
    uint8 valid;

    for(distance = 1; distance <= CROSS_IMAGE_ANCHOR_SEARCH; distance++)
    {
        y = from + direction * distance;
        if((y < 0) || (y >= VISION_VALID_H)) { break; }
        valid = 1;
        previous = -1;
        first = last = count = 0;
        for(sample = 0; sample < CROSS_IMAGE_STABLE_ROWS; sample++)
        {
            sample_y = y + direction * sample;
            if((sample_y < 0) || (sample_y > CROSS_IMAGE_LAST_ROW) ||
               (distance + sample > CROSS_IMAGE_ANCHOR_SEARCH)) { break; }
            if(0 == cross_image_run(image, sample_y)) { valid = 0; break; }
            edge = (0x04 == side) ? cross_image_left[sample_y] : cross_image_right[sample_y];
            if(((0x04 == side) && (edge <= CROSS_IMAGE_OPEN_MARGIN)) ||
               ((0x08 == side) && (edge >= VISION_W - 1 - CROSS_IMAGE_OPEN_MARGIN)))
            {
                if(count < CROSS_IMAGE_ANCHOR_ROWS) { valid = 0; }
                break;  /* A visible near edge can leave the camera farther down. */
            }
            if((previous >= 0) && (((edge - previous) > CROSS_IMAGE_ANCHOR_STEP) ||
                                  ((previous - edge) > CROSS_IMAGE_ANCHOR_STEP)))
            {
                valid = 0;
                break;
            }
            if(0 == count) { first = edge; }
            last = previous = edge;
            count++;
        }
        if(!valid || (count < CROSS_IMAGE_ANCHOR_ROWS)) { continue; }
        /* A rounded cross corner can move only a little per row yet bend away
         * from the road. Require the confirming span to follow one line. */
        for(sample = 1; sample < count - 1; sample++)
        {
            sample_y = y + direction * sample;
            edge = (0x04 == side) ? cross_image_left[sample_y] : cross_image_right[sample_y];
            residual = (int32)(edge - first) * (count - 1) - (int32)(last - first) * sample;
            if((residual > (int32)CROSS_IMAGE_STABLE_TOL * (count - 1)) ||
               (residual < -(int32)CROSS_IMAGE_STABLE_TOL * (count - 1)))
            {
                valid = 0;
                break;
            }
        }
        if(valid) { return y; }
    }
    return -1;
}


// Fit the still-visible far edge in Q8, preserving sub-pixel perspective.
static uint8 cross_image_trend (uint8 *image, int16 anchor, uint8 side, int16 *slope)
{
    int16 y;
    int16 edge;
    int16 previous = -1;
    int16 n = 0;
    int32 sy = 0, se = 0, syy = 0, sye = 0;
    int32 denominator;
    int32 value;
    for(y = anchor; (y >= 0) && (y > anchor - CROSS_IMAGE_TREND_ROWS); y--)
    {
        if(0 == cross_image_run(image, y)) { break; }
        edge = (0x04 == side) ? cross_image_left[y] : cross_image_right[y];
        if(((0x04 == side) && (edge <= CROSS_IMAGE_OPEN_MARGIN)) ||
           ((0x08 == side) && (edge >= VISION_W - 1 - CROSS_IMAGE_OPEN_MARGIN)) ||
           ((previous >= 0) && (((edge - previous) > CROSS_IMAGE_ANCHOR_STEP) ||
                                ((previous - edge) > CROSS_IMAGE_ANCHOR_STEP)))) { break; }
        n++;
        sy += y; se += edge;
        syy += (int32)y * y; sye += (int32)y * edge;
        previous = edge;
    }
    if(n < CROSS_IMAGE_ANCHOR_ROWS) { return 0; }
    denominator = (int32)n * syy - sy * sy;
    if(denominator <= 0) { return 0; }
    value = (((int32)n * sye - sy * se) * 256) / denominator;
    if(value > CROSS_BRIDGE_SLOPE_MAX * 256) { value = CROSS_BRIDGE_SLOPE_MAX * 256; }
    if(value < -CROSS_BRIDGE_SLOPE_MAX * 256) { value = -CROSS_BRIDGE_SLOPE_MAX * 256; }
    *slope = (int16)value;
    return 1;
}

static uint8 cross_image_find_side (uint8 *image, uint8 side, int16 *top,
                                    int16 *bottom, int16 *upper, int16 *lower)
{
    int16 y;
    int16 start = -1;
    int16 length;
    int16 best_length = 0;
    int16 upper_y;
    int16 lower_y;

    for(y = 0; y <= CROSS_IMAGE_LAST_ROW + 1; y++)
    {
        if((y <= CROSS_IMAGE_LAST_ROW) && (0 != (cross_image_row_state[y] & side)))
        {
            if(start < 0) { start = y; }
            continue;
        }
        if(start < 0) { continue; }
        length = y - start;
        if((length >= CROSS_WHITE_MIN_ROWS) && (length > best_length))
        {
            upper_y = cross_image_anchor(image, start, -1, side);
            lower_y = cross_image_anchor(image, y - 1, 1, side);
            // A near opening has no confirming rows beyond the image floor.
            if((upper_y >= 0) && ((lower_y >= 0) ||
               (y - 1 >= CROSS_IMAGE_LAST_ROW - CROSS_IMAGE_ANCHOR_ROWS + 1)))
            {
                *top = start;
                *bottom = y - 1;
                *upper = upper_y;
                *lower = lower_y;
                best_length = length;
            }
        }
        start = -1;
    }
    return (best_length > 0) ? 1 : 0;
}


static uint8 cross_image_bridge (uint8 *image)
{
    int16 y;
    int16 left_top;
    int16 left_bottom;
    int16 right_top;
    int16 right_bottom;
    int16 left_upper;
    int16 left_lower;
    int16 right_upper;
    int16 right_lower;
    int16 overlap_top;
    int16 overlap_bottom;
    int16 left_end;
    int16 right_end;
    int16 left_slope = 0;
    int16 right_slope = 0;
    int16 bridge_top;
    int16 bridge_bottom;
    int16 left;
    int16 right;
    uint8 mask;
    uint8 *row_p;

    for(y = 0; y < VISION_VALID_H; y++)
    {
        row_p = image + (uint32)y * VISION_W;
        // Most normal-road rows need only these two pixel comparisons.
        if((row_p[CROSS_IMAGE_OPEN_MARGIN] >= IMG_WHITE_LEVEL) ||
           (row_p[VISION_W - 1 - CROSS_IMAGE_OPEN_MARGIN] >= IMG_WHITE_LEVEL))
        {
            cross_image_run(image, y);
        }
    }
    if(0 == cross_image_find_side(image, 0x04, &left_top, &left_bottom, &left_upper, &left_lower))
    {
        cross_image_status = CROSS_IMAGE_STATUS_LEFT;
        return 0;
    }
    if(0 == cross_image_find_side(image, 0x08, &right_top, &right_bottom, &right_upper, &right_lower))
    {
        cross_image_status = CROSS_IMAGE_STATUS_RIGHT;
        return 0;
    }
    // A fresh far-only cross needs abrupt openings on both sides.
    // Gradual perspective widening at the image floor is not a new cross.
    if(((left_lower < 0) || (right_lower < 0)) && (0 == cross_last_valid) &&
       (((int16)cross_image_left[left_upper] - cross_image_left[left_top] < CROSS_IMAGE_OPEN_JUMP) ||
        ((int16)cross_image_right[right_top] - cross_image_right[right_upper] < CROSS_IMAGE_OPEN_JUMP)))
    {
        cross_image_status = CROSS_IMAGE_STATUS_PAIR;
        cross_image_fail_row = (left_top > right_top) ? left_top : right_top;
        return 0;
    }
    // Pair the gaps between stable anchors. A slanted bar reaches the
    // two image borders at different heights, while its inner gaps overlap.
    left_end = (left_lower >= 0) ? left_lower : CROSS_IMAGE_LAST_ROW + 1;
    right_end = (right_lower >= 0) ? right_lower : CROSS_IMAGE_LAST_ROW + 1;
    if((left_lower < 0) && (0 == cross_image_trend(image, left_upper, 0x04, &left_slope)))
    {
        cross_image_status = CROSS_IMAGE_STATUS_LEFT;
        return 0;
    }
    if((right_lower < 0) && (0 == cross_image_trend(image, right_upper, 0x08, &right_slope)))
    {
        cross_image_status = CROSS_IMAGE_STATUS_RIGHT;
        return 0;
    }
    overlap_top = (left_upper > right_upper) ? left_upper : right_upper;
    overlap_bottom = (left_end < right_end) ? left_end : right_end;
    if(overlap_bottom - overlap_top + 1 < CROSS_WHITE_MIN_ROWS)
    {
        cross_image_status = CROSS_IMAGE_STATUS_PAIR;
        cross_image_fail_row = overlap_top;
        return 0;
    }
    bridge_top = ((left_upper < right_upper) ? left_upper : right_upper) + 1;
    bridge_bottom = ((left_end > right_end) ? left_end : right_end) - 1;

    // Validate the complete guide before modifying any track row.
    for(y = bridge_top; y <= bridge_bottom; y++)
    {
        if(0 == cross_image_run(image, y))
        {
            cross_image_status = CROSS_IMAGE_STATUS_RUN;
            cross_image_fail_row = y;
            return 0;
        }
        left = cross_image_left[y];
        right = cross_image_right[y];
        mask = 0;
        if((y > left_upper) && (y < left_end))
        {
            if(left_lower >= 0)
            {
                left = cross_image_left[left_upper] +
                       (int16)(((int32)((int16)cross_image_left[left_lower] - cross_image_left[left_upper]) *
                                (y - left_upper)) / (left_lower - left_upper));
            }
            else
            {
                left = cross_image_left[left_upper] + (int16)(((int32)left_slope * (y - left_upper)) / 256);
            }
            if(left < 0) { left = 0; }
            if(left >= VISION_W) { left = VISION_W - 1; }
            mask |= 0x01;
        }
        if((y > right_upper) && (y < right_end))
        {
            if(right_lower >= 0)
            {
                right = cross_image_right[right_upper] +
                        (int16)(((int32)((int16)cross_image_right[right_lower] - cross_image_right[right_upper]) *
                                 (y - right_upper)) / (right_lower - right_upper));
            }
            else
            {
                right = cross_image_right[right_upper] + (int16)(((int32)right_slope * (y - right_upper)) / 256);
            }
            if(right < 0) { right = 0; }
            if(right >= VISION_W) { right = VISION_W - 1; }
            mask |= 0x02;
        }
        // Small threshold/rounded-corner errors must not cancel the whole bridge.
        // Snap only the virtual side inward; the output still stays fully white.
        if((0 != (mask & 0x01)) && (left < cross_image_left[y]) &&
           ((int16)cross_image_left[y] - left <= CROSS_IMAGE_WHITE_TOL))
        {
            left = cross_image_left[y];
        }
        if((0 != (mask & 0x02)) && (right > cross_image_right[y]) &&
           (right - (int16)cross_image_right[y] <= CROSS_IMAGE_WHITE_TOL))
        {
            right = cross_image_right[y];
        }
        if((left < cross_image_left[y]) || (right > cross_image_right[y]))
        {
            cross_image_status = CROSS_IMAGE_STATUS_GEOMETRY;
            cross_image_fail_row = y;
            return 0;
        }
        if(right - left < CROSS_IMAGE_WIDTH_MIN)
        {
            cross_image_status = CROSS_IMAGE_STATUS_WIDTH;
            cross_image_fail_row = y;
            return 0;
        }
        cross_guide_left[y] = (uint8)left;
        cross_guide_right[y] = (uint8)right;
        cross_guide_mask[y] = mask;
    }

    for(y = bridge_top; y <= bridge_bottom; y++)
    {
        mask = cross_guide_mask[y];
        if(0 != (mask & 0x01)) { track_left[y] = cross_guide_left[y]; }
        else { track_left[y] = cross_image_left[y]; }
        if(0 != (mask & 0x02)) { track_right[y] = cross_guide_right[y]; }
        else { track_right[y] = cross_image_right[y]; }
        track_lost[y] &= (uint8)(~mask);
    }
    cross_flag = 1;
    cross_state = ((left_lower < 0) || (right_lower < 0)) ? CROSS_STATE_INSIDE : CROSS_STATE_IN;
    cross_state_count = 0;
    cross_bridge_used = 1;
    cross_bridge_two_end = 1;
    cross_bridge_top = bridge_top;
    cross_bridge_bottom = bridge_bottom;
    cross_dbg_up_l = (uint8)((left_top < right_top) ? left_top : right_top);
    cross_dbg_down_l = (uint8)((left_bottom > right_bottom) ? left_bottom : right_bottom);
    cross_guide_active = 1;
    cross_image_status = ((left_lower < 0) || (right_lower < 0)) ? CROSS_IMAGE_STATUS_INSIDE : CROSS_IMAGE_STATUS_OK;
    cross_image_fail_row = -1;
    return 1;
}


void cross_road_reset_guidance (void)
{
    cross_guide_active = 0;
    cross_last_valid = 0;
    cross_hold_age = 0;
    cross_image_status = CROSS_IMAGE_STATUS_IDLE;
    cross_image_fail_row = -1;
}

// Save only a fresh route that also passed the final white guard.
void cross_road_save_final (void)
{
    int16 y;
    int16 count = 0;
    if((0 == cross_guide_active) || (0 != cross_hold_frame) ||
       ((CROSS_IMAGE_STATUS_OK != cross_image_status) &&
        (CROSS_IMAGE_STATUS_INSIDE != cross_image_status))) { return; }
    for(y = 0; y <= CROSS_IMAGE_LAST_ROW; y++)
    {
        if((track_right[y] < VISION_W) &&
           ((int16)track_right[y] - track_left[y] >= CROSS_IMAGE_WIDTH_MIN))
        {
            cross_last_left[y] = track_left[y];
            cross_last_right[y] = track_right[y];
            count++;
        }
        else { cross_last_left[y] = 255; cross_last_right[y] = 255; }
    }
    cross_last_valid = (count >= VISION_H / 3) ? 1 : 0;
    cross_hold_age = 0;
}

// A fully open junction may temporarily hide both visible anchor sets.
// Reuse a recently confirmed route only briefly and revalidate its white run.
static uint8 cross_image_hold (uint8 *image)
{
    int16 y;
    int16 left;
    int16 right;
    int16 count = 0;
    int16 near_white = 0;
    int16 top = -1;
    int16 bottom = -1;
    if((0 == cross_last_valid) || (cross_hold_age > CROSS_IMAGE_HOLD_FRAMES)) { return 0; }
    for(y = VISION_H * 2 / 3; y <= CROSS_IMAGE_LAST_ROW; y++)
    {
        if(0 != cross_row_white[y]) { near_white++; }
    }
    if(near_white < VISION_H / 4) { return 0; }
    for(y = 0; y <= CROSS_IMAGE_LAST_ROW; y++)
    {
        cross_guide_mask[y] = 0;
        if(cross_last_right[y] >= VISION_W) { continue; }
        if(0 == cross_image_run(image, y)) { return 0; }
        left = cross_last_left[y]; right = cross_last_right[y];
        if((left < cross_image_left[y]) &&
           ((int16)cross_image_left[y] - left <= CROSS_IMAGE_WHITE_TOL)) { left = cross_image_left[y]; }
        if((right > cross_image_right[y]) &&
           (right - (int16)cross_image_right[y] <= CROSS_IMAGE_WHITE_TOL)) { right = cross_image_right[y]; }
        if((left < cross_image_left[y]) || (right > cross_image_right[y]) ||
           (right - left < CROSS_IMAGE_WIDTH_MIN)) { return 0; }
        cross_guide_left[y] = (uint8)left; cross_guide_right[y] = (uint8)right;
        cross_guide_mask[y] = 0x03;
        if(top < 0) { top = y; }
        bottom = y; count++;
    }
    if(count < VISION_H / 3) { return 0; }
    for(y = top; y <= bottom; y++)
    {
        if(0 == cross_guide_mask[y]) { continue; }
        track_left[y] = cross_guide_left[y]; track_right[y] = cross_guide_right[y];
        track_lost[y] &= 0xfc;
    }
    cross_flag = 1; cross_state = CROSS_STATE_INSIDE; cross_state_count = 0;
    cross_bridge_used = 1; cross_bridge_two_end = 1;
    cross_bridge_top = top; cross_bridge_bottom = bottom;
    cross_dbg_up_l = (uint8)top; cross_dbg_down_l = (uint8)bottom;
    cross_guide_active = 1; cross_hold_frame = 1;
    cross_image_status = CROSS_IMAGE_STATUS_INSIDE; cross_image_fail_row = -1;
    return 1;
}

uint8 cross_road_get_guide (uint16 row, int16 *left, int16 *right)
{
    if((0 == cross_guide_active) || (0 == cross_flag) ||
       (0 == cross_bridge_used) || (row >= VISION_VALID_H)) { return 0; }
    if(0 == cross_guide_mask[row]) { return 0; }
    *left = cross_guide_left[row];
    *right = cross_guide_right[row];
    return cross_guide_mask[row];
}


//-------------------------------------------------------------------------------------------------------------------
// 功能简介     十字路口识别与补线
// 参数说明     *binary_image      灰度图或二值图，按 IMG_WHITE_LEVEL 判白
// 工作流程     清除本帧引导，更新原横杠状态，再优先尝试双侧重叠开口和分侧稳定图像锚点。
//              主路径通过整段白区验证后保存逐侧引导；白区 guard 在平滑后恢复该引导。
//              主路径失败时使用原全白/双侧丢线段桥接，支持双端插值和单端趋势延伸。
// 使用示例     track_edge_process(image_copy[0]);
//              cross_road_process(image_copy[0]);
//-------------------------------------------------------------------------------------------------------------------
static void cross_frame_begin (void)
{
    int16 y;
    cross_guide_active = 0;
    cross_hold_frame = 0;
    cross_image_status = CROSS_IMAGE_STATUS_IDLE;
    cross_image_fail_row = -1;
    if(0 != cross_last_valid)
    {
        cross_hold_age++;
        if(cross_hold_age > CROSS_IMAGE_HOLD_FRAMES) { cross_last_valid = 0; }
    }
    for(y = 0; y < VISION_VALID_H; y++)
    {
        cross_image_row_state[y] = 0;
        cross_guide_mask[y] = 0;
    }
    cross_bridge_used = 0;
    cross_bridge_two_end = 0;
    cross_bridge_top  = -1;
    cross_bridge_bottom = -1;
    cross_dbg_up_l    = 0;
    cross_dbg_down_l  = 0;
}

/* A pending ring may yield only to a fresh, image-confirmed two-sided cross.
 * main.c limits this probe to candidates and this frame's FIND -> IN transition.
 * Established ring states retain priority; no legacy fallback or history hold. */
uint8 cross_road_try_image (uint8 *image)
{
    cross_frame_begin();
    if(0 == cross_image_bridge(image)) { return 0; }
    track_center_update();
    return 1;
}

void cross_road_process (uint8 *binary_image)
{
    int16 y;
    int16 top      = 0;
    int16 bottom   = 0;
    int16 b_top    = 0;
    int16 b_bottom = 0;

    cross_frame_begin();

    // 每帧先把"哪几行是横杠行"算一遍
    for(y = 0; y < VISION_VALID_H; y++)
    {
        cross_row_white[y] = cross_row_is_white(binary_image, (uint16)y);
    }

    // ---- 状态：只看横杠在哪（只对外汇报 不驱动补线）----
    if(0 != cross_find_bar(1, &top, &bottom))
    {
        cross_dbg_up_l    = (uint8)top;
        cross_dbg_down_l  = (uint8)bottom;
        cross_flag        = 1;
        cross_state       = (bottom >= (VISION_H - 2)) ? CROSS_STATE_INSIDE : CROSS_STATE_IN;
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
    if(0 != cross_image_bridge(binary_image))
    {
        track_center_update();
        return;
    }

    if(0 != cross_image_hold(binary_image))
    {
        track_center_update();
        return;
    }
    if(0 != cross_find_bar(0, &b_top, &b_bottom))
    {
        cross_bridge_range(b_top, b_bottom);
    }

    track_center_update();
}
