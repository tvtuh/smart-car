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
* 说明              对侧直线、单侧开口和白区 P 点突变共同确认入口。
*                  入口用 P 到图像下角的虚拟边线与对侧图像边框求中线；
*                  出环后锁住旧入口，待拐点消失并出现新拐点才重新引导。
*                  左右环镜像处理。
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
static uint8  ring_guide_active = 0;
static uint8  ring_guide_blocked = 0;
static uint8  ring_clear_frames = 0;
static uint8  ring_confirm_type = RING_TYPE_NONE;
static uint8  ring_confirm_row = 0;
static uint8  ring_confirm_col = 0;
static uint8  ring_p_missing = 0;
static uint8  ring_right_physical[VISION_H];
static uint8  ring_upper_curve[VISION_H];
static uint8  ring_upper_curve_active = 0;
static uint8  ring_p_right_valid = 0;
static uint8  ring_p_left_valid = 0;
static uint8  ring_p_right_row = 0;
static uint8  ring_p_right_col = 0;
static uint8  ring_p_left_row = 0;
static uint8  ring_p_left_col = 0;
static uint8  ring_left_p_peak_col = 0;
static uint8  ring_left_p_peak_age = 0;
static uint8  ring_first_open_seen = 0;
static uint8  ring_first_open_count = 0;
static uint8  ring_first_open_col = 0;
static uint8  ring_first_min_col = VISION_W - 1;
static uint8  ring_first_missing = 0;
static uint8  ring_first_ready = 0;
static uint8  ring_first_age = 0;
static uint8  ring_wait_second = 0;
static uint8  ring_second_ready = 0;
static uint8  ring_cross_like = 0;
uint8  ring_exit_active = 0;
uint8  ring_exit_row = 0;
uint8  ring_exit_col = 0;
static uint8  ring_out_clear_frames = 0;
static uint8  ring_exit_missing = 0;
static uint8  ring_exit_seen = 0;
/* 255 means no checked continuation for this raw-image row. */
static uint8  ring_exit_bridge[VISION_H];
static uint8 *ring_guide_image = 0; /* Stable processing snapshot, before overlays. */
static uint8  ring_exit_max_row = 0;
static uint16 ring_diag_frame = 0;
static uint8  ring_diag_previous_state = RING_STATE_NONE;
static uint8  ring_diag_reason = RING_DIAG_NONE;
static uint8  ring_diag_p_source = 0;
static uint8  ring_diag_guide_applied = 0;
static uint8  ring_diag_candidate_side = RING_TYPE_NONE;
static uint8  ring_diag_candidate_row = 0;
static uint8  ring_diag_candidate_col = 0;
static uint8  ring_diag_candidate_run = 0;
static uint8  ring_diag_exit_probe = 0;
static uint8  ring_exit_tail_active = 0;
static uint8  ring_exit_tail_row = 0;
static uint8  ring_exit_tail_col = 0;
static int16  ring_exit_tail_delta = 0;
static uint8  ring_exit_tail_unpaired = 0;
static uint8  ring_exit_tail_upper_start = VISION_H;
static uint8  ring_exit_tail_upper_end = 0;
static uint8  ring_post_exit_type = RING_TYPE_NONE;
static uint8  ring_post_exit_max_row = 0;
static uint8  ring_post_exit_age = 0;

/* Bounded opposite-ring handoff. History is sampled before any line rewrite. */
typedef struct
{
    uint8 frames;
    uint8 source;
    uint8 pass_row;
    uint8 peak_col;
    uint8 peak_age;
    uint8 confirm;
    uint8 row;
    uint8 col;
    uint8 seen;
} ring_chain_t;
static ring_chain_t ring_chain;
static uint8 ring_block_after_cross = 0;
static uint8 ring_cross_confirm = 0;
static uint8 ring_cross_confirm_type = RING_TYPE_NONE;
static uint8 ring_cross_right_entry = 0;
static uint8 ring_cross_confirm_row = 0;
static uint8 ring_cross_confirm_col = 0;

/* Cache lifetime is one ring_road_process call, before cross/guard rewrites
 * track arrays. Eight slots cover the P search offsets {-4,-2,+2}.
 * Status: 0=not computed, 1=miss, 2=hit; 255 means span outputs untouched. */
#if RING_FRAME_CACHE_ENABLE
typedef struct
{
    uint8 row;
    uint8 side;
    uint8 status;
    uint8 lo;
    uint8 hi;
} ring_span_cache_t;
static ring_span_cache_t ring_span_cache[8];
static uint8 *ring_cache_image = 0;
static uint8 ring_p_cache_status[2];
static uint8 ring_p_cache_row[2];
static uint8 ring_p_cache_col[2];
static uint8 ring_lost_cache_status[2];
static int16 ring_lost_cache_a[2];
static int16 ring_lost_cache_b[2];
static int16 ring_lost_cache_n[2];
#endif
#if RING_ENTRY_GATE_ENABLE
static uint8 ring_probe_age = RING_ENTRY_PROBE_INTERVAL - 1;
#endif
#if RING_STATE_ROI_ENABLE
static uint8 ring_roi_valid = 0;
static uint8 ring_roi_side = 0;
static uint8 ring_roi_state = RING_STATE_NONE;
static uint8 ring_roi_row = 0;
static uint8 ring_roi_col = 0;
static uint8 ring_roi_probe_age = 0;
static uint8 ring_roi_stage = RING_ROI_STAGE_NONE;
#endif
#ifdef RING_OPT_TEST
static uint32 ring_test_span_scans = 0;
static uint32 ring_test_p_scans = 0;
static uint32 ring_test_p_candidates = 0;
static uint32 ring_test_roi_near = 0;
static uint32 ring_test_roi_wide = 0;
static uint32 ring_test_roi_global = 0;
#endif

static void ring_frame_cache_begin (uint8 *image)
{
#if RING_FRAME_CACHE_ENABLE
    uint8 i;
    ring_cache_image = image;
    for(i = 0; i < 2; i++)
    {
        ring_p_cache_status[i] = 0;
        ring_lost_cache_status[i] = 0;
    }
    for(i = 0; i < 8; i++) { ring_span_cache[i].status = 0; }
#else
    (void)image;
#endif
}

static void ring_frame_cache_end (void)
{
#if RING_FRAME_CACHE_ENABLE
    ring_cache_image = 0;
#endif
}

#define RING_FIRST_OPEN_RUN_MIN   (VISION_PX(33))
#define RING_FIRST_MOVE_MIN      (VISION_PX(20))
#define RING_SECOND_RESET_MIN    (VISION_PX(24))
#define RING_SMALL_SECOND_MOVE    (3)
#define RING_SMALL_LEFT_MOVE     (VISION_PX(16))
#define RING_SMALL_LEFT_AGE      (10)
#define RING_FIRST_MAX_FRAMES    (120)
#define RING_RIGHT_OPPO_LOST_MAX (VISION_PX(16))
#define RING_P_HOLD_FRAMES       (8)
#define RING_EXIT_HOLD_FRAMES    (2)
/* Bound the wide exit gap; fresh physical edges take priority every frame. */
#define RING_EXIT_RECOVERY_FRAMES (80)
#define RING_OUT_CLEAR_FRAMES   (8)
#define RING_EXIT_PASS_ROW      (VISION_PX(75))
#define RING_EXIT_RESET_GAP     (6)
#define RING_EXIT_LONG_GAP      (10)
#define RING_EXIT_RESTART_DROP  (VISION_PX(20))
#define RING_EXIT_RESTART_CLEAR (3)
#define RING_POST_EXIT_FRAMES   (40)

#define RING_PIXEL(image, row, col)     ((image)[(uint32)(row) * VISION_W + (col)])
#define RING_IS_WHITE(pixel)            (((pixel) >= IMG_WHITE_LEVEL) ? 1 : 0)   // 灰度直读：用编译期常量最快


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断一条边线是不是"连续直道"
// 参数说明     *edge               边线数组(track_left / track_right)
// 参数说明     mask                丢线标志位
// 返回参数     uint8               1:是连续直道  0:不是
// 备注信息     对应文章的"判断对面那条边界为连续直道"：
//              先看这段行里丢线多不多，再用最小二乘拟合一条直线看最大偏差
//-------------------------------------------------------------------------------------------------------------------
static uint8 ring_is_straight (uint8 *edge, uint8 mask, uint8 lost_max)
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

    if(lost_count > lost_max)
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
        row_p       = image + (uint32)y * VISION_W;
        prev        = (row_p[0] >= IMG_WHITE_LEVEL) ? 1 : 0;

        for(x = 0; x < VISION_W; x++)
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
static uint8 ring_find_lost_run_raw (uint8 mask, int16 *a, int16 *b, int16 *n)
{
    int16 y;
    int16 start = -1;
    int16 best_a = 0;
    int16 best_b = 0;
    int16 best_n = 0;

    *a = 0;
    *b = 0;
    *n = 0;

    for(y = 0; y <= VISION_VALID_H; y++)
    {
        if((y < VISION_VALID_H) && (0 != (track_lost[y] & mask)))
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
// 备注信息     入口需同时满足：
//              1.对面那条边线是连续直道（丢线少 + 接近直线）
//              2.入口这一侧有连续丢线，且图像白区在 P 点向该侧张开
//              3.这段行范围里对面基本不丢线
//                —— 这是圆环和十字最大的区别：十字的横杠会把左右两边同时盖住，
//                   圆环只会盖住一边（实车实测：右环 左0行/右56行）
//-------------------------------------------------------------------------------------------------------------------
static uint8 ring_find_lost_run (uint8 mask, int16 *a, int16 *b, int16 *n)
{
#if RING_FRAME_CACHE_ENABLE
    uint8 side;
    uint8 found;
    if((0 != ring_cache_image) && ((1 == mask) || (2 == mask)))
    {
        side = (uint8)(2 == mask);
        if(0 == ring_lost_cache_status[side])
        {
            found = ring_find_lost_run_raw(mask, &ring_lost_cache_a[side],
                       &ring_lost_cache_b[side], &ring_lost_cache_n[side]);
            ring_lost_cache_status[side] = (0 != found) ? 2 : 1;
        }
        *a = ring_lost_cache_a[side];
        *b = ring_lost_cache_b[side];
        *n = ring_lost_cache_n[side];
        return (uint8)(2 == ring_lost_cache_status[side]);
    }
#endif
    return ring_find_lost_run_raw(mask, a, b, n);
}

static uint8 ring_white_span_raw (uint8 *image, int16 row, uint8 to_left, int16 *lo, int16 *hi)
{
    uint8 *pixels = image + (uint32)row * VISION_W;
    uint8 *edge = (0 != to_left) ? track_right : track_left;
    uint8 mask = (0 != to_left) ? 0x02 : 0x01;
    int16 seed;
    int16 x;
    int16 offset;

#ifdef RING_OPT_TEST
    ring_test_span_scans++;
#endif
    if((row < 0) || (row >= VISION_VALID_H)) { return 0; }
    if(0 != (track_lost[row] & mask)) { return 0; }
    seed = (int16)edge[row] + ((0 != to_left) ? -VISION_PX(8) : VISION_PX(8));
    if((seed < 1) || (seed >= VISION_W - 1)) { return 0; }
    if(0 == RING_IS_WHITE(pixels[seed]))
    {
        for(offset = 1; offset <= VISION_PX(6); offset++)
        {
            if((seed - offset >= 1) && RING_IS_WHITE(pixels[seed - offset]))
            {
                seed -= offset;
                break;
            }
            if((seed + offset < VISION_W - 1) && RING_IS_WHITE(pixels[seed + offset]))
            {
                seed += offset;
                break;
            }
        }
        if(offset > VISION_PX(6)) { return 0; }
    }
    x = seed;
    while((x > 0) && RING_IS_WHITE(pixels[x - 1])) { x--; }
    *lo = x;
    x = seed;
    while((x < VISION_W - 1) && RING_IS_WHITE(pixels[x + 1])) { x++; }
    *hi = x;
    return ((*hi - *lo) >= VISION_PX(16)) ? 1 : 0;
}

static uint8 ring_white_span (uint8 *image, int16 row, uint8 to_left, int16 *lo, int16 *hi)
{
#if RING_FRAME_CACHE_ENABLE
    ring_span_cache_t *slot;
    int16 span_lo = -1;
    int16 span_hi = -1;
    uint8 found;
    if((0 != ring_cache_image) && (image == ring_cache_image))
    {
        slot = &ring_span_cache[(uint8)row & 7];
        if((0 == slot->status) || (slot->row != (uint8)row) ||
           (slot->side != to_left))
        {
            found = ring_white_span_raw(image, row, to_left, &span_lo, &span_hi);
            slot->row = (uint8)row;
            slot->side = to_left;
            slot->lo = (span_lo >= 0) ? (uint8)span_lo : 255;
            slot->hi = (span_hi >= 0) ? (uint8)span_hi : 255;
            slot->status = (0 != found) ? 2 : 1;
        }
        if(255 != slot->lo) { *lo = slot->lo; *hi = slot->hi; }
        return (uint8)(2 == slot->status);
    }
#endif
    return ring_white_span_raw(image, row, to_left, lo, hi);
}

/* Bounds are output P rows, not the y variable of the three-row stencil. */
static uint8 ring_find_p_scan_range (uint8 *image, uint8 to_left,
                                    int16 row_min, int16 row_max,
                                    uint8 *row_out, uint8 *col_out,
                                    int16 *jump_out, int16 *groups_out)
{
    int16 y;
    int16 upper_l, upper_r, lower_l, lower_r;
    int16 check_l, check_r;
    int16 upper_edge, lower_edge, upper_oppo, lower_oppo;
    int16 jump;
    int16 best_jump = 0;
    int16 best_row = -1;
    int16 best_col = -1;
    int16 last_row = -1;
    int16 last_col = -1;
    int16 groups = 0;
#if RING_FRAME_CACHE_ENABLE
    uint8 i;
    for(i = 0; i < 8; i++) { ring_span_cache[i].status = 0; }
#endif
    if(row_min < RING_P_ROW_MIN + VISION_PX(2)) { row_min = RING_P_ROW_MIN + VISION_PX(2); }
    if(row_max > RING_P_ROW_MAX - VISION_PX(6)) { row_max = RING_P_ROW_MAX - VISION_PX(6); }
    for(y = row_min + VISION_PX(2); y <= row_max + VISION_PX(2); y++)
    {
#ifdef RING_OPT_TEST
        ring_test_p_candidates++;
#endif
        if((0 == ring_white_span(image, y - VISION_PX(2), to_left, &upper_l, &upper_r)) ||
           (0 == ring_white_span(image, y + VISION_PX(2), to_left, &lower_l, &lower_r)) ||
           (0 == ring_white_span(image, y - VISION_PX(4), to_left, &check_l, &check_r)))
        {
            continue;
        }
        upper_edge = (0 != to_left) ? upper_l : upper_r;
        lower_edge = (0 != to_left) ? lower_l : lower_r;
        upper_oppo = (0 != to_left) ? upper_r : upper_l;
        lower_oppo = (0 != to_left) ? lower_r : lower_l;
        jump = (0 != to_left) ? (upper_edge - lower_edge) : (lower_edge - upper_edge);
        if((jump < RING_P_JUMP_MIN) ||
           (((0 != to_left) ? lower_edge : (VISION_W - 1 - lower_edge)) >
            (VISION_W - 1 - RING_P_BORDER_MIN)) ||
           (((0 != to_left) ? upper_edge : (VISION_W - 1 - upper_edge)) < VISION_PX(12)) ||
           ((upper_oppo - lower_oppo) > RING_P_OPPO_SHIFT_MAX) ||
           ((lower_oppo - upper_oppo) > RING_P_OPPO_SHIFT_MAX) ||
           (((0 != to_left) ? check_l : check_r) - upper_edge > RING_P_OPPO_SHIFT_MAX) ||
           (upper_edge - ((0 != to_left) ? check_l : check_r) > RING_P_OPPO_SHIFT_MAX))
        {
            continue;
        }
        /* Adjacent hits of the same corner form one group. Distinct local
         * corners trigger global arbitration instead of a partial winner. */
        if(0 != groups_out)
        {
            if((last_row < 0) || (y - last_row > VISION_PX(4)) ||
               (upper_edge - last_col > VISION_PX(4)) || (last_col - upper_edge > VISION_PX(4)))
            {
                groups++;
            }
            last_row = y;
            last_col = upper_edge;
        }
        if(jump > best_jump)
        {
            best_jump = jump;
            best_row = y - VISION_PX(2);
            best_col = upper_edge;
        }
    }
    if(best_row < 0) { return 0; }
    *row_out = (uint8)best_row;
    *col_out = (uint8)best_col;
    if(0 != jump_out) { *jump_out = best_jump; }
    if(0 != groups_out) { *groups_out = groups; }
    return 1;
}

static uint8 ring_find_p_scan (uint8 *image, uint8 to_left, uint8 *row_out, uint8 *col_out)
{
#ifdef RING_OPT_TEST
    ring_test_p_scans++;
#endif
    return ring_find_p_scan_range(image, to_left, RING_P_ROW_MIN + VISION_PX(2),
                                  RING_P_ROW_MAX - VISION_PX(6), row_out, col_out, 0, 0);
}

static uint8 ring_find_p (uint8 *image, uint8 to_left, uint8 *row_out, uint8 *col_out)
{
#if RING_FRAME_CACHE_ENABLE
    uint8 side = (uint8)(0 != to_left);
    uint8 found;
    if((0 != ring_cache_image) && (image == ring_cache_image))
    {
        if(0 == ring_p_cache_status[side])
        {
            found = ring_find_p_scan(image, to_left, &ring_p_cache_row[side],
                                    &ring_p_cache_col[side]);
            ring_p_cache_status[side] = (0 != found) ? 2 : 1;
        }
        if(2 == ring_p_cache_status[side])
        {
            *row_out = ring_p_cache_row[side];
            *col_out = ring_p_cache_col[side];
            return 1;
        }
        return 0;  /* Preserve caller coordinates on a cached miss. */
    }
#endif
    return ring_find_p_scan(image, to_left, row_out, col_out);
}

static void ring_roi_reset (void)
{
#if RING_STATE_ROI_ENABLE
    ring_roi_valid = 0;
    ring_roi_probe_age = 0;
#endif
}

static void ring_roi_commit (void)
{
#if RING_STATE_ROI_ENABLE
    if((1 == ring_diag_p_source) && (0 != ring_guide_active) &&
       (0 == ring_p_missing) && (0 == ring_guide_blocked) &&
       (0 == ring_exit_active) && (RING_TYPE_NONE != ring_type) &&
       ((RING_STATE_FIND == ring_state) || (RING_STATE_IN == ring_state) ||
        (RING_STATE_INSIDE == ring_state)))
    {
        ring_roi_valid = 1;
        ring_roi_side = (uint8)(RING_TYPE_LEFT == ring_type);
        ring_roi_state = ring_state;
        ring_roi_row = ring_corner_row;
        ring_roi_col = ring_corner_col;
    }
    else { ring_roi_reset(); }
#endif
}

#if RING_STATE_ROI_ENABLE
static uint8 ring_roi_confident (uint8 row, uint8 col, int16 jump,
                                 int16 groups, int16 row_min, int16 row_max)
{
    int16 delta = (int16)col - (int16)ring_roi_col;
    if((groups != 1) || (jump < RING_P_JUMP_MIN + RING_ROI_JUMP_MARGIN) ||
       (delta > RING_ROI_COL_MAX) || (delta < -RING_ROI_COL_MAX)) { return 0; }
    if((row_min > RING_P_ROW_MIN + VISION_PX(2)) &&
       ((int16)row <= row_min + RING_ROI_EDGE_GUARD)) { return 0; }
    if((row_max < RING_P_ROW_MAX - VISION_PX(6)) &&
       ((int16)row >= row_max - RING_ROI_EDGE_GUARD)) { return 0; }
    return 1;
}
#endif

/* Local results never enter the global P cache. A full result already
 * computed in this frame takes priority, including a cached full miss. */
static uint8 ring_find_p_follow (uint8 *image, uint8 to_left,
                                 uint8 *row_out, uint8 *col_out)
{
#if RING_STATE_ROI_ENABLE
    uint8 row;
    uint8 col;
    int16 row_min;
    int16 row_max;
    int16 jump;
    int16 groups;
    ring_roi_stage = RING_ROI_STAGE_GLOBAL;
#if RING_FRAME_CACHE_ENABLE
    if((image == ring_cache_image) && (0 != ring_cache_image) &&
       (0 != ring_p_cache_status[(uint8)(0 != to_left)]))
    {
        ring_roi_probe_age = 0;
        return ring_find_p(image, to_left, row_out, col_out);
    }
#endif
    if((0 != ring_roi_valid) && (to_left == ring_roi_side) &&
       (ring_roi_state == ring_state) &&
       ((RING_STATE_FIND == ring_state) || (RING_STATE_IN == ring_state) ||
        (RING_STATE_INSIDE == ring_state)) &&
       (0 == ring_guide_blocked) && (0 == ring_p_missing) &&
       (0 == ring_exit_active) && (RING_TYPE_NONE == ring_post_exit_type) &&
       (to_left == (uint8)(RING_TYPE_LEFT == ring_type)) &&
       (ring_roi_row >= RING_P_ROW_MIN + VISION_PX(2)) &&
       (ring_roi_row <= RING_P_ROW_MAX - VISION_PX(6)) &&
       (ring_roi_probe_age < RING_ROI_PROBE_INTERVAL - 1))
    {
        row_min = (int16)ring_roi_row - RING_ROI_RADIUS_NEAR;
        row_max = (int16)ring_roi_row + RING_ROI_RADIUS_NEAR;
#ifdef RING_OPT_TEST
        ring_test_roi_near++;
#endif
        if(ring_find_p_scan_range(image, to_left, row_min, row_max,
                                  &row, &col, &jump, &groups))
        {
            if(!ring_roi_confident(row, col, jump, groups, row_min, row_max))
            {
                goto ring_roi_global;
            }
            *row_out = row;
            *col_out = col;
            ring_roi_stage = RING_ROI_STAGE_NEAR;
            ring_roi_probe_age++;
            return 1;
        }
        row_min = (int16)ring_roi_row - RING_ROI_RADIUS_WIDE;
        row_max = (int16)ring_roi_row + RING_ROI_RADIUS_WIDE;
#ifdef RING_OPT_TEST
        ring_test_roi_wide++;
#endif
        if(ring_find_p_scan_range(image, to_left, row_min, row_max,
                                  &row, &col, &jump, &groups) &&
           ring_roi_confident(row, col, jump, groups, row_min, row_max))
        {
            *row_out = row;
            *col_out = col;
            ring_roi_stage = RING_ROI_STAGE_WIDE;
            ring_roi_probe_age++;
            return 1;
        }
    }
ring_roi_global:
    ring_roi_probe_age = 0;
#ifdef RING_OPT_TEST
    ring_test_roi_global++;
#endif
#endif
    return ring_find_p(image, to_left, row_out, col_out);
}

#if RING_ENTRY_GATE_ENABLE
/* Only reject cold, complete and geometrically quiet raw boundaries.
 * Read lost bits, not summary counters: seed retry rows can set bits without
 * increasing track_*_lost_count. Even one lost row forces both-side search. */
static uint8 ring_entry_may_need_scan (uint8 *image)
{
    int16 a;
    int16 b;
    int16 n;
    int16 y;
    int16 left;
    int16 right;
    int16 width;
    int16 delta;
    uint8 *pixels = image;
    if(0 == image) { return 1; }
    if((RING_STATE_NONE != ring_state) || (0 != ring_guide_blocked) ||
       (0 != ring_confirm) || (0 != ring_first_open_seen) ||
       (0 != ring_first_open_count) || (0 != ring_first_ready) ||
       (0 != ring_first_missing) || (0 != ring_wait_second) ||
       (0 != ring_second_ready) || (0 != ring_p_left_valid) ||
       (0 != ring_p_right_valid) ||
       ((0 != ring_left_p_peak_col) && (ring_left_p_peak_age <= RING_SMALL_LEFT_AGE)) ||
       (RING_TYPE_NONE != ring_post_exit_type)) { return 1; }
    if(ring_find_lost_run(1, &a, &b, &n) ||
       ring_find_lost_run(2, &a, &b, &n)) { return 1; }
    for(y = 0; y < VISION_VALID_H; y++)
    {
        left = track_left[y];
        right = track_right[y];
        width = right - left;
        if((left <= RING_GATE_BORDER_MARGIN) ||
           (right >= VISION_W - 1 - RING_GATE_BORDER_MARGIN) ||
           (width < VISION_PX(16)) || (width >= RING_WHITE_MIN)) { return 1; }
        /* Raw tracking may lag a newly visible P. Check physical consistency
         * before skipping its first history sample. +/-2 tolerates the one-
         * pixel black/white edge convention used by different track methods. */
        if(RING_IS_WHITE(pixels[left - 2]) ||
           RING_IS_WHITE(pixels[right + VISION_PX(2)]) ||
           !RING_IS_WHITE(pixels[left + VISION_PX(2)]) ||
           !RING_IS_WHITE(pixels[right - VISION_PX(2)]) ||
           !RING_IS_WHITE(pixels[left + VISION_PX(8)]) ||
           !RING_IS_WHITE(pixels[right - VISION_PX(8)]) ||
           !RING_IS_WHITE(pixels[(left + right) / 2])) { return 1; }
        pixels += VISION_W;
        if(y < VISION_PX(4)) { continue; }
        delta = left - (int16)track_left[y - VISION_PX(4)];
        if((delta >= RING_GATE_EDGE_JUMP) || (delta <= -RING_GATE_EDGE_JUMP)) { return 1; }
        delta = right - (int16)track_right[y - VISION_PX(4)];
        if((delta >= RING_GATE_EDGE_JUMP) || (delta <= -RING_GATE_EDGE_JUMP)) { return 1; }
        delta = width - ((int16)track_right[y - VISION_PX(4)] - (int16)track_left[y - VISION_PX(4)]);
        if((delta >= RING_GATE_EDGE_JUMP) || (delta <= -RING_GATE_EDGE_JUMP)) { return 1; }
    }
    return 0;
}

static uint8 ring_entry_scan_due (uint8 *image)
{
    if(0 != ring_entry_may_need_scan(image))
    {
        ring_probe_age = 0;
        return 1;
    }
    if(ring_probe_age >= RING_ENTRY_PROBE_INTERVAL - 1)
    {
        ring_probe_age = 0;
        return 1;
    }
    ring_probe_age++;
    return 0;
}
#endif

static uint8 ring_border_white_edge (uint8 *image, int16 row, uint8 to_left,
                                     int16 *edge_out)
{
    int16 x = (0 != to_left) ? 2 : VISION_W - 3;
    int16 limit = (0 != to_left) ? VISION_PX(10) : VISION_W - VISION_PX(11);

    while((0 != to_left) ? (x < limit) : (x > limit))
    {
        if(RING_IS_WHITE(RING_PIXEL(image, row, x))) { break; }
        x += (0 != to_left) ? 1 : -1;
    }
    if(!RING_IS_WHITE(RING_PIXEL(image, row, x))) { return 0; }
    if(0 != to_left)
    {
        while((x < VISION_W - 1) &&
              RING_IS_WHITE(RING_PIXEL(image, row, x + 1))) { x++; }
    }
    else
    {
        while((x > 0) && RING_IS_WHITE(RING_PIXEL(image, row, x - 1))) { x--; }
    }
    *edge_out = x;
    return 1;
}

// Only track a border-connected wedge after the ring side is already confirmed.
static uint8 ring_find_p_wedge_tip (uint8 *image, uint8 to_left,
                                    uint8 *row_out, uint8 *col_out)
{
    int16 y;
    int16 before;
    int16 upper;
    int16 lower;
    int16 jump;
    int16 slope;

    for(y = RING_P_ROW_MAX - VISION_PX(4); y >= RING_P_ROW_MIN + VISION_PX(12); y--)
    {
        if((0 == ring_border_white_edge(image, y - VISION_PX(12), to_left, &before)) ||
           (0 == ring_border_white_edge(image, y - VISION_PX(2), to_left, &upper)) ||
           (0 == ring_border_white_edge(image, y + VISION_PX(2), to_left, &lower)))
        {
            continue;
        }
        jump = (0 != to_left) ? lower - upper : upper - lower;
        slope = (0 != to_left) ? upper - before : before - upper;
        if((jump < RING_P_JUMP_MIN) || (slope < VISION_PX(10)) ||
           ((0 != to_left) ? (lower < VISION_W - VISION_PX(10)) : (lower > VISION_PX(9))) ||
           (upper < VISION_PX(12)) || (upper > VISION_W - VISION_PX(13)))
        {
            continue;
        }
        *row_out = (uint8)(y - 2);
        *col_out = (uint8)upper;
        return 1;
    }
    return 0;
}

static void ring_build_upper_curve (uint8 *image, uint8 to_left)
{
    int16 y;
    int16 x = ring_corner_col;
    int16 edge;
    int16 step;

    ring_upper_curve[ring_corner_row] = (uint8)x;
    if(0 != to_left)
    {
        for(y = (int16)ring_corner_row - 1; y >= 0; y--)
        {
            ring_upper_curve[y] = (uint8)(((int32)ring_corner_col * y +
                                         ring_corner_row / 2) / ring_corner_row);
        }
        ring_upper_curve_active = 1;
        return;
    }
    for(y = (int16)ring_corner_row - 1; y >= 0; y--)
    {
        if((0 == ring_border_white_edge(image, y, 0, &edge)) ||
           (edge < x - 2) || (edge > x + 8))
        {
            break;
        }
        ring_upper_curve[y] = (uint8)edge;
        x = edge;
    }
    if(y >= 0)
    {
        step = ((VISION_W - 1 - x) + y) / (y + 1);
        if(y + VISION_PX(5) <= ring_corner_row)
        {
            int16 traced_step = (x - ring_upper_curve[y + VISION_PX(5)] + VISION_PX(2)) / VISION_PX(4);
            if(traced_step > step) { step = traced_step; }
        }
        if(step < 1) { step = 1; }
        if(step > 6) { step = 6; }
        for(; y >= 0; y--)
        {
            x += step;
            if(x >= VISION_W) { x = VISION_W - 1; }
            ring_upper_curve[y] = (uint8)x;
        }
    }
    ring_upper_curve_active = 1;
}

static uint8 ring_find_exit_corner (uint8 type, uint8 *row_out, uint8 *col_out)
{
    uint8 mask = (RING_TYPE_RIGHT == type) ? 0x01 : 0x02;
    uint8 *edge = (RING_TYPE_RIGHT == type) ? track_left : track_right;
    int16 a = 0;
    int16 b = 0;
    int16 n = 0;
    int16 y;
    int16 best_row = -1;
    int16 best_x = 0;
    int16 best_score = 0;
    int16 good_count = 0;
    int16 bottom_sum = 0;
    int16 bottom_count = 0;
    int16 upper_shift;
    int16 lower_shift;
    int16 bottom_shift;
    int16 score;

    if((RING_TYPE_RIGHT != type) && (RING_TYPE_LEFT != type)) { return 0; }
    ring_diag_exit_probe = 2;
    for(y = VISION_H - VISION_PX(12); y < VISION_VALID_H; y++)
    {
        if(0 == (track_lost[y] & mask))
        {
            bottom_sum += edge[y];
            bottom_count++;
        }
    }
    if(bottom_count < VISION_PX(4))
    {
        ring_diag_exit_probe = 1;
        return 0;
    }

    if((0 != ring_find_lost_run(mask, &a, &b, &n)) &&
       (n >= VISION_PX(20)) && (b >= VISION_PX(22)) && (b <= VISION_H - VISION_PX(16)))
    {
        ring_diag_exit_probe = 3;
        for(y = b + 1; y < VISION_H - VISION_PX(5); y++)
        {
            if((0 != (track_lost[y] & mask)) ||
               (0 != (track_lost[y + 1] & mask)) ||
               ((int16)edge[y] - (int16)edge[y + 1] > 9) ||
               ((int16)edge[y + 1] - (int16)edge[y] > 9))
            {
                continue;
            }
            good_count++;
            if((best_row < 0) ||
               ((RING_TYPE_RIGHT == type) ? (edge[y] > best_x) : (edge[y] < best_x)))
            {
                best_x = edge[y];
                best_row = y;
            }
        }
        if((good_count >= VISION_PX(8)) && (best_row >= 0) &&
           (best_x >= VISION_PX(24)) && (best_x <= VISION_W - VISION_PX(25)))
        {
            bottom_shift = (RING_TYPE_RIGHT == type) ?
                           (best_x - bottom_sum / bottom_count) :
                           (bottom_sum / bottom_count - best_x);
            if(bottom_shift >= VISION_PX(12))
            {
                *row_out = (uint8)best_row;
                *col_out = (uint8)best_x;
                ring_diag_exit_probe = 6;
                return 1;
            }
        }
    }

    // A fully traced exit can turn without a long lost run; require both legs of that turn.
    if(((RING_TYPE_RIGHT == type) ? track_right_lost_count : track_left_lost_count) < VISION_PX(4))
    {
        if(2 == ring_diag_exit_probe) { ring_diag_exit_probe = 4; }
        return 0;
    }
    best_row = -1;
    for(y = VISION_PX(16); y < VISION_H - VISION_PX(16); y++)
    {
        if((0 != (track_lost[y - VISION_PX(12)] & mask)) ||
           (0 != (track_lost[y - 2] & mask)) ||
           (0 != (track_lost[y] & mask)) ||
           (0 != (track_lost[y + 1] & mask)) ||
           (0 != (track_lost[y + 2] & mask)) ||
           (0 != (track_lost[y + VISION_PX(12)] & mask)) ||
           ((int16)edge[y] - (int16)edge[y + 1] > 9) ||
           ((int16)edge[y + 1] - (int16)edge[y] > 9) ||
           (edge[y] < VISION_PX(24)) || (edge[y] > VISION_W - VISION_PX(25)))
        {
            continue;
        }
        if(RING_TYPE_RIGHT == type)
        {
            upper_shift = (int16)edge[y] - (int16)edge[y - VISION_PX(12)];
            lower_shift = (int16)edge[y] - (int16)edge[y + VISION_PX(12)];
            bottom_shift = (int16)edge[y] - bottom_sum / bottom_count;
        }
        else
        {
            upper_shift = (int16)edge[y - VISION_PX(12)] - (int16)edge[y];
            lower_shift = (int16)edge[y + VISION_PX(12)] - (int16)edge[y];
            bottom_shift = bottom_sum / bottom_count - (int16)edge[y];
        }
        if((upper_shift < VISION_PX(12)) || (lower_shift < VISION_PX(8)) || (bottom_shift < VISION_PX(18)))
        {
            continue;
        }
        score = upper_shift + lower_shift;
        if((best_row < 0) || (score > best_score))
        {
            best_score = score;
            best_x = edge[y];
            best_row = y;
        }
    }
    if(best_row < 0)
    {
        if(2 == ring_diag_exit_probe) { ring_diag_exit_probe = 5; }
        return 0;
    }
    *row_out = (uint8)best_row;
    *col_out = (uint8)best_x;
    ring_diag_exit_probe = 7;
    return 1;
}

// Continue a visible straight edge through its missing lower segment after exit.
static uint8 ring_find_exit_tail (uint8 type, uint8 *image)
{
    uint8 mask = (RING_TYPE_LEFT == type) ? 0x01 : 0x02;
    uint8 other_mask = (RING_TYPE_LEFT == type) ? 0x02 : 0x01;
    uint8 *edge = (RING_TYPE_LEFT == type) ? track_left : track_right;
    int16 last;
    int16 y;
    int16 lost;
    int16 other_valid;
    int16 delta;
    int16 bottom;
    int16 sample;
    int16 sample_x;

    if((RING_TYPE_LEFT != type) && (RING_TYPE_RIGHT != type)) { return 0; }
    for(last = VISION_H - VISION_PX(30); last >= VISION_PX(30); last--)
    {
        if((0 != (track_lost[last] & mask)) ||
           (0 == (track_lost[last + 1] & mask))) { continue; }
        for(y = last - VISION_PX(18); y <= last; y++)
        {
            if((0 != (track_lost[y] & mask)) ||
               ((y > last - VISION_PX(18)) &&
                (((int16)edge[y] - (int16)edge[y - 1] > 3) ||
                 ((int16)edge[y - 1] - (int16)edge[y] > 3)))) { break; }
        }
        if(y <= last) { continue; }
        lost = 0;
        other_valid = 0;
        for(y = last + 1; y < VISION_LAST_ROW; y++)
        {
            if(0 != (track_lost[y] & mask)) { lost++; }
            if(0 == (track_lost[y] & other_mask)) { other_valid++; }
        }
        if(lost < VISION_PX(24)) { continue; }
        delta = (int16)edge[last] - (int16)edge[last - VISION_PX(18)];
        if((RING_TYPE_LEFT == type) ? ((delta > -VISION_PX(4)) || (delta < -VISION_PX(28))) :
                                      ((delta < VISION_PX(4)) || (delta > VISION_PX(28)))) { continue; }
        bottom = (int16)edge[last] + delta * (VISION_LAST_ROW - last) / VISION_PX(18);
        if((bottom < -VISION_PX(10)) || (bottom > VISION_W + VISION_PX(9))) { continue; }
        if(other_valid < VISION_PX(18))
        {
            if(((0 == ring_exit_seen) && (ring_post_exit_type != type)) ||
               (((ring_exit_max_row > ring_post_exit_max_row) ?
                 ring_exit_max_row : ring_post_exit_max_row) < VISION_PX(60)) ||
               (last < VISION_PX(50)) ||
               (0 != (track_lost[last] & other_mask))) { continue; }
            for(sample = 0; sample < 3; sample++)
            {
                y = (sample == 0) ? last + VISION_PX(8) :
                    ((sample == 1) ? (last + VISION_LAST_ROW) / 2 : VISION_H - VISION_PX(4));
                sample_x = (int16)edge[last] + delta * (y - last) / VISION_PX(18) +
                           ((RING_TYPE_LEFT == type) ? VISION_PX(8) : -VISION_PX(8));
                if((sample_x < 1) || (sample_x >= VISION_W - 1) ||
                   (0 == RING_IS_WHITE(RING_PIXEL(image, y, sample_x)))) { break; }
            }
            if(sample < 3) { continue; }
        }
        ring_exit_tail_row = (uint8)last;
        ring_exit_tail_col = edge[last];
        ring_exit_tail_delta = delta;
        ring_exit_tail_unpaired = (uint8)(other_valid < VISION_PX(18));
        return 1;
    }
    return 0;
}

// A real left-roundabout entry has a near-side arc protruding into the lane.
// This rejects the straight-sided island seen before the left entrance.
static uint8 ring_left_arc_visible (uint8 corner_row)
{
    int16 row;
    int16 near_sum = 0;
    int16 far_sum = 0;
    int16 near_count = 0;
    int16 far_count = 0;

    if((int16)corner_row + VISION_PX(58) >= VISION_VALID_H) { return 0; }
    for(row = (int16)corner_row + VISION_PX(22); row <= (int16)corner_row + VISION_PX(32); row++)
    {
        if(0 == (track_lost[row] & 0x01))
        {
            near_sum += track_left[row];
            near_count++;
        }
    }
    for(row = (int16)corner_row + VISION_PX(45); row <= (int16)corner_row + VISION_PX(58); row++)
    {
        if(0 == (track_lost[row] & 0x01))
        {
            far_sum += track_left[row];
            far_count++;
        }
    }
    if((near_count < VISION_PX(2)) || (far_count < VISION_PX(6))) { return 0; }
    return (uint8)((far_sum / far_count) - (near_sum / near_count) >=
                   ((near_count < VISION_PX(5)) ? VISION_PX(12) : VISION_PX(8)));
}

/* A small left island has a local maximum in x below its second opening:
 * its upper edge moves into the lane, then its lower arc recedes again.
 * Test this shape directly; a straight first-opening edge is monotone.
 * No floats, history reset, extra image-wide scan, or change to global P. */
static uint8 ring_island_visible(uint8 corner_row, int16 opening_end, int16 opening_length, uint8 to_left)
{
    int16 y, start, end, x, first = -1, last = -1, peak = -1, peak_row = 0;
    int16 valid = 0, gaps = 0, rises = 0, falls = 0, previous = -1;
    if((corner_row < RING_P_ROW_MIN + VISION_PX(2)) || (corner_row > VISION_PX(32)) ||
       (opening_length < RING_P_OPEN_LOST_MIN) ||
       (opening_length > ((0 != to_left) ? RING_FIRST_OPEN_RUN_MIN - 1 : VISION_PX(40))) ||
       (opening_end < (int16)corner_row + ((0 != to_left) ? VISION_PX(12) : VISION_PX(6))) ||
       (opening_end > (int16)corner_row + VISION_PX(40))) { return 0; }
    start = opening_end + 2;
    end = opening_end + VISION_PX(70);
    if(end > VISION_H - VISION_PX(8)) { end = VISION_H - VISION_PX(8); }
    for(y = start; y <= end; y++)
    {
        x = (0 != to_left) ? track_left[y] : VISION_W - 1 - track_right[y];
        if((track_lost[y] & 0x03) || (x < VISION_PX(5)) || (x > VISION_W / 2))
        {
            if(first < 0 && y <= start + VISION_PX(3)) { continue; }
            if(++gaps > 2) { break; }
            continue;
        }
        if(first < 0) { first = x; start = y; }
        if(x > peak) { peak = x; peak_row = y; }
        if(previous >= 0)
        {
            if(x > previous) { rises++; }
            else if(x < previous) { falls++; }
        }
        previous = x;
        last = x;
        valid++;
    }
    return (uint8)((valid >= VISION_PX(24)) && (peak - first >= VISION_PX(8)) &&
                   (peak - last >= VISION_PX(12)) && (peak_row - start >= VISION_PX(6)) &&
                   (y - peak_row >= VISION_PX(12)) && (rises >= VISION_PX(6)) && (falls >= VISION_PX(8)));
}

static uint8 ring_left_island_visible(uint8 corner_row, int16 opening_end, int16 opening_length)
{
    return ring_island_visible(corner_row, opening_end, opening_length, 1);
}

static uint8 ring_right_island_visible(uint8 corner_row, int16 opening_end, int16 opening_length)
{
    return ring_island_visible(corner_row, opening_end, opening_length, 0);
}

// An early right-side P must have an inward-bending island edge below it.
// A straight diagonal edge can otherwise look like a roundabout opening.
static uint8 ring_right_arc_visible (uint8 corner_row)
{
    int16 row;
    int16 near_sum = 0;
    int16 near_count = 0;
    int16 far_count = 0;
    int16 far_min = VISION_W;
    int16 early_sum = 0;
    int16 early_count = 0;
    int16 curve_count = 0;
    int16 curve_min = VISION_W;

    if((int16)corner_row + VISION_PX(65) >= VISION_VALID_H) { return 0; }
    for(row = (int16)corner_row + VISION_PX(30); row <= (int16)corner_row + VISION_PX(42); row++)
    {
        if(0 == (track_lost[row] & 0x02))
        {
            near_sum += track_right[row];
            near_count++;
        }
    }
    for(row = (int16)corner_row + VISION_PX(45); row <= (int16)corner_row + VISION_PX(65); row++)
    {
        if(0 == (track_lost[row] & 0x02))
        {
            if(track_right[row] < far_min) { far_min = track_right[row]; }
            far_count++;
        }
    }
    if((near_count >= VISION_PX(5)) && (far_count >= VISION_PX(5)) &&
       ((near_sum / near_count) - far_min >= 3)) { return 1; }
    for(row = (int16)corner_row + VISION_PX(8); row <= (int16)corner_row + VISION_PX(18); row++)
    {
        if(0 == (track_lost[row] & 0x02))
        {
            early_sum += track_right[row];
            early_count++;
        }
    }
    for(row = (int16)corner_row + VISION_PX(30); row <= (int16)corner_row + VISION_PX(50); row++)
    {
        if(0 == (track_lost[row] & 0x02))
        {
            if(track_right[row] < curve_min) { curve_min = track_right[row]; }
            curve_count++;
        }
    }
    return (uint8)((early_count >= VISION_PX(6)) && (curve_count >= VISION_PX(8)) &&
                   ((early_sum / early_count) - curve_min >= VISION_PX(12)));
}

static void ring_first_open_reset (void)
{
    ring_first_open_seen = 0;
    ring_first_open_count = 0;
    ring_first_open_col = 0;
    ring_first_min_col = VISION_W - 1;
    ring_first_missing = 0;
    ring_first_ready = 0;
    ring_first_age = 0;
    ring_wait_second = 0;
    ring_second_ready = 0;
}

static void ring_right_opening_update (void)
{
    int16 a = 0;
    int16 b = 0;
    int16 n = 0;
    uint8 col = ring_p_right_col;
    uint8 was_missing = ring_first_missing;

    ring_wait_second = 0;
    ring_second_ready = 0;
    if((0 != ring_first_open_seen) && (ring_first_age < RING_FIRST_MAX_FRAMES + 1))
    {
        ring_first_age++;
    }
    if(ring_first_age > RING_FIRST_MAX_FRAMES) { ring_first_open_reset(); }
    if(0 == ring_p_right_valid)
    {
        ring_first_open_count = 0;
        if(0 != ring_first_open_seen)
        {
            if(ring_first_missing < 2) { ring_first_missing++; }
            if(ring_first_missing >= 2) { ring_first_ready = 1; }
        }
        return;
    }
    if(0 == ring_find_lost_run(0x02, &a, &b, &n)) { return; }

    if(n >= RING_FIRST_OPEN_RUN_MIN)
    {
        ring_wait_second = 1;
        ring_first_missing = 0;
        if(0 == ring_first_open_seen)
        {
            if((0 == ring_first_open_count) ||
               (((int16)col - (int16)ring_first_open_col <= VISION_PX(16)) &&
                ((int16)ring_first_open_col - (int16)col <= VISION_PX(16))))
            {
                if(ring_first_open_count < 2) { ring_first_open_count++; }
            }
            else
            {
                ring_first_open_count = 1;
            }
            ring_first_open_col = col;
            if(ring_first_open_count >= 2)
            {
                ring_first_open_seen = 1;
                ring_first_min_col = col;
            }
        }
    }
    else if(0 == ring_first_open_seen)
    {
        // A short opening can be the second opening if the first was off-screen.
        ring_first_open_count = 0;
        ring_second_ready = 1;
    }
    else
    {
        ring_first_missing = 0;
    }

    if(0 != ring_first_open_seen)
    {
        if(col > ring_first_open_col) { ring_first_open_col = col; }
        if(col < ring_first_min_col) { ring_first_min_col = col; }
        if((int16)ring_first_open_col - (int16)ring_first_min_col >= RING_FIRST_MOVE_MIN)
        {
            ring_first_ready = 1;
        }
        if((n < RING_FIRST_OPEN_RUN_MIN) && (0 != ring_first_ready) &&
           ((was_missing >= 2) ||
            ((int16)col - (int16)ring_first_min_col >= RING_SECOND_RESET_MIN) ||
            ((n <= RING_SECOND_RESET_MIN) && (ring_p_right_row <= VISION_PX(24)) &&
             ((int16)col - (int16)ring_first_min_col >= RING_SMALL_SECOND_MOVE) &&
             (0 != ring_right_arc_visible(ring_p_right_row)))))
        {
            ring_second_ready = 1;
            ring_wait_second = 0;
        }
        else
        {
            ring_wait_second = 1;
        }
    }
}

static uint8 ring_try_side_check (uint8 to_left, uint8 primary, uint8 commit)
{
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

    if((RING_STATE_NONE != ring_state) && (commit || (RING_STATE_OUT != ring_state)))                                             // 已经在圆环状态里就不重复识别
    {
        return 0;
    }
    if((0 == to_left) && (0 == ring_second_ready) && !ring_cross_right_entry)
    {
        if(0 == ring_fail) { ring_fail = -8; }
        return 0;
    }

    // ---- 条件 1：对面是连续直道 ----
    ring_straight_fail = 0;                                         // 每次重新判 不然会残留上一次的结果

    if(0 == ring_is_straight(oppo_edge, oppo_mask,
            (0 != to_left) ? RING_STRAIGHT_LOST_MAX : RING_RIGHT_OPPO_LOST_MAX))
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
        if((n < RING_P_OPEN_LOST_MIN) &&
           !((0 == to_left) && (n >= VISION_PX(4)) &&
              (0 != ring_p_right_valid) && (ring_p_right_row <= VISION_PX(24)) &&
              (0 != ring_right_arc_visible(ring_p_right_row))))
        {
            if(0 == ring_fail)
            {
                ring_fail     = -2;                             // 入口侧道路没张开
                ring_rep_lost = (uint8)n;                       // 把实测到的丢线段长度报出来 差多少一眼看得出
            }
            return 0;
        }
        if((0 == to_left) && (n >= RING_FIRST_OPEN_RUN_MIN) && !ring_cross_right_entry)
        {
            if(0 == ring_fail) { ring_fail = -8; }
            return 0;
        }
        if((0 != to_left) && (n >= RING_FIRST_OPEN_RUN_MIN) &&
           (b < VISION_H - VISION_PX(15)) &&
           !((b >= VISION_H * 3 / 4) &&
             (0 == (track_lost[VISION_LAST_ROW] & 0x01)) &&
             (track_left[VISION_LAST_ROW] < VISION_W / 7)))
        {
            if(0 != ring_p_left_valid) { ring_wait_second = 1; }
            if(0 == ring_fail) { ring_fail = -8; }
            return 0;
        }
        if((0 != to_left) && (n >= RING_FIRST_OPEN_RUN_MIN) &&
           (b >= VISION_H - VISION_PX(8)))
        {
            if(0 == ring_fail) { ring_fail = -8; }
            return 0;
        }
    }
    else
    {
        if(0 == ring_fail) { ring_fail = -2; }
        return 0;
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
        if((y < 0) || (y >= VISION_VALID_H))
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

    if(0 != to_left)
    {
        row = ring_p_left_row;
        col = ring_p_left_col;
    }
    else
    {
        row = ring_p_right_row;
        col = ring_p_right_col;
    }
    if((0 != to_left) ? (0 == ring_p_left_valid) : (0 == ring_p_right_valid))
    {
        if(0 == ring_fail) { ring_fail = -4; }
        return 0;
    }

    /* A global P from the car/another branch must belong to this opening. */
    if((0 == to_left) && ((row > VISION_H * 3 / 5) ||
                         ((int16)row < a - VISION_PX(4)) ||
                         ((int16)row > b + VISION_PX(4))))
    {
        if(0 == ring_fail) { ring_fail = -4; }
        return 0;
    }

    if((0 != to_left) && (col < VISION_W / 4))
    {
        if(0 == ring_fail) { ring_fail = -8; }
        return 0;
    }

    if((0 != to_left) && (row <= VISION_PX(32)) && (0 == ring_left_arc_visible(row)) &&
       !((row >= VISION_PX(20)) && (col >= VISION_W / 4) &&
         (col <= VISION_W / 2) && (n >= VISION_PX(16)) && (n <= VISION_PX(32)) &&
         (ring_left_p_peak_age <= RING_SMALL_LEFT_AGE) &&
         ((int16)ring_left_p_peak_col - (int16)col >= RING_SMALL_LEFT_MOVE) &&
         (b >= (int16)row + VISION_PX(18)) && (b <= VISION_H * 3 / 5) &&
         (0 == (track_lost[b + 1] & 0x01)) &&
         (0 == (track_lost[VISION_LAST_ROW] & 0x01)) &&
         ((int16)track_left[b + 1] - (int16)track_left[VISION_LAST_ROW] >= VISION_PX(12))) &&
       (0 == ring_left_island_visible(row, b, n)))
    {
        if(0 == ring_fail) { ring_fail = -6; }
        return 0;
    }
    if((0 == to_left) && (row <= VISION_PX(24)) &&
       (0 == ring_right_arc_visible(row)) && !ring_cross_right_entry)
    {
        if(0 == ring_fail) { ring_fail = -7; }
        return 0;
    }

    if(commit)
    {
        ring_open_row1  = (uint8)a;
        ring_open_row2  = (uint8)b;
        ring_corner_row = row;
        ring_corner_col = col;
    }
    return 1;
}

static uint8 ring_try_side (uint8 to_left, uint8 primary)
{
    return ring_try_side_check(to_left, primary, 1);
}

/* A cancel caused by cross recognition is different from passing a ring.
 * After the cross has ended, two unique small-left island geometries may
 * release that block even if a P-free gap never occurs. Old same-ring exit
 * blocks cannot use this path. */
static uint8 ring_cross_rearm_ready(uint8 left_valid, uint8 row, uint8 col, uint8 right_valid)
{
    int16 a = 0, b = 0, n = 0;
    int8 saved_fail;
    uint8 accepted = 0;
    if(ring_block_after_cross && left_valid && !right_valid &&
       !cross_flag && !cross_bridge_used &&
       ((CROSS_STATE_NONE == cross_state) || (CROSS_STATE_OUT == cross_state)))
    {
        ring_find_lost_run(0x01, &a, &b, &n);
        if((col >= VISION_W / 4) && ring_left_island_visible(row, b, n))
        {
            ring_p_left_valid = left_valid;
            ring_p_left_row = row;
            ring_p_left_col = col;
            saved_fail = ring_fail;
            accepted = ring_try_side_check(1, 0, 0);
            ring_fail = saved_fail;
        }
    }
    if(!accepted) { ring_cross_confirm = 0; ring_cross_confirm_type = RING_TYPE_NONE; return 0; }
    if(RING_TYPE_LEFT != ring_cross_confirm_type) { ring_cross_confirm = 0; }
    ring_cross_confirm_type = RING_TYPE_LEFT;
    if(((int16)row - ring_cross_confirm_row > VISION_PX(12)) ||
       ((int16)ring_cross_confirm_row - row > VISION_PX(12)) ||
       ((int16)col - ring_cross_confirm_col > VISION_PX(16)) ||
       ((int16)ring_cross_confirm_col - col > VISION_PX(16))) { ring_cross_confirm = 0; }
    ring_cross_confirm_row = row;
    ring_cross_confirm_col = col;
    if(ring_cross_confirm < RING_CONFIRM_FRAME) { ring_cross_confirm++; }
    return (uint8)(ring_cross_confirm >= RING_CONFIRM_FRAME);
}

/* A right island immediately after a cross may have no P-free gap. Only
 * fresh, unique far-P geometry can release a cross block; an old ring-exit
 * block cannot use this path. Opposite-edge and white-span tests stay intact. */
static uint8 ring_cross_right_rearm_ready(uint8 right_valid, uint8 row, uint8 col, uint8 left_valid)
{
    int16 a = 0, b = 0, n = 0;
    int8 saved_fail;
    uint8 accepted = 0;
    if(ring_block_after_cross && right_valid && !left_valid &&
       !cross_flag && !cross_bridge_used &&
       ((CROSS_STATE_NONE == cross_state) || (CROSS_STATE_OUT == cross_state)) &&
       (row <= VISION_PX(32)) && (col >= VISION_W / 3) &&
       (col <= VISION_W * 3 / 4) &&
       ring_find_lost_run(0x02, &a, &b, &n) &&
       (a <= (int16)row + VISION_PX(4)) && (row <= b + VISION_PX(4)) &&
       (n >= RING_P_OPEN_LOST_MIN) &&
       (n <= VISION_PX(40)) && ring_right_island_visible(row, b, n))
    {
        ring_p_right_valid = 1;
        ring_p_right_row = row;
        ring_p_right_col = col;
        saved_fail = ring_fail;
        ring_cross_right_entry = 1;
        accepted = ring_try_side_check(0, 0, 0);
        ring_fail = saved_fail;
        if(!accepted) { ring_cross_right_entry = 0; }
    }
    if(!accepted)
    {
        ring_cross_confirm = 0;
        ring_cross_confirm_type = RING_TYPE_NONE;
        return 0;
    }
    if((RING_TYPE_RIGHT != ring_cross_confirm_type) ||
       ((int16)row - ring_cross_confirm_row > VISION_PX(12)) ||
       ((int16)ring_cross_confirm_row - row > VISION_PX(12)) ||
       ((int16)col - ring_cross_confirm_col > VISION_PX(16)) ||
       ((int16)ring_cross_confirm_col - col > VISION_PX(16))) { ring_cross_confirm = 0; }
    ring_cross_confirm_type = RING_TYPE_RIGHT;
    ring_cross_confirm_row = row;
    ring_cross_confirm_col = col;
    if(ring_cross_confirm < RING_CONFIRM_FRAME) { ring_cross_confirm++; }
    return (uint8)(ring_cross_confirm >= RING_CONFIRM_FRAME);
}

static void ring_chain_reset(void)
{
    ring_chain.frames = 0;
    ring_chain.source = RING_TYPE_NONE;
    ring_chain.pass_row = 0;
    ring_chain.peak_col = 0;
    ring_chain.peak_age = 0;
    ring_chain.confirm = 0;
    ring_chain.row = 0;
    ring_chain.col = 0;
    ring_chain.seen = 0;
}

static void ring_chain_arm(uint8 source, uint8 pass_row)
{
    if((RING_TYPE_NONE == source) || (pass_row < RING_EXIT_PASS_ROW)) { return; }
    /* Do not restart the window every OUT frame or replenish an expired one. */
    if(ring_chain.seen && (source == ring_chain.source))
    {
        if(pass_row > ring_chain.pass_row) { ring_chain.pass_row = pass_row; }
        return;
    }
    ring_chain_reset();
    ring_chain.frames = RING_CHAIN_WINDOW_FRAMES + 1;
    ring_chain.source = source;
    ring_chain.pass_row = pass_row;
    ring_chain.seen = 1;
}

/* Only OUT and the blocked post-exit period pay for this observer. All
 * candidate scans share the existing one-frame raw-image cache. The original
 * opening, arc/history and opposite-edge geometry remains authoritative. */
static void ring_chain_observe(uint8 *image)
{
    uint8 to_left, valid, row, col, opposite_valid, accepted;
    int16 a = 0, b = 0, n = 0;
    int8 saved_fail;
    if((RING_STATE_NONE != ring_state) && (RING_STATE_OUT != ring_state))
    {
        if(ring_chain.seen) { ring_chain_reset(); }
        return;
    }
    if(cross_flag || cross_bridge_used || (CROSS_STATE_IN == cross_state) ||
       (CROSS_STATE_INSIDE == cross_state))
    {
        ring_chain_reset();
        return;
    }
    if((RING_STATE_OUT == ring_state) && ring_exit_seen &&
       (ring_exit_max_row >= RING_CHAIN_PASS_ROW))
    {
        ring_chain_arm(ring_type, ring_exit_max_row);
    }
    if(!ring_chain.frames) { return; }
    if((RING_STATE_NONE == ring_state) && !ring_guide_blocked)
    {
        ring_chain_reset();
        return;
    }
    ring_chain.frames--;
    if(!ring_chain.frames)
    {
        ring_chain.confirm = 0;
        return;
    }
    to_left = (uint8)(RING_TYPE_RIGHT == ring_chain.source);
    ring_p_left_valid = ring_find_p(image, 1, &ring_p_left_row, &ring_p_left_col);
    ring_p_right_valid = ring_find_p(image, 0, &ring_p_right_row, &ring_p_right_col);
    valid = to_left ? ring_p_left_valid : ring_p_right_valid;
    opposite_valid = to_left ? ring_p_right_valid : ring_p_left_valid;
    row = to_left ? ring_p_left_row : ring_p_right_row;
    col = to_left ? ring_p_left_col : ring_p_right_col;
    if(to_left)
    {
        /* Preserve the first P even before the six-row lost-run threshold.
         * A later small left opening needs this peak-to-current displacement. */
        if(valid && !opposite_valid &&
           ((col > ring_chain.peak_col) || (ring_chain.peak_age > RING_SMALL_LEFT_AGE)))
        {
            ring_chain.peak_col = col;
            ring_chain.peak_age = 0;
        }
        else if(ring_chain.peak_age <= RING_SMALL_LEFT_AGE) { ring_chain.peak_age++; }
        if(valid && opposite_valid) { ring_chain.peak_col = 0; }
        ring_left_p_peak_col = ring_chain.peak_col;
        ring_left_p_peak_age = ring_chain.peak_age;
    }
    else { ring_right_opening_update(); }
    accepted = 0;
    saved_fail = ring_fail;
    if(valid && !opposite_valid && (row <= RING_CHAIN_P_ROW_MAX) &&
       (row + RING_EXIT_RESTART_DROP <= ring_chain.pass_row))
    {
        /* A long first opening is deliberately not accelerated. The original
         * bottom-corner exception remains available after normal recovery. */
        if(ring_find_lost_run(to_left ? 0x01 : 0x02, &a, &b, &n) &&
           (n < RING_FIRST_OPEN_RUN_MIN))
        {
            accepted = ring_try_side_check(to_left, 0, 0);
        }
    }
    ring_fail = saved_fail;
    if(!accepted) { ring_chain.confirm = 0; return; }
    if(((int16)row - (int16)ring_chain.row > VISION_PX(12)) ||
       ((int16)ring_chain.row - (int16)row > VISION_PX(12)) ||
       ((int16)col - (int16)ring_chain.col > VISION_PX(16)) ||
       ((int16)ring_chain.col - (int16)col > VISION_PX(16))) { ring_chain.confirm = 0; }
    ring_chain.row = row;
    ring_chain.col = col;
    if(ring_chain.confirm < RING_CONFIRM_FRAME) { ring_chain.confirm++; }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环识别状态机
// 参数说明     *image              二值化图像
// 返回参数     void
// 备注信息     对应文章的状态推进：
//              状态一(识别到入口) -> 状态二(进环中) -> 状态三(环内) -> 出环
//              出环判据用文章说的"丢线数下降 + 赛道宽度恢复"
//-------------------------------------------------------------------------------------------------------------------
/* A P miss in a wide opening is not recovery. Re-arm only from fresh raw
 * physical edges, before cross/temporal/guidance changes the arrays. */
/* Keep a passed exit's geometry through a short white gap. This is a
 * virtual continuation, never evidence of a newly observed exit corner. */
static void ring_prepare_exit_bridge(uint8 *image)
{
    int16 y, x, other, inside, sample, end;
    uint8 mask = (RING_TYPE_LEFT == ring_type) ? 0x02 : 0x01;
    uint8 other_mask = (RING_TYPE_LEFT == ring_type) ? 0x01 : 0x02;
    if((RING_STATE_OUT != ring_state) || !ring_exit_seen || ring_exit_active ||
       ring_exit_tail_active || (ring_exit_max_row < RING_EXIT_PASS_ROW) ||
       (ring_exit_row < 1) || (ring_exit_missing <= RING_EXIT_HOLD_FRAMES) ||
       (ring_exit_missing > RING_EXIT_RECOVERY_FRAMES)) { return; }
    for(y = 0; y < VISION_H; y++)
    {
        x = (RING_TYPE_LEFT == ring_type) ? track_right[y] : track_left[y];
        if((0 == (track_lost[y] & mask)) && (x > 1) && (x < VISION_W - 2)) { continue; }
        if(RING_TYPE_LEFT == ring_type)
        {
            x = ((int32)ring_exit_col * y + ring_exit_row / 2) / ring_exit_row;
            other = (track_lost[y] & other_mask) ? 0 : track_left[y];
            inside = x - 1;
            if(x - other < VISION_PX(12)) { continue; }
        }
        else
        {
            x = VISION_W - 1 -
                ((int32)(VISION_W - 1 - ring_exit_col) * y + ring_exit_row / 2) / ring_exit_row;
            other = (track_lost[y] & other_mask) ? VISION_W - 1 : track_right[y];
            inside = x + 1;
            if(other - x < VISION_PX(12)) { continue; }
        }
        if((x < 2) || (x >= VISION_W - 2) || (other >= VISION_W) ||
           !RING_IS_WHITE(RING_PIXEL(image, y, inside)) ||
           !RING_IS_WHITE(RING_PIXEL(image, y, (x + other) / 2))) { continue; }
        end = (x > other) ? x : other;
        for(sample = ((x < other) ? x : other) + 1; sample < end; sample++)
        {
            if(!RING_IS_WHITE(RING_PIXEL(image, y, sample))) { break; }
        }
        if(sample < end) { continue; }
        ring_exit_bridge[y] = (uint8)x;
    }
}
static uint8 ring_rearm_lane_clear(uint8 *image)
{
    int16 y, left, right, run = 0;
    uint8 *pixels;
    if(cross_flag || cross_bridge_used || (CROSS_STATE_IN == cross_state) ||
       (CROSS_STATE_INSIDE == cross_state)) { return 0; }
    /* The arithmetic midpoint is dimensionless; never scale its divisor. */
    for(y = VISION_H / 2; y < VISION_H * 4 / 5; y++)
    {
        left = track_left[y];
        right = track_right[y];
        pixels = image + (uint32)y * VISION_W;
        if((0 == (track_lost[y] & 0x03)) && (left >= 2) &&
           (right < VISION_W - 2) && (right - left >= VISION_PX(20)) &&
           (pixels[left - 2] < IMG_WHITE_LEVEL) &&
           (pixels[right + VISION_PX(2)] < IMG_WHITE_LEVEL) &&
           (pixels[left + VISION_PX(2)] >= IMG_WHITE_LEVEL) &&
           (pixels[right - VISION_PX(2)] >= IMG_WHITE_LEVEL) &&
           (pixels[(left + right) / 2] >= IMG_WHITE_LEVEL))
        {
            if(++run >= VISION_PX(12)) { return 1; }
        }
        else { run = 0; }
    }
    return 0;
}

static void ring_state_machine (uint8 *image)
{
    uint16 ring_lost;
    uint16 oppo_lost;
    uint8  found;
    int16  a = 0;
    int16  b = 0;
    int16  n = 0;

    ring_lost = (RING_TYPE_LEFT == ring_type) ? track_left_lost_count : track_right_lost_count;
    oppo_lost = (RING_TYPE_LEFT == ring_type) ? track_right_lost_count : track_left_lost_count;

    switch(ring_state)
    {
        case RING_STATE_NONE:                                       // 不在圆环里：等识别条件
        {
            uint8 left_col = 0, right_col = 0, left_row = 0, right_row = 0, left_valid, right_valid;
            uint8 island_type = RING_TYPE_NONE;
            uint8 opposite_ready = 0;
            uint8 island_ready = 0;
            if(0 != ring_guide_blocked)
            {
                if(!ring_chain.frames)
                {
                    ring_first_open_reset();
                    ring_left_p_peak_col = 0;
                    ring_left_p_peak_age = 0;
                }
                left_valid = ring_find_p(image, 1, &left_row, &left_col);
                right_valid = ring_find_p(image, 0, &right_row, &right_col);
                /* A unique opposite-side P far ahead of the passed exit is
                 * a new element, not the old same-side opening. Let the
                 * original entry geometry/history/confirmation arbitrate it. */
                if(!cross_flag && !cross_bridge_used &&
                   (CROSS_STATE_IN != cross_state) && (CROSS_STATE_INSIDE != cross_state) &&
                   (ring_post_exit_max_row >= RING_EXIT_PASS_ROW))
                {
                    opposite_ready = (uint8)(
                        ((RING_TYPE_LEFT == ring_post_exit_type) && !left_valid && right_valid &&
                         (right_row + RING_EXIT_RESTART_DROP <= ring_post_exit_max_row)) ||
                        ((RING_TYPE_RIGHT == ring_post_exit_type) && !right_valid && left_valid &&
                         (left_row + RING_EXIT_RESTART_DROP <= ring_post_exit_max_row)));
                    if(opposite_ready && ring_chain.frames)
                    {
                        ring_find_lost_run((RING_TYPE_RIGHT == ring_post_exit_type) ?
                                           0x01 : 0x02, &a, &b, &n);
                        if((n < RING_P_OPEN_LOST_MIN) || (n >= RING_FIRST_OPEN_RUN_MIN))
                        {
                            opposite_ready = 0;
                        }
                    }
                }
                if(right_valid && !left_valid)
                {
                    island_type = RING_TYPE_RIGHT;
                    island_ready = ring_cross_right_rearm_ready(right_valid, right_row, right_col, left_valid);
                }
                else
                {
                    island_type = RING_TYPE_LEFT;
                    island_ready = ring_cross_rearm_ready(left_valid, left_row, left_col, right_valid);
                }
                if((!left_valid && !right_valid && ring_rearm_lane_clear(image)) || opposite_ready || island_ready)
                {
                    if(ring_clear_frames < RING_P_CLEAR_FRAMES) { ring_clear_frames++; }
                    if((ring_clear_frames >= RING_P_CLEAR_FRAMES) || opposite_ready || island_ready)
                    {
                        ring_guide_blocked = 0;
                        ring_post_exit_type = RING_TYPE_NONE;
                        ring_post_exit_age = 0;
                        ring_exit_tail_active = 0;
                        /* Two read-only geometry observations can seed the
                         * existing confirmation, never bypass its final check. */
                        if(opposite_ready && (ring_chain.confirm >= RING_CONFIRM_FRAME))
                        {
                            ring_confirm = RING_CONFIRM_FRAME - 1;
                            ring_confirm_type = (RING_TYPE_RIGHT == ring_chain.source) ?
                                                RING_TYPE_LEFT : RING_TYPE_RIGHT;
                            ring_confirm_row = ring_chain.row;
                            ring_confirm_col = ring_chain.col;
                        }
                        if(island_ready)
                        {
                            ring_confirm = RING_CONFIRM_FRAME - 1;
                            ring_confirm_type = island_type;
                            ring_confirm_row = ring_cross_confirm_row;
                            ring_confirm_col = ring_cross_confirm_col;
                        }
                        ring_block_after_cross = 0;
                        ring_cross_confirm = 0;
                        ring_chain_reset();
                    }
                }
                else
                {
                    ring_clear_frames = 0;
                }
                if(!opposite_ready && !island_ready)
                {
                    ring_type = RING_TYPE_NONE;
                    ring_flag = 0;
                    ring_fail = -5;
                    break;
                }
            }
            /* These rows also feed control-side entry confirmation. Keep them
             * fresh even when the cold-frame P search is gated off. */
            ring_scan_feature_rows(image);
#if RING_ENTRY_GATE_ENABLE
            if(!opposite_ready && !island_ready && (0 == ring_entry_scan_due(image)))
            {
                ring_type = RING_TYPE_NONE;
                ring_p_left_valid = 0;
                ring_p_right_valid = 0;
                ring_cross_like = 0;
                ring_corner_row = 0;
                ring_corner_col = 0;
                ring_fail = 0;  /* Not searched, not a failed observation. */
                break;         /* Continue process-level updates and guidance. */
            }
#endif

            found     = 0;
            ring_type = RING_TYPE_NONE;
            ring_fail = 0;
            ring_p_right_valid = ring_find_p(image, 0, &ring_p_right_row, &ring_p_right_col);
            ring_p_left_valid = ring_find_p(image, 1, &ring_p_left_row, &ring_p_left_col);
            if((0 != ring_p_left_valid) &&
               ((ring_p_left_col > ring_left_p_peak_col) ||
                (ring_left_p_peak_age > RING_SMALL_LEFT_AGE)))
            {
                ring_left_p_peak_col = ring_p_left_col;
                ring_left_p_peak_age = 0;
            }
            else if(ring_left_p_peak_age <= RING_SMALL_LEFT_AGE)
            {
                ring_left_p_peak_age++;
            }
            if((0 != ring_p_left_valid) &&
               ((0 == ring_p_right_valid) ||
                (track_left_lost_count >= track_right_lost_count)))
            {
                ring_diag_candidate_side = RING_TYPE_LEFT;
                ring_diag_candidate_row = ring_p_left_row;
                ring_diag_candidate_col = ring_p_left_col;
                ring_find_lost_run(0x01, &a, &b, &n);
            }
            else if(0 != ring_p_right_valid)
            {
                ring_diag_candidate_side = RING_TYPE_RIGHT;
                ring_diag_candidate_row = ring_p_right_row;
                ring_diag_candidate_col = ring_p_right_col;
                ring_find_lost_run(0x02, &a, &b, &n);
            }
            ring_diag_candidate_run = (uint8)n;
            ring_cross_like = (uint8)((0 != ring_p_right_valid) && (0 != ring_p_left_valid));
            if(0 != ring_cross_like)
            {
                ring_first_open_reset();
                ring_left_p_peak_col = 0;
                ring_left_p_peak_age = 0;
                ring_confirm = 0;
                ring_confirm_type = RING_TYPE_NONE;
                ring_corner_row = 0;
                ring_corner_col = 0;
                ring_fail = -9;
                break;
            }
            ring_right_opening_update();

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
                ring_fail = 0;
                if((ring_confirm_type != ring_type) ||
                   ((int16)ring_corner_row - (int16)ring_confirm_row > VISION_PX(12)) ||
                   ((int16)ring_confirm_row - (int16)ring_corner_row > VISION_PX(12)) ||
                   ((int16)ring_corner_col - (int16)ring_confirm_col > VISION_PX(16)) ||
                   ((int16)ring_confirm_col - (int16)ring_corner_col > VISION_PX(16)))
                {
                    ring_confirm = 0;
                }
                ring_confirm_type = ring_type;
                ring_confirm_row = ring_corner_row;
                ring_confirm_col = ring_corner_col;
                ring_confirm++;
                if((ring_confirm >= RING_CONFIRM_FRAME) ||
                   ((RING_TYPE_LEFT == ring_type) &&
                    (RING_TYPE_LEFT == ring_diag_candidate_side) &&
                    (ring_corner_row >= VISION_PX(32)) && (ring_corner_row <= VISION_PX(60)) &&
                    (ring_corner_col >= VISION_W / 4) &&
                    (ring_corner_col <= VISION_W / 2) &&
                    (n >= VISION_PX(56)) && (b >= VISION_PX(96)) &&
                    (0 == (track_lost[VISION_LAST_ROW] & 0x01)) &&
                    (track_left[VISION_LAST_ROW] < VISION_W / 5)))
                {
                    ring_flag         = 1;
                    ring_state        = RING_STATE_FIND;
                    ring_state_count  = 0;
                    ring_confirm      = 0;
                    ring_exit_seen    = 0;
                    ring_exit_missing = 0;
                    ring_exit_max_row = 0;
                    ring_first_open_reset();
                    ring_left_p_peak_col = 0;
                    ring_left_p_peak_age = 0;
                    ring_diag_reason = RING_DIAG_FIND;
                }
            }
            else
            {
                ring_confirm  = 0;
                ring_confirm_type = RING_TYPE_NONE;
                ring_corner_row = 0;
                ring_corner_col = 0;
            }
            break;
        }

        case RING_STATE_FIND:                                       // 识别到入口：等入口一侧大量丢线(张开)
        {
            ring_flag = 1;
            if(RING_TYPE_RIGHT == ring_type)
            {
                // A missed P is not proof that the exit corner has appeared.
                if(ring_p_missing > RING_P_HOLD_FRAMES)
                {
                    ring_find_fail++;
                    if(ring_find_fail >= RING_FIND_FAIL_MAX)
                    {
                        ring_road_cancel();
                    }
                }
                else
                {
                    ring_find_fail = 0;
                }
                break;
            }
            if(ring_lost >= RING_IN_LOST_MIN)
            {
                ring_state       = RING_STATE_IN;
                ring_state_count = 0;
                ring_find_fail   = 0;
                ring_diag_reason = RING_DIAG_IN;
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
                    ring_diag_reason = RING_DIAG_FIND_TIMEOUT;
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
                ring_diag_reason = RING_DIAG_INSIDE;
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
                ring_out_clear_frames = 0;
                ring_diag_reason = RING_DIAG_OUT_LOST;
            }
            break;
        }

        case RING_STATE_OUT:                                        // 出环：对面也恢复直道就结束
        {
            ring_flag = 1;

            if(0 != ring_exit_active)
            {
                ring_out_clear_frames = 0;
            }
            else if((ring_lost <= RING_OUT_LOST_MAX) && (oppo_lost <= RING_OUT_LOST_MAX))
            {
                if(ring_out_clear_frames < RING_OUT_CLEAR_FRAMES) { ring_out_clear_frames++; }
            }
            else
            {
                ring_out_clear_frames = 0;
            }
            if((ring_out_clear_frames >= RING_OUT_CLEAR_FRAMES) ||
               (ring_state_count >= RING_OUT_MAX_FRAMES))
            {
                ring_diag_reason = (ring_out_clear_frames >= RING_OUT_CLEAR_FRAMES) ?
                                   RING_DIAG_DONE_CLEAR : RING_DIAG_DONE_TIMEOUT;
                if(ring_out_clear_frames >= RING_OUT_CLEAR_FRAMES)
                {
                    ring_post_exit_type = ring_type;
                    ring_post_exit_max_row = (ring_exit_max_row >= RING_EXIT_PASS_ROW) ?
                                             ring_exit_max_row : RING_EXIT_PASS_ROW;
                    ring_post_exit_age = 0;
                    ring_chain_arm(ring_type, ring_post_exit_max_row);
                }
                else { ring_chain_reset(); }
                ring_flag        = 0;
                ring_type        = RING_TYPE_NONE;
                ring_state       = RING_STATE_NONE;
                ring_state_count = 0;
                ring_out_clear_frames = 0;
                ring_exit_seen = 0;
                ring_exit_missing = 0;
                ring_exit_max_row = 0;
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
            ring_diag_reason = RING_DIAG_TIMEOUT;
            ring_flag        = 0;
            ring_type        = RING_TYPE_NONE;
            ring_state       = RING_STATE_NONE;
            ring_state_count = 0;
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// The virtual lane is used only while the confirmed entrance corner is visible.
// The border on the opening side is a guide, not a detected physical edge.
//-------------------------------------------------------------------------------------------------------------------
#define RING_GUIDE_MIN_WIDTH     (VISION_PX(12))
uint8 ring_guide_row_valid (uint8 row)
{
    uint8 tail_type = (RING_STATE_OUT == ring_state) ? ring_type : ring_post_exit_type;
    uint8 tail_start = (0 != ring_exit_tail_unpaired) ?
            (uint8)(ring_exit_tail_row - VISION_PX(18)) : (uint8)(ring_exit_tail_row + 1);
#if(0 == RING_FIX_ENABLE)
    return 0;
#endif
    if((row >= VISION_H) || !ring_guide_image ||
       (track_center[row] >= VISION_W) ||
       !RING_IS_WHITE(RING_PIXEL(ring_guide_image, row, track_center[row]))) { return 0; }
    if((0 != ring_exit_active) && (row <= ring_exit_row)) { return 1; }
    if((5 == ring_diag_guide_applied) && (row < VISION_H) &&
       (ring_exit_bridge[row] < VISION_W) &&
       (track_left[row] < VISION_W) && (track_right[row] < VISION_W) &&
       ((int16)track_right[row] - track_left[row] >= RING_GUIDE_MIN_WIDTH))
    { return 1; }
    if((0 != ring_exit_tail_active) &&
       ((RING_STATE_OUT == ring_state) ||
        ((RING_STATE_NONE == ring_state) && (RING_TYPE_NONE != ring_post_exit_type))) &&
       (row >= ring_exit_tail_upper_start) && (row < ring_exit_tail_upper_end) &&
       (0 == cross_state) && (0 == cross_bridge_used) &&
       (track_left[row] < VISION_W) && (track_right[row] < VISION_W) &&
       ((int16)track_right[row] - (int16)track_left[row] >= RING_GUIDE_MIN_WIDTH))
    {
        return 1;
    }
    if((0 != ring_exit_tail_active) &&
       ((RING_STATE_OUT == ring_state) ||
        ((RING_STATE_NONE == ring_state) && (RING_TYPE_NONE != ring_post_exit_type))) &&
       (row >= tail_start) && (row < VISION_VALID_H) &&
       (0 == cross_state) && (0 == cross_bridge_used) &&
       ((0 != ring_exit_tail_unpaired) ||
        (0 == (track_lost[row] & ((RING_TYPE_LEFT == tail_type) ? 0x02 : 0x01)))) &&
       ((int16)track_right[row] - (int16)track_left[row] >= VISION_PX(12))) { return 1; }
    return (uint8)((0 != ring_guide_active) &&
                   ((row >= ring_corner_row) ||
                    (0 != ring_upper_curve_active)) &&
                   (row < VISION_VALID_H) &&
                   ((RING_STATE_FIND == ring_state) || (RING_STATE_IN == ring_state) ||
                   (RING_STATE_INSIDE == ring_state)));
}

#define RING_LEFT_EDGE_GAP_MAX   (VISION_PX(24))
#define RING_LEFT_EDGE_HOLD_MAX  (VISION_PX(6))

static uint8 ring_right_edge_usable (int16 row, int16 guide_left)
{
    return (uint8)((row >= ring_corner_row) && (row < VISION_VALID_H) &&
                   (ring_right_physical[row] < VISION_W) &&
                   ((int16)ring_right_physical[row] > guide_left + RING_GUIDE_MIN_WIDTH));
}

static uint8 ring_left_edge_usable (int16 row, int16 span)
{
    int16 guide_right;

    if((row < ring_corner_row) || (row >= VISION_VALID_H) ||
       (0 != (track_lost[row] & 0x01)))
    {
        return 0;
    }
    guide_right = (int16)(ring_corner_col +
            ((int32)(VISION_W - 1 - ring_corner_col) * (row - ring_corner_row) + span / 2) / span);
    return (uint8)((track_left[row] < VISION_W) &&
                   ((int16)track_left[row] + RING_GUIDE_MIN_WIDTH < guide_right));
}

static int16 ring_bottom_left_anchor (void)
{
    int16 y;
    int16 x;
    int16 bottom;

    for(y = VISION_H - 2; y >= VISION_H - VISION_PX(12); y--)
    {
        if((0 != (track_lost[y] & 0x01)) || (0 != (track_lost[y - 3] & 0x01)))
        {
            continue;
        }
        if(((int16)track_left[y] - (int16)track_left[y - VISION_PX(3)] > VISION_PX(6)) ||
           ((int16)track_left[y - VISION_PX(3)] - (int16)track_left[y] > VISION_PX(6)))
        {
            continue;
        }
        x = (int16)track_left[y] +
            ((int16)track_left[y] - (int16)track_left[y - 3]) *
            (VISION_LAST_ROW - y) / 3;
        if(0 == (track_lost[VISION_LAST_ROW] & 0x01))
        {
            bottom = track_left[VISION_LAST_ROW];
            if(((bottom - x) <= VISION_PX(6)) && ((x - bottom) <= VISION_PX(6))) { x = bottom; }
        }
        if(x < 0) { x = 0; }
        if(x >= VISION_W) { x = VISION_W - 1; }
        return x;
    }
    return 0;
}

static uint8 ring_apply_exit_tail (void)
{
    int16 y;
    int16 x;
    int16 physical;
    int16 start;
    int16 first_applied = VISION_H;
    int16 sample_end;
    int16 other_x;
    int16 step;
    int16 shift;
    uint8 applied = 0;
    uint8 tail_type = (RING_STATE_OUT == ring_state) ? ring_type : ring_post_exit_type;
    uint8 mask = (RING_TYPE_LEFT == tail_type) ? 0x01 : 0x02;
    uint8 other_mask = (RING_TYPE_LEFT == tail_type) ? 0x02 : 0x01;
    uint8 *other_edge = (RING_TYPE_LEFT == tail_type) ? track_right : track_left;

    if((0 == ring_exit_tail_active) || (RING_TYPE_NONE == tail_type)) { return 0; }
    start = (0 != ring_exit_tail_unpaired) ?
            (int16)ring_exit_tail_row - VISION_PX(18) : (int16)ring_exit_tail_row + 1;
    for(y = start; y < VISION_VALID_H; y++)
    {
        if((0 == ring_exit_tail_unpaired) &&
           (0 != (track_lost[y] & other_mask))) { continue; }
        x = (int16)ring_exit_tail_col +
            ring_exit_tail_delta * (y - ring_exit_tail_row) / VISION_PX(18);
        if((x < 3) || (x >= VISION_W - 3)) { continue; }
        if(RING_TYPE_LEFT == tail_type)
        {
            if((int16)track_right[y] < x + RING_GUIDE_MIN_WIDTH)
            {
                if(0 == ring_exit_tail_unpaired) { continue; }
                track_right[y] = VISION_W - 1;
                track_lost[y] |= other_mask;
            }
            physical = track_left[y];
            if((0 == (track_lost[y] & mask)) &&
               (physical - x <= VISION_PX(5)) && (x - physical <= VISION_PX(5))) { x = physical; }
            track_left[y] = (uint8)x;
        }
        else
        {
            if((int16)track_left[y] + RING_GUIDE_MIN_WIDTH > x)
            {
                if(0 == ring_exit_tail_unpaired) { continue; }
                track_left[y] = 0;
                track_lost[y] |= other_mask;
            }
            physical = track_right[y];
            if((0 == (track_lost[y] & mask)) &&
               (physical - x <= VISION_PX(5)) && (x - physical <= VISION_PX(5))) { x = physical; }
            track_right[y] = (uint8)x;
        }
        track_center[y] = (uint8)(((int16)track_left[y] + (int16)track_right[y]) / 2);
        if(0 == applied) { first_applied = y; }
        applied = 1;
    }
    if((0 != applied) && (0 == ring_exit_active) &&
       (0 == ring_exit_tail_unpaired) &&
       (first_applied + VISION_PX(4) < VISION_VALID_H) &&
       (other_edge[first_applied] > 3) &&
       (other_edge[first_applied] < VISION_W - 3))
    {
        sample_end = first_applied;
        for(y = first_applied + 1;
            (y <= first_applied + VISION_PX(18)) && (y < VISION_VALID_H); y++)
        {
            step = (int16)other_edge[y] - (int16)other_edge[y - 1];
            if((0 != (track_lost[y] & other_mask)) ||
               (other_edge[y] <= 3) || (other_edge[y] >= VISION_W - 3) ||
               ((RING_TYPE_LEFT == tail_type) ? ((step < 0) || (step > 3)) :
                                               ((step > 0) || (step < -3))))
            {
                break;
            }
            sample_end = y;
        }
        shift = (RING_TYPE_LEFT == tail_type) ?
                ((int16)other_edge[sample_end] - (int16)other_edge[first_applied]) :
                ((int16)other_edge[first_applied] - (int16)other_edge[sample_end]);
        if((sample_end - first_applied >= VISION_PX(4)) && (shift >= 2) &&
           (shift <= 2 * (sample_end - first_applied)))
        {
            x = (int16)ring_exit_tail_col -
                ring_exit_tail_delta * ring_exit_tail_row / VISION_PX(18);
            other_x = (int16)other_edge[first_applied] -
                      ((int16)other_edge[sample_end] -
                       (int16)other_edge[first_applied]) *
                      first_applied / (sample_end - first_applied);
            if((x >= 3) && (x < VISION_W - 3) &&
               (other_x >= 3) && (other_x < VISION_W - 3) &&
               ((RING_TYPE_LEFT == tail_type) ?
                (other_x - x >= RING_GUIDE_MIN_WIDTH) :
                (x - other_x >= RING_GUIDE_MIN_WIDTH)))
            {
                // Keep the extension continuous with the lower straight pair.
                for(y = first_applied - 1; y >= 0; y--)
                {
                    x = (int16)ring_exit_tail_col +
                        ring_exit_tail_delta * (y - ring_exit_tail_row) / VISION_PX(18);
                    other_x = (int16)other_edge[first_applied] +
                              ((int16)other_edge[sample_end] -
                               (int16)other_edge[first_applied]) *
                              (y - first_applied) / (sample_end - first_applied);
                    if((x < 3) || (x >= VISION_W - 3) ||
                       (other_x < 3) || (other_x >= VISION_W - 3) ||
                       ((RING_TYPE_LEFT == tail_type) ?
                        (other_x - x < RING_GUIDE_MIN_WIDTH) :
                        (x - other_x < RING_GUIDE_MIN_WIDTH)))
                    {
                        break;
                    }
                    if(RING_TYPE_LEFT == tail_type)
                    {
                        track_left[y] = (uint8)x;
                        track_right[y] = (uint8)other_x;
                    }
                    else
                    {
                        track_left[y] = (uint8)other_x;
                        track_right[y] = (uint8)x;
                    }
                    track_center[y] = (uint8)((x + other_x) / 2);
                    ring_exit_tail_upper_start = (uint8)y;
                }
                ring_exit_tail_upper_end = (uint8)first_applied;
            }
        }
    }
    if(0 != applied)
    {
        for(y = 0; y < VISION_VALID_H; y++)
        {
            x = (int16)ring_exit_tail_col +
                ring_exit_tail_delta * (y - ring_exit_tail_row) / VISION_PX(18);
            if((RING_TYPE_LEFT == tail_type) && (track_left[y] <= 1) &&
               ((0 != (track_lost[y] & mask)) ||
                (x - (int16)track_left[y] >= VISION_PX(20))))
            {
                track_left[y] = 255;
                track_center[y] = 255;
            }
            else if((RING_TYPE_RIGHT == tail_type) &&
                    (track_right[y] >= VISION_W - 2) &&
                    ((0 != (track_lost[y] & mask)) ||
                     ((int16)track_right[y] - x >= VISION_PX(20))))
            {
                track_right[y] = 255;
                track_center[y] = 255;
            }
        }
    }
    return applied;
}

/* Apply only raw-verified missing rows, after smoothing/pinning/white guard. */
static uint8 ring_apply_exit_bridge(void)
{
    int16 y, left, right;
    uint8 applied = 0;
    uint8 mask = (RING_TYPE_LEFT == ring_type) ? 0x02 : 0x01;
    if((RING_STATE_OUT != ring_state) || cross_state || cross_bridge_used) { return 0; }
    for(y = 0; y < VISION_H; y++)
    {
        if(ring_exit_bridge[y] >= VISION_W) { continue; }
        left = (RING_TYPE_LEFT == ring_type) ? track_right[y] : track_left[y];
        if(!(track_lost[y] & mask) && (left > 1) && (left < VISION_W - 2)) { continue; }
        if(RING_TYPE_LEFT == ring_type)
        {
            right = ring_exit_bridge[y];
            left = (track_lost[y] & 0x01) ? 0 : track_left[y];
        }
        else
        {
            left = ring_exit_bridge[y];
            right = (track_lost[y] & 0x02) ? VISION_W - 1 : track_right[y];
        }
        if((left >= VISION_W) || (right >= VISION_W) ||
           (right - left < RING_GUIDE_MIN_WIDTH)) { continue; }
        track_left[y] = (uint8)left;
        track_right[y] = (uint8)right;
        track_center[y] = (uint8)((left + right) / 2);
        applied = 1;
    }
    return applied;
}
static void ring_apply_guidance_frame (void)
{
    int16 y;
    int16 left;
    int16 right;
    int16 span;
    int16 scan;
    int16 last_good_row = -1;
    int16 next_good_row = -1;
    int16 last_good_x = 0;
    int16 next_good_x = 0;
    int16 bottom_left = 0;

    ring_diag_guide_applied = 0;
    ring_exit_tail_upper_start = VISION_H;
    ring_exit_tail_upper_end = 0;

#if(0 == RING_FIX_ENABLE)
    return;
#endif
    if((0 != ring_exit_active) && (0 == cross_state) && (0 == cross_bridge_used))
    {
        if(ring_exit_row < 1) { return; }
        ring_diag_guide_applied = 2;
        for(y = 0; y <= ring_exit_row; y++)
        {
            if(RING_TYPE_RIGHT == ring_type)
            {
                left = ring_exit_col +
                       ((int32)(VISION_W - 1 - ring_exit_col) *
                        (ring_exit_row - y) + ring_exit_row / 2) / ring_exit_row;
                right = ((0 == (track_lost[y] & 0x02)) &&
                         ((int16)track_right[y] > left + RING_GUIDE_MIN_WIDTH)) ?
                        track_right[y] : VISION_W - 1;
                if(right < left) { right = left; }
            }
            else if(RING_TYPE_LEFT == ring_type)
            {
                right = ((int32)ring_exit_col * y + ring_exit_row / 2) / ring_exit_row;
                left = ((0 == (track_lost[y] & 0x01)) &&
                        ((int16)track_left[y] + RING_GUIDE_MIN_WIDTH < right)) ?
                       track_left[y] : 0;
                if(left > right) { left = right; }
            }
            else
            {
                return;
            }
            track_left[y] = (uint8)left;
            track_right[y] = (uint8)right;
            track_center[y] = (uint8)((left + right) / 2);
        }
        if(0 != ring_apply_exit_tail()) { ring_diag_guide_applied = 4; }
        return;
    }
    if(((RING_STATE_OUT == ring_state) ||
        ((RING_STATE_NONE == ring_state) && (RING_TYPE_NONE != ring_post_exit_type))) &&
       (0 != ring_exit_tail_active) &&
       (0 == cross_state) && (0 == cross_bridge_used))
    {
        if(0 != ring_apply_exit_tail()) { ring_diag_guide_applied = 3; }
        return;
    }
    if(0 != ring_apply_exit_bridge())
    {
        ring_diag_guide_applied = 5;
        return;
    }
    if((0 == ring_guide_active) || (0 != cross_state) || (0 != cross_bridge_used))
    {
        return;
    }
    span = (int16)(VISION_LAST_ROW - ring_corner_row);
    if(span < 1) { return; }
    ring_diag_guide_applied = 1;
    if(RING_TYPE_RIGHT == ring_type)
    {
        bottom_left = ring_bottom_left_anchor();
        if(bottom_left + RING_GUIDE_MIN_WIDTH >= ring_corner_col) { bottom_left = 0; }
    }
    if(0 != ring_upper_curve_active)
    {
        for(y = 0; y < ring_corner_row; y++)
        {
            left = (RING_TYPE_RIGHT == ring_type) ? ring_upper_curve[y] : 0;
            right = (RING_TYPE_LEFT == ring_type) ? ring_upper_curve[y] : VISION_W - 1;
            track_left[y] = (uint8)left;
            track_right[y] = (uint8)right;
            track_center[y] = (uint8)((left + right) / 2);
        }
    }

    for(y = ring_corner_row; y < VISION_VALID_H; y++)
    {
        if(RING_TYPE_RIGHT == ring_type)
        {
            left = (int16)(bottom_left +
                    ((int32)(ring_corner_col - bottom_left) *
                     (VISION_LAST_ROW - y) + span / 2) / span);
            if(0 != ring_right_edge_usable(y, left))
            {
                right = ring_right_physical[y];
                last_good_row = y;
                last_good_x = right;
            }
            else
            {
                if(next_good_row <= y)
                {
                    next_good_row = -1;
                    for(scan = y + 1;
                        (scan < VISION_VALID_H) && (scan <= y + RING_LEFT_EDGE_GAP_MAX);
                        scan++)
                    {
                        int16 scan_left = (int16)(bottom_left +
                                ((int32)(ring_corner_col - bottom_left) *
                                 (VISION_LAST_ROW - scan) + span / 2) / span);
                        if(0 != ring_right_edge_usable(scan, scan_left))
                        {
                            next_good_row = scan;
                            next_good_x = ring_right_physical[scan];
                            break;
                        }
                    }
                }
                if((last_good_row >= 0) && (next_good_row >= 0) &&
                   (next_good_row - last_good_row <= RING_LEFT_EDGE_GAP_MAX))
                {
                    right = (int16)(last_good_x +
                            ((next_good_x - last_good_x) * (y - last_good_row) +
                             (next_good_row - last_good_row) / 2) /
                            (next_good_row - last_good_row));
                }
                else if((last_good_row < 0) && (next_good_row >= 0) &&
                        (next_good_row - ring_corner_row <= RING_LEFT_EDGE_GAP_MAX + VISION_PX(8)))
                {
                    right = (int16)((VISION_W - 1) +
                            ((next_good_x - (VISION_W - 1)) *
                             (y - ring_corner_row)) / (next_good_row - ring_corner_row));
                }
                else if((last_good_row >= 0) &&
                        (y - last_good_row <= RING_LEFT_EDGE_HOLD_MAX))
                {
                    right = last_good_x;
                }
                else
                {
                    right = VISION_W - 1;
                }
            }
        }
        else if(RING_TYPE_LEFT == ring_type)
        {
            right = (int16)(ring_corner_col +
                    ((int32)(VISION_W - 1 - ring_corner_col) * (y - ring_corner_row) + span / 2) / span);
            if(0 != ring_left_edge_usable(y, span))
            {
                left = track_left[y];
                last_good_row = y;
                last_good_x = left;
            }
            else
            {
                if(next_good_row <= y)
                {
                    next_good_row = -1;
                    for(scan = y + 1;
                        (scan < VISION_VALID_H) && (scan <= y + RING_LEFT_EDGE_GAP_MAX);
                        scan++)
                    {
                        if(0 != ring_left_edge_usable(scan, span))
                        {
                            next_good_row = scan;
                            next_good_x = track_left[scan];
                            break;
                        }
                    }
                }
                if((last_good_row >= 0) && (next_good_row >= 0) &&
                   (next_good_row - last_good_row <= RING_LEFT_EDGE_GAP_MAX))
                {
                    left = (int16)(last_good_x +
                            ((next_good_x - last_good_x) * (y - last_good_row) +
                             (next_good_row - last_good_row) / 2) /
                            (next_good_row - last_good_row));
                }
                else if((last_good_row >= 0) &&
                        (y - last_good_row <= RING_LEFT_EDGE_HOLD_MAX))
                {
                    left = last_good_x;
                }
                else if((next_good_row >= 0) &&
                        (next_good_row - y <= RING_LEFT_EDGE_HOLD_MAX))
                {
                    left = next_good_x;
                }
                else
                {
                    left = 0;
                }
            }
        }
        else
        {
            return;
        }
        track_left[y] = (uint8)left;
        track_right[y] = (uint8)right;
        track_center[y] = (uint8)((left + right) / 2);
    }
}

/* Virtual lines may be held briefly, but black pixels never become a route.
 * Clear the geometry as well as validity so ordinary single-edge inference
 * cannot accidentally reuse a rejected virtual line. */
void ring_road_apply_guidance (void)
{
    int16 y;
    ring_apply_guidance_frame();
    if(!ring_guide_image || !ring_diag_guide_applied) { return; }
    for(y = 0; y < VISION_H; y++)
    {
        if((track_center[y] < VISION_W) &&
           !RING_IS_WHITE(RING_PIXEL(ring_guide_image, y, track_center[y])))
        {
            track_left[y] = 255;
            track_right[y] = 255;
            track_center[y] = 255;
            track_lost[y] |= 0x03;
        }
    }
}
void ring_road_cancel (void)
{
    uint8 keep_cross_confirm = (uint8)((CROSS_STATE_OUT == cross_state) &&
                               !cross_flag && !cross_bridge_used &&
                               (RING_STATE_NONE == ring_state) && ring_block_after_cross);
    ring_cross_right_entry = 0;
    ring_frame_cache_end();
    ring_roi_reset();
    ring_chain_reset();
    ring_block_after_cross = (uint8)(cross_flag || cross_bridge_used ||
                                    (CROSS_STATE_NONE != cross_state));
    /* main.c cancels NONE on every passive cross-OUT frame. Preserve only
     * this fresh two-frame geometry confirmation, never an active ring. */
    memset(ring_exit_bridge, 255, sizeof(ring_exit_bridge));
    if(!keep_cross_confirm) { ring_cross_confirm = 0; }
    if(RING_STATE_NONE != ring_state) { ring_diag_reason = RING_DIAG_CANCEL; }
    ring_diag_p_source = 0;
    ring_first_open_reset();
    ring_left_p_peak_col = 0;
    ring_left_p_peak_age = 0;
    ring_cross_like = 0;
    ring_exit_active = 0;
    ring_exit_row = 0;
    ring_exit_col = 0;
    ring_out_clear_frames = 0;
    ring_exit_seen = 0;
    ring_exit_missing = 0;
    ring_exit_max_row = 0;
    ring_upper_curve_active = 0;
    ring_exit_tail_active = 0;
    ring_exit_tail_unpaired = 0;
    ring_post_exit_type = RING_TYPE_NONE;
    ring_post_exit_max_row = 0;
    ring_post_exit_age = 0;
    if((RING_STATE_NONE != ring_state) || cross_flag || cross_bridge_used ||
       (CROSS_STATE_IN == cross_state) || (CROSS_STATE_INSIDE == cross_state))
    {
        ring_guide_blocked = 1;
        ring_clear_frames = 0;
    }
    ring_guide_active = 0;
    ring_flag = 0;
    ring_type = RING_TYPE_NONE;
    ring_state = RING_STATE_NONE;
    ring_state_count = 0;
    ring_find_fail = 0;
    ring_confirm = 0;
    ring_confirm_type = RING_TYPE_NONE;
    ring_p_missing = 0;
}

static void ring_complete_exit (uint8 restart_corner)
{
    uint8 completed_type = ring_type;
    uint8 completed_max_row = ring_exit_max_row;
    uint8 completed_seen = ring_exit_seen;
    ring_chain_t saved_chain;
    saved_chain = ring_chain;
    /* External cancel clears the window; a verified completion preserves
     * observations of the next opposite-side ring collected during OUT. */
    ring_road_cancel();
    if(saved_chain.seen && (saved_chain.source == completed_type))
    {
        ring_chain = saved_chain;
        ring_left_p_peak_col = saved_chain.peak_col;
        ring_left_p_peak_age = saved_chain.peak_age;
    }
    else if(completed_seen) { ring_chain_arm(completed_type, completed_max_row); }
    if((0 != completed_seen) && (completed_max_row >= VISION_PX(60)))
    {
        ring_post_exit_type = completed_type;
        ring_post_exit_max_row = completed_max_row;
    }
    ring_diag_reason = (0 != restart_corner) ? RING_DIAG_DONE_RESTART : RING_DIAG_DONE_GAP;
    ring_guide_blocked = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环识别
// 参数说明     *binary_image      二值化图像(0/255) 通常传 image_copy[0]
// 返回参数     void
// 使用示例     ring_road_process(image_copy[0]);
// 备注信息     结果放在 ring_flag / ring_type / ring_state 中
//              只识别入口；track_white_lane_guard 之后调用 ring_road_apply_guidance 画虚拟边线
//-------------------------------------------------------------------------------------------------------------------
static void ring_road_process_frame (uint8 *binary_image)
{
    uint8 previous_state = ring_state;
    uint8 row;
    uint8 col;
    uint8 found_p = 0;
    uint8 wedge_p = 0;
    int16 y;

    memset(ring_exit_bridge, 255, sizeof(ring_exit_bridge));
    ring_cross_right_entry = 0;  /* Current-frame geometry only. */
    ring_diag_frame++;
    ring_diag_previous_state = previous_state;
    ring_diag_reason = RING_DIAG_NONE;
    ring_diag_p_source = 0;
    ring_diag_candidate_side = RING_TYPE_NONE;
    ring_diag_candidate_row = 0;
    ring_diag_candidate_col = 0;
    ring_diag_candidate_run = 0;
    ring_diag_exit_probe = 0;
    ring_exit_tail_active = 0;
    ring_exit_tail_unpaired = 0;
    if(RING_TYPE_NONE != ring_post_exit_type)
    {
        if(ring_post_exit_age < RING_POST_EXIT_FRAMES) { ring_post_exit_age++; }
        else { ring_post_exit_type = RING_TYPE_NONE; }
    }

    ring_guide_active = 0;
    ring_exit_active = 0;
    if((((RING_STATE_FIND == previous_state) &&
         (RING_TYPE_RIGHT == ring_type) &&
         (ring_p_missing > RING_EXIT_HOLD_FRAMES)) ||
        (RING_STATE_IN == previous_state) ||
        (RING_STATE_INSIDE == previous_state) ||
       (RING_STATE_OUT == previous_state)) &&
       (0 != ring_find_exit_corner(ring_type, &row, &col)))
    {
        if((RING_STATE_OUT == previous_state) &&
           (ring_exit_max_row >= RING_EXIT_PASS_ROW + VISION_PX(10)) &&
           (row + RING_EXIT_RESTART_DROP <= ring_exit_max_row) &&
           (ring_clear_frames >= RING_EXIT_RESTART_CLEAR) &&
           ((RING_TYPE_LEFT == ring_type) ?
            ((int16)ring_exit_col - (int16)col >= VISION_W / 3) :
            ((int16)col - (int16)ring_exit_col >= VISION_W / 3)))
        {
            ring_complete_exit(1);
            ring_exit_tail_active = ring_find_exit_tail(ring_post_exit_type, binary_image);
            return;
        }
        ring_exit_row = row;
        ring_exit_col = col;
        ring_exit_active = 1;
        ring_exit_seen = 1;
        ring_exit_missing = 0;
        if(row > ring_exit_max_row) { ring_exit_max_row = row; }
    }
    else if((RING_STATE_OUT == previous_state) &&
            (0 != ring_exit_seen))
    {
        if(ring_exit_missing <= RING_EXIT_RECOVERY_FRAMES) { ring_exit_missing++; }
        if((ring_exit_missing >= RING_EXIT_LONG_GAP) &&
           (ring_clear_frames >= RING_P_CLEAR_FRAMES) &&
           ((RING_TYPE_LEFT == ring_type) ?
            ((track_left_lost_count <= VISION_PX(24)) &&
             (track_right_lost_count <= RING_OUT_LOST_MAX)) :
            ((track_right_lost_count <= VISION_PX(24)) &&
             (track_left_lost_count <= RING_OUT_LOST_MAX))) &&
           (((0 != ring_find_p(binary_image, 1, &row, &col)) &&
             (col >= VISION_W / 4) && (row <= VISION_H / 2)) ||
            ((0 != ring_find_p(binary_image, 0, &row, &col)) &&
             (col <= VISION_W * 3 / 4) && (row <= VISION_H / 2))))
        {
            ring_complete_exit(0);
            ring_diag_reason = RING_DIAG_DONE_NEW_P;
            ring_exit_tail_active = ring_find_exit_tail(ring_post_exit_type, binary_image);
            return;
        }
        if((ring_out_clear_frames >= RING_EXIT_RESTART_CLEAR) &&
           (track_left_lost_count <= RING_OUT_LOST_MAX) &&
           (track_right_lost_count <= RING_OUT_LOST_MAX) &&
           (((ring_exit_max_row >= RING_EXIT_PASS_ROW) &&
             (ring_exit_missing >= RING_EXIT_RESET_GAP)) ||
            ((ring_exit_missing >= RING_EXIT_LONG_GAP) &&
             (ring_clear_frames >= RING_P_CLEAR_FRAMES))))
        {
            ring_complete_exit(0);
            ring_exit_tail_active = ring_find_exit_tail(ring_post_exit_type, binary_image);
            return;
        }
        if(ring_exit_missing <= RING_EXIT_HOLD_FRAMES)
        {
            ring_exit_active = 1;
        }
    }
    ring_chain_observe(binary_image);
    if((RING_STATE_OUT == ring_state) && ring_chain.frames &&
       (ring_chain.confirm >= RING_CONFIRM_FRAME) &&
       (ring_exit_max_row >= RING_CHAIN_PASS_ROW) &&
       (!ring_exit_active ||
        (ring_exit_row + RING_EXIT_RESTART_DROP <= ring_exit_max_row)))
    {
        /* A passed old exit plus two complete opposite-side entry geometries
         * ends OUT even if the next opening prevents a P-free recovery gap. */
        ring_complete_exit(0);
        ring_diag_reason = RING_DIAG_DONE_CHAIN;
        ring_frame_cache_begin(binary_image);
    }
    ring_state_machine(binary_image);
    if((RING_STATE_NONE != ring_state) && (RING_STATE_OUT != ring_state))
    {
        ring_post_exit_type = RING_TYPE_NONE;
    }
    if((0 != ring_exit_active) && (RING_TYPE_NONE != ring_type))
    {
        ring_state = RING_STATE_OUT;
        ring_flag = 1;
        if(RING_STATE_OUT != previous_state)
        {
            ring_state_count = 0;
            ring_out_clear_frames = 0;
            ring_diag_reason = RING_DIAG_OUT_CORNER;
        }
    }
    else
    {
        ring_exit_active = 0;
    }
    if(RING_STATE_OUT == ring_state)
    {
        ring_exit_tail_active = ring_find_exit_tail(ring_type, binary_image);
    }
    else if((RING_STATE_NONE == ring_state) && (RING_TYPE_NONE != ring_post_exit_type))
    {
        ring_exit_tail_active = ring_find_exit_tail(ring_post_exit_type, binary_image);
    }
    if(RING_TYPE_RIGHT == ring_type)
    {
        for(y = 0; y < VISION_VALID_H; y++)
        {
            ring_right_physical[y] = (0 == (track_lost[y] & 0x02)) ? track_right[y] : 255;
        }
    }
    if(RING_STATE_OUT == ring_state)
    {
        ring_guide_blocked = 1;
        if(RING_STATE_OUT != previous_state)
        {
            ring_clear_frames = 0;
        }
        else if(((ring_exit_missing > RING_EXIT_HOLD_FRAMES) ||
                 (ring_exit_max_row >= RING_EXIT_PASS_ROW + VISION_PX(15))) &&
                (0 == ring_find_p(binary_image, 0, &row, &col)) &&
                (0 == ring_find_p(binary_image, 1, &row, &col)))
        {
            if(ring_clear_frames < RING_P_CLEAR_FRAMES) { ring_clear_frames++; }
        }
        else
        {
            ring_clear_frames = 0;
        }
    }
    else if((RING_STATE_NONE != previous_state) &&
            (RING_STATE_NONE == ring_state))
    {
        ring_guide_blocked = 1;
        ring_clear_frames = 0;
    }
    if((0 == ring_guide_blocked) &&
       ((RING_STATE_FIND == ring_state) || (RING_STATE_IN == ring_state) ||
        (RING_STATE_INSIDE == ring_state)))
    {
        found_p = ring_find_p_follow(binary_image,
                                     (uint8)(RING_TYPE_LEFT == ring_type), &row, &col);
        if(0 == found_p)
        {
            wedge_p = ring_find_p_wedge_tip(binary_image,
                                            (uint8)(RING_TYPE_LEFT == ring_type),
                                            &row, &col);
            found_p = wedge_p;
        }
    }
    if(0 != found_p)
    {
        ring_diag_p_source = (0 != wedge_p) ? 2 : 1;
        ring_corner_row = row;
        ring_corner_col = col;
        ring_guide_active = 1;
        ring_p_missing = 0;
        if((RING_TYPE_LEFT == ring_type) ||
           ((0 != wedge_p) && (RING_TYPE_RIGHT == ring_type)))
        {
            ring_build_upper_curve(binary_image,
                                   (uint8)(RING_TYPE_LEFT == ring_type));
        }
        else
        {
            ring_upper_curve_active = 0;
        }
    }
    else if((0 == ring_guide_blocked) &&
            ((RING_STATE_FIND == ring_state) || (RING_STATE_IN == ring_state) ||
             (RING_STATE_INSIDE == ring_state)))
    {
        if(ring_p_missing <= RING_P_HOLD_FRAMES) { ring_p_missing++; }
        if(ring_p_missing <= RING_P_HOLD_FRAMES)
        {
            ring_guide_active = 1;
            ring_diag_p_source = 3;
        }
        else
        {
            if(0 == ring_guide_blocked) { ring_diag_reason = RING_DIAG_P_LOST; }
            ring_guide_blocked = 1;
            ring_clear_frames = 0;
        }
    }
    ring_prepare_exit_bridge(binary_image);
}

void ring_road_process (uint8 *binary_image)
{
    ring_guide_image = binary_image;
    ring_frame_cache_begin(binary_image);
#if RING_STATE_ROI_ENABLE
    ring_roi_stage = RING_ROI_STAGE_NONE;
#endif
    ring_road_process_frame(binary_image);
    ring_roi_commit();
    ring_frame_cache_end();
}

void ring_road_get_diag (ring_diag_t *diag)
{
    if(NULL == diag) { return; }
    diag->frame = ring_diag_frame;
    diag->previous_state = ring_diag_previous_state;
    diag->reason = ring_diag_reason;
    diag->p_source = ring_diag_p_source;
    diag->p_row = (0 != ring_diag_p_source) ? ring_corner_row : 0;
    diag->p_col = (0 != ring_diag_p_source) ? ring_corner_col : 0;
    diag->p_missing = ring_p_missing;
    diag->candidate_side = ring_diag_candidate_side;
    diag->candidate_row = ring_diag_candidate_row;
    diag->candidate_col = ring_diag_candidate_col;
    diag->candidate_run = ring_diag_candidate_run;
    diag->guide_active = ring_guide_active;
    diag->guide_blocked = ring_guide_blocked;
    diag->guide_applied = ring_diag_guide_applied;
    diag->exit_probe = ring_diag_exit_probe;
    diag->tail_active = ring_exit_tail_active;
    diag->tail_row = (0 != ring_exit_tail_active) ? ring_exit_tail_row : 0;
    diag->tail_col = (0 != ring_exit_tail_active) ? ring_exit_tail_col : 0;
    diag->exit_missing = ring_exit_missing;
    diag->exit_max_row = ring_exit_max_row;
    diag->out_clear_frames = ring_out_clear_frames;
    diag->clear_frames = ring_clear_frames;
    diag->wait_second = ring_wait_second;
    diag->open_row1 = ring_open_row1;
    diag->open_row2 = ring_open_row2;
    diag->lost_left = (uint8)track_left_lost_count;
    diag->lost_right = (uint8)track_right_lost_count;
    diag->cross_state = cross_state;
    diag->cross_bridge = cross_bridge_used;
}

/* Only this frame's uncommitted FIND -> IN transition may be corrected by
 * a validated two-sided cross. Established IN/INSIDE/OUT keep priority. */
uint8 ring_road_entry_transition (void)
{
    return (uint8)((RING_STATE_IN == ring_state) &&
                   (RING_STATE_FIND == ring_diag_previous_state));
}

uint8 ring_road_preempt_cross (void)
{
    if(0 != ring_exit_active) { return 1; }
    if((RING_TYPE_NONE != ring_type) &&
       ((RING_STATE_IN == ring_state) ||
        (RING_STATE_INSIDE == ring_state) ||
        (RING_STATE_OUT == ring_state))) { return 1; }
    if((0 != ring_wait_second) && (0 == ring_cross_like) &&
       (RING_STATE_NONE == ring_state)) { return 1; }
    if((0 != ring_guide_blocked) || (RING_TYPE_NONE == ring_type) ||
       (0 == ring_corner_row)) { return 0; }
    if(RING_STATE_NONE == ring_state) { return (uint8)(ring_confirm > 0); }
    return (uint8)((0 != ring_guide_active) &&
                   ((RING_STATE_FIND == ring_state) || (RING_STATE_IN == ring_state) ||
                    (RING_STATE_INSIDE == ring_state)));
}
