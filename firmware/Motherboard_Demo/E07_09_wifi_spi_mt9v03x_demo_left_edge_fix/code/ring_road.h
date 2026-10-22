/*********************************************************************************************************************
* 文件名称          ring_road.h
* 公司名称          成都逐飞科技有限公司
* 开发环境          MDK FOR C251
* 适用平台          STC32G144K
*
* 功能简介          圆环（环岛）识别，左环右环都支持
*                  算法参考：智能车图像处理(三)元素判断
*                  https://blog.csdn.net/weixin_50567482/article/details/119406477
*
* 文章要点          圆环判断给了两种方法（都只有文字描述，文章没给圆环的代码）：
*                  方法一(找拐点)：
*                    1.判断对面那条边界是连续直道
*                    2.找圆环入口一侧的下拐点       -> 状态一
*                    3.找中拐点                     -> 状态二
*                    4.找上拐点                     -> 状态三(进环)
*                    5.出环时找另一侧的下拐点 + 赛道变宽 + 丢线数大于阈值
*                  方法二(黑白跳变点)：
*                    1.判断对面那条边界是连续直道
*                    2.下拐点上方为全白；往下隔 3 行扫 4 行，
*                      只要有一行同时存在"白转黑"和"黑转白"跳变 -> 状态一
*                    3.左边界列数大于阈值 -> 状态二
*                    4.另一侧黑色块个数 多 -> 少 -> 多
*                    5.出环时判断两侧白色个数及赛道宽度
*
* 本实现            对侧直线 + 单侧开口 + 白区 P 拐点跨帧确认；全白/跳变行推进状态。
*                  入口用 P 到图像下角的虚拟边线和对侧图像边框求中线；出环停用并等待新拐点。
*                  左右环镜像处理。
*
* 修改记录
* 日期              作者           备注
* 2026-xx-xx        user           first version
********************************************************************************************************************/

#ifndef _ring_road_h_
#define _ring_road_h_

#include "zf_common_headfile.h"
#include "vision_geometry.h"

/* fw78: independent switches for hardware/offline A/B verification. */
#ifndef RING_FRAME_CACHE_ENABLE
#define RING_FRAME_CACHE_ENABLE (1)
#endif
#ifndef RING_ENTRY_GATE_ENABLE
#define RING_ENTRY_GATE_ENABLE (1)
#endif
#ifndef RING_ENTRY_PROBE_INTERVAL
#define RING_ENTRY_PROBE_INTERVAL (4)
#endif
#if RING_ENTRY_PROBE_INTERVAL < 1 || RING_ENTRY_PROBE_INTERVAL > 255
#error RING_ENTRY_PROBE_INTERVAL must be in 1..255
#endif
/* fw79: partial searches are confined to confirmed direct-P tracking. */
#ifndef RING_STATE_ROI_ENABLE
#define RING_STATE_ROI_ENABLE (1)
#endif
#define RING_ROI_RADIUS_NEAR  (VISION_PX(8))
#define RING_ROI_RADIUS_WIDE  (VISION_PX(20))
#define RING_ROI_EDGE_GUARD   (VISION_PX(2))
#define RING_ROI_COL_MAX      (VISION_PX(16))
#define RING_ROI_JUMP_MARGIN  (VISION_PX(4))
#ifndef RING_ROI_PROBE_INTERVAL
#define RING_ROI_PROBE_INTERVAL (4)
#endif
#if RING_ROI_PROBE_INTERVAL < 1 || RING_ROI_PROBE_INTERVAL > 255
#error RING_ROI_PROBE_INTERVAL must be in 1..255
#endif
#define RING_ROI_STAGE_NONE   (0)
#define RING_ROI_STAGE_NEAR   (1)
#define RING_ROI_STAGE_WIDE   (2)
#define RING_ROI_STAGE_GLOBAL (3)
#define RING_GATE_BORDER_MARGIN (VISION_PX(8))
#define RING_GATE_EDGE_JUMP     (VISION_PX(8))  /* Four-row displacement, in pixels. */
/* fw81: temporary opposite-side search after a verified exit, in frames. */
#define RING_CHAIN_WINDOW_FRAMES (24)
#define RING_CHAIN_PASS_ROW      (VISION_H * 3 / 4)
#define RING_CHAIN_P_ROW_MAX     (VISION_H / 2)


//------------------------------------------------------------------------------------------------------------------
// 圆环类型
//------------------------------------------------------------------------------------------------------------------
#define RING_TYPE_NONE          ( 0 )                               // 不是圆环
#define RING_TYPE_LEFT          ( 1 )                               // 左圆环(环在车的左边)
#define RING_TYPE_RIGHT         ( 2 )                               // 右圆环(环在车的右边)

//------------------------------------------------------------------------------------------------------------------
// 圆环状态
//------------------------------------------------------------------------------------------------------------------
#define RING_STATE_NONE         ( 0 )                               // 不在圆环里
#define RING_STATE_FIND         ( 1 )                               // 识别到圆环入口
#define RING_STATE_IN           ( 2 )                               // 正在进环
#define RING_STATE_INSIDE       ( 3 )                               // 已经在环里
#define RING_STATE_OUT          ( 4 )                               // 正在出环

// Serial diagnosis: why the state or guidance changed on this frame.
#define RING_DIAG_NONE          ( 0 )
#define RING_DIAG_FIND          ( 1 )
#define RING_DIAG_IN            ( 2 )
#define RING_DIAG_INSIDE        ( 3 )
#define RING_DIAG_OUT_LOST      ( 4 )
#define RING_DIAG_OUT_CORNER    ( 5 )
#define RING_DIAG_DONE_CLEAR    ( 6 )
#define RING_DIAG_DONE_TIMEOUT  ( 7 )
#define RING_DIAG_DONE_RESTART  ( 8 )
#define RING_DIAG_DONE_GAP      ( 9 )
#define RING_DIAG_FIND_TIMEOUT  ( 10 )
#define RING_DIAG_TIMEOUT       ( 11 )
#define RING_DIAG_CANCEL        ( 12 )
#define RING_DIAG_P_LOST        ( 13 )
#define RING_DIAG_DONE_NEW_P    ( 14 )
#define RING_DIAG_DONE_CHAIN    ( 15 )

typedef struct
{
    uint16 frame;
    uint8 previous_state;
    uint8 reason;
    uint8 p_source;                // 0=none, 1=direct, 2=wedge, 3=held
    uint8 p_row;
    uint8 p_col;
    uint8 p_missing;
    uint8 candidate_side;
    uint8 candidate_row;
    uint8 candidate_col;
    uint8 candidate_run;
    uint8 guide_active;
    uint8 guide_blocked;
    uint8 guide_applied;           // 0=none, 1=entry, 2=corner, 3=tail, 4=both, 5=exit gap
    uint8 exit_probe;              // 0=not checked, 1..5=rejection, 6/7=accepted
    uint8 tail_active;
    uint8 tail_row;
    uint8 tail_col;
    uint8 exit_missing;
    uint8 exit_max_row;
    uint8 out_clear_frames;
    uint8 clear_frames;
    uint8 wait_second;
    uint8 open_row1;
    uint8 open_row2;
    uint8 lost_left;
    uint8 lost_right;
    uint8 cross_state;
    uint8 cross_bridge;
} ring_diag_t;

//------------------------------------------------------------------------------------------------------------------
// 参数：对面边界"连续直道"判断（圆环外面那一侧应该是直的）
//------------------------------------------------------------------------------------------------------------------
#define RING_STRAIGHT_Y1        ( VISION_H * 3 / 5 )               // 取行范围 起点(靠近车身)
#define RING_STRAIGHT_Y2        ( VISION_LAST_ROW )                   //            终点
#define RING_STRAIGHT_LOST_MAX  ( VISION_PX(10) )                              // 这段里对面边线最多丢几行
                                                                    // 实车帧实测：左环入口时对面(右边)丢了 8 行，
                                                                    // 原来写 2 会把左环直接否掉；直线度由下面的偏差阈值保证
#define RING_STRAIGHT_DEV_MAX   ( VISION_PX(6) )                               // 直线度判据：拟合后偏差的**中位数**上限(像素)
                                                                    // 注意这里用中位数不用最大值：实车实测对面边线肉眼看是直的，
                                                                    // 但数组里偶尔有几行离群(种子行误差/跳变)，最大偏差被拉到 58 像素，
                                                                    // 用中位数就只剩几个像素，不会误判
#define RING_DEV_BIN            ( 64 )                              // 偏差直方图的分档数
#define RING_STRAIGHT_MIN_PT    ( VISION_PX(12) )                              // 拟合至少需要几个点

//------------------------------------------------------------------------------------------------------------------
// 参数：入口拐点(边界向左/向右的突变)
//------------------------------------------------------------------------------------------------------------------
#define RING_SCAN_NEAR          ( VISION_H - VISION_PX(6) )                   // 拐点扫描范围 起点(靠近车身)
#define RING_SCAN_FAR           ( VISION_H / 4 )                   //                终点
// 入口侧"道路张开"的判据：一段足够长的连续丢线
// PC 仿真 + 实车帧实测：圆环入口时，入口那一侧的丢线会连成一大段（实测右环 56 行），
// 而对面那一侧几乎不丢线 —— 这是圆环和十字最大的区别
#define RING_P_OPEN_LOST_MIN    ( VISION_PX(6) )                               // A verified P corner can use a short open run.
#define RING_OPPO_LOST_MAX      ( VISION_PX(8) )                               // 这段行范围里 对面最多允许丢几行
#define RING_OPPO_RUN_MAX       ( VISION_PX(15) )                              // 对面那条边线自己也不允许有太长的连续丢线段
                                                                    // 极端斜入十字时左右两边的丢线段是"接着"的，
                                                                    // 只靠上面那条区分不开，加上这一条就干净了
                                                                    // (实车左环入口 对面只丢 8 行，不影响)

//------------------------------------------------------------------------------------------------------------------
// 参数：全白行 / 黑白跳变行
//------------------------------------------------------------------------------------------------------------------
#define RING_WHITE_MIN          ( VISION_PX(170) )                             // 一行里白点超过本值算"全白行"(共188点)
#define RING_TRANS_MIN          ( 2 )                               // 一行里黑白跳变次数达到本值算"跳变行"
#define RING_FEATURE_ROW1       ( VISION_H / 6 )                   // 找全白行/跳变行的范围(图像上半部分)
#define RING_FEATURE_ROW2       ( VISION_H * 3 / 5 )

//------------------------------------------------------------------------------------------------------------------
// 参数：圆环补线
//------------------------------------------------------------------------------------------------------------------
// 1：开启圆环补线  0：只识别不补线
// 想对比"补了/没补"的差别时把它改成 0 就行
#define RING_FIX_ENABLE         ( 1 )
#define RING_P_ROW_MIN          ( VISION_PX(12) )
#define RING_P_ROW_MAX          ( VISION_H - VISION_PX(6) )
#define RING_P_JUMP_MIN         ( VISION_W * 14 / 100 )
#define RING_P_OPPO_SHIFT_MAX   ( VISION_W * 5 / 100 )
#define RING_P_BORDER_MIN       ( VISION_W * 90 / 100 )
#define RING_P_CLEAR_FRAMES     ( 3 )  // After exit, require a clear gap before a new corner can arm.

//------------------------------------------------------------------------------------------------------------------
// 参数：状态切换
//------------------------------------------------------------------------------------------------------------------
#define RING_IN_LOST_MIN        ( VISION_PX(35) )                              // 入口一侧丢线超过本值 -> 判定已经进环
#define RING_OUT_LOST_MAX       ( VISION_PX(12) )                              // 入口一侧丢线少于本值 -> 判定已经出环
// 状态1(识别到入口)里 连续多少帧还等不到"那一侧道路张开"就判定这个入口是假的 复位重来
// 实车事故：直道上误判成左环(ch4=11)，原来要等 RING_STATE_TIMEOUT=200帧(约10秒)才自己好，
//           中间左边线一直被补歪
#define RING_FIND_FAIL_MAX      ( 60 )
#define RING_STATE_TIMEOUT      ( 200 )                             // 状态保护：连续多少帧没进展就强制复位
#define RING_OUT_MAX_FRAMES     ( RING_STATE_TIMEOUT )              // Safety only; normal exit uses image evidence.
#define RING_CONFIRM_FRAME      ( 2 )                               // 连续多少帧都满足条件才确认(滤掉偶发)

//------------------------------------------------------------------------------------------------------------------
// 识别结果
//------------------------------------------------------------------------------------------------------------------
extern uint8 ring_flag;                                             // 圆环标志位 1:在圆环元素中
extern uint8 ring_type;                                             // RING_TYPE_NONE / LEFT / RIGHT
extern uint8 ring_state;                                            // RING_STATE_xxx
extern uint8 ring_exit_active;                                      // Exit corner is valid for this frame.
extern uint8 ring_exit_row;
extern uint8 ring_exit_col;
extern uint8 ring_corner_row;                                       // 入口拐点所在行
extern uint8 ring_corner_col;                                       // 入口拐点所在列
extern uint8 ring_white_row;                                        // 找到的全白行(没有则 0)
extern uint8 ring_trans_row;                                        // 找到的黑白跳变行(没有则 0)
extern uint8 ring_open_row1;                                        // 入口侧那段连续丢线的起止行
extern uint8 ring_open_row2;
extern int8  ring_fail;                                             // 没识别到时的原因(诊断用)：
                                                                    //   0 = 没跑
                                                                    //   -2 = 入口侧道路没张开
                                                                    //   -3 = 对面也有长丢线段 或 对面在张开段也丢线
                                                                    //   -1x = 对面不算连续直道，x 是子原因：
                                                                    //        -11 对面丢线太多   -12 对面有效点太少
                                                                    //        -13 拟合退化       -14 对面不够直
extern uint8 ring_dbg_dev;                                          // 对面边线拟合后的最大偏差(像素)
extern uint8 ring_dbg_lost;                                         // 对面边线在取行范围内的丢线行数
extern uint8 ring_rep_dev;                                          // 主尝试(丢线多那一侧)测到的拟合偏差
extern uint8 ring_rep_lost;                                         // 主尝试测到的对面丢线行数

//------------------------------------------------------------------------------------------------------------------
// 函数简介     圆环识别
// 参数说明     *binary_image      二值化图像(0/255) 通常传 image_copy[0]
// 返回参数     void
// 使用示例     track_edge_process(image_copy[0]);
//              cross_road_process(image_copy[0]);
//              ring_road_process(image_copy[0]);
//              track_white_lane_guard(image_copy[0]);
//              ring_road_apply_guidance();
// 备注信息     结果放在 ring_flag / ring_type / ring_state 中
//------------------------------------------------------------------------------------------------------------------
void ring_road_process (uint8 *binary_image);
uint8 ring_road_preempt_cross (void);
uint8 ring_road_entry_transition (void);
void ring_road_apply_guidance (void);
void ring_road_cancel (void);
uint8 ring_guide_row_valid (uint8 row);
void ring_road_get_diag (ring_diag_t *diag);

#endif
