/*********************************************************************************************************************
* 文件名称          cross_road.h
* 功能简介          十字路口识别、分侧图像补线与后备补线
*
* 主路径：
*   先用画框附近白点筛选开口，再读取包含本行参考中心的连续白段。
*   左右分别寻找开口段；两侧稳定锚点之间的缺边范围至少重叠三行，允许斜十字错位。
*   锚点最多向开口外搜索24行，并由外侧连续三行的平稳边界确认。
*   每侧在自己的上下锚点之间线性补线，未补的一侧使用本行真实白段端点。
*   所有组合行都必须落在同一白段内且宽度有效，整段验证通过后才接管边线。
*   独立保存引导线和逐侧标记，由 track_white_lane_guard 恢复，避免时间滤波修改引导。
*
* 后备路径：
*   保留90%白行连续横杠判据，以及原来的全白/双侧丢线段桥接。
*   主路径无完整上下锚点时，后备可以使用双端插值或单端趋势延伸。
*
* cross_state 报告进入/环节状态；cross_flag 和 cross_bridge_used 同时控制引导有效性。
* 圆环抢占或当前帧没有有效引导时，cross_road_get_guide 返回0。
********************************************************************************************************************/

#ifndef _cross_road_h_
#define _cross_road_h_

#include "zf_common_headfile.h"
#include "vision_geometry.h"

//------------------------------------------------------------------------------------------------------------------
// 横杠判据
//------------------------------------------------------------------------------------------------------------------
#define CROSS_WHITE_PERCENT     ( 90 )                              // 一行里白点比例超过本值 -> 横杠行
#define CROSS_WHITE_STEP        ( VISION_PX(6) )                               // 抽样步长(越小越准越慢)
#define CROSS_WHITE_MIN_ROWS    ( VISION_PX(3) )                               // 至少连续几行才算一段横杠

//------------------------------------------------------------------------------------------------------------------
// 补线（桥接）参数
//------------------------------------------------------------------------------------------------------------------
#define CROSS_BRIDGE_SLOPE_MAX  ( 3 )                               // 趋势/延伸时 每行最多变多少像素
#define CROSS_LOOK_BACK         ( VISION_PX(6) )                               // 估趋势时往回看几行
#define CROSS_BRIDGE_DRIFT_MAX  ( VISION_PX(20) )                              // 趋势延伸时 相对锚点行最多偏多少像素
// Image-confirmed, independent left/right bridges.
#define CROSS_IMAGE_OPEN_MARGIN    ( VISION_PX(8) )
#define CROSS_IMAGE_ANCHOR_SEARCH  ( VISION_PX(24) )
#define CROSS_IMAGE_ANCHOR_ROWS    ( VISION_PX(3) )
/* Eight samples expose the curved mouth even in the reduced image. */
#define CROSS_IMAGE_STABLE_ROWS    ( VISION_PX(16) )
#define CROSS_IMAGE_STABLE_TOL     ( VISION_PX(2) )
#define CROSS_IMAGE_ANCHOR_STEP    ( 3 )
#define CROSS_IMAGE_WIDTH_MIN      ( VISION_PX(20) )
#define CROSS_IMAGE_WHITE_SEARCH   ( VISION_PX(12) )
#define CROSS_IMAGE_LAST_ROW       ( VISION_H - VISION_PX(4) )
#define CROSS_IMAGE_TREND_ROWS     ( 8 )  // Keep eight samples to suppress coarse-pixel extrapolation noise.
#define CROSS_IMAGE_OPEN_JUMP      ( VISION_PX(12) )
#define CROSS_IMAGE_HOLD_FRAMES    ( 8 )
#define CROSS_IMAGE_WHITE_TOL      ( VISION_PX(4) )  // Snap small corner errors inside the real white run.

#define CROSS_IMAGE_STATUS_IDLE       (0)
#define CROSS_IMAGE_STATUS_OK         (1)
#define CROSS_IMAGE_STATUS_LEFT       (2)
#define CROSS_IMAGE_STATUS_RIGHT      (3)
#define CROSS_IMAGE_STATUS_PAIR       (4)
#define CROSS_IMAGE_STATUS_RUN        (5)
#define CROSS_IMAGE_STATUS_GEOMETRY   (6)
#define CROSS_IMAGE_STATUS_WIDTH      (7)
#define CROSS_IMAGE_STATUS_GUARD      (8)
#define CROSS_IMAGE_STATUS_INSIDE     (9)
extern int16 cross_image_fail_row;
void cross_road_reset_guidance (void);
void cross_road_save_final (void);
extern uint8 cross_image_status;

#define CROSS_BRIDGE_ANCHOR_MARGIN ( VISION_PX(8) )                            // 双端桥接的两侧锚点需离画框至少这么远
                                                                    // （透视只在短距离内近似线性，外推 50+ 行必须封顶）                               // 估趋势时往回看几行

//------------------------------------------------------------------------------------------------------------------
// 状态位（只用于对外汇报）
//------------------------------------------------------------------------------------------------------------------
#define CROSS_STATE_NONE        ( 0 )                               // 不在十字
#define CROSS_STATE_IN          ( 1 )                               // 横杠在车前面
#define CROSS_STATE_INSIDE      ( 2 )                               // 横杠已经在车身下面（车在十字里）
#define CROSS_STATE_OUT         ( 3 )                               // 刚出十字（横杠刚消失）
#define CROSS_OUT_HOLD          ( 15 )                              // 出十字后保持 OUT 多少帧

//------------------------------------------------------------------------------------------------------------------
// 对外变量
//------------------------------------------------------------------------------------------------------------------
extern uint8 cross_flag;                                            // 1:当前帧判定在十字里
extern uint8 cross_state;                                           // CROSS_STATE_xxx
extern uint8 cross_bridge_used;                                     // 1:这一帧桥接生效过(圆环模块拿它做互斥)
extern uint8 cross_bridge_two_end;                                  // 1:上下两端均可靠且边界未贴画框，可保留插值斜率
extern int16 cross_bridge_top;                                      // 本帧桥接的上边界行，-1 表示没有桥接
extern int16 cross_bridge_bottom;                                   // 本帧桥接的下边界行
extern uint8 cross_dbg_up_l;                                        // 诊断：横杠段【上】边界行(0=没有横杠)
extern uint8 cross_dbg_down_l;                                      // 诊断：横杠段【下】边界行(0=没有横杠)

//------------------------------------------------------------------------------------------------------------------
// 函数简介     十字路口识别与补线
// 参数说明     *binary_image      图像(灰度图或二值图都行 内部按 IMG_WHITE_LEVEL 判白)
// 返回参数     void
// 使用示例     track_edge_process(image_copy[0]);
//              cross_road_process(image_copy[0]);
// 备注信息     补线结果直接写进 track_left / track_right / track_center
//------------------------------------------------------------------------------------------------------------------
void cross_road_process (uint8 *binary_image);
// Fresh image bridge only; caller must preserve established ring priority.
uint8 cross_road_try_image (uint8 *image);

// Return 1/2/3 for a validated left/right/both guide, or 0 when inactive.
uint8 cross_road_get_guide (uint16 row, int16 *left, int16 *right);

#endif
