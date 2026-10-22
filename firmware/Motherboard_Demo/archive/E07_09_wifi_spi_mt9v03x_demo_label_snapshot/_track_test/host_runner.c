#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "cross_road.h"
#include "ring_road.h"
#include "../code/ring_road.c"

uint8 image_threshold = IMAGE_THRESHOLD_MANUAL;
uint8 image_threshold_auto = 0;
uint8 image_threshold_near = 0;
uint8 image_near_ref = 0;

static uint8 row_is_white(const uint8 *image, int row)
{
    int col;
    int white = 0;
    int total = 0;
    for(col = 0; col < MT9V03X_W; col += CROSS_WHITE_STEP)
    {
        white += image[row * MT9V03X_W + col] >= IMG_WHITE_LEVEL;
        total++;
    }
    return (white * 100 >= total * CROSS_WHITE_PERCENT);
}

static void diagnose_cross(const uint8 *image)
{
    int y;
    int start = -1;
    int length = 0;
    int best_top = -1;
    int best_bottom = -1;
    int best_length = 0;
    for(y = 0; y <= MT9V03X_H; y++)
    {
        int need = y < MT9V03X_H &&
                   (row_is_white(image, y) || (track_lost[y] & 3) == 3);
        if(need)
        {
            if(start < 0) { start = y; }
            length++;
        }
        else if(start >= 0)
        {
            if(length > best_length)
            {
                best_top = start;
                best_bottom = y - 1;
                best_length = length;
            }
            start = -1;
            length = 0;
        }
    }
    if(best_top >= 0)
    {
        int upper = best_top - 1;
        int lower = best_bottom + 1;
        fprintf(stderr, "bar=%d..%d upper_lost=%d lower_lost=%d ",
                best_top, best_bottom,
                upper >= 0 ? track_lost[upper] & 3 : -1,
                lower < MT9V03X_H ? track_lost[lower] & 3 : -1);
    }
}

int main(int argc, char **argv)
{
    uint8 image[MT9V03X_W * MT9V03X_H];
    int force_corner = -1;
    int skip_white_guard = 0;

    if(argc == 3 && strcmp(argv[1], "--force-right-ring") == 0)
    {
        force_corner = atoi(argv[2]);
        if(force_corner <= 0 || force_corner >= MT9V03X_H) { return 4; }
    }
    else if(argc == 2 && strcmp(argv[1], "--no-white-guard") == 0)
    {
        skip_white_guard = 1;
    }

    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);

    if(fread(image, 1, sizeof(image), stdin) != sizeof(image))
    {
        fprintf(stderr, "expected %u grayscale bytes\n", (unsigned)sizeof(image));
        return 2;
    }

    track_edge_process(image);
    diagnose_cross(image);
    cross_road_process(image);
    if(force_corner > 0)
    {
        ring_flag = 1;
        ring_type = RING_TYPE_RIGHT;
        ring_state = RING_STATE_FIND;
        ring_corner_row = (uint8)force_corner;
        ring_corner_col = track_right[force_corner];
        ring_fix_line();
    }
    else
    {
        ring_flag = 0;
        ring_type = RING_TYPE_NONE;
        ring_state = RING_STATE_NONE;
        ring_corner_row = 0;
        ring_corner_col = 0;
    }
    track_line_smooth();
    track_pin_lost_edges();
    track_temporal_filter();
    if(0 == skip_white_guard) { track_white_lane_guard(image); }

    fprintf(stderr, "cross=%u bridge=%u two_end=%u cross_flag=%u cross_rows=%u..%u ring=%u type=%u right_lost=%u ",
            (unsigned)cross_state, (unsigned)cross_bridge_used, (unsigned)cross_bridge_two_end,
            (unsigned)cross_flag, (unsigned)cross_dbg_up_l, (unsigned)cross_dbg_down_l,
            (unsigned)ring_state, (unsigned)ring_type,
            (unsigned)track_right_lost_count);
    fprintf(stderr, "ring_fail=%d corner=%u open=%u..%u dev=%u opp_lost=%u max_open=%u\n",
            (int)ring_fail, (unsigned)ring_corner_row,
            (unsigned)ring_open_row1, (unsigned)ring_open_row2,
            (unsigned)ring_dbg_dev, (unsigned)ring_dbg_lost,
            (unsigned)ring_rep_lost);

    if(fwrite(track_left, 1, MT9V03X_H, stdout) != MT9V03X_H ||
       fwrite(track_center, 1, MT9V03X_H, stdout) != MT9V03X_H ||
       fwrite(track_right, 1, MT9V03X_H, stdout) != MT9V03X_H ||
       fwrite(track_lost, 1, MT9V03X_H, stdout) != MT9V03X_H)
    {
        return 3;
    }

    return 0;
}
