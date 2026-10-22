/* Regression: servo occlusion in the bottom captured band must not erase
 * the visible lane above it or change its automatic threshold. */
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "cross_road.h"
#include "ring_road.h"

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
static uint8 processed[(MT9V03X_H / VISION_DOWNSAMPLE) * VISION_W];

int main(int argc, char **argv)
{
    int x, y, mode;
    int bottom = MT9V03X_H - VISION_DOWNSAMPLE;
    if(argc != 2) { return 2; }
    mode = atoi(argv[1]);
    if(mode == 3)
    {
        memset(mt9v03x_image, 100, sizeof(mt9v03x_image));
        for(y = bottom; y < MT9V03X_H; ++y)
        { memset(mt9v03x_image[y], 0, MT9V03X_W); }
        x = image_threshold_iteration();
        if(x != 100)
        { fprintf(stderr, "FAIL threshold: bottom occlusion changed visible gray 100 to %d\n", x); return 1; }
        puts("PASS automatic threshold ignores the bottom occluded band");
        return 0;
    }
    for(y = 0; y < MT9V03X_H; ++y)
    {
        for(x = 0; x < MT9V03X_W; ++x)
        { mt9v03x_image[y][x] = (x >= 40 && x <= 148) ? 220 : 0; }
    }
    if(mode == 1 || mode == 2)
    {
        for(y = bottom; y < MT9V03X_H; ++y)
        {
            memset(mt9v03x_image[y], 0, MT9V03X_W);
            if(mode == 2) { memset(mt9v03x_image[y], 220, 21); }
        }
    }
    if(mode == 4)
    {
        for(y = 50; y < 52; ++y)
        { memset(mt9v03x_image[y], 0, MT9V03X_W); }
    }
    image_gray_copy(processed);
    track_edge_process(processed);
    ring_road_process(processed);
    cross_road_process(processed);
    track_line_smooth();
    track_pin_lost_edges();
    track_temporal_filter();
    track_white_lane_guard(processed);
    cross_road_save_final();
    ring_road_apply_guidance();
    for(y = 0; y <= 58; ++y)
    {
        if(mode == 4 && y <= 25)
        {
            if(track_center[y] != 255)
            { fprintf(stderr, "FAIL interior black break was ignored at row %d\n", y); return 1; }
        }
        else if(track_left[y] != 20 || track_center[y] != 47 || track_right[y] != 74)
        {
            fprintf(stderr, "FAIL mode=%d row=%d lane=%u/%u/%u (expected 20/47/74)\n",
                    mode, y, track_left[y], track_center[y], track_right[y]);
            return 1;
        }
    }
    printf("PASS mode=%d visible lane preserved; processing image=%dx%d\n", mode, VISION_W, VISION_H);
    return 0;
}