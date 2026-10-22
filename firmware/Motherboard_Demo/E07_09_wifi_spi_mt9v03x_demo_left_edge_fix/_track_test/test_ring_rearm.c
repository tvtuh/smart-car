/* Public-pipeline regression: a cross cancellation must re-arm ring
 * detection after physical lane recovery, at both processing resolutions. */
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "cross_road.h"
#include "ring_road.h"

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
static uint8 processed[VISION_IMAGE_SIZE];

int main(int argc, char **argv)
{
    int frame, x, y, mode;
    ring_diag_t diag;
    if(argc != 2) { return 2; }
    mode = atoi(argv[1]);
    image_threshold = 100;
    memset(processed, 0, sizeof(processed));
    for(y = 0; y < VISION_H; ++y)
    {
        for(x = VISION_PX(40); x <= VISION_PX(148); ++x)
        { processed[y * VISION_W + x] = 220; }
    }
    if(mode == 2) { memset(processed, 220, sizeof(processed)); }

    cross_state = CROSS_STATE_IN;
    cross_flag = 1;
    ring_road_cancel();
    if(mode != 1) { cross_state = CROSS_STATE_NONE; cross_flag = 0; }
    for(frame = 1; frame <= RING_P_CLEAR_FRAMES + 2; ++frame)
    {
        track_edge_process(processed);
        ring_road_process(processed);
        ring_road_get_diag(&diag);
        printf("frame=%d mode=%d downsample=%d blocked=%u clear=%u fail=%d\n",
               frame, mode, VISION_DOWNSAMPLE, diag.guide_blocked,
               diag.clear_frames, ring_fail);
        if((mode == 0) && (frame < RING_P_CLEAR_FRAMES) && !diag.guide_blocked)
        { fputs("FAIL recovery released before stable lane confirmation\n", stderr); return 1; }
        if((mode != 0) && !diag.guide_blocked)
        { fputs("FAIL active cross or wide opening released the ring block\n", stderr); return 1; }
    }
    if((mode == 0) && diag.guide_blocked)
    { fputs("FAIL stable recovered lane never re-armed ring detection\n", stderr); return 1; }
    puts("PASS ring recovery behavior");
    return 0;
}
