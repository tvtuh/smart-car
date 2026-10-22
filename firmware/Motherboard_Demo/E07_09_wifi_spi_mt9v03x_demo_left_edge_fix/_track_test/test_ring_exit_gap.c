/* Seed an already confirmed OUT state; run the real vision pipeline through
 * a wide white exit gap, its recovery, and loss of the history budget. */
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "cross_road.h"
/* Internal state is fixture setup only; no production test API is needed. */
#include "../code/ring_road.c"

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
static uint8 processed[VISION_IMAGE_SIZE];

static void run_frame(void)
{
    track_edge_process(processed);
    ring_road_process(processed);
    track_line_smooth();
    track_pin_lost_edges();
    track_temporal_filter();
    track_white_lane_guard(processed);
    cross_road_save_final();
    ring_road_apply_guidance();
}

int main(int argc, char **argv)
{
    int mode, frame, x, y, border;
    uint8 *edge;
    if(argc != 2) { return 2; }
    mode = atoi(argv[1]);
    image_threshold = 100;
    memset(processed, 220, sizeof(processed));
    if(mode == 7) { memset(processed, 0, sizeof(processed)); }
    ring_state = RING_STATE_OUT;
    ring_type = (mode == 1) ? RING_TYPE_RIGHT : RING_TYPE_LEFT;
    ring_flag = 1;
    ring_exit_seen = (mode != 2);
    ring_exit_row = 46;
    ring_exit_col = (mode == 1) ? 22 : 71;
    ring_exit_max_row = 46;
    if(mode == 3) { cross_state = CROSS_STATE_INSIDE; cross_flag = 1; }
    for(frame = 1; frame <= 5; ++frame) { run_frame(); }
    border = (mode == 1) ? 0 : VISION_W - 1;
    edge = (mode == 1) ? track_left : track_right;
    printf("mode=%d missing=%u guide=%u edge25/40/50=%u/%u/%u\n",
           mode, ring_exit_missing, ring_diag_guide_applied, edge[25], edge[40], edge[50]);
    printf("state/type=%u/%u bridge=%u/%u/%u lost=%u/%u/%u center=%u/%u/%u\n",
           ring_state, ring_type, ring_exit_bridge[25], ring_exit_bridge[40], ring_exit_bridge[50],
           track_lost[25], track_lost[40], track_lost[50], track_center[25], track_center[40], track_center[50]);
    if(mode == 7)
    {
        if(ring_diag_guide_applied == 5 || ring_guide_row_valid(40))
        { fputs("FAIL old exit geometry was continued through a black frame\n", stderr); return 1; }
    }
    else if(mode == 2 || mode == 3)
    {
        if(edge[40] != border || ring_guide_row_valid(40))
        { fputs("FAIL fabricated exit history or overrode an active cross\n", stderr); return 1; }
    }
    else
    {
        if(edge[25] == border || edge[40] == border || edge[50] == border ||
           !ring_guide_row_valid(25) || !ring_guide_row_valid(40) || !ring_guide_row_valid(50))
        { fputs("FAIL confirmed exit gap fell back to the image border\n", stderr); return 1; }
    }
    if(mode == 4)
    {
        for(frame = 6; frame <= 100; ++frame) { run_frame(); }
        if(edge[40] != border || ring_guide_row_valid(40))
        { fputs("FAIL stale exit guide survived the recovery budget\n", stderr); return 1; }
    }
    if(mode == 5)
    {
        memset(processed, 0, sizeof(processed));
        for(y = 0; y < VISION_H; ++y)
        { for(x = 20; x <= 74; ++x) { processed[y * VISION_W + x] = 220; } }
        for(frame = 0; frame < 5; ++frame) { run_frame(); }
        for(y = 0; y < VISION_H; ++y)
        {
            if(track_left[y] != 20 || track_right[y] != 74 || track_center[y] != 47)
            { fputs("FAIL recovered physical lane was overwritten by stale guidance\n", stderr); return 1; }
        }
    }
    if(mode == 6)
    {
        for(y = 0; y < VISION_H; ++y)
        {
            for(x = 10; x <= 14; ++x) { processed[y * VISION_W + x] = 0; }
            track_left[y] = 0; track_right[y] = VISION_W - 1; track_lost[y] = 3;
        }
        memset(ring_exit_bridge, 255, sizeof(ring_exit_bridge));
        ring_prepare_exit_bridge(processed);
        if(ring_exit_bridge[40] != 255)
        { fputs("FAIL two white samples bridged a disconnected black band\n", stderr); return 1; }
    }
    if(mode == 8)
    {
        ring_road_cancel();
        track_edge_process(processed);
        ring_road_apply_guidance();
        if(ring_diag_guide_applied || ring_guide_row_valid(40) || track_right[40] != VISION_W - 1)
        { fputs("FAIL cancellation kept a stale exit continuation\n", stderr); return 1; }
    }
    puts("PASS exit gap behavior");
    return 0;
}
