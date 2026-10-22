/* Integration checks for the incoming physical-pair API and enabled temporal median. */
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "cross_road.h"
#include "ring_road.h"
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
static uint8 image[VISION_IMAGE_SIZE];
uint8 track_row_pair_current(uint8 row);
/* Baseline has no API. Link it to a failing fallback until the real vision
 * implementation is merged, so missing behavior is an assertion failure. */
uint8 missing_current_pair(uint8 row){(void)row;return 0;}
#pragma comment(linker,"/alternatename:track_row_pair_current=missing_current_pair")
#define CHECK(c,m) do{if(!(c)){fputs("FAIL " m "\n",stderr);return 1;}}while(0)
static void sample(int center)
{
    int y;
    for(y=0;y<VISION_H;y++)
    {
        track_left[y]=(uint8)(center-27);track_right[y]=(uint8)(center+27);
        track_center[y]=(uint8)center;track_lost[y]=0;
    }
}
static void lane_image(int left,int right)
{
    int y,x;
    memset(image,0,sizeof(image));
    for(y=0;y<VISION_H;y++)
    {
        for(x=left;x<=right;x++){image[y*VISION_W+x]=220;}
    }
    sample((left+right)/2);
}
static void pipeline_until_temporal(void)
{
    track_edge_process(image);ring_road_process(image);cross_road_process(image);
    track_line_smooth();track_pin_lost_edges();track_temporal_filter();
}
static void finish_pipeline(void)
{
    track_white_lane_guard(image);cross_road_save_final();ring_road_apply_guidance();
}
int main(int argc,char **argv)
{
    int mode,i,stage_center=0;
    if(argc!=2)return 2;
    mode=atoi(argv[1]);
    if(mode==8 || mode==9)
    {
        for(i=0;i<4;i++)
        {
            lane_image(20,74);pipeline_until_temporal();
            stage_center=track_center[33];finish_pipeline();
        }
        if(mode==8)
        {
            lane_image(26,80);pipeline_until_temporal();
            printf("pipeline before guard: center=%u edges=%u/%u lost=%u\n",track_center[33],track_left[33],track_right[33],track_lost[33]);
            CHECK(track_center[33]==stage_center,"temporal stage did not retain the previous straight sample");
            finish_pipeline();
            CHECK(track_center[33]==53 && track_row_pair_current(33),
                  "final guard did not revalidate the median against current physical edges");
        }
        else
        {
            memset(image,0,sizeof(image));pipeline_until_temporal();finish_pipeline();
            for(i=0;i<VISION_H;i++)
            {
                CHECK(track_center[i]==255 && !track_row_pair_current((uint8)i),
                      "temporal history created a valid route in a current black frame");
            }
        }
    }
    else if(mode==1 || mode==2)
    {
        for(i=0;i<4;i++){sample(47);track_temporal_filter();}
        sample(53);track_temporal_filter();
        CHECK(track_center[33]==47,"enabled temporal median must reject the first isolated offset");
        sample(mode==1?47:53);track_temporal_filter();
        CHECK(track_center[33]==(mode==1?47:53),"temporal median failed isolated-noise rejection or persistent-bend release");
    }
    else
    {
        lane_image(20,74);track_white_lane_guard(image);
        CHECK(track_row_pair_current(33),"current image's physical pair was not exposed");
        if(mode==3)
        {
            track_left[33]=30;track_right[33]=65;track_center[33]=47;
            CHECK(!track_row_pair_current(33),"virtual coordinates still advertised as the physical pair");
        }
        else if(mode==4)
        {
            memset(image,0,sizeof(image));track_white_lane_guard(image);
            CHECK(!track_row_pair_current(33),"a black next frame reused stale physical-pair metadata");
        }
        else if(mode==5)
        {
            memset(image,220,sizeof(image));sample(47);track_white_lane_guard(image);
            CHECK(!track_row_pair_current(33),"screen-clipped/all-white route counted as two physical edges");
        }
        else if(mode==6)
        {
            CHECK(!track_row_pair_current((uint8)VISION_H) && !track_row_pair_current(255),
                  "out-of-range/excluded row exposed a physical pair");
            CHECK(track_row_pair_current((uint8)(VISION_H-1)),"last valid row lost its physical-pair metadata");
        }
        else if(mode==7)
        {
            memset(image,0,sizeof(image));track_white_lane_guard(image);
            lane_image(20,74);track_white_lane_guard(image);
            CHECK(track_row_pair_current(33),"new physical edges were not reacquired after a black frame");
        }
    }
    printf("PASS merged vision mode=%d\n",mode);return 0;
}

