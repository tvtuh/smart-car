/* Actual cross/ring primitives from the visual revision; no copied algorithm. */
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "../code/cross_road.c"
#include "../code/ring_road.c"
uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
static uint8 image[VISION_IMAGE_SIZE];
#define CHECK(c,m) do{if(!(c)){fputs("FAIL " m "\n",stderr);return 1;}}while(0)
static uint8 right_island(uint8 row,int16 end,int16 length)
{
#ifdef TEST_RIGHT_ISLAND_AVAILABLE
    return ring_right_island_visible(row,end,length);
#else
    (void)row;(void)end;(void)length;return 0;
#endif
}
static void island_shape(uint8 right,int monotone)
{
    int y,x,d;
    for(y=0;y<VISION_H;y++)
    {track_left[y]=12;track_right[y]=81;track_center[y]=47;track_lost[y]=0;}
    for(y=18;y<=51;y++)
    {
        d=y-18;
        x=monotone?8+d/2:(d<=8?8+d:(d<=20?24-d:4));
        if(right){track_right[y]=(uint8)(VISION_W-1-x);}
        else{track_left[y]=(uint8)x;}
    }
}
int main(int argc,char **argv)
{
    int mode,y,x,i,from,direction,side,anchor,left,right;
    static const int curved[12]={20,21,23,26,29,31,32,32,32,32,32,32};
    if(argc!=2)return 2;mode=atoi(argv[1]);
    if(mode<=5)
    {
        direction=(mode==3)?-1:1;from=direction>0?19:40;
        side=(mode==1)?0x08:0x04;
        memset(image,0,sizeof(image));memset(cross_image_row_state,0,sizeof(cross_image_row_state));
        for(y=0;y<VISION_H;y++)
        {
            i=(y-from)*direction-1;x=20;
            if(i>=0 && i<12){x=(mode<=1)?curved[i]:20+i;}
            if(mode==4){x=0;}
            if(mode==5 && i>=2){x=0;}
            left=(side==0x04)?x:12;right=(side==0x08)?VISION_W-1-x:80;
            for(x=left;x<=right;x++){image[y*VISION_W+x]=220;}
            track_center[y]=(uint8)((left+right)/2);
        }
        anchor=cross_image_anchor(image,(int16)from,(int16)direction,(uint8)side);
        printf("anchor mode=%d row=%d\n",mode,anchor);
        if(mode<=1){CHECK(anchor==from+direction*6,"rounded cross corner was accepted as a straight anchor");}
        else if(mode==4){CHECK(anchor==-1,"open image edge was accepted as an anchor");}
        else{CHECK(anchor==from+direction,"genuine visible straight edge was rejected/delayed");}
    }
    else
    {
        island_shape(mode!=9,mode==7);
        if(mode==8){for(y=18;y<=51;y++){track_lost[y]=3;}}
        if(mode==6){CHECK(right_island(10,16,RING_P_OPEN_LOST_MIN),"right island was not recognized symmetrically");}
        if(mode==7 || mode==8){CHECK(!right_island(10,16,RING_P_OPEN_LOST_MIN),"diagonal/lost edges impersonated a right island");}
        if(mode==9){CHECK(ring_left_island_visible(10,16,RING_P_OPEN_LOST_MIN),"existing left-island recognition changed");}
#ifdef TEST_RIGHT_ISLAND_AVAILABLE
        if(mode>=10)
        {
            for(y=10;y<=16;y++){track_lost[y]=2;}
            ring_guide_blocked=1;ring_cross_right_entry=0;
            ring_block_after_cross=(mode==11)?1:0;
            cross_flag=(mode==11)?1:0;cross_bridge_used=0;
            cross_state=(mode==11)?CROSS_STATE_IN:CROSS_STATE_NONE;
            CHECK(!ring_cross_right_rearm_ready(1,10,60,0) && !ring_cross_right_entry,
                  "right-island rearm bypassed an old-exit block or active cross");
        }
#endif
    }
    printf("PASS visual revision2 mode=%d\n",mode);return 0;
}

