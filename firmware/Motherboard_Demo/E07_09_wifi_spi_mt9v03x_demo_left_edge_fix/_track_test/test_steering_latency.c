/* Real vision postprocessing: verify the selected temporal mode and retain
 * within-frame spike filtering. With median enabled, a new bend lags one frame. */
#include "zf_common_headfile.h"
#include "image_process.h"
#include "track_edge.h"
#include "cross_road.h"
#include "ring_road.h"

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
static void sample(int center)
{
    int row;
    for(row=0;row<VISION_H;row++)
    {
        track_left[row]=(uint8)(center-27);
        track_right[row]=(uint8)(center+27);
        track_center[row]=(uint8)center;
        track_lost[row]=0;
    }
}
int main(int argc,char **argv)
{
    int mode,i,center,expected,previous=47;
    if(argc!=2){return 2;}
    mode=atoi(argv[1]);
    for(i=0;i<4;i++){sample(47);track_temporal_filter();}
    if(mode==3)
    {
        sample(47);
        track_left[40]=28;track_right[40]=82;track_center[40]=55;
        track_line_smooth();track_temporal_filter();
        if(track_center[40]!=47)
        {fprintf(stderr,"FAIL within-frame spike survived: %u\n",track_center[40]);return 1;}
    }
    else
    {
        for(i=0;i<4;i++)
        {
            center=(mode==0)?53:((mode==1)?41:(48+i));
            sample(center);track_line_smooth();track_temporal_filter();
            expected=TRACK_TEMPORAL_ENABLE?previous:center;
            if(track_center[40]!=expected)
            {
                fprintf(stderr,"FAIL bend frame %d: expected filtered center=%d, got=%u\n",i,expected,track_center[40]);
                return 1;
            }
            previous=center;
        }
    }
    printf("PASS selected temporal-mode vision mode=%d median=%d\n",mode,TRACK_TEMPORAL_ENABLE);return 0;
}
