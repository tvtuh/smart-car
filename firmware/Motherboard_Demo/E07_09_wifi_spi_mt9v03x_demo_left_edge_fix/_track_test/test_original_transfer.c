/* Exercise the original main.c send helper and link window extracted verbatim. */
#include "zf_common_headfile.h"
#include "car_config.h"
#include "image_process.h"
volatile uint8 mt9v03x_finish_flag=0;
static uint16 test_tick,latency_ticks;
static int camera_calls,heartbeat_guards,overlays,warnings,recoveries;
static uint8 upload_on=1;
uint16 motor_tick_get(void){return test_tick;}
void motor_heartbeat_link_guard(void){heartbeat_guards++;}
void image_draw_ring_state(void){overlays++;}
void seekfree_assistant_camera_send(void){camera_calls++;test_tick=(uint16)(test_tick+latency_ticks);}
uint8 car_img_send_ok(void){return upload_on;}
void car_wireless_progress(const char *s)
{
    if(strstr(s,"image paused"))warnings++;
    if(strstr(s,"image resumed"))recoveries++;
}
#include "original_flow_under_test.inc"
#define CHECK(c,m) do{if(!(c)){fputs("FAIL " m "\n",stderr);return 1;}}while(0)
int main(int argc,char **argv)
{
    int mode,i,old_calls;
    if(argc!=2)return 2;mode=atoi(argv[1]);
    if(mode==0)
    {
        CHECK(car_send_image_once() && camera_calls==1 && heartbeat_guards==1,
              "original synchronous helper or pre-send heartbeat guard missing");
    }
    else if(mode==1)
    {
        latency_ticks=CAR_TELEMETRY_SLOW_MS/CAR_SPEED_PERIOD_MS;
        CHECK(!car_send_image_once(),"slow synchronous send was not classified slow");
    }
    else if(mode==2)
    {
        mt9v03x_finish_flag=1;original_transfer_step();original_transfer_step();
        CHECK(camera_calls==2,"original transfer still blocked by pending-frame or20ms scheduling");
    }
    else if(mode==3 || mode==4)
    {
        for(i=0;i<CAR_LINK_WIN_SENDS;i++)
        {
            latency_ticks=(mode==4 || i<CAR_LINK_WIN_MAX)?CAR_TELEMETRY_SLOW_MS/CAR_SPEED_PERIOD_MS:0;
            original_transfer_step();
        }
        CHECK(link_slow_giveup && warnings==1,"slow-send window did not pause image upload");
        old_calls=camera_calls;original_transfer_step();
        CHECK(camera_calls==old_calls,"paused link probed before its original interval");
        test_tick=(uint16)(link_probe_tick+CAR_LINK_PROBE_MS/CAR_SPEED_PERIOD_MS);
        latency_ticks=(mode==4)?CAR_TELEMETRY_SLOW_MS/CAR_SPEED_PERIOD_MS:0;
        original_transfer_step();
        if(mode==4)
        {
            CHECK(link_slow_giveup && !recoveries,"slow probe incorrectly resumed upload");
            test_tick=(uint16)(link_probe_tick+CAR_LINK_PROBE_MS/CAR_SPEED_PERIOD_MS);
            latency_ticks=0;original_transfer_step();
        }
        CHECK(!link_slow_giveup && recoveries==1,"successful original probe failed to resume upload");
    }
    else if(mode==5)
    {
        upload_on=0;original_transfer_step();CHECK(!camera_calls,"image toggle-off was ignored");
        upload_on=1;original_transfer_step();CHECK(camera_calls==1,"image toggle-on did not resume transmission");
    }
    else if(mode==6)
    {
        vision_ok=0;original_transfer_step();CHECK(!camera_calls,"failed vision link still sent an image");
    }
    else if(mode==7)
    {
        for(i=0;i<CAR_LINK_WIN_SENDS;i++)original_transfer_step();
        CHECK(camera_calls==CAR_LINK_WIN_SENDS && !link_slow_giveup && !warnings,
              "healthy original image window was throttled or paused");
    }
    else
    {
        test_tick=65530;latency_ticks=20;
        CHECK(car_send_image_once(),"normal synchronous send failed across tick wrap");
        test_tick=65530;latency_ticks=40;
        CHECK(!car_send_image_once(),"slow synchronous send escaped classification across tick wrap");
    }
    printf("PASS restored original transfer mode=%d camera_calls=%d\n",mode,camera_calls);return 0;
}

