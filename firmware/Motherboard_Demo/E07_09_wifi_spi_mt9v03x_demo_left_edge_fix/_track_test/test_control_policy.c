/* Current control policy: original frame fallback, no 300ms hard-stop. */
#include "zf_common_headfile.h"
#define PWME_CH4P_PA6 0
#define UART_8 8
#define UART8_TX_P93 93
#define UART8_RX_P92 92
void system_delay_ms(uint16 ms);
void uart_init(int uart, int baud, int tx, int rx);
uint8 uart_query_byte(int uart, uint8 *value);
void uart_write_string(int uart, const char *value);
#include "car_config.h"
#include "image_process.h"
#undef CAR_DEBUG_PRINT
#define CAR_DEBUG_PRINT 0
#include "../user/car_control.c"

uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
volatile uint8 mt9v03x_finish_flag = 0;
static uint8 test_off = 0;
static int test_duty_l = 1600, test_duty_r = 1600;
static int test_target_l = 38, test_target_r = 38;
static int test_stops = 0;
static uint8 test_image[VISION_IMAGE_SIZE];
static uint8 test_command = 0;
static int test_stop_message = 0;
static uint16 test_tick = 100;
static float test_servo = 0.0f;
static unsigned test_uart_bytes = 0;
static char test_uart_last[300];
static char test_tune_last[300];
static int test_run_calls = 0;
static float test_slew = CAR_SERVO_SLEW_DEG_PER_10MS;
void uart_init(int uart, int baud, int tx, int rx) { (void)uart; (void)baud; (void)tx; (void)rx; }
uint8 uart_query_byte(int uart, uint8 *value)
{ (void)uart; if(!test_command) { return 0; } *value = test_command; test_command = 0; return 1; }
void uart_write_string(int uart, const char *value)
{ (void)uart;test_uart_bytes+=(unsigned)strlen(value);strncpy(test_uart_last,value,sizeof(test_uart_last)-1); if(!strncmp(value,"DRV TUNE ",9)){strncpy(test_tune_last,value,sizeof(test_tune_last)-1);} if(strstr(value, "STOP: control line lost")) { test_stop_message++; } }
void motor_init(void) {}
void motor_run(uint8 enable) { (void)enable; test_run_calls++; }
void motor_test_duty(int16 l, int16 r) { (void)l; (void)r; }
int16 motor_enc_raw_left(void) { return 0; }
int16 motor_enc_raw_right(void) { return 0; }
uint8 motor_enc_inv_left(void) { return 0; }
uint8 motor_enc_inv_right(void) { return 0; }
void motor_enc_inv_set_left(uint8 on) { (void)on; }
void motor_enc_inv_set_right(uint8 on) { (void)on; }
uint8 motor_enc_mode_get(void) { return 0; }
void motor_enc_mode_set(uint8 mode) { (void)mode; }
uint8 motor_enc_socket_get(void) { return 0; }
void motor_enc_socket_set(uint8 socket) { (void)socket; }
void motor_enc_left_socket_set(uint8 socket) { (void)socket; }
int16 motor_speed_left(void) { return 0; }
int16 motor_speed_right(void) { return 0; }
int16 motor_enc_left(void) { return 0; }
int16 motor_enc_right(void) { return 0; }
int16 motor_duty_left(void) { return (int16)test_duty_l; }
int16 motor_duty_right(void) { return (int16)test_duty_r; }
void motor_pwm_debug_get(motor_pwm_debug_t *state) { memset(state, 0, sizeof(*state)); }
void motor_heartbeat(void) {}
uint16 motor_tick_get(void) { return test_tick; }
uint8 motor_is_fault(void) { return 0; }
uint8 motor_fault_reason(void) { return 0; }
uint8 motor_output_off_get(void) { return test_off; }
void motor_output_off(uint8 off) { test_off = off; }
void motor_stop(void)
{ test_stops++; test_duty_l = test_duty_r = test_target_l = test_target_r = 0; }
void motor_set_target(int16 l, int16 r) { test_target_l = l; test_target_r = r; }
void servo_set_steer(float steer) { test_servo = steer; }
void servo_init(void) {}
void servo_set_soft_limit(float deg) { (void)deg; }
void servo_set_offset(float deg) { (void)deg; }
void servo_nudge(float deg) { (void)deg; }
float servo_get_offset(void) { return 0.0f; }
void servo_set_slew(float deg) { test_slew=deg; }
float servo_get_slew(void) { return test_slew; }
float servo_get_steer(void) { return test_servo; }
void servo_center(void) {}
void system_delay_ms(uint16 ms) { (void)ms; }

static void lane(int valid)
{
    int y;
    for(y = 0; y < VISION_H; ++y)
    {
        track_left[y] = valid ? 20 : 255;
        track_right[y] = valid ? 74 : 255;
        track_center[y] = valid ? 47 : 255;
        track_lost[y] = valid ? 0 : 3;
    }
}

#define CHECK(condition, message) do { if(!(condition)) { fputs(message "\n",stderr); return 1; } } while(0)
static void distant_bend(int last_row, int shift)
{
    int y;
    lane(1);
    for(y=0;y<=last_row;y++)
    {track_left[y]+=shift;track_right[y]+=shift;track_center[y]+=shift;}
}
static void command(uint8 c) {test_command=c;car_wireless_poll();}
static int preview_case(int mode)
{
    int i;
    if(mode==24 || mode==25)
    {
        distant_bend(22,(mode==24)?6:-6);
        track_line_smooth();track_temporal_filter();car_control_update();
        printf("distant first-frame angle=%.2f\n",test_servo);
        CHECK((mode==24)?test_servo>0.4f:test_servo< -0.4f,"FAIL distant bend waited to reach the old preview row");
    }
    else if(mode==26)
    {
        for(i=0;i<5;i++)command('p');
        distant_bend(17,6);car_control_update();
        CHECK(test_servo>0.4f,"FAIL serial earlier-preview command did not affect next control frame");
        for(i=0;i<10;i++)command('P');
        distant_bend(17,6);car_control_update();
        CHECK(test_servo==0.0f,"FAIL later preview did not take effect cleanly (derivative kick)");
    }
    else if(mode==27)
    {
        mt9v03x_finish_flag=1;test_uart_bytes=0;command('p');
        CHECK(test_uart_bytes==0,"FAIL tuning acknowledgement delayed pending camera control");
        distant_bend(19,6);car_control_update();
        CHECK(test_servo>0.4f,"FAIL deferred acknowledgement also deferred applying tuning");
        mt9v03x_finish_flag=0;car_wireless_poll();
        CHECK(strstr(test_tune_last,"row=42/29/16"),"FAIL tuning acknowledgement did not resume/show actual rows");
    }
    else if(mode==28)
    {
        mt9v03x_finish_flag=1;
        for(i=0;i<100;i++)command('p');
        mt9v03x_finish_flag=0;command('?');
        CHECK(strstr(test_tune_last,"row=42/20/7"),"FAIL early-preview lower bound");
        mt9v03x_finish_flag=1;
        for(i=0;i<100;i++)command('P');
        mt9v03x_finish_flag=0;command('?');
        CHECK(strstr(test_tune_last,"row=42/40/27"),"FAIL late-preview bound/order");
    }
    else if(mode==29)
    {
        command('p');command('p');command('r');command('?');
        CHECK(strstr(test_tune_last,"row=42/30/17"),"FAIL preview reset did not restore compiled defaults");
        CHECK(!test_off && !test_stops && test_run_calls==0,"FAIL tuning/reset changed motor enable state");
    }
    else if(mode==30)
    {
        lane(1);car_control_update();
        test_run_calls=0;
        command('t');command('c');command('y');command('m');
        CHECK(!test_off && !test_stops && test_run_calls==0,"FAIL retired diagnostics still change vehicle operation");
    }
    else if(mode==31)
    {
        command('g');command('?');
        CHECK(strstr(test_tune_last,"slew100=425"),"FAIL faster slew command/value query");
        command('f');command('?');
        CHECK(strstr(test_tune_last,"slew100=400"),"FAIL slower slew command/value query");
    }
    else if(mode==34)
    {
        mt9v03x_finish_flag=1;test_uart_bytes=0;command('k');
        for(i=0;i<5;i++){test_tick+=40;car_wireless_poll();}
        CHECK(!s_servo_hold && !s_servo_test_on,"FAIL pending camera/tuning reply left servo test holding steering");
        CHECK(test_uart_bytes==0,"FAIL servo test status delayed pending camera control");
        distant_bend(VISION_H-1,6);car_control_update();
        CHECK(test_servo>0.4f,"FAIL vision did not regain steering after servo test");
    }
    else if(mode==32)
    {
        lane(1);track_left[CAR_ROW_FAR]=255;track_right[CAR_ROW_FAR]=255;
        track_center[CAR_ROW_FAR]=255;track_lost[CAR_ROW_FAR]=3;
        car_control_update();
        CHECK(test_servo==0 && !test_off && !test_stops,"FAIL invalid distant row contaminated valid near/mid guidance");
    }
    else
    {
        lane(1);car_control_update();
        CHECK(test_servo==0 && !test_off && !test_stops,"FAIL preview change introduced straight-track steering");
    }
    printf("PASS earlier preview/live tuning mode=%d\n",mode);return 0;
}
static int priority_case(int mode)
{
    int y;
    if(mode==23)
    {
        int side;
        for(side=0;side<2;side++)
        {
            lane(1);
            for(y=0;y<VISION_H;y++){track_center[y]=(uint8)(side?22:72);}
            car_control_update();
            CHECK(test_servo==(side?-18.0f:18.0f),"FAIL steering did not saturate symmetrically at +/-18 degrees");
        }
    }
    else if(mode==21)
    {
        mt9v03x_finish_flag=1;test_tick=1000;
        car_wireless_poll();
        CHECK(test_uart_bytes==0,"FAIL periodic/event UART report ran ahead of queued camera control");
        test_command='o';car_wireless_poll();
        CHECK(test_off,"FAIL pending camera blocked the motor-off command");
        mt9v03x_finish_flag=0;test_uart_bytes=0;test_tick+=100;
        car_wireless_poll();
        CHECK(test_uart_bytes>0,"FAIL deferred report never resumed during idle");
    }
    else
    {
        lane(1);car_control_update();test_uart_bytes=0;
        test_tick+=100;car_wireless_poll();
        lane(0);car_control_update();test_tick+=100;car_wireless_poll();
        CHECK(test_uart_bytes==0,"FAIL automatic diagnostics still transmit while driving");
    }    printf("PASS control priority/diagnostics mode=%d\n",mode);return 0;
}
static int steering_case(int mode)
{
    int frame,row,shift=(mode==19)?6:-6;
    for(frame=0;frame<5;frame++)
    {
        lane(1);
        if(frame==4)
        {
            for(row=0;row<VISION_H;row++)
            {track_left[row]+=shift;track_right[row]+=shift;track_center[row]+=shift;}
        }
        track_line_smooth();track_temporal_filter();car_control_update();test_tick++;
    }
#if TRACK_TEMPORAL_ENABLE
    CHECK(test_servo==0.0f,"FAIL selected temporal median did not hold the first isolated offset");
    lane(1);
    for(row=0;row<VISION_H;row++)
    {track_left[row]+=shift;track_right[row]+=shift;track_center[row]+=shift;}
    track_line_smooth();track_temporal_filter();car_control_update();test_tick++;
#endif
    CHECK((mode==19)?(test_servo>0.4f):(test_servo < -0.4f),
          "FAIL persistent bend did not reach steering under the selected temporal mode");
    CHECK(!test_off && !test_stops,"FAIL valid first bend triggered line stop");
    printf("PASS filtered control steering mode=%d median=%d angle=%.2f\n",mode,TRACK_TEMPORAL_ENABLE,test_servo);return 0;
}

static int legacy_loss_case(int mode)
{
    int i,total=CAR_OPEN_AS_LOST_FRAMES+CAR_LOST_STOP_FRAMES;
    lane(1);car_control_update();
    if(mode==0 || mode==2 || mode==5)
    {
        lane(0);car_control_update();
        test_tick=(uint16)(test_tick+(mode==2?3000:30));car_control_update();
        CHECK(!test_off && !test_stops && test_target_l>0 && test_target_r>0,
              "removed 300ms loss timer still shuts down or latches motor output");
        if(mode==5)
        {
            command('o');CHECK(test_off,"manual output-off command failed");
            command('o');lane(1);car_control_update();
            CHECK(!test_off && test_target_l>0,"manual re-enable failed after timer removal");
        }
    }
    else if(mode==1 || mode==3)
    {
        lane(0);
        for(i=0;i<total;i++){if(mode==1)test_tick++;car_control_update();}
        CHECK(test_target_l>0 && test_target_r>0,"legacy frame-count fallback stopped too early");
        car_control_update();
        CHECK(test_target_l==0 && test_target_r==0 && !test_off && !test_stops && s_started,
              "legacy sustained-loss speed-zero behavior missing or replaced with hard latch");
        lane(1);car_control_update();
        CHECK(test_target_l>0 && test_target_r>0,"legacy line recovery did not resume control");
    }
    else if(mode==4)
    {
        lane(0);for(i=0;i<total-2;i++)car_control_update();
        track_left[CAR_ROW_MID]=20;track_right[CAR_ROW_MID]=74;
        track_center[CAR_ROW_MID]=47;track_lost[CAR_ROW_MID]=0;
        car_control_update();
        CHECK(!s_open_frames && !s_lost_frames,"one valid control row did not reset frame fallback");
        lane(0);for(i=0;i<total;i++)car_control_update();
        CHECK(test_target_l>0,"separate loss episodes accumulated frame count");
        car_control_update();CHECK(test_target_l==0 && test_target_r==0,"new loss episode never reached original threshold");
    }
    else if(mode==6)
    {
        lane(1);for(i=0;i<VISION_H;i++)track_lost[i]=3;
        ring_state=RING_STATE_OUT;ring_type=RING_TYPE_LEFT;ring_exit_active=1;ring_exit_row=VISION_H-1;
        for(i=0;i<total+5;i++)car_control_update();
        CHECK(test_target_l>0 && !test_off && !test_stops,"valid ring guidance was counted as full lane loss");
    }
    else
    {
        car_control_stop();lane(1);car_control_update();
        CHECK(!s_started && test_stops && !test_target_l && !test_target_r,
              "explicit control stop or no-start gate was removed");
    }
    printf("PASS original frame-loss policy mode=%d\n",mode);return 0;
}
static int quiet_serial_case(int mode)
{
    int i;
    if(mode==35 || mode==41)
    {
        for(i=0;i<200;i++)
        {
            test_tick=(uint16)(test_tick+51);
            ring_state=(uint8)(i%5);ring_type=(uint8)(i%3);
            ring_exit_active=(uint8)(i%2);test_off=(uint8)(i%2);
            car_wireless_poll();
        }
        CHECK(test_uart_bytes==0,"FAIL idle/ring changes still cause unsolicited serial TX");
    }
    else if(mode==36)
    {
        command('@');
        for(i=0;i<5;i++){test_tick+=100;car_wireless_poll();}
        CHECK(test_uart_bytes==0,"FAIL retired continuous-drive logging command still sends");
    }
    else if(mode==37)
    {
        s_wl_ready=0;car_wireless_link_init();
        CHECK(strstr(test_uart_last,CAR_FW_TAG),"FAIL compact boot version is missing");
        test_uart_bytes=0;car_wireless_link_init();car_wireless_init();
        CHECK(test_uart_bytes==0,"FAIL duplicate init banners still send");
    }
    else if(mode==38)
    {
        car_wireless_progress("WARN camera timeout\r\n");
        CHECK(strstr(test_uart_last,"WARN camera timeout"),"FAIL critical warning was filtered out");
        CHECK(test_uart_bytes<80,"FAIL critical warning exceeds short-message budget");
    }
    else if(mode==39)
    {
        command('?');
        CHECK(strstr(test_uart_last,"DRV TUNE "),"FAIL manual tuning query lost first line");
        car_wireless_poll();
        CHECK(strstr(test_uart_last,"DRV SET "),"FAIL manual tuning query lost settings line");
        test_uart_bytes=0;
        for(i=0;i<20;i++){test_tick+=100;car_wireless_poll();}
        CHECK(test_uart_bytes==0,"FAIL tuning query starts continuous telemetry");
    }
    else
    {
        command('$');
        CHECK(strstr(test_uart_last,"DRV HW "),"FAIL explicit PWM query was removed");
        CHECK(test_uart_bytes<120,"FAIL manual PWM response is too long");
        test_uart_bytes=0;
        for(i=0;i<10;i++){test_tick+=100;car_wireless_poll();}
        CHECK(test_uart_bytes==0,"FAIL PWM query starts continuous telemetry");
    }
    printf("PASS quiet serial policy mode=%d\n",mode);return 0;
}
int main(int argc,char **argv)
{
    int mode;
    if(argc!=2)return 2;mode=atoi(argv[1]);if(mode==41)test_tick=65500;
    image_threshold=100;memset(test_image,220,sizeof(test_image));ring_road_process(test_image);
    s_started=1;s_wl_ready=1;s_wl_motor_on=1;
    if(mode>=35)return quiet_serial_case(mode);
    if(mode>=24)return preview_case(mode);
    if(mode>=21)return priority_case(mode);
    if(mode>=19)return steering_case(mode);
    return legacy_loss_case(mode);
}
