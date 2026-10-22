/* Real servo driver; only timer/PWM hardware is substituted. */
#include "zf_common_headfile.h"
#define PWME_CH4P_PA6 0
#define PWM_DUTY_MAX 10000
static uint16 test_tick=100;
static uint32 last_duty;
uint16 motor_tick_get(void){return test_tick;}
void pwm_init(int pin,int freq,int duty){(void)pin;(void)freq;(void)duty;}
void pwm_set_duty(int pin,uint32 duty){(void)pin;last_duty=duty;}
#include "../user/car_servo.c"
#define CHECK(c,m) do{if(!(c)){fprintf(stderr,"FAIL %s: steer=%.2f duty=%lu\n",m,servo_get_steer(),(unsigned long)last_duty);return 1;}}while(0)
int main(int argc,char **argv)
{
    int mode;
    if(argc!=2)return 2;
    mode=atoi(argv[1]);servo_init();servo_set_steer(0);
    if(mode==0 || mode==1)
    {
        test_tick+=10;servo_set_steer(mode? -18:18);
        CHECK(servo_get_steer()==(mode? -18:18),"low-FPS turn requires another image update");
    }
    else if(mode==2)
    {
        test_tick+=10;servo_set_steer(18);test_tick+=10;servo_set_steer(-18);
        CHECK(servo_get_steer()==-18,"low-FPS reversal waits extra image frames");
    }
    else if(mode==3)
    {
        test_tick++;servo_set_steer(18);CHECK(servo_get_steer()==4,"10ms time-based slew changed");
        servo_set_steer(18);CHECK(servo_get_steer()==4,"same tick consumed slew budget twice");
    }
    else if(mode==4)
    {
        test_tick+=100;servo_set_steer(1000);CHECK(servo_get_steer()==18,"right hard limit");
        test_tick+=100;servo_set_steer(-1000);CHECK(servo_get_steer()==-18,"left hard limit");
    }
    else if(mode==5)
    {
        test_tick=65535;servo_set_steer(0);test_tick=0;servo_set_steer(-18);
        CHECK(servo_get_steer()==-4,"time-wrap slew interval");
    }
    else
    {
        servo_set_slew(5);test_tick++;servo_set_steer(18);
        CHECK(servo_get_steer()==5,"runtime slew adjustment ineffective");
    }
    printf("PASS servo response mode=%d steer=%.2f\n",mode,servo_get_steer());return 0;
}
