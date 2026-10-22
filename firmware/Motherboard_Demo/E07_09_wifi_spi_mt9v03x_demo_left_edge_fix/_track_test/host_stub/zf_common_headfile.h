#ifndef HOST_ZF_COMMON_HEADFILE_H
#define HOST_ZF_COMMON_HEADFILE_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
#define MT9V03X_W 188
#define MT9V03X_H 120
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern volatile uint8 mt9v03x_finish_flag;
#endif
