#ifndef __CAR_DRIVE_H
#define __CAR_DRIVE_H

#include "zf_common_headfile.h"

typedef struct
{
    int16 left;
    int16 right;
    uint8 limited;
} car_drive_output_t;

/* Count startup assist only while motor output is permitted. */
int16 car_drive_start_boost(int16 speed, uint8 output_ready, uint16 *frames);

/* Positive steering means a right turn: the left wheel is outside. */
void car_drive_mix(int16 speed, float steer, float gain, uint8 normalized,
                   car_drive_output_t *out);

/* Optional corner cap. Never raises the requested speed. */
int16 car_drive_corner_limit(int16 speed, int16 curve, float steer, uint8 enabled);

#endif
