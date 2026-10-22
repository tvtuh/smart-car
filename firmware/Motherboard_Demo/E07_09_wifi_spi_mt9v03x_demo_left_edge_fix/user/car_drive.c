#include "car_drive.h"
#include "car_config.h"

int16 car_drive_start_boost(int16 speed, uint8 output_ready, uint16 *frames)
{
    if((speed <= 0) || (0 == output_ready))
    {
        *frames = 0;
        return speed;
    }
    if(*frames < (uint16)CAR_START_BOOST_FRAMES)
    {
        (*frames)++;
        return (int16)(speed + CAR_START_BOOST_CM_S);
    }
    return speed;
}

void car_drive_mix(int16 speed, float steer, float gain, uint8 normalized,
                   car_drive_output_t *out)
{
    float ratio;
    float left;
    float right;
    float peak;
    float scale;
    int16 diff;

    out->left = 0;
    out->right = 0;
    out->limited = 0;
    if(speed <= 0) { return; }

    if(0 == normalized)
    {
        /* Preserve the old mixer exactly for A/B comparison. */
        diff = (int16)(gain * steer);
        out->left = (int16)(speed + diff);
        out->right = (int16)(speed - diff);
        if(out->left < 0) { out->left = 0; out->limited = 1; }
        if(out->right < 0) { out->right = 0; out->limited = 1; }
        if(out->left > CAR_SPEED_MAX_CM_S)
        {
            out->left = CAR_SPEED_MAX_CM_S;
            out->limited = 1;
        }
        if(out->right > CAR_SPEED_MAX_CM_S)
        {
            out->right = CAR_SPEED_MAX_CM_S;
            out->limited = 1;
        }
        return;
    }

    /* At the reference speed, keep the existing gain's meaning.
     * For the same steering, the wheel-speed ratio stays constant.
     * This is an empirical approximation, not calibrated Ackermann geometry. */
    ratio = gain * steer / CAR_DIFF_REFERENCE_CM_S;
    if(ratio > CAR_DIFF_RATIO_MAX)
    {
        ratio = CAR_DIFF_RATIO_MAX;
        out->limited = 1;
    }
    if(ratio < -CAR_DIFF_RATIO_MAX)
    {
        ratio = -CAR_DIFF_RATIO_MAX;
        out->limited = 1;
    }

    left = (float)speed * (1.0f + ratio);
    right = (float)speed * (1.0f - ratio);
    peak = (left > right) ? left : right;
    if(peak > (float)CAR_SPEED_MAX_CM_S)
    {
        /* Scale both wheels together: clipping only the outside wheel
         * would change the requested wheel-speed ratio. */
        scale = (float)CAR_SPEED_MAX_CM_S / peak;
        left *= scale;
        right *= scale;
        out->limited = 1;
    }

    out->left = (int16)(left + 0.5f);
    out->right = (int16)(right + 0.5f);
}

int16 car_drive_corner_limit(int16 speed, int16 curve, float steer, uint8 enabled)
{
    float bend;
    float preview;
    float magnitude;

    if((0 == enabled) || (speed <= CAR_TURN_SPEED_CAP_CM_S)) { return speed; }
    magnitude = (steer < 0.0f) ? -steer : steer;
    bend = (magnitude - CAR_TURN_STEER_DEG) /
           (CAR_TURN_STEER_FULL_DEG - CAR_TURN_STEER_DEG);
    magnitude = (curve < 0) ? -(float)curve : (float)curve;
    preview = (magnitude - (float)CAR_TURN_CURVE_PX) /
              (float)(CAR_TURN_CURVE_FULL_PX - CAR_TURN_CURVE_PX);
    if(preview > bend) { bend = preview; }
    if(bend <= 0.0f) { return speed; }
    if(bend >= 1.0f) { return CAR_TURN_SPEED_CAP_CM_S; }

    /* Blend into the cap, avoiding a speed step at the turn threshold. */
    return (int16)((float)speed -
                   (float)(speed - CAR_TURN_SPEED_CAP_CM_S) * bend + 0.5f);
}
