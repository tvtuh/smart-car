#ifndef VISION_GEOMETRY_H
#define VISION_GEOMETRY_H

#include "zf_common_headfile.h"

/* The shipped DMA library owns the fixed 188x120 capture buffer. Downsample
 * the completed frame directly into a smaller, independent processing image.
 * Set this to 1 to restore full-size processing without changing DMA layout. */
#ifndef VISION_DOWNSAMPLE
#define VISION_DOWNSAMPLE (2)
#endif
#if VISION_DOWNSAMPLE != 1 && VISION_DOWNSAMPLE != 2
#error VISION_DOWNSAMPLE must be 1 or 2
#endif
#if (MT9V03X_W % VISION_DOWNSAMPLE) || (MT9V03X_H % VISION_DOWNSAMPLE)
#error Camera dimensions must be divisible by VISION_DOWNSAMPLE
#endif

#define VISION_W (MT9V03X_W / VISION_DOWNSAMPLE)
/* Ignore the bottom processing row obscured by the steering servo.
 * Capture/DMA stays 188x120; at 2x downsampling the valid ROI is 94x59. */
#define VISION_BOTTOM_SKIP_ROWS (1)
#define VISION_SOURCE_H (MT9V03X_H - VISION_BOTTOM_SKIP_ROWS * VISION_DOWNSAMPLE)
#define VISION_H (VISION_SOURCE_H / VISION_DOWNSAMPLE)
#define VISION_IMAGE_SIZE (VISION_W * VISION_H)
/* The current wire image is already cropped; all transmitted rows are valid. */
#define VISION_VALID_H (VISION_H)
#define VISION_LAST_ROW (VISION_VALID_H - 1)
#define VISION_RAW_VALID_H (VISION_SOURCE_H)

/* Positive spatial distances and row counts, calibrated at full resolution.
 * Keep slopes (pixels/row), intensity, state IDs and frame/time counts intact. */
#define VISION_PX(n) (((n) + VISION_DOWNSAMPLE - 1) / VISION_DOWNSAMPLE)
#define VISION_ERROR_REF(delta) ((delta) * VISION_DOWNSAMPLE)

#if VISION_W > 254 || VISION_H > 254 || VISION_H < 32
#error Unsupported processing image dimensions
#endif

#endif
