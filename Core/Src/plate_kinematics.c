#include "plate_kinematics.h"
#include <math.h>
#include <stddef.h>

#define PLATE_PI          (3.14159265358979323846f)
#define PLATE_DEG_TO_RAD  (PLATE_PI / 180.0f)
#define PLATE_RAD_TO_DEG  (180.0f / PLATE_PI)

plate_status_t plate_angles_from_tilt(float azimuth_deg, float tilt_deg,
                                      float *out_x_deg, float *out_y_deg)
{
    if (out_x_deg == NULL || out_y_deg == NULL)
        return PLATE_ERR_BAD_ARG;

    /* Finite check must come first: fabsf(NaN) > limit is false, so a NaN would
     * slip past the range test and reach the trig. */
    if (!isfinite(azimuth_deg) || !isfinite(tilt_deg))
        return PLATE_ERR_BAD_ARG;

    if (fabsf(tilt_deg) > PLATE_MAX_TILT_DEG)
        return PLATE_ERR_RANGE;

    const float az = azimuth_deg * PLATE_DEG_TO_RAD;
    const float ti = tilt_deg * PLATE_DEG_TO_RAD;

    /* Plate normal. At tilt 0 this is (0, 0, 1) for any azimuth, which falls
     * out of the maths -- no special case needed. */
    const float lean = sinf(ti);
    const float n_x  = lean * cosf(az);
    const float n_y  = lean * sinf(az);
    const float n_z  = cosf(ti);

    /* Each motor angle is the normal projected into that motor's plane of
     * rotation. The minus on n_y is the right-hand rule, not a typo: rotating
     * about +X sends the normal to (0, -sin a, cos a), while rotating about +Y
     * sends it to (+sin b, 0, cos b).
     *
     * atan2f rather than atanf(tanf(ti) * cosf(az)): algebraically the same,
     * but tanf diverges at 90 degrees and atan2f resolves quadrants for free. */
    *out_x_deg = PLATE_INVERT_TOWARD_X * atan2f( n_x, n_z) * PLATE_RAD_TO_DEG;
    *out_y_deg = PLATE_INVERT_TOWARD_Y * atan2f(-n_y, n_z) * PLATE_RAD_TO_DEG;

    return PLATE_OK;
}

plate_status_t plate_rates_from_spin(float azimuth_deg, float tilt_deg,
                                     float azimuth_rate_deg_s,
                                     float tilt_rate_deg_s,
                                     float *out_x_rate_deg_s,
                                     float *out_y_rate_deg_s)
{
    if (out_x_rate_deg_s == NULL || out_y_rate_deg_s == NULL)
        return PLATE_ERR_BAD_ARG;

    if (!isfinite(azimuth_rate_deg_s) || !isfinite(tilt_rate_deg_s))
        return PLATE_ERR_BAD_ARG;

    /* The motor angles are needed for the cos^2 term below. Computing them via
     * plate_angles_from_tilt rather than inline reuses one validation path and
     * one set of sign conventions -- the two functions cannot drift apart. */
    float x_deg, y_deg;
    plate_status_t st = plate_angles_from_tilt(azimuth_deg, tilt_deg,
                                               &x_deg, &y_deg);
    if (st != PLATE_OK)
        return st;

    const float az     = azimuth_deg * PLATE_DEG_TO_RAD;
    const float ti     = tilt_deg * PLATE_DEG_TO_RAD;
    const float tan_ti = tanf(ti);
    const float cos_ti = cosf(ti);
    const float cx     = cosf(x_deg * PLATE_DEG_TO_RAD);
    const float cy     = cosf(y_deg * PLATE_DEG_TO_RAD);

    /* sec^2(tilt). tilt is bounded to PLATE_MAX_TILT_DEG well below 90, so
     * cos_ti cannot approach zero and this cannot blow up. */
    const float sec2_ti = 1.0f / (cos_ti * cos_ti);

    /* Total derivative of the angle relations. Both azimuth and tilt move now
     * -- the tilt axis ramps in and out during a spin -- so both partials are
     * needed. Dropping the tilt term would leave the feedforward wrong during
     * exactly the ramp the tilt axis exists to smooth.
     *
     *     tan(x) =  tan(tilt) * cos(az)
     *     tan(y) = -tan(tilt) * sin(az)
     *
     * d/dt, with w = azimuth rate and r = tilt rate:
     *     dx/dt = cos^2(x) [  sec^2(tilt) cos(az) r - tan(tilt) sin(az) w ]
     *     dy/dt = cos^2(y) [ -sec^2(tilt) sin(az) r - tan(tilt) cos(az) w ]
     *
     * Every bracketed factor is dimensionless, so rates in deg/s give deg/s
     * out. cos() is even, so taking it of the already-inverted angle is the
     * same as taking it of the true one. */
    *out_x_rate_deg_s = PLATE_INVERT_TOWARD_X * cx * cx *
                        ( sec2_ti * cosf(az) * tilt_rate_deg_s
                        - tan_ti  * sinf(az) * azimuth_rate_deg_s);
    *out_y_rate_deg_s = PLATE_INVERT_TOWARD_Y * cy * cy *
                        (-sec2_ti * sinf(az) * tilt_rate_deg_s
                        - tan_ti  * cosf(az) * azimuth_rate_deg_s);

    return PLATE_OK;
}
