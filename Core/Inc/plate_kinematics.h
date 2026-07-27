#ifndef PLATE_KINEMATICS_H
#define PLATE_KINEMATICS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Gimbal plate kinematics.
 *
 * Two perpendicular motors share one center of rotation and tilt a plate. Both
 * rotation axes are fixed in the base frame and neither motor carries the other,
 * so each motor angle is an independent projection of the plate normal.
 *
 * Frame: Z up, plate level at motor angles (0, 0). Azimuth is measured from +X,
 * counter-clockwise looking down. Everything is in degrees, matching the user
 * units set up by odrive_init(conv = 32.0/360) -- degrees at the output shaft.
 *
 *      odrv1 / x_axis  shaft along Y, leans the plate toward +/-X
 *      odrv0 / y_axis  shaft along X, leans the plate toward +/-Y
 *
 * The SyncAxis globals are named for LEAN DIRECTION, so x_axis takes the angle
 * derived from the normal's x component. */

/* Maximum tilt from level. Requests beyond this are refused outright so a typo
 * cannot drive the plate into a hard stop. Confirm against real travel. */
#define PLATE_MAX_TILT_DEG      (30.0f)

/* Per-axis direction, +1.0f or -1.0f. Real motor direction depends on winding
 * and mounting; settle these with an inclinometer at bring-up.
 *
 * Named for lean direction to match the globals and the output parameters --
 * NOT for the shaft each motor sits on. "about X / about Y" would invert the
 * association, since odrv0's shaft lies along X but it leans toward Y. */
#define PLATE_INVERT_TOWARD_X   (+1.0f)  /* scales out_x_deg -- odrv1 / x_axis */
#define PLATE_INVERT_TOWARD_Y   (+1.0f)  /* scales out_y_deg -- odrv0 / y_axis */

typedef enum {
    PLATE_OK = 0,
    PLATE_ERR_RANGE,    /* tilt exceeds PLATE_MAX_TILT_DEG */
    PLATE_ERR_BAD_ARG   /* NULL out pointer, or non-finite input */
} plate_status_t;

/* Convert a plate pose to the two motor angles.
 *
 *   azimuth_deg  direction of lean, from +X, CCW looking down. Unbounded --
 *                450 behaves as 90.
 *   tilt_deg     lean away from level. Refused above PLATE_MAX_TILT_DEG.
 *   out_x_deg    angle for the motor leaning toward X (odrv1 / x_axis)
 *   out_y_deg    angle for the motor leaning toward Y (odrv0 / y_axis)
 *
 * At tilt 0 the plate is level and azimuth has no effect: every azimuth maps to
 * (0, 0). On any error the outputs are left untouched, so a failed call cannot
 * half-write a target. */
plate_status_t plate_angles_from_tilt(float azimuth_deg, float tilt_deg,
                                      float *out_x_deg, float *out_y_deg);

/* Motor velocities for a plate spinning at constant tilt -- the velocity
 * feedforward for streamed (CSP) position control.
 *
 *   azimuth_rate_deg_s  how fast the azimuth is advancing. Signed; 0 is legal
 *                       and yields zero rates.
 *   out_x_rate_deg_s    velocity for the motor leaning toward X (odrv1)
 *   out_y_rate_deg_s    velocity for the motor leaning toward Y (odrv0)
 *
 * Analytic derivative of plate_angles_from_tilt with respect to azimuth, so it
 * is exact rather than a finite difference. Validates and fails exactly as
 * plate_angles_from_tilt does, leaving the outputs untouched on error. */
plate_status_t plate_rates_from_spin(float azimuth_deg, float tilt_deg,
                                     float azimuth_rate_deg_s,
                                     float *out_x_rate_deg_s,
                                     float *out_y_rate_deg_s);

#ifdef __cplusplus
}
#endif

#endif /* PLATE_KINEMATICS_H */
