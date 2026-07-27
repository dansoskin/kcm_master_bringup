#ifndef ROULETTE_SM_H
#define ROULETTE_SM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Roulette: the stepper is a master azimuth axis and the two ODrives follow it
 * with streamed position setpoints (CSP), so the plate holds a constant tilt
 * while its high point orbits.
 *
 * The stepper's conversion is 3200/360 steps per degree of azimuth, so one motor
 * revolution is exactly one plate orbit.
 *
 * Tilt comes from the roulette_tilt_deg global and speed from the stepper's own
 * setSpeed; both are read once at start and are fixed for the run.
 *
 * ARMING exists so the PASSTHROUGH mode-switch frames land before the first
 * setpoint. STOPPING exists because deceleration takes seconds: the drives
 * cannot be put back into TRAP_TRAJ the moment a stop is requested, since the
 * stream is still running. */
typedef enum roulette_sm_states
{
    ROULETTE_IDLE,          /* drives in TRAP_TRAJ, nothing streaming        */
    ROULETTE_ARMING,        /* logging muted, PASSTHROUGH sent, settling     */
    ROULETTE_ACCELERATING,  /* streaming; master ramping toward target speed */
    ROULETTE_SPINNING,      /* streaming; master at speed                    */
    ROULETTE_STOPPING,      /* decelerating, still streaming                 */
    ROULETTE_FAULT          /* a drive left closed-loop control              */
} roulette_sm_states;

void set_roulette_sm_state(roulette_sm_states st);
roulette_sm_states get_roulette_sm_state(void);
void start_roulette_sm(void);
void stop_roulette_sm(void);
void roulette_sm_loop(void);

/* Read once at start and fixed for the run. All three are range-checked by
 * start_roulette_sm(), which refuses and says why rather than streaming a bad
 * setpoint -- PASSTHROUGH has no ramp to soften one. */
extern float roulette_tilt_deg;           /* held tilt, within PLATE_MAX_TILT_DEG */
extern float roulette_speed_deg_per_sec;  /* cruise azimuth speed, positive      */
extern float roulette_total_deg;          /* whole sequence; 360 is one orbit,
                                           * negative spins the other way        */

#ifdef __cplusplus
}
#endif

#endif /* ROULETTE_SM_H */
