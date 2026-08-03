#ifndef ROULETTE_SM_H
#define ROULETTE_SM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum roulette_sm_states
{
        ROULETTE_IDLE, ROULETTE_ARMING, ROULETTE_INITIAL_TILT,
        ROULETTE_ACCELERATING, ROULETTE_SPINNING, ROULETTE_DECELERATING, ROULETTE_BACK_TO_LEVELED,
        ROULETTE_DISARMING, ROULETTE_FAULT   
} roulette_sm_states;  

void set_roulette_sm_state(roulette_sm_states st);
roulette_sm_states get_roulette_sm_state(void);
void start_roulette_sm(float total_deg, float total_tilt);
void stop_roulette_sm(void);    /* R0: ramp down, then tear down            */
void estop_roulette_sm(void);   /* K:  halt now, tear down now, back to IDLE */
void roulette_sm_loop(void);

/* Command the tilt master. Safe to call mid-spin: stepper2 ramps to the new
 * tilt under its own trapezoid and the CSP stream tracks it tick by tick. */
void roulette_set_tilt(float deg);
void roulette_set_speed(float deg_per_sec);  /* use instead of a bare '@S' while spinning */
/* Acceleration and deceleration are not settable: start_roulette_sm derives them
 * from the speed so the plate always reaches it over one spin and stops within a
 * quarter of one. Change the speed and the ramps follow. */


#ifdef __cplusplus
}
#endif

#endif /* ROULETTE_SM_H */
