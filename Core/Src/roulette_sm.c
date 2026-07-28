#include "roulette_sm.h"
#include "globals.h"
#include "plate_kinematics.h"

#include <math.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Tuning                                                                      */
/* -------------------------------------------------------------------------- */

#define ROULETTE_TICK_MS          5U      /* 200 Hz setpoint stream            */
#define ROULETTE_AT_SPEED_FRAC    0.99f   /* when ACCELERATING becomes SPINNING*/

/* The ramps are derived from the speed rather than configured: reach top speed
 * over exactly one spin, shed it over a quarter of one.
 *      a = v^2 / (2 * 360)        d = v^2 / (2 * 90)
 * Deceleration is therefore always 4x acceleration, whatever the speed. A
 * sequence needs at least 360 + 90 = 450 deg to reach cruise at all; anything
 * shorter is a triangular profile that never gets there. */
#define ROULETTE_ACCEL_DEG      360.0f    /* one spin to reach speed     */
#define ROULETTE_DECEL_DEG       90.0f    /* quarter spin to stop        */

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

const char *roulette_sm_states_strings[] = {
        "ROULETTE_IDLE", "ROULETTE_ARMING", 
        "ROULETTE_ACCELERATING","ROULETTE_SPINNING", "ROULETTE_DECELERATING", 
        "ROULETTE_DISARMING", "ROULETTE_FAULT"
    };

roulette_sm_states roulette_sm_state = ROULETTE_IDLE;
uint32_t roulette_sm_timer = 0;

float roulette_tilt_deg = 10.0f;
float roulette_total_deg = 3600.0f;    /* azimuth for one whole sequence */

static uint32_t roulette_stream_timer = 0;

void set_roulette_sm_state(roulette_sm_states st)
{
    roulette_sm_state = st;
    send_uart(&myUart, "roulette_sm_state: %s\n", roulette_sm_states_strings[roulette_sm_state]);
    roulette_sm_timer = HAL_GetTick();
}

roulette_sm_states get_roulette_sm_state(void)
{
    return roulette_sm_state;
}

/* -------------------------------------------------------------------------- */
/* Helpers                                                                     */
/* -------------------------------------------------------------------------- */

static bool roulette_drives_ok(void)
{
    return odrv0.feedback.hb.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL
        && odrv1.feedback.hb.axis_state == ODRIVE_AXIS_STATE_CLOSED_LOOP_CONTROL;
}

static void set_csp(bool enable)
{
    if (enable)
    {
        odrive_set_controller_mode(&odrv0, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH);
        odrive_set_controller_mode(&odrv1, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_PASSTHROUGH);

        odrive_enable_logging(&odrv0, false);
        odrive_enable_logging(&odrv1, false);
    }
    else    /* Put the drives back the way the sync moves expect to find them. */
    {
        odrive_set_controller_mode(&odrv0, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_TRAP_TRAJ);
        odrive_set_controller_mode(&odrv1, ODRIVE_CONTROL_MODE_POSITION, ODRIVE_INPUT_MODE_TRAP_TRAJ);
        
        odrive_set_traj_vel_limit(&odrv0, y_axis.max_speed);
        odrive_set_traj_vel_limit(&odrv1, x_axis.max_speed);

        odrive_enable_logging(&odrv0, true);
        odrive_enable_logging(&odrv1, true);
    }
}

static void roulette_enter_fault(void)
{
    FlexyStepper_Estop(&stepper, 1);
    set_csp(false);
    send_uart(&myUart, "roulette: drive left closed loop (odrv0 st=%u odrv1 st=%u)\n",
              odrv0.feedback.hb.axis_state, odrv1.feedback.hb.axis_state);
    set_roulette_sm_state(ROULETTE_FAULT);
}

void roulette_set_tilt(float deg)
{
    if (deg < 0.0f || deg > PLATE_MAX_TILT_DEG)
    {
        send_uart(&myUart, "Tilt %.2f out of range [0,%.2f]\n", deg, PLATE_MAX_TILT_DEG);
    }

    roulette_tilt_deg = fminf(fmaxf(deg, 0.0f), PLATE_MAX_TILT_DEG);

    send_uart(&myUart, "Tilt set to %.2f\n", roulette_tilt_deg);
}

void roulette_set_speed(float deg_per_sec)
{
    if (!isfinite(deg_per_sec) || deg_per_sec <= 0.0f)
    {
        /* 0 would also make getTargetSpeed() zero, and the at-speed comparisons
         * can never be satisfied against zero. */
        send_uart(&myUart, "Speed %.2f must be positive\n", deg_per_sec);
        return;
    }

    FlexyStepper_setSpeed(&stepper, deg_per_sec);
    send_uart(&myUart, "Speed set to %.2f deg/s\n", deg_per_sec);

    if (roulette_sm_state == ROULETTE_ACCELERATING ||
        roulette_sm_state == ROULETTE_SPINNING)
    {
        set_roulette_sm_state(ROULETTE_ACCELERATING);
    }
}

/* One 200 Hz setpoint pair. */
static void roulette_stream(void)
{
    if (HAL_GetTick() - roulette_stream_timer < ROULETTE_TICK_MS)
        return;
    roulette_stream_timer = HAL_GetTick();

    float azimuth = fmodf(FlexyStepper_getCurrentPosition(&stepper), 360.0f);
    float rate    = FlexyStepper_getCurrentVelocity(&stepper);   /* signed */

    float tx, ty, vx, vy;

    if (plate_angles_from_tilt(azimuth, roulette_tilt_deg, &tx, &ty) != PLATE_OK)
        return;     

    /* Feedforward from the master's actual velocity, not the configured one, so
     * it stays correct through both the acceleration and the final ramp-down. */
    if (plate_rates_from_spin(azimuth, roulette_tilt_deg, rate, &vx, &vy) != PLATE_OK)
        vx = vy = 0.0f;

    odrive_set_input_pos(&odrv1, tx, vx, 0.0f);
    odrive_set_input_pos(&odrv0, ty, vy, 0.0f);
}

/* -------------------------------------------------------------------------- */
/* State machine                                                               */
/* -------------------------------------------------------------------------- */

void start_roulette_sm(float total_deg)
{
    /* The ramps are derived from the speed below, so a speed of zero would give
     * an acceleration of zero, and 1e6/sqrt(0) makes the step period infinite --
     * the move would never step. roulette_set_speed refuses zero, but a bare
     * '@S0' can still get here. */
    if (!(FlexyStepper_getTargetSpeed(&stepper) > 0.0f))
    {
        send_uart(&myUart, "roulette: stepper speed is 0, set it with S first\n");
        return;
    }

    roulette_total_deg = total_deg;
    FlexyStepper_setCurrentPosition(&stepper,fmodf(FlexyStepper_getCurrentPosition(&stepper), 360.0f)); //make position smaller

    /* Derive both ramps from the speed in force right now, so they always suit
     * it: one spin to reach it, a quarter spin to shed it. Order is
     * load-bearing -- setAcceleration resets deceleration to match, so the
     * deceleration write must come second. Doing this here also makes the
     * roulette immune to a stray '@A' or '@D' between sequences. */
    const float speed = FlexyStepper_getTargetSpeed(&stepper);
    const float accel = (speed * speed) / (2.0f * ROULETTE_ACCEL_DEG);
    const float decel = (speed * speed) / (2.0f * ROULETTE_DECEL_DEG);

    FlexyStepper_setAcceleration(&stepper, accel);
    FlexyStepper_setDeceleration(&stepper, decel);

    send_uart(&myUart, "roulette: %.1f deg at %.1f deg/s, accel %.2f, decel %.2f\n",
              total_deg, speed, accel, decel);

    set_csp(true);
    set_roulette_sm_state(ROULETTE_ARMING);
}

void stop_roulette_sm(void)
{
    if(roulette_sm_state == ROULETTE_IDLE)
        return;

    set_roulette_sm_state(ROULETTE_DECELERATING);
    if (fabsf(FlexyStepper_getCurrentVelocity(&stepper)) > 0.1f)
    {
        FlexyStepper_setTargetPositionToStop(&stepper);
    }
}

void estop_roulette_sm(void)
{
    if(roulette_sm_state == ROULETTE_IDLE)
        return;

    FlexyStepper_Estop(&stepper, 1);
    set_csp(false);
    set_roulette_sm_state(ROULETTE_IDLE);
}

void roulette_sm_loop(void)
{
    /* DISARMING is excluded: teardown has already run by then, and a late fault
     * would park in FAULT instead of completing the normal exit to IDLE. */
    if(roulette_sm_state != ROULETTE_IDLE &&
       roulette_sm_state != ROULETTE_DISARMING &&
       roulette_sm_state != ROULETTE_FAULT &&
       !roulette_drives_ok())
    {
        roulette_enter_fault();
    }

    switch (roulette_sm_state)
    {
        case ROULETTE_IDLE:
            break;

        case ROULETTE_ARMING:
            if (HAL_GetTick() - roulette_sm_timer >= 20)
            {
                FlexyStepper_setTargetPositionRelative(&stepper, roulette_total_deg, false);
                set_roulette_sm_state(ROULETTE_ACCELERATING);
            }
            break;

        case ROULETTE_ACCELERATING:
            roulette_stream();

            if (FlexyStepper_motionComplete(&stepper))
            {
                set_roulette_sm_state(ROULETTE_DECELERATING);
                break;
            }

            if (fabsf(FlexyStepper_getCurrentVelocity(&stepper)) >= FlexyStepper_getTargetSpeed(&stepper) * ROULETTE_AT_SPEED_FRAC)
            {
                set_roulette_sm_state(ROULETTE_SPINNING);
            }
            break;

        case ROULETTE_SPINNING:
            roulette_stream();

            /* Backstop, mirroring ACCELERATING: the at-speed test below is the
             * only other exit, and it can never fire if the target speed is 0.
             * motionComplete does not depend on speed at all. */
            if (FlexyStepper_motionComplete(&stepper))
            {
                set_roulette_sm_state(ROULETTE_DECELERATING);
                break;
            }

            if (fabsf(FlexyStepper_getCurrentVelocity(&stepper)) < FlexyStepper_getTargetSpeed(&stepper) * ROULETTE_AT_SPEED_FRAC)
            {
                set_roulette_sm_state(ROULETTE_DECELERATING);
            }
            break;
        
        case ROULETTE_DECELERATING:
            roulette_stream();
            if (FlexyStepper_getCurrentVelocity(&stepper) == 0.0f)
            {
                FlexyStepper_Estop(&stepper, 1);
                set_csp(false);
                set_roulette_sm_state(ROULETTE_DISARMING);
            }
            break;

        case ROULETTE_DISARMING:
            if (HAL_GetTick() - roulette_sm_timer >= 20)
            {
                set_roulette_sm_state(ROULETTE_IDLE);
            }
            break;

        case ROULETTE_FAULT:
            break;
    }
}
