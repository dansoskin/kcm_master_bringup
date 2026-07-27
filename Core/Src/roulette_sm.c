#include "roulette_sm.h"
#include "globals.h"
#include "plate_kinematics.h"

#include <math.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/* Tuning                                                                      */
/* -------------------------------------------------------------------------- */

#define ROULETTE_TICK_MS          5U      /* 200 Hz setpoint stream            */
#define ROULETTE_ARM_MS           20U     /* let the PASSTHROUGH frames land   */
#define ROULETTE_AT_SPEED_FRAC    0.99f   /* when ACCELERATING becomes SPINNING*/

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

const char *roulette_sm_states_strings[] =
    {"ROULETTE_IDLE", "ROULETTE_ARMING", "ROULETTE_ACCELERATING",
     "ROULETTE_SPINNING", "ROULETTE_STOPPING", "ROULETTE_FAULT"};

roulette_sm_states roulette_sm_state = ROULETTE_IDLE;
uint32_t roulette_sm_timer = 0;

float roulette_tilt_deg = 0.0f;
float roulette_speed_deg_per_sec = 120;
float roulette_total_deg = 3600;    /* azimuth for one whole sequence */

static uint32_t roulette_stream_timer = 0;

void set_roulette_sm_state(roulette_sm_states st)
{
    roulette_sm_state = st;
    send_uart(&myUart, "roulette_sm_state: %s\n",
              roulette_sm_states_strings[roulette_sm_state]);
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

/* Put the drives back the way the sync moves expect to find them. */
static void roulette_teardown(void)
{
    odrive_set_controller_mode(&odrv0, ODRIVE_CONTROL_MODE_POSITION,
                                       ODRIVE_INPUT_MODE_TRAP_TRAJ);
    odrive_set_controller_mode(&odrv1, ODRIVE_CONTROL_MODE_POSITION,
                                       ODRIVE_INPUT_MODE_TRAP_TRAJ);

    /* A previous synchronized move may have left a reduced traj vel limit
     * behind; restore full speed so the next one plans correctly. */
    odrive_set_traj_vel_limit(&odrv0, y_axis.max_speed);
    odrive_set_traj_vel_limit(&odrv1, x_axis.max_speed);

    odrive_enable_logging(&odrv0, true);
    odrive_enable_logging(&odrv1, true);
}

static void roulette_enter_fault(void)
{
    FlexyStepper_Estop(&stepper, false);
    roulette_teardown();
    send_uart(&myUart, "roulette: drive left closed loop (odrv0 st=%u odrv1 st=%u)\n",
              odrv0.feedback.hb.axis_state, odrv1.feedback.hb.axis_state);
    set_roulette_sm_state(ROULETTE_FAULT);
}

/* The sequence ends by itself: the stepper's own trapezoid decelerates into the
 * target, so there is nothing to command -- only to notice. */
static bool roulette_finish_if_done(void)
{
    if (!FlexyStepper_motionComplete(&stepper))
        return false;

    roulette_teardown();
    send_uart(&myUart, "roulette: done, resting at azimuth %.2f\n",
              fmodf(FlexyStepper_getCurrentPosition(&stepper), 360.0f));
    set_roulette_sm_state(ROULETTE_IDLE);
    return true;
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
        return;     /* unreachable: the tilt was range-checked at start */

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

void start_roulette_sm(void)
{
    if (roulette_sm_state != ROULETTE_IDLE && roulette_sm_state != ROULETTE_FAULT)
    {
        send_uart(&myUart, "roulette: already running (%s)\n",
                  roulette_sm_states_strings[roulette_sm_state]);
        return;
    }

    /* Checked here because this is the only gate between the globals and the
     * stream, and PASSTHROUGH has no ramp to soften a bad setpoint. */
    if (!isfinite(roulette_tilt_deg) || fabsf(roulette_tilt_deg) > PLATE_MAX_TILT_DEG)
    {
        send_uart(&myUart, "roulette: tilt %.2f out of range (max %.1f deg)\n",
                  roulette_tilt_deg, PLATE_MAX_TILT_DEG);
        return;
    }

    if (!isfinite(roulette_speed_deg_per_sec) || roulette_speed_deg_per_sec <= 0.0f)
    {
        send_uart(&myUart, "roulette: speed %.2f must be positive\n",
                  roulette_speed_deg_per_sec);
        return;
    }

    if (!isfinite(roulette_total_deg) || roulette_total_deg == 0.0f)
    {
        send_uart(&myUart, "roulette: total %.2f deg is not a sequence\n",
                  roulette_total_deg);
        return;
    }

    if (!odrv0.pos_valid || !odrv1.pos_valid)
    {
        send_uart(&myUart, "roulette: no encoder feedback yet (odrv0=%d odrv1=%d)\n",
                  odrv0.pos_valid, odrv1.pos_valid);
        return;
    }

    if (!roulette_drives_ok())
    {
        send_uart(&myUart, "roulette: drives not in closed loop (odrv0 st=%u odrv1 st=%u)\n",
                  odrv0.feedback.hb.axis_state, odrv1.feedback.hb.axis_state);
        return;
    }

    /* Position carries over between sequences and would otherwise creep; float32
     * degrees lose resolution as the number grows (about 0.06 deg by the time it
     * reaches 1e6). Dropping whole turns preserves the azimuth phase, so this
     * changes the bookkeeping and not the plate. */
    FlexyStepper_setCurrentPosition(&stepper,
            fmodf(FlexyStepper_getCurrentPosition(&stepper), 360.0f));

    /* Speed is a magnitude; the sign of the total sets the direction. */
    FlexyStepper_setSpeed(&stepper, roulette_speed_deg_per_sec);

    /* odrive_set_input_pos logs on every call. At 200 Hz across two drives that
     * is 400 UART lines a second, which would drown the console and starve the
     * main loop that generates the step pulses. */
    odrive_enable_logging(&odrv0, false);
    odrive_enable_logging(&odrv1, false);

    odrive_set_controller_mode(&odrv0, ODRIVE_CONTROL_MODE_POSITION,
                                       ODRIVE_INPUT_MODE_PASSTHROUGH);
    odrive_set_controller_mode(&odrv1, ODRIVE_CONTROL_MODE_POSITION,
                                       ODRIVE_INPUT_MODE_PASSTHROUGH);

    send_uart(&myUart, "roulette: %.1f deg (%.2f orbits) at %.1f deg/s, tilt %.2f\n",
              roulette_total_deg, roulette_total_deg / 360.0f,
              roulette_speed_deg_per_sec, roulette_tilt_deg);

    set_roulette_sm_state(ROULETTE_ARMING);
}

void stop_roulette_sm(void)
{
    switch (roulette_sm_state)
    {
        case ROULETTE_IDLE:
        case ROULETTE_STOPPING:
            return;

        case ROULETTE_FAULT:
            /* teardown already ran when the fault was raised */
            set_roulette_sm_state(ROULETTE_IDLE);
            return;

        case ROULETTE_ARMING:
            /* never started moving, so there is nothing to decelerate */
            roulette_teardown();
            set_roulette_sm_state(ROULETTE_IDLE);
            return;

        default:
            break;
    }

    /* setTargetPositionToStop derives its decel distance from the current step
     * period, so it is only meaningful while actually stepping. */
    if (FlexyStepper_getCurrentVelocity(&stepper) == 0.0f)
    {
        FlexyStepper_Estop(&stepper, false);
        roulette_teardown();
        set_roulette_sm_state(ROULETTE_IDLE);
        return;
    }

    FlexyStepper_setTargetPositionToStop(&stepper);
    set_roulette_sm_state(ROULETTE_STOPPING);
}

void roulette_sm_loop(void)
{
    switch (roulette_sm_state)
    {
        case ROULETTE_IDLE:
            break;

        case ROULETTE_ARMING:
            /* Hold off the first setpoint until the mode-switch frames have
             * landed, or the drive plans a trajectory with it instead. The
             * whole sequence is issued as one relative move: the stepper's own
             * trapezoid then handles ramp up, cruise and ramp down. */
            if (HAL_GetTick() - roulette_sm_timer >= ROULETTE_ARM_MS)
            {
                FlexyStepper_setTargetPositionRelative(&stepper, roulette_total_deg, false);
                set_roulette_sm_state(ROULETTE_ACCELERATING);
            }
            break;

        case ROULETTE_ACCELERATING:
            if (!roulette_drives_ok())
            {
                roulette_enter_fault();
                break;
            }
            roulette_stream();
            if (roulette_finish_if_done())
                break;      /* a short sequence can end before reaching speed */
            if (fabsf(FlexyStepper_getCurrentVelocity(&stepper)) >=
                roulette_speed_deg_per_sec * ROULETTE_AT_SPEED_FRAC)
            {
                set_roulette_sm_state(ROULETTE_SPINNING);
            }
            break;

        case ROULETTE_SPINNING:
            /* Covers cruise and the trapezoid's own ramp-down at the end; the
             * transition back to IDLE is what marks the sequence complete. */
            if (!roulette_drives_ok())
            {
                roulette_enter_fault();
                break;
            }
            roulette_stream();
            roulette_finish_if_done();
            break;

        case ROULETTE_STOPPING:
            /* Aborted early. Keep streaming through the ramp-down; the drives
             * stay in PASSTHROUGH until the master has actually stopped. */
            roulette_stream();
            roulette_finish_if_done();
            break;

        case ROULETTE_FAULT:
            break;
    }
}
