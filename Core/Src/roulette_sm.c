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

#define ROULETTE_ACCEL_DEG      360.0f    /* one spin to reach speed     */
#define ROULETTE_DECEL_DEG       90.0f    /* quarter spin to stop        */

/* -------------------------------------------------------------------------- */
/* State                                                                       */
/* -------------------------------------------------------------------------- */

const char *roulette_sm_states_strings[] = {
        "ROULETTE_IDLE", "ROULETTE_ARMING", "ROULETTE_INITIAL_TILT",
        "ROULETTE_ACCELERATING","ROULETTE_SPINNING", "ROULETTE_DECELERATING", "ROULETTE_BACK_TO_LEVELED",
        "ROULETTE_DISARMING", "ROULETTE_FAULT"
    };

roulette_sm_states roulette_sm_state = ROULETTE_IDLE;
uint32_t roulette_sm_timer = 0;

/* Last COMMANDED tilt. The authoritative value is stepper2's position -- this
 * is only the setpoint the axis is ramping toward, kept for reporting. */
float roulette_dst_tilt = 10.0f;
float roulette_dst_azimuth = 3600.0f;    /* azimuth for one whole sequence */
float roulette_initial_tilt = 10.0f;
uint32_t roulette_initial_tilt_time = 5000;

static uint32_t roulette_stream_timer = 0;
static bool     roulette_pose_bad = false;  /* latch, so the stream cannot spam */

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

static void roulette_halt_masters(void)
{
    FlexyStepper_Estop(&stepper1, 1);
    FlexyStepper_Estop(&stepper2, 1);
}

static void roulette_enter_fault(void)
{
    roulette_halt_masters();
    set_csp(false);
    send_uart(&myUart, "roulette: drive left closed loop (odrv0 st=%u odrv1 st=%u)\n",
              odrv0.feedback.hb.axis_state, odrv1.feedback.hb.axis_state);
    set_roulette_sm_state(ROULETTE_FAULT);
}

void roulette_set_tilt(float deg)
{
    if (!isfinite(deg) || deg < 0.0f || deg > PLATE_MAX_TILT_DEG)
    {
        send_uart(&myUart, "Tilt %.2f out of range [0,%.2f]\n", deg, PLATE_MAX_TILT_DEG);
    }

    roulette_dst_tilt = fminf(fmaxf(deg, 0.0f), PLATE_MAX_TILT_DEG);

    FlexyStepper_setTargetPosition(&stepper2, roulette_dst_tilt, false);

    send_uart(&myUart, "Tilt %.2f -> %.2f\n",
              FlexyStepper_getCurrentPosition(&stepper2), roulette_dst_tilt);
}

void roulette_set_speed(float deg_per_sec)
{
    if (!isfinite(deg_per_sec) || deg_per_sec <= 0.0f)
    {
        send_uart(&myUart, "Speed %.2f must be positive\n", deg_per_sec);
        return;
    }

    FlexyStepper_setSpeed(&stepper1, deg_per_sec);
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

    float azimuth   = fmodf(FlexyStepper_getCurrentPosition(&stepper1), 360.0f);
    float az_rate   = FlexyStepper_getCurrentVelocity(&stepper1);   /* signed */
    float tilt      = FlexyStepper_getCurrentPosition(&stepper2);
    float tilt_rate = FlexyStepper_getCurrentVelocity(&stepper2);   /* signed */

    float tx, ty, vx, vy;

    if (plate_angles_from_tilt(azimuth, tilt, &tx, &ty) != PLATE_OK)
    {
        /* The tilt was a pre-clamped global and could not fail here. It comes
         * off a physical axis now, so an unhomed or miscounted stepper2 can
         * read out of range -- which would otherwise stall the stream in
         * total silence. Latched so it cannot spam at 200 Hz. */
        if (!roulette_pose_bad)
        {
            roulette_pose_bad = true;
            send_uart(&myUart, "roulette: tilt %.2f out of range, stream halted\n", tilt);
        }
        return;
    }
    roulette_pose_bad = false;

    if (plate_rates_from_spin(azimuth, tilt, az_rate, tilt_rate, &vx, &vy) != PLATE_OK)
        vx = vy = 0.0f;

    odrive_set_input_pos(&odrv1, tx, vx, 0.0f);
    odrive_set_input_pos(&odrv0, ty, vy, 0.0f);
}

/* -------------------------------------------------------------------------- */
/* State machine                                                               */
/* -------------------------------------------------------------------------- */

void start_roulette_sm(float total_deg, float total_tilt)
{
    if (!(FlexyStepper_getTargetSpeed(&stepper1) > 0.0f))
    {
        send_uart(&myUart, "roulette: stepper speed is 0, set it with S first\n");
        return;
    }
    
    set_roulette_sm_state(ROULETTE_ARMING);
    //the actual odrive arming
    set_csp(true);

    roulette_dst_azimuth = total_deg;
    roulette_dst_tilt = total_tilt;
    roulette_pose_bad  = false;
    FlexyStepper_setCurrentPosition(&stepper1,fmodf(FlexyStepper_getCurrentPosition(&stepper1), 360.0f)); //make position smaller

    const float speed = FlexyStepper_getTargetSpeed(&stepper1);
    const float accel = (speed * speed) / (2.0f * ROULETTE_ACCEL_DEG);
    const float decel = (speed * speed) / (2.0f * ROULETTE_DECEL_DEG);

    FlexyStepper_setAcceleration(&stepper1, accel);
    FlexyStepper_setDeceleration(&stepper1, decel);

    send_uart(&myUart, "roulette: %.1f deg at %.1f deg/s, accel %.2f, decel %.2f\n",
              total_deg, speed, accel, decel);

}

void stop_roulette_sm(void)
{
    if(roulette_sm_state == ROULETTE_IDLE)
        return;

    set_roulette_sm_state(ROULETTE_DECELERATING);
    if (fabsf(FlexyStepper_getCurrentVelocity(&stepper1)) > 0.1f)
    {
        FlexyStepper_setTargetPositionToStop(&stepper1);
    }
}

void estop_roulette_sm(void)
{
    if(roulette_sm_state == ROULETTE_IDLE)
        return;

    roulette_halt_masters();
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
                FlexyStepper_setTargetPosition(&stepper1, 0, false);
                FlexyStepper_setTargetPosition(&stepper2, roulette_initial_tilt, false);
                set_roulette_sm_state(ROULETTE_INITIAL_TILT);
            }
            break;

        case ROULETTE_INITIAL_TILT:
            roulette_stream();
            if(stepper1.is_moving == false && stepper2.is_moving == false && HAL_GetTick() - roulette_sm_timer >= roulette_initial_tilt_time)
            {
                FlexyStepper_setTargetPositionRelative(&stepper1, roulette_dst_azimuth, false);
                FlexyStepper_setTargetPosition(&stepper2, roulette_dst_tilt, false);
                set_roulette_sm_state(ROULETTE_ACCELERATING);
            }
            break;

        case ROULETTE_ACCELERATING:
            roulette_stream();

            if (FlexyStepper_motionComplete(&stepper1))
            {
                set_roulette_sm_state(ROULETTE_DECELERATING);
                break;
            }

            if (fabsf(FlexyStepper_getCurrentVelocity(&stepper1)) >= FlexyStepper_getTargetSpeed(&stepper1) * ROULETTE_AT_SPEED_FRAC)
            {
                set_roulette_sm_state(ROULETTE_SPINNING);
            }
            break;

        case ROULETTE_SPINNING:
            roulette_stream();

            /* Backstop, mirroring ACCELERATING: the at-speed test below is the
             * only other exit, and it can never fire if the target speed is 0.
             * motionComplete does not depend on speed at all. */
            if (FlexyStepper_motionComplete(&stepper1))
            {
                set_roulette_sm_state(ROULETTE_DECELERATING);
                break;
            }

            if (fabsf(FlexyStepper_getCurrentVelocity(&stepper1)) < FlexyStepper_getTargetSpeed(&stepper1) * ROULETTE_AT_SPEED_FRAC)
            {
                set_roulette_sm_state(ROULETTE_DECELERATING);
            }
            break;
        
        case ROULETTE_DECELERATING:
            roulette_stream();
            if (FlexyStepper_getCurrentVelocity(&stepper1) == 0.0f)
            {
                FlexyStepper_setTargetPosition(&stepper2, 0.0f, false);
                set_roulette_sm_state(ROULETTE_BACK_TO_LEVELED);
            }
            break;

        case ROULETTE_BACK_TO_LEVELED:
            roulette_stream();
            if(stepper1.is_moving == false && stepper2.is_moving == false)
            {
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
