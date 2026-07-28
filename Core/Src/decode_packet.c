#include "decode_packet.h"
#include "odrive.h"
#include "uart.h"
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>

#include "globals.h"
#include "plate_kinematics.h"
#include "roulette_sm.h"


void decode_stepper(char* packet, int len);
void decode_odrive_commands(char* packet, int len);
void decode_sync_movement(char* packet, int len);
static void sync_move_plate(float x_target_deg, float y_target_deg, const char* tag);

void decode_uart(char* packet, char* response, int len)
{
    if(len < 1)
        return;

    send_uart(&myUart, "UART<< %s\n", packet);

    char result[10][20] = {0};	//10 values of 20 characters
    int count = split_csv_string(packet+1, result, ",\n");
    (void) count;

    switch(packet[0])
    {
        case '~':
        	NVIC_SystemReset();
            break;

        case '@':
            decode_stepper(packet + 1, len - 1);
            break;

        case '#':
            decode_odrive_commands(packet + 1, len - 1);
            break;
        
        case '$':
            decode_sync_movement(packet + 1, len - 1);
            break;

        case 'p':
        	send_uart(&myUart, "Pong\r\n");
        	break;

        case 'R':   
        {
            float total = atof(result[0]);
            if (total == 0.0f)
            {
                stop_roulette_sm();
                break;
            }

            start_roulette_sm(total);
        }
            break;
        
        case 'T':
        {
            float deg = atof(result[0]);
            roulette_set_tilt(deg);
        }
            break;

        case 'S':   /* orbit speed in deg/s. Prefer this over '@S' while spinning:
                     * it keeps the state machine's at-speed tracking honest. */
        {
            float deg_per_sec = atof(result[0]);
            roulette_set_speed(deg_per_sec);
        }
            break;

        case 'k':
        case 'K':
            FlexyStepper_Estop(&stepper, 1);
            estop_roulette_sm();
            odrive_estop(&odrv0);
            odrive_estop(&odrv1);
            break;

        default:
            send_uart(&myUart, "Unknown command: %s\n", packet);
            break;
    }
}

/* Drive both gimbal motors to absolute targets so they arrive together.
 *
 * The odrv <-> SyncAxis mapping lives here and ONLY here. The globals are named
 * for lean direction: x_axis leans the plate toward X and its shaft lies along
 * Y (odrv1); y_axis leans toward Y and its shaft lies along X (odrv0).
 * tag prefixes the log lines so the caller ("M" / "G") is identifiable. */
static void sync_move_plate(float x_target_deg, float y_target_deg, const char* tag)
{
    /* pos_estimate only means anything once an encoder frame has been decoded;
     * before that it reads 0 and every distance is wrong. */
    if (!odrv0.pos_valid || !odrv1.pos_valid)
    {
        send_uart(&myUart, "%s: no encoder feedback yet (odrv0=%d odrv1=%d)\n",
                  tag, odrv0.pos_valid, odrv1.pos_valid);
        return;
    }

    x_axis.current_position = odrv1.feedback.pos_estimate;
    y_axis.current_position = odrv0.feedback.pos_estimate;
    x_axis.target_position  = x_target_deg;
    y_axis.target_position  = y_target_deg;

    SyncAxis* axes[] = {&x_axis, &y_axis};
    if (!calculate_speeds_for_synchronized_movement(axes, 2))
    {
        send_uart(&myUart, "%s: solve failed (bad accel / max_speed / position)\n", tag);
        return;
    }

    /* Speed limit must land before input_pos: the ODrive plans its trapezoid
     * when the position command arrives, using whatever traj vel limit it holds
     * at that instant. An axis with zero distance gets no new limit -- 0 would
     * stall it on its next move. */
    if (x_axis.sync_speed > 0.0f)
        odrive_set_traj_vel_limit(&odrv1, x_axis.sync_speed);
    if (y_axis.sync_speed > 0.0f)
        odrive_set_traj_vel_limit(&odrv0, y_axis.sync_speed);

    odrive_set_input_pos(&odrv1, x_axis.target_position, 0, 0);
    odrive_set_input_pos(&odrv0, y_axis.target_position, 0, 0);

    send_uart(&myUart, "%s: x %.2f->%.2f @%.2f | y %.2f->%.2f @%.2f | t=%.3fs\n",
              tag,
              x_axis.current_position, x_axis.target_position, x_axis.sync_speed,
              y_axis.current_position, y_axis.target_position, y_axis.sync_speed,
              x_axis._time);
}


void decode_stepper(char* packet, int len)
{
    if(len < 1)
        return;

    char result[10][20] = {0};	//10 values of 20 characters
    int count = split_csv_string(packet+1, result, ",");
    UNUSED(count);

    switch(packet[0])
	{
        case 'S':
			FlexyStepper_setSpeed(&stepper, atof(result[0]));
		    break;

        case 'A':   /* also resets deceleration to match -- send @D after, not before */
            FlexyStepper_setAcceleration(&stepper, atof(result[0]));
            break;

        case 'D':
            FlexyStepper_setDeceleration(&stepper, atof(result[0]));
            break;

        case 'R':
            FlexyStepper_setTargetPositionRelative(&stepper, atof(result[0]), true);
            break;

        // case 'A':
        //     FlexyStepper_setTargetPosition(&stepper, atof(result[0]), true);
        //     break;

        case 'J':
            FlexyStepper_jog(&stepper, atof(result[0]));
            break;

        default:
            send_uart(&myUart, "Unknown stepper command: %s\n", packet);
            break;
    }
}


void decode_odrive_commands(char* packet, int len)
{
    if(len < 1)
        return;

    char result[10][20] = {0};	//10 values of 20 characters
    int count = split_csv_string(packet+1, result, ",\n");
    (void) count;

    int node = atoi(result[0]);
    odrive_t * odrv = NULL;
    if(node == 1)
        odrv = &odrv0;
    else if(node == 2)
        odrv = &odrv1;


    switch(packet[0])
	{
        case 'O':   /* encoder offset calibration */
            odrive_set_axis_state(odrv, ODRIVE_AXIS_STATE_ENCODER_OFFSET_CALIB);
            break;

        case 'C':   /* clear error */
            odrive_clear_errors(odrv);
            break;

        case 'E':   /* closed loop on/off*/
            odrive_set_closed_loop(odrv, atoi(result[1]));
            break;

        case 'B':   /* reboot */
            odrive_reboot(odrv, ODRIVE_REBOOT_REBOOT);
            break;

        case 'Z':
        {
            float arg = atof(result[1]);
            odrive_set_absolute_position(odrv, arg);
        }
            break;

        case 'p':
            send_uart(&myUart, "odrv%d: absolute_position %.2f\n", node, odrv ? odrv->feedback.pos_estimate : 0.0f);
            break;
        
        case 'A':
        {
            float arg = atof(result[1]);
            odrive_set_input_pos(odrv, arg, 0, 0);
        }
            break;

        case 'R':
        {
            float arg = atof(result[1]);
            odrive_set_relative_pos(odrv, arg);
        }
        break;
        
        case 'S':
        {
            float arg = atof(result[1]);
            odrive_set_traj_vel_limit(odrv, arg);
        }
            break;

        case 'J':
        {
            float arg = atof(result[1]);
            odrive_set_traj_accel_limits(odrv, arg, arg);
        }
            break;

        case 's':   /* heartbeat as text */
        {
            char hb[160];
            /* A NULL odrv (unknown node) formats as "no heartbeat". */
            odrive_heartbeat_str(odrv ? &odrv->feedback.hb : NULL, hb, sizeof(hb));
            send_uart(&myUart, "odrv%d: %s\n", node, hb);
        }
            break;


        default:
            send_uart(&myUart, "Unknown odrive command: %s\n", packet);
            break;
    }
}

void decode_sync_movement(char* packet, int len)
{
    if(len < 1)
        return;

    char result[10][20] = {0};	//10 values of 20 characters
    int count = split_csv_string(packet+1, result, ",\n");
    (void) count;

    switch(packet[0])
	{

        case 'S':
        {
            float spd = atof(result[0]) * odrv0.turns_per_unit;
            x_axis.max_speed = spd;
            y_axis.max_speed = spd;
        }
            break;

        case 'A':
        {
            float acl = atof(result[0]) * odrv0.turns_per_unit;
            x_axis.acceleration = acl;
            y_axis.acceleration = acl;
        }
            break;

        case 'M':   /* M<x_target>,<y_target> -- raw per-motor targets, synchronized arrival */
            /* Without this, a bare "M" leaves result[] empty, atof() yields 0
             * and both axes are commanded to absolute zero. */
            if (count < 2)
            {
                send_uart(&myUart, "Usage: M<x_target>,<y_target>\n");
                break;
            }
            sync_move_plate(atof(result[0]), atof(result[1]), "M");
            break;

        case 'G':   /* G<azimuth>,<tilt> -- plate pose in degrees */
        {
            if (count < 2)
            {
                send_uart(&myUart, "Usage: G<azimuth>,<tilt>\n");
                break;
            }

            float azimuth = atof(result[0]);
            float tilt    = atof(result[1]);
            float x_target, y_target;

            /* Validate the pose before touching the drives, so an out-of-range
             * tilt is refused even when the encoders are not ready yet. */
            plate_status_t ps = plate_angles_from_tilt(azimuth, tilt,
                                                       &x_target, &y_target);
            if (ps == PLATE_ERR_RANGE)
            {
                send_uart(&myUart, "G: tilt %.2f out of range (max %.1f deg)\n",
                          tilt, PLATE_MAX_TILT_DEG);
                break;
            }
            if (ps != PLATE_OK)
            {
                send_uart(&myUart, "G: bad arguments\n");
                break;
            }

            send_uart(&myUart, "G: az %.2f tilt %.2f -> x %.2f y %.2f\n",
                      azimuth, tilt, x_target, y_target);
            sync_move_plate(x_target, y_target, "G");
        }
            break;


        default:
            send_uart(&myUart, "Unknown syncronized movement command: %s\n", packet);
            break;
    }
}