#include "decode_packet.h"
#include "odrive.h"
#include "uart.h"
#include <string.h>
#include <stdlib.h>
#include <sys/cdefs.h>

#include "globals.h"


void decode_stepper(char* packet, int len);
void decode_odrive_commands(char* packet, int len);

void decode_uart(char* packet, char* response, int len)
{
    if(len < 1)
        return;

    send_uart(&myUart, "UART<< %s\n", packet);

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

        case 'p':
        	send_uart(&myUart, "Pong\r\n");
        	break;

        case 'k':
        case 'K':
            FlexyStepper_Estop(&stepper);
            odrive_estop(&odrv0);
            odrive_estop(&odrv1);
            break;

        default:
            send_uart(&myUart, "Unknown command: %s\n", packet);
            break;
    }
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

        case 'A':
            FlexyStepper_setAcceleration(&stepper, atof(result[0]));
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

        default:
            send_uart(&myUart, "Unknown odrive command: %s\n", packet);
            break;
    }
}
