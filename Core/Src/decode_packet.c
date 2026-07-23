#include "decode_packet.h"
#include "uart.h"

#include <string.h>

#include "cFlexyStepper.h"

extern myUART_t myUart;
extern FlexyStepper stepper;

void decode_stepper(char* packet, int len);

void decode_uart(char* packet, char* response, int len)
{
    if(len < 1)
        return;

    send_uart(&myUart, "UART<< %s\n", packet);
    

    char result[10][20] = {0};	//10 values of 20 characters
    int count = split_csv_string(packet, result, ",\n");
    UNUSED(count);
    
    switch(packet[0])
    {
        case '~':
        	NVIC_SystemReset();
            break;
        
        case '@':
            decode_stepper(packet + 1, len - 1);
            break;

        case 'p':
        	send_uart(&myUart, "Pong\r\n");
        	break;

        case 'k':
        case 'K':
            FlexyStepper_Estop(&stepper);
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