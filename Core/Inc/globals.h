#ifndef GLOBALS_H
#define GLOBALS_H

#include "main.h"
#include "uart.h"

#include "cFlexyStepper.h"
#include "odrive.h"



extern myUART_t myUart;
extern FlexyStepper stepper;
extern odrive_t odrv0, odrv1;



#endif