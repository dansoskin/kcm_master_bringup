#ifndef GLOBALS_H
#define GLOBALS_H

#include "main.h"
#include "uart.h"

#include "cFlexyStepper.h"
#include "odrive.h"
#include "synchronized_movement.h"



extern myUART_t myUart;
extern FlexyStepper stepper;
extern odrive_t odrv0, odrv1;
extern SyncAxis x_axis, y_axis;



#endif