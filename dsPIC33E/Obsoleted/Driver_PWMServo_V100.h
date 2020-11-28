// Author			: Fabian Kung
// Date				: 10 July 2015
// Filename			: Driver_PWMServo_V100.h

#ifndef _DRIVER_PWMSERVO_dsPIC33E_H
#define _DRIVER_PWMSERVO_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

//
// --- PUBLIC VARIABLES ---
//
// RC servo motor driver control variables

extern  UINT16 gunRCServoPW_us[10];                             // Public date. RC servo motor pulse width in usec.
                                                                // For version 1.00 only 8 variables are used, the last two are ignored.
extern  UINT16 gunPWStep_us[10];                                // Public data. RC servo motor pulse width increment/decrement
                                                                // step in usec.  Again for version 1.00 only 8 variables are used,
                                                                // the last two are ignored.
extern  UINT16 gnCurrentMotorIDA ;				// Public data.  RC servo motor ID.

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_PWMServo_Driver(TASK_ATTRIBUTE *);                   // PWM servo motor driver.

#endif