// Author			: Fabian Kung
// Date				: 25 Feb 2015
// Filename			: Driver_PWMServo_V105.h

#ifndef _DRIVER_PWMSERVO_dsPIC33E_H
#define _DRIVER_PWMSERVO_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

//
// --- PUBLIC VARIABLES ---
//
// RC servo motor driver control variables

extern  UINT16 gunRCServoPW_us[10];                              // Public date. RC servo motor pulse width in usec.
extern  UINT16 gunPWStep_us[10];                                 // Public data. RC servo motor pulse width increment/decrement
                                                                // step in usec.
extern  UINT16 gnCurrentMotorIDA ;				// Public data.  RC servo motor ID.

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_PWMServo_Driver(TASK_ATTRIBUTE *);                   // PWM servo motor driver.

#endif