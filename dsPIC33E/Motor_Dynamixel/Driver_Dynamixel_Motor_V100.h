// Author			: Fabian Kung
// Date				: 23 March 2015
// Filename			: Driver_Dynamixel_Motor_V100.h

#ifndef _DRIVER_DYNAMIXEL_MOTOR_dsPIC33_H
#define _DRIVER_DYNAMIXEL_MOTOR_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

// 
//
// --- PUBLIC VARIABLES ---
//
			
extern  BYTE    gbytInstruction_Array[15];	// Array to hold instruction packet data
extern  BYTE    gbytStatus_Array[8];		// Array to hold returned status packet data
extern  long	glngTime_Counter;		// Timer for time out watchers
extern  BYTE    gbytStatus_Return_Value = 0;	// Status packet return states ( NON , READ , ALL )

//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_Dynamixel_Motor_Driver(TASK_ATTRIBUTE *);

#endif
