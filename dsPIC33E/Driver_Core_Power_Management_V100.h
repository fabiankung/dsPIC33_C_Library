// Author			: Fabian Kung
// Date				: 23 Sep 2014
// Filename			: Driver_Core_Power_Management_V100.h

#ifndef _DRIVER_CORE_POWER_CONTROL_dsPIC33_H
#define _DRIVER_CORE_POWER_CONTROL_dsPIC33_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

//
// --- PUBLIC VARIABLES ---
//
extern INT16 gnProcessorPower;
   // Constants for gnProcessorPower:
    #define     __CPU_NORMAL    1
    #define     __CPU_POWERDOWN  0
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_Core_Power_Control_Driver(TASK_ATTRIBUTE *);		// ADC module driver.
												
#endif