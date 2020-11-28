// Author			: Fabian Kung
// Date				: 17 Aug 2015
// Filename			: Driver_FXOS8700CQ_V100.h

#ifndef _DRIVER_FXOS8700CQ_dsPIC33E_H
#define _DRIVER_FXOS8700CQ_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"

//
// --- PUBLIC VARIABLES ---
//
extern  int     gnTiltXraw;
extern  int     gnTiltYraw;
extern  int     gnTiltZraw;
extern  int     gunFXOS8700CQ;
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_FXOS8700CQ_Driver(TASK_ATTRIBUTE *);
#endif