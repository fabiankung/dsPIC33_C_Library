// Author			: Fabian Kung
// Date				: 17 Aug 2015
// Filename			: Driver_FXAS21002C_V100.h

#ifndef _DRIVER_FXAS21002C_dsPIC33E_H
#define _DRIVER_FXAS21002C_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"

//
// --- PUBLIC VARIABLES ---
//
extern  int     gnOmegaXraw;
extern  int     gnOmegaYraw;
extern  int     gnOmegaZraw;
extern  float   gfOmegaX;
extern  float   gfOmegaY;
extern  float   gfOmegaZ;
extern  int     gunFXAS21002C;
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_FXAS21002C_Driver(TASK_ATTRIBUTE *);
#endif