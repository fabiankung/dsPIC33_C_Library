// Author			: Fabian Kung
// Date				: 22 Aug 2015
// Filename			: Driver_FXAS21002C_V100.h

#ifndef _DRIVER_FXAS21002C_dsPIC33E_H
#define _DRIVER_FXAS21002C_dsPIC33E_H

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"

//
// --- PUBLIC VARIABLES ---
//
extern  int     gnTiltXraw;
extern  int     gnTiltYraw;
extern  int     gnTiltZraw;
extern  float   gfAccX;
extern  float   gfAccY;
extern  float   gfAccZ;
extern  float   gfThetaYrad;    // In radian
extern  int     gnThetaYdeg;    // In degree.
extern  int     gunFXOS8700CQ;         

extern  int     gnOmegaXraw;
extern  int     gnOmegaYraw;
extern  int     gnOmegaZraw;
extern  float   gfOmegaX;
extern  float   gfOmegaY;
extern  float   gfOmegaZ;
extern  int     gunFXAS21002C;

extern  int     gnIMUStatus;           
#define     _IMU_READY          1
#define     _IMU_NOT_READY      0
#define     _IMU_ERROR          -1
//
// --- PUBLIC FUNCTION PROTOTYPE ---
//
void Proce_FXAS21002C_FXOS8700C_Driver(TASK_ATTRIBUTE *);
#endif