//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2016, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Driver_FXAS21002C_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 17 Aug 2016
// Toolsuites		: Microchip MPLAB-X IDE v3.30 or above
//                	  MPLAB-x XC16 C-Compiler v1.26 or above

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"
#include "../Driver_I2C_V100.h"
#include "C:\Users\wlkung\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//

int     gnOmegaXraw;
int     gnOmegaYraw;
int     gnOmegaZraw;
float   gfOmegaX;
float   gfOmegaY;
float   gfOmegaZ;
int     gunFXAS21002C;           // 1 = Normal.
                                        // 0 = Device error or not attached.
//
// --- PRIVATE FUNCTION PROTOTYPES ---
//



//
// --- PRIVATE VARIABLES ---
//
// I2C address
#define     _FXAS21002_ADD           0x20  // Here we assume SA0 pin is tied to logic low.
#define     _FXAS21002_ADD_WRITE     0x40  // 0x20 shift left 1 bit, R/W = 0.
#define     _FXAS21002_ADD_READ      0x41  // 0x20 shift left 1 bit, R/W = 1.
#define     _FXOS21002_TIMEOUT_COUNT  20
// FXAS2100C Register address
#define     _FXAS21002_STATUS        0x00
#define     _FXAS21002_OUT_X_HIGH    0x01
#define     _FXAS21002_OUT_X_LOW     0x02
#define     _FXAS21002_OUT_Y_HIGH    0x03
#define     _FXAS21002_OUT_Y_LOW     0x04
#define     _FXAS21002_OUT_Z_HIGH    0x05
#define     _FXAS21002_OUT_Z_LOW     0x06
#define     _FXAS21002_DR_STATUS     0x07
#define     _FXAS21002_F_STATUS      0x08
#define     _FXAS21002_F_SETUP       0x09
#define     _FXAS21002_F_EVENT       0x0A
#define     _FXAS21002_INT_SRC_FLAG  0x0B
#define     _FXAS21002_WHO_AM_I      0x0C
#define     _FXAS21002_CTRL_REG0     0x0D
#define     _FXAS21002_RT_CFG        0x0E
#define     _FXAS21002_RT_SRC        0x0F
#define     _FXAS21002_RT_THS        0x10
#define     _FXAS21002_RT_COUNT      0x11
#define     _FXAS21002_TEMP          0x12
#define     _FXAS21002_CTRL_REG1     0x13
#define     _FXAS21002_CTRL_REG2     0x14
#define     _FXAS21002_CTRL_REG3     0x15

//
// --- Process Level Constants Definition ---
//


//
// --- Microcontroller Pin Usage ---
//


///
/// Function name	: Proce_FXAS21002C_Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 17 August 2016
///
/// Code Version	: 0.80
///
/// Processor/System Resource 
/// PINS            : 1. Pin RF5 = I2C2 SCL, output.
///                   2. Pin RF4 = I2C2 SDA, input/output.
///
/// MODULES         : 1. I2C2 (Internal).
///
/// RTOS            : Ver 1 or above, round-robin scheduling.
///
/// Global Variables    : 

#ifdef __OS_VER			// Check RTOS version compatibility.
	#if __OS_VER < 1
		#error "Proce_FXAS21002C_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_FXAS21002C_Driver: An RTOS is required with this function"
#endif

///
/// Description	: This is a driver for NXP's FXAS21002C MEMS 3-axis digital gyroscope.
/// The gyroscope communicates with the MCU via I2C bus.                
/// (A) Configuration of the I2C bus:
/// Baud rate = 200 kHz.
/// Mode: Single Master.
/// 
/// (B) Basic configurations of the gyroscope. 
/// 1. Output Data Rate (ODR) = 400 Hz. 
/// 2. FIFO is disabled.
/// 3. Internal low-pass filter enabled and cut-off frequency at 256 Hz.
/// 4. Internal high-pass filter cut-off = 0.99 Hz for ODR = 400 Hz.
/// 5. Internal high-pass filter enabled.
/// 6. Full scale = 1000 dps (same as the good old LYH3100A from STM).
///
/// (C) Conversion of raw output data to floating point format:
/// The raw output data is stored as 2's complement 16 bit format, range from -32768 to +32767.
/// Let FS = Full Scale in dps (degree per second).  For instance when FS = +-1000 dps, 
///     gnOmegaXraw = raw x-axis output
///     gfOmegaX = floating point x-axis output in dps.
/// Then
///     gfOmegaX = gnOmegaXraw x 0.03125 (from datasheet at FS = +-1000 dps the nominal
///                             sensitivity is 31.25 mdps/LSB)
/// Multiplication of the integer value with floating point (0.03051758) will result
/// in a floating point value.  The same applies to Y and Z axis data.
///
/// (D) Data update rate
///  At the moment due to the limited bandwidth of the I2C bus and the MCU, the X, Y and Z axis 
///  data are updated at a rate of 100 samples / second.


void Proce_FXAS21002C_Driver(TASK_ATTRIBUTE *ptrTask)
{
    static int nIndex = 0;
    static int nTimeOut = 0;
    static int nCount = 0;

    if (ptrTask->nTimer == 0)
    {
        // Check for I2C bus error, abort all operation if bus error is detected.
        if (gI2CStat.bCommError == 1)                       // Check for I2C bus error.
        {
            gunFXAS21002C = 0;
            OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
        }

        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 100 msec.
                nIndex = 0;                                 // Reset the index.
                gunFXAS21002C = 0;                          // Indicate device not attached.
                break;

            case 1: // State 1 - See if the MEMS gyroscope is attached.
                if (gI2CStat.bI2CBusy == 0)                 // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                   // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _FXAS21002_WHO_AM_I;    // Start address of register.
                    gbytI2CSlaveAdd =  _FXAS21002_ADD;
                    gI2CStat.bRead = 1;                     // Read from I2C slave device.
                    OSSetTaskContext(ptrTask, 2, 1);        // Next state = 2, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 1);        // Next state = 1, timer = 1.
                }                
                break;
                
            case 2: // State 2 - Verify device ID of Slave, else restart.
                if (gI2CStat.bRead == 0)                    // Check if Read operation is completed.
                {                                           // Read operation complete, check received data.
                    if (gbytI2CRXbuf[0] == 0xD7)            // Default ID for FXAS21002C.
                    {
                        gunFXAS21002C = 1;                  // Indicate device is attached.

                        OSSetTaskContext(ptrTask, 3, 1);    // Next state = 3, timer = 1.
                    }
                    else
                    {
                        gunFXAS21002C = 0;                  // Indicate device error or not attached.
                        OSSetTaskContext(ptrTask, 0, 10*__NUM_SYSTEMTICK_MSEC);        // Next state = 0, timer = 10 msec.
                    }
                }
                else
                {
                    OSSetTaskContext(ptrTask, 2, 1);            // Next state = 2, timer = 1.
                }                
                break;
            
            case 3: // State 3 - Device completed POR, is in standby mode.  Setup CTRL_REG0.
                    // Basic configurations: Output Data Rate (ODR) = 400 Hz. 
                    // FIFO is disabled.
                    // BW = 00: Internal low-pass filter cut-off frequency at 256 Hz.
                    // SPIW = 0
                    // SEL = 11: Internal high-pass filter cut-off = 1.95 Hz for ODR = 400 Hz.
                    // HPF_EN = 1: Internal high-pass filter enabled.
                    // FS = 01: Full scale = 1000 dps (same as the good old LYH3100A from STM).
                if (gI2CStat.bI2CBusy == 0)                     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXAS21002_CTRL_REG0;       // Start address of register.
                    //gbytI2CTXbuf[0] = 0x1D;                     // See comments above.     
                    gbytI2CTXbuf[0] = 0x15;
                    gbytI2CSlaveAdd =  _FXAS21002_ADD;
                    gI2CStat.bSend = 1;                         // Write to I2C slave device.
                    OSSetTaskContext(ptrTask, 4, 1);            // Next state = 4, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 3, 1);            // Next state = 3, timer = 1.
                }                    
                break;
 
            case 4: // State 4 - Configure CTRL_REG1, put the device in Active mode and configure ODR to 400 Hz.
                // RST = 0: Reset not triggered.
                // ST = 0: Self test disabled.
                // DR = 001: ODR = 400 Hz.
                // ACTIVE = 1: Device is active.
                // READY = X: We set to 0 here.
                if (gI2CStat.bI2CBusy == 0)                     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXAS21002_CTRL_REG1;       // Start address of register.
                    gbytI2CTXbuf[0] = 0x06;                     // See comments above.                                                              
                    gbytI2CSlaveAdd =  _FXAS21002_ADD;
                    gI2CStat.bSend = 1;                         // Write to I2C slave device.
                    OSSetTaskContext(ptrTask, 5, 1);            // Next state = 5, timer = 1.
                                            gnAudioTone[0] = 1;
                        gnAudioTone[1] = 4;
                        gnAudioTone[2] = 0;
                }
                else
                {
                    OSSetTaskContext(ptrTask, 4, 1);            // Next state = 4, timer = 1.
                }                    
                break;                
                
            case 5: // State 5 - Read data.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 7;                    // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _FXAS21002_STATUS;       // Start address of register.
                    //gbytI2CByteCount = 6;                       // Indicate no. of bytes to read.
                    //gbytI2CRegAdd = _FXAS21002_OUT_X_HIGH;     // Start address of register.                    
                    gbytI2CSlaveAdd =  _FXAS21002_ADD;
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 6, 1);        // Next state = 6, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 5, 1);        // Next state = 5, timer = 1.
                }                
                break;
                
            case 6: // State 6 - Process raw angular velocity data.
                if (gI2CStat.bRead == 0)                    // Check if Read operation is completed.
                {                                           // Read operation complete
                    gnOmegaXraw = gbytI2CRXbuf[1];
                    gnOmegaXraw = (gnOmegaXraw<<8)+gbytI2CRXbuf[2]; // Form word for X axis raw data.
                    gfOmegaX = gnOmegaXraw * 0.03125;       // Convert raw data to dps.
                    gnOmegaYraw = gbytI2CRXbuf[3];
                    gnOmegaYraw = (gnOmegaYraw<<8)+gbytI2CRXbuf[4]; // Form word for Y axis raw data.
                    gfOmegaY = gnOmegaYraw * 0.03125;       // Convert raw data to dps.                   
                    gnOmegaZraw = gbytI2CRXbuf[5];
                    gnOmegaZraw = (gnOmegaZraw<<8)+gbytI2CRXbuf[6]; // Form word for Z axis raw data.
                    gfOmegaZ = gnOmegaZraw * 0.03125;       // Convert raw data to dps.
                    
                    OSSetTaskContext(ptrTask, 5, 1);        // Next state = 5, timer = 1.
                                                            // From oscilloscope observation, with __SYSTEMTICK_US = 166.67 usec,
                                                            // this would gives roughly 250 readings per second for 7 bytes read. 
                }
                else
                {
                    OSSetTaskContext(ptrTask, 6, 1);        // Next state = 6, timer = 1.
                }                           
                break;
                
            case 100: // State 100 - Idle.
                break;

            default:
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}

 