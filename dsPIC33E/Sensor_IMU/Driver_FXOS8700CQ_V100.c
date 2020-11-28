//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2015, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Driver_FXOS8700CQ_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 17 Aug 2016
// Toolsuites		: Microchip MPLAB-X IDE v3.30 or above
//                	  MPLAB-x XC16 C-Compiler v1.26 or above

// Include common header to all drivers and sources.  Here absolute path is used for user header files.
// To edit if one change folder
#include <math.h>
#include "../osmain.h"
#include "../Driver_I2C_V100.h"

// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//

int     gnTiltXraw;
int     gnTiltYraw;
int     gnTiltZraw;

int     gunFXOS8700CQ;                // 1 = Normal.
                                      // 0 = Device error or not attached.
//
// --- PRIVATE FUNCTION PROTOTYPES ---
//



//
// --- PRIVATE VARIABLES ---
//
// I2C Address
#define     _FXO8700C_ADD            0x1E    // Here we assume both SA0 and SA1 pins are tied to logic low.  
#define     _FXOS8700C_ADD_WRITE     0x3C    // 0x1E shift left 1 bit, R/W = 0.
#define     _FXOS8700C_ADD_READ      0x3D    // 0x1E shift left 1 bit, R/W = 1.
#define     _FXOS8700_TIMEOUT_COUNT  20
// FXO8700 Register address
#define     _FXOS8700_STATUS         0x00
#define     _FXOS8700_OUT_X_HIGH     0x01
#define     _FXOS8700_OUT_X_LOW      0x02
#define     _FXOS8700_OUT_Y_HIGH     0x03
#define     _FXOS8700_OUT_Y_LOW      0x04
#define     _FXOS8700_OUT_Z_HIGH     0x05
#define     _FXOS8700_OUT_Z_LOW      0x06
#define     _FXOS8700_F_SETUP        0x09
#define     _FXOS8700_TRIG_CFG       0x0A
#define     _FXOS8700_SYSMOD         0x0B
#define     _FXOS8700_INT_SOURCE     0x0C
#define     _FXOS8700_WHO_AM_I       0x0D
#define     _FXOS8700_XYZ_DATA_CFG   0x0E
#define     _FXOS8700_HPF_CUTOFF     0x0F
#define     _FXOS8700_CTRL_REG1      0x2A
#define     _FXOS8700_M_CTRL_REG1    0x5B
#define     _FXOS8700_M_CTRL_REG2    0x5C
//
// --- Process Level Constants Definition ---
//



//
// --- Microcontroller Pin Usage ---
//



///
/// Function name	: Proce_FXOS8700CQ_Driver
///
/// Author		: Fabian Kung
///
/// Last modified	: 16 August 2015
///
/// Code Version	: 0.71
///
/// Processor/System Resource 
/// PINS		: 1. Pin RF5 = I2C2 SCL, output.
///                       2. Pin RF4 = I2C2 SDA, input/output.
///
/// MODULES		: 1. I2C2 (Internal).
///
/// RTOS		: Ver 1 or above, round-robin scheduling.
///
/// Global Variables    : 

#ifdef __OS_VER			// Check RTOS version compatibility.
	#if __OS_VER < 1
		#error "Proce_FXOS8700CQ_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_FXOS8700CQ_Driver: An RTOS is required with this function"
#endif

///
/// Description	: 
///
/// I2C bus properties:
/// Baud rate = 200 kHz.
/// Mode: Single Master.
/// 
/// --- FXOS8700CQ Settings and Usage ---
/// (B) Basic configurations of the accelerometer and magnetometer. 
/// 1. Output Data Rate (ODR) = 400 Hz, 200 Hz for accelerometer and 200 Hz for magnetometer (both
///    sensors share a single ADC in the chip.
/// 2. FIFO is disabled.
/// 3. No high-pass filter for accelerometer data.
/// 4. Accelerometer in low-noise mode (restricted to +/-2g or +/-4g full scale only).
/// 5. accelerometer range of +/-4g range with 0.488 mg/LSB (Note the good old ADXL335 is +/-3g)
/// NOTE: 19 Aug 2016, when using the accelerometer to measure the tilt angle of a system, the output is 
/// only valid from -50 deg to + 50 deg.  Out of this range the accelerometer actually saturates (it's
/// output won't go beyond +-73 degrees from what I discovered.

void Proce_FXOS8700CQ_Driver(TASK_ATTRIBUTE *ptrTask)
{
    static int nIndex = 0;
    static int nTimeOut = 0;
    static int nCount = 0;

    if (ptrTask->nTimer == 0)
    {
        // Check for I2C bus error, abort all operation if bus error is detected.
        if (gI2CStat.bCommError == 1)               // Check for I2C bus error.
        {
            gunFXOS8700CQ = 0;
            OSSetTaskContext(ptrTask, 100, 1);        // Next state = 100, timer = 1.
        }

	switch (ptrTask->nState)
	{
        case 0: // State 0 - Initialization.
            OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 100 msec.
            nIndex = 0;                             // Reset the index.
            gunFXOS8700CQ = 0;                      // Indicate device not attached.
            break;

        case 1: // State 1 - See if the MEMS accelerometer is attached.
            if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
            {
                gbytI2CByteCount = 1;       // Indicate no. of bytes to read.
                gbytI2CRegAdd = _FXOS8700_WHO_AM_I;       // Start address of register.
                gbytI2CSlaveAdd =  _FXO8700C_ADD;
                gI2CStat.bRead = 1;
                OSSetTaskContext(ptrTask, 2, 1);        // Next state = 2, timer = 1.
            }
            else
            {
                OSSetTaskContext(ptrTask, 1, 1);        // Next state = 1, timer = 1.
            }
            break;

        case 2: // State 2 - Verify device ID of Slave, else restart.
            if (gI2CStat.bRead == 0)                        // Check if Read operation is completed.
            {                                               // Read operation complete, check received data.
                if (gbytI2CRXbuf[0] == 0xC7)                // Default ID for FXOS8700CQ.
                {
                    gunFXOS8700CQ = 1;                      // Indicate device is attached.
                    OSSetTaskContext(ptrTask, 3, 1);        // Next state = 3, timer = 1.
                }
                else
                {
                    gunFXOS8700CQ = 0;                    // Indicate device error or not attached.
                    OSSetTaskContext(ptrTask, 0, 100*__NUM_SYSTEMTICK_MSEC);       // Next state = 0, timer = 100 msec.
                }
            }
            else
            {
                OSSetTaskContext(ptrTask, 2, 1);        // Next state = 2, timer = 1.
            }
            break;

        case 3: // State 3 - Set FXOS8700CQ to standby mode, also set operating parameters.
            if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
            {
                gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                gbytI2CRegAdd = _FXOS8700_CTRL_REG1;        // Start address of register.
                gbytI2CTXbuf[0] = 0x00;                     // bit0: active = 0, standby.                                                                
                gbytI2CSlaveAdd =  _FXO8700C_ADD;
                gI2CStat.bSend = 1;
                OSSetTaskContext(ptrTask, 4, 1);        // Next state = 4, timer = 1.
            }
            else
            {
                OSSetTaskContext(ptrTask, 3, 1);        // Next state = 3, timer = 1.
            }
            break;

            case 4: // State 4 - Setup magnetometer control register 1.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_M_CTRL_REG1;        // Start address of register.
                    gbytI2CTXbuf[0] = 0x1F;                     // bit7: m_acal = 0, auto calibration disabled.
                                                                // bit6: m_rst = 0, no one-shot magnetic reset.
                                                                // bit5: m_ost = 0, no one-shot magnetic measurement.
                                                                // bit[4:2]: m_os=111, 8x oversampling (for 200Hz) to
                                                                // reduce magnetometer noise.
                                                                // bit[1:0]: m_hms=11, select hybrid mode with accelerometer
                                                                // and magnetometer active.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 5, 1);        // Next state = 5, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 4, 1);        // Next state = 4, timer = 1.
                }
                break;

            case 5: // State 5 - Setup magnetometer control register 2.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_M_CTRL_REG2;        // Start address of register.
                    gbytI2CTXbuf[0] = 0x20;                     // bit[7:7]: reserved.
                                                                // bit5: hyb_autoinc_mode=1, to map the magnetometer registers
                                                                // to follow the accelerometer registers.
                                                                // bit4: m_maxmin_dis = 0, to retain default min/max latching
                                                                // eventhough not used.
                                                                // bit3: m_maxin_dis_ths = 0.
                                                                // bit2: m_maxin_rst = 0.
                                                                // bit[1:0]: m_rst_cnt=00, to enable magneti reset each cycle.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 6, 1);        // Next state = 6, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 5, 1);        // Next state = 5, timer = 1.
                }
                break;

            case 6: // State 6 - Setup accelerometer sensor data, XYZ configuration register.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_XYZ_DATA_CFG;        // Start address of register.

                   
                    
                    //gbytI2CTXbuf[0] = 0x01;                   // bit[7:5]: reserved.
                                                                // bit4: hpf_out = 0, output high-pass filter is disabled.
                                                                // bit[3:2]: reserved.
                                                                // bit[1:0]: fs=01, accelerometer range of +/-4g range with
                                                                // 0.488mg/LSB
                    gbytI2CTXbuf[0] = 0x00;                     // bit[7:5]: reserved.
                                                                // bit4: hpf_out = 0, output high-pass filter is disabled.
                                                                // bit[3:2]: reserved.
                                                                // bit[1:0]: fs=00, accelerometer range of +/-2g range with
                                                                // 0.244mg/LSB
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 7, 1);        // Next state = 7, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 6, 1);        // Next state = 6, timer = 1.
                }
                break;

            case 7: // State 7 - Setup accelerometer, place device into active mode.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_CTRL_REG1;        // Start address of register.
                    gbytI2CTXbuf[0] = 0x0D;                     // bit[7:6]: aslp_rate=00, auto-wave sample frequency (when device is
                                                                // in Sleep mode) is 50 Hz.
                                                                // bit[5:3]: dr=001, output data rate (ODR), 400Hz for accelerometer or
                                                                // magnetometer only, 200Hz data rate (when in hybrid mode).
                                                                // bit2: lnoise = 1, low noise mode.
                                                                // bit1: f_read = 0, normal 16 bit reads.
                                                                // bit0: active = 1, take device out of standby, enable sampling.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 8, 1);        // Next state = 8, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 7, 1);        // Next state = 7, timer = 1.
                }
                break;

            case 8: // State 8 - Start reading sensor data.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 13;       // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _FXOS8700_STATUS;       // Start address of register.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 9, 1);        // Next state = 9, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 8, 1);        // Next state = 8, timer = 1.
                }
                break;

            case 9: // State 9 - Wait until read data finish, process data.  Data is in 14 bits 2's complement format. 
                    //           We convert this data to positive integer between 0 to 16383.
                if (gI2CStat.bRead == 0)                    // Check if Read operation is completed.
                {                                           // Read operation complete.
                    // --- Get accelerometer data ---
                                                            // bit[13:6] are stored in the MSB, while bit[5:0] are
                                                            // stored in bit7-bit2 of the LSB.
                    
                    gnTiltXraw = gbytI2CRXbuf[1];           // Get bit[13:6] of Y-axis data.
                    gnTiltXraw = (gnTiltXraw<<8)+gbytI2CRXbuf[2]; // Form word.
                    if ((gnTiltXraw & 0x2000)>0)            // Check if bit13 is 1.
                    {                                       // If it is 1 we should also set bit14 and bit15 to 1.
                        gnTiltXraw = gnTiltXraw | 0xC000;   // to maintain the polarity of the value.
                    }              
                    gfAccX = gnTiltXraw*0.000122;           // Convert raw data to acceleration (m/s^2) for +-4g FS.
                    //gfThetaX = asinf(gfAccX);               // The multiplier is 0.000488/4 = 0.000122.
                    //gnThetaXdeg = gfThetaX*57.2958;         // Convert tilt angle from radian to deg (int).
                                                            // From gnThetaYdeg = ((gfThetaY/3.14159)*180)
                                                            // with 180/3.14159 = 57.2958.                    
                    
                    gnTiltYraw = gbytI2CRXbuf[3];           // Get bit[13:6] of Y-axis data.
                    gnTiltYraw = (gnTiltYraw<<8)+gbytI2CRXbuf[4]; // Form word.
                    if ((gnTiltYraw & 0x2000)>0)            // Check if bit13 is 1.
                    {                                       // If it is 1 we should also set bit14 and bit15 to 1.
                        gnTiltYraw = gnTiltYraw | 0xC000;   // to maintain the polarity of the value.
                    }              
                    gfAccY = gnTiltYraw*0.000122;           // Convert raw data to acceleration (m/s^2) for +-4g FS.
                    gfThetaY = asinf(gfAccY);               // The multiplier is 0.000488/4 = 0.000122.
                    gnThetaYdeg = gfThetaY*57.2958;         // Convert tilt angle from radian to deg (int).
                                                            // From gnThetaYdeg = ((gfThetaY/3.14159)*180)
                                                            // with 180/3.14159 = 57.2958.
                    OSSetTaskContext(ptrTask, 8, 10);       // Next state = 8, timer = 10.
                                                            // This would gives roughly 100 readings per second.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 9, 1);        // Next state = 9, timer = 1.
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

 