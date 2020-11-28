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
// Last modified	: 25 Aug 2016
// Toolsuites		: Microchip MPLAB-X IDE v3.30 or above
//                	  MPLAB-x XC16 C-Compiler v1.26 or above

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include <math.h>
#include "../osmain.h"
#include "../Driver_I2C_V100.h"
//#include "C:\Users\wlkung\Google Drive\Projects\Embedded_Systems\C_Library\dsPIC33E\Driver_Audio_V101.h"
// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//
int     gnTiltXraw;
int     gnTiltYraw;
int     gnTiltZraw;
float   gfAccX;
float   gfAccY;
float   gfAccZ;
float   gfThetaYrad;                // In radian
int     gnThetaYdeg;                // In degree.

int     gunFXOS8700CQ;              // 1 = Normal.
                                    // 0 = Device error or not attached.

int     gnOmegaXraw;
int     gnOmegaYraw;
int     gnOmegaZraw;
float   gfOmegaX;                   // In radian/sec
float   gfOmegaY;                   // In radian/sec
float   gfOmegaZ;                   // In radian/sec
int     gunFXAS21002C;              // 1 = Normal.
                                    // 0 = Device error or not attached.
int     gnIMUStatus = 0;            // IMU status.  It is recommended to check this status
                                    // upon power up to make sure the IMU is READY, otherwise
                                    // the outputs from the IMU are not valid.
#define     _IMU_READY          1
#define     _IMU_NOT_READY      0
#define     _IMU_ERROR          -1
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
/// Function name	: Proce_FXAS21002C_FXOS8700C_Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 25 August 2016
///
/// Code Version	: 0.91
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
/// Description	: This is a driver for NXP's FXAS21002C MEMS 3-axis digital gyroscope
///               and NXP's FXOS8700CQ MEMS 6 axis digital accelerometer and magnetometer.
///
/// Both FXAS21002C and FXOS8700CQ communicate with the MCU via I2C bus.                
/// (A) Configuration of the I2C bus:
/// Baud rate = 200 kHz.
/// Mode: Single Master.
/// 
/// --- FXAS21002C Settings and Usage ---
/// (B) Basic configurations of the gyroscope. 
/// 1. Output Data Rate (ODR) = 400 Hz. 
/// 2. FIFO is disabled.
/// 3. Internal low-pass filter enabled and cut-off frequency at 256 Hz.
/// 4. Internal high-pass filter cut-off = 1.95 Hz for ODR = 400 Hz.
/// 5. Internal high-pass filter disabled.
/// 6. Full scale = 1000 dps (same as the good old LYH3100A from STM).
///
/// NOTE 1: 24 Aug 2016, when the internal high-pass filter is enabled, the gyroscope
/// output would not contain any 'DC' components. This implies if we perform numerical
/// integration of the gyroscope output for any particular axis, this result will not
/// gives an approximation of the tilt angle on the axis.  Thus I discovered it is
/// best to bypass the high-pass filter.
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
/// (D) Usage and data update rate
///  At the moment due to the limited bandwidth of the I2C bus and the MCU, the X, Y and Z axis 
///  data are updated at a rate of 100 samples / second.
///
/// --- FXOS8700CQ Settings and Usage ---
/// (B) Basic configurations of the accelerometer and magnetometer. 
/// 1. Output Data Rate (ODR) = 400 Hz, 200 Hz for accelerometer and 200 Hz for magnetometer (both
///    sensors share a single ADC in the chip.
/// 2. FIFO is disabled.
/// 3. No high-pass filter for accelerometer data.
/// 4. Accelerometer in low-noise mode (restricted to +/-2g or +/-4g full scale only).
/// 5. Accelerometer range of +/-2g range with 0.244 mg/LSB (Note the good old ADXL335 is +/-3g)
///    I discovered that at +/-4g range the accelerometer output is noisy.  It sometimes contains    
///    'spike' when I rapidly or abruptly rotate the sensor.
///
/// NOTE: 19 Aug 2016, when using the accelerometer to measure the tilt angle of a system, the output is 
/// only valid from -50 deg to + 50 deg.  Out of this range the accelerometer actually saturates (it's
/// output won't go beyond +-73 degrees from what I discovered.

void Proce_FXAS21002C_FXOS8700C_Driver(TASK_ATTRIBUTE *ptrTask)
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
            gunFXOS8700CQ = 0;
            OSSetTaskContext(ptrTask, 100, 1);              // Next state = 100, timer = 1.
        }

        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 100 msec.
                nIndex = 0;                                 // Reset the index.
                gunFXAS21002C = 0;                          // Indicate device not attached.
                gunFXOS8700CQ = 0;
                break;
                 
            // --- Initialization and setup of gyroscope ---
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
                    // SEL = 01: Internal high-pass filter cut-off = 1.95 Hz for ODR = 400 Hz.
                    // HPF_EN = 0: Internal high-pass filter disabled.
                    // FS = 01: Full scale = 1000 dps (same as the good old LYH3100A from STM).
                if (gI2CStat.bI2CBusy == 0)                     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXAS21002_CTRL_REG0;       // Start address of register.
                    //gbytI2CTXbuf[0] = 0x15;                   // See comments above.     
                    gbytI2CTXbuf[0] = 0x11;
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
                }
                else
                {
                    OSSetTaskContext(ptrTask, 4, 1);            // Next state = 4, timer = 1.
                }                    
                break;                
                
            // --- Initialization and setup of accelerometer and magnetometer ---     
                
            case 5: // State 5 - See if the MEMS accelerometer is attached.
                if (gI2CStat.bI2CBusy == 0)                     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                       // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _FXOS8700_WHO_AM_I;         // Start address of register.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 6, 1);            // Next state = 6, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 5, 1);            // Next state = 5, timer = 1.
                }
                break;

            case 6: // State 6 - Verify device ID of Slave, else restart.
                if (gI2CStat.bRead == 0)                        // Check if Read operation is completed.
                {                                               // Read operation complete, check received data.
                    if (gbytI2CRXbuf[0] == 0xC7)                // Default ID for FXOS8700CQ.
                    {
                        gunFXOS8700CQ = 1;                      // Indicate device is attached.
                        OSSetTaskContext(ptrTask, 7, 1);        // Next state = 7, timer = 1.
                    }
                    else
                    {
                        gunFXOS8700CQ = 0;                      // Indicate device error or not attached.
                        OSSetTaskContext(ptrTask, 0, 100*__NUM_SYSTEMTICK_MSEC);       // Next state = 0, timer = 100 msec.
                    }
                }
                else
                {
                    OSSetTaskContext(ptrTask, 6, 1);            // Next state = 62, timer = 1.
                }
                break;

            case 7: // State 7 - Set FXOS8700CQ to standby mode, also set operating parameters.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_CTRL_REG1;        // Start address of register.
                    gbytI2CTXbuf[0] = 0x00;                     // bit0: active = 0, standby.                                                                
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 8, 1);            // Next state = 8, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 3, 1);            // Next state = 3, timer = 1.
                }
                break;

            case 8: // State 8 - Setup magnetometer control register 1.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_M_CTRL_REG1;        // Start address of register.
                    //gbytI2CTXbuf[0] = 0x1F;                     // bit7: m_acal = 0, auto calibration disabled.
                                                                // bit6: m_rst = 0, no one-shot magnetic reset.
                                                                // bit5: m_ost = 0, no one-shot magnetic measurement.
                                                                // bit[4:2]: m_os=111, 8x oversampling (for 200Hz) to
                                                                // reduce magnetometer noise.
                                                                // bit[1:0]: m_hms=11, select hybrid mode with accelerometer
                                                                // and magnetometer active.
                    gbytI2CTXbuf[0] = 0x1C;                     // Shut down magnetometer, only accelerometer is active.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 9, 1);        // Next state = 9, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 8, 1);        // Next state = 8, timer = 1.
                }
                break;

            case 9: // State 9 - Setup magnetometer control register 2.
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
                                                                // bit[1:0]: m_rst_cnt=00, to enable automatic magnetic reset each cycle.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 10, 1);        // Next state = 10, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 9, 1);        // Next state = 9, timer = 1.
                }
                break;

            case 10: // State 10 - Setup accelerometer sensor data, XYZ configuration register.  This also sets 
                     // the full-scale range.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;       // Indicate no. of bytes to send.
                    gbytI2CRegAdd = _FXOS8700_XYZ_DATA_CFG;        // Start address of register.
          
                    //gbytI2CTXbuf[0] = 0x01;                   // bit[7:5]: reserved.
                                                                // bit4: hpf_out = 0, output high-pass filter is disabled.
                                                                // bit[3:2]: reserved.
                                                                // bit[1:0]: fs=01, accelerometer range of +/-4g range with
                                                                // 0.488 mg/LSB
                    gbytI2CTXbuf[0] = 0x00;                     // bit[7:5]: reserved.
                                                                // bit4: hpf_out = 0, output high-pass filter is disabled.
                                                                // bit[3:2]: reserved.
                                                                // bit[1:0]: fs=00, accelerometer range of +/-2g range with
                                                                // 0.244 mg/LSB
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bSend = 1;
                    OSSetTaskContext(ptrTask, 11, 1);        // Next state = 11, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 10, 1);        // Next state = 10, timer = 1.
                }
                break;

            case 11: // State 11 - Setup accelerometer, place device into active mode, enable low noise operation and 
                     // set the output data rate (ODR).
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
                    OSSetTaskContext(ptrTask, 12, 1);           // Next state = 12, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 11, 1);           // Next state = 11, timer = 1.
                }
                break;

            // --- Start reading data ---
            // From oscilloscope observation, with __SYSTEMTICK_US = 166.67 usec,this requires roughly 7.9 msec per cycle.
            // This would gives roughly 126.6 readings per second for 7 bytes read of each sensors (i.e. x,y and z axis of
            // the gyroscope and accelerometer). If we can reduce the bytes to 3 for gyroscope and 5 for accelerometer (i.e.
            // x axis for gyroscope and x,y axes for accelerometer), the sampling interval can be reduced to 5.8 msec per
            // cycle or 172.4 samples per second.  Note that changing the I2C speed to 400 kHz has no effect as the 
            // bottleneck is due to the way the I2C driver is written and the round-robin priority scheduler.
                
            case 12: // State 12 - Read data.
                
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 3;                       // Indicate no. of bytes to read.
                    //gbytI2CByteCount = 7;                       // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _FXAS21002_STATUS;          // Start address of register.              
                    gbytI2CSlaveAdd =  _FXAS21002_ADD;
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 13, 1);           // Next state = 13, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 12, 1);           // Next state = 12, timer = 1.
                }                
                break;
                
            case 13: // State 13 - Process raw angular velocity data.  Raw data is 16 bits 2's complement format.
                if (gI2CStat.bRead == 0)                    // Check if Read operation is completed.
                {                                           // Read operation complete
                    gnOmegaXraw = gbytI2CRXbuf[1];
                    gnOmegaXraw = (gnOmegaXraw<<8)+gbytI2CRXbuf[2]; // Form word for X axis raw data.
                    //gfOmegaX = gnOmegaXraw * 0.03125;       // Convert raw data to dps for +-1000 FS.
                    gfOmegaX = gnOmegaXraw * 0.00054542;        // Convert raw data to radian/sec for +-1000 FS.
                                                                // 0.03125 x pi/180 = 0.00054542
                    //gnOmegaYraw = gbytI2CRXbuf[3];
                    //gnOmegaYraw = (gnOmegaYraw<<8)+gbytI2CRXbuf[4]; // Form word for Y axis raw data.
                    //gfOmegaY = gnOmegaYraw * 0.03125;       // Convert raw data to dps for +-1000 FS.  
                    //gfOmegaY = gnOmegaYraw * 0.00054542;        // Convert raw data to radian/sec for +-1000 FS.
                                                                // 0.03125 x pi/180 = 0.00054542                    
                    //gnOmegaZraw = gbytI2CRXbuf[5];
                    //gnOmegaZraw = (gnOmegaZraw<<8)+gbytI2CRXbuf[6]; // Form word for Z axis raw data.
                    //gfOmegaZ = gnOmegaZraw * 0.03125;       // Convert raw data to dps for +-1000 FS.
                    //gfOmegaZ = gnOmegaZraw * 0.00054542;        // Convert raw data to radian/sec for +-1000 FS.
                                                                // 0.03125 x pi/180 = 0.00054542                    
                    //OSSetTaskContext(ptrTask, 14, 1);       // Next state = 14, timer = 1.
                    
                    OSSetTaskContext(ptrTask, 12, 1);       // Next state = 12, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 13, 1);        // Next state = 13, timer = 1.
                }                           
                break;
                
            case 14: // State 14 - Start reading sensor data.
                if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 5;       // Indicate no. of bytes to read.
                    //gbytI2CByteCount = 7;       // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _FXOS8700_STATUS;       // Start address of register.
                    gbytI2CSlaveAdd =  _FXO8700C_ADD;
                    gI2CStat.bRead = 1;
                    OSSetTaskContext(ptrTask, 15, 1);        // Next state = 15, timer = 1.
                }
                else
                {
                    OSSetTaskContext(ptrTask, 14, 1);        // Next state = 14, timer = 1.
                }
                break;

            #define     _FXOS8700C_mgLSB    0.000244        // 0.244 mg/LSB for +/-2g full scale range.
                
            case 15: // State 15 - Wait until read data finish, process data.  Data is in 14 bits 2's complement format. 
                    //           We convert this data to positive integer between 0 to 16383.
                if (gI2CStat.bRead == 0)                    // Check if Read operation is completed.
                {                                           // Read operation complete.
                    // --- Get accelerometer data ---
                                                            // bit[13:6] are stored in the MSB, while bit[5:0] are
                                                            // stored in bit7-bit2 of the LSB.
                    gnTiltXraw = gbytI2CRXbuf[1];           // Get bit[13:6] of Y-axis data.
                    gnTiltXraw = (gnTiltXraw<<8)+gbytI2CRXbuf[2]; // Form word.
                    gnTiltXraw = gnTiltXraw>>2;             // Restore the word to the correct position.
                    if ((gnTiltXraw & 0x2000)>0)            // Check if bit13 is 1.
                    {                                       // If it is 1 we should also set bit14 and bit15 to 1.
                        gnTiltXraw = gnTiltXraw | 0xC000;   // to maintain the polarity of the value.
                    }              
                    gfAccX = gnTiltXraw*_FXOS8700C_mgLSB;   // Convert raw data to acceleration (m/s^2).            
                    
                    gnTiltYraw = gbytI2CRXbuf[3];           // Get bit[13:6] of Y-axis data.
                    gnTiltYraw = (gnTiltYraw<<8)+gbytI2CRXbuf[4]; // Get bit[5:0], and form left shifted word.
                                                            // bit1 and bit0 of gnTiltYraw are unknowns.
                    gnTiltYraw = gnTiltYraw>>2;             // Restore the word to the correct position.
                    if ((gnTiltYraw & 0x2000)>0)            // Check if bit13 is 1.
                    {                                       // If it is 1 we should also set bit14 and bit15 to 1.
                        gnTiltYraw = gnTiltYraw | 0xC000;   // to maintain the polarity of the value.
                    }              
                    gfAccY = gnTiltYraw*_FXOS8700C_mgLSB;   // Convert raw data to acceleration (m/s^2).
                    gfThetaYrad = asinf(gfAccY);            // The multiplier is 0.000488.
                    gnThetaYdeg = gfThetaYrad*57.2958;      // Convert tilt angle from radian to deg (int).
                                                            // From gnThetaYdeg = ((gfThetaY/3.14159)*180)
                                                            // with 180/3.14159 = 57.2958.

                    //gnTiltZraw = gbytI2CRXbuf[5];           // Get bit[13:6] of Y-axis data.
                    //gnTiltZraw = (gnTiltZraw<<8)+gbytI2CRXbuf[6]; // Form word.
                    //gnTiltZraw = gnTiltZraw>>2;             // Restore the word to the correct position.
                    //if ((gnTiltZraw & 0x2000)>0)            // Check if bit13 is 1.
                    //{                                       // If it is 1 we should also set bit14 and bit15 to 1.
                    //    gnTiltZraw = gnTiltZraw | 0xC000;   // to maintain the polarity of the value.
                    //}              
                    //gfAccZ = gnTiltZraw*_FXOS8700C_mgLSB;   // Convert raw data to acceleration (m/s^2.   
                    
                    OSSetTaskContext(ptrTask, 12, 1);       // Next state = 12, timer = 1.
                    gnIMUStatus = _IMU_READY;              // Indicate IMU is ready.       
                }
                else
                {
                    OSSetTaskContext(ptrTask, 15, 1);        // Next state = 15, timer = 1.
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

 