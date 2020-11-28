//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2017-2019, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File             : Drivers_A4988_Eight_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 13 June 2019
// Toolsuites		: Microchip MPLAB-X IDE v5.10 or above
//                	  MPLAB XC16 C-Compiler v1.33 or above
//			

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "../osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//
typedef struct StructProceA4988Driver
{
    UINT16  unEn4988;           // Set greater than 0 to enable A4988 module.
                                // Note: User process can reset the driver by setting this
                                // global variable to 0.  Reseting the driver also initialize
                                // all the velocity and distance variables.
    INT16   nSpeed1;            // Speed settings, from 0 (stop the motor) to
    INT16   nSpeed2;            // 800 (maximum speed).  Positive rotate clockwise,
                                // negative rotate anti-clockwise.
} DSPIC33E_A4988_DRIVER;

DSPIC33E_A4988_DRIVER    gobjDriverA4988;
long        gnDistanceMoveLW = 0;   // Distance traveled by left wheel in no. of steps, 32 bits integer.
long        gnDistanceMoveRW = 0;   // Distance traveled by right wheel in no. of steps, 32 bits integer.
long        gnDistanceMoveW = 0;    // Average distance traveled by both wheels in no. of steps, 32 bits integer.
int         gnHeading = 0;          // This indicates the direction, essentially the difference between right and left 
                                    // wheel distance.  Facing right if > 0, and facing left if < 0.			
unsigned int    gunDeadBandThres = 1 ; // Deadband threshold, speed setting less than this magnitude will be ignored.
//
// --- PRIVATE FUNCTION PROTOTYPES ---
//

//
// --- PRIVATE VARIABLES ---
//

#define PIN_STEPPER_DIR1                 _RB6 
#define PIN_STEPPER_DIR2                 _RB7 
#define PIN_A4988_ENABLE                 _RB9
#define PIN_STEPPER_STEP1                _RD4
#define PIN_STEPPER_STEP2                _RD5

#define _DISABLE_A4988                   1
#define _ENABLE_A4988                    0

//
// --- Process Level Constants Definition --- 
//


///
/// Process name	: Proce_A4988_Driver
///
/// Author			: Fabian Kung
///
/// Last modified   : 13 June 2019
///
/// Code Version	: 0.96
///
/// Processor		: dsPIC33EP256MU80X family.
///                   dsPIC33EP512MC80X family.
///
/// Processor/System Resources 
/// PINS			: 1. RD4 - Step output to A4988 chip 1.
///                   2. RB6 - Direction output to A4988 chip 1. 
///                   3. RD5 - Step output to A4988 chip 2.
///                   4. RB7 - Direction output to A4988 chip 2. 
///                   5. RB9 - Enable pin control for both A4988 chips.
///                            Low = disable FET outputs of both chips.
///                            High = enable FET outputs of both chips.
/// 
/// MODULES			: 1. OC7 (Internal) for Motor1.
///                   2. OC8 (Internal) for Motor2.
///                   3. TIMER3, with 70 MHz processor clock.
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables	: gobjDriverA4988
///                       gnDistanceMoveLW
///                       gnDistanceMoveRW
///                       gnDistanceMoveW
///                       gnHeading


#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_A4988_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_A4988_Driver: An RTOS is required with this function"
#endif

/// Description	: Subroutine to drive the A4988 or compatible (such as DRV8825) bi-phase stepper motor driver chips.
///               This driver supports two A4988 chips, e.g. two stepper motors, driven in 1/8th step (0.225 
///               degree) mode.
///               The codes make uses of linearization algorithm between the speed setting and 
///               pulse duration adapted from J. Brokking's codes for arduino,
///               in: http://www.brokking.net/yabr_main.html
///               The process uses the internal Output Compare (OC) module of the dsPIC33 processors in 
///               edge-aligned PWM mode to generate the periodic step pulses (OCM<2:0> = 110).  The corresponding
///               OCxTMR counter will increment from 0 to a value equal to OCxRS, then reset and repeat. Thus the
///               periodic frequency of the pulses is determined by value stored in OCxRS. The frequency of the
///               PWM pulses is proportional to the rotational speed needed.  Another I/O pin is used 
///               to control the DIR (direction).  In addition to driving the stepper motor, this process also 
///               keep track of the instantaneous distance traveled by the left and right wheels (assuming no 
///               slippage) by sampling the STEP pins to the stepper motor driver.
///                    
/// Example of usage: 
/// 1) To start the sampling and conversion, we first set gobjDriverA4988.unEn4988 to 1 or larger. 
/// 2) Then we set nSpeed1 or nSpeed2 to between -1600 and +1600.  Values between -gunDeadBandThres to +gunDeadBandThres
///    will stop the motor.  The driver codes has a linearization routines, which will
///    generate almost linear angular velocity output.  See the Excel file for more detail of the algorithm. 
///
/// 3) The global variables gnDistanceMoveLW, gnDistanceMoveRW, gnDistanceMoveW and gnHeading
///    stores the instantaneous left wheel distance, right wheel distance, average wheel 
///    distance and the heading.  User routines can read this value as frequently as needed.
///    These global variables will be cleared to 0 whenever the driver is disabled.
///    Disabling the driver by setting gobjDriverA4988.unEn4988 to 0 or smaller will reset
///    all these registers to zero.

#if __FOSC_MHz < 116    // Check processor clock frequency, this driver may not work
                        // properly if clock frequency is below 116 MHz.
    #error "Proce_A4988_Driver: Processor frequency is less than 59 MHz, this driver may not work properly"
#endif

#define  T3_PS	64                                                           // TIMER3 Prescalar.

#define	_STEPPER_MOTOR_PULSE_WIDTH      8       // The pulse width for the STEP pulse, in multiples of the period resolution.
                                                // For instance if the period resolution is 20 usec, and this value is 9, then
                                                // each pulse will be 160 usec.  Thus we need to make sure that 
                                                // _STEPPER_MOTOR_MIN_PERIOD_US is greater than this value.  Setting this value
                                                // smaller will allow higher maximum frequency of the STEP control pulse, but 
                                                // beware on the implication on EMC and signal integrity issue that might affect
                                                // the shape of the pulse.
                                                // With this parameter set, the smallest period acceptable will be:
                                                // Tperiod(min) = (_STEPPER_MOTOR_PULSE_WIDTH + 1)*20 usec = 180 usec.
                                                // or
                                                // f(max) = 1/Tperiod(min) = 5.555 kHz.
                                                // 
                                                // If the stepper motor step anlge is 0.45 degree (quarter step), this is
                                                // equivalent to (5555x0.45)/360 = 6.944 rotation/second.
#define _STEPPER_MOTOR_PERIOD_RESOLUTION_US    20      // This is the smallest increment/decrement.
#define _STEPPER_MOTOR_COUNT            _STEPPER_MOTOR_PERIOD_RESOLUTION_US/(__TCLK_US*T3_PS)  // No. of count needed in 
                                                                                               // timer T3 for 1 period
                                                                                               // resolution.
//#define _STEPPER_MOTOR_MIN_SPEED        0
//#define _STEPPER_MOTOR_MAX_SPEED        1600


void Proce_A4988_Driver(TASK_ATTRIBUTE *ptrTask)
{
    int nTemp, nTemp2;
    long nlTemp;
    static int  nLeftOutHigh = 0;
    static int  nRightOutHigh = 0;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization OC7 and OC8 to PWM mode, with clock source 
                    // provided by Timer3.
                
                TRISBbits.TRISB6 = 0;                   // Set RB6 as output.
                TRISBbits.TRISB7 = 0;                   // Set RB7 as output.
                TRISBbits.TRISB9 = 0;                   // Set RB9 as output.
                TRISFbits.TRISF4 = 0;                   // Set RF4 as output.
                TRISFbits.TRISF5 = 0;                   // Set RF5 as output.
                
                PIN_A4988_ENABLE = _DISABLE_A4988;      // First disable all the FET outputs of the A4988 chips.
                
                // Settings of remappable output pin. The cluster of bits RPnR
                // control the mapping of the pin internal peripheral output.
                // Note n = integer.  If we refer to the datasheet, pin RF4
                // is also called RP100, and RP100R bits are contained in
                // the special function register RPOR9 in the controller.
                RPOR2bits.RP68R = 0b010110;             // RP68 or RD4 connected to OC7's output.
                RPOR2bits.RP69R = 0b010111;             // RP69 or RD5 connected to OC8's output.
                //RPOR9bits.RP100R = 0b010110;             // RP100 or RF4 connected to OC7's output.
                //RPOR9bits.RP101R = 0b010111;             // RP101 or RF5 connected to OC8's output.
                
                OC7CON1 = 0;                            // Initialize both control registers for OC7.     
                OC7CON2 = 0;
                OC8CON1 = 0;                            // Initialize both control registers for OC8.     
                OC8CON2 = 0;                
                
                OC8CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                OC8CON2bits.SYNCSEL = 0b11111;          // 0x1F, No trigger or sync source is selected.  
                                                        // The internal timer OC8TMR resets when it reaches 
                                                        // the value of OC8RS.
                OC8CON1bits.OCTSEL = 0x01;              // Timer3 provides the clock source. 
                //OC8CON1bits.OCM = 0b110;                // Set OC8 to PWM mode.                 
                                                        // Note: 4/7/2017 F.Kung
                                                        // I discovered that this frequency is given by:
                                                        // fclock = fsource / pre-scalar
                                                        // fsource is the input clock source for Timer 3,
                                                        // which is the peripheral clock in this case.
                OC7CON1bits.OCSIDL = 1;                 // Same settings as OC8.
                OC7CON2bits.SYNCSEL = 0b11111;
                OC7CON1bits.OCTSEL = 0x01;
                //OC7CON1bits.OCM = 0b110;
                OC8RS = 0;  
                OC7RS = 0;
                OC8R = 0;
                OC7R = 0;
                OC8TMR = 0x0000;
                OC7TMR = 0x0000;       
                
                T3CONbits.TSIDL = 1;                    // Stop TIMER3 when in idle mode.
                T3CONbits.TCS = 0;                      // Clock source for TIMER3 is peripheral clock (Tclk/2).
                T3CONbits.TCKPS = 0b10;                 // TIMER3 prescalar = 1:64.
                T3CONbits.TON = 1;                      // Turn on TIMER3.
                TMR3 = 0x0000;                          // Reset TIMER.
                
                gobjDriverA4988.nSpeed1 = 0;
                gobjDriverA4988.nSpeed2 = 0;
                OSSetTaskContext(ptrTask, 1, 100*__NUM_SYSTEMTICK_MSEC); // Next state = 1, timer = 100 msec.						 
            break;

            case 1: // State 1 - Wait for module to be enabled.
                if (gobjDriverA4988.unEn4988 > 0)       // Check if module is enabled.
                {
                    OSSetTaskContext(ptrTask, 2, 1);    // Next state = 2, timer = 1.
                    if (OC8CON1bits.OCM != 0b110)       // If the OC8 is not in PWM mode,
                    {                                   // set it to PWM mode.
                        OC8CON1bits.OCM = 0b110;        
                    }
                    if (OC7CON1bits.OCM != 0b110)       // If the OC7 is not in PWM mode,
                    {                                   // set OC7 to PWM mode.
                        OC7CON1bits.OCM = 0b110;        
                    }
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;  // Cut-off the power to stepper motor.
                    OC7CON1bits.OCM = 0b000;            // Turn off OC7 and OC8.
                    OC8CON1bits.OCM = 0b000;
                    gobjDriverA4988.nSpeed1 = 0;        // Reset motor 1 and motor 2 speed settings.
                    gobjDriverA4988.nSpeed2 = 0;
                    gnDistanceMoveLW = 0;               // Clear all distance counters.
                    gnDistanceMoveRW = 0;
                    gnDistanceMoveW = 0;
                    gnHeading = 0;                      // Reset heading register.
                    OSSetTaskContext(ptrTask, 1, 1);    // Next state = 1, timer = 1.
                }                
                
                // Update distance counters by polling the OC7 and OC8 output pins.  Also compute average
                // distance and heading.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1)) // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1)              // Check for forward direction.
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                } 
                nlTemp = gnDistanceMoveLW + gnDistanceMoveRW;       // Compute the average distance traveled.  Here a long integer
                gnDistanceMoveW = nlTemp >> 1;                      // is used to prevent overflow. Divide by 2.
                gnHeading = gnDistanceMoveRW - gnDistanceMoveLW;    // Compute the direction or heading.
            break;

            case 2: // State 2 - Scan unSpeed1 and set the OC7 module accordingly.  
                nTemp = gobjDriverA4988.nSpeed1;                
                if (nTemp < 0)      
                {
                    PIN_STEPPER_DIR1 = 1;
                    nTemp = -nTemp;
                }
                else
                {
                    PIN_STEPPER_DIR1 = 0; 
                }
                
                if (nTemp > gunDeadBandThres)      // There is a deadband.  For two-wheels robot 
                {                   // a small deadband can reduce unwanted oscillation.    
                   // NOTE: 18 May 2019, by F. Kung.
                   // The wheel angular velocity is generated by a series of pulses to the STEP input of A4988
                   // chip.  The higher the frequency the faster is the rotation speed.  However the frequency
                   // is inversely proportional to the period of 1 pulse.  This inverse relation between the
                   // set velocity and period is non-linear, thus a compensation routine is needed.
                   // The compensation routine for non-linear relationship between angular 
                   // velocity and pulse period is based J. Brokking codes for 
                   // Arduino. Where for positive speed setting:        
                   // NewSpeedSetting = 805 - 5500/(SpeedSettings + 9)
                   // For negative speed setting:
                   // NewSpeedSetting = -805 - 5500/(SpeedSettings - 9)
                   // Since this is inversely proportional to the step interval, 
                   // we subtract from the maximum time interval.
                   // Here the first coefficient 5500 can be increase or decrease to change the slope of the mapping 
                   // between set velocity and actual motor shaft rotation speed.  See the corresponding Excel file
                   // for further information.
                   PIN_A4988_ENABLE = _ENABLE_A4988;
                   nTemp2 = 11000/(nTemp+9);
                   OC7R = _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT;      // Set OC7 output pin to high.  The width of the 
                                                                                // pulse is determined by the value in OC7R, i.e.
                                                                                // _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT
                   if (nTemp2 < (_STEPPER_MOTOR_PULSE_WIDTH+1))
                   {
                       nTemp2 = _STEPPER_MOTOR_PULSE_WIDTH+1;                   // We must make sure OC7RS >= OC7R at all times! 
                   }
                   OC7RS = nTemp2*_STEPPER_MOTOR_COUNT;                         // While the value in OC7RS sets the period.   
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;                          // Cut-off the power to stepper motor when
                                                                                // not turning.  This depends on application, 
                                                                                // it means there will be no holding torque
                                                                                // when the motor is not turning. Turning the motor
                                                                                // off when not moving reduces power consumption
                                                                                // and makes the stepper motor cooler.       
                    OC7R = 0;
                    OC7RS = 0;                    
                }                                                               
                                                                                               
                // Note: 30 Aug 2017, the above codes to set the Output Compare unit needs to execute first
                // before we update the distance ticks.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }                
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1))    // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1)              // Check for forward direction.
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                }                           
                OSSetTaskContext(ptrTask, 3, 1);                                // Next state = 3, timer = 1.                
            break;

            case 3: // State 3 - Scan unSpeed2 and set the OC8 module accordingly. 
                nTemp = gobjDriverA4988.nSpeed2;                
                if (nTemp < 0)
                {
                    PIN_STEPPER_DIR2 = 0;
                    nTemp = -nTemp;
                }
                else
                {
                    PIN_STEPPER_DIR2 = 1; 
                }
                if (nTemp > gunDeadBandThres)      // There is a deadband.
                {                    
                   PIN_A4988_ENABLE = _ENABLE_A4988;
                   nTemp2 = 11000/(nTemp+9);               
                   OC8R = _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT;      // Set OC8 output pin to high.  The width of the 
                                                                                // pulse is determined by the value in OC8R, i.e.
                                                                                // _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT
                   if (nTemp2 < (_STEPPER_MOTOR_PULSE_WIDTH+1))
                   {
                       nTemp2 = _STEPPER_MOTOR_PULSE_WIDTH+1;                   // We must make sure OC8RS >= OC8R at all times! 
                   }
                   OC8RS = nTemp2*_STEPPER_MOTOR_COUNT;                         // While the value in OC8RS sets the period.                                                                                   
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;                          // Cut-off the power to stepper motor when
                                                                                // not turning.  This depends on application,
                                                                                // it means there will be no holding torque
                                                                                // when the motor is not turning. Turning the motor
                                                                                // off when not moving reduces power consumption
                                                                                // and makes the stepper motor cooler.
                    OC8R = 0;                                                   // By setting OC8R = OC8RS, output pin of OC8
                    OC8RS = 0;                                                  // will be driven to low when not active.
                }                                                                
                                                                                
                // Note: 30 Aug 2017, the above codes to set the Output Compare unit needs to execute first
                // before we update the distance ticks.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }                
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1)) // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1) 
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                }      
                                  
                OSSetTaskContext(ptrTask, 1, 1);                                // Next state = 1, timer = 1.
                break;
                
            case 10: // State 10 - Stop the motor.
                OC7CON1bits.OCM = 0b000;
                OC8CON1bits.OCM = 0b000;
                OSSetTaskContext(ptrTask, 1, 1);       // Next state = 1, timer = 1.
                break;
                
            default:
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}

