//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2015, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File				: Driver_PWMServo_V105.c
// Author(s)		: Fabian Kung
// Last modified	: 25 Feb 2015
// Toolsuites		: Microchip MPLAB X IDE v2.20 or above
//                	  MPLAB XC16 C-Compiler v1.22 or above


// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"

#include <timer.h>
#include <outcompare.h>
#include <stdio.h>

// NOTE: Public function prototypes are declared in the corresponding *.h file.


//
// --- PUBLIC VARIABLES ---
//
// RC servo motor driver control variables
UINT16 gunRCServoPW_us[10];                  // RC servo motor pulse width in usec.
UINT16 gunPWStep_us[10];                     // RC servo motor pulse width increment/decrement
                                            // step in usec.
UINT16 gnCurrentMotorIDA = 0;               // Current ID of RC servo motor being triggered.


//
// --- PRIVATE FUNCTION PROTOTYPES ---
//


//
// --- PRIVATE VARIABLES ---
//


//
// --- Microcontroller Pin Usage ---
//
// RD0 = Servo motor 0
// RD1 = Servo motor 1
// RD2 = Servo motor 2
// RD3 = Servo motor 3
// RD4 = Servo motor 4
// RD5 = Servo motor 5
// RF4 = Servo motor 6
// RE0 = Servo motor 7
// RE4 = Servo motor 8
// RE5 = Servo motor 9

//
// --- Process Level Constants Definition --- 
//
#define		_MAX_PULSEWIDTH_US	2300			    	// Max pulse width in microseconds.
#define		_MIN_PULSEWIDTH_US	620				// Min pulse width in microseconds.

#define         _MOT_CONST              60                              // This is FOSC_MHz/2, which is 60 for 120 MHz clock
#define         _INITIAL_PW1_US         1300                            // A neutral position for the motor/actuator triggered.
#define         _INITIAL_PW2_US         1300
#define         _INITIAL_PW3_US         1300
#define         _INITIAL_PW4_US         1300
#define         _INITIAL_PW5_US         1300
#define         _INITIAL_PW6_US         1300
#define         _INITIAL_PW7_US         1300
#define         _INITIAL_PW8_US         1300
#define         _INITIAL_PW9_US         1300
#define         _INITIAL_PW10_US        1300

#define         _DEFAULT_PW_STEP_US     6

///
/// Process name		: Proce__PWMServo_Driver
///
/// Author			: Fabian Kung
///
/// Last modified		: 25 Feb 2015
///
/// Code Version		: 1.05
///
/// Processor			: dsPIC33EP256MU8XX
///                               dsPIC33EP512MU8XX
///
/// Processor/System Resources 
/// PINS			: See above.
///
/// MODULES			: 1. OC1 - output compare 1 (Internal).
///                               2. OC2 - output compare 2 (Internal).                               
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables            : gunRCServoPW - User set RC servo control pulse width.
///			          nRCServoState - RC servo driver state.

#ifdef 				  __OS_VER		// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce__PWMServo_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce__PWMServo_Driver: An RTOS is required with this function"
#endif

///
/// Description			: Driver for RC type servo motor or similar actuator using PWM.
///				  The routine uses OC1 module of dsPICE to procduce one pulse everytime
///				  it is called.  This process will trigger each servo motor with a pulse
///                               in a round-robin fashion.  For this version 10 SERVO MOTORS are supported.
///                               Two global parameters control the property of servo motor:
///                                 gunRCServoPW_us[n]
///                                 gunPWStep_us[n]
///
///                               Where n is the index of the servo motor from 0 to 8.
///                               The width of the control pulse to the servo motor is determined by the global
///                               variable gunRCServoPW_us[], with the unit in microseconds.
///                               The second parameter gunPWStep_us[] determines how fast the servo motor
///                               turns to its destination, typically value from 3 to 40 microseconds.
///
///                               For instance for MOTOR1 the current pulse width is 1000 usec.  Say we set
///                               gunRCServoPW_us[1] = 1600 usec, and gunPWStep_us[1] = 30 usec.
///                               In this case whenever a control pulse is generated for MOTOR1, the pulse width will
///                               be incremented from 1000 usec to 16000 usec in 30 usec step.  When the difference
///                               between the set value and current pulse width is < gunPWStep_us[1], a default
///                               step of 2usec (declared as unPWSmallStep in the routine) will be used.  Once
///                               the current pulse width is within +- 1usec of the set value, the pulse width
///                               will be fixed.  In this way by using smaller gunPWStep_us[1] we can slow down
///                               the angular rotation of the motor (in moving from one angle to another)
///                               and using larger gunPWStep_us[1] to speed up the transition.
///
/// Usage example:                To use:
///                               Set gnRCServoPW_us[] to between _MIN_PULSEWIDTH_US and _MIN_PULSEWIDTH_US.
///                               Setting gnRCServoPW to 0 will stop the module (e.g. no output).

#define __SERVO_DELAY       13

void Proce_PWMServo_Driver(TASK_ATTRIBUTE *ptrTask)
{
    INT16 nTemp;
    INT16 nIndex;
    unsigned long ulMotor;
    static UINT16 unCurrentPW[10];                       // Current pulse width (PW).
    static UINT16 unMotorPW;
    static UINT16 unPWStep = 10;                        // Normal increment step for pulse width setting.
    static UINT16 unPWSmallStep = 2;                    // Small increment step for pulse width setting.

    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization of OC1 and OC2 modules, uC pins and motor variables.

                // Setup IO pins mode.
                TRISDbits.TRISD0 = 0;                   // Set RD0 as output.
                TRISDbits.TRISD1 = 0;                   // Set RD1 as output.
                TRISDbits.TRISD2 = 0;                   // Set RD2 as output.
                TRISDbits.TRISD3 = 0;                   // Set RD3 as output.
                TRISDbits.TRISD4 = 0;                   // Set RD4 as output.
                TRISDbits.TRISD5 = 0;                   // Set RD5 as output.
                TRISEbits.TRISE0 = 0;                   // Set RE0 as output.
                TRISFbits.TRISF4 = 0;                   // Set RF4 as output.
                TRISEbits.TRISE4 = 0;                   // Set RE4 as output.
                TRISEbits.TRISE5 = 0;                   // Set RE5 as output.

                OC1CON1 = 0;                            // According to the datasheet it is a
                OC1CON2 = 0;                            // good habit to set all OC control bits to 0 first.
                OC2CON1 = 0;
                OC2CON2 = 0;
                         
                gnCurrentMotorIDA = 0;

                for (nIndex = 0; nIndex < 10; nIndex++)
                {
                    gunRCServoPW_us[nIndex] = 0;        // Initialize output duty cycle to 0, eg. to turn off motor.
                    gunPWStep_us[nIndex] = _DEFAULT_PW_STEP_US;  // Initialize duty cycle increment/decrement step.
                }
                                                        // NOTE: 5th March 2013 F. Kung
                                                        // We need to set the initial pulse width to some
                                                        // neutral postion, as upon
                                                        // activated this driver will send out the default
                                                        // pulse width to the first motor with no user input.
                                                        // If the pulse width is not correct, it might cause
                                                        // the motor or actuator to hit a barrier and stalled,
                                                        // causing damage to the motor/actuator.
                unCurrentPW[0] = _INITIAL_PW1_US;       // Initialize current duty cycle.
                unCurrentPW[1] = _INITIAL_PW2_US;       // Initialize current duty cycle.
                unCurrentPW[2] = _INITIAL_PW3_US;       // Initialize current duty cycle.
                unCurrentPW[3] = _INITIAL_PW4_US;       // Initialize current duty cycle.
                unCurrentPW[4] = _INITIAL_PW5_US;       // Initialize current duty cycle.
                unCurrentPW[5] = _INITIAL_PW6_US;       // Initialize current duty cycle.
                unCurrentPW[6] = _INITIAL_PW7_US;       // Initialize current duty cycle.
                unCurrentPW[7] = _INITIAL_PW8_US;       // Initialize current duty cycle.
                unCurrentPW[8] = _INITIAL_PW9_US;       // Initialize current duty cycle.
                unCurrentPW[9] = _INITIAL_PW10_US;       // Initialize current duty cycle.
                OSSetTaskContext(ptrTask, 1, 1000);     // Next state = 1, timer = 1000.
            break;

            case 1: // State 1 - Gradually increase or decrease the servo motor angle by adjusting the duty cycle.
                    // Without this the RC servo motor might move too fast from one position/angle to another
                    // position/angle, resulting in overshoot.  With modern digital servo, which uses PID
                    // feedback control, this step can be eliminated.
                
                OC1CON1 = 0;                                        // According to the datasheet it is a
                OC1CON2 = 0;                                        // good habit to set all control bits to 0 first.
                OC2CON1 = 0;
                OC2CON2 = 0;
                                                                    // Disconnect OC1 and OC2 from all output pins.
                RPOR0bits.RP64R = 0b000000;                         // Disconnect RP64 (RD0) from OC peripheral.
                RPOR0bits.RP65R = 0b000000;                         // Disconnect RP65 (RD1) from OC peripheral.
                RPOR1bits.RP66R = 0b000000;                         // Disconnect RP66 (RD2) from OC peripheral.
                RPOR1bits.RP67R = 0b000000;                         // Disconnect RP67 (RD3) from OC peripheral.
                RPOR2bits.RP68R = 0b000000;                         // Disconnect RP68 (RD4) from OC peripheral.
                RPOR2bits.RP69R = 0b000000;                         // Disconnect RP67 (RD3) from OC peripheral.
                RPOR9bits.RP100R = 0b000000;                        // Disconnect RP100 (RF4) from OC peripheral.
                RPOR4bits.RP80R = 0b000000;                         // Disconnect RP80 (RE0) from OC peripheral.
                RPOR5bits.RP84R = 0b000000;                         // Disconnect RP84 (RE4) from OC peripheral.
                RPOR6bits.RP85R = 0b000000;                         // Disconnect RP85 (RE5) from OC peripheral.

                unPWStep = gunPWStep_us[gnCurrentMotorIDA];         // Get step change for pulse width.
                
                if (gunRCServoPW_us[gnCurrentMotorIDA] > 0)         // Only generate an output pulse if gunRCServoPW_us is > 0.
                {                                                   // Increment or decrement the pulse width setting by
                                                                    // comparing current pulse width setting with user setting.
                    nTemp = gunRCServoPW_us[gnCurrentMotorIDA] - unCurrentPW[gnCurrentMotorIDA];
                                                                    // Get the difference between current and user set pulse
                                                                    // width.
                    if (nTemp > 0)                                  // User set pulse width is larger than current pulse width.
                    {                                               
                                                                    // Check the difference between current setting and user setting.
                        if (nTemp > unPWStep)                       // If it is larger than the increment step unPWStep, increase
                        {                                           // the current setting by unPWStep.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] + unPWStep;
                        }
                        else
                        {                                           // Else increase the current setting by a small step, this is
                                                                    // to minimize the error between user setting and current setting.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] +  unPWSmallStep;
                        }
                    }
                    else if (nTemp < 0)                             // User set pulse width is smaller than current pulse width.
                    {
                        nTemp = -nTemp;                             // Check the difference between current setting and user setting.
                        if (nTemp > unPWStep)                       // If it is smaller than the increment step unPWStep, decrease
                        {                                           // the current setting by unPWStep.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] - unPWStep;
                        }
                        else
                        {                                           // Else decrease the current setting by a small step, this is
                                                                    // to minimize the error between user setting and current setting.
                            unCurrentPW[gnCurrentMotorIDA] = unCurrentPW[gnCurrentMotorIDA] -  unPWSmallStep;
                        }             
                    }

                    // Check for over and under limits (of the pulse width).
                    if (unCurrentPW[gnCurrentMotorIDA] > _MAX_PULSEWIDTH_US)       // Make sure the maximum PWM pulsewidth do not
                    {                                                              // exceed the MAX_PULSEWIDTH_US limit.
                        unCurrentPW[gnCurrentMotorIDA] = _MAX_PULSEWIDTH_US;
                    }
                    if (unCurrentPW[gnCurrentMotorIDA] < _MIN_PULSEWIDTH_US)       // Make sure the minimum PWM pulsewidth is not
                    {                                                              // smaller than the MIN_PULSEWIDTH_US limit.
                        unCurrentPW[gnCurrentMotorIDA] = _MIN_PULSEWIDTH_US;
                    }

                    switch (gnCurrentMotorIDA)
                    {
                        case 0:
                            OSSetTaskContext(ptrTask, 2, 1);            // Next state = 2, timer = 1.
                            break;
                        case 1:
                            OSSetTaskContext(ptrTask, 3, 1);            // Next state = 3, timer = 1.
                            break;
                        case 2:
                            OSSetTaskContext(ptrTask, 4, 1);            // Next state = 4, timer = 1.
                            break;
                        case 3:
                            OSSetTaskContext(ptrTask, 5, 1);            // Next state = 5, timer = 1.
                            break;
                        case 4:
                            OSSetTaskContext(ptrTask, 6, 1);            // Next state = 6, timer = 1.
                            break;
                        case 5:
                            OSSetTaskContext(ptrTask, 7, 1);            // Next state = 7, timer = 1.
                            break;
                        case 6:
                            OSSetTaskContext(ptrTask, 8, 1);            // Next state = 8, timer = 1.
                            break;
                        case 7:
                            OSSetTaskContext(ptrTask, 9, 1);            // Next state = 9, timer = 1.
                            break;
                        case 8:
                            OSSetTaskContext(ptrTask, 10, 1);            // Next state = 10, timer = 1.
                            break;
                        default:
                            OSSetTaskContext(ptrTask, 11, 1);            // Next state = 11, timer = 1.
                            break;
                    }
                    unMotorPW = unCurrentPW[gnCurrentMotorIDA];
                    
                }
                else
                {
                    OSSetTaskContext(ptrTask, 1, 15);           // Next state = 1, timer = 15.
                }

                gnCurrentMotorIDA++;
                if (gnCurrentMotorIDA == 10)
                {
                    gnCurrentMotorIDA = 0;                      // Reset motor/actuator ID.
                }
            break;

            case 2: // State 2 - Generate 1 pulse for motor/device 1 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                                                            // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR0bits.RP64R = 0b010000;             // RP64 or RD0 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bits mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR0bits.RP64R = 0b010001;             // RP64 or RD0 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.
                              
                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;                   // 32 bits mode.
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }                                      
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer =14.
            break;

            case 3: // State 3 - Generate 1 pulse for motor/device 2 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.


                                                            // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                
                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    
                    RPOR0bits.RP65R = 0b010000;             // RP65 or RD1 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bits mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    
                    RPOR0bits.RP65R = 0b010001;             // RP65 or RD1 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;                   // 32 bits mode.
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 4: // State 4 - Generate 1 pulse for motor/device 3 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                                                            // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR1bits.RP66R = 0b010000;             // RP66 or RD2 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR1bits.RP66R = 0b010001;             // RP66 or RD2 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 5: // State 5 - Generate 1 pulse for motor/device 4 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    //RPOR9bits.RP101R = 0b010000;          // RP101 or RF5 connected to OC1's output.
                                                            // NOTE: 23 April 2013 F. Kung
                                                            // Somehow RF5 output did not function properly when I activate
                                                            // the driver for MRF24J40 RF module.  Cannot trace the root
                                                            // cause, could be error with the CPU core itself.  I decided
                                                            // to use pin RD3 instead.
                    RPOR1bits.RP67R = 0b010000;             // RP67 or RD3 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    //RPOR9bits.RP101R = 0b010001;            // RP101 or RF5 connected to OC2's output.
                    RPOR1bits.RP67R = 0b010001;             // RP67 or RD3 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 6: // State 6 - Generate 1 pulse for motor/device 5 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    //RPOR9bits.RP101R = 0b010000;          // RP101 or RF5 connected to OC1's output.
                                                            // NOTE: 23 April 2013 F. Kung
                                                            // Somehow RF5 output did not function properly when I activate
                                                            // the driver for MRF24J40 RF module.  Cannot trace the root
                                                            // cause, could be error with the CPU core itself.  I decided
                                                            // to use pin RD3 instead.
                    RPOR2bits.RP68R = 0b010000;             // RP68 or RD4 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR2bits.RP68R = 0b010001;             // RP68 or RD4 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 7: // State 7 - Generate 1 pulse for motor/device 6 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    //RPOR9bits.RP101R = 0b010000;          // RP101 or RF5 connected to OC1's output.
                                                            // NOTE: 23 April 2013 F. Kung
                                                            // Somehow RF5 output did not function properly when I activate
                                                            // the driver for MRF24J40 RF module.  Cannot trace the root
                                                            // cause, could be error with the CPU core itself.  I decided
                                                            // to use pin RD3 instead.
                    RPOR2bits.RP69R = 0b010000;             // RP69 or RD5 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR2bits.RP69R = 0b010001;             // RP69 or RD5 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 8: // State 8 - Generate 1 pulse for motor/device 7 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR9bits.RP100R = 0b010000;            // RP100 or RF4 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR9bits.RP100R = 0b010001;            // RP100 or RF4 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 9: // State 9 - Generate 1 pulse for motor/device 8 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.
                

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR4bits.RP80R = 0b010000;             // RP80 or RE0 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR4bits.RP80R = 0b010001;             // RP80 or RE0 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 10: // State 10 - Generate 1 pulse for motor/device 9 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR5bits.RP84R = 0b010000;             // RP84 or RE4 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR5bits.RP84R = 0b010001;             // RP84 or RE4 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            case 11: // State 11 - Generate 1 pulse for motor/device 10 based on current motor position.
                    // The sequence begins by setting up the external data selector to the
                    // appropriate channel, setting OC1 or OC2 module to Dual Compare Single Pulse mode,
                    // and then setup the reference timebase for OC1/OC2.  See datasheet for more info.
                    // Since the timer for each OC1 is 16 bits (65535 unsigned integer max), for delay
                    // up to 1092 clock cycles, only OC1 is used.  For delay greater than 1092
                    // clock cycles, OC1 and OC2 are connected in cascade (OC1 output drives OC2).

                ulMotor = unMotorPW;
                ulMotor = ulMotor * _MOT_CONST;             // For __TCLK_US=0.01667 usec, the inverse is around 60.

                // Note: 5th March 2013 F. Kung.
                                                            // The peripheral in dsPIC can be connected to more than one
                                                            // I/O pins.  Thus we need to disconnect the peripheral first
                                                            // if we wish to change the peripheral connection to another
                                                            // pin.

                if (unMotorPW < 1093)                       // Check if exceed the limit of single OC module (16 bits).
                                                            // This corresponds to 65535/0.01667 = 1092.468
                {
                    // Single OC mode (e.g. OC1 is used).
                    RPOR6bits.RP85R = 0b010000;             // RP85 or RE5 connected to OC1's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.

                    // --- Turn on OC1 module ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC1R = 0;                               // Update OC1 main Register.
                    OC1TMR = 0x0000;                        // Clear OC1  internal counter.
                    OC1CON2bits.OC32 = 0;                   // 16 bit mode
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                else
                {
                    // Cascaded OC mode (e.g. both OC1 and OC2 are cascaded).
                    RPOR6bits.RP85R = 0b010001;             // RP85 or RE5 connected to OC2's output.
                    OC1CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC2CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                    OC1CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                    OC2CON1bits.OCTSEL = 0x7;
                    OC1CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC2CON2bits.SYNCSEL = 0b11111;          // No sync or trigger source.
                    OC1CON2bits.TRIGSTAT = 1;               // OC1 output tri-stated, as it is not needed.
                    // --- Turn on OC1 and OC2 modules ---
                    OC1RS = ulMotor;                        // Take the lower word.
                    OC2RS = ulMotor >> 16;                  // Take the upper word.

                    OC1R = 0;                               // Update OC1 and OC2 main Register.
                    OC2R = 0;
                    OC1TMR = 0x0000;                        // Clear OC1 and OC2 internal counter.
                    OC2TMR = 0x0000;

                    // Even module must be enabled first, odd module must be enabled last.
                    OC2CON2bits.OC32 = 1;
                    OC1CON2bits.OC32 = 1;
                    OC2CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                    OC1CON1bits.OCM = 0b100;                // Set to dual compare single pulse mode and on the Output Compare module.
                }
                OSSetTaskContext(ptrTask, 1, __SERVO_DELAY);           // Next state = 1, timer = 14.
            break;

            default:
                OSSetTaskContext(ptrTask, 0, 1);            // Back to state = 0, timer = 1.
            break;
	}
    }
}



