
#include "osmain.h"


///
/// Process name    : Robot_Sensor_EyeLED
///
/// Author			: Fabian Kung
///
/// Last modified	: 13 Jan 2018
///
/// Code Version	: 1.18
///
/// Processor       : dsPIC33EP256MU80X family.
///
/// Processor/System Resources
/// PINS			: RE4 - IR LED drive pin.
///                   RB4 - PEYE_LED_DRIVER, visible light LED drive pin
///                   RB2 - Left IR sensor 
///                   RE0 - Right IR sensor
///
/// MODULES			: OC5 or OC6.
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables: gnEyeLED
///                   gnEyeLEDDuration
///                   gnEyeLEDDefault
///                   gnIRSenStatus
///                   gunIRDisSensor
///                   gunMachineVisionStatus

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Robot_Sensor_EyeLED: Incompatible OS version"
	#endif
#else
	#error "Robot_Sensor_EyeLED: An RTOS is required with this function"
#endif

/// Description:
/// This is a process which coordinate or controls all the various sensors on the robot.  The sensors in 
/// this case are:
/// 1. Active IR object detection sensor.
/// 2. Visible LED for the left and right eyes.  
/// The visible LEDs are not a sensor but since IR and visible LEDs are mounted together in the robot, 
/// we include them in this process as well.
/// 
/// Each eye in the robot contains 2 LEDs, i.e. an IR LED, and a visible light LED.  The LEDs of similar
/// type are connected together to a control pin, here the connection as follows:
///     VISIBLE LIGHT LEDs -> PEYE_LED_DRIVER
///     IR LED -> RE5
/// 
/// OC5 is used to drive the IR LED, it is connected to pin RE5 and is synchronized 
/// or triggered by TIMER3. Dual-compare continuous pulse mode is used for the OC module.
/// The IR LED pin is driven at 38kHz for 500 usec, then stop for 3 msec, and repeat.  This is to accommodate
/// the IR Receiver characteristics.  The IR Receiver cannot be continuously exposed with IR light of 50% duty  
/// ASK signals, else it will be insensitive (I think it saturates internally).  
/// The IR LED pin duty cycle is fixed by the variable nIntensity.  
///
/// Usage:
/// --- Visible Light LED ---
/// To light up the visible light LED, set gnEyeLED to 1 to 4 (4 being the brightest), setting gnEyeLED = 0 
/// to turn off the LED.  Also set gnEyeLEDDuration to a value > 0.  This variable determines the duration where
/// the visible light LED will be driven at the intensity level of gnEyeLED.  Each unit of gnEyeLEDDuration will
/// lights up the visible light LED for around 3.5 msec.  The routine will decrement gnEyeLEDDuration every cycle,
/// until gnEyeLEDDuration reaches 0.  The the intensity setting will be switch to the level set by gnEyeLEDDefault. 
///
/// --- IR LED Brightness Control ---
/// To control the IR LED brightness or the sensitivity of the IR Sensor, we can adjust the variable nIntensity.
/// Valid nIntensity values range from 10 to 1500, with 30 being dimmest and 1500 brightest. 
/// However from experiments we should limit the values to between 30 to 300, as too high intensity then there is
/// the possibility of photo leakage.  This means the IR light from the IR LED gets coupled to the IR sensor 
/// without being reflected from external objects, causing false output.  We can set nIntensity to 200 initially,
/// this give a detection range of > 10 cm.  If an object is detected then the nIntensity can be reduced by stages 
/// to 100 to confirm the presence of object.  Alternatively nIntensity can be switched between 100, 200 and 300 
/// periodically to gauge the distance of the object from the robot.
///
/// --- Active IR Object Detector Status ---
/// gnIRSenStatus:
/// bit0 = Right active IR sensor status, 1 = object detected, 0 = no object.
/// bit1 = Left active IR sensor status, 1 = object detected, 0 = no object.
///

#define     _IR_LED_INTENSITY_NORMAL    100         // Infrared LED in the eyes intensity control.
#define     _IR_LED_INTENSITY_HIGH      200    


#define     T3_PS	8                                                           // TIMER3 Prescalar.

//#define	LED_PULSE_PERIOD_CONTROL	(1/(5.0*T3_PS*0.001*__TCLK_US))-1       // TIMER3 count to generate 5kHz.
#define     LED_PULSE_PERIOD_CONTROL	(1/(38.0*T3_PS*0.001*__TCLK_US))-1      // TIMER3 count to generate 38kHz.
#define     MAX_LED_DURATION            T3_PS*LED_PULSE_PERIOD_CONTROL
#define     EYE_LED_COEFFICIENT         2                                       // This is the integer of LED_PULSE_PERIOD_CONTROL/100
#define     PEYE_LED_DRIVER             _RB4
#define     PIN_IRSENL                  _RB2                                    // IR sensor input, active low.
#define     PIN_IRSENR                  _RE0                                    // IR sensor input, active low.


 void Robot_Sensor_EyeLED(TASK_ATTRIBUTE *ptrTask)
{
    static int nIntensity = 0;                          
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
        {
            case 0: // State 0 - Initialization.
                // Setup IO pins mode.
                gnEyeLED =  0;
                PEYE_LED_DRIVER = 0;
                TRISEbits.TRISE4 = 0;                   // Set RE4 as output.
                //TRISEbits.TRISE5 = 0;                   // Set RE5 as output.
                TRISBbits.TRISB4 = 0;                   // Set RB4 as output.
                TRISBbits.TRISB2 = 1;                   // Set RB2 as input.
                TRISEbits.TRISE0 = 1;                   // Set RE0 as input. 
                // Settings of remappable output pin. The cluster of bits RPnR
                // control the mapping of the pin internal peripheral output.
                // Note n = integer.  If we refer to the datasheet, pin RE4
                // is also called RP84, and RP84R bits are contained in
                // the special function register RPOR65 in the controller.
                RPOR5bits.RP84R = 0b010101;             // RP84 or RE4 connected to OC6's output.
                //RPOR9bits.RP101R = 0b010101;             // RP101 or RF5 connected to OC6's output.
                OC6CON1 = 0;                            // According to the datasheet it is a
                OC6CON2 = 0;                            // good habit to set all OC control bits to 0 first.
                //OC5CON1bits.OCSIDL = 1;                 // Output Compare 5 halts when in idle mode.
                //OC5CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                //OC5CON2bits.OC32 = 0;                   // 16s bit mode
                //OC5CON2bits.SYNCSEL = 0b01101;          // Synchronization source is TIMER3.
                OC6CON1bits.OCSIDL = 1;                 // Output Compare 6 halts when in idle mode.
                OC6CON1bits.OCTSEL = 0x7;               // Use peripheral clock (Tclk/2) as clock source.
                OC6CON2bits.OC32 = 0;                   // 16 bits mode
                OC6CON2bits.SYNCSEL = 0b01101;          // Synchronization source is TIMER3.
                T3CONbits.TSIDL = 1;                    // Stop TIMER3 when in idle mode.
                T3CONbits.TCS = 0;                      // Clock source for TIMER3 is peripheral clock (Tclk/2).
                T3CONbits.TCKPS = 0b01;                 // TIMER3 prescalar = 1:8.
                nIntensity = _IR_LED_INTENSITY_NORMAL;  // Initial brightness of IR LED, typical.
                gunMachineVisionStatus = 0;
                OSSetTaskContext(ptrTask, 2, 100*__NUM_SYSTEMTICK_MSEC);     // Next state = 2, timer = 100 msec.
            break;
            
            case 2: // State 2 - Pulse IR LEDs and visible light LEDs.    
                // Visible light LEDs.
                if (gnEyeLEDDuration > 0)
                {                     
                    if (gnEyeLED > 0)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }               
                }
                else
                {                
                    if (gnEyeLEDDefault > 0)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }   
                }
                // IR LEDs
                
                if (nIntensity > MAX_LED_DURATION)      // Check for overflow.                 
                {
                    nIntensity = MAX_LED_DURATION - 1;
                }
                if (nIntensity < 0)
                {
                    nIntensity = 0;
                }
                
                //OC5RS = nIntensity;                     // Set intensity level for LED1 driver 1.
                //OC5R = 0;                               // Update OC5 main Register.
                //OC5TMR = 0x0000;                        // Clear OC5  internal counter.
                //OC5CON1bits.OCM = 0b101;                // Set to dual compare continuous pulse mode and on the Output Compare module.
                OC6RS = nIntensity;                     // Set intensity level for LED1 driver 1.
                OC6R = 0;                               // Update OC6 main Register.
                OC6TMR = 0x0000;                        // Clear OC6  internal counter.
                OC6CON1bits.OCM = 0b101;                // Set to dual compare continuous pulse mode and on the Output Compare module.                
                T3CONbits.TON = 1;                      // Turn on TIMER3.
                PR3 = LED_PULSE_PERIOD_CONTROL;         // Reload TIMER3 compare register.  TIMER3 will increment on every positive clock edge until
                                                        // TIMER3 = PR3, then it will reset.
                TMR3 = 0x0000;                          // Reset TIMER .
                OSSetTaskContext(ptrTask, 3, 3);       // Next state = 3, timer = 3 ticks, about 0.5 msec for 1 tick = 166.67 usec.
            break;

            case 3: // State 3 - Check global variable gnEyeLED and set the intensity of visible light LED accordingly.
                    //           Also turn off IR LED.
                if (gnEyeLEDDuration > 0)
                {                     
                    if (gnEyeLED > 1)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }
                }
                else
                {                
                    if (gnEyeLEDDefault > 1)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }   
                }   
                
                if (PIN_IRSENR == 0)
                {
                    gnIRSenStatus = gnIRSenStatus | 0x0001;     // Set bit 0.
                }
                else
                {
                    gnIRSenStatus = gnIRSenStatus & 0xFFFE;     // Clear bit 0.
                }
                if (PIN_IRSENL == 0)
                {
                    gnIRSenStatus = gnIRSenStatus | 0x0002;     // Set bit 1.
                }
                else
                {
                    gnIRSenStatus = gnIRSenStatus & 0xFFFD;     // Clear bit 1.
                }                
                //OC5CON1bits.OCM = 0b000;                // Turn off LED1 driver.
                OC6CON1bits.OCM = 0b000;                // Turn off LED1 driver.
                OSSetTaskContext(ptrTask, 4, 1*__NUM_SYSTEMTICK_MSEC);       // Next state = 4, timer = 1 msec.
            break;
            
            case 4: // State 4 - Continue to drive visible light LED.
                if (gnEyeLEDDuration > 0)
                {                
                    if (gnEyeLED > 2)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }      
                }
                else
                {                
                    if (gnEyeLEDDefault > 2)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }   
                }  
                
                if (PIN_IRSENR == 0)
                {
                    gnIRSenStatus = gnIRSenStatus | 0x0001;     // Set bit 0.
                }
                else
                {
                    gnIRSenStatus = gnIRSenStatus & 0xFFFE;     // Clear bit 0.
                }
                if (PIN_IRSENL == 0)
                {
                    gnIRSenStatus = gnIRSenStatus | 0x0002;     // Set bit 1.
                }
                else
                {
                    gnIRSenStatus = gnIRSenStatus & 0xFFFD;     // Clear bit 1.
                }                 
                OSSetTaskContext(ptrTask, 5, 1*__NUM_SYSTEMTICK_MSEC);       // Next state = 5, timer = 1 msec.
                break;
                
            case 5: // State 5 - Continue to drive visible light LED.
                if (gnEyeLEDDuration > 0)
                {
                    if (gnEyeLED > 3)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }                       
                }
                else
                {                
                    if (gnEyeLEDDefault > 3)
                    {
                        PEYE_LED_DRIVER = 1;
                    }
                    else
                    {
                        PEYE_LED_DRIVER = 0;
                    }   
                }  
                
                if (gnEyeLEDDuration > 0)
                {     
                    gnEyeLEDDuration--;
                }                                                                   
                
                if (PIN_IRSENR == 0)
                {
                    gnIRSenStatus = gnIRSenStatus | 0x0001;     // Set bit 0.
                }
                else
                {
                    gnIRSenStatus = gnIRSenStatus & 0xFFFE;     // Clear bit 0.
                }
                if (PIN_IRSENL == 0)
                {
                    gnIRSenStatus = gnIRSenStatus | 0x0002;     // Set bit 1.
                }
                else
                {
                    gnIRSenStatus = gnIRSenStatus & 0xFFFD;     // Clear bit 1.
                }                
                
                OSSetTaskContext(ptrTask, 2, 1*__NUM_SYSTEMTICK_MSEC);       // Next state = 2, timer = 1 msec.
                break; 
                
            default:
                OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
        }
    }
}