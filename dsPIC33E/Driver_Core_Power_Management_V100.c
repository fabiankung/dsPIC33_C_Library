//////////////////////////////////////////////////////////////////////////////////////////////
//
//	USER DRIVER ROUTINES DECLARATION (PROCESSOR DEPENDENT)
//
//  (c) Copyright 2012, Fabian Kung Wai Lee, Selangor, MALAYSIA
//  All Rights Reserved  
//   
//////////////////////////////////////////////////////////////////////////////////////////////
//
// File			: Drivers_Core_Power_Management_V100.c
// Author(s)		: Fabian Kung
// Last modified	: 8 Nov 2014
// Toolsuites		: Microchip MPLAB-X IDE v2.10 or above
//                	  MPLAB XC16 C-Compiler v1.21 or above
//			

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one change folder
#include "osmain.h"


// NOTE: Public function prototypes are declared in the corresponding *.h file.

//
// --- PUBLIC VARIABLES ---
//
INT16 gnProcessorPower;
   // Constants for gnProcessorPower:
    #define     __CPU_NORMAL    1
    #define     __CPU_POWERDOWN  0
//
// --- PRIVATE FUNCTION PROTOTYPES ---
//

//
// --- PRIVATE VARIABLES ---
//

//
// --- Process Level Constants Definition --- 
//


///
/// Process name		: Proce_Core_Power_Control_Driver
///
/// Author			: Fabian Kung
///
/// Last modified		: 8 Nov 2014
///
/// Code Version		: 1.00
///

/// Processor			: dsPIC33EP256MU80X family.
///
/// Processor/System Resources 
/// PINS			: 1. PIN_DPSW (digital power switch control).
/// 
/// MODULES			: 1. 3.3V LDO and pushbutton switch circuitry (External).
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables            : gnProcessorPower

#ifdef 				  __OS_VER			// Check RTOS version compatibility.
	#if 			  __OS_VER < 1
		#error "Proce_ADC_Driver: Incompatible OS version"
	#endif
#else
	#error "Proce_ADC_Driver: An RTOS is required with this function"
#endif

/// Description	: 
/// This is a driver for core digital power management.
/// The micro-controller receives power from a LDO voltage regulator with
/// ON/&OFF& control.  Normally this pin is pulled to GND and connected to a
/// pushbutton switch, called the "Power" switch. When the user presses the
/// power switch, the LDO will supply power to the micro-controller and the
/// system will start up.  In order to continue to function even after the
/// user releases the pushbutton switch, the micro-controller will assert
/// the ON/&OFF& control to logic high state. In addition, the
/// micro-controller will:
/// 1. Monitors the state of the Power switch.
/// 2. If the user presses the Power switch again for longer than 2 seconds,
/// this is recognized as shut down command and the system will shut itself
/// down by asserting the ON/&OFF& control to GND.
///
/// An I/O pin of the micro-controller will be dedicated to this driver.  This
/// pin, denoted PIN_DPSW will be bi-directional.  It is input when polling the
/// Power switch, and becomes output when asserting the ON/&OFF& control on
/// the LDO voltage regulator.  A large capacitor is connect to the ON/&OFF&
/// control, so the PIN_DPSW pin can momentarily become input to check the
/// status of the Power switch.
///

void Proce_Core_Power_Control_Driver(TASK_ATTRIBUTE *ptrTask)
{
    static int nCount = 0;
    
    if (ptrTask->nTimer == 0)
    {
        switch (ptrTask->nState)
	{
            case 0: // State 0 - Power up, assert ON/&OFF& control pin (PIN_DPSW)
                    // to logic high.
                TRISBbits.TRISB8 = 0;   // Set RB8 (PIN_DPSW) as output.
                PIN_DPSW = 1;
                PIN_DPSW = 1;
                gnProcessorPower = __CPU_NORMAL;        // Inform all other process
                                                        // that CPU is power up.
                OSSetTaskContext(ptrTask, 1, 100);
                break;

            case 1: // State 1 - Change PIN_DPSW pin to input and delay.  Allowing
                    // a short delay for the voltage on the pin to drop
                    // before starts sampling.
                TRISBbits.TRISB8 = 1;   // Set RB8 as input.
                TRISBbits.TRISB8 = 1;   // Set RB8 as input.
                OSSetTaskContext(ptrTask, 2, 250000/__SYSTEMTICK_US); // Delay for 0.25 second.
                break;

            case 2: // State 2 - Check input for Power push button pressed.
                if (PIN_DPSW == 1)      // Power switch is pressed.
                {                       // Delay for another and check again.
                    nCount++;           // Counter will be incremented
                                        // continously if Power switch is pressed.
                    if (nCount > 3)     // Check if counter threshold is exceeded.
                    {
                        OSSetTaskContext(ptrTask, 3, 100000/__SYSTEMTICK_US);  // Delay for 0.1 second.
                    }
                    else
                    {
                        OSSetTaskContext(ptrTask, 0, 100000/__SYSTEMTICK_US);
                    }
                }
                else
                {
                    nCount = 0;             // Reset counter if Power switch is
                                            // not pressed.
                    OSSetTaskContext(ptrTask, 0, 100000/__SYSTEMTICK_US);
                }
                break;

            case 3: // State 3 - Issue shutdown notice to all processes.
                PIN_ILED2 = 1;          // Lights up indicator LED2.
                gnProcessorPower = __CPU_POWERDOWN;
                OSSetTaskContext(ptrTask, 4, 300000/__SYSTEMTICK_US);
                break;

            case 4: // State 4 - Shut down system.
                TRISBbits.TRISB8 = 0;   // Set RB8 as output.
                PIN_DPSW = 0;           // Turn off system.
                PIN_DPSW = 0;
                OSSetTaskContext(ptrTask, 4, 100);
                break;

            default:
		OSSetTaskContext(ptrTask, 0, 1); // Back to state = 0, timer = 1.
            break;
	}
    }
}

 

