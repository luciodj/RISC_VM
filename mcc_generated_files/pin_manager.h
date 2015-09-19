/**
  @Generated Pin Manager Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    pin_manager.h

  @Summary:
    This is the Pin Manager file generated using MPLAB� Code Configurator

  @Description:
    This header file provides implementations for pin APIs for all pins selected in the GUI.
    Generation Information :
        Product Revision  :  MPLAB� Code Configurator - v2.25.2
        Device            :  PIC16F1716
        Version           :  1.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 v1.34
        MPLAB             :  MPLAB X v2.35 or v3.00
 */

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */

#ifndef PIN_MANAGER_H
#define PIN_MANAGER_H

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set TX aliases
#define TX_TRIS               TRISC4
#define TX_LAT                LATC4
#define TX_PORT               RC4
#define TX_WPU                WPUC4
#define TX_ANS                ANSC4
#define TX_SetHigh()    do { LATC4 = 1; } while(0)
#define TX_SetLow()   do { LATC4 = 0; } while(0)
#define TX_Toggle()   do { LATC4 = ~LATC4; } while(0)
#define TX_GetValue()         RC4
#define TX_SetDigitalInput()    do { TRISC4 = 1; } while(0)
#define TX_SetDigitalOutput()   do { TRISC4 = 0; } while(0)

#define TX_SetPullup()    do { WPUC4 = 1; } while(0)
#define TX_ResetPullup()   do { WPUC4 = 0; } while(0)
#define TX_SetAnalogMode()   do { ANSC4 = 1; } while(0)
#define TX_SetDigitalMode()   do { ANSC4 = 0; } while(0)
// get/set RX aliases
#define RX_TRIS               TRISC5
#define RX_LAT                LATC5
#define RX_PORT               RC5
#define RX_WPU                WPUC5
#define RX_ANS                ANSC5
#define RX_SetHigh()    do { LATC5 = 1; } while(0)
#define RX_SetLow()   do { LATC5 = 0; } while(0)
#define RX_Toggle()   do { LATC5 = ~LATC5; } while(0)
#define RX_GetValue()         RC5
#define RX_SetDigitalInput()    do { TRISC5 = 1; } while(0)
#define RX_SetDigitalOutput()   do { TRISC5 = 0; } while(0)

#define RX_SetPullup()    do { WPUC5 = 1; } while(0)
#define RX_ResetPullup()   do { WPUC5 = 0; } while(0)
#define RX_SetAnalogMode()   do { ANSC5 = 1; } while(0)
#define RX_SetDigitalMode()   do { ANSC5 = 0; } while(0)

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    GPIO and peripheral I/O initialization
 * @Example
    PIN_MANAGER_Initialize();
 */
void PIN_MANAGER_Initialize(void);

/**
 * @Param
    none
 * @Returns
    none
 * @Description
    Interrupt on Change Handling routine
 * @Example
    PIN_MANAGER_IOC();
 */
void PIN_MANAGER_IOC(void);

#endif // PIN_MANAGER_H
/**
 End of File
 */