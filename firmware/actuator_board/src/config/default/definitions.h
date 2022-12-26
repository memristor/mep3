/*******************************************************************************
  System Definitions

  File Name:
    definitions.h

  Summary:
    project system definitions.

  Description:
    This file contains the system-wide prototypes and definitions for a project.

 *******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "peripheral/coretimer/plib_coretimer.h"
#include "peripheral/uart/plib_uart3.h"
#include "peripheral/can/plib_can4.h"
#include "peripheral/clk/plib_clk.h"
#include "peripheral/gpio/plib_gpio.h"
#include "peripheral/evic/plib_evic.h"
#include "peripheral/uart/plib_uart6.h"

#include "pumps.h"
//#include "ax12.h"


#define VACUUM_PUMP_CANID 0x00006C10
#define AX12_CANID 0x00006C00


#define RD5 LATDbits.LATD5
#define RC8 LATCbits.LATC8
#define RC6 LATCbits.LATC6
#define RB9 LATBbits.LATB9
#define RD6 LATDbits.LATD6
#define RC7 LATCbits.LATC7

#define RG6 LATGbits.LATG6
#define RB14 LATBbits.LATB14
#define RA12 LATAbits.LATA12
#define RG9 LATGbits.LATG9
#define RA0 LATAbits.LATA0
#define RA11 LATAbits.LATA11


#define SERVO_SELECT LATEbits.LATE13


#define PUMP_1 1
#define PUMP_2 2
#define PUMP_3 3
#define PUMP_4 4
#define PUMP_5 5
#define PUMP_6 6

#define SWITCH_1 1
#define SWITCH_2 2
#define SWITCH_3 3
#define SWITCH_4 4
#define SWITCH_5 5
#define SWITCH_6 6


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END

/* CPU clock frequency */
#define CPU_CLOCK_FREQUENCY 120000000

// *****************************************************************************
// *****************************************************************************
// Section: System Functions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* System Initialization Function

  Function:
    void SYS_Initialize( void *data )

  Summary:
    Function that initializes all modules in the system.

  Description:
    This function initializes all modules in the system, including any drivers,
    services, middleware, and applications.

  Precondition:
    None.

  Parameters:
    data            - Pointer to the data structure containing any data
                      necessary to initialize the module. This pointer may
                      be null if no data is required and default initialization
                      is to be used.

  Returns:
    None.

  Example:
    <code>
    SYS_Initialize ( NULL );

    while ( true )
    {
        SYS_Tasks ( );
    }
    </code>

  Remarks:
    This function will only be called once, after system reset.
*/

void SYS_Initialize( void *data );

/* Nullify SYS_Tasks() if only PLIBs are used. */
#define     SYS_Tasks()

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************




//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

#endif /* DEFINITIONS_H */
/*******************************************************************************
 End of File
*/
