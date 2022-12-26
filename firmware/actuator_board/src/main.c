/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    can_t msg;
    
    msg.length = 0;

  
    /* Initialize all modules */
    SYS_Initialize ( NULL );

    
    VacuumPump_Add(PUMP_1, SWITCH_1, 1);
    VacuumPump_Add(PUMP_2, SWITCH_2, 2);
    VacuumPump_Add(PUMP_3, SWITCH_3, 3);
    VacuumPump_Add(PUMP_4, SWITCH_4, 4);
    VacuumPump_Add(PUMP_5, SWITCH_5, 5);
    VacuumPump_Add(PUMP_6, SWITCH_6, 6);
    
    
    AX12_SpeedInit(100);
    
    
    while ( true )
    {            
        msg.ID = 0;
        msg.length = 0;
        for(int i = 0; i < 8; i++)
            msg.data[i] = 0;
        
        while( !CAN4_MessageReceive(&msg.ID, &msg.length, msg.data, 0, 1, &msg.msgAttr) ) ;
        //UART6_Write(&msg.ID, 2);
        
        if ( AX12_OnMessage(msg) == 0) continue;

        if ( VacuumPump_OnMessage(msg) == 0) continue;

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

