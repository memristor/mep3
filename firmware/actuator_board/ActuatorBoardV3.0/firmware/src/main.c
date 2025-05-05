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

    
    while ( true )
    {
        memset(&msg, 0, sizeof(can_t)); 
        while( !CAN4_MessageReceive(&msg.ID, &msg.length, msg.data, 0, 2, &msg.msgAttr));
         
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        if(msg.ID == 0x00006C01)
        {
            set_pin_state(msg.data);
        }
        
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

