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
#include "config/default/definitions.h"
#include "pwm_turbine.h"          // SYS function prototypes

//sd1
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{

    GPIO_RA10_Set();
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    can_t msg;
    msg.length = 0;
    MCPWM_Start();
    while ( true )
    {
        
        memset(&msg, 0, sizeof(can_t)); 
        while( !CAN4_MessageReceive(&msg.ID, &msg.length, msg.data, 0, 2, &msg.msgAttr));
         
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        if(msg.ID == 0x00006C01)
        {
            set_pin_state(msg.data);
        }
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        if(msg.ID == 0x00006C02) //jedan smer turbine
        {
            set_turbine_pwm(msg.data);
        }   
        
        if(msg.ID == 0x00006C03)
        {
            if(msg.data[0]==1)
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6 , 360);
            else if(msg.data[0]==2)
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6 , 720);
            else if(msg.data[0]==3)
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6 , 1080);
            else
                MCPWM_ChannelPrimaryDutySet(MCPWM_CH_6 , 0);
        }
        
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

