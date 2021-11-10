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

#include <stdio.h>
#include <stdint.h>

#include "motor.h"
#include "control_system.h"
#include "protocol.h"
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int fputc(int ch, FILE *f)
{
    while (UART3_WriteIsBusy());
    UART3_Write(&ch, 1);

    return 0;
}

int32_t prev_left = 0, prev_right = 0;

int main(void)
{
    /* Initialize all modules */
    SYS_Initialize(NULL);
    CORETIMER_Start();

    QEI3_Start();   // odometry left
    QEI1_Start();   // odometry right
    
    motor_init();   
    control_system_initRegulators();
    TMR3_Start();   // interrupt for control system (regulation)
    

    while (true)
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks();
        
        /* Poll for new CANbus messages*/
        uint32_t id;
        uint8_t length;
        uint8_t data_rcv[8];
        uint16_t timestamp;
        CAN_MSG_RX_ATTRIBUTE msgAttr;
        
        bool status = false;
        while ((status = CAN4_MessageReceive(&id, &length, data_rcv, &timestamp, 1, &msgAttr)))
        {
            if (status == true)
            {
                protocol_parse_msg(id, length, data_rcv);
                status = false;
                /*if (id == 0x00000500)
                {
                    float kpl, kil, kdl, kpr, kir, kdr;
                    kpl = control_system_getKpLeft();
                    kil = control_system_getKiLeft();
                    kdl = control_system_getKdLeft();
                    
                    kpr = control_system_getKpRight();
                    kir = control_system_getKiRight();
                    kdr = control_system_getKdRight();
                    
                    printf("L P: %f\tI: %f\tD: %f\n\r", kpl, kil, kdl);
                    printf("R P: %f\tI: %f\tD: %f\n\r", kpr, kir, kdr);
                }*/
            }
        }
     
    }


    /* Execution should not come here during normal operation */



    return ( EXIT_FAILURE);
}


/*******************************************************************************
 End of File
 */

