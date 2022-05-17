#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes

#include <stdio.h>
#include <stdint.h>

#include "motor.h"
#include "control.h"
#include "protocol.h"

// printf over UART3
int fputc(int ch, FILE *f)
{
    while (UART3_WriteIsBusy());
    UART3_Write(&ch, 1);

    return 0;
}


int main(void)
{
    /* Initialize all modules */
    SYS_Initialize(NULL);
    CORETIMER_Start();

    QEI3_Start();   // odometry left
    QEI1_Start();   // odometry right
    
    motor_init();   
    control_init();
    TMR3_Start();   // interrupt for control system (regulation)
    TMR4_Start();   // encoder report interrupt (100 Hz)
    

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
                protocol_process_msg(id, length, data_rcv);
                status = false;
            }
        }
     
    }


    /* Execution should not come here during normal operation */



    return ( EXIT_FAILURE);
}


/*******************************************************************************
 End of File
 */

