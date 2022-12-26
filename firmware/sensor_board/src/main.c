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
#include "definitions.h"
#include "config/SensorControlBoard_final/definitions.h"                // SYS function prototypes

#define ADCFreq 2 //frekvencija AD konvertora u Hz (max 100Hz)
#define CSFreq 1 //frekvencija senzora boje u Hz (max 100Hz)

volatile bool flag;

int interapt = 0;
// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

void integer_to_string(int i, char* broj)
{
    
    *(broj)=i/10000+48;
    *(broj+1)=(i%10000)/1000+48;
    *(broj+2)=(i%1000)/100+48;
    *(broj+3)=(i%100)/10+48;
    *(broj+4)=i%10+48;
    *(broj+5)='\n';
    *(broj+6)=0;
}

int main ( void )
{
    uint32_t addr;
    uint8_t len;
    uint8_t message;
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    TMR9_Start(); //global timer start
    
    
//    int CSPeriod = 100/CSFreq;
//    int ADCPeriod = 100/ADCFreq;
    
    while ( true )
    {
        
        /*if(flag)
        {
            interapt++;*/
            
            /*if(interapt % ADCPeriod == 0) //ADConversion
            {
                integer_to_string(ADConversion(3), niz);
                UART3_Write(niz, strlen(niz));
                while(UART3_WriteIsBusy());
                interapt = 0;
            }*/
            
            /*if(interapt % CSPeriod == 0) //read color sensor
            {
                integer_to_string(readSensor(5), niz);
                UART3_Write(niz, strlen(niz));
                while(UART3_WriteIsBusy());
                interapt = 0;
            }*/
            

           if(CAN4_MessageReceive(&addr, &len, &message, 0, 1));
            
           switch(addr)
           {
               case ADC_CAN_ID+0:
                    ADConversion(0);
                    break;
                case ADC_CAN_ID+1:
                    ADConversion(1);
                    break;
               case ADC_CAN_ID+2:
                    ADConversion(2);
                    break;
                case ADC_CAN_ID+3:
                    ADConversion(3);
                    break;
               case ADC_CAN_ID+4:
                    ADConversion(4);
                    break;
                case ADC_CAN_ID+5:
                    ADConversion(5);
                    break;
               case BINARY_SENSOR_CAN_ID+0:
                   binary_sensor_get_single(0);
                   break;
               case BINARY_SENSOR_CAN_ID+1:
                   binary_sensor_get_single(1);
                   break;
               case BINARY_SENSOR_CAN_ID+2:
                   binary_sensor_get_single(2);
                   break;
               case BINARY_SENSOR_CAN_ID+3:
                   binary_sensor_get_single(3);
                   break;
           }
           addr=0;
            
           binary_sensor_update_all();

    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

