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
#include "I2C_lcd.h"
#include "can.h"
#include "sensor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

//#define USE_LCD

typedef enum {WAIT_CAN_MSG, PARSE_CAN_MSG} state_t;

#ifdef USE_LCD
static void lcd_init(void);
static void lcd_show_points(uint16_t points);
#endif

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );

#ifdef USE_LCD    
    lcd_init();
#endif
    sens_init();
    
    can_t msg;
    state_t state = WAIT_CAN_MSG;
    memset(&msg, 0, sizeof(can_t)); 
    
    TMR2_Stop();
    TMR2_InterruptDisable();
    TMR2_CallbackRegister((TMR_CALLBACK)cinch_callback, 0);
    
    while ( true )
    {
        
        switch(state)
        {
            case WAIT_CAN_MSG:
                if(CAN4_MessageReceive(&msg.ID, &msg.length, msg.data, 0, 2, &msg.msgAttr))
                {
                    state = PARSE_CAN_MSG;
                }
                break;
            case PARSE_CAN_MSG:
#ifdef USE_LCD
                if(msg.ID == 0x6D20)
                {
                    lcd_show_points(msg.data[1]<<8|msg.data[0]);
                }
#endif
                memset(&msg, 0, sizeof(can_t));
                state = WAIT_CAN_MSG;
                break;

            default:
                memset(&msg, 0, sizeof(can_t));
                state = WAIT_CAN_MSG;            
        }
        
    
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


#ifdef USE_LCD

static void lcd_init(void)
{
    PCF8574_LCDInit(LCDCursorTypeOff, 2, 16, 0x27);
    PCF8574_LCDClearScreen();
    PCF8574_LCDBackLightSet(true);
}

static void lcd_show_points(uint16_t points)
{
        PCF8574_LCDClearScreen();
        PCF8574_LCDGOTO(LCDLineNumberOne, 5);
        PCF8574_LCDPrintf("%u", points);
}

#endif
/*******************************************************************************
 End of File
*/

