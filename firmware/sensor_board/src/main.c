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
#include "pin.h"
#include <xc.h>

// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

#define USE_LCD
#define USE_ACTUATOR_SHIELD

typedef enum {WAIT_CAN_MSG, PARSE_CAN_MSG_LCD, PARSE_CAN_MSG_PUMP} state_t;

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
    /*Dokaz da je ziva plocica*/
    uint8_t podaci[1];
    podaci[0] = 4;  // poslace broj 4 na CAN da signalizira da je LCD uspesno inicijalizovan
    CAN4_MessageTransmit((0x6D09 | 0x80000000),1,podaci, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);  
    /*Kraaaj dokaza*/
    
    sens_init();
    
    can_t msg;
    can_t msg2;
    state_t state = WAIT_CAN_MSG;
    memset(&msg, 0, sizeof(can_t));
    memset(&msg2, 0, sizeof(can_t));   
    
    
    TMR2_Stop();
    TMR2_InterruptDisable();
    TMR2_CallbackRegister((TMR_CALLBACK)cinch_callback, 0);
    
    while ( true )
    {     
        switch(state)
        {
            case WAIT_CAN_MSG:
               
#ifdef USE_LCD
                if(CAN4_MessageReceive(&msg.ID, &msg.length, msg.data, 0, 2, &msg.msgAttr))
                {
                    if(msg.ID == 0x6D20) {
                        state = PARSE_CAN_MSG_LCD;
                    }
                }          
#endif

#ifdef USE_ACTUATOR_SHIELD
                if(CAN4_MessageReceive(&msg2.ID, &msg2.length, msg2.data, 0, 3, &msg2.msgAttr))
                {
                    if(msg2.ID == 0x6C01){
                        state = PARSE_CAN_MSG_PUMP;
                    }
                }
#endif
                break;
            case PARSE_CAN_MSG_LCD:
#ifdef USE_LCD
                lcd_show_points(msg.data[1]<<8|msg.data[0]);
#endif
                memset(&msg, 0, sizeof(can_t));
                state = WAIT_CAN_MSG;
                break;
            case PARSE_CAN_MSG_PUMP:
                
                set_pin_state(msg2.data);
                
                memset(&msg2, 0, sizeof(can_t));
                state = WAIT_CAN_MSG;
                break;
            default:
                memset(&msg, 0, sizeof(can_t));
                memset(&msg2, 0, sizeof(can_t));
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
    PCF8574_LCDGOTO(LCDLineNumberOne, 5);
    PCF8574_LCDPrintf("%u", 0);
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

