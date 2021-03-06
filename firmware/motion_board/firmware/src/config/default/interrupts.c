/*******************************************************************************
 System Interrupts File

  Company:
    Microchip Technology Inc.

  File Name:
    interrupt.c

  Summary:
    Interrupt vectors mapping

  Description:
    This file maps all the interrupt vectors to their corresponding
    implementations. If a particular module interrupt is used, then its ISR
    definition can be found in corresponding PLIB source file. If a module
    interrupt is not used, then its ISR implementation is mapped to dummy
    handler.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "interrupts.h"
#include "definitions.h"

#include "motor.h"
#include "control.h"
#include "protocol.h"


// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************


void TIMER_2_InterruptHandler( void );
void TIMER_3_InterruptHandler( void );
void TIMER_4_InterruptHandler( void );
void UART3_FAULT_InterruptHandler( void );
void UART3_RX_InterruptHandler( void );
void UART3_TX_InterruptHandler( void );



/* All the handlers are defined here.  Each will call its PLIB-specific function. */
void __ISR(_TIMER_2_VECTOR, ipl4SOFT) TIMER_2_Handler (void)
{
    TIMER_2_InterruptHandler();
}

void __ISR(_TIMER_3_VECTOR, ipl2SOFT) TIMER_3_Handler (void)
{
    TIMER_3_InterruptHandler();
    control_interrupt();
}
    
void __ISR(_TIMER_4_VECTOR, ipl3SOFT) TIMER_4_Handler (void)
{
    TIMER_4_InterruptHandler();
    if (report_encoders)
        {
            const int32_t encoder_left = -(int32_t)QEI3_PositionGet(); 
            const int32_t encoder_right = (int32_t)QEI1_PositionGet();
            uint8_t buff[8];
            protocol_pack_int32(buff, encoder_left);
            protocol_pack_int32(buff + 4, encoder_right);
            can_send_quick(CAN_ENCODER_ID, 8, buff);
        }
}

void __ISR(_UART3_FAULT_VECTOR, ipl1SOFT) UART3_FAULT_Handler (void)
{
    UART3_FAULT_InterruptHandler();
}

void __ISR(_UART3_RX_VECTOR, ipl1SOFT) UART3_RX_Handler (void)
{
    UART3_RX_InterruptHandler();
}

void __ISR(_UART3_TX_VECTOR, ipl1SOFT) UART3_TX_Handler (void)
{
    UART3_TX_InterruptHandler();
}




/*******************************************************************************
 End of File
*/
