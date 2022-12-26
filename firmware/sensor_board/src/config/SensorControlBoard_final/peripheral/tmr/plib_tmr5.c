/*******************************************************************************
  TMR Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_tmr5.c

  Summary
    TMR5 peripheral library source file.

  Description
    This file implements the interface to the TMR peripheral library.  This
    library provides access to and control of the associated peripheral
    instance.

*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
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

#include "device.h"
#include "plib_tmr5.h"

static TMR_TIMER_OBJECT tmr5Obj;


void TMR5_Initialize(void)
{
    /* Disable Timer */
    T5CONCLR = _T5CON_ON_MASK;

    /*
    SIDL = 0
    SYNC = 0
    TGATE = 0
    TCKPS =0
    T32   = 0
    TCS = 1
    */
    T5CONSET = 0x2;

    /* Clear counter */
    TMR5 = 0x0;

    /*Set period */
    PR5 = 6000U;

    IEC0SET = _IEC0_T5IE_MASK;

}


void TMR5_Start(void)
{
    T5CONSET = _T5CON_ON_MASK;
}


void TMR5_Stop (void)
{
    T5CONCLR = _T5CON_ON_MASK;
}

void TMR5_PeriodSet(uint16_t period)
{
    PR5  = period;
}

uint16_t TMR5_PeriodGet(void)
{
    return (uint16_t)PR5;
}

uint16_t TMR5_CounterGet(void)
{
    return (uint16_t)(TMR5);
}


uint32_t TMR5_FrequencyGet(void)
{
    return (600000);
}

void TIMER_5_InterruptHandler (void)
{
    uint32_t status = IFS0bits.T5IF;
    IFS0CLR = _IFS0_T5IF_MASK;

    if((tmr5Obj.callback_fn != NULL))
    {
        tmr5Obj.callback_fn(status, tmr5Obj.context);
    }
}


void TMR5_InterruptEnable(void)
{

    IEC0SET = _IEC0_T5IE_MASK;
}


void TMR5_InterruptDisable(void)
{
    IEC0CLR = _IEC0_T5IE_MASK;
}


void TMR5_CallbackRegister( TMR_CALLBACK callback_fn, uintptr_t context )
{
    /* Save callback_fn and context in local memory */
    tmr5Obj.callback_fn = callback_fn;
    tmr5Obj.context = context;
}
