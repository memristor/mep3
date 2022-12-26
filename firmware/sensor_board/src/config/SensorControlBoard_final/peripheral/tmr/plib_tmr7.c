/*******************************************************************************
  TMR Peripheral Library Interface Source File

  Company
    Microchip Technology Inc.

  File Name
    plib_tmr7.c

  Summary
    TMR7 peripheral library source file.

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
#include "plib_tmr7.h"

static TMR_TIMER_OBJECT tmr7Obj;


void TMR7_Initialize(void)
{
    /* Disable Timer */
    T7CONCLR = _T7CON_ON_MASK;

    /*
    SIDL = 0
    SYNC = 0
    TGATE = 0
    TCKPS =0
    T32   = 0
    TCS = 1
    */
    T7CONSET = 0x2;

    /* Clear counter */
    TMR7 = 0x0;

    /*Set period */
    PR7 = 6000U;

    IEC2SET = _IEC2_T7IE_MASK;

}


void TMR7_Start(void)
{
    T7CONSET = _T7CON_ON_MASK;
}


void TMR7_Stop (void)
{
    T7CONCLR = _T7CON_ON_MASK;
}

void TMR7_PeriodSet(uint16_t period)
{
    PR7  = period;
}

uint16_t TMR7_PeriodGet(void)
{
    return (uint16_t)PR7;
}

uint16_t TMR7_CounterGet(void)
{
    return (uint16_t)(TMR7);
}


uint32_t TMR7_FrequencyGet(void)
{
    return (600000);
}

void TIMER_7_InterruptHandler (void)
{
    uint32_t status = IFS2bits.T7IF;
    IFS2CLR = _IFS2_T7IF_MASK;

    if((tmr7Obj.callback_fn != NULL))
    {
        tmr7Obj.callback_fn(status, tmr7Obj.context);
    }
}


void TMR7_InterruptEnable(void)
{

    IEC2SET = _IEC2_T7IE_MASK;
}


void TMR7_InterruptDisable(void)
{
    IEC2CLR = _IEC2_T7IE_MASK;
}


void TMR7_CallbackRegister( TMR_CALLBACK callback_fn, uintptr_t context )
{
    /* Save callback_fn and context in local memory */
    tmr7Obj.callback_fn = callback_fn;
    tmr7Obj.context = context;
}
