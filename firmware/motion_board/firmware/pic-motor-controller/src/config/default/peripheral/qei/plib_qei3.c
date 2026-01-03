/*******************************************************************************
  Quadrature Encoder Interface (QEI3) Peripheral Library (PLIB)

  Company:
    Microchip Technology Inc.

  File Name:
    plib_qei3.c

  Summary:
    QEI3 Source File

  Description:
    None

*******************************************************************************/

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
#include "device.h"
#include "plib_qei3.h"

// *****************************************************************************

// *****************************************************************************
// Section: QEI3 Implementation
// *****************************************************************************
// *****************************************************************************


void QEI3_Initialize (void)
{

    /* QEI3CON register  */
    /*  CCM    = 0 */
    /*  GATEN  = 0 */
    /*  CNTPOL = 0 */
    /*  INTDIV = 0 */
    /*  IMV    = 0  */
    /*  PIMOD  = 0  */
    /*  QEISIDL = 0 */
    QEI3CON = 0x0;

    /* QEI3IOC register  */
    /*  QEAPOL    = 0 */
    /*  QEBPOL  = 0 */
    /*  IDXPOL = 0 */
    /*  HOMPOL = 0 */
    /*  SWPAB    = 0  */
    /*  OUTFNC  = 0  */
    /*  QFDIV   = 0   */
    /*  FLTREN  = 0   */
    QEI3IOC = 0x0;

    QEI3ICC = 0U;
    QEI3CMPL = 0U;

    /* QEI3STAT register  */
    /*  IDXIEN    = false */
    /*  HOMIEN  = false */
    /*  VELOVIEN = false */
    /*  POSOVIEN = false */
    /*  PCIIEN    = false  */
    /*  PCLEQIEN  = false    */
    /*  PCHEQIEN = false     */
    QEI3STAT = 0x0;

}


void QEI3_Start(void)
{
    /* Enable QEI channel */
    QEI3CON |= _QEI3CON_QEIEN_MASK;
}

void QEI3_Stop(void)
{
    /* Disable QEI channel */
    QEI3CON &= ~_QEI3CON_QEIEN_MASK;
}

uint32_t QEI3_PulseIntervalGet(void)
{
    return (INT3HLD);
}

void QEI3_PositionWindowSet(uint32_t high_threshold, uint32_t low_threshold)
{
    QEI3ICC  = high_threshold;
    QEI3CMPL = low_threshold;
}

void QEI3_PositionCountSet(uint32_t position_count)
{
    POS3CNT = position_count;
}

void QEI3_VelocityCountSet(uint32_t velocity_count)
{
    VEL3CNT = velocity_count;
}


