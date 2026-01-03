/*******************************************************************************
  Quadrature Encoder Interface (QEI1) Peripheral Library (PLIB)

  Company:
    Microchip Technology Inc.

  File Name:
    plib_qei1.c

  Summary:
    QEI1 Source File

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
#include "plib_qei1.h"

// *****************************************************************************

// *****************************************************************************
// Section: QEI1 Implementation
// *****************************************************************************
// *****************************************************************************


void QEI1_Initialize (void)
{

    /* QEI1CON register  */
    /*  CCM    = 0 */
    /*  GATEN  = 0 */
    /*  CNTPOL = 0 */
    /*  INTDIV = 0 */
    /*  IMV    = 0  */
    /*  PIMOD  = 0  */
    /*  QEISIDL = 0 */
    QEI1CON = 0x0;

    /* QEI1IOC register  */
    /*  QEAPOL    = 0 */
    /*  QEBPOL  = 0 */
    /*  IDXPOL = 0 */
    /*  HOMPOL = 0 */
    /*  SWPAB    = 0  */
    /*  OUTFNC  = 0  */
    /*  QFDIV   = 0   */
    /*  FLTREN  = 0   */
    QEI1IOC = 0x0;

    QEI1ICC = 0U;
    QEI1CMPL = 0U;

    /* QEI1STAT register  */
    /*  IDXIEN    = false */
    /*  HOMIEN  = false */
    /*  VELOVIEN = false */
    /*  POSOVIEN = false */
    /*  PCIIEN    = false  */
    /*  PCLEQIEN  = false    */
    /*  PCHEQIEN = false     */
    QEI1STAT = 0x0;

}


void QEI1_Start(void)
{
    /* Enable QEI channel */
    QEI1CON |= _QEI1CON_QEIEN_MASK;
}

void QEI1_Stop(void)
{
    /* Disable QEI channel */
    QEI1CON &= ~_QEI1CON_QEIEN_MASK;
}

uint32_t QEI1_PulseIntervalGet(void)
{
    return (INT1HLD);
}

void QEI1_PositionWindowSet(uint32_t high_threshold, uint32_t low_threshold)
{
    QEI1ICC  = high_threshold;
    QEI1CMPL = low_threshold;
}

void QEI1_PositionCountSet(uint32_t position_count)
{
    POS1CNT = position_count;
}

void QEI1_VelocityCountSet(uint32_t velocity_count)
{
    VEL1CNT = velocity_count;
}


