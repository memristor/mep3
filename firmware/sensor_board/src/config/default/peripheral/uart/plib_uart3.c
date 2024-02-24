/*******************************************************************************
  UART3 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart3.c

  Summary:
    UART3 PLIB Implementation File

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
#include "plib_uart3.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART3 Implementation
// *****************************************************************************
// *****************************************************************************


void static UART3_ErrorClear( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint8_t dummyData = 0u;

    errors = (UART_ERROR)(U3STA & (_U3STA_OERR_MASK | _U3STA_FERR_MASK | _U3STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        /* If it's a overrun error then clear it to flush FIFO */
        if(U3STA & _U3STA_OERR_MASK)
        {
            U3STACLR = _U3STA_OERR_MASK;
        }

        /* Read existing error bytes from FIFO to clear parity and framing error flags */
        while(U3STA & _U3STA_URXDA_MASK)
        {
            dummyData = U3RXREG;
        }

    }

    // Ignore the warning
    (void)dummyData;
}

void UART3_Initialize( void )
{
    /* Set up UxMODE bits */
    /* STSEL  = 0*/
    /* PDSEL = 0 */
    /* BRGH = 1 */
    /* RXINV = 0 */
    /* ABAUD = 0 */
    /* LPBACK = 0 */
    /* WAKE = 0 */
    /* SIDL = 0 */
    /* RUNOVF = 0 */
    /* CLKSEL = 0 */
    /* SLPEN = 0 */
    /* UEN = 0 */
    U3MODE = 0x8;

    /* Enable UART3 Receiver and Transmitter */
    U3STASET = (_U3STA_UTXEN_MASK | _U3STA_URXEN_MASK );

    /* BAUD Rate register Setup */
    U3BRG = 129;

    /* Turn ON UART3 */
    U3MODESET = _U3MODE_ON_MASK;
}

bool UART3_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    uint32_t status_ctrl;
    bool brgh = 1;
    int32_t uxbrg = 0;

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if ((baud == 0) || ((setup->dataWidth == UART_DATA_9_BIT) && (setup->parity != UART_PARITY_NONE)))
        {
            return status;
        }

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART3_FrequencyGet();
        }

        /* Calculate BRG value */
        if (brgh == 0)
        {
            uxbrg = (((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
        }
        else
        {
            uxbrg = (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;
        }

        /* Check if the baud value can be set with low baud settings */
        if((uxbrg < 0) || (uxbrg > UINT16_MAX))
        {
            return status;
        }

        /* Turn OFF UART3. Save UTXEN, URXEN and UTXBRK bits as these are cleared upon disabling UART */

        status_ctrl = U3STA & (_U3STA_UTXEN_MASK | _U3STA_URXEN_MASK | _U3STA_UTXBRK_MASK);

        U3MODECLR = _U3MODE_ON_MASK;

        if(setup->dataWidth == UART_DATA_9_BIT)
        {
            /* Configure UART3 mode */
            U3MODE = (U3MODE & (~_U3MODE_PDSEL_MASK)) | setup->dataWidth;
        }
        else
        {
            /* Configure UART3 mode */
            U3MODE = (U3MODE & (~_U3MODE_PDSEL_MASK)) | setup->parity;
        }

        /* Configure UART3 mode */
        U3MODE = (U3MODE & (~_U3MODE_STSEL_MASK)) | setup->stopBits;

        /* Configure UART3 Baud Rate */
        U3BRG = uxbrg;

        U3MODESET = _U3MODE_ON_MASK;

        /* Restore UTXEN, URXEN and UTXBRK bits. */
        U3STASET = status_ctrl;

        status = true;
    }

    return status;
}

bool UART3_Read(void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t* )buffer;
    uint32_t errorStatus = 0;
    size_t processedSize = 0;

    if(lBuffer != NULL)
    {

        /* Clear error flags and flush out error data that may have been received when no active request was pending */
        UART3_ErrorClear();

        while( size > processedSize )
        {
            while(!(U3STA & _U3STA_URXDA_MASK));

            /* Error status */
            errorStatus = (U3STA & (_U3STA_OERR_MASK | _U3STA_FERR_MASK | _U3STA_PERR_MASK));

            if(errorStatus != 0)
            {
                break;
            }
            if (( U3MODE & (_U3MODE_PDSEL0_MASK | _U3MODE_PDSEL1_MASK)) == (_U3MODE_PDSEL0_MASK | _U3MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                *(uint16_t*)lBuffer = (U3RXREG );
                lBuffer += 2;
            }
            else
            {
                /* 8-bit mode */
                *lBuffer++ = (U3RXREG );
            }

            processedSize++;
        }

        if(size == processedSize)
        {
            status = true;
        }
    }

    return status;
}

bool UART3_Write( void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t*)buffer;
    size_t processedSize = 0;

    if(lBuffer != NULL)
    {
        while( size > processedSize )
        {
            /* Wait while TX buffer is full */
            while (U3STA & _U3STA_UTXBF_MASK);

            if (( U3MODE & (_U3MODE_PDSEL0_MASK | _U3MODE_PDSEL1_MASK)) == (_U3MODE_PDSEL0_MASK | _U3MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                U3TXREG = *(uint16_t*)lBuffer;
                lBuffer += 2;
            }
            else
            {
                /* 8-bit mode */
                U3TXREG = *lBuffer++;
            }

            processedSize++;
        }

        status = true;
    }

    return status;
}

UART_ERROR UART3_ErrorGet( void )
{
    UART_ERROR errors = UART_ERROR_NONE;

    errors = (UART_ERROR)(U3STA & (_U3STA_OERR_MASK | _U3STA_FERR_MASK | _U3STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        UART3_ErrorClear();
    }

    /* All errors are cleared, but send the previous error state */
    return errors;
}

bool UART3_AutoBaudQuery( void )
{
    if(U3MODE & _U3MODE_ABAUD_MASK)
        return true;
    else
        return false;
}

void UART3_AutoBaudSet( bool enable )
{
    if( enable == true )
    {
        U3MODESET = _U3MODE_ABAUD_MASK;
    }

    /* Turning off ABAUD if it was on can lead to unpredictable behavior, so that
       direction of control is not allowed in this function.                      */
}

  
void UART3_WriteByte(int data)
{
    while ((U3STA & _U3STA_UTXBF_MASK));

    U3TXREG = data;
}

bool UART3_TransmitterIsReady( void )
{
    bool status = false;

    if(!(U3STA & _U3STA_UTXBF_MASK))
    {
        status = true;
    }

    return status;
}

int UART3_ReadByte( void )
{
    return(U3RXREG);
}

bool UART3_ReceiverIsReady( void )
{
    bool status = false;

    if(_U3STA_URXDA_MASK == (U3STA & _U3STA_URXDA_MASK))
    {
        status = true;
    }

    return status;
}

bool UART3_TransmitComplete( void )
{
    bool transmitComplete = false;

    if((U3STA & _U3STA_TRMT_MASK))
    {
        transmitComplete = true;
    }

    return transmitComplete;
}