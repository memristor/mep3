/*******************************************************************************
  UART6 PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_uart6.c

  Summary:
    UART6 PLIB Implementation File

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
#include "plib_uart6.h"

// *****************************************************************************
// *****************************************************************************
// Section: UART6 Implementation
// *****************************************************************************
// *****************************************************************************

UART_OBJECT uart6Obj;

void static UART6_ErrorClear( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint8_t dummyData = 0u;

    errors = (UART_ERROR)(U6STA & (_U6STA_OERR_MASK | _U6STA_FERR_MASK | _U6STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        /* If it's a overrun error then clear it to flush FIFO */
        if(U6STA & _U6STA_OERR_MASK)
        {
            U6STACLR = _U6STA_OERR_MASK;
        }

        /* Read existing error bytes from FIFO to clear parity and framing error flags */
        while(U6STA & _U6STA_URXDA_MASK)
        {
            dummyData = U6RXREG;
        }

        /* Clear error interrupt flag */
        IFS5CLR = _IFS5_U6EIF_MASK;

        /* Clear up the receive interrupt flag so that RX interrupt is not
         * triggered for error bytes */
        IFS5CLR = _IFS5_U6RXIF_MASK;
    }

    // Ignore the warning
    (void)dummyData;
}

void UART6_Initialize( void )
{
    /* Set up UxMODE bits */
    /* STSEL  = 0*/
    /* PDSEL = 0 */
    /* BRGH = 0 */
    /* RXINV = 0 */
    /* ABAUD = 0 */
    /* LPBACK = 0 */
    /* WAKE = 0 */
    /* SIDL = 0 */
    /* RUNOVF = 0 */
    /* CLKSEL = 0 */
    /* SLPEN = 0 */
    /* UEN = 0 */
    U6MODE = 0x0;

    /* Enable UART6 Receiver, Transmitter and TX Interrupt selection */
    U6STASET = (_U6STA_UTXEN_MASK | _U6STA_URXEN_MASK | _U6STA_UTXISEL1_MASK );

    /* BAUD Rate register Setup */
    U6BRG = 64;

    /* Disable Interrupts */
    IEC5CLR = _IEC5_U6EIE_MASK;

    IEC5CLR = _IEC5_U6RXIE_MASK;

    IEC5CLR = _IEC5_U6TXIE_MASK;

    /* Initialize instance object */
    uart6Obj.rxBuffer = NULL;
    uart6Obj.rxSize = 0;
    uart6Obj.rxProcessedSize = 0;
    uart6Obj.rxBusyStatus = false;
    uart6Obj.rxCallback = NULL;
    uart6Obj.txBuffer = NULL;
    uart6Obj.txSize = 0;
    uart6Obj.txProcessedSize = 0;
    uart6Obj.txBusyStatus = false;
    uart6Obj.txCallback = NULL;
    uart6Obj.errors = UART_ERROR_NONE;

    /* Turn ON UART6 */
    U6MODESET = _U6MODE_ON_MASK;
}

bool UART6_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    int32_t brgValHigh = 0;
    int32_t brgValLow = 0;
    uint32_t brgVal = 0;
    uint32_t uartMode;

    if((uart6Obj.rxBusyStatus == true) || (uart6Obj.txBusyStatus == true))
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if (baud == 0)
        {
            return status;
        }

        /* Turn OFF UART6 */
        U6MODECLR = _U6MODE_ON_MASK;

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART6_FrequencyGet();
        }

        /* Calculate BRG value */
        brgValLow = (((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
        brgValHigh = (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;

        /* Check if the baud value can be set with low baud settings */
        if((brgValLow >= 0) && (brgValLow <= UINT16_MAX))
        {
            brgVal =  brgValLow;
            U6MODECLR = _U6MODE_BRGH_MASK;
        }
        else if ((brgValHigh >= 0) && (brgValHigh <= UINT16_MAX))
        {
            brgVal = brgValHigh;
            U6MODESET = _U6MODE_BRGH_MASK;
        }
        else
        {
            return status;
        }

        if(setup->dataWidth == UART_DATA_9_BIT)
        {
            if(setup->parity != UART_PARITY_NONE)
            {
               return status;
            }
            else
            {
               /* Configure UART6 mode */
               uartMode = U6MODE;
               uartMode &= ~_U6MODE_PDSEL_MASK;
               U6MODE = uartMode | setup->dataWidth;
            }
        }
        else
        {
            /* Configure UART6 mode */
            uartMode = U6MODE;
            uartMode &= ~_U6MODE_PDSEL_MASK;
            U6MODE = uartMode | setup->parity ;
        }

        /* Configure UART6 mode */
        uartMode = U6MODE;
        uartMode &= ~_U6MODE_STSEL_MASK;
        U6MODE = uartMode | setup->stopBits ;

        /* Configure UART6 Baud Rate */
        U6BRG = brgVal;

        U6MODESET = _U6MODE_ON_MASK;

        status = true;
    }

    return status;
}

bool UART6_Read(void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t* )buffer;

    if(lBuffer != NULL)
    {
        /* Check if receive request is in progress */
        if(uart6Obj.rxBusyStatus == false)
        {
            /* Clear error flags and flush out error data that may have been received when no active request was pending */
            UART6_ErrorClear();

            uart6Obj.rxBuffer = lBuffer;
            uart6Obj.rxSize = size;
            uart6Obj.rxProcessedSize = 0;
            uart6Obj.rxBusyStatus = true;
            uart6Obj.errors = UART_ERROR_NONE;
            status = true;

            /* Enable UART6_FAULT Interrupt */
            IEC5SET = _IEC5_U6EIE_MASK;

            /* Enable UART6_RX Interrupt */
            IEC5SET = _IEC5_U6RXIE_MASK;
        }
    }

    return status;
}

bool UART6_Write( void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t*)buffer;

    if(lBuffer != NULL)
    {
        /* Check if transmit request is in progress */
        if(uart6Obj.txBusyStatus == false)
        {
            uart6Obj.txBuffer = lBuffer;
            uart6Obj.txSize = size;
            uart6Obj.txProcessedSize = 0;
            uart6Obj.txBusyStatus = true;
            status = true;

            /* Initiate the transfer by writing as many bytes as we can */
             while((!(U6STA & _U6STA_UTXBF_MASK)) && (uart6Obj.txSize > uart6Obj.txProcessedSize) )
            {
                if (( U6MODE & (_U6MODE_PDSEL0_MASK | _U6MODE_PDSEL1_MASK)) == (_U6MODE_PDSEL0_MASK | _U6MODE_PDSEL1_MASK))
                {
                    /* 9-bit mode */
                    U6TXREG = ((uint16_t*)uart6Obj.txBuffer)[uart6Obj.txProcessedSize++];
                }
                else
                {
                    /* 8-bit mode */
                    U6TXREG = uart6Obj.txBuffer[uart6Obj.txProcessedSize++];
                }
            }

            IEC5SET = _IEC5_U6TXIE_MASK;
        }
    }

    return status;
}

UART_ERROR UART6_ErrorGet( void )
{
    UART_ERROR errors = uart6Obj.errors;

    uart6Obj.errors = UART_ERROR_NONE;

    /* All errors are cleared, but send the previous error state */
    return errors;
}

bool UART6_AutoBaudQuery( void )
{
    if(U6MODE & _U6MODE_ABAUD_MASK)
        return true;
    else
        return false;
}

void UART6_AutoBaudSet( bool enable )
{
    if( enable == true )
    {
        U6MODESET = _U6MODE_ABAUD_MASK;
    }

    /* Turning off ABAUD if it was on can lead to unpredictable behavior, so that
       direction of control is not allowed in this function.                      */
}

void UART6_ReadCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart6Obj.rxCallback = callback;

    uart6Obj.rxContext = context;
}

bool UART6_ReadIsBusy( void )
{
    return uart6Obj.rxBusyStatus;
}

size_t UART6_ReadCountGet( void )
{
    return uart6Obj.rxProcessedSize;
}

bool UART6_ReadAbort(void)
{
    if (uart6Obj.rxBusyStatus == true)
    {
        /* Disable the fault interrupt */
        IEC5CLR = _IEC5_U6EIE_MASK;

        /* Disable the receive interrupt */
        IEC5CLR = _IEC5_U6RXIE_MASK;

        uart6Obj.rxBusyStatus = false;

        /* If required application should read the num bytes processed prior to calling the read abort API */
        uart6Obj.rxSize = uart6Obj.rxProcessedSize = 0;
    }

    return true;
}

void UART6_WriteCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart6Obj.txCallback = callback;

    uart6Obj.txContext = context;
}

bool UART6_WriteIsBusy( void )
{
    return uart6Obj.txBusyStatus;
}

size_t UART6_WriteCountGet( void )
{
    return uart6Obj.txProcessedSize;
}

void UART6_FAULT_InterruptHandler (void)
{
    /* Save the error to be reported later */
    uart6Obj.errors = (UART_ERROR)(U6STA & (_U6STA_OERR_MASK | _U6STA_FERR_MASK | _U6STA_PERR_MASK));

    /* Disable the fault interrupt */
    IEC5CLR = _IEC5_U6EIE_MASK;

    /* Disable the receive interrupt */
    IEC5CLR = _IEC5_U6RXIE_MASK;

    /* Clear size and rx status */
    uart6Obj.rxBusyStatus = false;

    UART6_ErrorClear();

    /* Client must call UARTx_ErrorGet() function to get the errors */
    if( uart6Obj.rxCallback != NULL )
    {
        uart6Obj.rxCallback(uart6Obj.rxContext);
    }
}

void UART6_RX_InterruptHandler (void)
{
    if(uart6Obj.rxBusyStatus == true)
    {
        /* Clear UART6 RX Interrupt flag */
        IFS5CLR = _IFS5_U6RXIF_MASK;

        while((_U6STA_URXDA_MASK == (U6STA & _U6STA_URXDA_MASK)) && (uart6Obj.rxSize > uart6Obj.rxProcessedSize) )
        {
            if (( U6MODE & (_U6MODE_PDSEL0_MASK | _U6MODE_PDSEL1_MASK)) == (_U6MODE_PDSEL0_MASK | _U6MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                ((uint16_t*)uart6Obj.rxBuffer)[uart6Obj.rxProcessedSize++] = (uint16_t )(U6RXREG);
            }
            else
            {
                /* 8-bit mode */
                uart6Obj.rxBuffer[uart6Obj.rxProcessedSize++] = (uint8_t )(U6RXREG);
            }
        }


        /* Check if the buffer is done */
        if(uart6Obj.rxProcessedSize >= uart6Obj.rxSize)
        {
            uart6Obj.rxBusyStatus = false;

            /* Disable the fault interrupt */
            IEC5CLR = _IEC5_U6EIE_MASK;

            /* Disable the receive interrupt */
            IEC5CLR = _IEC5_U6RXIE_MASK;

            if(uart6Obj.rxCallback != NULL)
            {
                uart6Obj.rxCallback(uart6Obj.rxContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}

void UART6_TX_InterruptHandler (void)
{
    if(uart6Obj.txBusyStatus == true)
    {
        /* Clear UART6TX Interrupt flag */
        IFS5CLR = _IFS5_U6TXIF_MASK;

        while((!(U6STA & _U6STA_UTXBF_MASK)) && (uart6Obj.txSize > uart6Obj.txProcessedSize) )
        {
            if (( U6MODE & (_U6MODE_PDSEL0_MASK | _U6MODE_PDSEL1_MASK)) == (_U6MODE_PDSEL0_MASK | _U6MODE_PDSEL1_MASK))
            {
                /* 9-bit mode */
                U6TXREG = ((uint16_t*)uart6Obj.txBuffer)[uart6Obj.txProcessedSize++];
            }
            else
            {
                /* 8-bit mode */
                U6TXREG = uart6Obj.txBuffer[uart6Obj.txProcessedSize++];
            }
        }


        /* Check if the buffer is done */
        if(uart6Obj.txProcessedSize >= uart6Obj.txSize)
        {
            uart6Obj.txBusyStatus = false;

            /* Disable the transmit interrupt, to avoid calling ISR continuously */
            IEC5CLR = _IEC5_U6TXIE_MASK;

            if(uart6Obj.txCallback != NULL)
            {
                uart6Obj.txCallback(uart6Obj.txContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}


