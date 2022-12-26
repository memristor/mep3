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

UART_OBJECT uart3Obj;

void static UART3_ErrorClear( void )
{
    /* rxBufferLen = (FIFO level + RX register) */
    uint8_t rxBufferLen = UART_RXFIFO_DEPTH;
    uint8_t dummyData = 0u;

    /* If it's a overrun error then clear it to flush FIFO */
    if(U3STA & _U3STA_OERR_MASK)
    {
        U3STACLR = _U3STA_OERR_MASK;
    }

    /* Read existing error bytes from FIFO to clear parity and framing error flags */
    while(U3STA & (_U3STA_FERR_MASK | _U3STA_PERR_MASK))
    {
        dummyData = (uint8_t )(U3RXREG );
        rxBufferLen--;

        /* Try to flush error bytes for one full FIFO and exit instead of
         * blocking here if more error bytes are received */
        if(rxBufferLen == 0u)
        {
            break;
        }
    }

    // Ignore the warning
    (void)dummyData;

    /* Clear error interrupt flag */
    IFS1CLR = _IFS1_U3EIF_MASK;

    /* Clear up the receive interrupt flag so that RX interrupt is not
     * triggered for error bytes */
    IFS1CLR = _IFS1_U3RXIF_MASK;

    return;
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
    U3MODE = 0x8;

    /* Enable UART3 Receiver, Transmitter and TX Interrupt selection */
    U3STASET = (_U3STA_UTXEN_MASK | _U3STA_URXEN_MASK | _U3STA_UTXISEL0_MASK);

    /* BAUD Rate register Setup */
    U3BRG = 129;

    /* Disable Interrupts */
    IEC1CLR = _IEC1_U3EIE_MASK;

    IEC1CLR = _IEC1_U3RXIE_MASK;

    IEC2CLR = _IEC2_U3TXIE_MASK;

    /* Initialize instance object */
    uart3Obj.rxBuffer = NULL;
    uart3Obj.rxSize = 0;
    uart3Obj.rxProcessedSize = 0;
    uart3Obj.rxBusyStatus = false;
    uart3Obj.rxCallback = NULL;
    uart3Obj.txBuffer = NULL;
    uart3Obj.txSize = 0;
    uart3Obj.txProcessedSize = 0;
    uart3Obj.txBusyStatus = false;
    uart3Obj.txCallback = NULL;

    /* Turn ON UART3 */
    U3MODESET = _U3MODE_ON_MASK;
}

bool UART3_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud = setup->baudRate;
    uint32_t brgValHigh = 0;
    uint32_t brgValLow = 0;
    uint32_t brgVal = 0;
    uint32_t uartMode;

    if((uart3Obj.rxBusyStatus == true) || (uart3Obj.txBusyStatus == true))
    {
        /* Transaction is in progress, so return without updating settings */
        return status;
    }

    if (setup != NULL)
    {
        /* Turn OFF UART3 */
        U3MODECLR = _U3MODE_ON_MASK;

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART3_FrequencyGet();
        }

        /* Calculate BRG value */
        brgValLow = ((srcClkFreq / baud) >> 4) - 1;
        brgValHigh = ((srcClkFreq / baud) >> 2) - 1;

        /* Check if the baud value can be set with low baud settings */
        if((brgValHigh >= 0) && (brgValHigh <= UINT16_MAX))
        {
            brgVal =  (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;
            U3MODESET = _U3MODE_BRGH_MASK;
        }
        else if ((brgValLow >= 0) && (brgValLow <= UINT16_MAX))
        {
            brgVal = ( ((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
            U3MODECLR = _U3MODE_BRGH_MASK;
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
               /* Configure UART3 mode */
               uartMode = U3MODE;
               uartMode &= ~_U3MODE_PDSEL_MASK;
               U3MODE = uartMode | setup->dataWidth;
            }
        }
        else
        {
            /* Configure UART3 mode */
            uartMode = U3MODE;
            uartMode &= ~_U3MODE_PDSEL_MASK;
            U3MODE = uartMode | setup->parity ;
        }

        /* Configure UART3 mode */
        uartMode = U3MODE;
        uartMode &= ~_U3MODE_STSEL_MASK;
        U3MODE = uartMode | setup->stopBits ;

        /* Configure UART3 Baud Rate */
        U3BRG = brgVal;

        U3MODESET = _U3MODE_ON_MASK;

        status = true;
    }

    return status;
}

bool UART3_Read(void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t* )buffer;

    if(lBuffer != NULL)
    {
        /* Check if receive request is in progress */
        if(uart3Obj.rxBusyStatus == false)
        {
            /* Clear errors before submitting the request.
             * ErrorGet clears errors internally. */
            UART3_ErrorGet();

            uart3Obj.rxBuffer = lBuffer;
            uart3Obj.rxSize = size;
            uart3Obj.rxProcessedSize = 0;
            uart3Obj.rxBusyStatus = true;
            status = true;

            /* Enable UART3_FAULT Interrupt */
            IEC1SET = _IEC1_U3EIE_MASK;

            /* Enable UART3_RX Interrupt */
            IEC1SET = _IEC1_U3RXIE_MASK;
        }
    }

    return status;
}

bool UART3_Write( void* buffer, const size_t size )
{
    bool status = false;
    uint8_t* lBuffer = (uint8_t*)buffer;

    if(lBuffer != NULL)
    {
        /* Check if transmit request is in progress */
        if(uart3Obj.txBusyStatus == false)
        {
            uart3Obj.txBuffer = lBuffer;
            uart3Obj.txSize = size;
            uart3Obj.txProcessedSize = 0;
            uart3Obj.txBusyStatus = true;
            status = true;

            /* Initiate the transfer by sending first byte */
            if(!(U3STA & _U3STA_UTXBF_MASK))
            {
                U3TXREG = *lBuffer;
                uart3Obj.txProcessedSize++;
            }

            IEC2SET = _IEC2_U3TXIE_MASK;
        }
    }

    return status;
}

UART_ERROR UART3_ErrorGet( void )
{
    UART_ERROR errors = UART_ERROR_NONE;
    uint32_t status = U3STA;

    errors = (UART_ERROR)(status & (_U3STA_OERR_MASK | _U3STA_FERR_MASK | _U3STA_PERR_MASK));

    if(errors != UART_ERROR_NONE)
    {
        UART3_ErrorClear();
    }

    /* All errors are cleared, but send the previous error state */
    return errors;
}

void UART3_ReadCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart3Obj.rxCallback = callback;

    uart3Obj.rxContext = context;
}

bool UART3_ReadIsBusy( void )
{
    return uart3Obj.rxBusyStatus;
}

size_t UART3_ReadCountGet( void )
{
    return uart3Obj.rxProcessedSize;
}

void UART3_WriteCallbackRegister( UART_CALLBACK callback, uintptr_t context )
{
    uart3Obj.txCallback = callback;

    uart3Obj.txContext = context;
}

bool UART3_WriteIsBusy( void )
{
    return uart3Obj.txBusyStatus;
}

size_t UART3_WriteCountGet( void )
{
    return uart3Obj.txProcessedSize;
}

void UART3_FAULT_InterruptHandler (void)
{
    /* Clear size and rx status */
    uart3Obj.rxBusyStatus = false;

    /* Disable the fault interrupt */
    IEC1CLR = _IEC1_U3EIE_MASK;
    /* Disable the receive interrupt */
    IEC1CLR = _IEC1_U3RXIE_MASK;

    /* Client must call UARTx_ErrorGet() function to clear the errors */
    if( uart3Obj.rxCallback != NULL )
    {
        uart3Obj.rxCallback(uart3Obj.rxContext);
    }
}

void UART3_RX_InterruptHandler (void)
{
    if(uart3Obj.rxBusyStatus == true)
    {
        while((_U3STA_URXDA_MASK == (U3STA & _U3STA_URXDA_MASK)) && (uart3Obj.rxSize > uart3Obj.rxProcessedSize) )
        {
            uart3Obj.rxBuffer[uart3Obj.rxProcessedSize++] = (uint8_t )(U3RXREG);
        }

        /* Clear UART3 RX Interrupt flag after reading data buffer */
        IFS1CLR = _IFS1_U3RXIF_MASK;

        /* Check if the buffer is done */
        if(uart3Obj.rxProcessedSize >= uart3Obj.rxSize)
        {
            uart3Obj.rxBusyStatus = false;

            /* Disable the receive interrupt */
            IEC1CLR = _IEC1_U3RXIE_MASK;

            if(uart3Obj.rxCallback != NULL)
            {
                uart3Obj.rxCallback(uart3Obj.rxContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}

void UART3_TX_InterruptHandler (void)
{
    if(uart3Obj.txBusyStatus == true)
    {
        while((!(U3STA & _U3STA_UTXBF_MASK)) && (uart3Obj.txSize > uart3Obj.txProcessedSize) )
        {
            U3TXREG = uart3Obj.txBuffer[uart3Obj.txProcessedSize++];
        }

        /* Clear UART3TX Interrupt flag after writing to buffer */
        IFS2CLR = _IFS2_U3TXIF_MASK;

        /* Check if the buffer is done */
        if(uart3Obj.txProcessedSize >= uart3Obj.txSize)
        {
            uart3Obj.txBusyStatus = false;

            /* Disable the transmit interrupt, to avoid calling ISR continuously */
            IEC2CLR = _IEC2_U3TXIE_MASK;

            if(uart3Obj.txCallback != NULL)
            {
                uart3Obj.txCallback(uart3Obj.txContext);
            }
        }
    }
    else
    {
        // Nothing to process
        ;
    }
}


