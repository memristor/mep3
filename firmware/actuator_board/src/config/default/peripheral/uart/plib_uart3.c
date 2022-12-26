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

UART_RING_BUFFER_OBJECT uart3Obj;

#define UART3_READ_BUFFER_SIZE      256
#define UART3_READ_BUFFER_SIZE_9BIT (256 >> 1)
#define UART3_RX_INT_DISABLE()      IEC1CLR = _IEC1_U3RXIE_MASK;
#define UART3_RX_INT_ENABLE()       IEC1SET = _IEC1_U3RXIE_MASK;

static uint8_t UART3_ReadBuffer[UART3_READ_BUFFER_SIZE];

#define UART3_WRITE_BUFFER_SIZE     256
#define UART3_WRITE_BUFFER_SIZE_9BIT       (256 >> 1)
#define UART3_TX_INT_DISABLE()      IEC2CLR = _IEC2_U3TXIE_MASK;
#define UART3_TX_INT_ENABLE()       IEC2SET = _IEC2_U3TXIE_MASK;

static uint8_t UART3_WriteBuffer[UART3_WRITE_BUFFER_SIZE];

#define UART3_IS_9BIT_MODE_ENABLED()    ( U3MODE & (_U3MODE_PDSEL0_MASK | _U3MODE_PDSEL1_MASK)) == (_U3MODE_PDSEL0_MASK | _U3MODE_PDSEL1_MASK) ? true:false

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

        /* Clear error interrupt flag */
        IFS1CLR = _IFS1_U3EIF_MASK;

        /* Clear up the receive interrupt flag so that RX interrupt is not
         * triggered for error bytes */
        IFS1CLR = _IFS1_U3RXIF_MASK;

    }

    // Ignore the warning
    (void)dummyData;
}

void UART3_Initialize( void )
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
    U3MODE = 0x0;

    /* Enable UART3 Receiver, Transmitter and TX Interrupt selection */
    U3STASET = (_U3STA_UTXEN_MASK | _U3STA_URXEN_MASK | _U3STA_UTXISEL1_MASK );

    /* BAUD Rate register Setup */
    U3BRG = 64;

    IEC2CLR = _IEC2_U3TXIE_MASK;

    /* Initialize instance object */
    uart3Obj.rdCallback = NULL;
    uart3Obj.rdInIndex = 0;
    uart3Obj.rdOutIndex = 0;
    uart3Obj.isRdNotificationEnabled = false;
    uart3Obj.isRdNotifyPersistently = false;
    uart3Obj.rdThreshold = 0;

    uart3Obj.wrCallback = NULL;
    uart3Obj.wrInIndex = 0;
    uart3Obj.wrOutIndex = 0;
    uart3Obj.isWrNotificationEnabled = false;
    uart3Obj.isWrNotifyPersistently = false;
    uart3Obj.wrThreshold = 0;

    uart3Obj.errors = UART_ERROR_NONE;

    if (UART3_IS_9BIT_MODE_ENABLED())
    {
        uart3Obj.rdBufferSize = UART3_READ_BUFFER_SIZE_9BIT;
        uart3Obj.wrBufferSize = UART3_WRITE_BUFFER_SIZE_9BIT;
    }
    else
    {
        uart3Obj.rdBufferSize = UART3_READ_BUFFER_SIZE;
        uart3Obj.wrBufferSize = UART3_WRITE_BUFFER_SIZE;
    }


    /* Turn ON UART3 */
    U3MODESET = _U3MODE_ON_MASK;

    /* Enable UART3_FAULT Interrupt */
    IEC1SET = _IEC1_U3EIE_MASK;

    /* Enable UART3_RX Interrupt */
    IEC1SET = _IEC1_U3RXIE_MASK;
}

bool UART3_SerialSetup( UART_SERIAL_SETUP *setup, uint32_t srcClkFreq )
{
    bool status = false;
    uint32_t baud;
    int32_t brgValHigh = 0;
    int32_t brgValLow = 0;
    uint32_t brgVal = 0;
    uint32_t uartMode;

    if (setup != NULL)
    {
        baud = setup->baudRate;

        if (baud == 0)
        {
            return status;
        }

        /* Turn OFF UART3 */
        U3MODECLR = _U3MODE_ON_MASK;

        if(srcClkFreq == 0)
        {
            srcClkFreq = UART3_FrequencyGet();
        }

        /* Calculate BRG value */
        brgValLow = (((srcClkFreq >> 4) + (baud >> 1)) / baud ) - 1;
        brgValHigh = (((srcClkFreq >> 2) + (baud >> 1)) / baud ) - 1;

        /* Check if the baud value can be set with low baud settings */
        if((brgValLow >= 0) && (brgValLow <= UINT16_MAX))
        {
            brgVal =  brgValLow;
            U3MODECLR = _U3MODE_BRGH_MASK;
        }
        else if ((brgValHigh >= 0) && (brgValHigh <= UINT16_MAX))
        {
            brgVal = brgValHigh;
            U3MODESET = _U3MODE_BRGH_MASK;
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

        if (UART3_IS_9BIT_MODE_ENABLED())
        {
            uart3Obj.rdBufferSize = UART3_READ_BUFFER_SIZE_9BIT;
            uart3Obj.wrBufferSize = UART3_WRITE_BUFFER_SIZE_9BIT;
        }
        else
        {
            uart3Obj.rdBufferSize = UART3_READ_BUFFER_SIZE;
            uart3Obj.wrBufferSize = UART3_WRITE_BUFFER_SIZE;
        }

        U3MODESET = _U3MODE_ON_MASK;

        status = true;
    }

    return status;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static inline bool UART3_RxPushByte(uint16_t rdByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    tempInIndex = uart3Obj.rdInIndex + 1;

    if (tempInIndex >= uart3Obj.rdBufferSize)
    {
        tempInIndex = 0;
    }

    if (tempInIndex == uart3Obj.rdOutIndex)
    {
        /* Queue is full - Report it to the application. Application gets a chance to free up space by reading data out from the RX ring buffer */
        if(uart3Obj.rdCallback != NULL)
        {
            uart3Obj.rdCallback(UART_EVENT_READ_BUFFER_FULL, uart3Obj.rdContext);

            /* Read the indices again in case application has freed up space in RX ring buffer */
            tempInIndex = uart3Obj.rdInIndex + 1;

            if (tempInIndex >= uart3Obj.rdBufferSize)
            {
                tempInIndex = 0;
            }
        }
    }

    /* Attempt to push the data into the ring buffer */
    if (tempInIndex != uart3Obj.rdOutIndex)
    {
        if (UART3_IS_9BIT_MODE_ENABLED())
        {
            ((uint16_t*)&UART3_ReadBuffer)[uart3Obj.rdInIndex] = rdByte;
        }
        else
        {
            UART3_ReadBuffer[uart3Obj.rdInIndex] = (uint8_t)rdByte;
        }

        uart3Obj.rdInIndex = tempInIndex;

        isSuccess = true;
    }
    else
    {
        /* Queue is full. Data will be lost. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART3_ReadNotificationSend(void)
{
    uint32_t nUnreadBytesAvailable;

    if (uart3Obj.isRdNotificationEnabled == true)
    {
        nUnreadBytesAvailable = UART3_ReadCountGet();

        if(uart3Obj.rdCallback != NULL)
        {
            if (uart3Obj.isRdNotifyPersistently == true)
            {
                if (nUnreadBytesAvailable >= uart3Obj.rdThreshold)
                {
                    uart3Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart3Obj.rdContext);
                }
            }
            else
            {
                if (nUnreadBytesAvailable == uart3Obj.rdThreshold)
                {
                    uart3Obj.rdCallback(UART_EVENT_READ_THRESHOLD_REACHED, uart3Obj.rdContext);
                }
            }
        }
    }
}

size_t UART3_Read(uint8_t* pRdBuffer, const size_t size)
{
    size_t nBytesRead = 0;
    uint32_t rdOutIndex = 0;
    uint32_t rdInIndex = 0;

    /* Take a snapshot of indices to avoid creation of critical section */
    rdOutIndex = uart3Obj.rdOutIndex;
    rdInIndex = uart3Obj.rdInIndex;

    while (nBytesRead < size)
    {
        if (rdOutIndex != rdInIndex)
        {
            if (UART3_IS_9BIT_MODE_ENABLED())
            {
                ((uint16_t*)pRdBuffer)[nBytesRead++] = ((uint16_t*)&UART3_ReadBuffer)[rdOutIndex++];
            }
            else
            {
                pRdBuffer[nBytesRead++] = UART3_ReadBuffer[rdOutIndex++];
            }

            if (rdOutIndex >= uart3Obj.rdBufferSize)
            {
                rdOutIndex = 0;
            }
        }
        else
        {
            /* No more data available in the RX buffer */
            break;
        }
    }

    uart3Obj.rdOutIndex = rdOutIndex;

    return nBytesRead;
}

size_t UART3_ReadCountGet(void)
{
    size_t nUnreadBytesAvailable;
    uint32_t rdInIndex;
    uint32_t rdOutIndex;

    /* Take a snapshot of indices to avoid processing in critical section */
    rdInIndex = uart3Obj.rdInIndex;
    rdOutIndex = uart3Obj.rdOutIndex;

    if ( rdInIndex >=  rdOutIndex)
    {
        nUnreadBytesAvailable =  rdInIndex -  rdOutIndex;
    }
    else
    {
        nUnreadBytesAvailable =  (uart3Obj.rdBufferSize -  rdOutIndex) + rdInIndex;
    }

    return nUnreadBytesAvailable;
}

size_t UART3_ReadFreeBufferCountGet(void)
{
    return (uart3Obj.rdBufferSize - 1) - UART3_ReadCountGet();
}

size_t UART3_ReadBufferSizeGet(void)
{
    return (uart3Obj.rdBufferSize - 1);
}

bool UART3_ReadNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart3Obj.isRdNotificationEnabled;

    uart3Obj.isRdNotificationEnabled = isEnabled;

    uart3Obj.isRdNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART3_ReadThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart3Obj.rdThreshold = nBytesThreshold;
    }
}

void UART3_ReadCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart3Obj.rdCallback = callback;

    uart3Obj.rdContext = context;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static bool UART3_TxPullByte(uint16_t* pWrByte)
{
    bool isSuccess = false;
    uint32_t wrOutIndex = uart3Obj.wrOutIndex;
    uint32_t wrInIndex = uart3Obj.wrInIndex;

    if (wrOutIndex != wrInIndex)
    {
        if (UART3_IS_9BIT_MODE_ENABLED())
        {
            *pWrByte = ((uint16_t*)&UART3_WriteBuffer)[wrOutIndex++];
        }
        else
        {
            *pWrByte = UART3_WriteBuffer[wrOutIndex++];
        }

        if (wrOutIndex >= uart3Obj.wrBufferSize)
        {
            wrOutIndex = 0;
        }

        uart3Obj.wrOutIndex = wrOutIndex;

        isSuccess = true;
    }

    return isSuccess;
}

static inline bool UART3_TxPushByte(uint16_t wrByte)
{
    uint32_t tempInIndex;
    bool isSuccess = false;

    uint32_t wrOutIndex = uart3Obj.wrOutIndex;
    uint32_t wrInIndex = uart3Obj.wrInIndex;

    tempInIndex = wrInIndex + 1;

    if (tempInIndex >= uart3Obj.wrBufferSize)
    {
        tempInIndex = 0;
    }
    if (tempInIndex != wrOutIndex)
    {
        if (UART3_IS_9BIT_MODE_ENABLED())
        {
            ((uint16_t*)&UART3_WriteBuffer)[wrInIndex] = wrByte;
        }
        else
        {
            UART3_WriteBuffer[wrInIndex] = (uint8_t)wrByte;
        }

        uart3Obj.wrInIndex = tempInIndex;

        isSuccess = true;
    }
    else
    {
        /* Queue is full. Report Error. */
    }

    return isSuccess;
}

/* This routine is only called from ISR. Hence do not disable/enable USART interrupts. */
static void UART3_WriteNotificationSend(void)
{
    uint32_t nFreeWrBufferCount;

    if (uart3Obj.isWrNotificationEnabled == true)
    {
        nFreeWrBufferCount = UART3_WriteFreeBufferCountGet();

        if(uart3Obj.wrCallback != NULL)
        {
            if (uart3Obj.isWrNotifyPersistently == true)
            {
                if (nFreeWrBufferCount >= uart3Obj.wrThreshold)
                {
                    uart3Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart3Obj.wrContext);
                }
            }
            else
            {
                if (nFreeWrBufferCount == uart3Obj.wrThreshold)
                {
                    uart3Obj.wrCallback(UART_EVENT_WRITE_THRESHOLD_REACHED, uart3Obj.wrContext);
                }
            }
        }
    }
}

static size_t UART3_WritePendingBytesGet(void)
{
    size_t nPendingTxBytes;

    /* Take a snapshot of indices to avoid processing in critical section */

    uint32_t wrOutIndex = uart3Obj.wrOutIndex;
    uint32_t wrInIndex = uart3Obj.wrInIndex;

    if ( wrInIndex >=  wrOutIndex)
    {
        nPendingTxBytes =  wrInIndex - wrOutIndex;
    }
    else
    {
        nPendingTxBytes =  (uart3Obj.wrBufferSize -  wrOutIndex) + wrInIndex;
    }

    return nPendingTxBytes;
}

size_t UART3_WriteCountGet(void)
{
    size_t nPendingTxBytes;

    nPendingTxBytes = UART3_WritePendingBytesGet();

    return nPendingTxBytes;
}

size_t UART3_Write(uint8_t* pWrBuffer, const size_t size )
{
    size_t nBytesWritten  = 0;

    while (nBytesWritten < size)
    {
        if (UART3_IS_9BIT_MODE_ENABLED())
        {
            if (UART3_TxPushByte(((uint16_t*)pWrBuffer)[nBytesWritten]) == true)
            {
                nBytesWritten++;
            }
            else
            {
                /* Queue is full, exit the loop */
                break;
            }
        }
        else
        {
            if (UART3_TxPushByte(pWrBuffer[nBytesWritten]) == true)
            {
                nBytesWritten++;
            }
            else
            {
                /* Queue is full, exit the loop */
                break;
            }
        }

    }

    /* Check if any data is pending for transmission */
    if (UART3_WritePendingBytesGet() > 0)
    {
        /* Enable TX interrupt as data is pending for transmission */
        UART3_TX_INT_ENABLE();
    }

    return nBytesWritten;
}

size_t UART3_WriteFreeBufferCountGet(void)
{
    return (uart3Obj.wrBufferSize - 1) - UART3_WriteCountGet();
}

size_t UART3_WriteBufferSizeGet(void)
{
    return (uart3Obj.wrBufferSize - 1);
}

bool UART3_WriteNotificationEnable(bool isEnabled, bool isPersistent)
{
    bool previousStatus = uart3Obj.isWrNotificationEnabled;

    uart3Obj.isWrNotificationEnabled = isEnabled;

    uart3Obj.isWrNotifyPersistently = isPersistent;

    return previousStatus;
}

void UART3_WriteThresholdSet(uint32_t nBytesThreshold)
{
    if (nBytesThreshold > 0)
    {
        uart3Obj.wrThreshold = nBytesThreshold;
    }
}

void UART3_WriteCallbackRegister( UART_RING_BUFFER_CALLBACK callback, uintptr_t context)
{
    uart3Obj.wrCallback = callback;

    uart3Obj.wrContext = context;
}

UART_ERROR UART3_ErrorGet( void )
{
    UART_ERROR errors = uart3Obj.errors;

    uart3Obj.errors = UART_ERROR_NONE;

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

void UART3_FAULT_InterruptHandler (void)
{
    /* Save the error to be reported later */
    uart3Obj.errors = (UART_ERROR)(U3STA & (_U3STA_OERR_MASK | _U3STA_FERR_MASK | _U3STA_PERR_MASK));

    UART3_ErrorClear();

    /* Client must call UARTx_ErrorGet() function to clear the errors */
    if( uart3Obj.rdCallback != NULL )
    {
        uart3Obj.rdCallback(UART_EVENT_READ_ERROR, uart3Obj.rdContext);
    }
}

void UART3_RX_InterruptHandler (void)
{
    /* Clear UART3 RX Interrupt flag */
    IFS1CLR = _IFS1_U3RXIF_MASK;

    /* Keep reading until there is a character availabe in the RX FIFO */
    while((U3STA & _U3STA_URXDA_MASK) == _U3STA_URXDA_MASK)
    {
        if (UART3_RxPushByte(  (uint16_t )(U3RXREG) ) == true)
        {
            UART3_ReadNotificationSend();
        }
        else
        {
            /* UART RX buffer is full */
        }
    }
}

void UART3_TX_InterruptHandler (void)
{
    uint16_t wrByte;

    /* Check if any data is pending for transmission */
    if (UART3_WritePendingBytesGet() > 0)
    {
        /* Clear UART3TX Interrupt flag */
        IFS2CLR = _IFS2_U3TXIF_MASK;

        /* Keep writing to the TX FIFO as long as there is space */
        while(!(U3STA & _U3STA_UTXBF_MASK))
        {
            if (UART3_TxPullByte(&wrByte) == true)
            {
                if (UART3_IS_9BIT_MODE_ENABLED())
                {
                    U3TXREG = wrByte;
                }
                else
                {
                    U3TXREG = (uint8_t)wrByte;
                }

                /* Send notification */
                UART3_WriteNotificationSend();
            }
            else
            {
                /* Nothing to transmit. Disable the data register empty interrupt. */
                UART3_TX_INT_DISABLE();
                break;
            }
        }
    }
    else
    {
        /* Nothing to transmit. Disable the data register empty interrupt. */
        UART3_TX_INT_DISABLE();
    }
}

