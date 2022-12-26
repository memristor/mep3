/*******************************************************************************
  Controller Area Network (CAN) Peripheral Library Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_can4.c

  Summary:
    CAN peripheral library interface.

  Description:
    This file defines the interface to the CAN peripheral library. This
    library provides access to and control of the associated peripheral
    instance.

  Remarks:
    None.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END
// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include <sys/kmem.h>
#include "plib_can4.h"

// *****************************************************************************
// *****************************************************************************
// Global Data
// *****************************************************************************
// *****************************************************************************
/* Number of configured FIFO */
#define CAN_NUM_OF_FIFO             2
/* Maximum number of CAN Message buffers in each FIFO */
#define CAN_FIFO_MESSAGE_BUFFER_MAX 32

#define CAN_CONFIGURATION_MODE      0x4
#define CAN_OPERATION_MODE          0x0
#define CAN_NUM_OF_FILTER           2
/* FIFO Offset in word (4 bytes) */
#define CAN_FIFO_OFFSET             0x10
/* Filter Offset in word (4 bytes) */
#define CAN_FILTER_OFFSET           0x4
/* Acceptance Mask Offset in word (4 bytes) */
#define CAN_ACCEPTANCE_MASK_OFFSET  0x4
#define CAN_MESSAGE_RAM_CONFIG_SIZE 33
#define CAN_MSG_IDE_MASK            0x10000000
#define CAN_MSG_SID_MASK            0x7FF
#define CAN_MSG_TIMESTAMP_MASK      0xFFFF0000
#define CAN_MSG_EID_MASK            0x1FFFFFFF
#define CAN_MSG_DLC_MASK            0xF
#define CAN_MSG_RTR_MASK            0x200
#define CAN_MSG_SRR_MASK            0x20000000

static CAN_TX_RX_MSG_BUFFER __attribute__((coherent, aligned(32))) can_message_buffer[CAN_MESSAGE_RAM_CONFIG_SIZE];

// *****************************************************************************
// *****************************************************************************
// CAN4 PLib Interface Routines
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
/* Function:
    void CAN4_Initialize(void)

   Summary:
    Initializes given instance of the CAN peripheral.

   Precondition:
    None.

   Parameters:
    None.

   Returns:
    None
*/
void CAN4_Initialize(void)
{
    /* Switch the CAN module ON */
    C4CONSET = _C4CON_ON_MASK;

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    C4CON = (C4CON & ~_C4CON_REQOP_MASK) | ((CAN_CONFIGURATION_MODE << _C4CON_REQOP_POSITION) & _C4CON_REQOP_MASK);
    while(((C4CON & _C4CON_OPMOD_MASK) >> _C4CON_OPMOD_POSITION) != CAN_CONFIGURATION_MODE);

    /* Set the Bitrate to 500 Kbps */
    C4CFG = ((4 << _C4CFG_BRP_POSITION) & _C4CFG_BRP_MASK)
                            | ((2 << _C4CFG_SJW_POSITION) & _C4CFG_SJW_MASK)
                            | ((2 << _C4CFG_SEG2PH_POSITION) & _C4CFG_SEG2PH_MASK)
                            | ((6 << _C4CFG_SEG1PH_POSITION) & _C4CFG_SEG1PH_MASK)
                            | ((0 << _C4CFG_PRSEG_POSITION) & _C4CFG_PRSEG_MASK)
                            | _C4CFG_SEG2PHTS_MASK;

    /* Set FIFO base address for all message buffers */
    C4FIFOBA = (uint32_t)KVA_TO_PA(can_message_buffer);

    /* Configure CAN FIFOs */
    C4FIFOCON0 = (((1 - 1) << _C4FIFOCON0_FSIZE_POSITION) & _C4FIFOCON0_FSIZE_MASK) | _C4FIFOCON0_TXEN_MASK | ((0x0 << _C4FIFOCON0_TXPRI_POSITION) & _C4FIFOCON0_TXPRI_MASK) | ((0x0 << _C4FIFOCON0_RTREN_POSITION) & _C4FIFOCON0_RTREN_MASK);
    C4FIFOCON1 = (((32 - 1) << _C4FIFOCON1_FSIZE_POSITION) & _C4FIFOCON1_FSIZE_MASK);

    /* Configure CAN Filters */
    C4RXF0 = (27648 & _C4RXF0_EID_MASK) | (((27648 & 0x1FFC0000u) >> 18u) << _C4RXF0_SID_POSITION) | _C4RXF0_EXID_MASK;
    C4FLTCON0SET = ((0x1 << _C4FLTCON0_FSEL0_POSITION) & _C4FLTCON0_FSEL0_MASK)
                                                         | ((0x0 << _C4FLTCON0_MSEL0_POSITION) & _C4FLTCON0_MSEL0_MASK)| _C4FLTCON0_FLTEN0_MASK;
    C4RXF1 = (27664 & _C4RXF1_EID_MASK) | (((27664 & 0x1FFC0000u) >> 18u) << _C4RXF1_SID_POSITION) | _C4RXF1_EXID_MASK;
    C4FLTCON0SET = ((0x1 << _C4FLTCON0_FSEL1_POSITION) & _C4FLTCON0_FSEL1_MASK)
                                                         | ((0x1 << _C4FLTCON0_MSEL1_POSITION) & _C4FLTCON0_MSEL1_MASK)| _C4FLTCON0_FLTEN1_MASK;

    /* Configure CAN Acceptance Filter Masks */
    C4RXM0 = (536870911 & _C4RXM0_EID_MASK) | (((536870911 & 0x1FFC0000u) >> 18u) << _C4RXM0_SID_POSITION) | _C4RXM0_MIDE_MASK;
    C4RXM1 = (536870904 & _C4RXM1_EID_MASK) | (((536870904 & 0x1FFC0000u) >> 18u) << _C4RXM1_SID_POSITION) | _C4RXM1_MIDE_MASK;

    /* Switch the CAN module to CAN_OPERATION_MODE. Wait until the switch is complete */
    C4CON = (C4CON & ~_C4CON_REQOP_MASK) | ((CAN_OPERATION_MODE << _C4CON_REQOP_POSITION) & _C4CON_REQOP_MASK);
    while(((C4CON & _C4CON_OPMOD_MASK) >> _C4CON_OPMOD_POSITION) != CAN_OPERATION_MODE);
}

// *****************************************************************************
/* Function:
    bool CAN4_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum, CAN_MSG_TX_ATTRIBUTE msgAttr)

   Summary:
    Transmits a message into CAN bus.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - 11-bit / 29-bit identifier (ID).
    length      - length of data buffer in number of bytes.
    data        - pointer to source data buffer
    fifoNum     - FIFO number
    msgAttr     - Data frame or Remote frame to be transmitted

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum, CAN_MSG_TX_ATTRIBUTE msgAttr)
{
    CAN_TX_RX_MSG_BUFFER *txMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((fifoNum > (CAN_NUM_OF_FIFO - 1)) || (data == NULL))
    {
        return status;
    }

    if ((*(volatile uint32_t *)(&C4FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C4FIFOINT0_TXNFULLIF_MASK) == _C4FIFOINT0_TXNFULLIF_MASK)
    {
        txMessage = (CAN_TX_RX_MSG_BUFFER *)PA_TO_KVA1(*(volatile uint32_t *)(&C4FIFOUA0 + (fifoNum * CAN_FIFO_OFFSET)));

        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CAN_MSG_SID_MASK)
        {
            txMessage->msgSID = (id & CAN_MSG_EID_MASK) >> 18;
            txMessage->msgEID = ((id & 0x3FFFF) << 10) | CAN_MSG_IDE_MASK;
        }
        else
        {
            txMessage->msgSID = id;
            txMessage->msgEID = 0;
        }

        if (msgAttr == CAN_MSG_TX_REMOTE_FRAME)
        {
            txMessage->msgEID |= CAN_MSG_RTR_MASK;
        }
        else
        {
            if (length > 8)
            {
                length = 8;
            }
            txMessage->msgEID |= length;

            while(count < length)
            {
                txMessage->msgData[count++] = *data++;
            }
        }

        /* Request the transmit */
        *(volatile uint32_t *)(&C4FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C4FIFOCON0_UINC_MASK;
        *(volatile uint32_t *)(&C4FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C4FIFOCON0_TXREQ_MASK;

        status = true;
    }
    return status;
}

// *****************************************************************************
/* Function:
    bool CAN4_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint16_t *timestamp, uint8_t fifoNum, CAN_MSG_RX_ATTRIBUTE *msgAttr)

   Summary:
    Receives a message from CAN bus.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - Pointer to 11-bit / 29-bit identifier (ID) to be received.
    length      - Pointer to data length in number of bytes to be received.
    data        - Pointer to destination data buffer
    timestamp   - Pointer to Rx message timestamp, timestamp value is 0 if Timestamp is disabled in C4CON
    fifoNum     - FIFO number
    msgAttr     - Data frame or Remote frame to be received

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint16_t *timestamp, uint8_t fifoNum, CAN_MSG_RX_ATTRIBUTE *msgAttr)
{
    CAN_TX_RX_MSG_BUFFER *rxMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((fifoNum > (CAN_NUM_OF_FIFO - 1)) || (data == NULL) || (length == NULL) || (id == NULL))
    {
        return status;
    }

    /* Check if there is a message available in FIFO */
    if ((*(volatile uint32_t *)(&C4FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C4FIFOINT0_RXNEMPTYIF_MASK) == _C4FIFOINT0_RXNEMPTYIF_MASK)
    {
        /* Get a pointer to RX message buffer */
        rxMessage = (CAN_TX_RX_MSG_BUFFER *)PA_TO_KVA1(*(volatile uint32_t *)(&C4FIFOUA0 + (fifoNum * CAN_FIFO_OFFSET)));

        /* Check if it's a extended message type */
        if (rxMessage->msgEID & CAN_MSG_IDE_MASK)
        {
            *id = ((rxMessage->msgSID & CAN_MSG_SID_MASK) << 18) | ((rxMessage->msgEID >> 10) & _C4RXM0_EID_MASK);
            if (rxMessage->msgEID & CAN_MSG_RTR_MASK)
            {
                *msgAttr = CAN_MSG_RX_REMOTE_FRAME;
            }
            else
            {
                *msgAttr = CAN_MSG_RX_DATA_FRAME;
            }
        }
        else
        {
            *id = rxMessage->msgSID & CAN_MSG_SID_MASK;
            if (rxMessage->msgEID & CAN_MSG_SRR_MASK)
            {
                *msgAttr = CAN_MSG_RX_REMOTE_FRAME;
            }
            else
            {
                *msgAttr = CAN_MSG_RX_DATA_FRAME;
            }
        }

        *length = rxMessage->msgEID & CAN_MSG_DLC_MASK;

        /* Copy the data into the payload */
        while (count < *length)
        {
           *data++ = rxMessage->msgData[count++];
        }

        if (timestamp != NULL)
        {
            *timestamp = (rxMessage->msgSID & CAN_MSG_TIMESTAMP_MASK) >> 16;
        }

        /* Message processing is done, update the message buffer pointer. */
        *(volatile uint32_t *)(&C4FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C4FIFOCON0_UINC_MASK;

        /* Message is processed successfully, so return true */
        status = true;
    }

    return status;
}

// *****************************************************************************
/* Function:
    void CAN4_MessageAbort(uint8_t fifoNum)

   Summary:
    Abort request for a FIFO.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoNum - FIFO number

   Returns:
    None.
*/
void CAN4_MessageAbort(uint8_t fifoNum)
{
    if (fifoNum > (CAN_NUM_OF_FIFO - 1))
    {
        return;
    }
    *(volatile uint32_t *)(&C4FIFOCON0CLR + (fifoNum * CAN_FIFO_OFFSET)) = _C4FIFOCON0_TXREQ_MASK;
}

// *****************************************************************************
/* Function:
    void CAN4_MessageAcceptanceFilterSet(uint8_t filterNum, uint32_t id)

   Summary:
    Set Message acceptance filter configuration.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    filterNum - Filter number
    id        - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN4_MessageAcceptanceFilterSet(uint8_t filterNum, uint32_t id)
{
    uint32_t filterEnableBit = 0;
    uint8_t filterRegIndex = 0;

    if (filterNum < CAN_NUM_OF_FILTER)
    {
        filterRegIndex = filterNum >> 2;
        filterEnableBit = (filterNum % 4 == 0)? _C4FLTCON0_FLTEN0_MASK : 1 << ((((filterNum % 4) + 1) * 8) - 1);

        *(volatile uint32_t *)(&C4FLTCON0CLR + (filterRegIndex * CAN_FILTER_OFFSET)) = filterEnableBit;

        if (id > CAN_MSG_SID_MASK)
        {
            *(volatile uint32_t *)(&C4RXF0 + (filterNum * CAN_FILTER_OFFSET)) = (id & _C4RXF0_EID_MASK)
                                                                           | (((id & 0x1FFC0000u) >> 18) << _C4RXF0_SID_POSITION)
                                                                           | _C4RXF0_EXID_MASK;
        }
        else
        {
            *(volatile uint32_t *)(&C4RXF0 + (filterNum * CAN_FILTER_OFFSET)) = (id & CAN_MSG_SID_MASK) << _C4RXF0_SID_POSITION;
        }
        *(volatile uint32_t *)(&C4FLTCON0SET + (filterRegIndex * CAN_FILTER_OFFSET)) = filterEnableBit;
    }
}

// *****************************************************************************
/* Function:
    uint32_t CAN4_MessageAcceptanceFilterGet(uint8_t filterNum)

   Summary:
    Get Message acceptance filter configuration.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    filterNum - Filter number

   Returns:
    Returns Message acceptance filter identifier
*/
uint32_t CAN4_MessageAcceptanceFilterGet(uint8_t filterNum)
{
    uint32_t id = 0;

    if (filterNum < CAN_NUM_OF_FILTER)
    {
        if (*(volatile uint32_t *)(&C4RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C4RXF0_EXID_MASK)
        {
            id = (*(volatile uint32_t *)(&C4RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C4RXF0_EID_MASK)
               | ((*(volatile uint32_t *)(&C4RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C4RXF0_SID_MASK) >> 3);
        }
        else
        {
            id = (*(volatile uint32_t *)(&C4RXF0 + (filterNum * CAN_FILTER_OFFSET)) & _C4RXF0_SID_MASK) >> _C4RXF0_SID_POSITION;
        }
    }
    return id;
}

// *****************************************************************************
/* Function:
    void CAN4_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)

   Summary:
    Set Message acceptance filter mask configuration.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number (0 to 3)
    id                      - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN4_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)
{
    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    C4CON = (C4CON & ~_C4CON_REQOP_MASK) | ((CAN_CONFIGURATION_MODE << _C4CON_REQOP_POSITION) & _C4CON_REQOP_MASK);
    while(((C4CON & _C4CON_OPMOD_MASK) >> _C4CON_OPMOD_POSITION) != CAN_CONFIGURATION_MODE);

    if (id > CAN_MSG_SID_MASK)
    {
        *(volatile uint32_t *)(&C4RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) = (id & _C4RXM0_EID_MASK)
                                                                       | (((id & 0x1FFC0000u) >> 18) << _C4RXM0_SID_POSITION) | _C4RXM0_MIDE_MASK;
    }
    else
    {
        *(volatile uint32_t *)(&C4RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) = (id & CAN_MSG_SID_MASK) << _C4RXM0_SID_POSITION;
    }

    /* Switch the CAN module to CAN_OPERATION_MODE. Wait until the switch is complete */
    C4CON = (C4CON & ~_C4CON_REQOP_MASK) | ((CAN_OPERATION_MODE << _C4CON_REQOP_POSITION) & _C4CON_REQOP_MASK);
    while(((C4CON & _C4CON_OPMOD_MASK) >> _C4CON_OPMOD_POSITION) != CAN_OPERATION_MODE);
}

// *****************************************************************************
/* Function:
    uint32_t CAN4_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)

   Summary:
    Get Message acceptance filter mask configuration.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number (0 to 3)

   Returns:
    Returns Message acceptance filter mask.
*/
uint32_t CAN4_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)
{
    uint32_t id = 0;

    if (*(volatile uint32_t *)(&C4RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C4RXM0_MIDE_MASK)
    {
        id = (*(volatile uint32_t *)(&C4RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C4RXM0_EID_MASK)
           | ((*(volatile uint32_t *)(&C4RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C4RXM0_SID_MASK) >> 3);
    }
    else
    {
        id = (*(volatile uint32_t *)(&C4RXM0 + (acceptanceFilterMaskNum * CAN_ACCEPTANCE_MASK_OFFSET)) & _C4RXM0_SID_MASK) >> _C4RXM0_SID_POSITION;
    }
    return id;
}

// *****************************************************************************
/* Function:
    CAN_ERROR CAN4_ErrorGet(void)

   Summary:
    Returns the error during transfer.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    None.

   Returns:
    Error during transfer.
*/
CAN_ERROR CAN4_ErrorGet(void)
{
    CAN_ERROR error = CAN_ERROR_NONE;
    uint32_t errorStatus = C4TREC;

    /* Check if error occurred */
    error = (CAN_ERROR)((errorStatus & _C4TREC_EWARN_MASK) |
                        (errorStatus & _C4TREC_RXWARN_MASK) |
                        (errorStatus & _C4TREC_TXWARN_MASK) |
                        (errorStatus & _C4TREC_RXBP_MASK) |
                        (errorStatus & _C4TREC_TXBP_MASK) |
                        (errorStatus & _C4TREC_TXBO_MASK));

    return error;
}

// *****************************************************************************
/* Function:
    void CAN4_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount)

   Summary:
    Returns the transmit and receive error count during transfer.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    txErrorCount - Transmit Error Count to be received
    rxErrorCount - Receive Error Count to be received

   Returns:
    None.
*/
void CAN4_ErrorCountGet(uint8_t *txErrorCount, uint8_t *rxErrorCount)
{
    *txErrorCount = (uint8_t)((C4TREC & _C4TREC_TERRCNT_MASK) >> _C4TREC_TERRCNT_POSITION);
    *rxErrorCount = (uint8_t)(C4TREC & _C4TREC_RERRCNT_MASK);
}

// *****************************************************************************
/* Function:
    bool CAN4_InterruptGet(uint8_t fifoNum, CAN_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)

   Summary:
    Returns the FIFO Interrupt status.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoNum               - FIFO number
    fifoInterruptFlagMask - FIFO interrupt flag mask

   Returns:
    true - Requested fifo interrupt is occurred.
    false - Requested fifo interrupt is not occurred.
*/
bool CAN4_InterruptGet(uint8_t fifoNum, CAN_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)
{
    if (fifoNum > (CAN_NUM_OF_FIFO - 1))
    {
        return false;
    }
    return ((*(volatile uint32_t *)(&C4FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & fifoInterruptFlagMask) != 0x0);
}

// *****************************************************************************
/* Function:
    bool CAN4_TxFIFOIsFull(uint8_t fifoNum)

   Summary:
    Returns true if Tx FIFO is full otherwise false.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoNum - FIFO number

   Returns:
    true  - Tx FIFO is full.
    false - Tx FIFO is not full.
*/
bool CAN4_TxFIFOIsFull(uint8_t fifoNum)
{
    return ((*(volatile uint32_t *)(&C4FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C4FIFOINT0_TXNFULLIF_MASK) != _C4FIFOINT0_TXNFULLIF_MASK);
}

// *****************************************************************************
/* Function:
    bool CAN4_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)

   Summary:
    Set the Auto RTR response for remote transmit request.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.
    Auto RTR Enable must be set to 0x1 for the requested Transmit FIFO in MHC configuration.

   Parameters:
    id          - 11-bit / 29-bit identifier (ID).
    length      - length of data buffer in number of bytes.
    data        - pointer to source data buffer
    fifoNum     - FIFO number

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)
{
    CAN_TX_RX_MSG_BUFFER *txMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((*(volatile uint32_t *)(&C4FIFOINT0 + (fifoNum * CAN_FIFO_OFFSET)) & _C4FIFOINT0_TXNFULLIF_MASK) == _C4FIFOINT0_TXNFULLIF_MASK)
    {
        txMessage = (CAN_TX_RX_MSG_BUFFER *)PA_TO_KVA1(*(volatile uint32_t *)(&C4FIFOUA0 + (fifoNum * CAN_FIFO_OFFSET)));

        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CAN_MSG_SID_MASK)
        {
            txMessage->msgSID = (id & CAN_MSG_EID_MASK) >> 18;
            txMessage->msgEID = ((id & 0x3FFFF) << 10) | CAN_MSG_IDE_MASK;
        }
        else
        {
            txMessage->msgSID = id;
            txMessage->msgEID = 0;
        }

        if (length > 8)
        {
            length = 8;
        }
        txMessage->msgEID |= length;

        while(count < length)
        {
            txMessage->msgData[count++] = *data++;
        }

        /* Set UINC to respond to RTR */
        *(volatile uint32_t *)(&C4FIFOCON0SET + (fifoNum * CAN_FIFO_OFFSET)) = _C4FIFOCON0_UINC_MASK;

        status = true;
    }
    return status;
}

