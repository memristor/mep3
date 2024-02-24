/*******************************************************************************
  CANFD Peripheral Library Interface Source File

  Company:
    Microchip Technology Inc.

  File Name:
    plib_canfd4.c

  Summary:
    CANFD peripheral library interface.

  Description:
    This file defines the interface to the CANFD peripheral library. This
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
#include "plib_canfd4.h"


// *****************************************************************************
// *****************************************************************************
// Global Data
// *****************************************************************************
// *****************************************************************************
/* CAN4 Message memory size */
#define CANFD_MESSAGE_RAM_CONFIG_SIZE 1104
/* Number of configured FIFO */
#define CANFD_NUM_OF_FIFO             2
/* Maximum number of CAN Message buffers in each FIFO */
#define CANFD_FIFO_MESSAGE_BUFFER_MAX 32

#define CANFD_CONFIGURATION_MODE      0x4
#define CANFD_OPERATION_MODE          0x0
#define CANFD_NUM_OF_FILTER           1
/* FIFO Offset in word (4 bytes) */
#define CANFD_FIFO_OFFSET             0xc
/* Filter Offset in word (4 bytes) */
#define CANFD_FILTER_OFFSET           0x4
#define CANFD_FILTER_OBJ_OFFSET       0x8
/* Acceptance Mask Offset in word (4 bytes) */
#define CANFD_ACCEPTANCE_MASK_OFFSET  0x8
#define CANFD_MSG_SID_MASK            0x7FF
#define CANFD_MSG_EID_MASK            0x1FFFFFFF
#define CANFD_MSG_DLC_MASK            0x0000000F
#define CANFD_MSG_IDE_MASK            0x00000010
#define CANFD_MSG_RTR_MASK            0x00000020
#define CANFD_MSG_BRS_MASK            0x00000040
#define CANFD_MSG_FDF_MASK            0x00000080
#define CANFD_MSG_SEQ_MASK            0xFFFFFE00
#define CANFD_MSG_TX_EXT_SID_MASK     0x1FFC0000
#define CANFD_MSG_TX_EXT_EID_MASK     0x0003FFFF
#define CANFD_MSG_RX_EXT_SID_MASK     0x000007FF
#define CANFD_MSG_RX_EXT_EID_MASK     0x1FFFF800
#define CANFD_MSG_FLT_EXT_SID_MASK    0x1FFC0000
#define CANFD_MSG_FLT_EXT_EID_MASK    0x0003FFFF

static uint8_t __attribute__((coherent, aligned(16))) can_message_buffer[CANFD_MESSAGE_RAM_CONFIG_SIZE];
static const uint8_t dlcToLength[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/******************************************************************************
Local Functions
******************************************************************************/
static void CANLengthToDlcGet(uint8_t length, uint8_t *dlc)
{
    if (length <= 8)
    {
        *dlc = length;
    }
    else if (length <= 12)
    {
        *dlc = 0x9;
    }
    else if (length <= 16)
    {
        *dlc = 0xA;
    }
    else if (length <= 20)
    {
        *dlc = 0xB;
    }
    else if (length <= 24)
    {
        *dlc = 0xC;
    }
    else if (length <= 32)
    {
        *dlc = 0xD;
    }
    else if (length <= 48)
    {
        *dlc = 0xE;
    }
    else
    {
        *dlc = 0xF;
    }
}

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
    CFD4CON |= _CFD4CON_ON_MASK;

    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    CFD4CON = (CFD4CON & ~_CFD4CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD4CON_REQOP_POSITION) & _CFD4CON_REQOP_MASK);
    while(((CFD4CON & _CFD4CON_OPMOD_MASK) >> _CFD4CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE);

    /* Set the Data bitrate to 500 Kbps */
    CFD4DBTCFG = ((14 << _CFD4DBTCFG_BRP_POSITION) & _CFD4DBTCFG_BRP_MASK)
               | ((10 << _CFD4DBTCFG_TSEG1_POSITION) & _CFD4DBTCFG_TSEG1_MASK)
               | ((3 << _CFD4DBTCFG_TSEG2_POSITION) & _CFD4DBTCFG_TSEG2_MASK)
               | ((3 << _CFD4DBTCFG_SJW_POSITION) & _CFD4DBTCFG_SJW_MASK);

    /* Set the Nominal bitrate to 500 Kbps */
    CFD4NBTCFG = ((14 << _CFD4NBTCFG_BRP_POSITION) & _CFD4NBTCFG_BRP_MASK)
               | ((10 << _CFD4NBTCFG_TSEG1_POSITION) & _CFD4NBTCFG_TSEG1_MASK)
               | ((3 << _CFD4NBTCFG_TSEG2_POSITION) & _CFD4NBTCFG_TSEG2_MASK)
               | ((3 << _CFD4NBTCFG_SJW_POSITION) & _CFD4NBTCFG_SJW_MASK);

    /* Set Message memory base address for all FIFOs/Queue */
    CFD4FIFOBA = (uint32_t)KVA_TO_PA(can_message_buffer);

    /* Tx Event FIFO Configuration */
    CFD4TEFCON = (((1 - 1) << _CFD4TEFCON_FSIZE_POSITION) & _CFD4TEFCON_FSIZE_MASK);
    CFD4CON |= _CFD4CON_STEF_MASK;

    /* Tx Queue Configuration */
    CFD4TXQCON = (((1 - 1) << _CFD4TXQCON_FSIZE_POSITION) & _CFD4TXQCON_FSIZE_MASK)
               | ((0x7 << _CFD4TXQCON_PLSIZE_POSITION) & _CFD4TXQCON_PLSIZE_MASK)
               | ((0x0 << _CFD4TXQCON_TXPRI_POSITION) & _CFD4TXQCON_TXPRI_MASK);
    CFD4CON |= _CFD4CON_TXQEN_MASK;


    /* Configure CAN FIFOs */
    CFD4FIFOCON1 = (((32 - 1) << _CFD4FIFOCON1_FSIZE_POSITION) & _CFD4FIFOCON1_FSIZE_MASK) | _CFD4FIFOCON1_TXEN_MASK | ((0x0 << _CFD4FIFOCON1_TXPRI_POSITION) & _CFD4FIFOCON1_TXPRI_MASK) | ((0x0 << _CFD4FIFOCON1_RTREN_POSITION) & _CFD4FIFOCON1_RTREN_MASK) | ((0x0 << _CFD4FIFOCON1_PLSIZE_POSITION) & _CFD4FIFOCON1_PLSIZE_MASK);
    CFD4FIFOCON2 = (((32 - 1) << _CFD4FIFOCON2_FSIZE_POSITION) & _CFD4FIFOCON2_FSIZE_MASK) | ((0x0 << _CFD4FIFOCON2_PLSIZE_POSITION) & _CFD4FIFOCON2_PLSIZE_MASK);

    /* Configure CAN Filters */
    /* Filter 0 configuration */
    CFD4FLTOBJ0 = ((((27904 & CANFD_MSG_FLT_EXT_SID_MASK) >> 18) | ((27904 & CANFD_MSG_FLT_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK) | _CFD4FLTOBJ0_EXIDE_MASK;
    CFD4MASK0 = ((((536870656 & CANFD_MSG_FLT_EXT_SID_MASK) >> 18) | ((536870656 & CANFD_MSG_FLT_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK) | _CFD4MASK0_MIDE_MASK;
    CFD4FLTCON0 |= (((0x2 << _CFD4FLTCON0_F0BP_POSITION) & _CFD4FLTCON0_F0BP_MASK)| _CFD4FLTCON0_FLTEN0_MASK);

    /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
    CFD4CON = (CFD4CON & ~_CFD4CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD4CON_REQOP_POSITION) & _CFD4CON_REQOP_MASK);
    while(((CFD4CON & _CFD4CON_OPMOD_MASK) >> _CFD4CON_OPMOD_POSITION) != CANFD_OPERATION_MODE);
}

// *****************************************************************************
/* Function:
    bool CAN4_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoQueueNum, CANFD_MODE mode, CANFD_MSG_TX_ATTRIBUTE msgAttr)

   Summary:
    Transmits a message into CAN bus.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    id           - 11-bit / 29-bit identifier (ID).
    length       - Length of data buffer in number of bytes.
    data         - Pointer to source data buffer
    fifoQueueNum - If fifoQueueNum is 0 then Transmit Queue otherwise FIFO
    mode         - CAN mode Classic CAN or CAN FD without BRS or CAN FD with BRS
    msgAttr      - Data frame or Remote frame to be transmitted

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_MessageTransmit(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoQueueNum, CANFD_MODE mode, CANFD_MSG_TX_ATTRIBUTE msgAttr)
{
    CANFD_TX_MSG_OBJECT *txMessage = NULL;
    static uint32_t sequence = 0;
    uint8_t count = 0;
    uint8_t dlc = 0;
    bool status = false;

    if (fifoQueueNum == 0)
    {
        if ((CFD4TXQSTA & _CFD4TXQSTA_TXQNIF_MASK) == _CFD4TXQSTA_TXQNIF_MASK)
        {
            txMessage = (CANFD_TX_MSG_OBJECT *)PA_TO_KVA1(CFD4TXQUA);
            status = true;
        }
    }
    else if (fifoQueueNum <= CANFD_NUM_OF_FIFO)
    {
        if ((*(volatile uint32_t *)(&CFD4FIFOSTA1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)) & _CFD4FIFOSTA1_TFNRFNIF_MASK) == _CFD4FIFOSTA1_TFNRFNIF_MASK)
        {
            txMessage = (CANFD_TX_MSG_OBJECT *)PA_TO_KVA1(*(volatile uint32_t *)(&CFD4FIFOUA1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)));
            status = true;
        }
    }

    if (status)
    {
        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CANFD_MSG_SID_MASK)
        {
            txMessage->t0 = (((id & CANFD_MSG_TX_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_TX_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK;
            txMessage->t1 = CANFD_MSG_IDE_MASK;
        }
        else
        {
            txMessage->t0 = id;
            txMessage->t1 = 0;
        }
        if (length > 64)
            length = 64;

        CANLengthToDlcGet(length, &dlc);

        txMessage->t1 |= (dlc & CANFD_MSG_DLC_MASK);

        if(mode == CANFD_MODE_FD_WITH_BRS)
        {
            txMessage->t1 |= CANFD_MSG_FDF_MASK | CANFD_MSG_BRS_MASK;
        }
        else if (mode == CANFD_MODE_FD_WITHOUT_BRS)
        {
            txMessage->t1 |= CANFD_MSG_FDF_MASK;
        }
        if (msgAttr == CANFD_MSG_TX_REMOTE_FRAME)
        {
            txMessage->t1 |= CANFD_MSG_RTR_MASK;
        }
        else
        {
            while(count < length)
            {
                txMessage->data[count++] = *data++;
            }
        }

        txMessage->t1 |= ((++sequence << 9) & CANFD_MSG_SEQ_MASK);

        if (fifoQueueNum == 0)
        {
            /* Request the transmit */
            CFD4TXQCON |= _CFD4TXQCON_UINC_MASK;
            CFD4TXQCON |= _CFD4TXQCON_TXREQ_MASK;
        }
        else
        {
            /* Request the transmit */
            *(volatile uint32_t *)(&CFD4FIFOCON1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)) |= _CFD4FIFOCON1_UINC_MASK;
            *(volatile uint32_t *)(&CFD4FIFOCON1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)) |= _CFD4FIFOCON1_TXREQ_MASK;
        }
    }
    return status;
}

// *****************************************************************************
/* Function:
    bool CAN4_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint32_t *timestamp, uint8_t fifoNum, CANFD_MSG_RX_ATTRIBUTE *msgAttr)

   Summary:
    Receives a message from CAN bus.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - Pointer to 11-bit / 29-bit identifier (ID) to be received.
    length      - Pointer to data length in number of bytes to be received.
    data        - Pointer to destination data buffer
    timestamp   - Pointer to Rx message timestamp, timestamp value is 0 if Timestamp is disabled in CFD4TSCON
    fifoNum     - FIFO number
    msgAttr     - Data frame or Remote frame to be received

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_MessageReceive(uint32_t *id, uint8_t *length, uint8_t *data, uint32_t *timestamp, uint8_t fifoNum, CANFD_MSG_RX_ATTRIBUTE *msgAttr)
{
    CANFD_RX_MSG_OBJECT *rxMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if ((fifoNum > CANFD_NUM_OF_FIFO) || (id == NULL))
    {
        return status;
    }

    /* Check if there is a message available in FIFO */
    if ((*(volatile uint32_t *)(&CFD4FIFOSTA1 + ((fifoNum - 1) * CANFD_FIFO_OFFSET)) & _CFD4FIFOSTA1_TFNRFNIF_MASK) == _CFD4FIFOSTA1_TFNRFNIF_MASK)
    {
        /* Get a pointer to RX message buffer */
        rxMessage = (CANFD_RX_MSG_OBJECT *)PA_TO_KVA1(*(volatile uint32_t *)(&CFD4FIFOUA1 + ((fifoNum - 1) * CANFD_FIFO_OFFSET)));

        /* Check if it's a extended message type */
        if (rxMessage->r1 & CANFD_MSG_IDE_MASK)
        {
            *id = (((rxMessage->r0 & CANFD_MSG_RX_EXT_SID_MASK) << 18) | ((rxMessage->r0 & CANFD_MSG_RX_EXT_EID_MASK) >> 11)) & CANFD_MSG_EID_MASK;
        }
        else
        {
            *id = rxMessage->r0 & CANFD_MSG_SID_MASK;
        }

        if ((rxMessage->r1 & CANFD_MSG_RTR_MASK) && ((rxMessage->r1 & CANFD_MSG_FDF_MASK) == 0))
        {
            *msgAttr = CANFD_MSG_RX_REMOTE_FRAME;
        }
        else
        {
            *msgAttr = CANFD_MSG_RX_DATA_FRAME;
        }

        *length = dlcToLength[(rxMessage->r1 & CANFD_MSG_DLC_MASK)];

        if (timestamp != NULL)
        {
        }

        /* Copy the data into the payload */
        while (count < *length)
        {
            *data++ = rxMessage->data[count++];
        }

        /* Message processing is done, update the message buffer pointer. */
        *(volatile uint32_t *)(&CFD4FIFOCON1 + ((fifoNum - 1) * CANFD_FIFO_OFFSET)) |= _CFD4FIFOCON1_UINC_MASK;

        /* Message is processed successfully, so return true */
        status = true;
    }

    return status;
}

// *****************************************************************************
/* Function:
    void CAN4_MessageAbort(uint8_t fifoQueueNum)

   Summary:
    Abort request for a Queue/FIFO.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoQueueNum - If fifoQueueNum is 0 then Transmit Queue otherwise FIFO

   Returns:
    None.
*/
void CAN4_MessageAbort(uint8_t fifoQueueNum)
{
    if (fifoQueueNum == 0)
    {
        CFD4TXQCON &= ~_CFD4TXQCON_TXREQ_MASK;
    }
    else if (fifoQueueNum <= CANFD_NUM_OF_FIFO)
    {
        *(volatile uint32_t *)(&CFD4FIFOCON1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)) &= ~_CFD4FIFOCON1_TXREQ_MASK;
    }
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

    if (filterNum < CANFD_NUM_OF_FILTER)
    {
        filterRegIndex = filterNum >> 2;
        filterEnableBit = (filterNum % 4 == 0)? _CFD4FLTCON0_FLTEN0_MASK : 1 << ((((filterNum % 4) + 1) * 8) - 1);

        *(volatile uint32_t *)(&CFD4FLTCON0 + (filterRegIndex * CANFD_FILTER_OFFSET)) &= ~filterEnableBit;

        if (id > CANFD_MSG_SID_MASK)
        {
            *(volatile uint32_t *)(&CFD4FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) = ((((id & CANFD_MSG_FLT_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_FLT_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK) | _CFD4FLTOBJ0_EXIDE_MASK;
        }
        else
        {
            *(volatile uint32_t *)(&CFD4FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) = id & CANFD_MSG_SID_MASK;
        }
        *(volatile uint32_t *)(&CFD4FLTCON0 + (filterRegIndex * CANFD_FILTER_OFFSET)) |= filterEnableBit;
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

    if (filterNum < CANFD_NUM_OF_FILTER)
    {
        if (*(volatile uint32_t *)(&CFD4FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & _CFD4FLTOBJ0_EXIDE_MASK)
        {
            id = (((*(volatile uint32_t *)(&CFD4FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & CANFD_MSG_RX_EXT_SID_MASK) << 18)
               | ((*(volatile uint32_t *)(&CFD4FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & CANFD_MSG_RX_EXT_EID_MASK) >> 11))
               & CANFD_MSG_EID_MASK;
        }
        else
        {
            id = (*(volatile uint32_t *)(&CFD4FLTOBJ0 + (filterNum * CANFD_FILTER_OBJ_OFFSET)) & CANFD_MSG_SID_MASK);
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
    acceptanceFilterMaskNum - Acceptance Filter Mask number
    id                      - 11-bit or 29-bit identifier

   Returns:
    None.
*/
void CAN4_MessageAcceptanceFilterMaskSet(uint8_t acceptanceFilterMaskNum, uint32_t id)
{
    /* Switch the CAN module to Configuration mode. Wait until the switch is complete */
    CFD4CON = (CFD4CON & ~_CFD4CON_REQOP_MASK) | ((CANFD_CONFIGURATION_MODE << _CFD4CON_REQOP_POSITION) & _CFD4CON_REQOP_MASK);
    while(((CFD4CON & _CFD4CON_OPMOD_MASK) >> _CFD4CON_OPMOD_POSITION) != CANFD_CONFIGURATION_MODE);

    if (id > CANFD_MSG_SID_MASK)
    {
        *(volatile uint32_t *)(&CFD4MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) = ((((id & CANFD_MSG_FLT_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_FLT_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK) | _CFD4MASK0_MIDE_MASK;
    }
    else
    {
        *(volatile uint32_t *)(&CFD4MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) = id & CANFD_MSG_SID_MASK;
    }

    /* Switch the CAN module to CANFD_OPERATION_MODE. Wait until the switch is complete */
    CFD4CON = (CFD4CON & ~_CFD4CON_REQOP_MASK) | ((CANFD_OPERATION_MODE << _CFD4CON_REQOP_POSITION) & _CFD4CON_REQOP_MASK);
    while(((CFD4CON & _CFD4CON_OPMOD_MASK) >> _CFD4CON_OPMOD_POSITION) != CANFD_OPERATION_MODE);
}

// *****************************************************************************
/* Function:
    uint32_t CAN4_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)

   Summary:
    Get Message acceptance filter mask configuration.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    acceptanceFilterMaskNum - Acceptance Filter Mask number

   Returns:
    Returns Message acceptance filter mask.
*/
uint32_t CAN4_MessageAcceptanceFilterMaskGet(uint8_t acceptanceFilterMaskNum)
{
    uint32_t id = 0;

    if (*(volatile uint32_t *)(&CFD4MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & _CFD4MASK0_MIDE_MASK)
    {
        id = (((*(volatile uint32_t *)(&CFD4MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & CANFD_MSG_RX_EXT_SID_MASK) << 18)
           | ((*(volatile uint32_t *)(&CFD4MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & CANFD_MSG_RX_EXT_EID_MASK) >> 11))
           & CANFD_MSG_EID_MASK;
    }
    else
    {
        id = (*(volatile uint32_t *)(&CFD4MASK0 + (acceptanceFilterMaskNum * CANFD_ACCEPTANCE_MASK_OFFSET)) & CANFD_MSG_SID_MASK);
    }
    return id;
}

// *****************************************************************************
/* Function:
    bool CAN4_TransmitEventFIFOElementGet(uint32_t *id, uint32_t *sequence, uint32_t *timestamp)

   Summary:
    Get the Transmit Event FIFO Element for the transmitted message.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    id          - Pointer to 11-bit / 29-bit identifier (ID) to be received.
    sequence    - Pointer to Tx message sequence number to be received
    timestamp   - Pointer to Tx message timestamp to be received, timestamp value is 0 if Timestamp is disabled in CFD4TSCON

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_TransmitEventFIFOElementGet(uint32_t *id, uint32_t *sequence, uint32_t *timestamp)
{
    CANFD_TX_EVENT_FIFO_ELEMENT *txEventFIFOElement = NULL;
    bool status = false;

    /* Check if there is a message available in Tx Event FIFO */
    if ((CFD4TEFSTA & _CFD4TEFSTA_TEFNEIF_MASK) == _CFD4TEFSTA_TEFNEIF_MASK)
    {
        /* Get a pointer to Tx Event FIFO Element */
        txEventFIFOElement = (CANFD_TX_EVENT_FIFO_ELEMENT *)PA_TO_KVA1(CFD4TEFUA);

        /* Check if it's a extended message type */
        if (txEventFIFOElement->te1 & CANFD_MSG_IDE_MASK)
        {
            *id = txEventFIFOElement->te0 & CANFD_MSG_EID_MASK;
        }
        else
        {
            *id = txEventFIFOElement->te0 & CANFD_MSG_SID_MASK;
        }

        *sequence = ((txEventFIFOElement->te1 & CANFD_MSG_SEQ_MASK) >> 9);

        if (timestamp != NULL)
        {
        }

        /* Tx Event FIFO Element read done, update the Tx Event FIFO tail */
        CFD4TEFCON |= _CFD4TEFCON_UINC_MASK;

        /* Tx Event FIFO Element read successfully, so return true */
        status = true;
    }
    return status;
}

// *****************************************************************************
/* Function:
    CANFD_ERROR CAN4_ErrorGet(void)

   Summary:
    Returns the error during transfer.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    None.

   Returns:
    Error during transfer.
*/
CANFD_ERROR CAN4_ErrorGet(void)
{
    CANFD_ERROR error = CANFD_ERROR_NONE;
    uint32_t errorStatus = CFD4TREC;

    /* Check if error occurred */
    error = (CANFD_ERROR)((errorStatus & _CFD4TREC_EWARN_MASK) |
                        (errorStatus & _CFD4TREC_RXWARN_MASK) |
                        (errorStatus & _CFD4TREC_TXWARN_MASK) |
                        (errorStatus & _CFD4TREC_RXBP_MASK) |
                        (errorStatus & _CFD4TREC_TXBP_MASK) |
                        (errorStatus & _CFD4TREC_TXBO_MASK));

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
    *txErrorCount = (uint8_t)((CFD4TREC & _CFD4TREC_TERRCNT_MASK) >> _CFD4TREC_TERRCNT_POSITION);
    *rxErrorCount = (uint8_t)(CFD4TREC & _CFD4TREC_RERRCNT_MASK);
}

// *****************************************************************************
/* Function:
    bool CAN4_InterruptGet(uint8_t fifoQueueNum, CANFD_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)

   Summary:
    Returns the FIFO Interrupt status.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoQueueNum          - FIFO number
    fifoInterruptFlagMask - FIFO interrupt flag mask

   Returns:
    true - Requested fifo interrupt is occurred.
    false - Requested fifo interrupt is not occurred.
*/
bool CAN4_InterruptGet(uint8_t fifoQueueNum, CANFD_FIFO_INTERRUPT_FLAG_MASK fifoInterruptFlagMask)
{
    if (fifoQueueNum == 0)
    {
        return ((CFD4TXQSTA & fifoInterruptFlagMask) != 0x0);
    }
    else
    {
        return ((*(volatile uint32_t *)(&CFD4FIFOSTA1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)) & fifoInterruptFlagMask) != 0x0);
    }
}

// *****************************************************************************
/* Function:
    bool CAN4_TxFIFOQueueIsFull(uint8_t fifoQueueNum)

   Summary:
    Returns true if Tx FIFO/Queue is full otherwise false.

   Precondition:
    CAN4_Initialize must have been called for the associated CAN instance.

   Parameters:
    fifoQueueNum - FIFO/Queue number

   Returns:
    true  - Tx FIFO/Queue is full.
    false - Tx FIFO/Queue is not full.
*/
bool CAN4_TxFIFOQueueIsFull(uint8_t fifoQueueNum)
{
    if (fifoQueueNum == 0)
    {
        return ((CFD4TXQSTA & _CFD4TXQSTA_TXQNIF_MASK) != _CFD4TXQSTA_TXQNIF_MASK);
    }
    else
    {
        return ((*(volatile uint32_t *)(&CFD4FIFOSTA1 + ((fifoQueueNum - 1) * CANFD_FIFO_OFFSET)) & _CFD4FIFOSTA1_TFNRFNIF_MASK) != _CFD4FIFOSTA1_TFNRFNIF_MASK);
    }
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
    id           - 11-bit / 29-bit identifier (ID).
    length       - Length of data buffer in number of bytes.
    data         - Pointer to source data buffer
    fifoNum      - FIFO Number

   Returns:
    Request status.
    true  - Request was successful.
    false - Request has failed.
*/
bool CAN4_AutoRTRResponseSet(uint32_t id, uint8_t length, uint8_t* data, uint8_t fifoNum)
{
    CANFD_TX_MSG_OBJECT *txMessage = NULL;
    uint8_t count = 0;
    bool status = false;

    if (fifoNum <= CANFD_NUM_OF_FIFO)
    {
        if ((*(volatile uint32_t *)(&CFD4FIFOSTA1 + ((fifoNum - 1) * CANFD_FIFO_OFFSET)) & _CFD4FIFOSTA1_TFNRFNIF_MASK) == _CFD4FIFOSTA1_TFNRFNIF_MASK)
        {
            txMessage = (CANFD_TX_MSG_OBJECT *)PA_TO_KVA1(*(volatile uint32_t *)(&CFD4FIFOUA1 + ((fifoNum - 1) * CANFD_FIFO_OFFSET)));
            status = true;
        }
    }

    if (status)
    {
        /* Check the id whether it falls under SID or EID,
         * SID max limit is 0x7FF, so anything beyond that is EID */
        if (id > CANFD_MSG_SID_MASK)
        {
            txMessage->t0 = (((id & CANFD_MSG_TX_EXT_SID_MASK) >> 18) | ((id & CANFD_MSG_TX_EXT_EID_MASK) << 11)) & CANFD_MSG_EID_MASK;
            txMessage->t1 = CANFD_MSG_IDE_MASK;
        }
        else
        {
            txMessage->t0 = id;
            txMessage->t1 = 0;
        }

        /* Limit length */
        if (length > 8)
            length = 8;
        txMessage->t1 |= length;

        while(count < length)
        {
            txMessage->data[count++] = *data++;
        }

        /* Set UINC to respond to RTR */
        *(volatile uint32_t *)(&CFD4FIFOCON1 + ((fifoNum - 1) * CANFD_FIFO_OFFSET)) |= _CFD4FIFOCON1_UINC_MASK;
    }
    return status;
}

