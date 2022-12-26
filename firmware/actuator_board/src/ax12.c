/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#include "definitions.h"
#include "peripheral/uart/plib_uart3.h"
#include "device.h"

#define AX12_TIMEOUT 15//ms
//#define AX12_MESSAGE_DEBUG
//#define AX12_INITSPEED_DEBUG

void AX12_Initialize()
{
    UART3_Initialize();
    
    SERVO_SELECT =  AX12;
}


int AX12_SpeedInit(uint16_t speed)
{
    uint8_t txpacket[9];
    uint8_t checksum = 0;
    
    if(speed < 0 || speed > 1023)
        return -1;
    
    // SET SPEED TO speed
    txpacket[0] = 0xff;
    txpacket[1] = 0xff;
    txpacket[2] = 0xfe;     // SEND PACKET TO ALL SERVOS
    txpacket[3] = 0x05;
    txpacket[4] = 0x03;
    txpacket[5] = 0x20;
    txpacket[6] = speed%256;
    txpacket[7] = speed/256;
    
    for(int i = 2; i < 8; i++)
        checksum += txpacket[i];
    
    txpacket[8] = ~ (checksum % 256);
    
    UART3_Write(txpacket, 9);

    
#ifdef AX12_INITSPEED_DEBUG
    UART6_Write(txpacket, 9);
    while(UART6_WriteIsBusy());
#endif
    
    return 0;
}


int AX12_OnMessage(can_t AX12_CanMsg)
{
    uint8_t txpacket[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t checksum = 0;
    
    uint8_t rxpacket[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rxlength = 0;
    
    size_t rxmsg_length = 0;
    
    if(AX12_CanMsg.ID != AX12_CANID)
        return -1;

    txpacket[0] = 0xff;
    txpacket[1] = 0xff;
    
    for(int i = 0; i < AX12_CanMsg.length; i++)
    {
        txpacket[i+2] = AX12_CanMsg.data[i];
        checksum = checksum + AX12_CanMsg.data[i];
    }

    txpacket[AX12_CanMsg.length+2] = ~ ( checksum % 256) ;

    // Send packet to AX //
    UART3_Read(rxpacket, 256);  // RX Buffer clear
    UART3_Write(txpacket, AX12_CanMsg.length+3);

    // Delay - Wait for response //
    CORETIMER_DelayMs(AX12_TIMEOUT);
    
    // Ignore garbage - Transmitted message //
    UART3_Read(rxpacket, AX12_CanMsg.length+3);

    rxmsg_length = UART3_ReadCountGet();
    if(rxmsg_length > (size_t)0)
    {
        // Read message //
        uint8_t i = 0;
        while(rxmsg_length > (size_t)0)
        {
            UART3_Read((uint8_t*)(rxpacket+i), 1);
            i++;
            rxmsg_length = UART3_ReadCountGet();
        }

        // Calculate rxpacket length = Length + (1 Byte ID) + (1 Byte Length) - (1 Byte Checksum) //
        rxlength = rxpacket[3] + 2 - 1;

#ifdef AX12_MESSAGE_DEBUG
        UART6_Write(rxpacket, rxlength+2);
        while(UART6_WriteIsBusy());
#endif

        while(CAN4_MessageTransmit(AX12_CANID, rxlength, (uint8_t*)(rxpacket+2), 0, CAN_MSG_TX_DATA_FRAME) == false);
    }

    return 0;
}