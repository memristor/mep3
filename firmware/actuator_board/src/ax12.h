/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#include "can.h"

typedef enum _servoSelect{
    AX12 = 0,
    RX24 = 1
} ServoSelect;


void AX12_Initialize();

int AX12_SpeedInit(uint16_t speed);

int AX12_OnMessage(can_t AX12_CanMsg);