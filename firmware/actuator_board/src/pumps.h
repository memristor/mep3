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

#include "ax12.h"

#define VACUUM_PUMP_CONFIG_COUNT_MAX 6
    

typedef struct _VacuumPump{
    uint8_t pumpPin;
    uint8_t switchPin;
    uint16_t CanID;
} VacuumPump;

typedef enum _PinValue {
	PIN_LOW = 0,
	PIN_HIGH = 1
} PinValue;
  
    
void VacuumPump_Add(uint8_t pumpPin, uint8_t switchPin, uint16_t number);

int VacuumPump_OnMessage(can_t Pump_CanMsg);

void SinglePump_State(uint8_t pumpPin, PinValue value);

void SingleSwitch_State(uint8_t switchPin, PinValue value);