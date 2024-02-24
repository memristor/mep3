
#ifndef _SENSOR_H  
#define _SENSOR_H

#include "config/default/peripheral/gpio/plib_gpio.h"
#include "can.h"
#include "config/default/peripheral/canfd/plib_canfd4.h"

void sens_init(void);
void sens_handler(GPIO_PIN pin, uintptr_t context);
void cinch_callback(uintptr_t context);

#endif 

/* *****************************************************************************
 End of File
 */
