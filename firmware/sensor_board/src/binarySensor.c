#include "definitions.h"

#define RA11 PORTAbits.RA11  
#define RA0 PORTAbits.RA0 
#define RA1 PORTAbits.RA1  
#define RB0 PORTBbits.RB0  
#define BIN_SENSOR_NUM 4
//#define BINARY_SENSOR_CAN_ID 0x00008D10

volatile bool flag;


int newValue[4];

void binary_sensor_get_single(int sensor_num){
    uint8_t bin_Message;
    
    newValue[0] = RA11;
    newValue[1] = RA0;
    newValue[2] = RA1;
    newValue[3] = RB0;
    
    bin_Message = newValue[sensor_num];
    
    while(CAN4_MessageTransmit(BINARY_SENSOR_CAN_ID + sensor_num, 1, &bin_Message, 0, 0) == false);
}
void binary_sensor_update_all()
{ 
    uint8_t i;
    
    for(i=0; i<BIN_SENSOR_NUM; i++){
        binary_sensor_update_single(i);
    }
    
        /*if(sensor > -1)
            {
            uint8_t Message;
            Message = 1;
    
            while(CAN4_MessageTransmit(BINARY_SENSOR_CAN_ID + sensor, 1, &Message, 0, 0) == false);

            }   */
    
}

void binary_sensor_update_single(int sensor_num){
    uint8_t bin_Message;
    static int8_t lastValue[4] = {0, 0, 0, 0};
    newValue[0] = RA11;
    newValue[1] = RA0;
    newValue[2] = RA1;
    newValue[3] = RB0;
    
    if(newValue[sensor_num] != lastValue[sensor_num])
            {
            lastValue[sensor_num] = newValue[sensor_num];
            
            bin_Message = newValue[sensor_num];
    
            while(CAN4_MessageTransmit(BINARY_SENSOR_CAN_ID + sensor_num, 1, &bin_Message, 0, 0) == false);

    }

}