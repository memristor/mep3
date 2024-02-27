#include "sensor.h"
#include "peripheral/tmr/plib_tmr2.h"

void sens_init(void){
    GPIO_PinIntEnable(SENS1_PIN, GPIO_INTERRUPT_ON_BOTH_EDGES);  //Cinch
    GPIO_PinInterruptCallbackRegister(SENS1_PIN, sens_handler, 0);
//    SENS2_InterruptEnable();
//    GPIO_PinInterruptCallbackRegister(SENS2_PIN, sens_handler, 0);
//    SENS3_InterruptEnable();
//    GPIO_PinInterruptCallbackRegister(SENS3_PIN, sens_handler, 0);
//    SENS4_InterruptEnable();
//    GPIO_PinInterruptCallbackRegister(SENS4_PIN, sens_handler, 0);
//    SENS5_InterruptEnable();
//    GPIO_PinInterruptCallbackRegister(SENS5_PIN, sens_handler, 0);
//    SENS6_InterruptEnable();
//    GPIO_PinInterruptCallbackRegister(SENS6_PIN, sens_handler, 0);
    //SENS7_InterruptEnable();
    GPIO_PinIntEnable(SENS7_PIN, GPIO_INTERRUPT_ON_BOTH_EDGES);     // BS7
    GPIO_PinInterruptCallbackRegister(SENS7_PIN, sens_handler, 0);
    GPIO_PinIntEnable(SENS8_PIN, GPIO_INTERRUPT_ON_BOTH_EDGES);     // BS8
    GPIO_PinInterruptCallbackRegister(SENS8_PIN, sens_handler, 0);
    GPIO_PinIntEnable(SENS9_PIN, GPIO_INTERRUPT_ON_BOTH_EDGES);     // BS9
    GPIO_PinInterruptCallbackRegister(SENS9_PIN, sens_handler, 0);
    GPIO_PinIntEnable(SENS10_PIN, GPIO_INTERRUPT_ON_BOTH_EDGES);    // BS10
    GPIO_PinInterruptCallbackRegister(SENS10_PIN, sens_handler, 0);
}
   

void sens_handler(GPIO_PIN pin, uintptr_t context)
{
    uint8_t data[1];
    switch(pin)
    {
        
        case SENS1_PIN:
        // Cinch pin has pull-down resistor
            if(!GPIO_PinRead(SENS1_PIN))
            {
                TMR2_InterruptEnable();
                TMR2_Start();
            }
            else 
            {
                // Cinch detection
                data[0]=0;
                CAN4_MessageTransmit((0x6D00 | 0x80000000),1,data, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
            }
            break;
        case SENS7_PIN:
            if(GPIO_PinRead(SENS7_PIN))
            {
                data[0]=1;
            }
            else
            {
                data[0]=0;
            }
            CAN4_MessageTransmit((0x6D13 | 0x80000000),1,data, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
            break;
        case SENS8_PIN:
            if(GPIO_PinRead(SENS8_PIN))
            {
                data[0]=1;
            }
            else
            {
                data[0]=0;
            }
            CAN4_MessageTransmit((0x6D12 | 0x80000000),1,data, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
            break;
        case SENS9_PIN:
            if(GPIO_PinRead(SENS9_PIN))
            {
                data[0]=1;
            }
            else
            {
                data[0]=0;
            }
            CAN4_MessageTransmit((0x6D11 | 0x80000000),1,data, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
            break;
        case SENS10_PIN:
            if(GPIO_PinRead(SENS10_PIN))
            {
                data[0]=1;
            }
            else
            {
                data[0]=0;
            }
            CAN4_MessageTransmit((0x6D10 | 0x80000000),1,data, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
            break;            
    }
}


void cinch_callback(uintptr_t context)
{
    static uint8_t num=0;
    
    
    
    if(num >= 10)
    {
        TMR2_InterruptDisable();
        TMR2_Stop();
        num = 0;
    }
    else
    {
        uint8_t data[1] = {1};
        CAN4_MessageTransmit((0x6D00 | 0x80000000),1,data, 0, CANFD_MODE_NORMAL, CANFD_MSG_TX_DATA_FRAME);
        num++;
    }
}