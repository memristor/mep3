#include "definitions.h"

//#define ADC_CAN_ID 0x00008D70

/*int ADConversion(int channel) 
{
    ADCHS_ChannelConversionStart(channel);
    while(ADCHS_ChannelResultIsReady(channel) == false);
    
    return ADCHS_ChannelResultGet(channel);
}*/

void ADConversion(int channel)
{
    uint16_t Message ;
    uint8_t tmp[2] = {0,0};
  
    ADCHS_ChannelConversionStart(channel);
    while(ADCHS_ChannelResultIsReady(channel) == false);
    
    Message = ADCHS_ChannelResultGet(channel);
    
    memcpy(tmp,&Message,sizeof(Message));
    while(CAN4_MessageTransmit(ADC_CAN_ID+channel, 2, tmp, 0, 0) == false);
}


/* ************************************************************************** */
/* ************************************************************************** */
// Section: Interface Functions                                               */
/* ************************************************************************** */
/* ************************************************************************** */

/*  A brief description of a section can be given directly below the section
    banner.
 */

// *****************************************************************************

/** 
  @Function
    int ExampleInterfaceFunctionName ( int param1, int param2 ) 

  @Summary
    Brief one-line description of the function.

  @Remarks
    Refer to the example_file.h interface header for function usage details.
 */



/* *****************************************************************************
 End of File
 */
