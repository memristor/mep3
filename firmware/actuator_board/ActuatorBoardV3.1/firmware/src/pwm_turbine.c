#include "pwm_turbine.h"

void  set_turbine_pwm(uint8_t *data)
{
    //dir = 0 - stop; dir = 1 - pos; dir = -1 - neg 
    static int8_t dir = 0;
    
    int8_t tmp = (int8_t)data[0];
    
    //TODO:
    // if change direction but turbine is on in oposite direction
    // wait CHANGE_DIR_DELAY seconds
    if((tmp < 0 && dir == 1)||(tmp>0 && dir == -1))
    {
        MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 900);
        CORETIMER_DelayMs(CHANGE_DIR_DELAY);
    }
    
    switch(tmp)
    {
        case -6:
            TURBINE_Clear();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 2200);
            dir = -1;
            break;
        case -5:
            TURBINE_Clear();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 2000);
            dir = -1;
            break;
        case -4:
            TURBINE_Clear();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1800);
            dir = -1;
            break;
        case -3:
            TURBINE_Clear();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1600);
            dir = -1;
            break;
        case -2:
            TURBINE_Clear();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1400);
            dir = -1;
            break;
        case -1:
            TURBINE_Clear();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1200);
            dir = -1;
            break;
        case 0:      
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 900);
            dir = 0;
            break;
        case 1:
            TURBINE_Set();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1200);
            dir = 1;
            break;
        case 2:
            TURBINE_Set();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1400);
            dir = 1;
            break;
        case 3:
            TURBINE_Set();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1600);
            dir = 1;
            break;
        case 4:
            TURBINE_Set();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 1800);
            dir = 1;
            break;
        case 5:
            TURBINE_Set();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 2000);
            dir = 1;
            break;
        case 6:
            TURBINE_Set();
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 2200);
            dir = 1;
            break;
        default:
            MCPWM_ChannelPrimaryDutySet(MCPWM_CH_2, 800);
            dir = 0;
    }
   
        
}
