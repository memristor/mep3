#include "protocol.h"
#include "control.h"

bool report_encoders = false;
bool setpoints_enabled = true;

/* Helper functions to pack/unpack vars to/from byte arrays */
void protocol_pack_int16(uint8_t *buffer, int16_t val)
{
    buffer[1] = val & 0xFF;
    buffer[0] = (val >> 8) & 0xFF;
}

void protocol_pack_int32(uint8_t *buffer, int32_t val)
{
    for (int i = 3; i >= 0; i--)
    {
        buffer[i] = (val >> 8 * (3 - i)) & 0xFF;
    }
}

void protocol_pack_uint32(uint8_t *buffer, uint32_t val)
{
    for (int i = 3; i >= 0; i--)
    {
        buffer[i] = (val >> 8 * (3 - i)) & 0xFF;
    }
}

void protocol_pack_float(uint8_t *buffer, float val)
{
    uint32_t as_integer = *((uint32_t *) & val); // access float on byte level
    protocol_pack_uint32(buffer, as_integer);
}

float protocol_unpack_float(uint8_t *buffer)
{
    uint32_t val = 0;
    for (int i = 0; i < 4; i++)
    {
        val |= ((uint32_t) buffer[i] << 8 * (3 - i));
    }

    float as_float = *((float *) &val);

    return as_float;
}

uint16_t protocol_unpack_uint16(uint8_t *buffer)
{
    uint16_t val = 0;
    val = (uint16_t) buffer[0] << 8;
    val |= (uint16_t) buffer[1];

    return val;
}

int16_t protocol_unpack_int16(uint8_t *buffer)
{
    uint16_t val = 0;
    val = (uint16_t) buffer[0] << 8;
    val |= (uint16_t) buffer[1];

    return (int16_t) val;
}

/****************************************************/

void can_send_quick(uint32_t id, uint8_t length, uint8_t *data)
{
    while (CAN4_TxFIFOIsFull(0));
    // OR'ed with 0x80000000 so id becomes extended
    CAN4_MessageTransmit((id | 0x80000000), length, data, 0, CAN_MSG_TX_DATA_FRAME);
}

void protocol_process_msg(uint32_t id, uint8_t length, uint8_t *data)
{
    if (id != CAN_BASE_ID || length == 0)
        return;

    const uint8_t command = data[0];

    switch (command)
    {
    case CMD_READ_ENCODERS:
    {
        const int32_t encoder_left = -(int32_t)QEI3_PositionGet(); 
        const int32_t encoder_right = (int32_t)QEI1_PositionGet();
        uint8_t buff[8];
        protocol_pack_int32(buff, encoder_left);
        protocol_pack_int32(buff + 4, encoder_right);
        can_send_quick(CAN_ENCODER_ID, 8, buff);
        break;
    }
    case CMD_SET_SETPOINTS:
    {   
        if (!setpoints_enabled)
            break;
        if (length < 5)
            break;
        int16_t setpoint_linear = protocol_unpack_int16(&data[1]);
        int16_t setpoint_angular = protocol_unpack_int16(&data[3]);

        control_set_setpoint_linear(setpoint_linear);
        control_set_setpoint_angular(setpoint_angular);
        break;
    }
    case CMD_SET_KP_LINEAR:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kp_linear(val);
        break;
    }
    case CMD_SET_KI_LINEAR:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_ki_linear(val);
        break;
    }
    case CMD_SET_KD_LINEAR:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kd_linear(val);
        break;
    }
    case CMD_SET_KP_ANGULAR:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kp_angular(val);
        break;
    }
    case CMD_SET_KI_ANGULAR:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_ki_angular(val);
        break;
    }
    case CMD_SET_KD_ANGULAR:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kd_angular(val);
        break;
    }
    case CMD_GET_SETPOINTS:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_SETPOINTS;
        int16_t setpoint_linear = control_get_setpoint_linear();
        int16_t setpoint_angular = control_get_setpoint_angular();
        
        protocol_pack_int16(buff + 1, setpoint_linear);
        protocol_pack_int16(buff + 1 + 2, setpoint_angular);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KP_LINEAR:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KP_LINEAR;
        float val = control_get_kp_linear();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KI_LINEAR:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KI_LINEAR;
        float val = control_get_ki_linear();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KD_LINEAR:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KD_LINEAR;
        float val = control_get_kd_linear();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KP_ANGULAR:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KP_ANGULAR;
        float val = control_get_kp_angular();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KI_ANGULAR:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KI_ANGULAR;
        float val = control_get_ki_angular();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KD_ANGULAR:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KD_ANGULAR;
        float val = control_get_kd_angular();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_MOTOR_OFF:
    {
        motor_turn_off();
        break;
    }
    case CMD_MOTOR_ON:
    {
        motor_init();
        break;
    }
    case CMD_RESET_REGULATORS:
    {
        control_reset();
        break;
    }
    case CMD_RESET_ENCODERS:
    {
        QEI3_PositionCountSet(0);
        QEI1_PositionCountSet(0);
        break;
    }
    case CMD_ENABLE_ENCODER_REPORT:
    {
        report_encoders = true;
        break;
    }
    case CMD_DISABLE_ENCODER_REPORT:
    {
        report_encoders = false;
        break;
    }
    case CMD_SETPOINTS_ENABLE:
    {
        setpoints_enabled = true;
        break;
    }
    case CMD_SETPOINTS_DISABLE:
    {
        setpoints_enabled = false;
        break;
    }
    default:
        break;
    }
    
}