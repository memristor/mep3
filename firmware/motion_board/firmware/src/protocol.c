#include "protocol.h"
#include "control.h"

bool report_encoders = false;

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
        if (length < 5)
            break;
        int16_t setpoint_left = protocol_unpack_int16(&data[1]);
        int16_t setpoint_right = protocol_unpack_int16(&data[3]);

        control_set_setpoint_left((float)setpoint_left);
        control_set_setpoint_right((float)setpoint_right);
        break;
    }
    case CMD_SET_KP_LEFT:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kp_left(val);
        break;
    }
    case CMD_SET_KI_LEFT:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_ki_left(val);
        break;
    }
    case CMD_SET_KD_LEFT:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kd_left(val);
        break;
    }
    case CMD_SET_KP_RIGHT:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kp_right(val);
        break;
    }
    case CMD_SET_KI_RIGHT:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_ki_right(val);
        break;
    }
    case CMD_SET_KD_RIGHT:
    {
        if (length < 5)
            break;
        float val = protocol_unpack_float(&data[1]);
        control_set_kd_right(val);
        break;
    }
    case CMD_GET_SETPOINTS:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_SETPOINTS;
        int16_t setpoint_left = control_get_setpoint_left();
        int16_t setpoint_right = control_get_setpoint_right();
        
        protocol_pack_int16(buff + 1, setpoint_left);
        protocol_pack_int16(buff + 1 + 2, setpoint_right);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KP_LEFT:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KP_LEFT;
        float val = control_get_kp_left();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KI_LEFT:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KI_LEFT;
        float val = control_get_ki_left();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KD_LEFT:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KD_LEFT;
        float val = control_get_kd_left();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KP_RIGHT:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KP_RIGHT;
        float val = control_get_kp_right();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KI_RIGHT:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KI_RIGHT;
        float val = control_get_ki_right();
        
        protocol_pack_float(buff + 1, val);
        can_send_quick(CAN_GENERAL_RESPONSE_ID, 5, buff);
        break;
    }
    case CMD_GET_KD_RIGHT:
    {
        uint8_t buff[5];
        buff[0] = CMD_GET_KD_RIGHT;
        float val = control_get_kd_right();
        
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
        report_encoders = false;
    default:
        break;
    }
    
}