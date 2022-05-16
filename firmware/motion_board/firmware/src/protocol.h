#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "definitions.h"
#include <stdint.h>

#define CAN_BASE_ID             0x00002000UL
#define CAN_GENERAL_RESPONSE_ID (CAN_BASE_ID+2)
#define CAN_ENCODER_ID          (CAN_BASE_ID+4)

/*COMMANDS*/
#define CMD_READ_ENCODERS       0x00

/*2x int16_t = 4 bytes data*/
#define CMD_SET_SETPOINTS       0x01
#define CMD_GET_SETPOINTS       0x02
/***********************************/

/*float = 4 bytes data*/
#define CMD_SET_KP_LINEAR         0x03
#define CMD_GET_KP_LINEAR         0x04
#define CMD_SET_KI_LINEAR         0x05
#define CMD_GET_KI_LINEAR         0x06
#define CMD_SET_KD_LINEAR         0x07
#define CMD_GET_KD_LINEAR         0x08

#define CMD_SET_KP_ANGULAR        0x09
#define CMD_GET_KP_ANGULAR        0x0A
#define CMD_SET_KI_ANGULAR        0x0B
#define CMD_GET_KI_ANGULAR        0x0C
#define CMD_SET_KD_ANGULAR        0x0D
#define CMD_GET_KD_ANGULAR        0x0E
/***********************************/

#define CMD_MOTOR_OFF               0x0F
#define CMD_MOTOR_ON                0x10
#define CMD_RESET_REGULATORS        0x11
#define CMD_RESET_ENCODERS          0x12
#define CMD_ENABLE_ENCODER_REPORT   0x13
#define CMD_DISABLE_ENCODER_REPORT  0x14


extern bool report_encoders;

void protocol_process_msg(uint32_t id, uint8_t length, uint8_t *data);
void can_send_quick(uint32_t id, uint8_t length, uint8_t *data);

void protocol_pack_int16(uint8_t *buffer, int16_t val);
void protocol_pack_int32(uint8_t *buffer, int32_t val);
void protocol_pack_uint32(uint8_t *buffer, uint32_t val);
void protocol_pack_float(uint8_t *buffer, float val);
float protocol_unpack_float(uint8_t *buffer);
uint16_t protocol_unpack_uint16(uint8_t *buffer);
int16_t protocol_unpack_int16(uint8_t *buffer);
#endif