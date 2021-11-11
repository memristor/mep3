# pic-motor-controller
New PIC32 Motor driver board code, starting from 2022.\
Performs velocity control of two wheels with new PID controller implementation.

## Requirements

- MPLAB X IDE ([download](https://www.microchip.com/en-us/development-tools-tools-and-software/mplab-x-ide))
- MPLAB XC32/32++ Compiler ([download](https://www.microchip.com/en-us/development-tools-tools-and-software/mplab-xc-compilers#tabs))

**Important:** `pic-motor-controller.X` and `src` folders must be inside folder named `firmware` in order for MPLAB Harmony Configurator v3 to work properly.

## Communication protocol
Communication is done over CAN bus at 500 kbps.\
Board receives message with id CAN_BASE_ID, which is defined in [`protocol.h`](src/protocol.h).\
First byte in message is the command byte. Command byte values are also defined in [`protocol.h`](src/protocol.h).\
After the command byte is the remaining data, if required.\
For example, to set KP gain of left wheel regulator, user would send CMD_SET_KP_LEFT command byte and put 4 bytes of data which represents a float value. Values must be sent in **big endian** format, sending most significant byte first after the command byte.  

If command requires a response for the board, for example CMD_GET_KP_LEFT, board will respond with ID equal to CAN_BASE_ID+1, command byte and the required data.

Special case is CMD_READ_ENCODERS. Board will respond with ID equal to CAN_BASE_ID+2 and 2x4 bytes of data which represent 2 int32_t encoder values. Left encoder value is sent first.

Setting the reference wheel velocities is done through command CMD_SET_SETPOINTS by sending 2 int16_t values for left and the right wheel.

| Description | Request | Response |
|---|---|---|
| Read encoder values (number of ticks) | `CMD_READ_ENCODERS` | `CAN_ENCODER_ID`, encoder left (`int32`), encoder right (`int32`) |

## TODO
Configure CAN filter after we decide which CAN_BASE_ID to use.\
Test if 500 Hz control loop works correctly on robot.
