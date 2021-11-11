# pic-motor-controller
New PIC32 Motor driver board code, starting from 2022.\
Performs velocity control of two wheels with new PID controller implementation.

## Requirements

- MPLAB X IDE ([download](https://www.microchip.com/en-us/development-tools-tools-and-software/mplab-x-ide))
- MPLAB XC32/32++ Compiler ([download](https://www.microchip.com/en-us/development-tools-tools-and-software/mplab-xc-compilers#tabs))
- MPLAB Harmony v3 Software Development Framework (install through MPLAB X IDE)

**Important:** `pic-motor-controller.X` and `src` folders must be inside folder named `firmware` in order for MPLAB Harmony Configurator v3 to work properly.

## Communication protocol
Communication is done over CAN bus at 500 kbps.\
Board receives message with id CAN_BASE_ID, which is defined in [`protocol.h`](frimware/src/protocol.h).\
First byte in message is the command byte. Command byte values are also defined in [`protocol.h`](firmware/src/protocol.h).\
After the command byte is the remaining data, if required.\
For example, to set KP gain of left wheel regulator, user would send CMD_SET_KP_LEFT command byte and put 4 bytes of data which represents a float value. Values must be sent in **big endian** format, sending most significant byte first after the command byte.  

If command requires a response for the board, for example CMD_GET_KP_LEFT, board will respond with ID equal to CAN_BASE_ID+1, command byte and the required data.

Special case is CMD_READ_ENCODERS. Board will respond with ID equal to CAN_BASE_ID+2 and 2x4 bytes of data which represent 2 int32_t encoder values. Left encoder value is sent first.

Setting the reference wheel velocities is done through command CMD_SET_SETPOINTS by sending 2 int16_t values for left and the right wheel.

| Description | Request CAN ID | Request payload | Response CAN ID | Response payload |
|---|---|---|---|---|
| Read encoder values (number of ticks) | `CAN_BASE_ID` | `CMD_READ_ENCODERS` | `CAN_ENCODER_ID` | encoder left (`int32`),<br> encoder right (`int32`) |
| Reset encoder position counters | `CAN_BASE_ID` | `CMD_RESET_ENCODERS` | | |
| Set wheel velocity setpoints | `CAN_BASE_ID` | `CMD_SET_SETPOINTS`,<br> velocity left (`int16`),<br> velocity right (`int16`) | | | 
| Get wheel velocity setpoints | `CAN_BASE_ID` | `CMD_GET_SETPOINTS` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_SETPOINTS`,<br> velocity left (`int16`),<br> velocity right (`int16`) | 
| Set KP gain left | `CAN_BASE_ID` | `CMD_SET_KP_LEFT`,<br> KP gain (`float`) | | |
| Set KI gain left | `CAN_BASE_ID` | `CMD_SET_KI_LEFT`,<br> KI gain (`float`) | | |
| Set KD gain left | `CAN_BASE_ID` | `CMD_SET_KD_LEFT`,<br> KD gain (`float`) | | |
| Set KP gain right | `CAN_BASE_ID` | `CMD_SET_KP_RIGHT`,<br> KP gain (`float`) | | |
| Set KI gain right | `CAN_BASE_ID` | `CMD_SET_KI_RIGHT`,<br> KI gain (`float`) | | |
| Set KD gain right | `CAN_BASE_ID` | `CMD_SET_KD_RIGHT`,<br> KD gain (`float`) | | |
| Get KP gain left | `CAN_BASE_ID` | `CMD_GET_KP_LEFT` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_KP_LEFT`,<br> KP gain left (`float`) |
| Get KI gain left | `CAN_BASE_ID` | `CMD_GET_KI_LEFT` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_KI_LEFT`,<br> KI gain left (`float`) |
| Get KD gain left | `CAN_BASE_ID` | `CMD_GET_KD_LEFT` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_KD_LEFT`,<br> KD gain left (`float`) |
| Get KP gain right | `CAN_BASE_ID` | `CMD_GET_KP_RIGHT` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_KP_RIGHT`,<br> KP gain right (`float`) |
| Get KI gain right | `CAN_BASE_ID` | `CMD_GET_KI_RIGHT` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_KI_RIGHT`,<br> KI gain right (`float`) |
| Get KD gain right | `CAN_BASE_ID` | `CMD_GET_KD_RIGHT` | `CAN_GENERAL_RESPONSE_ID` | `CMD_GET_KD_RIGHT`,<br> KD gain right (`float`) |
| Turn off motors | `CAN_BASE_ID` | `CMD_MOTOR_OFF` | | |
| Turn on motors | `CAN_BASE_ID` | `CMD_MOTOR_ON` | | |
| Reset left and right pid regulators | `CAN_BASE_ID` | `CMD_RESET_REGULATORS` | | |




## TODO
Configure CAN filter after we decide which CAN_BASE_ID to use.\
Test if 500 Hz control loop works correctly on robot.
