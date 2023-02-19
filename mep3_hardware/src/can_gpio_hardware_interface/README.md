# CAN GPIO Hardware Interface

Controls and reads GPIO pins over CAN.

## Protocol Definition

We can work with digital and analog pins.

### Digital Pins

Every two bits in a message represent a pin state:
- `00`: reset
- `01`: set
- `10`: ignore
- `11`: ignore

For example, in case you want to set Pin #0 and reset Pin #7:
```
01.11.11.11#11.11.11.00#11.11.11.11.11...
```
