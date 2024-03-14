/*
 * File: HD44780_I2C_lcd.c
 * Description: 
 * HD44780-based character LCD 16x02 I2C(PCF8574) library source file.
 * Description: See header full details.
 */

// Section : Includes
#include "I2C_lcd.h"


// Section : Variables
uint8_t _LCDBackLight = LCD_BACKLIGHTON_MASK;
//I2C  address for I2C module PCF8574 backpack on LCD
uint8_t _LCDSlaveAddresI2C = 0x27;
uint8_t _NumRowsLCD = 2;
uint8_t _NumColsLCD = 16;

// Section : Functions

//  Func Desc: Send data byte to  lcd
//  Param1: data byte

void PCF8574_LCDSendData(unsigned char data) {
    unsigned char data_l, data_u;
    uint8_t data_I2C[4];

    data_l = (data << 4)&0xf0; //select lower nibble by moving it to the upper nibble position
    data_u = data & 0xf0; //select upper nibble  
    data_I2C[0] = data_u | (LCD_DATA_BYTE_ON & _LCDBackLight); //   DATA-led-en-rw-rs ,enable=1 and rs =1
    data_I2C[1] = data_u | (LCD_DATA_BYTE_OFF & _LCDBackLight); //  DATA-led-en-rw-rs ,enable=0 and rs =1
    data_I2C[2] = data_l | (LCD_DATA_BYTE_ON & _LCDBackLight); //   DATA-led-en-rw-rs ,enable=1 and rs =1
    data_I2C[3] = data_l | (LCD_DATA_BYTE_OFF & _LCDBackLight); // DATA-led-en-rw-rs ,enable=0 and rs =1

    I2C4_Write(_LCDSlaveAddresI2C, data_I2C, 4);
    while (I2C4_IsBusy() == true);
}

//  Func Desc: Send command byte to  lcd
//  Param1: command byte

void PCF8574_LCDSendCmd(unsigned char cmd) {
    unsigned char cmd_l, cmd_u;
    uint8_t cmd_I2C[4];

    cmd_l = (cmd << 4)&0xf0; //select lower nibble by moving it to the upper nibble position
    cmd_u = cmd & 0xf0; //select upper nibble
    cmd_I2C[0] = cmd_u | (LCD_CMD_BYTE_ON & _LCDBackLight); //  COMD-led-en-rw-rs ,enable=1 and rs =0
    cmd_I2C[1] = cmd_u | (LCD_CMD_BYTE_OFF & _LCDBackLight); // COMD-led-en-rw-rs ,enable=0 and rs =0
    cmd_I2C[2] = cmd_l | (LCD_CMD_BYTE_ON & _LCDBackLight); //  COMD-led-en-rw-rs ,enable=1 and rs =0
    cmd_I2C[3] = cmd_l | (LCD_CMD_BYTE_OFF & _LCDBackLight); // COMD-led-en-rw-rs ,enable=0 and rs =0

    I2C4_Write(_LCDSlaveAddresI2C, cmd_I2C, 4);
    while (I2C4_IsBusy() == true);
}

// Func Desc: Clear a line by writing spaces to every position
// Param1: enum LCDLineNumber_e lineNo, row number 1-4

void PCF8574_LCDClearLine(LCDLineNumber_e lineNo) {
    switch (lineNo) {
        case LCDLineNumberOne:
            PCF8574_LCDSendCmd(LCD_LINE_ADR1);
            break;
        case LCDLineNumberTwo:
            PCF8574_LCDSendCmd(LCD_LINE_ADR2);
            break;
        case LCDLineNumberThree:
            if (_NumColsLCD == 20)
                PCF8574_LCDSendCmd(LCD_LINE_ADR3_20);
            else
                PCF8574_LCDSendCmd(LCD_LINE_ADR3_16);
            break;
        case LCDLineNumberFour:
            if (_NumColsLCD == 20)
                PCF8574_LCDSendCmd(LCD_LINE_ADR4_20);
            else
                PCF8574_LCDSendCmd(LCD_LINE_ADR4_16);
            ;
            break;
    }

    for (uint8_t i = 0; i < _NumColsLCD; i++) {
        PCF8574_LCDSendData(' ');
    }
}

// Func Desc: Clear screen by writing spaces to every position
// Note : See also LCDClearScreenCmd for software command  clear alternative.

void PCF8574_LCDClearScreen(void) {
    if (_NumRowsLCD >= 1)
        PCF8574_LCDClearLine(LCDLineNumberOne);
    if (_NumRowsLCD >= 2)
        PCF8574_LCDClearLine(LCDLineNumberTwo);
    if (_NumRowsLCD >= 3)
        PCF8574_LCDClearLine(LCDLineNumberThree);
    if (_NumRowsLCD == 4)
        PCF8574_LCDClearLine(LCDLineNumberFour);
}


// Func Desc: Reset screen 
// Param1: enum LCD_CURSOR_TYPE_e cursor type, 4 choices
// Warning: if you are using a non-default entry mode this function will
// reset entry mode to default 

void PCF8574_LCDResetScreen(LCDCursorType_e cursorType) {
    PCF8574_LCDSendCmd(LCD_MODE_4BIT);
    PCF8574_LCDSendCmd(LCD_DISPLAY_ON);
    PCF8574_LCDSendCmd(cursorType);
    PCF8574_LCDSendCmd(LCDEntryModeThree);
    PCF8574_LCDSendCmd(LCD_CLRSCR);
    CORETIMER_DelayMs(5);

}


// Func Desc: Turn Screen on and off 
// Param1: passed bool, True = display on , false = display off

void PCF8574_LCDDisplayON(bool OnOff) {
    OnOff ? PCF8574_LCDSendCmd(LCD_DISPLAY_ON) : PCF8574_LCDSendCmd(LCD_DISPLAY_OFF);
    CORETIMER_DelayMs(5);
}


// Func Desc: Initialise LCD
// Param1: enum LCD_CURSOR_TYPE_e cursor type, 4 choices. 
// Param2: num of rows in LCD
// Param3: num of cols in LCD
// Param4: PCF8574 I2C slave address

void PCF8574_LCDInit(LCDCursorType_e cursorType, uint8_t numRow, uint8_t numCol, uint8_t I2Caddress) {

    CORETIMER_DelayMs(50);
    PCF8574_LCDSendCmd(LCD_HOME);
    CORETIMER_DelayMs(5);
    PCF8574_LCDSendCmd(LCD_HOME);
    CORETIMER_DelayMs(5);
    PCF8574_LCDSendCmd(LCD_HOME);
    CORETIMER_DelayMs(1);
    PCF8574_LCDSendCmd(LCD_MODE_4BIT);
    PCF8574_LCDSendCmd(LCD_DISPLAY_ON);
    PCF8574_LCDSendCmd(cursorType);
    PCF8574_LCDSendCmd(LCDEntryModeThree);
    PCF8574_LCDSendCmd(LCD_CLRSCR);
    CORETIMER_DelayMs(5);


    _NumRowsLCD = numRow;
    _NumColsLCD = numCol;
    _LCDSlaveAddresI2C = I2Caddress;

}

// Func Desc: Send string to LCD
// Param1: Pointer to the char array

void PCF8574_LCDSendString(char *str) {
    while (*str) PCF8574_LCDSendData(*str++);
}


// Func Desc: Sends a character to screen , simply wraps SendData command.
// Param1: Character to display

void PCF8574_LCDSendChar(char data) {
    PCF8574_LCDSendData(data);
}

// Func Desc: Moves cursor 
// Param1. enum LCD_DIRECTION_TYPE_e left or right 
// Param2. uint8_t number of spaces to move

void PCF8574_LCDMoveCursor(LCDDirectionType_e direction, uint8_t moveSize) {
    uint8_t i = 0;
    if (direction == LCDMoveRight) {
        for (i = 0; i < moveSize; i++) {
            PCF8574_LCDSendCmd(LCD_MOV_CURSOR_RIGHT);
        }
    } else {
        for (i = 0; i < moveSize; i++) {
            PCF8574_LCDSendCmd(LCD_MOV_CURSOR_LEFT);
        }
    }

}

// Func Desc: Scrolls screen 
// Param1. enum LCD_DIRECTION_TYPE_e , left or right 
// Param2. uint8_t number of spaces to scroll

void PCF8574_LCDScroll(LCDDirectionType_e direction, uint8_t ScrollSize) {
    uint8_t i = 0;
    if (direction == LCDMoveRight) {
        for (i = 0; i < ScrollSize; i++) {
            PCF8574_LCDSendCmd(LCD_SCROLL_RIGHT);
        }
    } else {
        for (i = 0; i < ScrollSize; i++) {
            PCF8574_LCDSendCmd(LCD_SCROLL_LEFT);
        }
    }

}
// Func Desc: moves cursor to X Y position 
// Param1: enum LCDLineNumber_e  row 1-4
// Param2: uint8_t col 0 -> _NumColsLCD value

void PCF8574_LCDGOTO(LCDLineNumber_e line, uint8_t col) {
    switch (line) {
        case LCDLineNumberOne:
            PCF8574_LCDSendCmd(LCD_LINE_ADR1 | col);
            break;
        case LCDLineNumberTwo:
            PCF8574_LCDSendCmd(LCD_LINE_ADR2 | col);
            break;
        case LCDLineNumberThree:
            if (_NumColsLCD == 20)
                PCF8574_LCDSendCmd(LCD_LINE_ADR3_20 + col);
            else
                PCF8574_LCDSendCmd(LCD_LINE_ADR3_16 | col);
            break;
        case LCDLineNumberFour:
            if (_NumColsLCD == 20)
                PCF8574_LCDSendCmd(LCD_LINE_ADR4_20 + col);
            else
                PCF8574_LCDSendCmd(LCD_LINE_ADR4_16 | col);
            ;
            break;
    }
}

// Func Desc: Saves a custom character to a location in CG_RAM
// Param1: CG_RAM location 0-7 we only have 8 locations 0-7
// Param2: An array of 8 bytes representing a custom character data

void PCF8574_LCDCreateCustomChar(uint8_t location, uint8_t * charmap) {
    if (location >= 8) {
        return;
    }
    // Base ram address 64 + location * 8
    PCF8574_LCDSendCmd(LCD_CG_RAM | (location << 3));
    for (uint8_t i = 0; i < 8; i++) {
        PCF8574_LCDSendData(charmap[i]);
    }
}

// Print out a custom character from CGRAM
// Param1 CGRAM location 0-7 

void PCF8574_LCDPrintCustomChar(uint8_t location) {
    if (location >= 8) {return;}

    PCF8574_LCDSendData(location);
}

// Func Desc: Turn LED backlight on and off 
// Param1: passed bool True = LED on , false = display LED off
// Note: another command or data must be issued before takes effect.

void PCF8574_LCDBackLightSet(bool OnOff) {
    OnOff ? (_LCDBackLight = LCD_BACKLIGHTON_MASK) : (_LCDBackLight = LCD_BACKLIGHTOFF_MASK);
}


// Func Desc: vsprintf wrapper to print numerical data
// Parameters: https://www.tutorialspoint.com/c_standard_library/c_function_vsprintf.htm 
// The C library function int vsprintf(char *str, const char *format, va_list arg) 
// sends formatted output to a string using an argument list passed to it.
// Returns: If successful, the total number of characters written is returned, 
// otherwise a negative number is returned.
// Note: requires stdio.h and stdarg.h libraries

int PCF8574_LCDPrintf(const char *fmt, ...) {
    int length;
    char buffer[(_NumColsLCD * _NumRowsLCD) + 1];
    va_list ap;

    va_start(ap, fmt);
    length = vsprintf(buffer, fmt, ap);
    va_end(ap);
    if (length > 0) {
        PCF8574_LCDSendString(buffer);
    }
    return length;
}

// Clear display using software command , set cursor position to zero
// See also LCDClearScreen for manual clear 

void PCF8574_LCDClearScreenCmd(void) {
    PCF8574_LCDSendCmd(LCD_CLRSCR);
    CORETIMER_DelayMs(2); // Requires a delay
}

// Set Cursor position to zero

void PCF8574_LCDHome(void) {
    PCF8574_LCDSendCmd(LCD_HOME);
    CORETIMER_DelayMs(2); // Requires a delay
}

// ********* EOF ********