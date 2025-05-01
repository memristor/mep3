#include "pin.h"
#include "config/default/peripheral/uart/plib_uart3.h"

#define MAX_PINS 7

typedef struct{
    volatile uint32_t *reg_set;
    volatile uint32_t *reg_rst;
    uint8_t pin;
    uint8_t mask;
}
pin_t;


//broj 1 na ActuatorShield-u - RC9 pin sa senzorske
const pin_t p1 = {
    .reg_set=&LATCSET,
    .reg_rst=&LATCCLR,
    .pin=9,
    .mask = 12
};

//broj 2 na ActuatorShield-u - RB14 pin sa senzorske
const pin_t p2 = {
    .reg_set=&LATBSET,
    .reg_rst=&LATBCLR,
    .pin=14,
    .mask = 10
};

//broj 3 na ActuatorShield-u - RB12 pin sa senzorske
const pin_t p3 = {
    .reg_set=&LATBSET,
    .reg_rst=&LATBCLR,
    .pin=12,
    .mask = 8
};

//broj 4 na ActuatorShield-u - RF0 pin sa senzorske
const pin_t p4 = {
    .reg_set=&LATFSET,
    .reg_rst=&LATFCLR,
    .pin=0,
    .mask = 6
};

//broj 5 na ActuatorShield-u - RF1 pin sa senzorske
const pin_t p5 = {
    .reg_set=&LATFSET,
    .reg_rst=&LATFCLR,
    .pin=1,
    .mask = 4
};

//broj 6 na ActuatorShieldu - RA10 pin sa senzorske
const pin_t p6 = {
    .reg_set=&LATASET,
    .reg_rst=&LATACLR,
    .pin=10,
    .mask = 2
};

//broj 7 na ActuatorShieldu - RB15 pin sa senzorske
const pin_t p7 = {
    .reg_set=&LATBSET,
    .reg_rst=&LATBCLR,
    .pin=15,
    .mask = 0
};

const pin_t pins[MAX_PINS] = {
    p1, p2, p3, p4, p5, p6, p7
};

static void set_pin(uint32_t msg, uint8_t pin)
{
    switch ((msg & (3<<pins[pin].mask))>>pins[pin].mask){
        case 0:
            *(pins[pin].reg_rst) = (1U << pins[pin].pin);
            break;
        case 1:
            *(pins[pin].reg_set) = (1U << pins[pin].pin);
            break;
        case 2:
        case 3:
            break;   
    }
}

void set_pin_state(uint8_t *data)
{
    uint32_t msg = ((uint32_t)data[0]<<8)|((uint32_t)data[1]);
    
    for(uint8_t i=0; i<MAX_PINS; i++)
    {
        set_pin(msg, i);
    }
}

