#include "pin.h"
#include "config/default/peripheral/uart/plib_uart6.h"

#define MAX_PINS 12

typedef struct{
    volatile uint32_t *reg_set;
    volatile uint32_t *reg_rst;
    uint8_t pin;
    uint8_t mask;
}
pin_t;

const pin_t p1 = {
    .reg_set = &LATDSET,
    .reg_rst = &LATDCLR,
    .pin = 5,
    .mask = 22
};

const pin_t p2 = {
    .reg_set = &LATCSET,
    .reg_rst = &LATCCLR,
    .pin = 8,
    .mask = 20
};

const pin_t p3 = {
    .reg_set=&LATCSET,
    .reg_rst=&LATCCLR,
    .pin = 6,
    .mask = 18
};

const pin_t p4 = {
    .reg_set=&LATBSET,
    .reg_rst=&LATBCLR,
    .pin=9,
    .mask = 16
};

const pin_t p5 = {
    .reg_set=&LATDSET,
    .reg_rst=&LATDCLR,
    .pin=6,
    .mask = 14
};

const pin_t p6 = {
    .reg_set=&LATCSET,
    .reg_rst=&LATCCLR,
    .pin=7,
    .mask = 12
};

const pin_t v1 = {
    .reg_set=&LATGSET,
    .reg_rst=&LATGCLR,
    .pin=6,
    .mask = 10
};

const pin_t v2 = {
    .reg_set=&LATBSET,
    .reg_rst=&LATBCLR,
    .pin=14,
    .mask = 8
};

const pin_t v3 = {
    .reg_set=&LATASET,
    .reg_rst=&LATACLR,
    .pin=12,
    .mask = 6
};

const pin_t v4 = {
    .reg_set=&LATGSET,
    .reg_rst=&LATGCLR,
    .pin=9,
    .mask = 4
};

const pin_t v5 = {
    .reg_set=&LATASET,
    .reg_rst=&LATACLR,
    .pin=0,
    .mask = 2
};

const pin_t v6 = {
    .reg_set=&LATASET,
    .reg_rst=&LATACLR,
    .pin=11,
    .mask = 0
};

const pin_t pins[MAX_PINS] = {
    p1, p2, p3, p4, p5, p6, 
    v1, v2, v3, v4, v5, v6
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
    uint32_t msg = ((uint32_t)data[0]<<16)|((uint32_t)data[1]<<8)|((uint32_t)data[2]);
    
    for(uint8_t i=0; i<MAX_PINS; i++)
    {
        set_pin(msg, i);
    }
}

