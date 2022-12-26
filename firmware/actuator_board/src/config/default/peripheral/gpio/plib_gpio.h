/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h

  Summary:
    GPIO PLIB Header File

  Description:
    This library provides an interface to control and interact with Parallel
    Input/Output controller (GPIO) module.

*******************************************************************************/

/*******************************************************************************
* Copyright (C) 2019 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*******************************************************************************/

#ifndef PLIB_GPIO_H
#define PLIB_GPIO_H

#include <device.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Data types and constants
// *****************************************************************************
// *****************************************************************************


/*** Macros for RB14_VALVE_2 pin ***/
#define RB14_VALVE_2_Set()               (LATBSET = (1<<14))
#define RB14_VALVE_2_Clear()             (LATBCLR = (1<<14))
#define RB14_VALVE_2_Toggle()            (LATBINV= (1<<14))
#define RB14_VALVE_2_OutputEnable()      (TRISBCLR = (1<<14))
#define RB14_VALVE_2_InputEnable()       (TRISBSET = (1<<14))
#define RB14_VALVE_2_Get()               ((PORTB >> 14) & 0x1)
#define RB14_VALVE_2_PIN                  GPIO_PIN_RB14

/*** Macros for RG6_VALVE_1 pin ***/
#define RG6_VALVE_1_Set()               (LATGSET = (1<<6))
#define RG6_VALVE_1_Clear()             (LATGCLR = (1<<6))
#define RG6_VALVE_1_Toggle()            (LATGINV= (1<<6))
#define RG6_VALVE_1_OutputEnable()      (TRISGCLR = (1<<6))
#define RG6_VALVE_1_InputEnable()       (TRISGSET = (1<<6))
#define RG6_VALVE_1_Get()               ((PORTG >> 6) & 0x1)
#define RG6_VALVE_1_PIN                  GPIO_PIN_RG6

/*** Macros for RG9_VALVE_4 pin ***/
#define RG9_VALVE_4_Set()               (LATGSET = (1<<9))
#define RG9_VALVE_4_Clear()             (LATGCLR = (1<<9))
#define RG9_VALVE_4_Toggle()            (LATGINV= (1<<9))
#define RG9_VALVE_4_OutputEnable()      (TRISGCLR = (1<<9))
#define RG9_VALVE_4_InputEnable()       (TRISGSET = (1<<9))
#define RG9_VALVE_4_Get()               ((PORTG >> 9) & 0x1)
#define RG9_VALVE_4_PIN                  GPIO_PIN_RG9

/*** Macros for RA12_VALVE_3 pin ***/
#define RA12_VALVE_3_Set()               (LATASET = (1<<12))
#define RA12_VALVE_3_Clear()             (LATACLR = (1<<12))
#define RA12_VALVE_3_Toggle()            (LATAINV= (1<<12))
#define RA12_VALVE_3_OutputEnable()      (TRISACLR = (1<<12))
#define RA12_VALVE_3_InputEnable()       (TRISASET = (1<<12))
#define RA12_VALVE_3_Get()               ((PORTA >> 12) & 0x1)
#define RA12_VALVE_3_PIN                  GPIO_PIN_RA12

/*** Macros for RA11_VALVE_6 pin ***/
#define RA11_VALVE_6_Set()               (LATASET = (1<<11))
#define RA11_VALVE_6_Clear()             (LATACLR = (1<<11))
#define RA11_VALVE_6_Toggle()            (LATAINV= (1<<11))
#define RA11_VALVE_6_OutputEnable()      (TRISACLR = (1<<11))
#define RA11_VALVE_6_InputEnable()       (TRISASET = (1<<11))
#define RA11_VALVE_6_Get()               ((PORTA >> 11) & 0x1)
#define RA11_VALVE_6_PIN                  GPIO_PIN_RA11

/*** Macros for RA0_VALVE_5 pin ***/
#define RA0_VALVE_5_Set()               (LATASET = (1<<0))
#define RA0_VALVE_5_Clear()             (LATACLR = (1<<0))
#define RA0_VALVE_5_Toggle()            (LATAINV= (1<<0))
#define RA0_VALVE_5_OutputEnable()      (TRISACLR = (1<<0))
#define RA0_VALVE_5_InputEnable()       (TRISASET = (1<<0))
#define RA0_VALVE_5_Get()               ((PORTA >> 0) & 0x1)
#define RA0_VALVE_5_PIN                  GPIO_PIN_RA0

/*** Macros for RB9_PUMP_4 pin ***/
#define RB9_PUMP_4_Set()               (LATBSET = (1<<9))
#define RB9_PUMP_4_Clear()             (LATBCLR = (1<<9))
#define RB9_PUMP_4_Toggle()            (LATBINV= (1<<9))
#define RB9_PUMP_4_OutputEnable()      (TRISBCLR = (1<<9))
#define RB9_PUMP_4_InputEnable()       (TRISBSET = (1<<9))
#define RB9_PUMP_4_Get()               ((PORTB >> 9) & 0x1)
#define RB9_PUMP_4_PIN                  GPIO_PIN_RB9

/*** Macros for RC6_PUMP_3 pin ***/
#define RC6_PUMP_3_Set()               (LATCSET = (1<<6))
#define RC6_PUMP_3_Clear()             (LATCCLR = (1<<6))
#define RC6_PUMP_3_Toggle()            (LATCINV= (1<<6))
#define RC6_PUMP_3_OutputEnable()      (TRISCCLR = (1<<6))
#define RC6_PUMP_3_InputEnable()       (TRISCSET = (1<<6))
#define RC6_PUMP_3_Get()               ((PORTC >> 6) & 0x1)
#define RC6_PUMP_3_PIN                  GPIO_PIN_RC6

/*** Macros for RC7_PUMP_6 pin ***/
#define RC7_PUMP_6_Set()               (LATCSET = (1<<7))
#define RC7_PUMP_6_Clear()             (LATCCLR = (1<<7))
#define RC7_PUMP_6_Toggle()            (LATCINV= (1<<7))
#define RC7_PUMP_6_OutputEnable()      (TRISCCLR = (1<<7))
#define RC7_PUMP_6_InputEnable()       (TRISCSET = (1<<7))
#define RC7_PUMP_6_Get()               ((PORTC >> 7) & 0x1)
#define RC7_PUMP_6_PIN                  GPIO_PIN_RC7

/*** Macros for RC8_PUMP_2 pin ***/
#define RC8_PUMP_2_Set()               (LATCSET = (1<<8))
#define RC8_PUMP_2_Clear()             (LATCCLR = (1<<8))
#define RC8_PUMP_2_Toggle()            (LATCINV= (1<<8))
#define RC8_PUMP_2_OutputEnable()      (TRISCCLR = (1<<8))
#define RC8_PUMP_2_InputEnable()       (TRISCSET = (1<<8))
#define RC8_PUMP_2_Get()               ((PORTC >> 8) & 0x1)
#define RC8_PUMP_2_PIN                  GPIO_PIN_RC8

/*** Macros for RD5_PUMP_1 pin ***/
#define RD5_PUMP_1_Set()               (LATDSET = (1<<5))
#define RD5_PUMP_1_Clear()             (LATDCLR = (1<<5))
#define RD5_PUMP_1_Toggle()            (LATDINV= (1<<5))
#define RD5_PUMP_1_OutputEnable()      (TRISDCLR = (1<<5))
#define RD5_PUMP_1_InputEnable()       (TRISDSET = (1<<5))
#define RD5_PUMP_1_Get()               ((PORTD >> 5) & 0x1)
#define RD5_PUMP_1_PIN                  GPIO_PIN_RD5

/*** Macros for RD6_PUMP_5 pin ***/
#define RD6_PUMP_5_Set()               (LATDSET = (1<<6))
#define RD6_PUMP_5_Clear()             (LATDCLR = (1<<6))
#define RD6_PUMP_5_Toggle()            (LATDINV= (1<<6))
#define RD6_PUMP_5_OutputEnable()      (TRISDCLR = (1<<6))
#define RD6_PUMP_5_InputEnable()       (TRISDSET = (1<<6))
#define RD6_PUMP_5_Get()               ((PORTD >> 6) & 0x1)
#define RD6_PUMP_5_PIN                  GPIO_PIN_RD6


// *****************************************************************************
/* GPIO Port

  Summary:
    Identifies the available GPIO Ports.

  Description:
    This enumeration identifies the available GPIO Ports.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all ports are available on all devices.  Refer to the specific
    device data sheet to determine which ports are supported.
*/

typedef enum
{
    GPIO_PORT_A = 0,
    GPIO_PORT_B = 1,
    GPIO_PORT_C = 2,
    GPIO_PORT_D = 3,
    GPIO_PORT_E = 4,
    GPIO_PORT_F = 5,
    GPIO_PORT_G = 6,
} GPIO_PORT;

typedef enum
{
    GPIO_INTERRUPT_ON_MISMATCH,
    GPIO_INTERRUPT_ON_RISING_EDGE,
    GPIO_INTERRUPT_ON_FALLING_EDGE,
    GPIO_INTERRUPT_ON_BOTH_EDGES,
}GPIO_INTERRUPT_STYLE;

// *****************************************************************************
/* GPIO Port Pins

  Summary:
    Identifies the available GPIO port pins.

  Description:
    This enumeration identifies the available GPIO port pins.

  Remarks:
    The caller should not rely on the specific numbers assigned to any of
    these values as they may change from one processor to the next.

    Not all pins are available on all devices.  Refer to the specific
    device data sheet to determine which pins are supported.
*/

typedef enum
{
    GPIO_PIN_RA0 = 0,
    GPIO_PIN_RA1 = 1,
    GPIO_PIN_RA4 = 4,
    GPIO_PIN_RA7 = 7,
    GPIO_PIN_RA8 = 8,
    GPIO_PIN_RA10 = 10,
    GPIO_PIN_RA11 = 11,
    GPIO_PIN_RA12 = 12,
    GPIO_PIN_RB0 = 16,
    GPIO_PIN_RB1 = 17,
    GPIO_PIN_RB2 = 18,
    GPIO_PIN_RB3 = 19,
    GPIO_PIN_RB4 = 20,
    GPIO_PIN_RB5 = 21,
    GPIO_PIN_RB6 = 22,
    GPIO_PIN_RB7 = 23,
    GPIO_PIN_RB8 = 24,
    GPIO_PIN_RB9 = 25,
    GPIO_PIN_RB10 = 26,
    GPIO_PIN_RB11 = 27,
    GPIO_PIN_RB12 = 28,
    GPIO_PIN_RB13 = 29,
    GPIO_PIN_RB14 = 30,
    GPIO_PIN_RB15 = 31,
    GPIO_PIN_RC0 = 32,
    GPIO_PIN_RC1 = 33,
    GPIO_PIN_RC2 = 34,
    GPIO_PIN_RC6 = 38,
    GPIO_PIN_RC7 = 39,
    GPIO_PIN_RC8 = 40,
    GPIO_PIN_RC9 = 41,
    GPIO_PIN_RC10 = 42,
    GPIO_PIN_RC11 = 43,
    GPIO_PIN_RC12 = 44,
    GPIO_PIN_RC13 = 45,
    GPIO_PIN_RC15 = 47,
    GPIO_PIN_RD5 = 53,
    GPIO_PIN_RD6 = 54,
    GPIO_PIN_RD8 = 56,
    GPIO_PIN_RE12 = 76,
    GPIO_PIN_RE13 = 77,
    GPIO_PIN_RE14 = 78,
    GPIO_PIN_RE15 = 79,
    GPIO_PIN_RF0 = 80,
    GPIO_PIN_RF1 = 81,
    GPIO_PIN_RG6 = 102,
    GPIO_PIN_RG7 = 103,
    GPIO_PIN_RG8 = 104,
    GPIO_PIN_RG9 = 105,

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
    GPIO_PIN_NONE = -1

} GPIO_PIN;


void GPIO_Initialize(void);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on multiple pins of a port
// *****************************************************************************
// *****************************************************************************

uint32_t GPIO_PortRead(GPIO_PORT port);

void GPIO_PortWrite(GPIO_PORT port, uint32_t mask, uint32_t value);

uint32_t GPIO_PortLatchRead ( GPIO_PORT port );

void GPIO_PortSet(GPIO_PORT port, uint32_t mask);

void GPIO_PortClear(GPIO_PORT port, uint32_t mask);

void GPIO_PortToggle(GPIO_PORT port, uint32_t mask);

void GPIO_PortInputEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortOutputEnable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: GPIO Functions which operates on one pin at a time
// *****************************************************************************
// *****************************************************************************

static inline void GPIO_PinWrite(GPIO_PIN pin, bool value)
{
    GPIO_PortWrite((GPIO_PORT)(pin>>4), (uint32_t)(0x1) << (pin & 0xF), (uint32_t)(value) << (pin & 0xF));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return (bool)(((GPIO_PortRead((GPIO_PORT)(pin>>4))) >> (pin & 0xF)) & 0x1);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (bool)((GPIO_PortLatchRead((GPIO_PORT)(pin>>4)) >> (pin & 0xF)) & 0x1);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((GPIO_PORT)(pin>>4), 0x1 << (pin & 0xF));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
