/*******************************************************************************
  GPIO PLIB

  Company:
    Microchip Technology Inc.

  File Name:
    plib_gpio.h UUUUUUUUU

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


/*** Macros for VALVE_2 pin ***/
#define VALVE_2_Set()               (LATBSET = (1U<<14))
#define VALVE_2_Clear()             (LATBCLR = (1U<<14))
#define VALVE_2_Toggle()            (LATBINV= (1U<<14))
#define VALVE_2_OutputEnable()      (TRISBCLR = (1U<<14))
#define VALVE_2_InputEnable()       (TRISBSET = (1U<<14))
#define VALVE_2_Get()               ((PORTB >> 14) & 0x1U)
#define VALVE_2_PIN                  GPIO_PIN_RB14

/*** Macros for VALVE_1 pin ***/
#define VALVE_1_Set()               (LATGSET = (1U<<6))
#define VALVE_1_Clear()             (LATGCLR = (1U<<6))
#define VALVE_1_Toggle()            (LATGINV= (1U<<6))
#define VALVE_1_OutputEnable()      (TRISGCLR = (1U<<6))
#define VALVE_1_InputEnable()       (TRISGSET = (1U<<6))
#define VALVE_1_Get()               ((PORTG >> 6) & 0x1U)
#define VALVE_1_PIN                  GPIO_PIN_RG6

/*** Macros for VALVE_4 pin ***/
#define VALVE_4_Set()               (LATGSET = (1U<<9))
#define VALVE_4_Clear()             (LATGCLR = (1U<<9))
#define VALVE_4_Toggle()            (LATGINV= (1U<<9))
#define VALVE_4_OutputEnable()      (TRISGCLR = (1U<<9))
#define VALVE_4_InputEnable()       (TRISGSET = (1U<<9))
#define VALVE_4_Get()               ((PORTG >> 9) & 0x1U)
#define VALVE_4_PIN                  GPIO_PIN_RG9

/*** Macros for VALVE_3 pin ***/
#define VALVE_3_Set()               (LATASET = (1U<<12))
#define VALVE_3_Clear()             (LATACLR = (1U<<12))
#define VALVE_3_Toggle()            (LATAINV= (1U<<12))
#define VALVE_3_OutputEnable()      (TRISACLR = (1U<<12))
#define VALVE_3_InputEnable()       (TRISASET = (1U<<12))
#define VALVE_3_Get()               ((PORTA >> 12) & 0x1U)
#define VALVE_3_PIN                  GPIO_PIN_RA12

/*** Macros for VALVE_6 pin ***/
#define VALVE_6_Set()               (LATASET = (1U<<11))
#define VALVE_6_Clear()             (LATACLR = (1U<<11))
#define VALVE_6_Toggle()            (LATAINV= (1U<<11))
#define VALVE_6_OutputEnable()      (TRISACLR = (1U<<11))
#define VALVE_6_InputEnable()       (TRISASET = (1U<<11))
#define VALVE_6_Get()               ((PORTA >> 11) & 0x1U)
#define VALVE_6_PIN                  GPIO_PIN_RA11

/*** Macros for VALVE_5 pin ***/
#define VALVE_5_Set()               (LATASET = (1U<<0))
#define VALVE_5_Clear()             (LATACLR = (1U<<0))
#define VALVE_5_Toggle()            (LATAINV= (1U<<0))
#define VALVE_5_OutputEnable()      (TRISACLR = (1U<<0))
#define VALVE_5_InputEnable()       (TRISASET = (1U<<0))
#define VALVE_5_Get()               ((PORTA >> 0) & 0x1U)
#define VALVE_5_PIN                  GPIO_PIN_RA0

/*** Macros for PUMP_4 pin ***/
#define PUMP_4_Set()               (LATBSET = (1U<<9))
#define PUMP_4_Clear()             (LATBCLR = (1U<<9))
#define PUMP_4_Toggle()            (LATBINV= (1U<<9))
#define PUMP_4_OutputEnable()      (TRISBCLR = (1U<<9))
#define PUMP_4_InputEnable()       (TRISBSET = (1U<<9))
#define PUMP_4_Get()               ((PORTB >> 9) & 0x1U)
#define PUMP_4_PIN                  GPIO_PIN_RB9

/*** Macros for PUMP_3 pin ***/
#define PUMP_3_Set()               (LATCSET = (1U<<6))
#define PUMP_3_Clear()             (LATCCLR = (1U<<6))
#define PUMP_3_Toggle()            (LATCINV= (1U<<6))
#define PUMP_3_OutputEnable()      (TRISCCLR = (1U<<6))
#define PUMP_3_InputEnable()       (TRISCSET = (1U<<6))
#define PUMP_3_Get()               ((PORTC >> 6) & 0x1U)
#define PUMP_3_PIN                  GPIO_PIN_RC6

/*** Macros for PUMP_6 pin ***/
#define PUMP_6_Set()               (LATCSET = (1U<<7))
#define PUMP_6_Clear()             (LATCCLR = (1U<<7))
#define PUMP_6_Toggle()            (LATCINV= (1U<<7))
#define PUMP_6_OutputEnable()      (TRISCCLR = (1U<<7))
#define PUMP_6_InputEnable()       (TRISCSET = (1U<<7))
#define PUMP_6_Get()               ((PORTC >> 7) & 0x1U)
#define PUMP_6_PIN                  GPIO_PIN_RC7

/*** Macros for PUMP_2 pin ***/
#define PUMP_2_Set()               (LATCSET = (1U<<8))
#define PUMP_2_Clear()             (LATCCLR = (1U<<8))
#define PUMP_2_Toggle()            (LATCINV= (1U<<8))
#define PUMP_2_OutputEnable()      (TRISCCLR = (1U<<8))
#define PUMP_2_InputEnable()       (TRISCSET = (1U<<8))
#define PUMP_2_Get()               ((PORTC >> 8) & 0x1U)
#define PUMP_2_PIN                  GPIO_PIN_RC8

/*** Macros for PUMP_1 pin ***/
#define PUMP_1_Set()               (LATDSET = (1U<<5))
#define PUMP_1_Clear()             (LATDCLR = (1U<<5))
#define PUMP_1_Toggle()            (LATDINV= (1U<<5))
#define PUMP_1_OutputEnable()      (TRISDCLR = (1U<<5))
#define PUMP_1_InputEnable()       (TRISDSET = (1U<<5))
#define PUMP_1_Get()               ((PORTD >> 5) & 0x1U)
#define PUMP_1_PIN                  GPIO_PIN_RD5

/*** Macros for PUMP_5 pin ***/
#define PUMP_5_Set()               (LATDSET = (1U<<6))
#define PUMP_5_Clear()             (LATDCLR = (1U<<6))
#define PUMP_5_Toggle()            (LATDINV= (1U<<6))
#define PUMP_5_OutputEnable()      (TRISDCLR = (1U<<6))
#define PUMP_5_InputEnable()       (TRISDSET = (1U<<6))
#define PUMP_5_Get()               ((PORTD >> 6) & 0x1U)
#define PUMP_5_PIN                  GPIO_PIN_RD6


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


#define    GPIO_PORT_A  (0)
#define    GPIO_PORT_B  (1)
#define    GPIO_PORT_C  (2)
#define    GPIO_PORT_D  (3)
#define    GPIO_PORT_E  (4)
#define    GPIO_PORT_F  (5)
#define    GPIO_PORT_G  (6)
typedef uint32_t GPIO_PORT;

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


#define     GPIO_PIN_RA0  (0U)
#define     GPIO_PIN_RA1  (1U)
#define     GPIO_PIN_RA4  (4U)
#define     GPIO_PIN_RA7  (7U)
#define     GPIO_PIN_RA8  (8U)
#define     GPIO_PIN_RA10  (10U)
#define     GPIO_PIN_RA11  (11U)
#define     GPIO_PIN_RA12  (12U)
#define     GPIO_PIN_RB0  (16U)
#define     GPIO_PIN_RB1  (17U)
#define     GPIO_PIN_RB2  (18U)
#define     GPIO_PIN_RB3  (19U)
#define     GPIO_PIN_RB4  (20U)
#define     GPIO_PIN_RB5  (21U)
#define     GPIO_PIN_RB6  (22U)
#define     GPIO_PIN_RB7  (23U)
#define     GPIO_PIN_RB8  (24U)
#define     GPIO_PIN_RB9  (25U)
#define     GPIO_PIN_RB10  (26U)
#define     GPIO_PIN_RB11  (27U)
#define     GPIO_PIN_RB12  (28U)
#define     GPIO_PIN_RB13  (29U)
#define     GPIO_PIN_RB14  (30U)
#define     GPIO_PIN_RB15  (31U)
#define     GPIO_PIN_RC0  (32U)
#define     GPIO_PIN_RC1  (33U)
#define     GPIO_PIN_RC2  (34U)
#define     GPIO_PIN_RC6  (38U)
#define     GPIO_PIN_RC7  (39U)
#define     GPIO_PIN_RC8  (40U)
#define     GPIO_PIN_RC9  (41U)
#define     GPIO_PIN_RC10  (42U)
#define     GPIO_PIN_RC11  (43U)
#define     GPIO_PIN_RC12  (44U)
#define     GPIO_PIN_RC13  (45U)
#define     GPIO_PIN_RC15  (47U)
#define     GPIO_PIN_RD5  (53U)
#define     GPIO_PIN_RD6  (54U)
#define     GPIO_PIN_RD8  (56U)
#define     GPIO_PIN_RE12  (76U)
#define     GPIO_PIN_RE13  (77U)
#define     GPIO_PIN_RE14  (78U)
#define     GPIO_PIN_RE15  (79U)
#define     GPIO_PIN_RF0  (80U)
#define     GPIO_PIN_RF1  (81U)
#define     GPIO_PIN_RG6  (102U)
#define     GPIO_PIN_RG7  (103U)
#define     GPIO_PIN_RG8  (104U)
#define     GPIO_PIN_RG9  (105U)

    /* This element should not be used in any of the GPIO APIs.
       It will be used by other modules or application to denote that none of the GPIO Pin is used */
#define    GPIO_PIN_NONE   (-1)

typedef uint32_t GPIO_PIN;


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
	 uint32_t xvalue = (uint32_t)value;
    GPIO_PortWrite((pin>>4U), (uint32_t)(0x1U) << (pin & 0xFU), (xvalue) << (pin & 0xFU));
}

static inline bool GPIO_PinRead(GPIO_PIN pin)
{
    return ((((GPIO_PortRead((GPIO_PORT)(pin>>4U))) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline bool GPIO_PinLatchRead(GPIO_PIN pin)
{
    return (((GPIO_PortLatchRead((GPIO_PORT)(pin>>4U)) >> (pin & 0xFU)) & 0x1U) != 0U);
}

static inline void GPIO_PinToggle(GPIO_PIN pin)
{
    GPIO_PortToggle((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinSet(GPIO_PIN pin)
{
    GPIO_PortSet((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinClear(GPIO_PIN pin)
{
    GPIO_PortClear((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinInputEnable(GPIO_PIN pin)
{
    GPIO_PortInputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}

static inline void GPIO_PinOutputEnable(GPIO_PIN pin)
{
    GPIO_PortOutputEnable((pin>>4U), (uint32_t)0x1U << (pin & 0xFU));
}


// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
