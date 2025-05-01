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


/*** Macros for P2 pin ***/
#define P2_Set()               (LATBSET = (1U<<14))
#define P2_Clear()             (LATBCLR = (1U<<14))
#define P2_Toggle()            (LATBINV= (1U<<14))
#define P2_OutputEnable()      (TRISBCLR = (1U<<14))
#define P2_InputEnable()       (TRISBSET = (1U<<14))
#define P2_Get()               ((PORTB >> 14) & 0x1U)
#define P2_PIN                  GPIO_PIN_RB14

/*** Macros for P7 pin ***/
#define P7_Set()               (LATBSET = (1U<<15))
#define P7_Clear()             (LATBCLR = (1U<<15))
#define P7_Toggle()            (LATBINV= (1U<<15))
#define P7_OutputEnable()      (TRISBCLR = (1U<<15))
#define P7_InputEnable()       (TRISBSET = (1U<<15))
#define P7_Get()               ((PORTB >> 15) & 0x1U)
#define P7_PIN                  GPIO_PIN_RB15

/*** Macros for SENS10 pin ***/
#define SENS10_Set()               (LATASET = (1U<<11))
#define SENS10_Clear()             (LATACLR = (1U<<11))
#define SENS10_Toggle()            (LATAINV= (1U<<11))
#define SENS10_OutputEnable()      (TRISACLR = (1U<<11))
#define SENS10_InputEnable()       (TRISASET = (1U<<11))
#define SENS10_Get()               ((PORTA >> 11) & 0x1U)
#define SENS10_PIN                  GPIO_PIN_RA11
#define SENS10_InterruptEnable()   (CNENASET = (1U<<11))
#define SENS10_InterruptDisable()  (CNENACLR = (1U<<11))

/*** Macros for SENS9 pin ***/
#define SENS9_Set()               (LATASET = (1U<<0))
#define SENS9_Clear()             (LATACLR = (1U<<0))
#define SENS9_Toggle()            (LATAINV= (1U<<0))
#define SENS9_OutputEnable()      (TRISACLR = (1U<<0))
#define SENS9_InputEnable()       (TRISASET = (1U<<0))
#define SENS9_Get()               ((PORTA >> 0) & 0x1U)
#define SENS9_PIN                  GPIO_PIN_RA0
#define SENS9_InterruptEnable()   (CNENASET = (1U<<0))
#define SENS9_InterruptDisable()  (CNENACLR = (1U<<0))

/*** Macros for SENS8 pin ***/
#define SENS8_Set()               (LATASET = (1U<<1))
#define SENS8_Clear()             (LATACLR = (1U<<1))
#define SENS8_Toggle()            (LATAINV= (1U<<1))
#define SENS8_OutputEnable()      (TRISACLR = (1U<<1))
#define SENS8_InputEnable()       (TRISASET = (1U<<1))
#define SENS8_Get()               ((PORTA >> 1) & 0x1U)
#define SENS8_PIN                  GPIO_PIN_RA1
#define SENS8_InterruptEnable()   (CNENASET = (1U<<1))
#define SENS8_InterruptDisable()  (CNENACLR = (1U<<1))

/*** Macros for SENS7 pin ***/
#define SENS7_Set()               (LATBSET = (1U<<0))
#define SENS7_Clear()             (LATBCLR = (1U<<0))
#define SENS7_Toggle()            (LATBINV= (1U<<0))
#define SENS7_OutputEnable()      (TRISBCLR = (1U<<0))
#define SENS7_InputEnable()       (TRISBSET = (1U<<0))
#define SENS7_Get()               ((PORTB >> 0) & 0x1U)
#define SENS7_PIN                  GPIO_PIN_RB0
#define SENS7_InterruptEnable()   (CNENBSET = (1U<<0))
#define SENS7_InterruptDisable()  (CNENBCLR = (1U<<0))

/*** Macros for SENS6 pin ***/
#define SENS6_Set()               (LATBSET = (1U<<1))
#define SENS6_Clear()             (LATBCLR = (1U<<1))
#define SENS6_Toggle()            (LATBINV= (1U<<1))
#define SENS6_OutputEnable()      (TRISBCLR = (1U<<1))
#define SENS6_InputEnable()       (TRISBSET = (1U<<1))
#define SENS6_Get()               ((PORTB >> 1) & 0x1U)
#define SENS6_PIN                  GPIO_PIN_RB1
#define SENS6_InterruptEnable()   (CNENBSET = (1U<<1))
#define SENS6_InterruptDisable()  (CNENBCLR = (1U<<1))

/*** Macros for SENS5 pin ***/
#define SENS5_Set()               (LATBSET = (1U<<2))
#define SENS5_Clear()             (LATBCLR = (1U<<2))
#define SENS5_Toggle()            (LATBINV= (1U<<2))
#define SENS5_OutputEnable()      (TRISBCLR = (1U<<2))
#define SENS5_InputEnable()       (TRISBSET = (1U<<2))
#define SENS5_Get()               ((PORTB >> 2) & 0x1U)
#define SENS5_PIN                  GPIO_PIN_RB2
#define SENS5_InterruptEnable()   (CNENBSET = (1U<<2))
#define SENS5_InterruptDisable()  (CNENBCLR = (1U<<2))

/*** Macros for SENS4 pin ***/
#define SENS4_Set()               (LATBSET = (1U<<3))
#define SENS4_Clear()             (LATBCLR = (1U<<3))
#define SENS4_Toggle()            (LATBINV= (1U<<3))
#define SENS4_OutputEnable()      (TRISBCLR = (1U<<3))
#define SENS4_InputEnable()       (TRISBSET = (1U<<3))
#define SENS4_Get()               ((PORTB >> 3) & 0x1U)
#define SENS4_PIN                  GPIO_PIN_RB3
#define SENS4_InterruptEnable()   (CNENBSET = (1U<<3))
#define SENS4_InterruptDisable()  (CNENBCLR = (1U<<3))

/*** Macros for SENS3 pin ***/
#define SENS3_Set()               (LATCSET = (1U<<0))
#define SENS3_Clear()             (LATCCLR = (1U<<0))
#define SENS3_Toggle()            (LATCINV= (1U<<0))
#define SENS3_OutputEnable()      (TRISCCLR = (1U<<0))
#define SENS3_InputEnable()       (TRISCSET = (1U<<0))
#define SENS3_Get()               ((PORTC >> 0) & 0x1U)
#define SENS3_PIN                  GPIO_PIN_RC0
#define SENS3_InterruptEnable()   (CNENCSET = (1U<<0))
#define SENS3_InterruptDisable()  (CNENCCLR = (1U<<0))

/*** Macros for SENS2 pin ***/
#define SENS2_Set()               (LATCSET = (1U<<1))
#define SENS2_Clear()             (LATCCLR = (1U<<1))
#define SENS2_Toggle()            (LATCINV= (1U<<1))
#define SENS2_OutputEnable()      (TRISCCLR = (1U<<1))
#define SENS2_InputEnable()       (TRISCSET = (1U<<1))
#define SENS2_Get()               ((PORTC >> 1) & 0x1U)
#define SENS2_PIN                  GPIO_PIN_RC1
#define SENS2_InterruptEnable()   (CNENCSET = (1U<<1))
#define SENS2_InterruptDisable()  (CNENCCLR = (1U<<1))

/*** Macros for SENS1 pin ***/
#define SENS1_Set()               (LATCSET = (1U<<2))
#define SENS1_Clear()             (LATCCLR = (1U<<2))
#define SENS1_Toggle()            (LATCINV= (1U<<2))
#define SENS1_OutputEnable()      (TRISCCLR = (1U<<2))
#define SENS1_InputEnable()       (TRISCSET = (1U<<2))
#define SENS1_Get()               ((PORTC >> 2) & 0x1U)
#define SENS1_PIN                  GPIO_PIN_RC2
#define SENS1_InterruptEnable()   (CNENCSET = (1U<<2))
#define SENS1_InterruptDisable()  (CNENCCLR = (1U<<2))

/*** Macros for P1 pin ***/
#define P1_Set()               (LATCSET = (1U<<9))
#define P1_Clear()             (LATCCLR = (1U<<9))
#define P1_Toggle()            (LATCINV= (1U<<9))
#define P1_OutputEnable()      (TRISCCLR = (1U<<9))
#define P1_InputEnable()       (TRISCSET = (1U<<9))
#define P1_Get()               ((PORTC >> 9) & 0x1U)
#define P1_PIN                  GPIO_PIN_RC9

/*** Macros for P4 pin ***/
#define P4_Set()               (LATFSET = (1U<<0))
#define P4_Clear()             (LATFCLR = (1U<<0))
#define P4_Toggle()            (LATFINV= (1U<<0))
#define P4_OutputEnable()      (TRISFCLR = (1U<<0))
#define P4_InputEnable()       (TRISFSET = (1U<<0))
#define P4_Get()               ((PORTF >> 0) & 0x1U)
#define P4_PIN                  GPIO_PIN_RF0

/*** Macros for P5 pin ***/
#define P5_Set()               (LATFSET = (1U<<1))
#define P5_Clear()             (LATFCLR = (1U<<1))
#define P5_Toggle()            (LATFINV= (1U<<1))
#define P5_OutputEnable()      (TRISFCLR = (1U<<1))
#define P5_InputEnable()       (TRISFSET = (1U<<1))
#define P5_Get()               ((PORTF >> 1) & 0x1U)
#define P5_PIN                  GPIO_PIN_RF1

/*** Macros for P3 pin ***/
#define P3_Set()               (LATBSET = (1U<<12))
#define P3_Clear()             (LATBCLR = (1U<<12))
#define P3_Toggle()            (LATBINV= (1U<<12))
#define P3_OutputEnable()      (TRISBCLR = (1U<<12))
#define P3_InputEnable()       (TRISBSET = (1U<<12))
#define P3_Get()               ((PORTB >> 12) & 0x1U)
#define P3_PIN                  GPIO_PIN_RB12

/*** Macros for P6 pin ***/
#define P6_Set()               (LATASET = (1U<<10))
#define P6_Clear()             (LATACLR = (1U<<10))
#define P6_Toggle()            (LATAINV= (1U<<10))
#define P6_OutputEnable()      (TRISACLR = (1U<<10))
#define P6_InputEnable()       (TRISASET = (1U<<10))
#define P6_Get()               ((PORTA >> 10) & 0x1U)
#define P6_PIN                  GPIO_PIN_RA10


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

typedef  void (*GPIO_PIN_CALLBACK) ( GPIO_PIN pin, uintptr_t context);

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

void GPIO_PortInterruptEnable(GPIO_PORT port, uint32_t mask);

void GPIO_PortInterruptDisable(GPIO_PORT port, uint32_t mask);

// *****************************************************************************
// *****************************************************************************
// Section: Local Data types and Prototypes
// *****************************************************************************
// *****************************************************************************

typedef struct {

    /* target pin */
    GPIO_PIN                 pin;

    /* Callback for event on target pin*/
    GPIO_PIN_CALLBACK        callback;

    /* Callback Context */
    uintptr_t               context;

} GPIO_PIN_CALLBACK_OBJ;

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

#define GPIO_PinInterruptEnable(pin)       GPIO_PinIntEnable(pin, GPIO_INTERRUPT_ON_MISMATCH)
#define GPIO_PinInterruptDisable(pin)      GPIO_PinIntDisable(pin)

void GPIO_PinIntEnable(GPIO_PIN pin, GPIO_INTERRUPT_STYLE style);
void GPIO_PinIntDisable(GPIO_PIN pin);

bool GPIO_PinInterruptCallbackRegister(
    GPIO_PIN pin,
    const   GPIO_PIN_CALLBACK callBack,
    uintptr_t context
);

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
// DOM-IGNORE-END
#endif // PLIB_GPIO_H
