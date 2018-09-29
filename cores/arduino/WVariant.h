/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (C) 2018 Industruino <connect@industruino.com>

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

//  Part of the SAML code ported from Mattairtech ArduinoCore-samd (https://github.com/mattairtech/ArduinoCore-samd):
//     Copyright: Copyright (c) 2017-2018 MattairTech LLC. All right reserved.
//     License: LGPL http://www.gnu.org/licenses/lgpl-2.1.html

#pragma once

#include <stdint.h>
#include "sam.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Definitions and types for pins */
typedef enum _EAnalogChannel
{
  No_ADC_Channel=-1,
#if (SAMD21_SERIES)
  ADC_Channel0=0,
  ADC_Channel1=1,
  ADC_Channel2=2,
  ADC_Channel3=3,
  ADC_Channel4=4,
  ADC_Channel5=5,
  ADC_Channel6=6,
  ADC_Channel7=7,
#if defined __SAMD21J18A__
  ADC_Channel8=8,
  ADC_Channel9=9,
#endif // __SAMD21J18A__
  ADC_Channel10=10,
  ADC_Channel11=11,
#if defined __SAMD21J18A__
  ADC_Channel12=12,
  ADC_Channel13=13,
  ADC_Channel14=14,
  ADC_Channel15=15,
#endif // __SAMD21J18A__
  ADC_Channel16=16,
  ADC_Channel17=17,
  ADC_Channel18=18,
  ADC_Channel19=19,
  DAC_Channel0
#elif (SAML21B_SERIES)
  ADC_Channel0=0,
  ADC_Channel1=1,
  ADC_Channel2=2,
  ADC_Channel3=3,
  ADC_Channel4=4,
  ADC_Channel5=5,
  ADC_Channel6=6,
  ADC_Channel7=7,
  ADC_Channel8=8,
  ADC_Channel9=9,
  ADC_Channel10=10,
  ADC_Channel11=11,
  ADC_Channel12=12,
  ADC_Channel13=13,
  ADC_Channel14=14,
  ADC_Channel15=15,
  ADC_Channel16=16,
  ADC_Channel17=17,
  ADC_Channel18=18,
  ADC_Channel19=19,
  DAC_Channel0,
  DAC_Channel1
#endif
} EAnalogChannel ;

// Definitions for TC channels
// Timer Enable (1 bit: 0=enabled, 1=disabled) | Timer Number (3 bits: 0-7) | Timer Type (1 bit: 0=TCC, 1=TC) | Timer Channel (3 bits: 0-7)
typedef enum _ETCChannel
{
  TCC0_CH0 = (0<<7)|(0<<4)|(0<<3)|(0),
  TCC0_CH1 = (0<<7)|(0<<4)|(0<<3)|(1),
  TCC0_CH2 = (0<<7)|(0<<4)|(0<<3)|(2),
  TCC0_CH3 = (0<<7)|(0<<4)|(0<<3)|(3),
  TCC0_CH4 = (0<<7)|(0<<4)|(0<<3)|(0), // Channel 4 is 0!
  TCC0_CH5 = (0<<7)|(0<<4)|(0<<3)|(1), // Channel 5 is 1!
  TCC0_CH6 = (0<<7)|(0<<4)|(0<<3)|(2), // Channel 6 is 2!
  TCC0_CH7 = (0<<7)|(0<<4)|(0<<3)|(3), // Channel 7 is 3!
  TCC1_CH0 = (0<<7)|(1<<4)|(0<<3)|(0),
  TCC1_CH1 = (0<<7)|(1<<4)|(0<<3)|(1),
  TCC1_CH2 = (0<<7)|(1<<4)|(0<<3)|(0), // Channel 2 is 0!
  TCC1_CH3 = (0<<7)|(1<<4)|(0<<3)|(1), // Channel 3 is 1!
  TCC2_CH0 = (0<<7)|(2<<4)|(0<<3)|(0),
  TCC2_CH1 = (0<<7)|(2<<4)|(0<<3)|(1),
  TCC2_CH2 = (0<<7)|(2<<4)|(0<<3)|(0), // Channel 2 is 0!
  TCC2_CH3 = (0<<7)|(2<<4)|(0<<3)|(1), // Channel 3 is 1!
  TC0_CH0  = (0<<7)|(0<<4)|(1<<3)|(0),
  TC0_CH1  = (0<<7)|(0<<4)|(1<<3)|(1),
  TC1_CH0  = (0<<7)|(1<<4)|(1<<3)|(0),
  TC1_CH1  = (0<<7)|(1<<4)|(1<<3)|(1),
  TC2_CH0  = (0<<7)|(2<<4)|(1<<3)|(0),
  TC2_CH1  = (0<<7)|(2<<4)|(1<<3)|(1),
  TC3_CH0  = (0<<7)|(3<<4)|(1<<3)|(0),
  TC3_CH1  = (0<<7)|(3<<4)|(1<<3)|(1),
  TC4_CH0  = (0<<7)|(4<<4)|(1<<3)|(0),
  TC4_CH1  = (0<<7)|(4<<4)|(1<<3)|(1),
  TC5_CH0  = (0<<7)|(5<<4)|(1<<3)|(0),
  TC5_CH1  = (0<<7)|(5<<4)|(1<<3)|(1),
  TC6_CH0  = (0<<7)|(6<<4)|(1<<3)|(0),
  TC6_CH1  = (0<<7)|(6<<4)|(1<<3)|(1),
  TC7_CH0  = (0<<7)|(7<<4)|(1<<3)|(0),
  TC7_CH1  = (0<<7)|(7<<4)|(1<<3)|(1),
  NOT_ON_TIMER = (1<<7),
} ETCChannel ;

extern const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM] ;

#define GetTCNumber( x ) ( ((x) >> 4) & 0x07 )
#define GetTCType( x ) ( ((x) >> 3) & 0x01 )
#define GetTCChannelNumber( x ) ( (x) & 0x07 )
#if (SAMD21_SERIES)
#define GetTC( x ) ( g_apTCInstances[GetTCNumber(x)] )
#elif (SAML21B_SERIES)
#define GetTC( x ) ( GetTCType(x) == 0 ? g_apTCInstances[GetTCNumber(x)] : (GetTCNumber(x) == 4 ? TC4 : g_apTCInstances[GetTCNumber(x) + TCC_INST_NUM]) )
#else
#error "WVariant.h: Unsupported chip"
#endif

// Definitions for PWM channels
typedef enum _EPWMChannel
{
#if (SAMD21_SERIES)
  PWM0_CH0=TCC0_CH0,
  PWM0_CH1=TCC0_CH1,
  PWM0_CH2=TCC0_CH2,
  PWM0_CH3=TCC0_CH3,
  PWM0_CH4=TCC0_CH4,
  PWM0_CH5=TCC0_CH5,
  PWM0_CH6=TCC0_CH6,
  PWM0_CH7=TCC0_CH7,
  PWM1_CH0=TCC1_CH0,
  PWM1_CH1=TCC1_CH1,
  PWM1_CH2=TCC1_CH2,
  PWM1_CH3=TCC1_CH3,
  PWM2_CH0=TCC2_CH0,
  PWM2_CH1=TCC2_CH1,
  PWM2_CH2=TCC2_CH2,
  PWM2_CH3=TCC2_CH3,
  PWM3_CH0=TC3_CH0,
  PWM3_CH1=TC3_CH1,
  PWM4_CH0=TC4_CH0,
  PWM4_CH1=TC4_CH1,
  PWM5_CH0=TC5_CH0,
  PWM5_CH1=TC5_CH1,
#if defined __SAMD21J18A__
  PWM6_CH0=TC6_CH0,
  PWM6_CH1=TC6_CH1,
  PWM7_CH0=TC7_CH0,
  PWM7_CH1=TC7_CH1,
#endif // __SAMD21J18A__
#elif (SAML21B_SERIES)
  PWM0_CH0=TCC0_CH0,
  PWM0_CH1=TCC0_CH1,
  PWM0_CH2=TCC0_CH2,
  PWM0_CH3=TCC0_CH3,
  PWM0_CH4=TCC0_CH4,
  PWM0_CH5=TCC0_CH5,
  PWM0_CH6=TCC0_CH6,
  PWM0_CH7=TCC0_CH7,
  PWM1_CH0=TCC1_CH0,
  PWM1_CH1=TCC1_CH1,
  PWM1_CH2=TCC1_CH2,
  PWM1_CH3=TCC1_CH3,
  PWM2_CH0=TCC2_CH0,
  PWM2_CH1=TCC2_CH1,
  PWM3_CH0=TC0_CH0,
  PWM3_CH1=TC0_CH1,
  PWM4_CH0=TC1_CH0,
  PWM4_CH1=TC1_CH1,
  PWM5_CH0=TC2_CH0,
  PWM5_CH1=TC2_CH1,
  PWM6_CH0=TC3_CH0,
  PWM6_CH1=TC3_CH1,
  PWM7_CH0=TC4_CH0,
  PWM7_CH1=TC4_CH1,
#endif
  NOT_ON_PWM=NOT_ON_TIMER
} EPWMChannel ;

typedef enum _EPortType
{
  NOT_A_PORT=-1,
  PORTA=0,
  PORTB=1,
  PORTC=2,
} EPortType ;

typedef enum
{
  EXTERNAL_INT_0 = 0,
  EXTERNAL_INT_1,
  EXTERNAL_INT_2,
  EXTERNAL_INT_3,
  EXTERNAL_INT_4,
  EXTERNAL_INT_5,
  EXTERNAL_INT_6,
  EXTERNAL_INT_7,
  EXTERNAL_INT_8,
  EXTERNAL_INT_9,
  EXTERNAL_INT_10,
  EXTERNAL_INT_11,
  EXTERNAL_INT_12,
  EXTERNAL_INT_13,
  EXTERNAL_INT_14,
  EXTERNAL_INT_15,
  EXTERNAL_INT_NMI,
  EXTERNAL_NUM_INTERRUPTS,
  NOT_AN_INTERRUPT = -1,
  EXTERNAL_INT_NONE = NOT_AN_INTERRUPT,
} EExt_Interrupts ;

//A    B                 C       D          E      F   G   H
//EIC REF ADC AC PTC DAC SERCOM SERCOM_ALT TC/TCC TCC COM AC/GCLK

typedef enum _EPioType
{
  PIO_NOT_A_PIN=-1,     /* Not under control of a peripheral. */
  PIO_EXTINT=0,         /* The pin is controlled by the associated signal of peripheral A. */
  PIO_ANALOG,           /* The pin is controlled by the associated signal of peripheral B. */
  PIO_SERCOM,           /* The pin is controlled by the associated signal of peripheral C. */
  PIO_SERCOM_ALT,       /* The pin is controlled by the associated signal of peripheral D. */
  PIO_TIMER,            /* The pin is controlled by the associated signal of peripheral E. */
  PIO_TIMER_ALT,        /* The pin is controlled by the associated signal of peripheral F. */
  PIO_COM,              /* The pin is controlled by the associated signal of peripheral G. */
  PIO_AC_CLK,           /* The pin is controlled by the associated signal of peripheral H. */
  PIO_DIGITAL,          /* The pin is controlled by PORT. */
  PIO_INPUT,            /* The pin is controlled by PORT and is an input. */
  PIO_INPUT_PULLUP,     /* The pin is controlled by PORT and is an input with internal pull-up resistor enabled. */
  PIO_OUTPUT,           /* The pin is controlled by PORT and is an output. */

  PIO_PWM=PIO_TIMER,
  PIO_PWM_ALT=PIO_TIMER_ALT,
} EPioType ;

/**
 * Pin Attributes to be OR-ed
 */
#define PIN_ATTR_NONE          (0UL<<0)
#define PIN_ATTR_COMBO         (1UL<<0)
#define PIN_ATTR_ANALOG        (1UL<<1)
#define PIN_ATTR_DIGITAL       (1UL<<2)
#define PIN_ATTR_PWM           (1UL<<3)
#define PIN_ATTR_TIMER         (1UL<<4)
#define PIN_ATTR_TIMER_ALT     (1UL<<5)
#define PIN_ATTR_EXTINT        (1UL<<6)

/* Types used for the table below */
typedef struct _PinDescription
{
  EPortType       ulPort ;
  uint32_t        ulPin ;
  EPioType        ulPinType ;
  uint32_t        ulPinAttribute ;
  EAnalogChannel  ulADCChannelNumber ; /* ADC Channel number in the SAM device */
  EPWMChannel     ulPWMChannel ;
  ETCChannel      ulTCChannel ;
  EExt_Interrupts ulExtInt ;
} PinDescription ;

/* Pins table to be instantiated into variant.cpp */
extern const PinDescription g_APinDescription[] ;

/* Generic Clock Multiplexer IDs */
#if (SAMD21_SERIES)
#define GCM_DFLL48M_REF           (0x00U)
#define GCM_FDPLL96M_INPUT        (0x01U)
#define GCM_FDPLL96M_32K          (0x02U)
#define GCM_WDT                   (0x03U)
#define GCM_RTC                   (0x04U)
#define GCM_EIC                   (0x05U)
#define GCM_USB                   (0x06U)
#define GCM_EVSYS_CHANNEL_0       (0x07U)
#define GCM_EVSYS_CHANNEL_1       (0x08U)
#define GCM_EVSYS_CHANNEL_2       (0x09U)
#define GCM_EVSYS_CHANNEL_3       (0x0AU)
#define GCM_EVSYS_CHANNEL_4       (0x0BU)
#define GCM_EVSYS_CHANNEL_5       (0x0CU)
#define GCM_EVSYS_CHANNEL_6       (0x0DU)
#define GCM_EVSYS_CHANNEL_7       (0x0EU)
#define GCM_EVSYS_CHANNEL_8       (0x0FU)
#define GCM_EVSYS_CHANNEL_9       (0x10U)
#define GCM_EVSYS_CHANNEL_10      (0x11U)
#define GCM_EVSYS_CHANNEL_11      (0x12U)
#define GCM_SERCOMx_SLOW          (0x13U)
#define GCM_SERCOM0_CORE          (0x14U)
#define GCM_SERCOM1_CORE          (0x15U)
#define GCM_SERCOM2_CORE          (0x16U)
#define GCM_SERCOM3_CORE          (0x17U)
#define GCM_SERCOM4_CORE          (0x18U)
#define GCM_SERCOM5_CORE          (0x19U)
#define GCM_TCC0_TCC1             (0x1AU)
#define GCM_TCC2_TC3              (0x1BU)
#define GCM_TC4_TC5               (0x1CU)
#define GCM_TC6_TC7               (0x1DU)
#define GCM_ADC                   (0x1EU)
#define GCM_AC_DIG                (0x1FU)
#define GCM_AC_ANA                (0x20U)
#define GCM_DAC                   (0x21U)
#define GCM_PTC                   (0x22U)
#define GCM_I2S_0                 (0x23U)
#define GCM_I2S_1                 (0x24U)
#elif (SAML21B_SERIES)
#define GCM_DFLL48M_REF           (0x00U)
#define GCM_FDPLL96M_INPUT        (0x01U)
#define GCM_FDPLL96M_32K          (0x02U)
#define GCM_EIC                   (0x03U)
#define GCM_USB                   (0x04U)
#define GCM_EVSYS_CHANNEL_0       (0x05U)
#define GCM_EVSYS_CHANNEL_1       (0x06U)
#define GCM_EVSYS_CHANNEL_2       (0x07U)
#define GCM_EVSYS_CHANNEL_3       (0x08U)
#define GCM_EVSYS_CHANNEL_4       (0x09U)
#define GCM_EVSYS_CHANNEL_5       (0x0AU)
#define GCM_EVSYS_CHANNEL_6       (0x0BU)
#define GCM_EVSYS_CHANNEL_7       (0x0CU)
#define GCM_EVSYS_CHANNEL_8       (0x0DU)
#define GCM_EVSYS_CHANNEL_9       (0x0EU)
#define GCM_EVSYS_CHANNEL_10      (0x0FU)
#define GCM_EVSYS_CHANNEL_11      (0x10U)
#define GCM_SERCOMx_SLOW          (0x11U)
#define GCM_SERCOM0_CORE          (0x12U)
#define GCM_SERCOM1_CORE          (0x13U)
#define GCM_SERCOM2_CORE          (0x14U)
#define GCM_SERCOM3_CORE          (0x15U)
#define GCM_SERCOM4_CORE          (0x16U)
#define GCM_SERCOM5_SLOW          (0x17U)
#define GCM_SERCOM5_CORE          (0x18U)
#define GCM_TCC0_TCC1             (0x19U)
#define GCM_TCC2                  (0x1AU)
#define GCM_TC0_TC1               (0x1BU)
#define GCM_TC2_TC3               (0x1CU)
#define GCM_TC4                   (0x1DU)
#define GCM_ADC                   (0x1EU)
#define GCM_AC                    (0x1FU)
#define GCM_DAC                   (0x20U)
#define GCM_PTC                   (0x21U)
#define GCM_CCL                   (0x22U)
#endif

#ifdef __cplusplus
} // extern "C"
#endif
