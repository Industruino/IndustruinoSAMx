/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
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

#ifndef _VARIANT_INDUSTRUINO_420MAKER_
#define _VARIANT_INDUSTRUINO_420MAKER_

// The definitions here needs a core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

//----------------------------------------------------------------------------
//        Definitions
//----------------------------------------------------------------------------

// Frequency of the board main oscillator
#define VARIANT_MAINOSC (32768ul)

// Master clock frequency
#define VARIANT_MCK (48000000ul)

//----------------------------------------------------------------------------
//        Headers
//----------------------------------------------------------------------------

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

//----------------------------------------------------------------------------
//        Pins
//----------------------------------------------------------------------------

// Number of pins defined in PinDescription array
#define PINS_COUNT         (14u)
#define NUM_DIGITAL_PINS   (7u)
#define NUM_ANALOG_INPUTS  (7u)
#define NUM_ANALOG_OUTPUTS (2u)
#define analogInputToDigitalPin(p)  ((p < 6u) ? (p) + 14u : -1)

#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->OUT.reg) )
#define portInputRegister(port)    ( &(port->IN.reg) )
#define portModeRegister(port)     ( &(port->DIR.reg) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAMD
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// LED
#define PIN_LED_0   (0u)
#define PIN_LED     PIN_LED_0
#define LED_BUILTIN PIN_LED_0

//
// Analog pins
//
#define PIN_A0         (7ul)
#define PIN_A1         (1ul)
#define PIN_A2         (2ul)
#define PIN_A3         (3ul)
#define PIN_A4         (4ul)
#define PIN_A5         (5ul)
#define PIN_A6         (6ul)
#define PIN_DAC0       (11ul)
#define PIN_DAC0_ADCIN PIN_DAC0
#define PIN_DAC1       (12ul)
#define PIN_DAC1_ADCIN PIN_A2

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define PIN_ANALOG_SPARSE

#define ADC_RESOLUTION 12

//
// Serial interfaces
//

#if defined(SERCOM_CONFIG_2)

#define PIN_SERIAL_RX (6ul)
#define PIN_SERIAL_TX (5ul)
#define PAD_SERIAL_TX (UART_TX_RTS_CTS_PAD_0_2_3)
#define PAD_SERIAL_RX (SERCOM_RX_PAD_1)

#elif defined(SERCOM_CONFIG_3)

#define PIN_SERIAL_RX (2ul)
#define PIN_SERIAL_TX (1ul)
#define PAD_SERIAL_TX (UART_TX_RTS_CTS_PAD_0_2_3)
#define PAD_SERIAL_RX (SERCOM_RX_PAD_1)

#endif

//
// SPI Interfaces
//

#if (defined(SERCOM_CONFIG_1) || defined(SERCOM_CONFIG_2))

#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO (1u)
#define PIN_SPI_MOSI (3u)
#define PIN_SPI_SCK  (4u)
#define PIN_SPI_SS   (2u)
#define PERIPH_SPI   sercom0
#define PAD_SPI_TX   SPI_PAD_2_SCK_3
#define PAD_SPI_RX   SERCOM_RX_PAD_0

static const uint8_t SS	  = PIN_SPI_SS ;
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

#else

#define SPI_INTERFACES_COUNT 0

#endif

//
// Wire Interfaces
//

#if (defined(SERCOM_CONFIG_1) || defined(SERCOM_CONFIG_3))

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA    (5u)
#define PIN_WIRE_SCL    (6u)
#define PERIPH_WIRE     sercom2
#define WIRE_IT_HANDLER SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#else

#define WIRE_INTERFACES_COUNT 0

#endif

//
// USB
//

#define PIN_USB_HOST_ENABLE (8ul)
#define PIN_USB_DM          (9ul)
#define PIN_USB_DP          (10ul)

//
// External 3.3V rail enable
//

#define PIN_EXT_3V3_ENABLE (13ul)

#ifdef __cplusplus
}
#endif

//----------------------------------------------------------------------------
//        Arduino objects - C++ only
//----------------------------------------------------------------------------

#ifdef __cplusplus

//	=========================
//	===== SERCOM DEFINITION
//	=========================

extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

#if (defined(SERCOM_CONFIG_2) || defined(SERCOM_CONFIG_3))
extern Uart Serial;
#endif

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
#define SERIAL_PORT_MONITOR         SerialUSB

#if (defined(SERCOM_CONFIG_2) || defined(SERCOM_CONFIG_3))
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial
#endif

#endif   // _VARIANT_INDUSTRUINO_420MAKER_
