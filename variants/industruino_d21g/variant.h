/*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (C) 2017 Industruino <connect@industruino.com>  All right reserved.

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

#ifndef _VARIANT_INDUSTRUINO_D21G_
#define _VARIANT_INDUSTRUINO_D21G_

// The definitions here need a SAMD core >=1.6.10
#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC (32768ul)

/** Master clock frequency */
#define VARIANT_MCK (48000000ul)

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT         (29u)
#define NUM_DIGITAL_PINS   (27u)
#define NUM_ANALOG_INPUTS  (12u)
#define NUM_ANALOG_OUTPUTS (1u)

#define analogInputToDigitalPin(p) ( p )
#define digitalPinToPort(P)        ( &(PORT->Group[g_APinDescription[P].ulPort]) )
#define digitalPinToBitMask(P)     ( 1 << g_APinDescription[P].ulPin )
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
#define PIN_LED_26  (26u)
#define PIN_LED     PIN_LED_26
#define LED_BUILTIN PIN_LED

// Buttons
#define D21G_BUTTON1 (25u)
#define D21G_BUTTON2 (24u)
#define D21G_BUTTON3 (23u)

#define D21G_BUTTON_TOP    D21G_BUTTON1
#define D21G_BUTTON_MIDDLE D21G_BUTTON2
#define D21G_BUTTON_BOTTOM D21G_BUTTON3

#define D21G_BUTTON_UP    D21G_BUTTON1
#define D21G_BUTTON_ENTER D21G_BUTTON2
#define D21G_BUTTON_DOWN  D21G_BUTTON3

//
// Analog pins
//
#define PIN_A4         (4ul)
#define PIN_A5         (5ul)
#define PIN_A6         (6ul)
#define PIN_A7         (7ul)
#define PIN_A8         (8ul)
#define PIN_A9         (9ul)
#define PIN_A10        (10ul)
#define PIN_A11        (11ul)
#define PIN_A12        (12ul)
#define PIN_A13        (13ul)
#define PIN_A18        (18ul)
#define PIN_A27        (27ul)
#define PIN_DAC0       (29ul)
#define PIN_DAC0_ADCIN PIN_A18

static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t A6   = PIN_A6;
static const uint8_t A7   = PIN_A7;
static const uint8_t A8   = PIN_A8;
static const uint8_t A9   = PIN_A9;
static const uint8_t A10  = PIN_A10;
static const uint8_t A11  = PIN_A11;
static const uint8_t A12  = PIN_A12;
static const uint8_t A13  = PIN_A13;
static const uint8_t A18  = PIN_A18;
static const uint8_t A27  = PIN_A27;
static const uint8_t DAC0 = PIN_DAC0;

#define PIN_ANALOG_SPARSE

#define ADC_RESOLUTION 12

//
// Serial interfaces
//

// Serial
#define PIN_SERIAL_RX (0ul)
#define PIN_SERIAL_TX (1ul)
#define PAD_SERIAL_TX (UART_TX_PAD_2)
#define PAD_SERIAL_RX (SERCOM_RX_PAD_3)

// Serial1
#define PIN_SERIAL1_RX (10ul)
#define PIN_SERIAL1_TX (5ul)
#define PAD_SERIAL1_TX (UART_TX_PAD_2)
#define PAD_SERIAL1_RX (SERCOM_RX_PAD_3)

//
// SPI Interfaces
//

#define SPI_INTERFACES_COUNT 2

#define PIN_SPI_MISO (14u)
#define PIN_SPI_MOSI (16u)
#define PIN_SPI_SCK  (15u)
#define PIN_SPI_SS   (10u)
#define PERIPH_SPI   sercom5
#define PAD_SPI_TX   SPI_PAD_3_SCK_1
#define PAD_SPI_RX   SERCOM_RX_PAD_2

static const uint8_t SS   = PIN_SPI_SS;   // HW SS isn't used. Set here only for reference
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define PIN_SPI1_MISO (33u)
#define PIN_SPI1_MOSI (20u)
#define PIN_SPI1_SCK  (21u)
#define PERIPH_SPI1   sercom4
#define PAD_SPI1_TX   SPI_PAD_0_SCK_3
#define PAD_SPI1_RX   SERCOM_RX_PAD_2

//
// Wire Interfaces
//

#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA    (2u)
#define PIN_WIRE_SCL    (3u)
#define PERIPH_WIRE     sercom1
#define WIRE_IT_HANDLER SERCOM1_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

//
// USB
//

#define PIN_USB_HOST_ENABLE (30ul)
#define PIN_USB_DM          (31ul)
#define PIN_USB_DP          (32ul)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

/*	=========================
 *	===== SERCOM DEFINITION
 *	=========================
*/
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;

extern Uart Serial;
extern Uart Serial1;

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
// Serial has no physical pins broken out, so it's not listed as HARDWARE port
#define SERIAL_PORT_HARDWARE        Serial
#define SERIAL_PORT_HARDWARE_OPEN   Serial

#endif   // _VARIANT_INDUSTRUINO_D21G_
