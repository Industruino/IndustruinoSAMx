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

#include <Arduino.h>
#include "variant.h"

//
// Pins descriptions
//
const PinDescription g_APinDescription[] =
{
   // 0 - Digital pin, LED
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 0          |                      |  PA27  |                     | LED
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 },   // LED

#if defined(SERCOM_CONFIG_1)   // SPI (SERCOM0) + I2C (SERCOM2)

   // 1..6 - Digital, analog, communication pins
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 1          | D1/A1                |  PA04  | PIN_SPI_MISO        | Screw terminal pin 1
   // | 2          | D2/A2                |  PA05  | PIN_SPI_SS          | Screw terminal pin 2
   // | 3          | D3/A3                |  PA06  | PIN_SPI_MOSI        | Screw terminal pin 3
   // | 4          | D4/A4                |  PA07  | PIN_SPI_SCK         | Screw terminal pin 4
   // | 5          | D5/A5                |  PA08  | PIN_WIRE_SDA        | Screw terminal pin 5
   // | 6          | D6/A6                |  PA09  | PIN_WIRE_SCL        | Screw terminal pin 6
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 4, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel4,  PWM0_CH0, TCC0_CH0, EXTERNAL_INT_4   },   // SERCOM0/PAD[0] - MISO
   { PORTA, 5, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel5,  PWM0_CH1, TCC0_CH1, EXTERNAL_INT_5   },   // SERCOM0/PAD[1] - SS
   { PORTA, 6, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel6,  PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6   },   // SERCOM0/PAD[2] - MOSI
   { PORTA, 7, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel7,  PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7   },   // SERCOM0/PAD[3] - SCK
   { PORTA, 8, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel16, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_NMI },   // SERCOM2/PAD[0] - SDA
   { PORTA, 9, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel17, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_9   },   // SERCOM2/PAD[1] - SCL

#elif defined(SERCOM_CONFIG_2)   // SPI (SERCOM0) + UART (SERCOM2)

   // 1..6 - Digital, analog, communication pins
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 1          | D1/A1                |  PA04  | PIN_SPI_MISO        | Screw terminal pin 1
   // | 2          | D2/A2                |  PA05  | PIN_SPI_SS          | Screw terminal pin 2
   // | 3          | D3/A3                |  PA06  | PIN_SPI_MOSI        | Screw terminal pin 3
   // | 4          | D4/A4                |  PA07  | PIN_SPI_SCK         | Screw terminal pin 4
   // | 5          | D5/A5                |  PA08  | PIN_SERIAL_TX       | Screw terminal pin 5
   // | 6          | D6/A6                |  PA09  | PIN_SERIAL_RX       | Screw terminal pin 6
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 4, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel4,  PWM0_CH0, TCC0_CH0, EXTERNAL_INT_4   },   // SERCOM0/PAD[0] - MISO
   { PORTA, 5, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel5,  PWM0_CH1, TCC0_CH1, EXTERNAL_INT_5   },   // SERCOM0/PAD[1] - SS
   { PORTA, 6, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel6,  PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6   },   // SERCOM0/PAD[2] - MOSI
   { PORTA, 7, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel7,  PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7   },   // SERCOM0/PAD[3] - SCK
   { PORTA, 8, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel16, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_NMI },   // SERCOM2/PAD[0] - TX
   { PORTA, 9, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel17, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_9   },   // SERCOM2/PAD[1] - RX

#elif defined(SERCOM_CONFIG_3)   // UART (SERCOM0) + I2C (SERCOM2)

   // 1..6 - Digital, analog, communication pins
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 1          | D1/A1                |  PA04  | PIN_SERIAL_TX       | Screw terminal pin 1
   // | 2          | D2/A2                |  PA05  | PIN_SERIAL_RX       | Screw terminal pin 2
   // | 3          | D3/A3                |  PA06  |                     | Screw terminal pin 3
   // | 4          | D4/A4                |  PA07  |                     | Screw terminal pin 4
   // | 5          | D5/A5                |  PA08  | PIN_WIRE_SDA        | Screw terminal pin 5
   // | 6          | D6/A6                |  PA09  | PIN_WIRE_SCL        | Screw terminal pin 6
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 4, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel4,  PWM0_CH0, TCC0_CH0, EXTERNAL_INT_4   },   // SERCOM0/PAD[0] - TX
   { PORTA, 5, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel5,  PWM0_CH1, TCC0_CH1, EXTERNAL_INT_5   },   // SERCOM0/PAD[1] - RX
   { PORTA, 6, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel6,  PWM1_CH0, TCC1_CH0, EXTERNAL_INT_6   },   // SERCOM0/PAD[2] - RTS
   { PORTA, 7, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER),     ADC_Channel7,  PWM1_CH1, TCC1_CH1, EXTERNAL_INT_7   },   // SERCOM0/PAD[3] - CTS
   { PORTA, 8, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel16, PWM0_CH2, TCC0_CH2, EXTERNAL_INT_NMI },   // SERCOM2/PAD[0] - SDA
   { PORTA, 9, PIO_SERCOM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_ANALOG|PIN_ATTR_PWM|PIN_ATTR_TIMER_ALT), ADC_Channel17, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_9   },   // SERCOM2/PAD[1] - SCL

#endif

   // 7 - Internal VREF input
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 7          |                      |  PA03  | Ref_2.5V            | External voltage reference input A
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // VREFA

   // 8..10 - USB
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 8          |                      |  PA14  | PIN_USB_HOST_ENABLE | Not used
   // | 9          |                      |  PA24  | PIN_USB_DM          | USB/D-
   // | 10         |                      |  PA25  | PIN_USB_DP          | USB/D+
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 14, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // USB Host enable (not used)
   { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // USB/DM
   { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },   // USB/DP

   // 11,12 - DAC outputs
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 11         |                      |  PA02  | PIN_DAC0            | DAC0 output
   // | 12         | D2/A2                |  PA05  | PIN_DAC1            | DAC1 output
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },   // DAC/VOUT[0]
   { PORTA, 5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },   // DAC/VOUT[1]

   // 13 - External 3.3V rail enable
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | Pin number | 4-20mA.ker Board pin |  PIN   | Label/Name          | Description
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   // | 13         |                      |  PA15  | PIN_EXT_3V3_ENABLE  | External 3.3V rail enable
   // +------------+----------------------+--------+---------------------+------------------------------------------------------------------------------------------------------------------------------------------------------------
   { PORTA, 15, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }
};

const void* g_apTCInstances[TCC_INST_NUM + TC_INST_NUM] = { TCC0, TCC1, TCC2, TC0, TC1, TC4 };

// Multi-serial objects instantiation
SERCOM sercom0(SERCOM0);
SERCOM sercom1(SERCOM1);
SERCOM sercom2(SERCOM2);
SERCOM sercom3(SERCOM3);
SERCOM sercom4(SERCOM4);
SERCOM sercom5(SERCOM5);

#if defined(SERCOM_CONFIG_2)

Uart Serial(&sercom2, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX);

void SERCOM2_Handler()
{
  Serial.IrqHandler();
}

#elif defined(SERCOM_CONFIG_3)

Uart Serial(&sercom0, PIN_SERIAL_RX, PIN_SERIAL_TX, PAD_SERIAL_RX, PAD_SERIAL_TX);

void SERCOM0_Handler()
{
  Serial.IrqHandler();
}

#endif

void initVariant()
{
#if EXT_3V3_ON
   digitalWrite(PIN_EXT_3V3_ENABLE, HIGH);
#else
   digitalWrite(PIN_EXT_3V3_ENABLE, LOW);
#endif

   pinMode(PIN_EXT_3V3_ENABLE, OUTPUT);
}
