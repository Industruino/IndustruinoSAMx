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

#ifdef __cplusplus
extern "C" {
#endif

/*
 * \brief SAMD products have only one reference for ADC
 */
typedef enum _eAnalogReference
{
#if (SAMD21_SERIES)
  AR_DEFAULT,          // INTVCC1 * 2
  AR_INTERNAL,         // INTVCC0
  AR_EXTERNAL,         // VREFA
  AR_INTERNAL1V0,      // INT1V
  AR_INTERNAL1V65,     // INTVCC1
  AR_INTERNAL2V23,     // INTVCC0
  AR_EXTERNAL_VREFA,   // VREFA
  AR_EXTERNAL_VREFB    // VREFB
#elif (SAML21B_SERIES)
  AR_DEFAULT,          // INTVCC2
  AR_INTERNAL,         // INTVCC0
  AR_EXTERNAL,         // VREFA
  AR_INTERNAL1V0,      // INTREF @1V
  AR_INTERNAL1V65,     // INTVCC1
  AR_INTERNAL2V06,     // INTVCC0
  AR_EXTERNAL_VREFA,   // VREFA
  AR_EXTERNAL_VREFB,   // VREFB
  AR_INTERNAL_REF,     // INTREF
  AR_INTREF,           // INTREF
  AR_INTREF_1V0,       // INTREF @1V
  AR_INTREF_1V1,       // INTREF @1.1V
  AR_INTREF_1V2,       // INTREF @1.2V
  AR_INTREF_1V25,      // INTREF @1.25V
  AR_INTREF_2V0,       // INTREF @2V
  AR_INTREF_2V2,       // INTREF @2.2V
  AR_INTREF_2V4,       // INTREF @2.4V
  AR_INTREF_2V5        // INTREF @2.5V
#else
  #error "wiring_analog.h: Unsupported chip"
#endif
} eAnalogReference ;


/*
 * \brief Configures the reference voltage used for analog input (i.e. the value used as the top of the input range).
 * This function is kept only for compatibility with existing AVR based API.
 *
 * \param ulMmode Should be set to AR_DEFAULT.
 */
extern void analogReference( eAnalogReference ulMode ) ;

/*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
extern void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;

/*
 * \brief Reads the value from the specified analog pin.
 *
 * \param ulPin
 *
 * \return Read value from selected pin, if no error.
 */
extern uint32_t analogRead( uint32_t ulPin ) ;

/*
 * \brief Set the resolution of analogRead return values. Default is 10 bits (range from 0 to 1023).
 *
 * \param res
 */
extern void analogReadResolution(int res);

/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
extern void analogWriteResolution(int res);

extern void analogOutputInit( void ) ;

#ifdef __cplusplus
}
#endif
