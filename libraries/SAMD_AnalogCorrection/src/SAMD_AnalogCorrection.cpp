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

#include "SAMD_AnalogCorrection.h"

void analogReadCorrection (int offset, uint16_t gain)
{
  // Set correction values
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset);
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain);

  // Enable digital correction logic
#if (SAMD21_SERIES)
  ADC->CTRLB.bit.CORREN = 1;
  while(ADC->STATUS.bit.SYNCBUSY);
#elif (SAML21B_SERIES)
  ADC->CTRLC.bit.CORREN = 1;
  while(ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK);
#else
  #error "SAMD_AnalogCorrection.h: Unsupported chip"
#endif
}
