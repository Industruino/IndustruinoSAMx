/*
  Copyright (c) 2014 Arduino LLC.  All right reserved.
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

#include "Arduino.h"
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

// Mapping of timer numbers (array index) to generic clock IDs
// GCM_* values are defined in WVariant.h in the core.
const uint8_t timerClockIDs[] =
{
#if (SAMD21_SERIES)
  GCM_TCC0_TCC1,
  GCM_TCC0_TCC1,
  GCM_TCC2_TC3,
  GCM_TCC2_TC3,
  GCM_TC4_TC5,
  GCM_TC4_TC5,
  GCM_TC6_TC7,
  GCM_TC6_TC7
#elif (SAML21B_SERIES)
  GCM_TCC0_TCC1,
  GCM_TCC0_TCC1,
  GCM_TCC2,
  GCM_TC0_TC1,
  GCM_TC0_TC1,
  GCM_TC2_TC3,
  GCM_TC2_TC3,
  GCM_TC4
#else
#error "wiring_analog.c: Unsupported chip"
#endif
};

static int _readResolution = 10;
static int _ADCResolution = 10;
static int _writeResolution = 8;

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
#if (SAMD21_SERIES)
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
#elif (SAML21B_SERIES)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
#if (SAMD21_SERIES)
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
#elif (SAML21B_SERIES)
  while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
#if (SAMD21_SERIES)
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
#elif (SAML21B_SERIES)
  while (TCx->COUNT16.SYNCBUSY.reg & (TC_SYNCBUSY_SWRST | TC_SYNCBUSY_ENABLE | TC_SYNCBUSY_CTRLB | TC_SYNCBUSY_STATUS | TC_SYNCBUSY_COUNT));
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

void analogReadResolution(int res)
{
  _readResolution = res;
  if (res > 10) {
#if (SAMD21_SERIES)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
#elif (SAML21B_SERIES)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#endif
    _ADCResolution = 12;
  } else if (res > 8) {
#if (SAMD21_SERIES)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
#elif (SAML21B_SERIES)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#endif
    _ADCResolution = 10;
  } else {
#if (SAMD21_SERIES)
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
#elif (SAML21B_SERIES)
    ADC->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
#endif
    _ADCResolution = 8;
  }
  syncADC();
}

void analogWriteResolution(int res)
{
  _writeResolution = res;
}

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : The maximum I/O voltage is Vcc
 */
void analogReference(eAnalogReference mode)
{
  syncADC();
  switch (mode)
  {
#if (SAMD21_SERIES)
    case AR_INTERNAL:
    case AR_INTERNAL2V23:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;        // Gain Factor Selection (1X)
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;   // 1/1.48 VDDANA = 1/1.48 * 3V3 = 2.2297
      break;

    case AR_EXTERNAL:
    case AR_EXTERNAL_VREFA:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;        // Gain Factor Selection (1X)
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
      break;

    case AR_EXTERNAL_VREFB:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;        // Gain Factor Selection (1X)
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFB_Val;
      break;

    case AR_INTERNAL1V0:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;        // Gain Factor Selection (1X)
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;     // 1.0V voltage reference
      break;

    case AR_INTERNAL1V65:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;        // Gain Factor Selection (1X)
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;   // 1/2 VDDANA = 0.5 * 3V3 = 1.65V
      break;

    case AR_DEFAULT:
    default:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;      // Gain Factor Selection (0.5X)
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;   // 1/2 VDDANA = 0.5 * 3V3 = 1.65V
      break;
#elif (SAML21B_SERIES)
    case AR_INTERNAL:
    case AR_INTERNAL2V06:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;   // 1/1.6 VDDANA = 0.625 * 3.3V = 2.0625V
      break;

    case AR_EXTERNAL:
    case AR_EXTERNAL_VREFA:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
      break;

    case AR_EXTERNAL_VREFB:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFB_Val;
      break;

    case AR_INTERNAL1V0:
    case AR_INTREF_1V0:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V0_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_1V1:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V1_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_1V2:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V2_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_1V25:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V25_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_2V0:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V0_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_2V2:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V2_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_2V4:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V4_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTREF_2V5:
      SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V5_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTERNAL_REF:
    case AR_INTREF:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;
      break;

    case AR_INTERNAL1V65:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;   // 1/2 VDDANA = 0.5 * 3V3 = 1.65V
      break;

    case AR_DEFAULT:
    default:
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC2_Val;   // VDDANA = 3V3
#endif
  }

#if (SAMD21_SERIES)
  syncADC();
#endif
}

uint32_t analogRead(uint32_t pin)
{
  uint32_t valueRead = 0;

#if !defined(PIN_ANALOG_SPARSE)
  if (pin < A0) {
    pin += A0;
  }
#endif

  pinPeripheral(pin, PIO_ANALOG);

#if (SAMD21_SERIES)
  if (pin == PIN_DAC0_ADCIN) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
    //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
    syncDAC();
  }
#elif (SAML21B_SERIES)
  if ((DAC->DACCTRL[0].bit.ENABLE) && ((pin == PIN_DAC0_ADCIN) || (pin == PIN_DAC0))) { // Disable DAC0, if analogWrite(DAC0,dval) used previously the DAC is enabled
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller (so that DACCTRL can be modified)
    delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
    syncDAC();

    DAC->DACCTRL[0].bit.ENABLE = 0x00; // The DAC0 output is turned off.

    if (DAC->DACCTRL[1].bit.ENABLE) {   // Is DAC1 enabled?
      delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller, so that the DAC1 can function
      syncDAC();
      while ((DAC->STATUS.reg & DAC_STATUS_READY1) == 0);   // Must wait for DAC1 to start
    }
  }
  else if ((DAC->DACCTRL[1].bit.ENABLE) && ((pin == PIN_DAC1_ADCIN) || (pin == PIN_DAC1))) { // Disable DAC1, if analogWrite(DAC1,dval) used previously the DAC is enabled
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller (so that DACCTRL can be modified)
    delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
    syncDAC();

    DAC->DACCTRL[1].bit.ENABLE = 0x00; // The DAC1 output is turned off.

    if (DAC->DACCTRL[0].bit.ENABLE) {   // Is DAC0 enabled?
      delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
      DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller, so that the DAC0 can function
      syncDAC();
      while ((DAC->STATUS.reg & DAC_STATUS_READY0) == 0);   // Must wait for DAC0 to start
    }
  }
#endif

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS(g_APinDescription[pin].ulADCChannelNumber);   // Selection for the positive ADC input

  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();

  return mapResolution(valueRead, _ADCResolution, _readResolution);
}


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t pin, uint32_t value)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
  {
    // DAC handling code
#if (SAMD21_SERIES)
    if ((pin != PIN_DAC0) && (pin != PIN_DAC0_ADCIN)) {
      return;
    }

    value = mapResolution(value, _writeResolution, 10);

    syncDAC();
    DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
    DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
    syncDAC();
#elif (SAML21B_SERIES)
    value = mapResolution(value, _writeResolution, 12);

    if ((pin == PIN_DAC0) || (pin == PIN_DAC0_ADCIN)) {
      if (!DAC->DACCTRL[0].bit.ENABLE) {
        DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC controller (so that DACCTRL can be modified)
        delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC

        DAC->DATA[0].reg = value & 0xFFF;  // DAC0 on 12 bits.
        syncDAC();
        DAC->DACCTRL[0].bit.ENABLE = 0x01; // The DAC0 output is turned on.
        delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
        DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller
        syncDAC();
        while ((DAC->STATUS.reg & DAC_STATUS_READY0) == 0);   // Must wait for DAC0 to start
      }

      DAC->DATA[0].reg = value & 0xFFF;  // DAC0 on 12 bits.
      syncDAC();
    }
    else if ((pin == PIN_DAC1) || (pin == PIN_DAC1_ADCIN)) {
      if (!DAC->DACCTRL[1].bit.ENABLE) {
        DAC->CTRLA.bit.ENABLE = 0x00;   // Disable DAC controller (so that DACCTRL can be modified)
        delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC

        DAC->DATA[1].reg = value & 0xFFF;  // DAC1 on 12 bits.
        syncDAC();
        DAC->DACCTRL[1].bit.ENABLE = 0x01; // The DAC1 output is turned on.
        delayMicroseconds(40);   // DAC errata: after the DAC is disabled in refresh mode, wait for at least 30us before re-enabling the DAC
        DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC controller
        syncDAC();
        while ((DAC->STATUS.reg & DAC_STATUS_READY1) == 0);   // Must wait for DAC1 to start
      }

      DAC->DATA[1].reg = value & 0xFFF;  // DAC1 on 12 bits.
      syncDAC();
    }
#endif
    return;
  }

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  {
    value = mapResolution(value, _writeResolution, 16);

    uint8_t tcType = GetTCType(pinDesc.ulPWMChannel);
    uint8_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
    static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

    if (attr & PIN_ATTR_TIMER) {
      #if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
      // Compatibility for cores based on SAMD core <=1.6.2
      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
        pinPeripheral(pin, PIO_TIMER_ALT);
      } else
      #endif
      {
        pinPeripheral(pin, PIO_TIMER);
      }
    } else {
      // We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
      pinPeripheral(pin, PIO_TIMER_ALT);
    }

#if (SAMD21_SERIES)
    uint8_t tcIndex = tcNum;
#elif (SAML21B_SERIES)
    uint8_t tcIndex = (tcType == 1 ? tcNum + TCC_INST_NUM : tcNum);
#endif

    if (!tcEnabled[tcIndex]) {
      tcEnabled[tcIndex] = true;

#if (SAMD21_SERIES)
      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(timerClockIDs[tcIndex]));
      while (GCLK->STATUS.bit.SYNCBUSY == 1);
#elif (SAML21B_SERIES)
      GCLK->PCHCTRL[timerClockIDs[tcIndex]].reg = (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0);
      while ((GCLK->PCHCTRL[timerClockIDs[tcIndex]].reg & GCLK_PCHCTRL_CHEN) == 0);
#endif

      // Set PORT
      if (tcType == 1) {
        // -- Configure TC
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 0;
        syncTC_16(TCx);
        // Set Timer counter Mode to 16 bits
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
        syncTC_16(TCx);
        // Set TCx as normal PWM
#if (SAMD21_SERIES)
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
#elif (SAML21B_SERIES)
        TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;
#endif
        syncTC_16(TCx);
        // Set the initial value
        TCx->COUNT16.CC[tcChannel].reg = (uint16_t) value;
        syncTC_16(TCx);
        // Enable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 1;
        syncTC_16(TCx);
      } else {
        // -- Configure TCC
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCCx
        TCCx->CTRLA.bit.ENABLE = 0;
        syncTCC(TCCx);
        // Set TCCx as normal PWM
        TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
        syncTCC(TCCx);
        // Set the initial value
        TCCx->CC[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        // Set PER to maximum counter value (resolution : 0xFFFF)
        TCCx->PER.reg = 0xFFFF;
        syncTCC(TCCx);
        // Enable TCCx
        TCCx->CTRLA.bit.ENABLE = 1;
        syncTCC(TCCx);
      }
    } else {
      if (tcType == 1) {
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
#if (SAMD21_SERIES)
        TCx->COUNT16.CC[tcChannel].reg = (uint16_t) value;
#elif (SAML21B_SERIES)
        // SAML has double-buffered TCs
        TCx->COUNT16.CCBUF[tcChannel].reg = (uint16_t) value;
#endif
        syncTC_16(TCx);
      } else {
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
#if (SAMD21_SERIES)
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
#elif (SAML21B_SERIES)
        // LUPD caused endless spinning in syncTCC() on SAML. Note that CCBUF writes are already
        // atomic. The LUPD bit is intended for updating several registers at once, which analogWrite() does not do.
        TCCx->CCBUF[tcChannel].reg = (uint32_t) value;
#endif
        syncTCC(TCCx);
      }
    }
    return;
  }

  // -- Defaults to digital write
  pinMode(pin, OUTPUT);
  value = mapResolution(value, _writeResolution, 8);
  if (value < 128) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

#ifdef __cplusplus
}
#endif
