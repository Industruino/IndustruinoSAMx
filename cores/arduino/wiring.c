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

#include "Arduino.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * System Core Clock is at 1MHz (8MHz/8) at Reset.
 * It is switched to 48MHz in the Reset Handler (startup.c)
 */
uint32_t SystemCoreClock=1000000ul ;

/*
void calibrateADC()
{
  volatile uint32_t valeur = 0;

  for(int i = 0; i < 5; ++i)
  {
    ADC->SWTRIG.bit.START = 1;
    while( ADC->INTFLAG.bit.RESRDY == 0 || ADC->STATUS.bit.SYNCBUSY == 1 )
    {
      // Waiting for a complete conversion and complete synchronization
    }

    valeur += ADC->RESULT.bit.RESULT;
  }

  valeur = valeur/5;
}*/

/*
 * Arduino Zero board initialization
 *
 * Good to know:
 *   - At reset, ResetHandler did the system clock configuration. Core is running at 48MHz.
 *   - Watchdog is disabled by default, unless someone plays with NVM User page
 *   - During reset, all PORT lines are configured as inputs with input buffers, output buffers and pull disabled.
 */
void init( void )
{
  // Set Systick to 1ms interval, common to all Cortex-M variants
  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }
  NVIC_SetPriority (SysTick_IRQn,  (1 << __NVIC_PRIO_BITS) - 2);  /* set Priority for Systick Interrupt (2nd lowest) */

  // Clock PORT for Digital I/O
//  PM->APBBMASK.reg |= PM_APBBMASK_PORT ;
//
//  // Clock EIC for I/O interrupts
//  PM->APBAMASK.reg |= PM_APBAMASK_EIC ;

  // Clock SERCOM for Serial
#if (SAMD21_SERIES)
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;
#elif (SAML21B_SERIES)
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0 | MCLK_APBCMASK_SERCOM1 | MCLK_APBCMASK_SERCOM2 | MCLK_APBCMASK_SERCOM3 | MCLK_APBCMASK_SERCOM4 ;
#endif

  // Clock TC/TCC for Pulse and Analog
#if (SAMD21_SERIES)
#if defined __SAMD21J18A__
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7 ;
#else
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 ;
#endif
#elif (SAML21B_SERIES)
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5 | MCLK_APBDMASK_TC4;   // SERCOM5 and TC4 are on the low power bridge
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC0 | MCLK_APBCMASK_TCC1 | MCLK_APBCMASK_TCC2 | MCLK_APBCMASK_TC0 | MCLK_APBCMASK_TC1 | MCLK_APBCMASK_TC2 ;
#endif

  // Clock ADC/DAC for Analog
#if (SAMD21_SERIES)
  PM->APBCMASK.reg |= PM_APBCMASK_ADC | PM_APBCMASK_DAC ;
#elif (SAML21B_SERIES)
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_ADC;   // ADC is on the low power bridge
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_DAC;
#else
  #error "wiring.c: Unsupported chip"
#endif

  // Setup all pins (digital and analog) in INPUT mode (default is nothing)
  for (uint32_t ul = 0 ; ul < NUM_DIGITAL_PINS ; ul++ )
  {
    pinMode( ul, INPUT ) ;
  }

  // Initialize Analog Controller
  // Setting clock
#if (SAMD21_SERIES)
  while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_ADC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 |    // Divide Clock by 512.
                   ADC_CTRLB_RESSEL_10BIT;         // 10 bits resolution as default

  ADC->SAMPCTRL.reg = 0x3f;                        // Set max Sampling Time Length

#elif (SAML21B_SERIES)
  SUPC->VREF.reg |= SUPC_VREF_VREFOE;           // Enable Supply Controller Reference output for use with ADC and DAC (AR_INTREF)

  GCLK->PCHCTRL[GCM_ADC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
  while ( (GCLK->PCHCTRL[GCM_ADC].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV256;                    // Divide Clock by 256.
  ADC->CTRLC.reg = ADC_CTRLC_RESSEL_10BIT;                        // 10 bits resolution as default
#endif

#if (SAMD21_SERIES)
  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains
#elif (SAML21B_SERIES)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );   // Wait for synchronization of registers between the clock domains
#endif

  ADC->SAMPCTRL.reg = 0x3f;                        // Set max Sampling Time Length

  ADC->INPUTCTRL.reg = ADC_INPUTCTRL_MUXNEG_GND;   // No Negative input (Internal Ground)

#if (SAMD21_SERIES)
  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains
#elif (SAML21B_SERIES)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );   // Wait for synchronization of registers between the clock domains
#endif

  // Averaging (see datasheet table in AVGCTRL register description)
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |    // 1 sample only (no oversampling nor averaging)
                     ADC_AVGCTRL_ADJRES(0x0ul);   // Adjusting result by 0

#if (SAMD21_SERIES)
  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains
#elif (SAML21B_SERIES)
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );   // Wait for synchronization of registers between the clock domains
#endif

  analogReference( AR_DEFAULT ) ; // Analog Reference is AREF pin (3.3v)

  // Initialize DAC
#if (SAMD21_SERIES)
  // Setting clock
  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY );
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCM_DAC ) | // Generic Clock ADC
                      GCLK_CLKCTRL_GEN_GCLK0     | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( DAC->STATUS.bit.SYNCBUSY == 1 ); // Wait for synchronization of registers between the clock domains
  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_AVCC | // Using the 3.3V reference
                   DAC_CTRLB_EOEN ;        // External Output Enable (Vout)
#elif (SAML21B_SERIES)
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

  GCLK->PCHCTRL[GCM_DAC].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );

  while ( (GCLK->PCHCTRL[GCM_DAC].reg & GCLK_PCHCTRL_CHEN) == 0 );      // wait for sync
  while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );

  DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VDDANA;
  DAC->DACCTRL[0].reg = (DAC_DACCTRL_REFRESH(3) | DAC_DACCTRL_CCTRL(2));
  DAC->DACCTRL[1].reg = (DAC_DACCTRL_REFRESH(3) | DAC_DACCTRL_CCTRL(2));
#endif
}

#ifdef __cplusplus
}
#endif
