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

#include "sam.h"
#include "variant.h"

#include <stdio.h>

////////////////////////////////////////////////////////////////////////////////

#if (SAMD21_SERIES)

/**
 * \brief SystemInit() configures the needed clocks and according Flash Read Wait States.
 * At reset:
 * - OSC8M clock source is enabled with a divider by 8 (1MHz).
 * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 1
 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 * 6) Modify PRESCaler value of OSCM to have 8MHz
 * 7) Put OSC8M as source for Generic Clock Generator 3
 */
// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSC32K    (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

void SystemInit( void )
{
  /* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
  NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val ;

  /* Turn on the digital interface clock */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;


#if defined(CRYSTALLESS)

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable OSC32K clock (Internal 32.768Hz oscillator)
   */

  uint32_t calib = (*((uint32_t *) FUSES_OSC32K_CAL_ADDR) & FUSES_OSC32K_CAL_Msk) >> FUSES_OSC32K_CAL_Pos;

  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(calib) |
                        SYSCTRL_OSC32K_STARTUP( 0x6u ) | // cf table 15.10 of product datasheet in chapter 15.8.6
                        SYSCTRL_OSC32K_EN32K |
                        SYSCTRL_OSC32K_ENABLE;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 ); // Wait for oscillator stabilization

#else // has crystal

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
   */
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | /* cf table 15.10 of product datasheet in chapter 15.8.6 */
                         SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K ;
  SYSCTRL->XOSC32K.bit.ENABLE = 1 ; /* separate call, as described in chapter 15.6.3 */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 )
  {
    /* Wait for oscillator stabilization */
  }

#endif

  /* Software reset the module to ensure it is re-initialized correctly */
  /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
   * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete, as described in chapter 13.8.1
   */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST ;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) )
  {
    /* Wait for reset to complete */
  }

  /* ----------------------------------------------------------------------------------------------
   * 2) Put XOSC32K as source of Generic Clock Generator 1
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_XOSC32K ) ; // Generic Clock Generator 1

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 1 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC32K ) | // Generic Clock Generator 1
#if defined(CRYSTALLESS)
                      GCLK_GENCTRL_SRC_OSC32K | // Selected source is Internal 32KHz Oscillator
#else
                      GCLK_GENCTRL_SRC_XOSC32K | // Selected source is External 32KHz Oscillator
#endif
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
   */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GENERIC_CLOCK_MULTIPLEXER_DFLL48M ) | // Generic Clock Multiplexer 0
                      GCLK_CLKCTRL_GEN_GCLK1 | // Generic Clock Generator 1 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 4) Enable DFLL48M clock
   */

  /* DFLL Configuration in Closed Loop mode, cf product datasheet chapter 15.6.7.1 - Closed-Loop Operation */

  /* Remove the OnDemand mode, Bug http://avr32.icgroup.norway.atmel.com/bugzilla/show_bug.cgi?id=9905 */
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 511 ) | // Fine step is 511, half of the max value
                         SYSCTRL_DFLLMUL_MUL( (VARIANT_MCK + VARIANT_MAINOSC/2) / VARIANT_MAINOSC ) ; // External 32KHz is the reference

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

#if defined(CRYSTALLESS)

  #define NVM_SW_CALIB_DFLL48M_COARSE_VAL 58

  // Turn on DFLL
  uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32) )
                   & ((1 << 6) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  // TODO(tannewt): Load this value from memory we've written previously. There
  // isn't a value from the Atmel factory.
  uint32_t fine = 0x1ff;

  SYSCTRL->DFLLVAL.bit.COARSE = coarse;
  SYSCTRL->DFLLVAL.bit.FINE = fine;
  /* Write full configuration to DFLL control register */
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 0x1f / 4 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 10 ) |
                         SYSCTRL_DFLLMUL_MUL( (48000) ) ;

  SYSCTRL->DFLLCTRL.reg = 0;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  SYSCTRL->DFLLCTRL.reg =  SYSCTRL_DFLLCTRL_MODE |
                           SYSCTRL_DFLLCTRL_CCDIS |
                           SYSCTRL_DFLLCTRL_USBCRM | /* USB correction */
                           SYSCTRL_DFLLCTRL_BPLCKC;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;

#else   // has crystal

  /* Write full configuration to DFLL control register */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | /* Enable the closed loop mode */
                           SYSCTRL_DFLLCTRL_WAITLOCK |
                           SYSCTRL_DFLLCTRL_QLDIS ; /* Disable Quick lock */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
          (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0 )
  {
    /* Wait for locks flags */
  }

#endif

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) ; // Generic Clock Generator 0

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | // Generic Clock Generator 0
                      GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_IDC | // Set 50/50 duty cycle
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 6) Modify PRESCaler value of OSC8M to have 8MHz
   */
  SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;  //CMSIS 4.5 changed the prescaler defines
  SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

  /* ----------------------------------------------------------------------------------------------
   * 7) Put OSC8M as source for Generic Clock Generator 3
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) ; // Generic Clock Generator 3

  /* Write Generic Clock Generator 3 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) | // Generic Clock Generator 3
                      GCLK_GENCTRL_SRC_OSC8M | // Selected source is RC OSC 8MHz (already enabled at reset)
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /*
   * Now that all system clocks are configured, we can set CPU and APBx BUS clocks.
   * There values are normally the one present after Reset.
   */
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;

  SystemCoreClock=VARIANT_MCK ;

  /* ----------------------------------------------------------------------------------------------
   * 8) Load ADC factory calibration values
   */

  // ADC Bias Calibration
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;

  // ADC Linearity bits 4:0
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;

  // ADC Linearity bits 7:5
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  /*
   * 9) Disable automatic NVM write operations
   */
  NVMCTRL->CTRLB.bit.MANW = 1;
}

////////////////////////////////////////////////////////////////////////////////

#elif (SAML21B_SERIES)

//
// SystemInit() configures the needed clocks and according Flash Read Wait States
//

// Constants for Clock generators (the D51 has 12 generators and all others have 9 generators). Unused generators are automatically stopped to reduce power consumption.
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)  /* Used for the CPU/APB clocks. With the D51, it runs at either 96MHz (divided by 2 in MCLK) or 120MHz undivided. Otherwise, it runs at 48MHz. */
#define GENERIC_CLOCK_GENERATOR_XOSC      (1u)  /* The high speed crystal is connected to GCLK1 in order to use the 16-bit prescaler. */
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u)  /* Initialized at reset for WDT (D21 and D11 only). Not used by core. */
#define GENERIC_CLOCK_GENERATOR_OSC_HS    (3u)  /* 8MHz from internal RC oscillator (D21, D11, and L21 only). Setup by core but not used. */
#define GENERIC_CLOCK_GENERATOR_48MHz     (4u)  /* Used for USB or any peripheral that has a 48MHz (60MHz for D51) maximum peripheral clock. GCLK0 is now only 96MHz or 120MHz with the D51. */
#define GENERIC_CLOCK_GENERATOR_TIMERS    (5u)  /* Used by the timers for controlling PWM frequency. Can be up to 48MHz (up to 96MHz with the D51). */
#define GENERIC_CLOCK_GENERATOR_192MHz    (6u)  /* Used only by D51 for any peripheral that has a 200MHz maximum peripheral clock (note that GCLK8 - GCLK11 must be <= 100MHz). */
#define GENERIC_CLOCK_GENERATOR_I2S       (7u)  /* Used by D51 and D21 for I2S peripheral. This define is not currently used. The generator is defined in each variant.h. */
#define GENERIC_CLOCK_GENERATOR_I2S1      (8u)  /* Used by D51 and D21 for I2S peripheral. This define is not currently used. The generator is defined in each variant.h. */
#define GENERIC_CLOCK_GENERATOR_DFLL      (9u)  /* Used only by D51 (only when the cpu is 120MHz) with CLOCKCONFIG_INTERNAL or CLOCKCONFIG_INTERNAL_USB to generate 2MHz output for the PLL input. */
#define GENERIC_CLOCK_GENERATOR_96MHz     (10u) /* Used only by D51 for any peripheral that has a 100MHz maximum peripheral clock. */
#define GENERIC_CLOCK_GENERATOR_UNUSED11  (11u) /* Unused for now. D51 only. */

// Constants when using a GCLK as a source to a PLL
// Make sure thay are consistent with the constants above
#define GCLK_PCHCTRL_GEN_XOSC GCLK_PCHCTRL_GEN_GCLK1
#define GCLK_PCHCTRL_GEN_DFLL GCLK_PCHCTRL_GEN_GCLK9

// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL      (0u)
#define GENERIC_CLOCK_MULTIPLEXER_FDPLL     (1u)
#define GENERIC_CLOCK_MULTIPLEXER_FDPLL_32K (2u)

void waitForSync()
{
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK);
}

void waitForDFLL()
{
  while ((OSCCTRL->STATUS.reg & OSCCTRL_STATUS_DFLLRDY) == 0);
}

void waitForPLL()
{
  while (OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK);
}

void SystemInit()
{
  // Set 2 Flash Wait State for 48MHz (for the L21), cf tables 20.9 and 35.27 in SAMD21 Datasheet
  // Disable automatic NVM write operations (errata reference 13134)
  NVMCTRL->CTRLB.reg = (NVMCTRL_CTRLB_RWS_DUAL | NVMCTRL_CTRLB_MANW);   // Two wait states

  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;   // Clear flag
  PM->PLCFG.reg |= PM_PLCFG_PLSEL_PL2;   // Must set to highest performance level
  while ((PM->INTFLAG.reg & PM_INTFLAG_PLRDY) != PM_INTFLAG_PLRDY);
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;   // Clear flag

#if defined(CRYSTALLESS)
  // ----------------------------------------------------------------------------------------------
  // Enable DFLL48M clock (D21/L21) or RC oscillator (C21)
  //

  // Defines missing from CMSIS
  #ifndef FUSES_DFLL48M_COARSE_CAL_ADDR
    #define FUSES_DFLL48M_COARSE_CAL_ADDR (NVMCTRL_OTP5)
  #endif
  #ifndef FUSES_DFLL48M_COARSE_CAL_Pos
    #define FUSES_DFLL48M_COARSE_CAL_Pos 26
  #endif
  #ifndef FUSES_DFLL48M_COARSE_CAL_Msk
    #define FUSES_DFLL48M_COARSE_CAL_Msk (0x3Ful << FUSES_DFLL48M_COARSE_CAL_Pos)
  #endif

  OSCCTRL->DFLLCTRL.bit.ONDEMAND = 0;
  waitForDFLL();

  // Load NVM Coarse calibration value
  uint32_t calib = (*((uint32_t *) FUSES_DFLL48M_COARSE_CAL_ADDR) & FUSES_DFLL48M_COARSE_CAL_Msk) >> FUSES_DFLL48M_COARSE_CAL_Pos;
  OSCCTRL->DFLLVAL.reg = OSCCTRL_DFLLVAL_COARSE(calib) | OSCCTRL_DFLLVAL_FINE(512);

  // Write full configuration to DFLL control register
  OSCCTRL->DFLLMUL.reg = OSCCTRL_DFLLMUL_CSTEP(31) |    // Coarse step is 31, half of the max value
                         OSCCTRL_DFLLMUL_FSTEP(0xA) |   // Value from datasheet USB Characteristics
                         OSCCTRL_DFLLMUL_MUL(0xBB80);   // 1KHz USB SOF signal (48MHz Fcpu / 1KHz SOF)

  OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_USBCRM |   // USB correction
                          OSCCTRL_DFLLCTRL_MODE |     // Closed loop mode
                          OSCCTRL_DFLLCTRL_CCDIS;
  waitForDFLL();

  // Enable the DFLL
  OSCCTRL->DFLLCTRL.reg |= OSCCTRL_DFLLCTRL_ENABLE;
  waitForDFLL();

#else

  // ----------------------------------------------------------------------------------------------
  // Enable XOSC32K clock (External on-board 32.768Hz crystal oscillator)
  //

  #define DPLLRATIO_LDR     2929u
  #define DPLLRATIO_LDRFRAC 0u

  OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP(0x4u) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN1K);
  OSC32KCTRL->XOSC32K.bit.ENABLE = 1;

  // Wait for oscillator stabilization
  while ((OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0);

  // Set PLL multiplier
  OSCCTRL->DPLLRATIO.reg = (OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC));
  waitForPLL();

  // Select 32KHz crystal input
  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0);

  OSCCTRL->DPLLPRESC.reg = 0;
  waitForPLL();

  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE;
  waitForPLL();

  while ((OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY);

#endif


//
// Setup GCLK0 (GENERIC_CLOCK_GENERATOR_MAIN) which is used for the CPU/APB, as well as other required GCLKs.
//

// Setup GCLK0 (GENERIC_CLOCK_GENERATOR_MAIN) which is used for the CPU
#if defined(CRYSTALLESS)
  // Switch Generic Clock Generator 0 to 48MHz DFLL48M output. The output is undivided
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = (GCLK_GENCTRL_DIV(1) | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN);
  waitForSync();
#else
  // Switch Generic Clock Generator 0 to 96MHz PLL output. The output is divided by two to obtain a 48MHz CPU clock
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = (GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN);
  waitForSync();
#endif

  SystemCoreClock = VARIANT_MCK;

  // ----------------------------------------------------------------------------------------------
  // Load ADC factory calibration values
  //

  uint32_t biasrefbuf = (*((uint32_t *) ADC_FUSES_BIASREFBUF_ADDR) & ADC_FUSES_BIASREFBUF_Msk) >> ADC_FUSES_BIASREFBUF_Pos;
  uint32_t biascomp = (*((uint32_t *) ADC_FUSES_BIASCOMP_ADDR) & ADC_FUSES_BIASCOMP_Msk) >> ADC_FUSES_BIASCOMP_Pos;

  ADC->CALIB.reg = ADC_CALIB_BIASREFBUF(biasrefbuf) | ADC_CALIB_BIASCOMP(biascomp);

  // ----------------------------------------------------------------------------------------------
  // Disable automatic NVM write operations
  //

  NVMCTRL->CTRLB.bit.MANW = 1;
}

#endif
