/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.
  Copyright (c) 2015 Atmel Corporation/Thibaut VIARD.  All right reserved.
  Copyright (C) 2018 Industruino <connect@industruino.com>  All right reserved.

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

// These are undefined in CMSIS for L21
#if (SAML21B_SERIES)
#ifndef GCLK_CLKCTRL_ID_SERCOM0_CORE_Val
  #define GCLK_CLKCTRL_ID_SERCOM0_CORE_Val 18
  #define GCLK_CLKCTRL_ID_SERCOM1_CORE_Val 19
  #define GCLK_CLKCTRL_ID_SERCOM2_CORE_Val 20
  #define GCLK_CLKCTRL_ID_SERCOM3_CORE_Val 21
  #define GCLK_CLKCTRL_ID_SERCOM4_CORE_Val 22
  #define GCLK_CLKCTRL_ID_SERCOM5_CORE_Val 24
#endif
#endif

// These are undefined in CMSIS for D21
#if (SAMD21_SERIES)
#ifndef GCLK_GENCTRL_SRC_DPLL96M
  #define GCLK_GENCTRL_SRC_Pos         8      /**< \brief (GCLK_GENCTRL) Source Select */
  #define GCLK_GENCTRL_SRC_DPLL96M_Val 0x8ul  /**< \brief (GCLK_GENCTRL) DPLL96M output */
  #define GCLK_GENCTRL_SRC_DPLL96M     (GCLK_GENCTRL_SRC_DPLL96M_Val << GCLK_GENCTRL_SRC_Pos)
#endif
#endif

#define CLOCKCONFIG_32768HZ_CRYSTAL 0
#define CLOCKCONFIG_HS_CRYSTAL      1
#define CLOCKCONFIG_INTERNAL        2
#define CLOCKCONFIG_INTERNAL_USB    3

#if defined(BOARD_ID_arduino_zero)
  #include "board_definitions_arduino_zero.h"
#elif defined(BOARD_ID_genuino_zero)
  #include "board_definitions_genuino_zero.h"
#elif defined(BOARD_ID_industruino_420maker)
  #include "board_definitions_industruino_420maker.h"
#else
  #error You must define a BOARD_ID and add the corresponding definitions in board_definitions.h
#endif

// Common definitions
// ------------------

#define BOOT_PIN_MASK (1U << (BOOT_LOAD_PIN & 0x1f))
