//*****************************************************************************
//
//! @file deepsleep.c
//!
//! @brief Example demonstrating how to enter deepsleep.
//!
//! @addtogroup power_examples Power Examples
//!
//! @defgroup deepsleep Deepsleep Example
//! @ingroup power_examples
//! @{
//!
//! Purpose: This example demonstrates deep sleep functionality
//! including memory optimization, ROM auto power down, and minimal DTCM retention.
//! The example provides a clean environment for measuring deep sleep current
//! without interrupt interference, enabling power consumption analysis for
//! low-power applications.
//!
//! @section deepsleep_features Key Features
//!
//! 1. @b Deep @b Sleep @b Mode: Configures device to enter deep sleep mode
//!    for power conservation and current measurement
//!
//! 2. @b Memory @b Configuration: Optimizes memory retention settings for
//!    minimum DTCM retention while maintaining essential functionality
//!
//! 3. @b Power @b Measurement: Provides clean environment for measuring
//!    deep sleep current without interrupt interference
//!
//! 4. @b ROM @b Auto @b Power @b Down: Enables ROM automatic power down
//!    feature for lowest possible power consumption
//!
//! 5. @b UART @b Communication: Initializes UART for status reporting.
//!    Disables UART before entering sleep mode for debugging and monitoring
//!
//! @section deepsleep_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes memory for performance and low power operation
//! - Configures ROM auto power down for minimum power consumption
//! - Optimizes DTCM retention settings for power efficiency
//! - Places device into deep sleep mode for current measurement
//! - Provides UART communication for status reporting and debugging
//! - Enables power measurement environment
//!
//! @section deepsleep_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor UART output for initialization and status messages
//! 3. Observe device entering deep sleep mode
//! 4. Measure current draw during deep sleep operation
//! 5. Verify power consumption features
//!
//! @section deepsleep_configuration Configuration
//!
//! - @b ALL_RETAIN: Memory retention configuration (0 for min TCM, 1 for all)
//! - @b UART: Communication interface (115,200 BAUD, 8 bit, no parity)
//! - @b ROM_MODE: ROM auto power down configuration
//! - @b DTCM_CONFIG: DTCM memory configuration for power optimization
//! <br>
//!   1) Initialize memory for performance and low power<br>
//!   2) Place device into deepsleep<br>
//!   3) Measure current draw<br>
//! <br>
//! The example begins by printing out a banner announcement message through<br>
//! the UART, which is then completely disabled for the remainder of execution.<br>
//! <br>
//! Text is output to the UART at 115,200 BAUD, 8 bit, no parity.<br>
//! Please note that text end-of-line is a newline (LF) character only.<br>
//! Therefore, the UART terminal must be set to simulate a CR/LF.<br>
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk5p0p1-61912905f0 of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#ifdef AM_BSP_IS_SIP
#include "am_devices_em9305.h"
#endif

#define ALL_RETAIN  0 // 0 for min TCM retain (default), 1 for all retain

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
    {
         //
         // In order to demonstrate the lowest possible power,
         // this example enables the ROM automatic power down feature.
         // This should not be used in general for most applications.
         //
        .eROMMode       = AM_HAL_PWRCTRL_ROM_AUTO,
#if ALL_RETAIN
#ifdef AM_PART_APOLLO330P_510L
        .eDTCMCfg       = AM_HAL_PWRCTRL_DTCM256K,
#else
        .eDTCMCfg       = AM_HAL_PWRCTRL_ITCM256K_DTCM512K,
#endif
        .eRetainDTCM    = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#else
#ifdef AM_PART_APOLLO330P_510L
        .eDTCMCfg       = AM_HAL_PWRCTRL_DTCM128K,
#else
        .eDTCMCfg       = AM_HAL_PWRCTRL_ITCM32K_DTCM128K,
#endif
        .eRetainDTCM    = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#endif
#ifdef AM_PART_APOLLO330P_510L
        .eNVMCfg        = AM_HAL_PWRCTRL_NVM,
#else
        .eNVMCfg        = AM_HAL_PWRCTRL_NVM0_ONLY,
#endif
        .bKeepNVMOnInDeepSleep     = false
    };

    am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg =
    {
#if ALL_RETAIN
#ifdef AM_PART_APOLLO330P_510L
        .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_1P75M,
#else
        .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_3M,
#endif
#else
        .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_NONE,
#endif
        .eActiveWithMCU   = AM_HAL_PWRCTRL_SRAM_NONE,
        .eActiveWithGFX   = AM_HAL_PWRCTRL_SRAM_NONE,
        .eActiveWithDISP  = AM_HAL_PWRCTRL_SRAM_NONE,
#if ALL_RETAIN
#ifdef AM_PART_APOLLO330P_510L
        .eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_1P75M
#else
        .eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_3M
#endif
#else
        .eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_NONE
#endif

    };

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    //
    // If Apollo510B device, turn off EM9305 to save power
    //
#ifdef AM_BSP_IS_SIP
    am_devices_em9305_shutdown();
#endif

#ifdef AM_PART_APOLLO330P_510L

    //
    // Power off the RSS
    //
    am_hal_pwrctrl_rss_pwroff();
#endif

    //
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Deepsleep Example\n");

    //
    // To minimize power during the run, disable the UART.
    //
    am_bsp_uart_printf_disable();



    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC); // Use LFRC instead of XT

    //
    // Configure XTAL for deepsleep
    //
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, 0);

    //
    // Disable RTC Oscillator
    //
    am_hal_rtc_osc_disable();

    //
    // Disable all peripherals including Crypto
    //
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL, 0);

    //
    // Power off voltage comparator
    //
    VCOMP -> PWDKEY = VCOMP_PWDKEY_PWDKEY_Key;

    //
    // Disable Debug Subsystem
    //
    MCUCTRL->DBGCTRL = 0;

    am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);

    //
    // MRAM0 LP Setting affects the Latency to wakeup from Sleep. Hence, it should be configured in the application.
    // Configure the MRAM for low power mode
    //
    MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0LPREN = 1;
    MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0SLPEN = 0;
    MCUCTRL->MRAMCRYPTOPWRCTRL_b.MRAM0PWRCTRL = 1;

    //
    // Disable SRAM
    //
    am_hal_pwrctrl_sram_config(&SRAMMemCfg);
    while (1)
    {
        //
        // Go to Deep Sleep and stay there.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
