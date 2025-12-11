//*****************************************************************************
//
//! @file reset_states.c
//!
//! @brief Example of various reset options in Apollo.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup reset_states Reset States Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates reset state management
//! and watchdog timer functionality for system monitoring and recovery.
//! The application showcases various reset types including watchdog reset,
//! power-on reset, external reset, and brown-out reset.
//!
//! @section reset_states_features Key Features
//!
//! 1. @b Reset @b State @b Detection: Implements reset
//!    state detection and decoding for various reset types
//!
//! 2. @b Watchdog @b Timer @b Management: Demonstrates watchdog timer
//!    configuration with interrupt and reset generation capabilities
//!
//! 3. @b Multiple @b Reset @b Types: Supports various reset types including
//!    watchdog, power-on, external, and brown-out reset detection
//!
//! 4. @b System @b Recovery: Implements automatic system recovery and
//!    restart mechanisms for operation
//!
//! 5. @b Reset @b Sequence @b Control: Provides controlled reset sequence
//!    management with configurable reset behavior
//!
//! @section reset_states_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes watchdog timer with interrupt and reset capabilities
//! - Implements reset state detection and decoding for various reset types
//! - Provides controlled reset sequence management and monitoring
//! - Demonstrates watchdog interrupt handling and reset generation
//! - Implements system recovery and restart mechanisms
//! - Monitors reset states and provides detailed status reporting
//!
//! @section reset_states_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM output for reset state information and watchdog events
//! 3. Observe reset sequence: Power cycle -> 5 WDT Interrupts -> WDT Reset
//! 4. Test external reset by pressing and releasing external reset pin
//! 5. Verify brown-out reset by lowering and restoring VDD_MCU
//!
//! @section reset_states_configuration Configuration
//!
//! - @b WDT_CLOCK_SOURCE: Watchdog clock source (default: LFRC_DIV64)
//! - @b WDT_INTERRUPT_TIMEOUT: Interrupt timeout value (default: 12 seconds)
//! - @b WDT_RESET_TIMEOUT: Reset timeout value (default: 24 seconds)
//! - @b RESET_SEQUENCE: Configurable reset sequence behavior
//!
//! The program will repeat the following sequence on the console after power cycling:
//! Power cycle (POA Reset) - 5 WDT Interrupts - WDT Reset - 3 WDT Interrupts -
//! POR Reset - 3 WDT Interrupts - repeat "POI Reset - 3 WDT Interrupts"
//!
//! Pressing then releasing external reset pin can trigger an external reset.
//! Writing 1 to AIRCR.SYSRESETREQ can trigger an AIRCR SYSRESETREQ reset. (Writing
//! 0x05FA to VECTKEY is needed.)
//! Lowering down VDD_MCU then restoring it can trigger a brown-out BOUNREG reset.
//!
//! Printing takes place over the ITM at 1MHz.
//!
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

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint8_t g_ui8NumWatchdogInterrupts = 0;

am_hal_wdt_config_t g_sWatchdogConfig =
{
    // Set the clock for ~16HZ
    .eClockSource = AM_HAL_WDT_LFRC_DIV64,

    // Enable the interrupt, and set it for 3/4 of a second.
    .bInterruptEnable = true,
    .ui32InterruptValue = 16 * 3 / 4,

    // Enable the reset, and set it for 1.5 seconds.
    .bResetEnable = true,
    .ui32ResetValue = 16 * 3 / 2,
};

enum
{
     NEXT_WATCHDOG = 0,
     NEXT_SWPOR,
     NEXT_SWPOI,
}g_eNextReset = NEXT_WATCHDOG;

//*****************************************************************************
//
// Interrupt handler for the watchdog.
//
//*****************************************************************************
void
am_watchdog_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and clear the interrupt status.
    //
    am_hal_wdt_interrupt_status_get(AM_HAL_WDT_MCU, &ui32Status, true);
    am_hal_wdt_interrupt_clear(AM_HAL_WDT_MCU, ui32Status);
    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Send a status message and give it some time to print.
    //
    am_util_stdio_printf("WDT Interrupt!\n");
    am_util_delay_ms(10);

    //
    // Catch the first four watchdog interrupts, but let the fifth through
    // untouched.
    //
    if ( g_ui8NumWatchdogInterrupts < 4 )
    {
        am_util_stdio_printf("Pet the watchdog.\n\n");
        //
        // Restart the watchdog.
        //
        am_hal_wdt_restart(AM_HAL_WDT_MCU);
    }
    else
    {
        am_util_stdio_printf("Do not pet the watchdog, trigger the WDT reset.\n\n");
        am_bsp_debug_printf_disable();
    }

    //
    // On the second interrupt do a different kind of reset
    //
    if (g_ui8NumWatchdogInterrupts == 2)
    {
        //
        //  If next reset is not a watch dog reset we will do other reset here.
        //
        if (g_eNextReset != NEXT_WATCHDOG )
        {
            //
            // Stop the watchdog.
            //
            am_hal_wdt_stop(AM_HAL_WDT_MCU);

            //
            // Check if we want to do a SW_POR or SW_POI.
            //
            if (g_eNextReset == NEXT_SWPOR)
            {
                am_util_stdio_printf("Trigger the SW POR.\n\n");
                am_bsp_debug_printf_disable();
                am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOR, 0);
            }
            else
            {
                am_util_stdio_printf("Trigger the SW POI.\n\n");
                am_bsp_debug_printf_disable();
                am_hal_reset_control(AM_HAL_RESET_CONTROL_SWPOI, 0);
            }
        }
    }

    //
    // Increment the number of watchdog interrupts.
    //
    g_ui8NumWatchdogInterrupts++;
}

//*****************************************************************************
//
// Function to decode and print the reset cause.
//
//*****************************************************************************
void
reset_decode(void)
{
    uint32_t ui32ResetStatus;
    am_hal_reset_status_t sStatus;

    //
    // Print out reset status register upon entry.
    //
    am_hal_reset_status_get(&sStatus);
    ui32ResetStatus = sStatus.eStatus;
    am_util_stdio_printf("Reset Status Register = 0x%04X\n", ui32ResetStatus);

    //
    // POWER CYCLE.
    //
    if ( ui32ResetStatus & AM_HAL_RESET_STATUS_POA )
    {
        am_util_stdio_printf("Power Cycle Initiated Reset\n");
        g_eNextReset = NEXT_WATCHDOG;
    }

    //
    // WATCHDOG.
    //
    if ( ui32ResetStatus & AM_HAL_RESET_STATUS_WDT )
    {
        am_util_stdio_printf("Watchdog Initiated Reset\n");
        g_eNextReset = NEXT_SWPOR;
    }

    //
    // SOFTWARE POR.
    //
    if ( ui32ResetStatus & AM_HAL_RESET_STATUS_SWPOR )
    {
        am_util_stdio_printf("Software POR Initiated Reset\n");
        g_eNextReset = NEXT_SWPOI;
    }

    //
    // SOFTWARE POI.
    //
    if ( ui32ResetStatus & AM_HAL_RESET_STATUS_SWPOI )
    {
        am_util_stdio_printf("Software POI Initiated Reset\n");
        g_eNextReset = NEXT_SWPOI;
    }

    //
    // EXTERNAL RESET PIN.
    // Pressing then releasing external reset pin can trigger an external reset.
    //
    if ( ui32ResetStatus & AM_HAL_RESET_STATUS_EXTERNAL )
    {
        am_util_stdio_printf("External Reset Pin Initiated This Reset\n");
        g_eNextReset = NEXT_WATCHDOG;
    }

    //
    // AIRCR SYSRESETREQ RESET.
    // Writing 1 to AIRCR.SYSRESETREQ can trigger an AIRCR SYSRESETREQ reset.
    // (Writing 0x05FA to VECTKEY is needed.)
    //
    if ( ui32ResetStatus & AM_HAL_RESET_STATUS_AIRCR )
    {
        am_util_stdio_printf("AIRCR SYSRESETREQ Initiated Reset\n");
        g_eNextReset = NEXT_WATCHDOG;
    }

    //
    // BROWNOUT RESET.
    // Lowering down VDD_MCU then restoring it can trigger a brown-out BOUNREG reset.
    //
    if ( ui32ResetStatus &
         (AM_HAL_RESET_STATUS_BOD | AM_HAL_RESET_STATUS_BOUNREG | AM_HAL_RESET_STATUS_BOCORE |
          AM_HAL_RESET_STATUS_BOMEM | AM_HAL_RESET_STATUS_BOHPMEM) )
    {
        am_util_stdio_printf("Brownout Initiated Reset\n");
        g_eNextReset = NEXT_WATCHDOG;
    }

    am_util_stdio_printf("\n");

    if ( g_eNextReset == NEXT_WATCHDOG )
    {
        am_util_stdio_printf("Next reset will be from WDT, we set up 5 WDT interrupts before next reset action in this example.\n");
    }
    else
    {
        am_util_stdio_printf("Next reset will not be from WDT, we set up 3 WDT interrupts before next reset action in this example.\n");
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set g_eNextReset to a watchdog timeout reset.
    //
    g_eNextReset = NEXT_WATCHDOG;

    //
    // Stop the watch dog if we are coming in from a reset
    // other than a power cycle
    //
    am_hal_wdt_stop(AM_HAL_WDT_MCU);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Initialize the LED
    //
#if AM_BSP_NUM_LEDS > 0
    am_devices_led_init(am_bsp_psLEDs);
#endif

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal screen, and print a quick message to show that we're
    // alive.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Reset State Tracking Example.\n\n");

    //
    // Decode and print the reset state that got us here.
    //
    reset_decode();

    //
    // Give user a little time to read the type of reset.
    //
    am_util_delay_ms(2000);

    //
    // Configure the watchdog.
    //
    am_hal_wdt_config(AM_HAL_WDT_MCU, &g_sWatchdogConfig);
    am_hal_wdt_interrupt_enable(AM_HAL_WDT_MCU, AM_HAL_WDT_INTERRUPT_MCU);

    //
    // Enable the interrupt for the watchdog in the NVIC.
    //
    NVIC_SetPriority(WDT_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(WDT_IRQn);
    am_hal_interrupt_master_enable();

    //
    // Enable the watchdog.
    //
    am_hal_wdt_start(AM_HAL_WDT_MCU, false);

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // We are done printing. Disable debug printf messages on ITM.
        //
        am_bsp_debug_printf_disable();

        //
        // Disable interrupts.
        //
        am_hal_interrupt_master_disable();

        //
        // Turn OFF the indicator LED.
        //
#if AM_BSP_NUM_LEDS > 0
        am_devices_led_off(am_bsp_psLEDs, 0);
#endif

        //
        // Go to sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        //
        // Turn ON the indicator LED.
        //
#if AM_BSP_NUM_LEDS > 0
        am_devices_led_on(am_bsp_psLEDs, 0);
#endif

        //
        // An interrupt woke us up so now enable them and take it.
        //
        am_hal_interrupt_master_enable();
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

