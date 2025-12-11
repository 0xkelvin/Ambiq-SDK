//*****************************************************************************
//
//! @file watchdog.c
//!
//! @brief Example of a basic configuration and usage of the watchdog.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup watchdog Watchdog Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates watchdog timer configuration
//! and operation for system monitoring and recovery. The application showcases
//! watchdog setup with both interrupt and reset generation capabilities,
//! implementing a monitoring system that provides multiple
//! levels of system protection. The watchdog ISR demonstrates proper
//! interrupt handling and timer management, with controlled system reset
//! behavior for system operation and recovery.
//!
//! @section watchdog_features Key Features
//!
//! 1. @b Watchdog @b Configuration: Demonstrates basic watchdog setup with
//!    interrupt and reset generation capabilities
//!
//! 2. @b Interrupt @b Handling: Implements watchdog interrupt service routine
//!    for monitoring and control of watchdog behavior
//!
//! 3. @b Reset @b Generation: Configures watchdog to trigger system reset
//!    after timeout for system recovery and safety
//!
//! 4. @b Timer @b Management: Provides watchdog timer control with configurable
//!    timeout values and clock sources
//!
//! 5. @b System @b Recovery: Demonstrates automatic system recovery through
//!    watchdog reset functionality for operation
//!
//! @section watchdog_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures watchdog timer with interrupt and reset capabilities
//! - Sets up watchdog ISR for monitoring and control operations
//! - Implements controlled watchdog petting sequence (4 times)
//! - Demonstrates system reset behavior after timeout
//! - Provides real-time monitoring via ITM debug interface
//! - Implements system recovery mechanisms
//! - Monitors and reports watchdog status and behavior
//!
//! @section watchdog_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM output for watchdog status and interrupt messages
//! 3. Observe watchdog petting sequence and reset behavior
//! 4. Verify system recovery and restart functionality
//!
//! @section watchdog_configuration Configuration
//!
//! - @b WDT_CLOCK_SOURCE: Configurable watchdog clock source (default: LFRC_DIV64)
//! - @b WDT_INTERRUPT_TIMEOUT: Interrupt timeout value (default: 12 seconds)
//! - @b WDT_RESET_TIMEOUT: Reset timeout value (default: 15 seconds)
//! - @b WDT_PET_COUNT: Number of watchdog pets before reset (default: 4)
//!
//! Printing takes place over the ITM at 1MHz.
//!
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

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint8_t g_ui8NumWatchdogInterrupts = 0;
uint32_t g_ui32ResetStatus = 0;

am_hal_wdt_config_t g_sWatchdogConfig =
{
    // Set the clock for 16HZ
    .eClockSource = AM_HAL_WDT_LFRC_DIV64,

    // Enable the interrupt, and set it for 3/4 of a second.
    .bInterruptEnable = true,
    .ui32InterruptValue = 16 * 3 / 4,

    // Enable the reset, and set it for 15 seconds.
    .bResetEnable = true,
    .ui32ResetValue = 16 * 15,
};

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
    // Send a status message and give it some time to print.
    //
    am_util_stdio_printf("Interrupt #: %d\n", g_ui8NumWatchdogInterrupts);

    //
    // Catch the first four watchdog interrupts, but let the fifth through
    // untouched.
    //
    if ( g_ui8NumWatchdogInterrupts < 4 )
    {
        //
        // Restart the watchdog.
        //
        am_hal_wdt_restart(AM_HAL_WDT_MCU);
    }
    else
    {
        am_util_stdio_printf("\nAllowing watchdog to keep running.\n");
        am_util_stdio_printf("Resetting MCU when watchdog expires...\n");
    }

    am_util_delay_ms(100);

    //
    // Increment the number of watchdog interrupts.
    //
    g_ui8NumWatchdogInterrupts++;

} // am_watchdog_isr()

void *g_ComUART;

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    am_hal_reset_status_t sStatus;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#if AM_BSP_NUM_LEDS > 0
    //
    // Initialize device drivers for the LEDs on the board.
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
#endif

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    // am_hal_uart_initialize(0, &g_ComUART);
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal screen, and print a quick message to show that we're
    // alive.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Watchdog Example.\n");

    //
    // Print out reset status register upon entry.
    //
    am_hal_reset_status_get(&sStatus);
    g_ui32ResetStatus = sStatus.eStatus;

    am_util_stdio_printf("Reset Status Register = 0x%x\n",
                         g_ui32ResetStatus);

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
        //am_bsp_debug_printf_disable();

        //
        // Disable interrupts.
        //
        am_hal_interrupt_master_disable();

#if AM_BSP_NUM_LEDS > 0
        //
        // Turn OFF the indicator LED.
        //
        am_devices_led_off(am_bsp_psLEDs, 0);
#endif

        //
        // am_bsp_debug_printf_deepsleep_prepare() must be called when keeping SWO/ITM
        // enabled during deepsleep. It must be called again immediately after exiting
        // deepsleep.
        //
        am_bsp_debug_printf_deepsleep_prepare(true);

        //
        // Go to deep sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        am_bsp_debug_printf_deepsleep_prepare(false);

#if AM_BSP_NUM_LEDS > 0
        //
        // Turn ON the indicator LED.
        //
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

