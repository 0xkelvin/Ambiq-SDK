//*****************************************************************************
//
//! @file stimer.c
//!
//! @brief Example using a STIMER with interrupts.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup stimer STIMER Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates STIMER (Secure Timer)
//! functionality for precise timing and interrupt-driven operations. The
//! application showcases STIMER configuration, interrupt handling, and
//! periodic task execution with LED control for visual feedback. The
//! example implements timing control using 32kHz crystal
//! oscillator for accurate timekeeping and demonstrates proper interrupt
//! service routine management for reliable system timing.
//!
//! @section stimer_features Key Features
//!
//! 1. @b STIMER @b Configuration: Demonstrates STIMER setup
//!    and interrupt configuration for precise timing operations
//!
//! 2. @b Interrupt @b Driven @b Control: Implements STIMER interrupt service
//!    routine for periodic task execution and timing control
//!
//! 3. @b LED @b Visual @b Feedback: Provides LED control for visual
//!    feedback of timer operation and system status
//!
//! 4. @b Crystal @b Oscillator @b Timing: Uses 32kHz crystal oscillator
//!    for accurate and stable timing operations
//!
//! 5. @b Periodic @b Task @b Execution: Implements periodic task execution
//!    with configurable timing intervals for system operations
//!
//! @section stimer_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes STIMER with 32kHz crystal oscillator configuration
//! - Implements STIMER interrupt service routine for periodic tasks
//! - Provides LED visual feedback for timer operation status
//! - Implements precise timing control with configurable intervals
//! - Demonstrates proper interrupt handling and task scheduling
//! - Monitors system timing and provides visual status updates
//!
//! @section stimer_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Observe LED behavior for timer operation feedback
//! 3. Monitor ITM output for timing status and configuration
//! 4. Verify STIMER interrupt operation and timing accuracy
//!
//! @section stimer_configuration Configuration
//!
//! - @b WAKE_INTERVAL_IN_MS: Configurable wake interval (default: 1000ms)
//! - @b XT_PERIOD: Crystal oscillator period (32kHz)
//! - @b WAKE_INTERVAL: Calculated timer interval value
//! - @b AM_IRQ_PRIORITY_DEFAULT: Configurable interrupt priority
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
// Macro definitions
//
//*****************************************************************************
#define WAKE_INTERVAL_IN_MS     1000
#define XT_PERIOD               32768
#define WAKE_INTERVAL           (XT_PERIOD * WAKE_INTERVAL_IN_MS / 1000)

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
static uint32_t g_ui32Count = 0;

//*****************************************************************************
//
// Init function for Timer A0.
//
//*****************************************************************************
void
stimer_init(void)
{
    //
    // Enable compare A interrupt in STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_SetPriority(STIMER_CMPR0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);

    //
    // Configure the STIMER and run
    //
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);

}

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_stimer_cmpr0_isr(void)
{
    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);

#if AM_BSP_NUM_LEDS > 0
#if AM_BSP_NUM_LEDS == 1
    //
    // If only a single LED, toggle it.
    //
    am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, g_ui32Count & 1);
#else
    //
    // If multiple LEDs, walk through them.
    //
    am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, 1 << (g_ui32Count % AM_BSP_NUM_LEDS));
#endif

#else
    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    am_util_stdio_printf("%d ", g_ui32Count & 7);

    //
    // Do a linefeed after 32 prints.
    //
    if ( (g_ui32Count & 0x1F) == 31 )
    {
        am_util_stdio_printf("\n");
    }

    //
    // Disable SWO printing.
    //
    am_bsp_debug_printf_disable();
#endif // AM_BSP_NUM_LEDS

    g_ui32Count++;
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
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#if (AM_BSP_NUM_LEDS > 0)
    //
    // Configure the pins for this board.
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
#endif

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("STIMER Example\n");

    am_util_stdio_printf("Stimer wakes from deepsleep every %ds to ",
                         WAKE_INTERVAL_IN_MS / 1000);

#if AM_BSP_NUM_LEDS > 0
    am_util_stdio_printf("walk the %d LEDs.\n\n", AM_BSP_NUM_LEDS);
#else
    am_util_stdio_printf("print to SWO.\n"
                         " Note that the count is 0-7 to emulate a board having 3 LEDs.\n\n");
#endif
    am_util_delay_ms(10);

    //
    // Disable SWO printing.
    //
    am_bsp_debug_printf_disable();

    //
    // STIMER init.
    //
    stimer_init();

    //
    // Enable the timer interrupt in the NVIC.
    //
    am_hal_interrupt_master_enable();

    //
    // Sleep forever while waiting for an interrupt.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
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

