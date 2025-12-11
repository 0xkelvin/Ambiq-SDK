//*****************************************************************************
//
//! @file systick_int.c
//!
//! @brief A simple example of using the SysTick interrupt.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup systick_int SYSTICK Interrupt Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates SysTick interrupt
//! functionality for system timing and periodic task scheduling. The
//! application showcases SysTick timer configuration, interrupt handling,
//! and periodic task execution for real-time system operations.
//!
//! @section systick_int_features Key Features
//!
//! 1. @b SysTick @b Configuration: Demonstrates SysTick
//!    timer setup and interrupt configuration for system timing
//!
//! 2. @b Interrupt @b Service @b Routine: Implements SysTick interrupt
//!    handler for periodic task execution and timing control
//!
//! 3. @b Periodic @b Task @b Scheduling: Provides periodic task execution
//!    with configurable timing intervals for system operations
//!
//! 4. @b Real @b Time @b Monitoring: Implements real-time status monitoring
//!    and periodic debug output via SWO interface
//!
//! 5. @b Timing @b Control: Provides precise timing control with
//!    configurable interrupt intervals and task scheduling
//!
//! @section systick_int_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes SysTick timer with configurable interrupt intervals
//! - Implements SysTick interrupt service routine for periodic tasks
//! - Provides periodic status monitoring and debug output
//! - Implements precise timing control for system operations
//! - Demonstrates proper interrupt handling and task scheduling
//! - Monitors system timing and provides real-time status updates
//!
//! @section systick_int_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor SWO output for periodic timing status updates
//! 3. Verify SysTick interrupt operation and timing accuracy
//! 4. Observe periodic task execution and system timing
//!
//! @section systick_int_configuration Configuration
//!
//! - @b INT_NUM_MS: Configurable interrupt interval (default: 50ms)
//! - @b SYSTICK_CNT: SysTick counter value calculation
//! - @b PNT_PERIOD_MS: Print period for status updates (default: 2000ms)
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via SWO
//!
//! Since the clock to the core is gated during sleep, whether deep sleep or
//! normal sleep, the SysTick interrupt cannot be used to wake the device.
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
// Local defines
//
//*****************************************************************************
//
// Set the desired number of milliseconds between interrupts
//
#define INT_NUM_MS      50  // Number of desired ms between interrupts

//
// Compute the number of needed ticks between interrupts
//
#define SYSTICK_CNT     (AM_HAL_CLKGEN_FREQ_MAX_HZ / 1000 * INT_NUM_MS)

#define PNT_PERIOD_MS       2000

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
uint32_t g_ui32IntCount = 0;
uint32_t g_ui32IntPntCnt = 0;
uint32_t g_ui32PrintSecs = 0;

//*****************************************************************************
//
// SysTick ISR
//
//*****************************************************************************
void
SysTick_Handler(void)
{
    g_ui32IntPntCnt++;

    //
    // Periodically print time status to SWO
    //
    if ( (g_ui32IntPntCnt * INT_NUM_MS) >= PNT_PERIOD_MS )
    {
        g_ui32IntPntCnt = 0;

        //
        // Print to SWO
        //
        am_bsp_debug_printf_enable();

        g_ui32PrintSecs += (PNT_PERIOD_MS / 1000);
        am_util_stdio_printf("%7d", g_ui32PrintSecs );

        if ( ((g_ui32PrintSecs / (PNT_PERIOD_MS / 1000)) % 10) == 0 )
        {
            am_util_stdio_printf("\n");
        }

        am_bsp_debug_printf_disable();
    }

} // SysTick_Handler()

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32TickCount;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM output
    //
    if ( am_bsp_debug_printf_enable() )
    {
        //
        // An error occurred.
        // Just as an indication, force a 3 second delay before proceeding.
        //
        am_hal_delay_us(3000000);
    }

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("systick_int\n\n");
    am_util_stdio_printf("This example is a simple demonstration of the SysTick timer and interrupts.\n");
    am_util_stdio_printf(" The seconds count is printed every 2 seconds.\n\n");

    //
    // Enable interrupts to the core.
    //
    am_hal_interrupt_master_enable();

    //
    // Configure Systick to use the internal (Core) clock.
    //
    am_hal_systick_init(AM_HAL_SYSTICK_CLKSRC_INT);

    //
    // Retrieve and calculate the number of Tick for the desired interrupt interval.
    //
    am_hal_systick_ticks_per_ms(&ui32TickCount);

    //
    // Compute the total number of ticks for the desired number of milliseconds
    // between each interrupt.
    //
    ui32TickCount *= INT_NUM_MS;

    //
    // Check the count as SysTick is 24 bits.
    //
    if ( ui32TickCount >= (1 << 24) )
    {
        am_util_stdio_printf("systick_int: Error, tick count is too large.\n");
        am_util_stdio_printf("             Halting the example.\n");
        am_bsp_debug_printf_disable();
        while(1);
    }

    //
    // Disable printf messages on ITM.
    //
    am_bsp_debug_printf_disable();

    //
    // Enable the Systick
    //
    am_hal_systick_int_enable();

    //
    // Set the Systick interrupt priority.
    //
    NVIC_SetPriority (SysTick_IRQn, AM_IRQ_PRIORITY_DEFAULT);

    //
    // Load the tick value.
    //
    am_hal_systick_load(ui32TickCount);

    //
    // Start systick
    //
    am_hal_systick_start();

    while (1)
    {
    }
} // main()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

