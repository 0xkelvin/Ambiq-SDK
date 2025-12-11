//*****************************************************************************
//
//! @file freertos_sample.c
//!
//! @brief Example program which demonstrates using FreeRTOS
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup freertos_sample FreeRTOS Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates FreeRTOS real-time
//! operating system functionality for embedded applications. The application
//! showcases FreeRTOS features including task management, timer
//! interrupts, deep sleep operation, and GPIO status monitoring. The example
//! provides a foundation for FreeRTOS-based embedded development with
//! power management and real-time task scheduling capabilities.
//!
//! @section freertos_sample_features Key Features
//!
//! 1. @b FreeRTOS @b Task @b Management: Implements task
//!    scheduling and management within FreeRTOS framework
//!
//! 2. @b GPIO @b Status @b Monitoring: Provides GPIO-based status indication
//!    for task execution and system state monitoring
//!
//! 3. @b Deep @b Sleep @b Operation: Implements deep sleep functionality
//!    for power optimization in embedded applications
//!
//! 4. @b Timer @b Interrupt @b Support: Provides timer interrupt handling
//!    for real-time system operation
//!
//! 5. @b Power @b Management: Optimized for low power operation with
//!    intelligent power management features
//!
//! @section freertos_sample_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes FreeRTOS with task management and scheduling
//! - Implements GPIO status monitoring for system state indication
//! - Provides deep sleep operation for power optimization
//! - Manages timer interrupts for real-time operation
//! - Supports FreeRTOS functionality demonstration
//! - Implements power management and cache optimization
//!
//! @section freertos_sample_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor GPIO pins for task execution status
//! 3. Observe deep sleep entry and exit operations
//! 4. Monitor timer interrupt execution
//! 5. Test FreeRTOS task scheduling and management
//!
//! @section freertos_sample_configuration Configuration
//!
//! - @b GPIO @b Pins: Status monitoring pins (0-4)
//! - @b ITM: Output for debug messages
//! - @b Power @b Management: Deep sleep optimization
//! - @b Cache @b Configuration: I-Cache and D-Cache enablement
//!
//! The following GPIOs are configured for observation:
//!
//!   GPIO           Toggles
//! --------- ---------------------------------
//!    0     on each iteration of Hello Task loop
//!    1     on each iteration of Idle Task
//!    2     before deepsleep entry
//!    3     immediately following deepsleep exit
//!    4     on each Timer ISR entry
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
#include "am_util.h"
#include "freertos_sample.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

//*****************************************************************************
//
// FreeRTOS Sample Test
//
//*****************************************************************************
int
main(void)
{


    //
    // Clear status GPIOs
    //
    am_hal_gpio_state_write(GPIO_HELLO_TASK_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_state_write(GPIO_IDLE_TASK_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_state_write(GPIO_SLEEP_ENTRY_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_state_write(GPIO_SLEEP_EXIT_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_gpio_state_write(GPIO_TIMER_ISR_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

    //
    // Initialize status GPIOs as output
    //
    am_hal_gpio_pinconfig(GPIO_HELLO_TASK_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(GPIO_IDLE_TASK_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(GPIO_SLEEP_ENTRY_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(GPIO_SLEEP_EXIT_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(GPIO_TIMER_ISR_PIN, am_hal_gpio_pincfg_output);

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

#ifdef AM_PART_APOLLO330P_510L
    //
    // Power off the RSS
    //
    am_hal_pwrctrl_rss_pwroff();
#endif

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Initialize the printf interface for ITM output
    //
    if (am_bsp_debug_printf_enable())
    {
        // Cannot print - so no point proceeding
        while(1);
    }

    //
    // Initialize plotting interface.
    //
    am_util_stdio_printf("\nRunning freertos_sample\n");

    //
    // Start the tasks.
    //
    run_tasks();

    //
    // We shouldn't ever get here.
    //
    while (1)
    {
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
