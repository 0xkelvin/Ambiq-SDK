//*****************************************************************************
//
//! @file nemagfx_watchface.c
//!
//! @brief NemaGFX Watchface Example - Real-time Clock Display
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_watchface NemaGFX Watchface Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates the creation of a watch face
//! application using NemaGFX graphics library. The watch face features:
//!
//! - Real-time clock display with rotating hour, minute, and second hands
//! - Background image rendering with proper scaling and positioning
//! - SVG format weather icon display with dynamic updates
//! - Power-optimized rendering with GPU and display power management
//!
//! @section nemagfx_watchface_features Key Features
//!
//! 1. @b Rotating @b Clock @b Hands: Uses raw NemaGFX APIs for smooth hand rotation
//!    with proper pivot point calculations and matrix transformations
//!
//! 2. @b GPU @b Power @b Management: Implements intelligent GPU power cycling where
//!    the GPU powers off after command list execution and restarts for next frame
//!
//! 3. @b Display @b Power @b Optimization: DC and DSI are powered up only during
//!    frame transfer operations and immediately shut down after completion
//!
//! 4. @b Multi-threaded @b Architecture: Uses FreeRTOS tasks for rendering and
//!    display management with proper synchronization
//!
//! @section nemagfx_watchface_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel with DSI interface
//! - External power supply for stable operation
//!
//! @section nemagfx_watchface_usage Usage
//!
//! The application automatically starts displaying the watch face upon boot.
//! The clock hands update in real-time, and weather icons cycle periodically.
//!
//! @note This example requires proper display configuration and external power
//! for optimal performance and power management features.
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
#include "nemagfx_watchface.h"
#include "rtos.h"

//*****************************************************************************
//
//! @brief Main application entry point
//!
//! @details Initializes the system, configures power management, and starts
//! the FreeRTOS tasks for watch face rendering and display management.
//!
//! @return int Returns 0 on successful execution (never returns)
//!
//*****************************************************************************
int
main(void)
{
    //
    // External power on
    //
    am_bsp_external_pwr_on();

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // Print debug information.
    //
    am_util_debug_printf("FreeRTOS nemagfx_watchface Example\n");

    //
    // Run the application.
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

