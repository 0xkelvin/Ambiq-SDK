//*****************************************************************************
//
//! @file nemagfx_vg_ttf.c
//!
//! @brief NemaGFX Vector Graphics True Type Font Rendering Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_vg_ttf NemaGFX Vector Graphics True Type Font Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates True Type Font (TTF) rendering
//! capabilities using NemaVG vector graphics library. The example showcases
//! 1MHztext rendering with various alignment options and font features.
//!
//! @section nemagfx_vg_ttf_features Key Features
//!
//! 1. @b True @b Type @b Font @b Support: Renders TTF fonts with full Unicode support
//!    and scalable vector graphics for crisp text at any resolution
//!
//! 2. @b Text @b Alignment @b Options: Demonstrates various text alignment features
//!    including left, center, right, and justified text positioning
//!
//! 3. @b Vector @b Graphics @b Rendering: Uses NemaVG for hardware-accelerated
//!    vector graphics rendering with optimal performance
//!
//! 4. @b Multi-threaded @b Architecture: Implements FreeRTOS tasks for rendering
//!    and display management with proper synchronization
//!
//! 5. @b High @b Performance @b Mode: Supports high-performance mode for optimal
//!    rendering performance on compatible hardware
//!
//! @section nemagfx_vg_ttf_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for text rendering visualization
//! - Sufficient memory for font caching and rendering
//!
//! @section nemagfx_vg_ttf_usage Usage
//!
//! The application automatically loads TTF fonts and displays various text
//! samples with different alignment options. Text rendering is performed
//! using vector graphics for optimal quality at all resolutions.
//!
//! @note This example requires TTF font files to be properly configured
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

#include "nemagfx_vg_ttf.h"

//*****************************************************************************
//
//! @brief Global variables for TTF rendering configuration
//!
//! @details Global variables and configuration structures for TTF font rendering
//! and vector graphics operations
//!
//*****************************************************************************

//*****************************************************************************
//
//! @brief Main application entry point for TTF rendering example
//!
//! @details Initializes the system, configures high-performance mode if enabled,
//! and starts the FreeRTOS tasks for TTF font rendering and display management.
//!
//! @return int Returns 0 on successful execution (never returns)
//!
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Status;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Configure the SEGGER SystemView Interface.
    //
#ifdef SYSTEM_VIEW
    SEGGER_SYSVIEW_Conf();
#endif

    //
    // Enable global IRQ.
    //
    am_hal_interrupt_master_enable();

    //
    // Initialize plotting interface.
    //
    am_util_stdio_printf("nemagfx_vg_ttf Example\n");

#ifdef RUN_IN_HP_MODE
#ifdef AM_PART_APOLLO330P_510L
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("HP_LP:Enter HP mode failed!\n");
    }
#else
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("HP_LP:Enter HP mode failed!\n");
    }
#endif
#endif

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

