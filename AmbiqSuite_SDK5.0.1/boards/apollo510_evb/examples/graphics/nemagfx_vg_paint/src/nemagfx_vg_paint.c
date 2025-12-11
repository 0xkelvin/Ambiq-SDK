//*****************************************************************************
//
//! @file nemagfx_vg_paint.c
//!
//! @brief NemaGFX Vector Graphics Paint and Fill Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_vg_paint NemaGFX Vector Graphics Paint Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates the paint and fill
//! capabilities of NemaGFX Vector Graphics library. The application showcases
//! various paint types and fill patterns for creating rich visual effects.
//!
//! @section nemagfx_vg_paint_features Key Features
//!
//! 1. @b Linear @b Gradient @b Paint: Demonstrates smooth color transitions
//!    along linear paths with configurable color stops
//!
//! 2. @b Texture @b Paint: Shows texture-based filling with image patterns
//!    and repeat modes for complex visual effects
//!
//! 3. @b Conic @b Gradient @b Paint: Implements radial color transitions
//!    from center outward with angular color distribution
//!
//! 4. @b Color @b Fill @b Paint: Basic solid color filling with alpha
//!    transparency support
//!
//! 5. @b Outline-Only @b Paint: Demonstrates stroke rendering without
//!    fill for wireframe and line-based graphics
//!
//! @section nemagfx_vg_paint_types Paint Types
//!
//! - @b Linear @b Gradient: Smooth color transitions along straight lines
//! - @b Texture @b Paint: Image-based pattern filling with tiling options
//! - @b Conic @b Gradient: Radial color transitions with angular control
//! - @b Solid @b Color: Uniform color filling with transparency
//! - @b Outline: Stroke-only rendering for wireframe effects
//!
//! @section nemagfx_vg_paint_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for paint effect visualization
//! - Sufficient memory for texture and gradient operations
//!
//! @section nemagfx_vg_paint_usage Usage
//!
//! The application automatically demonstrates various paint types on
//! different shapes and objects. Each paint type shows different
//! visual effects and rendering capabilities.
//!
//! @note Paint operations use hardware acceleration for optimal performance
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

#include "nemagfx_vg_paint.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

//*****************************************************************************
//
// Main Function
//
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
    am_util_stdio_printf("nemagfx_vg_paint Example\n");

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

