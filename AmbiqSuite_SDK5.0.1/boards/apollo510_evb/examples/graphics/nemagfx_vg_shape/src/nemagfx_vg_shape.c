//*****************************************************************************
//
//! @file nemagfx_vg_shape.c
//!
//! @brief NemaGFX Vector Graphics Shape Rendering Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_vg_shape NemaGFX Vector Graphics Shape Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates the shape rendering
//! capabilities of NemaGFX Vector Graphics library. The application showcases
//! various predefined shapes with different paint settings and transformations.
//!
//! @section nemagfx_vg_shape_features Key Features
//!
//! 1. @b Basic @b Shapes: Demonstrates fundamental geometric shapes including
//!    circles, ellipses, rectangles, and lines
//!
//! 2. @b Shapes: Shows complex shapes like rings and rounded
//!    rectangles with configurable parameters
//!
//! 3. @b Paint @b Integration: Combines shape rendering with various paint
//!    features including gradients, patterns, and color fills
//!
//! 4. @b Vector @b Graphics: Uses hardware-accelerated vector graphics for
//!    scalable shape rendering
//!
//! 5. @b High @b Performance @b Mode: Supports high-performance mode for optimal
//!    rendering performance on compatible hardware
//!
//! @section nemagfx_vg_shape_shapes Available Shapes
//!
//! - @b Circle: Perfect circular shapes with configurable radius
//! - @b Ellipse: Elliptical shapes with independent x/y scaling
//! - @b Ring: Circular rings with inner and outer radius
//! - @b Line: Straight line segments with configurable thickness
//! - @b Rectangle: Rectangular shapes with sharp corners
//! - @b Rounded @b Rectangle: Rectangles with rounded corners
//!
//! @section nemagfx_vg_shape_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for shape visualization
//! - Sufficient memory for vector graphics operations
//!
//! @section nemagfx_vg_shape_usage Usage
//!
//! The application automatically renders various shapes with different
//! paint settings and displays them on screen. Each shape demonstrates
//! different vector graphics capabilities and rendering techniques.
//!
//! @note Shapes are rendered using hardware acceleration for optimal performance
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

#include "nemagfx_vg_shape.h"

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

