//*****************************************************************************
//
//! @file nemagfx_font_render.c
//!
//! @brief NemaGFX Font Rendering and Layout Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_font_render NemaGFX Font Render Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates font rendering capabilities
//! using NemaGFX GPU acceleration. The application showcases the display
//! of fonts in multiple bit depths (1b, 2b, 4b, 8b), the impact of enabling
//! font kerning, and various text layout options supported by nema_print.
//! It also provides guidance for integrating custom fonts into projects.
//!
//! @section nemagfx_font_render_features Key Features
//!
//! 1. @b Multi-Bit @b Depth @b Fonts: Renders fonts in 1-bit, 2-bit, 4-bit, and
//!    8-bit formats to demonstrate quality and memory trade-offs
//!
//! 2. @b Font @b Kerning: Shows the visual impact of enabling/disabling kerning
//!    for improved text appearance
//!
//! 3. @b Text @b Layout: Demonstrates different text alignment and layout
//!    options available in the NemaGFX SDK
//!
//! 4. @b Custom @b Font @b Integration: Provides instructions for converting and
//!    integrating custom fonts using the provided font conversion utility
//!
//! 5. @b Hardware @b Acceleration: Utilizes GPU for efficient font rendering
//!    and layout calculations
//!
//! @section nemagfx_font_render_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for font visualization
//! - Sufficient memory for font bitmaps and frame buffers
//!
//! @section nemagfx_font_render_usage Usage
//!
//! The application automatically renders sample text using different font
//! bit depths, kerning settings, and layout options. For custom fonts,
//! refer to the how_to_use.md in third_party/ThinkSi/NemaGFX_SDK/utils/fontConvert.
//!
//! @note Debug output is available via ITM when AM_DEBUG_PRINTF is enabled
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

#include "nemagfx_font_render.h"

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************

//*****************************************************************************
//
// Function Definitions
//
//*****************************************************************************
extern void load_objects(void);
extern int32_t font_render(void);

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
    //
    // External power on
    //
    am_bsp_external_pwr_on();
    am_util_delay_ms(100);

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
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
#ifdef AM_DEBUG_PRINTF
    am_bsp_debug_printf_enable();
#endif

    //
    // Initialize display
    //
    am_devices_display_init(RESX,
                            RESY,
                            COLOR_FORMAT_RGB888,
                            false);

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // Power up GPU
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);

    //
    // Initialize NemaGFX
    //
    nema_init();

    am_util_stdio_printf("nemagfx_font_render Example\n");

    //
    // Load the fonts
    //
    load_objects();

    //
    // Display font with different settings
    //
    font_render();

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

