//*****************************************************************************
//
//! @file nemagfx_extended_arcs.c
//!
//! @brief NemaGFX Extended Arc Rendering Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_extended_arcs NemaGFX Extended Arcs Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates arc rendering capabilities
//! using the NemaGFX_Extended library. The application showcases round line
//! end arcs with full alpha blending, dynamically animating the start and end
//! angles to create smooth, visually appealing transitions. The example also
//! demonstrates compositing with alpha channel images and blending factors.
//!
//! @section nemagfx_extended_arcs_features Key Features
//!
//! 1. @b Extended @b Arc @b Rendering: Draws arcs with round line ends and
//!    anti-aliasing for graphics
//!
//! 2. @b Alpha @b Blending: Demonstrates full alpha blending with RGBA images
//!    and arc primitives
//!
//! 3. @b Dynamic @b Animation: Continuously animates arc angles for real-time
//!    visual effects
//!
//! 4. @b Image @b Compositing: Overlays alpha channel images with blending
//!    options for complex UI effects
//!
//! 5. @b Hardware @b Acceleration: Utilizes NemaGFX GPU for efficient arc
//!    and image rendering
//!
//! @section nemagfx_extended_arcs_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for arc visualization
//! - Sufficient memory for frame buffers and RGBA images
//!
//! @section nemagfx_extended_arcs_usage Usage
//!
//! The application automatically renders animated arcs with round ends and
//! overlays an alpha channel image. The effect demonstrates the GPU's
//! ability to produce anti-aliased primitives and blended
//! images for modern graphical interfaces.
//!
//! @note Apollo510 devices use the MIPI DSI interface by default
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
#include "nemagfx_extended_arcs.h"
#include "ambiq_alpha_240x76_rgba.h"
#include "nemagfx_extensions.h"

//*****************************************************************************
//
// Variables Definition
//
//*****************************************************************************
static uint16_t g_ui16RESX;
static uint16_t g_ui16RESY;

static img_obj_t g_sFB;

//*****************************************************************************
//
//! @brief initialize framebuffer
//!
//! This function initialize framebuffer(s)
//!
//! @return status.
//
//*****************************************************************************
static void
init_framebuffer(const am_devices_disp_color_e eDispColor)
{
    g_sFB.w = g_ui16RESX;
    g_sFB.h = g_ui16RESY;
    if ( eDispColor == COLOR_FORMAT_RGB888 )
    {
        g_sFB.format = NEMA_RGB24;
    }
    else if ( eDispColor == COLOR_FORMAT_RGB565 )
    {
        g_sFB.format = NEMA_RGB565;
    }
    else
    {
        am_util_stdio_printf("Invalid color format!\n");
        while(1);
    }
    g_sFB.stride = g_ui16RESX * nema_format_size(g_sFB.format);
    g_sFB.bo = nema_buffer_create(g_sFB.stride * g_sFB.h);
    if ( g_sFB.bo.base_virt == (void *)NULL )
    {
        //
        // have no enough space.this check is important!
        //
        am_util_stdio_printf("Failed to create FB!\n");
        while(1);
    }
}

//*****************************************************************************
//
//! @brief nemagfx_extended_arc_round_ending_display
//!
//!
//! @return status      - None
//
//*****************************************************************************
void
nemagfx_extended_arc_round_ending_display(void)
{
    nema_cmdlist_t sCL;
    uint32_t col;
    uint32_t x = g_ui16RESX / 2;
    uint32_t y = g_ui16RESY / 2;
    uint32_t w = 20;
    uint32_t i32Radius;

    float start_angle = 30;
    float end_angle = 180;

    img_obj_t sAmbiqLogo = {{0}, 240, 76, -1, 0, NEMA_RGBA8888, 0};

    sAmbiqLogo.bo = nema_buffer_create(sizeof(ambiq_alpha_240x76_rgba));
    if ( (uintptr_t)NULL == sAmbiqLogo.bo.base_phys )
    {
        am_util_debug_printf("Fail to create the assets space\n");
        while(1);           //error
    }
    memcpy((void*)sAmbiqLogo.bo.base_phys, ambiq_alpha_240x76_rgba, sizeof(ambiq_alpha_240x76_rgba));

    while (1)
    {
        sCL = nema_cl_create();
        nema_cl_bind(&sCL);

        nema_clear(0xFFFFFFFF);

        nema_bind_dst_tex(g_sFB.bo.base_phys, g_sFB.w, g_sFB.h, g_sFB.format, g_sFB.stride);

        nema_set_clip(0, 0, g_ui16RESX, g_ui16RESY);
        nema_set_blend_fill(NEMA_BL_SIMPLE);

        start_angle += 1;
        end_angle += 1 ;

        col = nema_rgba(0xff, 0x00, 0x00, 0xFF);
        i32Radius = 80;
        nema_raster_stroked_arc_aa_ex(x, y, i32Radius, w, start_angle, 0, end_angle, 1, col);

        col = nema_rgba(0xff, 0x99, 0x00, 0xFF);
        i32Radius = 105;
        nema_raster_stroked_arc_aa_ex(x, y, i32Radius, w, start_angle, 1, end_angle, 0, col);

        col = nema_rgba(0x6d, 0x9e, 0xeb, 0xA0);
        i32Radius = 130;
        nema_raster_stroked_arc_aa_ex(x, y, i32Radius, w, start_angle, 1, end_angle, 1, col);

        nema_bind_src_tex(sAmbiqLogo.bo.base_phys,
                          sAmbiqLogo.w,
                          sAmbiqLogo.h,
                          sAmbiqLogo.format,
                          sAmbiqLogo.stride,
                          NEMA_FILTER_PS);

        nema_set_blend_blit(NEMA_BL_SIMPLE | NEMA_BLOP_MODULATE_A);
        nema_blit(104, 90);

        nema_cl_submit(&sCL);
        nema_cl_wait(&sCL);

        am_devices_display_transfer_frame(g_sFB.w,
                                          g_sFB.h,
                                          g_sFB.bo.base_phys,
                                          NULL, NULL);
        am_devices_display_wait_transfer_done();

        nema_cl_destroy(&sCL);

        nema_calculate_fps();
    }

    //
    // remember to destory when out of the loop
    //
    // nema_buffer_destroy(&g_sGreekIslandRGBA8888.bo);

}

static void am_board_para_init(am_devices_disp_color_e color_format)
{
    if ( color_format == COLOR_FORMAT_RGB565 )
    {
        g_sDispCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X12;
    }

#if AM_BSP_MULTI_DISPLAY_VERSION_SUPPORT
    if ( am_bsp_external_board_version_get() == BOARD_INFO_V2 )
    {
        g_ui16RESX = 468;
        g_ui16RESY = 468;
    }
    else
    {
        g_ui16RESX = 456;
        g_ui16RESY = 456;
    }
#else
    g_ui16RESX = 468;
    g_ui16RESY = 468;
#endif
}

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
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // color format
    //
    am_devices_disp_color_e eDispColor = COLOR_FORMAT_RGB888;

    am_board_para_init(eDispColor);

    //
    // Initialize display
    //
    am_devices_display_init(g_ui16RESX,
                            g_ui16RESY,
                            eDispColor,
                            false);

    //
    // Initialize GPU
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);

    nema_init();

    init_framebuffer(eDispColor);

#ifdef AM_DEBUG_PRINTF
    //
    // Enable debug printing to the console.
    //
    am_bsp_debug_printf_enable();
#endif


#ifdef BAREMETAL
    am_util_stdio_printf("nemafgx_extended_arcs Example\n");
    nemagfx_extended_arc_round_ending_display();
#else // BAREMETAL
    //
    // Initialize plotting interface.
    //
    am_util_debug_printf("FreeRTOS nemafgx_extended_arcs Example\n");

    //
    // Run the application.
    //
    run_tasks();
#endif // BAREMETAL

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

