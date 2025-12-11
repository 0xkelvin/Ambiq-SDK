//*****************************************************************************
//
//! @file nemagfx_transition_effects.c
//!
//! @brief NemaGFX Transition Effects and Animation Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_transition_effects NemaGFX Transition Effects Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates transition effects between two
//! images using NemaGFX GPU acceleration. The application showcases various
//! hardware-accelerated transition animations with smooth visual effects
//! suitable for modern user interfaces.
//!
//! @section nemagfx_transition_effects_features Key Features
//!
//! 1. @b Horizontal @b Transitions: Linear, cube, inner cube, and stack effects
//!    for horizontal image transitions
//!
//! 2. @b Vertical @b Transitions: Linear, cube, inner cube, and stack effects
//!    for vertical image transitions
//!
//! 3. @b Fade @b Effects: Smooth fade and fade-zoom transitions for elegant
//!    image blending
//!
//! 4. @b Hardware @b Acceleration: GPU-accelerated transition effects for
//!    smooth, real-time animations
//!
//! 5. @b Dual @b Buffer @b Support: Implements both single and dual buffer modes
//!    for flexible rendering strategies
//!
//! @section nemagfx_transition_effects_transitions Available Transitions
//!
//! - @b NEMA_TRANS_LINEAR_H/V: Simple linear slide transitions
//! - @b NEMA_TRANS_CUBE_H/V: 3D cube rotation effects
//! - @b NEMA_TRANS_INNERCUBE_H/V: Inner cube perspective effects
//! - @b NEMA_TRANS_STACK_H/V: Stacked card transition effects
//! - @b NEMA_TRANS_FADE: Smooth cross-fade between images
//! - @b NEMA_TRANS_FADE_ZOOM: Fade with zoom effect
//! - @b NEMA_TRANS_MAX/NONE: Special transition modes
//!
//! @section nemagfx_transition_effects_configuration Configuration Options
//!
//! - @b BAREMETAL: Comment out to enable FreeRTOS support
//! - @b AM_DEBUG_PRINTF: Enable for ITM debug output
//! - @b FRAME_BUFFERS: Configure for single or dual buffer mode
//!
//! @section nemagfx_transition_effects_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for transition effect visualization
//! - Sufficient memory for dual image buffers and effects
//!
//! @section nemagfx_transition_effects_usage Usage
//!
//! The application automatically cycles through various transition effects
//! between two sample images. Each transition demonstrates different
//! animation techniques and visual effects.
//!
//! @note Transition effects are hardware-accelerated for optimal performance
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

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "nemagfx_transition_effects.h"
#include "ant_bgr24.h"
#include "beach_bgr24.h"
//*****************************************************************************
//
// Variables Definition.
//
//*****************************************************************************
static img_obj_t            g_sScreen0 = {{0}, 256, 256, -1, 0, (uint8_t)NEMA_RGB24, 0};
static img_obj_t            g_sScreen1 = {{0}, 256, 256, -1, 0, (uint8_t)NEMA_RGB24, 0};

static img_obj_t            g_sFB[FRAME_BUFFERS];
static nema_cmdlist_t       g_sCLDrawEntireScene;
static nema_transition_t    g_eEffect = NEMA_TRANS_LINEAR_H;
static nema_cmdlist_t       g_sCL;

static void
load_objects(void)
{
    for (int32_t i = 0; i < FRAME_BUFFERS; ++i)
    {
        g_sFB[i].w = RESX;
        g_sFB[i].h = RESY / SMALLFB_STRIPES;
        g_sFB[i].format = NEMA_RGB24;

        g_sFB[i].stride = nema_format_size(g_sFB[i].format) * g_sFB[i].w;

        g_sFB[i].bo = nema_buffer_create_pool(NEMA_MEM_POOL_FB, g_sFB[i].stride * g_sFB[i].h);

        (void)nema_buffer_map(&g_sFB[i].bo);

    }

    //
    // Preload the textures from MRAM to SSRAM
    // Set the textures address in SSRAM for GPU
    //

    g_sScreen0.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, sizeof(g_ui8Ant_bgr24));
    nema_memcpy((void*)g_sScreen0.bo.base_phys, g_ui8Ant_bgr24, sizeof(g_ui8Ant_bgr24));
    g_sScreen1.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, sizeof(g_ui8Beach_bgr24));
    nema_memcpy((void*)g_sScreen1.bo.base_phys, g_ui8Beach_bgr24, sizeof(g_ui8Beach_bgr24));

}

//*****************************************************************************
//
//! @brief render frame buffers and display transition effects.
//!
//! @param  step            - transition effects step.
//!
//! this function could render and display concurrently through Dual-buffers,
//! or render and display serially through single-buffer.
//
//*****************************************************************************
static void
display(float step)
{
    static bool is_first_frame = true;
    uint8_t ui8CurrentIndex = 0;
    for (int32_t stripe = 0; stripe < SMALLFB_STRIPES; ++stripe)
    {
        nema_cl_bind(&g_sCLDrawEntireScene);
        nema_cl_rewind(&g_sCLDrawEntireScene);
        nema_bind_tex(NEMA_TEX1,
                  g_sScreen0.bo.base_phys + stripe * g_sScreen0.bo.size / SMALLFB_STRIPES,
                  g_sScreen0.w,
                  g_sScreen0.h,
                  g_sScreen0.format,
                  g_sScreen0.stride,
                  NEMA_FILTER_BL | NEMA_TEX_BORDER);

        nema_bind_tex(NEMA_TEX2,
                  g_sScreen1.bo.base_phys + stripe * g_sScreen1.bo.size / SMALLFB_STRIPES,
                  g_sScreen1.w,
                  g_sScreen1.h,
                  g_sScreen1.format,
                  g_sScreen1.stride,
                  NEMA_FILTER_BL | NEMA_TEX_BORDER);

        nema_set_tex_color(0);

        nema_transition(g_eEffect,
                        NEMA_TEX1,
                        NEMA_TEX2,
                        NEMA_BL_SRC,
                        step,
                        RESX,
                        RESY / SMALLFB_STRIPES);

        nema_cl_bind(&g_sCL);
        nema_cl_rewind(&g_sCL);
        //
        // Bind Framebuffer
        //
        nema_bind_dst_tex(g_sFB[ui8CurrentIndex].bo.base_phys,
                          g_sFB[ui8CurrentIndex].w,
                          g_sFB[ui8CurrentIndex].h,
                          g_sFB[ui8CurrentIndex].format,
                          g_sFB[ui8CurrentIndex].stride);

        //
        // Set Clipping Rectangle
        //
        nema_set_clip(0,
                      0,
                      RESX,
                      RESY / SMALLFB_STRIPES);

        nema_cl_jump(&g_sCLDrawEntireScene);

        //
        // make sure display transfer is completed before submit next cl.
        //
        if ( is_first_frame == false )
        {
            am_devices_display_wait_transfer_done();
        }
        nema_cl_submit(&g_sCL);
#if (FRAME_BUFFERS == 1)
        //
        //wait GPU render completed before transfer frame when using singlebuffer.
        //
        nema_cl_wait(&g_sCL);
#endif
#if (SMALLFB_STRIPES != 1)
        //
        // Reposition the display area if necessary
        //
        am_devices_display_set_region(RESX,
                                      RESY / SMALLFB_STRIPES,
                                      PANEL_OFFSET,
                                      PANEL_OFFSET + RESY / SMALLFB_STRIPES * stripe);
#endif
        //
        // transfer frame to the display
        //
        am_devices_display_transfer_frame(g_sFB[ui8CurrentIndex].w,
                                          g_sFB[ui8CurrentIndex].h,
                                          g_sFB[ui8CurrentIndex].bo.base_phys,
                                          NULL,
                                          NULL);
        is_first_frame = false;
        ui8CurrentIndex = (ui8CurrentIndex + 1) % FRAME_BUFFERS;
#if (FRAME_BUFFERS > 1)
        //
        //wait GPU render completed after transfer frame when using dualbuffers.
        //
        nema_cl_wait(&g_sCL);
#endif

    }
}

//*****************************************************************************
//
//! @brief change transition effects.
//!
//! this function used to change transition effects,when user defined macro
//! SMALLFB_STRIPES isn't equivalent to 1.please note that some transition effects
//! could be not suit for small frame buffer.
//
//*****************************************************************************
static void
next_effect(void)
{
#if (SMALLFB_STRIPES != 1)
    if ( g_eEffect == NEMA_TRANS_LINEAR_H )
    {
        g_eEffect = NEMA_TRANS_LINEAR_V;
    }
    else
    {
        g_eEffect = NEMA_TRANS_LINEAR_H;
    }
#else
    g_eEffect = (nema_transition_t)(((int32_t)g_eEffect + 1) % NEMA_TRANS_MAX);
#endif
}

void
transition_effects(void)
{
    float step = MIN_STEP;
    float step_step = ANIMATION_STEP_0_1;

    load_objects();

    g_sCLDrawEntireScene  = nema_cl_create();
    g_sCL  = nema_cl_create();

    while (1)
    {
        display(step);

        if (step <= MIN_STEP)
        {
            step = MIN_STEP;
            step_step = ANIMATION_STEP_0_1;
            next_effect();
        }
        else if (step >= MAX_STEP)
        {
            step = MAX_STEP;
            step_step = -ANIMATION_STEP_0_1;
            next_effect();
        }

        step += step_step;

        nema_calculate_fps();
    }
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


#ifdef BAREMETAL
    //
    // Show the transition effect.
    //
    transition_effects();
#else //!< BAREMETAL
    //
    // Initialize plotting interface.
    //
    am_util_debug_printf("FreeRTOS Nemagfx Transition Effect Example\n");

    //
    // Run the application.
    //
    run_tasks();
#endif //!< BAREMETAL

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

