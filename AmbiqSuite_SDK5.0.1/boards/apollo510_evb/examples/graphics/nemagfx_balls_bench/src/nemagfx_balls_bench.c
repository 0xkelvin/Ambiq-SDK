//*****************************************************************************
//
//! @file nemagfx_balls_bench.c
//!
//! @brief NemaGFX Animated Balls Performance Benchmark Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_balls_bench NemaGFX Balls Bench Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates animated balls with alpha blending
//! for performance benchmarking using NemaGFX GPU acceleration. The
//! application creates multiple randomly positioned balls that bounce
//! around the screen with realistic physics, showcasing the GPU's ability
//! to handle real-time animation and blending operations efficiently.
//!
//! @section nemagfx_balls_bench_features Key Features
//!
//! 1. @b Animated @b Balls: Renders multiple circles with independent
//!    movement, speed, and direction for dynamic visual effects
//!
//! 2. @b Alpha @b Blending: Each ball uses alpha blending for smooth
//!    visual integration and transparency effects
//!
//! 3. @b Physics @b Simulation: Implements realistic bouncing behavior
//!    when balls hit screen boundaries
//!
//! 4. @b Performance @b Benchmarking: Measures and reports FPS and
//!    rendering performance metrics
//!
//! 5. @b Configurable @b Ball @b Count: Adjust MAX_CIRCLE_NUM to test
//!    different performance scenarios
//!
//! @section nemagfx_balls_bench_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for animation visualization
//! - Sufficient memory for frame buffers and ball data
//!
//! @section nemagfx_balls_bench_usage Usage
//!
//! The application automatically animates balls bouncing around the
//! screen with alpha blending. Performance metrics including FPS are
//! reported for benchmarking purposes.
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

//*****************************************************************************
//
// This application has a large number of common include files. For
// convenience, we'll collect them all together in a single header and include
// that everywhere.
//
//*****************************************************************************
#include "nemagfx_balls_bench.h"

uint16_t g_ui16RESX;
uint16_t g_ui16RESY;

#define MAX_CIRCLE_NUM  (15)
//*****************************************************************************
//
//! define number of framebuffer,could define FRAME_BUFFERS equivalent to 2 for
//! ping-pong mode demonstate graphics performance if have enough space.
//
//*****************************************************************************
#define FRAME_BUFFERS   2

//*****************************************************************************
//
// Variables Definition
//
//*****************************************************************************

static img_obj_t        g_sFB[FRAME_BUFFERS];

//
// ball structure.
//
typedef struct
{
    int32_t i32LastX;
    int32_t i32LastY;

    int32_t i32CenterX;
    int32_t i32CenterY;

    int32_t i32Radius;

    int32_t i32SpeedX;
    int32_t i32SpeedY;

    uint8_t ui8Red;
    uint8_t ui8Green;
    uint8_t ui8Blue;
    uint8_t ui8Aplha;
}
Circle_t;

Circle_t g_sCircle[MAX_CIRCLE_NUM];

//*****************************************************************************
//
//! @brief initialize framebuffer and DC's layer if have.
//!
//! This function initialize framebuffer(s) and layer(s)
//!
//! @return status.
//
//*****************************************************************************
static void
init_framebuffer(const am_devices_disp_color_e eDispColor)
{
    for (uint8_t i = 0; i < FRAME_BUFFERS; ++i)
    {
        g_sFB[i].w = g_ui16RESX;
        g_sFB[i].h = g_ui16RESY;
        if ( eDispColor == COLOR_FORMAT_RGB888 )
        {
            g_sFB[i].format = NEMA_RGB24;
        }
        else if ( eDispColor == COLOR_FORMAT_RGB565 )
        {
            g_sFB[i].format = NEMA_RGB565;
        }
        else
        {
            am_util_stdio_printf("Invalid color format!\n");
            while(1);
        }
        g_sFB[i].stride = g_sFB[i].w * nema_format_size(g_sFB[i].format);
        g_sFB[i].bo = nema_buffer_create_pool(NEMA_MEM_POOL_FB, g_sFB[i].stride * g_sFB[i].h);
        if ( g_sFB[i].bo.base_virt == (void *)NULL )
        {
            //
            // have no enough space.this check is important!
            //
            am_util_stdio_printf("Failed to create FB!\n");
            while(1);
        }
        nema_buffer_map(&g_sFB[i].bo);

    }
}

//*****************************************************************************
//
//! @brief update balls position and speed direction
//!
//! This function make sure all the balls in panel visible region.it can bounce
//! back balls when out of bounds.
//
//*****************************************************************************
void
update_circle(void)
{
    int16_t i16CurrentY;
    int16_t i16CurrentX;
    for (uint16_t i = 0; i < MAX_CIRCLE_NUM; i++)
    {
        i16CurrentY = g_sCircle[i].i32CenterY + g_sCircle[i].i32SpeedY;
        if (i16CurrentY > g_ui16RESY - g_sCircle[i].i32Radius || i16CurrentY < g_sCircle[i].i32Radius)
        {
            if (g_sCircle[i].i32SpeedY > 0)
            {
                g_sCircle[i].i32CenterY = g_ui16RESY - g_sCircle[i].i32Radius;
                g_sCircle[i].i32SpeedY = -g_sCircle[i].i32SpeedY;
            }
            else
            {
                g_sCircle[i].i32CenterY = g_sCircle[i].i32Radius;
                g_sCircle[i].i32SpeedY = abs(g_sCircle[i].i32SpeedY);
            }
        }
        else
        {
            g_sCircle[i].i32CenterY = i16CurrentY;
        }

        i16CurrentX = g_sCircle[i].i32CenterX + g_sCircle[i].i32SpeedX;
        if (i16CurrentX > g_ui16RESX - g_sCircle[i].i32Radius || i16CurrentX < g_sCircle[i].i32Radius)
        {
            if (g_sCircle[i].i32SpeedX > 0)
            {
                g_sCircle[i].i32CenterX = g_ui16RESX - g_sCircle[i].i32Radius;
                g_sCircle[i].i32SpeedX = -g_sCircle[i].i32SpeedX;
            }
            else
            {
                g_sCircle[i].i32CenterX = g_sCircle[i].i32Radius;
                g_sCircle[i].i32SpeedX = abs(g_sCircle[i].i32SpeedX);
            }
        }
        else
        {
            g_sCircle[i].i32CenterX = i16CurrentX;
        }

        nema_fill_circle(g_sCircle[i].i32CenterX, g_sCircle[i].i32CenterY, g_sCircle[i].i32Radius,
                         nema_rgba(g_sCircle[i].ui8Red, g_sCircle[i].ui8Green, g_sCircle[i].ui8Blue, g_sCircle[i].ui8Aplha));
    }
}

//*****************************************************************************
//
//! @brief render balls and transfer frame to display.
//!
//! draw all balls move in display region,it can bounce back balls when these out of
//! bounds. the application also printing some interval time,especially fps value.
//!
//! @return status      - generic or interface specific status.
//
//*****************************************************************************
void
balls_bench(void)
{
    float start_time;
    float end_time;
    static uint8_t ui8CurrentIndex = 0;
    uint32_t ui32PrintTimer = 0;
    nema_cmdlist_t sCLCircles;

    srand(1);

    for (uint16_t i = 0; i < MAX_CIRCLE_NUM; i++)
    {
        g_sCircle[i].i32Radius = rand() % 100;
        g_sCircle[i].i32CenterX = rand() % (g_ui16RESX - g_sCircle[i].i32Radius * 2) + g_sCircle[i].i32Radius;
        g_sCircle[i].i32CenterY = rand() % (g_ui16RESY - g_sCircle[i].i32Radius * 2) + g_sCircle[i].i32Radius;

        g_sCircle[i].ui8Red = rand() % 256;
        g_sCircle[i].ui8Green = rand() % 256;
        g_sCircle[i].ui8Blue = rand() % 256;
        g_sCircle[i].ui8Aplha = rand() % 256;

        g_sCircle[i].i32SpeedX = rand() % 5 + 1;
        g_sCircle[i].i32SpeedY = rand() % 5 + 1;
    }

    sCLCircles = nema_cl_create();
    nema_cl_bind(&sCLCircles);
    while (1)
    {
        //
        // transfer frame to the display
        //
        am_devices_display_transfer_frame(g_sFB[ui8CurrentIndex].w,
                                          g_sFB[ui8CurrentIndex].h,
                                          g_sFB[ui8CurrentIndex].bo.base_phys,
                                          NULL, NULL);

        ui8CurrentIndex = (ui8CurrentIndex + 1) % FRAME_BUFFERS;

        nema_bind_dst_tex(g_sFB[ui8CurrentIndex].bo.base_phys,
                          g_sFB[ui8CurrentIndex].w,
                          g_sFB[ui8CurrentIndex].h,
                          g_sFB[ui8CurrentIndex].format,
                          g_sFB[ui8CurrentIndex].stride);
        nema_set_clip(0, 0, g_ui16RESX, g_ui16RESY);
        nema_clear(nema_rgba(0x00, 0x00, 0x00, 0xff));
        nema_set_blend_fill(NEMA_BL_SIMPLE);

        start_time = nema_get_time();
        update_circle();
        end_time = nema_get_time();
        if (++ui32PrintTimer % 100 == 0)
        {
            am_util_stdio_printf("update_circle used %f s\n", end_time - start_time);
        }

        start_time = nema_get_time();
        nema_cl_submit(&sCLCircles);
        nema_calculate_fps();
        nema_cl_wait(&sCLCircles);
        end_time = nema_get_time();
        if (ui32PrintTimer % 100 == 0)
        {
            am_util_stdio_printf("nema_cl_wait used %f s\n", end_time - start_time);
        }
        nema_cl_rewind(&sCLCircles);
        //
        // wait transfer done
        //
        am_devices_display_wait_transfer_done();
#ifndef BAREMETAL
        taskYIELD();
#endif
    }
}

static void am_board_para_init(am_devices_disp_color_e color_format)
{
    if ( color_format == COLOR_FORMAT_RGB565 )
    {
        g_sDispCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X12;
    }

    if (g_sDispCfg.eIC == DISP_IC_NT38350)
    {
        g_ui16RESX = g_sDispCfg.ui16ResX;
        g_ui16RESY = g_sDispCfg.ui16ResY;
    }
    else
    {
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
    am_util_stdio_printf("nemafgx_balls_bench Example\n");
    balls_bench();
#else // BAREMETAL
    //
    // Initialize plotting interface.
    //
    am_util_debug_printf("FreeRTOS nemafgx_balls_bench Example\n");

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
