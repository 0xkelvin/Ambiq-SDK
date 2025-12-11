//*****************************************************************************
//
//! @file joins_caps.c
//!
//! @brief Demo of different dash patterns in stroke path.
//!
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

#include "nemagfx_vg_test.h"

#ifdef RUN_DASH_EXAMPLE

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define RESX 452
#define RESY 452

#define NUM_COORDS 20
#define NUM_SEGS 10
#define NUM_DASH_PATTERN 4

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//! framebuffrer
nema_img_obj_t g_sFrameBuffer =
{
    {0}, RESX, RESY, RESX*3, 0, NEMA_RGB24, 0
};


//! DC layer
nemadc_layer_t g_sDCLayer =
{
    (void *)0, 0, RESX, RESY, -1, 0, 0, RESX, RESY, 0xff,
    NEMADC_BL_SRC, 0, NEMADC_RGB24, 0, 0, 0, 0, 0, 0, 0, 0
};



//! Path data
float coords[NUM_COORDS] =
{
    20.0f, 20.0f,   //first path
    120.0f, 20.0f,
    120.0f, 80.0f,
    70.0f, 120.0f,
    20.0f,  40.0f,
    140.0f, 20.0f,  //second path
    240.0f, 20.0f,
    240.0f, 80.0f,
    190.0f, 120.0f,
    140.0f, 40.0f
};

//! Path segments
uint8_t cmds[NUM_SEGS] =
{
    NEMA_VG_PRIM_MOVE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_MOVE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_LINE,
    NEMA_VG_PRIM_LINE
};

//! Dash pattern
float dash_pattern[NUM_DASH_PATTERN] =
{
    10.f, 30.f, 40.f, 20.f
};



//*****************************************************************************
//
//! @brief Create framebuffer buffer
//!
//
//*****************************************************************************
static void bufferCreate(void)
{
    //Load memory objects
    g_sFrameBuffer.bo = nema_buffer_create_pool(NEMA_MEM_POOL_FB, g_sFrameBuffer.stride*g_sFrameBuffer.h);
    nema_buffer_map(&g_sFrameBuffer.bo);
    am_util_stdio_printf("FB: V:%p P:0x%08x\n", (void *)g_sFrameBuffer.bo.base_virt, g_sFrameBuffer.bo.base_phys);

    if (g_sFrameBuffer.bo.base_virt == NULL)
    {
        am_util_stdio_printf("Frame buffer malloc failed!");
    }

    //set dc layer memory
    g_sDCLayer.baseaddr_phys = g_sFrameBuffer.bo.base_phys;
    g_sDCLayer.baseaddr_virt = g_sFrameBuffer.bo.base_virt;
}

//*****************************************************************************
//
//! @brief Destroy buffers
//!
//
//*****************************************************************************
static void bufferDestroy(void)
{
    nema_buffer_unmap(&g_sFrameBuffer.bo);
    nema_buffer_destroy(&g_sFrameBuffer.bo);
}

//*****************************************************************************
//
//! @brief run the dash line demo
//!
//
//*****************************************************************************
int draw_stroke_with_dash()
{
    // Initialize NemaGFX
    if ( nema_init() != 0 )
    {
        return -1;
    }

    //Init NemaVG
    nema_vg_init(RESX, RESY);

    //Initialize NemaDC
    if ( display_setup(RESX, RESY) != 0 )
    {
        return -2;
    }

    // Create buffer
    bufferCreate();

    // Set DC layer
    nemadc_set_layer(0, &g_sDCLayer);

    // Create command list to hold VG commands
    nema_cmdlist_t cl_vg  = nema_cl_create();

    // Create command list for clearing the screen
    nema_cmdlist_t cl_clear  = nema_cl_create();
    nema_cl_bind(&cl_clear);

    // Clear frame buffer
    nema_bind_dst_tex(g_sFrameBuffer.bo.base_phys, g_sFrameBuffer.w, g_sFrameBuffer.h, g_sFrameBuffer.format, g_sFrameBuffer.stride);
    nema_set_clip(0, 0, RESX, RESY);
    nema_clear(nema_rgba(0xff, 0xff, 0x00, 0xff)); //yellow

    // Set fill rule
    nema_vg_set_fill_rule(NEMA_VG_STROKE);

    // Create path and set it to a rhombus implicitely
    NEMA_VG_PATH_HANDLE path = nema_vg_path_create();
    nema_vg_path_set_shape(path, NUM_SEGS, cmds, NUM_COORDS, coords);

    // Create paint
    NEMA_VG_PAINT_HANDLE paint = nema_vg_paint_create();
    nema_vg_paint_set_type(paint, NEMA_VG_PAINT_COLOR);

    //set stroking
    nema_vg_stroke_set_width(10.f);
    nema_vg_stroke_set_dash_pattern(dash_pattern, NUM_DASH_PATTERN);
    nema_matrix3x3_t m_path;
    int x_off = 0;
    int y_off = 0;

    uint32_t vg_errorcode = 0;

    // Draw first line with dash pattern
    // second line with the same dash pattern and dash phase
    // third line with the same dash pattern, same dash phase and dash phase reset enabled
    // fourth line disable dashing

    for ( int i = 0; i < 4; i++ )
    {
        // clear frame buffer
        nema_cl_submit(&cl_clear);

        // Bind command list
        nema_cl_bind(&cl_vg);
        nema_cl_rewind(&cl_vg);


        switch(i)
        {
            case 0:
                nema_vg_stroke_set_dash_pattern(dash_pattern, NUM_DASH_PATTERN);
                break;
            case 1:
                nema_vg_stroke_set_dash_pattern(dash_pattern, NUM_DASH_PATTERN);
                nema_vg_stroke_set_dash_phase(20);
                break;
            case 2:
                nema_vg_stroke_set_dash_pattern(dash_pattern, NUM_DASH_PATTERN);
                nema_vg_stroke_set_dash_phase(20);
                nema_vg_stroke_reset_dash_phase(1);
                break;
            case 3:
                nema_vg_stroke_set_dash_pattern(dash_pattern, 0);
                break;
        }

        x_off = 100;
        y_off = 20;
        nema_mat3x3_load_identity(m_path);
        nema_mat3x3_translate(m_path, x_off, y_off);
        nema_vg_path_set_matrix(path, m_path);
        nema_vg_stroke_set_cap_style(NEMA_VG_CAP_ROUND, NEMA_VG_CAP_ROUND);
        nema_vg_stroke_set_join_style(NEMA_VG_JOIN_BEVEL);
        nema_vg_paint_set_paint_color(paint, nema_rgba(0xff, 0x00, 0x00, 0xff)); //red color
        nema_vg_draw_path(path, paint);

        y_off += 150;
        nema_mat3x3_load_identity(m_path);
        nema_mat3x3_translate(m_path, x_off, y_off);
        nema_vg_path_set_matrix(path, m_path);
        nema_vg_stroke_set_join_style(NEMA_VG_JOIN_MITER);
        nema_vg_stroke_set_cap_style(NEMA_VG_CAP_SQUARE, NEMA_VG_CAP_SQUARE);
        nema_vg_paint_set_paint_color(paint, nema_rgba(0x00, 0xff, 0x00, 0xff)); //green color
        nema_vg_draw_path(path, paint);

        y_off += 150;
        nema_mat3x3_load_identity(m_path);
        nema_mat3x3_translate(m_path, x_off, y_off);
        nema_vg_path_set_matrix(path, m_path);
        nema_vg_stroke_set_join_style(NEMA_VG_JOIN_ROUND);
        nema_vg_stroke_set_cap_style(NEMA_VG_CAP_BUTT, NEMA_VG_CAP_BUTT);
        nema_vg_paint_set_paint_color(paint, nema_rgba(0x00, 0x00, 0xff, 0xff)); //blue color
        nema_vg_draw_path(path, paint);

        vg_errorcode = nema_vg_get_error();
        am_util_stdio_printf("VG errorcode = 0x%08x \n", vg_errorcode);

        // Submit command list
        nema_cl_submit(&cl_vg);
        nema_cl_wait(&cl_vg);

        //Start DC
        display_refresh_start();
        //Wait DC complete interrupt.
        nemadc_wait_vsync();
        //Do follow-up operations required by hardware.
        display_refresh_end();


        am_util_delay_ms(2000);
    }

    // Destroy memory object (Deallocate memory)
    nema_vg_paint_destroy(paint);
    nema_vg_path_destroy(path);
    nema_vg_deinit();
    nema_cl_destroy(&cl_vg);
    nema_cl_destroy(&cl_clear);
    bufferDestroy();

    return 0;
}

#endif
