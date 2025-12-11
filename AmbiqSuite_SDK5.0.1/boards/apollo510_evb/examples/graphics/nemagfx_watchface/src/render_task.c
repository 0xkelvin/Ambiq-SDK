//*****************************************************************************
//
//! @file render_task.c
//!
//! @brief NemaGFX Watchface Rendering Task Implementation
//!
//! Purpose: This module contains the FreeRTOS task responsible for rendering
//! the watch face graphics including clock hands, background, and weather icons.
//! The task implements power-optimized rendering with GPU power cycling and
//! coordinates with the display task for frame buffer management.
//!
//! @section render_task_features Key Features
//!
//! - Real-time clock hand rendering with smooth rotation
//! - Background image and weather icon rendering
//! - GPU power management with automatic power cycling
//! - Frame buffer allocation and management
//! - Texture loading and caching optimization
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
// Global includes for this project.
//
//*****************************************************************************
#include "nemagfx_watchface.h"
#include "render_task.h"
#include "display_task.h"
#include "rtos.h"


//*****************************************************************************
//
// Asserts.
//
//*****************************************************************************
#include "0.l8.h"
#include "1.rgba.h"
#include "2.rgba.h"
#include "3.rgba.h"

#include "blowing.h"
#include "climate.h"
#include "day.h"

//*****************************************************************************
//
// Clock hands position.
//
//*****************************************************************************
#define PIVOT_HOUR_X        (-11)
#define PIVOT_HOUR_Y        (-140)

#define PIVOT_MIN_X         (-14)
#define PIVOT_MIN_Y         (-139)

#define PIVOT_SEC_X         (-9)
#define PIVOT_SEC_Y         (-148)

//*****************************************************************************
//
// Weather Icon position.
//
//*****************************************************************************
#define ICON_WIDTH  (100)
#define ICON_HEIGHT (100)

#define ICON_POS_X  ((FB_RESX - ICON_WIDTH) / 2)
#define ICON_POS_Y  (FB_RESY - ICON_HEIGHT - 50)

#ifdef GPU_FORMAT_RGB888
#define FB_COLOR_FORMAT (NEMA_RGB24)
#else
#define FB_COLOR_FORMAT (NEMA_RGB565)
#endif

//*****************************************************************************
//
// Render task handle.
//
//*****************************************************************************
TaskHandle_t RenderTaskHandle;


//*****************************************************************************
//
// Watchface assert.
//
//*****************************************************************************
static img_obj_t g_sWatchface          = {{0}, 390, 390, -1, 0, NEMA_L8, NEMA_FILTER_BL};
static img_obj_t g_sHourhand           = {{0},  22, 104, -1, 0, NEMA_RGBA8888, NEMA_FILTER_BL};
static img_obj_t g_sMinuteHand         = {{0},  28, 153, -1, 0, NEMA_RGBA8888, NEMA_FILTER_BL};
static img_obj_t g_sSecondHand         = {{0},  18, 219, -1, 0, NEMA_RGBA8888, NEMA_FILTER_BL};

const unsigned char* icon_to_draw = blowing;
const unsigned char* icon_array[3] = {blowing, climate, day};

//*****************************************************************************
//
// Frame buffer.
//
//*****************************************************************************
img_obj_t g_sFrameBuffer =
{
    .bo = {0}, .w = FB_RESX, .h = FB_RESY, .stride = -1, .color = 0, .format = FB_COLOR_FORMAT, .sampling_mode = 0,
};

//*****************************************************************************
//
//! @brief Create and allocate frame buffer in GPU memory
//!
//! @details Allocates a frame buffer in the GPU's memory pool for rendering
//! operations. The buffer is created in SSRAM for optimal performance and
//! is sized according to the specified image dimensions and format.
//!
//! @param image Pointer to the image object structure to initialize
//!
//! @return int Returns 0 on success, -1 on allocation failure
//!
//! @note This function allocates memory from the NEMA_MEM_POOL_FB pool
//!
//*****************************************************************************
int
framebuffer_create(img_obj_t* image)
{
    uint32_t size = nema_texture_size(image->format, 0, image->w, image->h);

    // Create buffer in SSRAM
    image->bo = nema_buffer_create_pool(NEMA_MEM_POOL_FB, size);
    if ( image->bo.base_virt == NULL )
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

//*****************************************************************************
//
//! @brief Load texture data from MRAM to SSRAM
//!
//! @details Copies texture or image data from MRAM (main memory) to SSRAM
//! (GPU memory) for optimal rendering performance. The function allocates
//! memory in the assets pool and performs cache-safe memory operations.
//!
//! @param image Pointer to the image object structure to populate
//! @param ptr Pointer to the source texture data in MRAM
//!
//! @return int Returns 0 on success, -1 on allocation failure
//!
//! @note This function allocates memory from the NEMA_MEM_POOL_ASSETS pool
//! and performs cache flush operations for data consistency
//!
//*****************************************************************************
int
texture_load(img_obj_t* image, const unsigned char *ptr)
{
    uint32_t size = nema_texture_size(image->format, 0, image->w, image->h);

    // Create buffer in SSRAM
    image->bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, size);
    if ( image->bo.base_virt == NULL )
    {
        return -1;
    }

    // Do memcpy and cache flush.
    nema_memcpy(image->bo.base_virt, (void *)ptr, size);
    return 0;
}


//*****************************************************************************
//
//! @brief Draw and rotate a clock hand at specified position
//!
//! @details Renders a clock hand image with rotation and translation using
//! NemaGFX matrix transformations. The function calculates the rotation matrix,
//! transforms the hand's corner points, and renders the rotated hand using
//! texture binding and quad blitting operations.
//!
//! @param img Pointer to the hand image object structure
//! @param x0 Initial x-coordinate of the hand
//! @param y0 Initial y-coordinate of the hand
//! @param angle Rotation angle in radians (negative for clockwise)
//! @param cx Center x-coordinate for rotation pivot
//! @param cy Center y-coordinate for rotation pivot
//!
//! @return None
//!
//! @note The function uses NemaGFX matrix operations for efficient rotation
//!
//*****************************************************************************
static void
draw_hand( img_obj_t *img, float x0, float y0, float angle, int cx, int cy )
{
    float x1 = x0 + img->w,  y1 = y0;
    float x2 = x0 + img->w,  y2 = y0 + img->h;
    float x3 = x0       ,  y3 = y0 + img->h;

    //
    // calculate rotation matrix
    //
    nema_matrix3x3_t m;
    nema_mat3x3_load_identity(m);
    nema_mat3x3_rotate(m, -angle);
    nema_mat3x3_translate(m, cx, cy);

    //
    // rotate points
    //
    nema_mat3x3_mul_vec(m, &x0, &y0);
    nema_mat3x3_mul_vec(m, &x1, &y1);
    nema_mat3x3_mul_vec(m, &x2, &y2);
    nema_mat3x3_mul_vec(m, &x3, &y3);

    //
    //draw hand
    //
    nema_bind_src_tex( img->bo.base_phys, img->w, img->h, img->format, img->stride, NEMA_FILTER_BL);
    nema_blit_quad_fit(x0, y0,
                       x1, y1,
                       x2, y2,
                       x3, y3);
}

//*****************************************************************************
//
//! @brief Draw the weather icon.
//
//*****************************************************************************
static void draw_vg_icon(const uint8_t * icon)
{
    //
    // Set global matrix
    //
    uint32_t svg_width;
    uint32_t svg_height;
    nema_vg_get_tsvg_resolution(icon, &svg_width, &svg_height);

    float scale_x = ICON_WIDTH  / (float)svg_width;
    float scale_y = ICON_HEIGHT / (float)svg_height;

    nema_matrix3x3_t matrix;
    nema_mat3x3_load_identity(matrix);
    nema_mat3x3_scale(matrix, scale_x, scale_y);
    nema_mat3x3_translate(matrix, ICON_POS_X, ICON_POS_Y);
    nema_vg_set_global_matrix(matrix);

    nema_vg_set_blend(NEMA_BL_SIMPLE);

    //
    // Draw icon
    //
    nema_vg_draw_tsvg(icon);

    //
    // Reset global matrix
    //
    nema_vg_reset_global_matrix();

}

//*****************************************************************************
//
//! @brief Draw the whole watchface
//!
//! This function renders the sources of the watch face into the destination frame
//! buffer with the expected rotating angle and fitting scales.
//
//*****************************************************************************
void
draw_watch(nema_img_obj_t* fb, nema_cmdlist_t* cl, float sec)
{
    float angle;

    //
    // rewind and bind the CL
    //
    nema_cl_rewind(cl);
    nema_cl_bind_circular(cl);

    //
    // bind the destination buffer
    //
    nema_bind_dst_tex(fb->bo.base_phys,
                      fb->w,
                      fb->h,
                      fb->format,
                      fb->stride);

    nema_set_clip(0, 0, fb->w, fb->h);

    //
    // bind the source buffer
    //
    nema_bind_src_tex(g_sWatchface.bo.base_phys,
                      g_sWatchface.w,
                      g_sWatchface.h,
                      g_sWatchface.format,
                      g_sWatchface.stride,
                      0);

    //
    // draw watchface background with const color
    //
    nema_set_blend_blit(NEMA_BLOP_MODULATE_RGB | NEMA_BL_SRC);
    nema_set_const_color(0x80ffa0a0);
    nema_blit_rect_fit(0, 0, fb->w, fb->h);

    //
    // draw weather icon
    //
    draw_vg_icon(icon_to_draw);

    //
    // Set clock hands blending mode.
    //
    nema_set_blend_blit(NEMA_BL_SIMPLE);

    //
    // draw hour hand
    //
    angle = sec / 3600 / 12.f * 360.f;
    draw_hand(&g_sHourhand, PIVOT_HOUR_X, PIVOT_HOUR_Y, -angle, fb->w / 2, fb->h / 2);

    //
    // draw minute hand
    //
    angle = sec / 60 / 60.f * 360.f;
    draw_hand(&g_sMinuteHand, PIVOT_MIN_X , PIVOT_MIN_Y , -angle, fb->w / 2, fb->h / 2);

    //
    // draw second hand
    //
    angle = sec / 60.f * 360.f;
    draw_hand(&g_sSecondHand, PIVOT_SEC_X, PIVOT_SEC_Y, -angle, fb->w / 2, fb->h / 2);

    //
    // submit command list
    //
    nema_cl_submit(cl);

}

void icon_change(TimerHandle_t timer)
{
    static uint32_t count = 0;

    icon_to_draw = icon_array[count];

    count++;
    if ( count >= 3 )
    {
        count = 0;
    }
}


//*****************************************************************************
//
// Render task
//
//*****************************************************************************
void
RenderTask(void *pvParameters)
{
    //
    // Power up GPU for initialization.
    //
    nemagfx_power_control(AM_HAL_SYSCTRL_WAKE, false);

    //
    // Initialize NemaSDK
    //
    nema_init();
    nema_vg_init(FB_RESX, FB_RESY);

    //
    // Power down GPU to reduce power consumption.
    //
    nemagfx_power_control(AM_HAL_SYSCTRL_DEEPSLEEP, true);

    //
    // Prepare command list
    //
    nema_cmdlist_t cl = nema_cl_create_sized(4 * 1024);

    //
    // Load texture to SSRAM
    //
    texture_load(&g_sWatchface, __0_l8);
    texture_load(&g_sHourhand, __1_rgba);
    texture_load(&g_sMinuteHand, __2_rgba);
    texture_load(&g_sSecondHand, __3_rgba);

    //
    // Prepare frame buffer
    //
    framebuffer_create(&g_sFrameBuffer);

    //
    // Create timer to change icon type
    //
    TimerHandle_t icon_timer = xTimerCreate("IconTimer", 1000, pdTRUE, NULL, icon_change);
    xTimerStart(icon_timer, 0);

    while (1)
    {
        xSemaphoreTake(g_semDisplayEnd, portMAX_DELAY);

        //
        // Power on GPU for rendering.
        //
        nemagfx_power_control(AM_HAL_SYSCTRL_WAKE, true);

        //
        // Get time.
        //
        float time = (float)xTaskGetTickCount() / configTICK_RATE_HZ;

        //
        // Render watchface
        //
        draw_watch(&g_sFrameBuffer, &cl, time);

        //
        // Wait rendering complete
        //
        nema_cl_wait(&cl);

        //
        // Power off the GPU after rendering.
        //
        nemagfx_power_control(AM_HAL_SYSCTRL_DEEPSLEEP, true);
        //
        // Release semphore
        //
        xSemaphoreGive(g_semDisplayStart);

    }
}

