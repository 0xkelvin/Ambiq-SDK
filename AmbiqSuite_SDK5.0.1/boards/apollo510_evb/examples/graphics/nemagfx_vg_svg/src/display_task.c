//*****************************************************************************
//
//! @file display_task.c
//!
//! @brief Task to handle DISPLAY operations.
//!
//! AM_DEBUG_PRINTF
//! If enabled, debug messages will be sent over ITM.
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
#include "nemagfx_vg_svg.h"

#include "nema_dc.h"
#include "nema_core.h"
#include "nema_math.h"
#include "nema_utils.h"
#include "nema_event.h"
#include "nema_dc_mipi.h"
//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************
#define DC_PIXEL_FORMAT COLOR_FORMAT_RGB565


//*****************************************************************************
//
// DISPLAY task handle.
//
//*****************************************************************************
TaskHandle_t DisplayTaskHandle;
uint32_t ui32MipiCfg = MIPICFG_8RGB888_OPT0; // default config

//layer
nemadc_layer_t g_sLayerOne =
{
    .startx        = 0,
    .sizex         = FB_RESX,
    .resx          = FB_RESX,
    .starty        = 0,
    .sizey         = FB_RESY,
    .resy          = FB_RESY,
    .stride        = -1,
    .format        = NEMADC_RGB565,
    .blendmode     = NEMADC_BL_SRC,
    .buscfg        = 0,
    .alpha         = 0xff,
    .flipx_en      = 0,
    .flipy_en      = 0,
    .extra_bits    = 0
};

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************

//*****************************************************************************
//
// Short Description.
//
//*****************************************************************************

void
DisplayTask(void *pvParameters)
{
    int ret;

    am_util_stdio_printf("Display task start!\n");

    //Init display system, including DC, DSI(or SPI), panel
    ret = am_devices_display_init(FB_RESX, FB_RESY, DC_PIXEL_FORMAT, false);
    if ( ret == AM_DEVICES_DISPLAY_STATUS_OUT_OF_RANGE )
    {
        //Check the dsi frequency does exceed the limit of the screen or not, if it is exceeded, set it according to the screen maximum
        #if (DC_PIXEL_FORMAT == COLOR_FORMAT_RGB565)
        if ((g_sDispCfg.eDsiFreq & 0x0f) > (AM_HAL_DSI_FREQ_TRIM_X13 & 0x0f))
        {
            g_sDispCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X13;
            am_util_stdio_printf("Warning: The dsi frequency exceeds screen limit, reset to AM_HAL_DSI_FREQ_TRIM_X13\n");
        }
        #elif (DC_PIXEL_FORMAT == COLOR_FORMAT_RGB888)
        if ((g_sDispCfg.eDsiFreq & 0x0f) >= (AM_HAL_DSI_FREQ_TRIM_X20 & 0x0f))
        {
            g_sDispCfg.eDsiFreq = AM_HAL_DSI_FREQ_TRIM_X20;
            am_util_stdio_printf("Warning: The dsi frequency exceeds screen limit, reset to AM_HAL_DSI_FREQ_TRIM_X20\n");
        }
        #endif

        //reinit
        ret = am_devices_display_init(FB_RESX,
                                      FB_RESY,
                                      DC_PIXEL_FORMAT,
                                      false);
        if (ret != 0)
        {
            am_util_stdio_printf("display init failed!\n");
            //suspend and delete this task.
            vTaskDelete(NULL);
        }
    }
    else if (ret != 0)
    {
        am_util_stdio_printf("display init failed!\n");

        //suspend and delete this task.
        vTaskDelete(NULL);
    }


    while(1)
    {
        //Wait start.
        xSemaphoreTake( g_semDCStart, portMAX_DELAY);

#ifdef USE_DEBUG_PIN_IN_GUI_TASK
        am_hal_gpio_output_clear(DEBUG_PIN_4);
        am_hal_gpio_output_clear(DEBUG_PIN_5);
#endif

        //Set the data location to be send by DC.
        g_sLayerOne.baseaddr_phys = g_pFrameBufferDC->bo.base_phys;
        g_sLayerOne.baseaddr_virt = g_pFrameBufferDC->bo.base_virt;
        nemadc_set_layer(0, &g_sLayerOne);

        //Start DC
        nemadc_transfer_frame_prepare(false);
        //
        //It's necessary to launch frame manually when TE is disabled.
        //
        nemadc_transfer_frame_launch();

#ifdef USE_DEBUG_PIN_IN_GUI_TASK
        am_hal_gpio_output_set(DEBUG_PIN_2);
#endif

        //Wait DC complete interrupt.
        nemadc_wait_vsync();

#ifdef USE_DEBUG_PIN_IN_GUI_TASK
        am_hal_gpio_output_clear(DEBUG_PIN_2);
#endif

        xSemaphoreGive(g_semDCEnd);

#ifdef USE_DEBUG_PIN_IN_GUI_TASK
        am_hal_gpio_output_set(DEBUG_PIN_4);
#endif

    }
}
