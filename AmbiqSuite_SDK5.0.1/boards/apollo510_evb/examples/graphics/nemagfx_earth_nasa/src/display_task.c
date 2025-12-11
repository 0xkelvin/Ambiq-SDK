//*****************************************************************************
//
//! @file display_task.c
//!
//! @brief Task to handle display operations.
//!
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
// Includes.
//
//*****************************************************************************
#include "nemagfx_earth_nasa.h"
#include "rtos.h"
#include "display_task.h"
#include "render_task.h"

//*****************************************************************************
//
// Color foramt.
//
//*****************************************************************************
#ifdef GPU_FORMAT_RGB888
#define DISPLAY_COLOR_FORMAT (COLOR_FORMAT_RGB888)
#else
#define DISPLAY_COLOR_FORMAT (COLOR_FORMAT_RGB565)
#endif


//*****************************************************************************
//
// Display task handle.
//
//*****************************************************************************
TaskHandle_t DisplayTaskHandle;

//*****************************************************************************
//
// Display task handle.
//
//*****************************************************************************
TaskHandle_t DisplayTaskHandle;

SemaphoreHandle_t g_semDCStart = NULL;
SemaphoreHandle_t g_semDCEnd = NULL;

//*****************************************************************************
//
// Display task.
//
//*****************************************************************************
void
DisplayTask(void *pvParameters)
{
    //
    // Initialize Display controller and the Panel.
    //
    am_devices_display_init(RESX,
                            RESY,
                            DISPLAY_COLOR_FORMAT,
                            false);

    while (1)
    {
        //
        // Release the semphone to indicate the display is ready.
        //
        xSemaphoreGive(g_semDisplayEnd);
#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
        //
        // Power down DC/DSI
        //
        if (g_sDispCfg.eInterface == DISP_IF_DSI)
        {
            am_hal_dsi_power_control(AM_HAL_SYSCTRL_DEEPSLEEP, true);
        }
        nemadc_power_control(AM_HAL_SYSCTRL_DEEPSLEEP, true);
#endif
        xSemaphoreTake(g_semDisplayStart, portMAX_DELAY);
#if (DISP_CTRL_IP == DISP_CTRL_IP_DC)
        //
        // Power up DC/DSI
        //
        nemadc_power_control(AM_HAL_SYSCTRL_WAKE, true);
        if (g_sDispCfg.eInterface == DISP_IF_DSI)
        {
            am_hal_dsi_power_control(AM_HAL_SYSCTRL_WAKE, true);
        }
#endif
        //
        // Transfer frame to the penel
        //
        am_devices_display_transfer_frame(g_sFrameBuffer[0].w,
                                          g_sFrameBuffer[0].h,
                                          g_sFrameBuffer[0].bo.base_phys,
                                          NULL,
                                          NULL);
        //
        // wait transfer done
        //
        am_devices_display_wait_transfer_done();

    }
}
