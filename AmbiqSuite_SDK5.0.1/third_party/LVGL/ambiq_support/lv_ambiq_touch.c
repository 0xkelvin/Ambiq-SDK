//*****************************************************************************
//
//! @file lv_ambiq_touch.c
//!
//! @brief APIs for touch feature on LVGL.
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
// Global includes for this project.
//
//*****************************************************************************
#ifdef LV_AMBIQ_TOUCH_USED

#include "lv_ambiq_touch.h"

#include "am_bsp.h"
#include "am_mcu_apollo.h"
#if defined(apollo510_evb) || defined(apollo510b_evb)
#include "am_devices_chsc5816.c"
#include "am_devices_chsc5816.h"
#elif defined(apollo4b_evb_disp_shield) || defined(apollo4p_evb_disp_shield_rev2)
#include "am_devices_tma525.c"
#include "am_devices_tma525.h"
#else
#error "not supported"
#endif

//*****************************************************************************
//
// Macro definitions
//
//*****************************************************************************

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
static void lv_ambiq_touch_handler(void *x);

#if defined(apollo510_evb) || defined(apollo510b_evb)
static am_devices_tc_chsc5816_info_t g_sTouchInfo = {0,0,0,0,0,0,0,0,0,0,0,0,0,true,0,0,0};
#elif defined(apollo4b_evb_disp_shield) || defined(apollo4p_evb_disp_shield_rev2)
static am_devices_tc_tma525_info_t g_sTouchInfo = {0,0,0,0,0,0,0,0,0,0,0,0,0,true,0,0,0};
#else
#error "not supported"
#endif

//*****************************************************************************
//
// Touch functions
//
//*****************************************************************************
static void lv_ambiq_touch_handler(void *x)
{
#if defined(apollo510_evb) || defined(apollo510b_evb)
    am_devices_chsc5816_get_point((am_devices_tc_chsc5816_info_t *)&g_sTouchInfo);                       
#elif defined(apollo4b_evb_disp_shield) || defined(apollo4p_evb_disp_shield_rev2)
    am_devices_tma525_nonblocking_get_point((am_devices_tc_tma525_info_t *)&g_sTouchInfo);
#else
#error "not supported"
#endif
}

void lv_ambiq_touch_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    /*Save the pressed coordinates and the state*/
    if(g_sTouchInfo.touch_released == true)
    {
        data->point.x = g_sTouchInfo.x0;
        data->point.y = g_sTouchInfo.y0;
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->point.x = g_sTouchInfo.x0;
        data->point.y = g_sTouchInfo.y0;
        data->state = LV_INDEV_STATE_PR;
    }
}

void lv_ambiq_touch_init(void)
{
#if defined(apollo510_evb) || defined(apollo510b_evb)
    am_devices_chsc5816_init(AM_BSP_TP_IOM_MODULE, lv_ambiq_touch_handler, NULL);
#elif defined(apollo4b_evb_disp_shield) || defined(apollo4p_evb_disp_shield_rev2)
    am_devices_tma525_init(AM_BSP_TP_IOM_MODULE, lv_ambiq_touch_handler, NULL);
#else
#error "not supported"
#endif
}

#endif