//*****************************************************************************
//
//! @file am_devices_dc_dbi_novatek.c
//!
//! @brief Driver for Novatek display panels with DBI interface (NT35510).
//!
//! @addtogroup devices_dc_dbi_novatek Novatek DC DBI Driver
//! @ingroup devices
//! @{
//!
//! Purpose: This module provides a hardware abstraction layer
//!          for Novatek display controllers with DBI (Display Bus Interface)
//!          support, specifically optimized for NT35510 driver IC. It enables
//!          high-resolution graphics rendering, display initialization, and
//!          region-based rendering for embedded applications requiring visual
//!          interfaces. The driver supports multiple color modes, hardware
//!          reset functionality, and features like pre-divider
//!          configuration for optimal display performance and system integration.
//!
//! @section devices_dc_dbi_novatek_features Key Features
//!
//! 1. @b DBI @b Protocol: Parallel display bus interface.
//! 2. @b NT35510 @b Support: Optimized for Novatek NT35510 IC.
//! 3. @b High-resolution @b Rendering: Supports 480x800 pixel graphics.
//! 4. @b Color @b Modes: Multiple color coding options.
//! 5. @b Hardware @b Reset: Reliable panel initialization.
//!
//! @section devices_dc_dbi_novatek_functionality Functionality
//!
//! - Initialize and configure Novatek DBI display
//! - Set display parameters and pixel format
//! - Manage region updates and hardware reset
//! - Support for pre-divider configuration
//!
//! @section devices_dc_dbi_novatek_usage Usage
//!
//! 1. Initialize display with am_devices_dc_dbi_novatek_init()
//! 2. Set region and color mode as needed
//! 3. Use hardware reset for display control
//!
//! @section devices_dc_dbi_novatek_configuration Configuration
//!
//! - Configure DBI timing and pixel format
//! - Set up region parameters
//! - Adjust color coding and pre-divider settings
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

#include "am_devices_dc_dbi_novatek.h"
#include "am_util_delay.h"
#include "am_bsp.h"
#include "am_util.h"
#include "nema_math.h"

#ifndef SIMULATION
#define DELAY am_util_delay_ms
#else
#define DELAY(...)
#endif

//
// the arguments of model type NT35510
//
#define NT35510_RESX                    480
#define NT35510_RESY                    800
#define NT35510_WRX_MAX_FREQ            (30.3)  // the periodic cycle of write is 33ns
#define NT35510_COLOR_CODING            MIPICFG_16RGB565_OPT0

#define NT35510_RDXID_MAX_FREQ          (6.25)  // the periodic cycle of ID read is 160ns
#define NT35510_RDXFM_MAX_FREQ          (2.5)   // the periodic cycle of frame read is 400ns
//*****************************************************************************
//
// Reset the display panel
//
//*****************************************************************************
void
am_devices_dc_dbi_novatek_hardware_reset(void)
{
    am_bsp_disp_reset_pins_set();
    DELAY(5);
    am_bsp_disp_reset_pins_clear();
    DELAY(20);
    am_bsp_disp_reset_pins_set();
    DELAY(150);
}

//*****************************************************************************
//
// Get PreDivider value
//
//*****************************************************************************
static inline uint32_t
get_dbi_pre_divider(void)
{
    if (nemadc_get_ipversion() >= 0x00230200U)
    {
        return nemadc_reg_read(NEMADC_REG_CLKCTRL) >> 25;
    }
    else
    {
        return nemadc_reg_read(NEMADC_REG_CLKCTRL) >> 27;
    }
}

//*****************************************************************************
//
// set PreDivider value
//
//*****************************************************************************
static void
set_dbi_pre_divider(uint32_t ui32Divider)
{
    if (nemadc_get_ipversion() >= 0x00230200U)
    {
        nemadc_reg_write(NEMADC_REG_CLKCTRL, (ui32Divider << 25) | (nemadc_reg_read(NEMADC_REG_CLKCTRL) & (0x1FFFFFF)));
    }
    else
    {
        nemadc_reg_write(NEMADC_REG_CLKCTRL, (ui32Divider << 27) | (nemadc_reg_read(NEMADC_REG_CLKCTRL) & (0x7FFFFFF)));
    }
}

//*****************************************************************************
//
// DBI write
//
//*****************************************************************************
static inline uint32_t
dc_dbi_write(uint8_t ui8Command,
             uint8_t* p_ui8Para,
             uint8_t ui8ParaLen)
{
    nemadc_mipi_cmd_write(ui8Command,
                          p_ui8Para,
                          ui8ParaLen,
                          true,
                          false);
    return 0;
}

//*****************************************************************************
//
// DBI read
//
//*****************************************************************************
static inline uint32_t
dc_dbi_read(uint8_t ui8Command,
            uint8_t* p_ui8Para,
            uint8_t ui8ParaLen,
            uint32_t* p_ui32Data,
            uint8_t ui8DataLen)
{
    uint32_t ui32PreDivider = get_dbi_pre_divider();
    if ( MIPI_read_memory_start == ui8Command )
    {
        //
        // this function couldn't read frame buffer.
        //
        //set_dbi_pre_divider((uint32_t)(ui32PreDivider * NT35510_WRX_MAX_FREQ / NT35510_RDXFM_MAX_FREQ));
    }
    else
    {
        //
        // set the right clock divider for DBI read.
        //
        set_dbi_pre_divider((uint32_t)nema_ceil(ui32PreDivider * NT35510_WRX_MAX_FREQ / NT35510_RDXID_MAX_FREQ));

        nemadc_mipi_cmd_read(ui8Command,
                             p_ui8Para,
                             ui8ParaLen,
                             p_ui32Data,
                             ui8DataLen,
                             true,
                             false);
    }
    set_dbi_pre_divider(ui32PreDivider);

    return 0;
}

//*****************************************************************************
//
// Set default parameters for DC and the panel
//
//*****************************************************************************
void
am_devices_dc_dbi_set_parameters(am_devices_dc_config_t *psDisplayPanelConfig, nemadc_initial_config_t *pDCConfig)
{
    //
    // DC configurations
    //
    pDCConfig->eInterface                   = DISP_INTERFACE_DBI;
    pDCConfig->ui16ResX                     = NT35510_RESX;
    pDCConfig->ui16ResY                     = NT35510_RESY;

    if ( pDCConfig->ui32PixelFormat == 0 )
    {
        pDCConfig->ui32PixelFormat          = NT35510_COLOR_CODING;
    }
    //
    // The WRX frequency is the half of format clock.
    //
    pDCConfig->fCLKMaxFreq                  = NT35510_WRX_MAX_FREQ * 2;

    pDCConfig->bTEEnable                    = false;

    //
    // The panel parameters
    //
    psDisplayPanelConfig->ui16ResX          = pDCConfig->ui16ResX;
    psDisplayPanelConfig->ui16ResY          = pDCConfig->ui16ResY;
    psDisplayPanelConfig->ui16MinX          = 0;
    psDisplayPanelConfig->ui16MinY          = 0;
    psDisplayPanelConfig->ui32PixelFormat   = pDCConfig->ui32PixelFormat;
    psDisplayPanelConfig->bTEEnable         = pDCConfig->bTEEnable;

}
//*****************************************************************************
//
// Initialize novatek's display driver IC NT35510
//
//*****************************************************************************
uint32_t
am_devices_dc_dbi_novatek_init(am_devices_dc_config_t *psDisplayPanelConfig)
{
    uint8_t ui8CmdBuf[4];

    //
    // Sleep out(1100h):internal display oscillator is started,and the panel scanning is started.
    //
    dc_dbi_write(MIPI_exit_sleep_mode, NULL, 0);
    DELAY(10);

    //
    // Display inversion off(2000h):recover from display inversion mode
    //
    dc_dbi_write(MIPI_exit_invert_mode, NULL, 0);
    DELAY(10);

    //
    // Normal display on(1300h):
    //
    dc_dbi_write(MIPI_enter_normal_mode, NULL, 0);
    DELAY(10);

    //
    // Memory data access control(3600h):row/column address order,row/column exchange,vertical/horizontal refresh order,
    // RGB-BGR order, and flip horizontal/vertical.
    //
    ui8CmdBuf[0] = 0x00;
    dc_dbi_write(MIPI_set_address_mode, ui8CmdBuf, 1);
    DELAY(10);

    //
    // Interface pixel format(3A00h):
    //
    ui8CmdBuf[0] = (uint8_t)(psDisplayPanelConfig->ui32PixelFormat & 0x3f);
    dc_dbi_write(MIPI_set_pixel_format, ui8CmdBuf, 1);
    DELAY(10);

    //
    // set default display area.
    //
    am_devices_dc_dbi_novatek_set_region(psDisplayPanelConfig->ui16ResX,
                                         psDisplayPanelConfig->ui16ResY,
                                         psDisplayPanelConfig->ui16MinX,
                                         psDisplayPanelConfig->ui16MinY);
    DELAY(10);

    //
    // Display on(2900h):recover from display off mode,the output from frame memory is enabled.
    //
    dc_dbi_write(MIPI_set_display_on, NULL, 0);
    DELAY(10);

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Set the visible display area
//
//*****************************************************************************
uint32_t
am_devices_dc_dbi_novatek_set_region(uint16_t ui16ResX,
                                     uint16_t ui16ResY,
                                     uint16_t ui16MinX,
                                     uint16_t ui16MinY)
{
    uint8_t ui8CmdBuf[4];
    uint16_t ui16MaxX, ui16MaxY;

    ui16MaxX = ui16MinX + ui16ResX;
    ui16MaxY = ui16MinY + ui16ResY;
    //
    // Column address set(2A00h)
    //
    ui8CmdBuf[0] = (uint8_t)(ui16MinX >> 8U);
    ui8CmdBuf[1] = (uint8_t)(ui16MinX & 0xFFU);
    ui8CmdBuf[2] = (uint8_t)(ui16MaxX >> 8U);
    ui8CmdBuf[3] = (uint8_t)(ui16MaxX & 0xFFU);
    dc_dbi_write(MIPI_set_column_address, ui8CmdBuf, 4);
    DELAY(10);

    //
    // Row address set(2B00h)
    //
    ui8CmdBuf[0] = (uint8_t)(ui16MinY >> 8U);
    ui8CmdBuf[1] = (uint8_t)(ui16MinY & 0xFFU);
    ui8CmdBuf[2] = (uint8_t)(ui16MaxY >> 8U);
    ui8CmdBuf[3] = (uint8_t)(ui16MaxY & 0xFFU);
    dc_dbi_write(MIPI_set_page_address, ui8CmdBuf, 4);
    DELAY(10);

    return AM_DEVICES_DISPLAY_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
