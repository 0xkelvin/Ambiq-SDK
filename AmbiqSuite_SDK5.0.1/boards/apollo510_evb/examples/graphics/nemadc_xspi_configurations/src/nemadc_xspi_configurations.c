//*****************************************************************************
//
//! @file nemadc_xspi_configurations.c
//!
//! @brief NemaDC XSPI Interface Configuration Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemadc_xspi_configurations NemaDC XSPI Configuration Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates how to configure and drive various
//! SPI-based display panels using NemaDC. The application supports SPI4,
//! DSPI (Dual-SPI), and QSPI (Quad-SPI) interfaces with configurable
//! clock polarity and phase settings. It also showcases dynamic power
//! control of the display controller for power management.
//!
//! @section nemadc_xspi_configurations_features Key Features
//!
//! 1. @b SPI4 @b Interface: 4-wire SPI with CSX, CLK, DATA, and DCX signals
//!    for standard SPI display communication
//!
//! 2. @b DSPI @b Interface: 1P1T 2-wire Dual-SPI with CSX, CLK, DATA0,
//!    and DATA1 for enhanced data throughput
//!
//! 3. @b QSPI @b Interface: Quad-SPI with CSX, CLK, and DATA0-3 for
//!    maximum data transfer rates
//!
//! 4. @b Clock @b Configuration: Configurable clock polarity and phase
//!    settings for compatibility with various display panels
//!
//! 5. @b Dynamic @b Power @b Control: Demonstrates power management of the
//!    display controller for energy efficiency
//!
//! @section nemadc_xspi_configurations_modes Operation Modes
//!
//! - @b Test @b Pattern @b Mode (TESTMODE_EN=1): Displays test patterns for
//!   interface validation and debugging
//! - @b Image @b Display @b Mode (TESTMODE_EN=0): Shows actual images for
//!   normal operation demonstration
//!
//! @section nemadc_xspi_configurations_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - SPI-compatible display panel (SPI4/DSPI/QSPI)
//! - Proper signal routing for SPI interface signals
//!
//! @section nemadc_xspi_configurations_usage Usage
//!
//! The application automatically configures the selected SPI interface
//! and displays content based on the mode setting. Use TESTMODE_EN to
//! switch between test patterns and image display modes.
//!
//! @note Layer size must be a multiple of 4 for Apollo5 compatibility
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

#include "nemadc_xspi_configurations.h"
#include "am_devices_dc_xspi_raydium.h"
#include "oli_200x200_rgba.h"

#define TESTMODE_EN 1

//
// It has the restriction that the layer size is a multiple of 4 for Apollo5.
//
#if TESTMODE_EN
    #define FB_RESX 400
    #define FB_RESY 400
#else
    #define FB_RESX 200
    #define FB_RESY 200
#endif

//*****************************************************************************
//
//! @brief Test SPI4,DSPI,QSPI interface with CLOCK polarity/phase and pixel format
//!
//! @param ui32SPIMode      SPI mode(SPI4,DSPI,QSPI and Clock polarity/phase).
//! @param i32PixelFormat   Panel pixel format.
//!
//! @return None.
//
//*****************************************************************************
void
test_MIPI_SPI(uint32_t ui32SPIMode, uint32_t ui32PixelFormat)
{
    uint16_t ui16MinX, ui16MinY;
    nemadc_initial_config_t sDCConfig;
    am_devices_dc_xspi_raydium_config_t sDisplayPanelConfig;

    am_bsp_disp_pins_enable();
    //
    // Set the display region to center
    //
    if (FB_RESX < g_sDispCfg.ui16ResX)
    {
        sDisplayPanelConfig.ui16ResX = FB_RESX;
    }
    else
    {
        sDisplayPanelConfig.ui16ResX = g_sDispCfg.ui16ResX;
    }
    ui16MinX = (g_sDispCfg.ui16ResX - sDisplayPanelConfig.ui16ResX) >> 1;
    ui16MinX = (ui16MinX >> 1) << 1;

    if (FB_RESY < g_sDispCfg.ui16ResY)
    {
        sDisplayPanelConfig.ui16ResY = FB_RESY;
    }
    else
    {
        sDisplayPanelConfig.ui16ResY = g_sDispCfg.ui16ResY;
    }
    ui16MinY = (g_sDispCfg.ui16ResY - sDisplayPanelConfig.ui16ResY) >> 1;
    ui16MinY = (ui16MinY >> 1) << 1;

    g_sDispCfg.eTEType = DISP_TE_DISABLE;
    sDCConfig.ui16ResX = sDisplayPanelConfig.ui16ResX;
    sDCConfig.ui16ResY = sDisplayPanelConfig.ui16ResY;
    sDCConfig.bTEEnable = (g_sDispCfg.eTEType == DISP_TE_DC);
    sDisplayPanelConfig.ui16MinX = ui16MinX + g_sDispCfg.ui16Offset;
    sDisplayPanelConfig.ui16MinY = ui16MinY;
    sDisplayPanelConfig.bTEEnable = (g_sDispCfg.eTEType != DISP_TE_DISABLE);
    sDisplayPanelConfig.bFlip = g_sDispCfg.bFlip;

    //
    // Initialize the display
    //
    am_devices_dc_xspi_raydium_hardware_reset();
    sDisplayPanelConfig.ui32PixelFormat = ui32PixelFormat;
    sDCConfig.ui32PixelFormat = ui32PixelFormat | ui32SPIMode;
    switch (g_sDispCfg.eInterface)
    {
        case DISP_IF_SPI4:
            sDCConfig.eInterface = DISP_INTERFACE_SPI4;
            break;
        case DISP_IF_DSPI:
            sDCConfig.eInterface = DISP_INTERFACE_DSPI;
            break;
        case DISP_IF_QSPI:
            sDCConfig.eInterface = DISP_INTERFACE_QSPI;
            break;
        default:
            ; //NOP
    }
    nemadc_configure(&sDCConfig);
    am_devices_dc_xspi_raydium_init(&sDisplayPanelConfig);

#if TESTMODE_EN
    nemadc_layer_enable(0);
    if ( 0!=(ui32SPIMode & MIPICFG_QSPI) )
    {
        uint32_t dbi_cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out(dbi_cfg | MIPICFG_SPI_HOLD);
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | CMD1_DATA4);
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_MASK_QSPI | MIPI_CMD24 |
                        (MIPI_write_memory_start << CMD_OFFSET));
        //
        // Enable frame end interrupt
        //
        nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 4);
        //
        // Send One Frame
        //
        nemadc_set_mode(NEMADC_ONE_FRAME | NEMADC_TESTMODE);
        //
        // Wait for transfer to be completed
        //
        nemadc_wait_vsync();

        nemadc_MIPI_CFG_out(dbi_cfg);
    }
    else if ( 0!=(ui32SPIMode & MIPICFG_DSPI) )
    {
        uint32_t dbi_cfg = nemadc_reg_read(NEMADC_REG_DBIB_CFG);
        nemadc_MIPI_CFG_out(dbi_cfg & (~MIPICFG_DSPI) & (~MIPICFG_QSPI));
        //
        // Start MIPI Panel Memory Write
        //
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_write_memory_start);
        nemadc_MIPI_CFG_out(((dbi_cfg & (~MIPICFG_SPI4)) | MIPICFG_SPI3)
                            | MIPICFG_SPIDC_DQSPI | MIPICFG_SPI_HOLD);
        //
        // Enable frame end interrupt
        //
        nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 4);
        //
        // Send One Frame
        //
        nemadc_set_mode(NEMADC_ONE_FRAME | NEMADC_TESTMODE);
        //
        // Wait for transfer to be completed
        //
        nemadc_wait_vsync();

        nemadc_MIPI_CFG_out(dbi_cfg);
    }
    else
    {
        //
        // Start MIPI Panel Memory Write
        //
        nemadc_MIPI_out(MIPI_DBIB_CMD | MIPI_write_memory_start);
        //
        // Enable frame end interrupt
        //
        nemadc_reg_write(NEMADC_REG_INTERRUPT, 1 << 4);
        //
        // Send One Frame
        //
        nemadc_set_mode(NEMADC_ONE_FRAME | NEMADC_TESTMODE);
        //
        // Wait for transfer to be completed
        //
        nemadc_wait_vsync();
    }

#else
    //
    // send layer 0 to display via NemaDC
    //
    nemadc_layer_t sLayer0 = {0};
    sLayer0.resx          = FB_RESX;
    sLayer0.resy          = FB_RESY;
    sLayer0.buscfg        = 0;
    sLayer0.format        = NEMADC_RGBA8888;
    sLayer0.blendmode     = NEMADC_BL_SRC;
    sLayer0.stride        = sLayer0.resx * 4;
    sLayer0.startx        = 0;
    sLayer0.starty        = 0;
    sLayer0.sizex         = sLayer0.resx;
    sLayer0.sizey         = sLayer0.resy;
    sLayer0.alpha         = 0xff;
    sLayer0.flipx_en      = 0;
    sLayer0.flipy_en      = 0;
    sLayer0.baseaddr_virt = tsi_malloc(sLayer0.resy * sLayer0.stride);
    sLayer0.baseaddr_phys = (unsigned)(sLayer0.baseaddr_virt);

    nema_memcpy((char*)sLayer0.baseaddr_virt, ui8Oli200x200RGBA, sizeof(ui8Oli200x200RGBA));

    //
    // Program NemaDC Layer0,this function includes layer enable.
    //
    nemadc_set_layer(0, &sLayer0);

    nemadc_transfer_frame_prepare(false);
    //
    //It's necessary to launch frame manually when TE is disabled.
    //
    nemadc_transfer_frame_launch();
    nemadc_wait_vsync();

    tsi_free(sLayer0.baseaddr_virt);
#endif
    nemadc_power_control(AM_HAL_SYSCTRL_DEEPSLEEP, true);

    am_util_delay_ms(1000);
    //
    // The restore registers are redundant in this example because they will be overwritten after the function nemadc_configure();
    //
    nemadc_power_control(AM_HAL_SYSCTRL_WAKE, true);
}

//*****************************************************************************
//
//! @brief demonstrate SPI4/DSPI/QSPI interfaces
//!
//! @note Please make sure the panel could support these interfaces.
//!
//! @return status.
//
//*****************************************************************************
static int
nemadc_xspi_configurations(void)
{
    nema_sys_init();
    //
    // Initialize NemaDC
    //
    if (nemadc_init() != 0)
    {
        return -2;
    }

    //
    // demonstrate QSPI configuration.
    //
    g_sDispCfg.eInterface = DISP_IF_QSPI;
    //
    // QSPI driving display with clock polarity = 0&phase = 0(default)
    //
    test_MIPI_SPI(MIPICFG_QSPI | MIPICFG_SPI4, MIPICFG_4RGB565_OPT0);
    //
    // QSPI driving display with clock polarity = 1&phase = 1
    //
    test_MIPI_SPI(MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_SPI_CPOL | MIPICFG_SPI_CPHA,
                    MIPICFG_4RGB666_OPT0);
    test_MIPI_SPI(MIPICFG_QSPI | MIPICFG_SPI4 | MIPICFG_SPI_CPOL | MIPICFG_SPI_CPHA,
                    MIPICFG_4RGB888_OPT0);

    return 0;
}

int
main(void)
{
    //
    // External power on
    //
    am_bsp_external_pwr_on();
    am_util_delay_ms(100);
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
    // Print a banner.
    //
    am_util_stdio_printf("nemadc_xspi_configurations example.\n");

    //
    // Please enable peripheral first, then executes clock control.
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_DISP);

#ifdef AM_PART_APOLLO330P_510L
    nemadc_clock_control(DISP_CLOCK_ENABLE, DISPCLKSRC_HFRC_192MHz, 2);
#else
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DISPCLKSEL_HFRC96, NULL);
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_DCCLK_ENABLE, NULL);
#endif

    nemadc_xspi_configurations();

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
