//*****************************************************************************
//
//! @file nemagfx_vg_test.c
//!
//! @brief NemaGFX Vector Graphics Test Suite
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_vg_test NemaGFX Vector Graphics Test Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example provides a test suite for all built-in
//! NemaGFX Vector Graphics (VG) demos. The application includes multiple
//! demonstration modules that showcase various VG capabilities including
//! masking, painting, font rendering, shapes, transformations, and SVG operations.
//!
//! @section nemagfx_vg_test_demos Available Demos
//!
//! 1. @b Masking @b Example (RUN_MASKING_EXAMPLE): Demonstrates masking features
//!    with NemaVG for complex clipping and overlay operations
//!
//! 2. @b Paint @b Example (RUN_PAINT_EXAMPLE): Shows different paint features
//!    including gradients, patterns, and color fills
//!
//! 3. @b Paint @b LUT @b Example (RUN_PAINT_LUT_EXAMPLE): Demonstrates LUT format
//!    texture usage in VG paint objects for color mapping
//!
//! 4. @b Font @b Rendering (RUN_RENDER_VG_FONT): Shows TTF font rendering
//!    capabilities with NemaVG vector graphics
//!
//! 5. @b Shapes (RUN_SHAPE): Displays various pre-defined shapes with
//!    different paint settings and transformations
//!
//! 6. @b Text @b Transformation (RUN_TEXT_TRANSFORMATION): Demonstrates text
//!    object manipulation using different transform matrices
//!
//! 7. @b TSVG @b Benchmark (RUN_TSVG_BENCHMARK): Performance benchmark using
//!    rotating tiger head SVG image
//!
//! 8. @b TSVG @b Render (RUN_TSVG_RENDER_EXAMPLE): Renders SVG images containing
//!    both shapes and fonts
//!
//! 9. @b TSVG @b Measure (RUN_TSVG_MEASURE): SVG measurement and analysis tools
//!
//! 10. @b Raster @b Arc (RUN_RASTER_ARC): Arc rendering and rasterization
//!
//! 11. @b Raw @b TTF (RUN_RENDER_RAW_TTF): Raw True Type Font rendering
//!
//! 12. @b Joins @b and @b Caps (RUN_JOINS_CAPS): Line join and cap style rendering
//!
//! 13. @b Dash @b Example (RUN_DASH_EXAMPLE): Stroke dash pattern rendering
//!
//! @section nemagfx_vg_test_configuration Configuration
//!
//! Modify macro definitions in nemagfx_vg_test.h to select the desired demo.
//! Only one demo can be enabled at a time for proper operation.
//!
//! @section nemagfx_vg_test_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - PSRAM for graphics buffer management
//! - Display panel for VG demo visualization
//!
//! @section nemagfx_vg_test_usage Usage
//!
//! Configure the desired demo in the header file, then run the application
//! to see the selected vector graphics demonstration in action.
//!
//! @note Only one demo macro can be enabled at a time
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

#include "nemagfx_vg_test.h"
#include "nemagfx_buffer_customize.h"

#define MSPI_PSRAM_TIMING_CHECK

#if defined(apollo510_evb) || defined(apollo510b_evb)
#include "am_devices_mspi_psram_aps25616ba_1p2v.h"
#include "am_devices_mspi_psram_aps25616ba_1p2v.c"
#else
#include "am_devices_mspi_psram_aps25616n.h"
#include "am_devices_mspi_psram_aps25616n.c"
#endif

//*****************************************************************************
//
// Global type definitions.
//
//*****************************************************************************
typedef struct
{
    uint8_t  devName[30];

    uint32_t (*mspi_init)(uint32_t ui32Module,
                          am_devices_mspi_psram_config_t *psMSPISettings,
                          void **ppHandle,
                          void **ppMspiHandle);

    uint32_t (*mspi_term)(void *pHandle);

    uint32_t (*mspi_read_id)(void *pHandle);

    uint32_t (*mspi_read)(void *pHandle, uint8_t *pui8RxBuffer,
                          uint32_t ui32ReadAddress,
                          uint32_t ui32NumBytes,
                          bool bWaitForCompletion);

    uint32_t (*mspi_read_adv)(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           uint32_t ui32PauseCondition,
                           uint32_t ui32StatusSetClr,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);

    uint32_t (*mspi_read_callback)(void *pHandle, uint8_t *pui8RxBuffer,
                                   uint32_t ui32ReadAddress,
                                   uint32_t ui32NumBytes);

    uint32_t (*mspi_write)(void *pHandle, uint8_t *ui8TxBuffer,
                           uint32_t ui32WriteAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion);

    uint32_t (*mspi_write_adv)(void *pHandle,
                               uint8_t *puiTxBuffer,
                               uint32_t ui32WriteAddress,
                               uint32_t ui32NumBytes,
                               uint32_t ui32PauseCondition,
                               uint32_t ui32StatusSetClr,
                               am_hal_mspi_callback_t pfnCallback,
                               void *pCallbackCtxt);

    uint32_t (*mspi_mass_erase)(void *pHandle);
    uint32_t (*mspi_sector_erase)(void *pHandle, uint32_t ui32SectorAddress);
    uint32_t (*mspi_xip_enable)(void *pHandle);
    uint32_t (*mspi_xip_disable)(void *pHandle);
    uint32_t (*mspi_scrambling_enable)(void *pHandle);
    uint32_t (*mspi_scrambling_disable)(void *pHandle);

    uint32_t (*mspi_init_timing_check)(uint32_t ui32Module,
                                       am_devices_mspi_psram_config_t *pDevCfg,
                                       am_devices_mspi_psram_ddr_timing_config_t *pDevSdrCfg);

    uint32_t (*mspi_init_timing_apply)(void *pHandle,
                                       am_devices_mspi_psram_ddr_timing_config_t *pDevSdrCfg);
} mspi_device_func_t;

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
//static uint32_t        ui32DMATCBBuffer[2560];
void            *g_pPsramHandle;
void            *g_pMSPIPsramHandle;

am_devices_mspi_psram_config_t g_sMspiPsramConfig =
{
    .eDeviceConfig            = AM_HAL_MSPI_FLASH_HEX_DDR_CE0,
    .eClockFreq               = AM_HAL_MSPI_CLK_192MHZ,
    .ui32NBTxnBufLength       = 0,
    .pNBTxnBuf                = NULL,
    .ui32ScramblingStartAddr  = 0,
    .ui32ScramblingEndAddr    = 0,
};

//! MSPI interrupts.
static const IRQn_Type MspiInterrupts[] =
{
    MSPI0_IRQn,
    MSPI1_IRQn,
    MSPI2_IRQn,
};

//
// Take over the interrupt handler for whichever MSPI we're using.
//
#define psram_mspi_isr                                                          \
    am_mspi_isr1(MSPI_PSRAM_MODULE)
#define am_mspi_isr1(n)                                                        \
    am_mspi_isr(n)
#define am_mspi_isr(n)                                                         \
    am_mspi ## n ## _isr

//*****************************************************************************
//
// MSPI ISRs.
//
//*****************************************************************************
void psram_mspi_isr(void)
{
   uint32_t      ui32Status;

   am_hal_mspi_interrupt_status_get(g_pMSPIPsramHandle, &ui32Status, false);

   am_hal_mspi_interrupt_clear(g_pMSPIPsramHandle, ui32Status);

   am_hal_mspi_interrupt_service(g_pMSPIPsramHandle, ui32Status);

}

#ifdef MSPI_PSRAM_TIMING_CHECK
am_devices_mspi_psram_ddr_timing_config_t MSPIDdrTimingConfig;
#endif

mspi_device_func_t mspi_device_func =
{
#if defined(apollo510_evb) || defined(apollo510b_evb)
    .devName = "MSPI PSRAM APS25616BA",
    .mspi_init = am_devices_mspi_psram_aps25616ba_ddr_init,
    .mspi_init_timing_check = am_devices_mspi_psram_aps25616ba_ddr_init_timing_check,
    .mspi_xip_enable = am_devices_mspi_psram_aps25616ba_ddr_enable_xip,
#else
    .devName = "MSPI PSRAM APS25616N",
    .mspi_init = am_devices_mspi_psram_aps25616n_ddr_init,
    .mspi_init_timing_check = am_devices_mspi_psram_aps25616n_ddr_init_timing_check,
    .mspi_xip_enable = am_devices_mspi_psram_aps25616n_ddr_enable_xip,
#endif
};

//*****************************************************************************
//
// Main Function
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Status;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#ifdef DEBUG_PIN
    //debug pin setting
    am_hal_gpio_pinconfig(DEBUG_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(DEBUG_PIN);
#endif

#ifdef NEMA_WAIT_CL_PIN_1
    am_hal_gpio_pinconfig(NEMA_WAIT_CL_PIN_1, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(NEMA_WAIT_CL_PIN_1);
#endif

#ifdef NEMA_WAIT_CL_PIN_2
    am_hal_gpio_pinconfig(NEMA_WAIT_CL_PIN_2, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(NEMA_WAIT_CL_PIN_2);
#endif

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Enable global IRQ.
    //
    am_hal_interrupt_master_enable();

#ifdef MSPI_PSRAM_TIMING_CHECK
    am_util_stdio_printf("Starting MSPI DDR Timing Scan: \n");
    if ( AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS == mspi_device_func.mspi_init_timing_check(MSPI_PSRAM_MODULE, &g_sMspiPsramConfig, &MSPIDdrTimingConfig) )
    {
#if defined(apollo510_evb) || defined(apollo510b_evb)
        am_util_stdio_printf("==== Scan Result: RXDQSDELAY0 = %d \n", MSPIDdrTimingConfig.sTimingCfg.ui8RxDQSDelay);
#else
        am_util_stdio_printf("==== Scan Result: RXDQSDELAY0 = %d \n", MSPIDdrTimingConfig.ui32Rxdqsdelay);
#endif
    }
    else
    {
        am_util_stdio_printf("==== Scan Result: Failed, no valid setting.  \n");
        while(1);
    }
#endif

    //
    // Configure the MSPI and PSRAM Device.
    //
    ui32Status = mspi_device_func.mspi_init(MSPI_PSRAM_MODULE, &g_sMspiPsramConfig, &g_pPsramHandle, &g_pMSPIPsramHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to configure the MSPI and PSRAM Device correctly!\n");
        while(1);
    }

    NVIC_SetPriority(MspiInterrupts[MSPI_PSRAM_MODULE], 5);
    NVIC_EnableIRQ(MspiInterrupts[MSPI_PSRAM_MODULE]);

    //
    // Enable XIP mode.
    //
    ui32Status = mspi_device_func.mspi_xip_enable(g_pPsramHandle);
    if (AM_DEVICES_MSPI_PSRAM_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to enable XIP mode in the MSPI!\n");
    }

    //
    // Initialize plotting interface.
    //
    am_util_stdio_printf("nemagfx_vg_test Example\n");

#ifdef CPU_RUN_IN_HP_MODE
    //
    //CPU switch to HP mode.
    //
#ifdef AM_PART_APOLLO330P_510L
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("HP_LP:Enter HP mode failed!\n");
    }
#else
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("HP_LP:Enter HP mode failed!\n");
    }
#endif
#endif

#ifndef AM_PART_APOLLO330P_510L
#ifdef GPU_RUN_IN_HP_MODE
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_GFX);

    //
    //GPU switch to HP mode.
    //
    am_hal_pwrctrl_gpu_mode_e current_mode;
    am_hal_pwrctrl_gpu_mode_select(AM_HAL_PWRCTRL_GPU_MODE_HIGH_PERFORMANCE);
    am_hal_pwrctrl_gpu_mode_status(&current_mode);
    if ( AM_HAL_PWRCTRL_GPU_MODE_HIGH_PERFORMANCE != current_mode )
    {
        am_util_stdio_printf("gpu switch to HP mode failed!\n");
    }
#endif
#endif

    //
    // Power on GPU
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);

    //
    // Customize graphics .
    //
#ifdef NEMA_USE_CUSTOM_MALLOC
    graphic_heap_init();
#endif

    //
    // Run different VG test item, please change the macro define in the nemagfx_vg_test.h
    //
#if defined(RUN_MASKING_EXAMPLE)
    extern int masking_example(void);
    masking_example();
#elif defined(RUN_PAINT_EXAMPLE)
    extern int paint_example(void);
    paint_example();
#elif defined(RUN_PAINT_LUT_EXAMPLE)
    extern int paint_lut_example(void);
    paint_lut_example();
#elif defined(RUN_RENDER_VG_FONT)
    extern int render_vg_font(void);
    render_vg_font();
#elif defined(RUN_SHAPE)
    extern int shapes(void);
    shapes();
#elif defined(RUN_TEXT_TRANSFORMATION)
    extern int text_transformation(void);
    text_transformation();
#elif defined(RUN_TSVG_BENCHMARK)
    extern int tsvg_benchmark(void);
    tsvg_benchmark();
#elif defined(RUN_TSVG_RENDER_EXAMPLE)
    extern int tsvg_render_example(void);
    tsvg_render_example();
#elif defined(RUN_TSVG_MEASURE)
    extern int tsvg_measure(void);
    tsvg_measure();
#elif defined(RUN_RASTER_ARC)
    extern int raster_arc(void);
    raster_arc();
#elif defined(RUN_RENDER_RAW_TTF)
    extern int render_raw_ttf(void);
    render_raw_ttf();
#elif defined(RUN_JOINS_CAPS)
    extern int render_joins_caps(void);
    render_joins_caps();
#elif defined(RUN_DASH_EXAMPLE)
    extern int draw_stroke_with_dash(void);
    draw_stroke_with_dash();
#else
    #error "Not supported selection!"
#endif

    while(1)
    {
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

