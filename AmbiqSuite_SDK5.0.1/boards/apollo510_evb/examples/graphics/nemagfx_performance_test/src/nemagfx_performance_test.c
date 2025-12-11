//*****************************************************************************
//
//! @file nemagfx_performance_test.c
//!
//! @brief NemaGFX Performance Testing Suite
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_performance_test NemaGFX Performance Test Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example provides GPU performance testing for
//! various NemaGFX operations including fill, copy, blend, scale, rotate,
//! scale+rotate, perspective transformation, SVG image rendering, and TTF
//! font rendering. The test suite generates detailed performance metrics
//! for optimization and benchmarking purposes.
//!
//! @section nemagfx_performance_test_features Key Features
//!
//! 1. @b Testing: Covers all major GPU operations including
//!    basic graphics, transformations, and rendering
//!
//! 2. @b Configurable @b Test @b Suites: Multiple configuration files for different
//!    performance scenarios and optimization targets
//!
//! 3. @b Detailed @b Metrics: Provides precise performance measurements for
//!    each operation with timing and throughput data
//!
//! 4. @b CSV @b Output: Generates CSV-formatted results for easy analysis
//!    in spreadsheet applications
//!
//! 5. @b PSRAM @b Integration: Tests GPU performance with external memory
//!    for realistic system performance evaluation
//!
//! @section nemagfx_performance_test_configurations Test Configurations
//!
//! - @b config_base.h: Complete test suite with all operations
//! - @b config_morton.h: Rotation performance with Morton/tiling settings
//! - @b config_rotate.h: Rotation performance with various parameters
//! - @b config_scale.h: Scaling performance optimization
//! - @b config_scale_rotate.h: Affine transformation performance
//! - @b config_write_param.h: GPU PSRAM write performance
//!
//! @section nemagfx_performance_test_usage Usage Instructions
//!
//! 1. Select desired test configuration in test_common.c (lines 54-59)
//! 2. Compile and download to target device
//! 3. Collect SWO output during test execution
//! 4. Save results as CSV file and import to Excel for analysis
//!
//! @section nemagfx_performance_test_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - PSRAM for realistic memory performance testing
//! - Display panel for visual verification
//! - SWO interface for performance data collection
//!
//! @note PSRAM must be connected before running performance tests
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

#include "nemagfx_performance_test.h"
#include "display.h"

#ifndef MSPI_PSRAM_TIMING_CHECK
#define MSPI_PSRAM_TIMING_CHECK
#endif

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


extern int32_t nemagfx_performance_test(void);

//*****************************************************************************
//
// Main function
//
//*****************************************************************************
int
main(void)
{
    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Enable cache.
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

    //
    // Initialize the printf interface for ITM output
    //
    am_bsp_debug_printf_enable();

    //
    // External power on
    //
    am_bsp_external_pwr_on();
    am_util_delay_ms(100);

#ifdef CPU_RUN_IN_HP_MODE
    //
    //CPU switch to HP mode.
    //
#ifdef AM_PART_APOLLO330P_510L
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("CPU Enter HP mode failed!\n");
    }
#else
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("CPU Enter HP mode failed!\n");
    }
#endif
#endif

    //
    //CPU status display.
    //
    am_hal_pwrctrl_mcu_mode_e cpu_current_status;
    am_hal_pwrctrl_mcu_mode_status(&cpu_current_status);
#ifdef AM_PART_APOLLO330P_510L
    if ( AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2 != cpu_current_status )
    {
        am_util_stdio_printf("CPU Operate in Normal Mode\n");
    }
#else
    if ( AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE != cpu_current_status )
    {
        am_util_stdio_printf("CPU Operate in Normal Mode\n");
    }
#endif
    else
    {
        am_util_stdio_printf("CPU Operate in High Performance Mode\n");
    }

    //
    // Run MSPI DDR timing scan
    //
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
    uint32_t ui32Status = mspi_device_func.mspi_init(MSPI_PSRAM_MODULE, &g_sMspiPsramConfig, &g_pPsramHandle, &g_pMSPIPsramHandle);
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
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

    //
    // Init GPIO for time measure
    //
#ifdef PIN_MEASURE
    am_hal_gpio_pinconfig(PIN_MEASURE, am_hal_gpio_pincfg_output);
#endif

#ifndef AM_PART_APOLLO330P_510L
#ifdef GPU_RUN_IN_HP_MODE
    //
    //GPU switch to HP mode.
    //
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_GFX);
    if ( am_hal_pwrctrl_gpu_mode_select(AM_HAL_PWRCTRL_GPU_MODE_HIGH_PERFORMANCE)  != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("GPU Enter HP mode failed!\n");
    }
#endif

    //
    //GPU status display.
    //
    am_hal_pwrctrl_gpu_mode_e gpu_current_mode;
    am_hal_pwrctrl_gpu_mode_status(&gpu_current_mode);
    if ( AM_HAL_PWRCTRL_GPU_MODE_HIGH_PERFORMANCE != gpu_current_mode )
    {
        am_util_stdio_printf("GPU Operate in Normal Mode\n");
    }
    else
    {
        am_util_stdio_printf("GPU Operate in High Performance Mode\n");
    }
#else
    am_util_stdio_printf("GPU Operate in Normal Mode\n");
#endif

    //
    //Power on GPU
    //
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);

    //
    // Initialize NemaGFX
    //
    nema_init();

    //
    // Set GPU WCACHE timeout setting
    // Bit 31: When set to 1, write cache is enabled.
    // Bit 30-16: Timeout for the lines of the WCache.
    //
#ifdef CHANGE_WCACHE_TIMEOUT
    nema_reg_write(0xe4, 0x80640007);
#endif

    //
    // Run the performace test
    //
    nemagfx_performance_test();

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

