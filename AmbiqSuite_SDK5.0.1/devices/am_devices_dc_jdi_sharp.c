//*****************************************************************************
//
//! @file am_devices_dc_jdi_sharp.c
//!
//! @brief Driver for SHARP display panel with JDI interface (LS014B7DD01).
//!
//! @addtogroup devices_dc_jdi_sharp SHARP DC JDI Driver
//! @ingroup devices
//! @{
//!
//! Purpose: This module provides a hardware abstraction layer
//!          for SHARP display controllers with JDI interface, optimized for the
//!          LS014B7DD01 model. It enables high-resolution graphics rendering, display
//!          initialization, and region-based rendering for embedded applications
//!          requiring visual interfaces. The driver supports timing configuration,
//!          PWM-based VCOM/VA signal generation, and features for optimal
//!          display performance and system integration.
//!
//! @section devices_dc_jdi_sharp_features Key Features
//!
//! 1. @b High-resolution @b Rendering: Supports 280x280 pixel graphics.
//! 2. @b Region-based @b Rendering: Efficient partial updates.
//! 3. @b PWM-based @b VCOM/VA: Signal generation for display operation.
//! 4. @b Timing @b Configuration: Customizable display timing parameters.
//! 5. @b Power @b Management: Optimized for embedded systems.
//!
//! @section devices_dc_jdi_sharp_functionality Functionality
//!
//! - Initialize and configure SHARP JDI display
//! - Set up PWM timers for VCOM/VA signals
//! - Manage display timing and region updates
//! - Support for power-efficient operation
//!
//! @section devices_dc_jdi_sharp_usage Usage
//!
//! 1. Initialize display with am_devices_dc_jdi_sharp_init()
//! 2. Start/stop PWM timers for VCOM/VA signals
//! 3. Update display regions as needed
//!
//! @section devices_dc_jdi_sharp_configuration Configuration
//!
//! - Configure timing parameters for display
//! - Set up PWM duty cycles and frequencies
//! - Adjust region update settings
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

#include "am_devices_dc_jdi_sharp.h"
#include "am_util_delay.h"
#include "am_bsp.h"

#ifndef SIMULATION
#define DELAY am_util_delay_ms
#else
#define DELAY(...)
#endif

//
// the arguments of model type LS014B7DD01
//
#define LS014B7DD01_RESX                        280
#define LS014B7DD01_RESY                        280

#define LS014B7DD01_VCK_GCK_FF_MAX_FREQ         (0.5)       //500KHz
#define LS014B7DD01_HCK_BCK_MAX_FREQ            (0.758)     //0.758MHz

#define LS014B7DD01_XRST_INTB_DELAY             1
#define LS014B7DD01_XRST_INTB_WIDTH             566

#define LS014B7DD01_VST_GSP_DELAY               (LS014B7DD01_XRST_INTB_DELAY + 72)
#define LS014B7DD01_VST_GSP_WIDTH               576

#define LS014B7DD01_VCK_GCK_DELAY               (LS014B7DD01_XRST_INTB_DELAY + 216)
#define LS014B7DD01_VCK_GCK_WIDTH               288
#define LS014B7DD01_VCK_GCK_CLOSING_PULSES      6

#define LS014B7DD01_HST_BSP_DELAY               2
#define LS014B7DD01_HST_BSP_WIDTH               4

#define LS014B7DD01_HCK_BCK_DATA_START          1

#define LS014B7DD01_ENB_GEN_WIDTH               90
#define LS014B7DD01_ENB_GEN_DELAY               (LS014B7DD01_VCK_GCK_WIDTH - LS014B7DD01_ENB_GEN_WIDTH) / 2

#define SHARP_VA_VCOM_DUTYCYCLE     50

static uint32_t sharp_timer_number = SHARP_VA_VCOM_TIMER;
//*****************************************************************************
//
// Initialize timer that generate VA,VCOM(VB) signals
//
//*****************************************************************************
static uint32_t
dc_jdi_pwm_init(uint32_t ui32TimerNum, am_hal_timer_clock_e eTimerClock, uint32_t ui32Frequency)
{
    uint64_t ui64BaseFreq, ui64EndCounts;
    am_hal_timer_config_t TimerConfig ;
    am_hal_timer_default_config_set( &TimerConfig ) ;
    TimerConfig.eFunction = AM_HAL_TIMER_FN_PWM;
    TimerConfig.eInputClock = eTimerClock;
    //
    // Configure the TIMER.
    //
    am_hal_timer_config(ui32TimerNum, &TimerConfig);
    //
    // Config output VA pins
    //
    am_hal_timer_output_config(am_bsp_disp_jdi_timer_pins(0), AM_HAL_TIMER_OUTPUT_TMR0_OUT1 + ui32TimerNum * 2);
    //
    // Config output VCOM pins
    //
    am_hal_timer_output_config(am_bsp_disp_jdi_timer_pins(1), AM_HAL_TIMER_OUTPUT_TMR0_OUT0 + ui32TimerNum * 2);

    ui64BaseFreq = 96000000ull >> (2 * eTimerClock + 2);

    ui64EndCounts = (uint64_t)(ui64BaseFreq / ui32Frequency + 0.5f);
    TimerConfig.ui32Compare0    = (uint32_t) ui64EndCounts;
    am_hal_timer_compare0_set(ui32TimerNum, TimerConfig.ui32Compare0);

    //am_util_stdio_printf("\nDuty cycle %d,\t Frequency %d\n", SHARP_VA_VCOM_DUTYCYCLE, ui32Frequency);
    //
    // this is the duty cycle computation for PWM mode
    //
    TimerConfig.ui32Compare1 = (uint32_t)(ui64EndCounts * (1 - SHARP_VA_VCOM_DUTYCYCLE / 100.0f) + 0.5f);
    am_hal_timer_compare1_set(ui32TimerNum, TimerConfig.ui32Compare1);

    sharp_timer_number = ui32TimerNum;
    return 0;
}

//*****************************************************************************
//
// Start PWM timer for JDI interface
//
//*****************************************************************************
void
am_devices_dc_jdi_sharp_timer_start(void)
{
    //
    // Enable the TIMER.
    //
    am_hal_timer_enable(sharp_timer_number);
    //
    // Start the TIMER.
    //
    am_hal_timer_start(sharp_timer_number);
}

//*****************************************************************************
//
// Stop PWM timer for JDI interface
//
//*****************************************************************************
void
am_devices_dc_jdi_sharp_timer_stop(void)
{
    //
    // Stop the TIMER.
    //
    am_hal_timer_clear_stop(sharp_timer_number);
    //
    // Disable the TIMER.
    //
    am_hal_timer_disable(sharp_timer_number);
}
//*****************************************************************************
//
// Initialize sharp JDI interface
//
//*****************************************************************************
uint32_t
am_devices_dc_jdi_sharp_init(am_devices_dc_jdi_timer_config_t *pTimerConfig,
                             nemadc_initial_config_t *pDCConfig)
{
    dc_jdi_pwm_init(pTimerConfig->ui32TimerNum, pTimerConfig->eTimerClock, pTimerConfig->ui32Frequency);

    pDCConfig->eInterface               = DISP_INTERFACE_JDI;
    pDCConfig->ui16ResX                 = LS014B7DD01_RESX;
    pDCConfig->ui16ResY                 = LS014B7DD01_RESY;

    pDCConfig->fHCKBCKMaxFreq           = LS014B7DD01_HCK_BCK_MAX_FREQ;         //BCK maximum frequency(MHz)

    pDCConfig->ui32XRSTINTBDelay        = LS014B7DD01_XRST_INTB_DELAY;          // Delay inserted prior of XRST or INTB in multiples of format_clk
    pDCConfig->ui32XRSTINTBWidth        = LS014B7DD01_XRST_INTB_WIDTH;          // Width of High state of XRST or INTB in multiples of format_clk

    pDCConfig->ui32VSTGSPDelay          = LS014B7DD01_VST_GSP_DELAY;            // Delay inserted prior of VST or GSP in multiples of format_clk
    pDCConfig->ui32VSTGSPWidth          = LS014B7DD01_VST_GSP_WIDTH;            // Width of High state of VST or GSP in multiples of format_clk

    pDCConfig->ui32VCKGCKDelay          = LS014B7DD01_VCK_GCK_DELAY;            // Delay inserted prior of VCK or GCK in multiples of format_clk
    pDCConfig->ui32VCKGCKWidth          = LS014B7DD01_VCK_GCK_WIDTH;            // Width of High state of VCK or GCK in multiples of format_clk
    pDCConfig->ui32VCKGCKClosingPulses  = LS014B7DD01_VCK_GCK_CLOSING_PULSES;   // Number of VCK or GCK pulses without ENB or GEN signal at the end of frame

    pDCConfig->ui32HSTBSPDelay          = LS014B7DD01_HST_BSP_DELAY;            // Delay inserted prior of HST or BSP in multiples of format_clk
    pDCConfig->ui32HSTBSPWidth          = LS014B7DD01_HST_BSP_WIDTH;            // Width of High state of HST or BSP in multiples of format_clk

    pDCConfig->ui32HCKBCKDataStart      = LS014B7DD01_HCK_BCK_DATA_START;       // The HCK or BCK cycle the pixel data should start at

    pDCConfig->ui32ENBGENDelay          = LS014B7DD01_ENB_GEN_DELAY;            // Delay inserted prior of ENB or GEN in multiples of format_clk
    pDCConfig->ui32ENBGENWidth          = LS014B7DD01_ENB_GEN_WIDTH;            // Width of High state of ENB or GEN in multiples of format_clk

    pDCConfig->fVCKGCKFFMaxFreq         = LS014B7DD01_VCK_GCK_FF_MAX_FREQ;      //GCK maximum frequency(MHz)
    return 0;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
