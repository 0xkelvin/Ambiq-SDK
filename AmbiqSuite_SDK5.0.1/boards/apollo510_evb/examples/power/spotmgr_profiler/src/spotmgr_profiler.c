//*****************************************************************************
//
//! @file spotmgr_profiler.c
//!
//! @brief Example demonstrating the operation of spotmgr_profiler.
//!
//! @addtogroup power_examples Power Examples
//
//! @defgroup spotmgr_profiler Spot Manager Profiler Example
//! @ingroup power_examples
//! @{
//!
//! Purpose: This example demonstrates spotmgr (Spot Manager)
//! profiler functionality for power management analysis and optimization.
//! The application showcases power profiling capabilities with
//! timestamp logging, change tracking, and power state analysis.
//!
//! @section spotmgr_profiler_features Key Features
//!
//! 1. @b Power @b Profiling: Implements power management
//!    profiling for power analysis and optimization
//!
//! 2. @b Timestamp @b Logging: Provides timestamp-based logging for
//!    accurate power state change tracking and analysis
//!
//! 3. @b Change @b Tracking: Implements change tracking functionality
//!    for power state transitions and stimulus monitoring
//!
//! 4. @b Stream @b Accumulate @b Modes: Supports both stream and accumulate
//!    modes for flexible logging and analysis capabilities
//!
//! 5. @b Power @b State @b Analysis: Provides detailed power state analysis
//!    for optimization and debugging purposes
//!
//! @section spotmgr_profiler_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes spotmgr profiler with timestamp logging capabilities
//! - Implements power state change tracking and analysis
//! - Provides stream and accumulate logging modes
//! - Manages power state transition monitoring
//! - Supports power profiling functionality
//! - Implements power management analysis features
//!
//! @section spotmgr_profiler_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Ensure HAL is compiled with AM_HAL_SPOTMGR_PROFILING macro
//! 3. Monitor power state changes and profiling data
//! 4. Analyze power state transitions and stimulus events
//! 5. Optimize power management based on profiling results
//!
//! @section spotmgr_profiler_configuration Configuration
//!
//! - @b AM_HAL_SPOTMGR_PROFILING: Enable spotmgr profiling (required)
//! - @b PROFILER_STREAM_MODE: Stream or accumulate logging mode
//! - @b MAX_LOG_ENTRY: Maximum log entries for power state tracking
//! - @b PROFILER_TIMESTAMP_TIMER: Timer instance for timestamp source
//!
//! Note: Make sure that HAL is compiled with macro AM_HAL_SPOTMGR_PROFILING
//!       defined.
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

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include  <string.h>
#ifdef AM_BSP_IS_SIP
#include "am_devices_em9305.h"
#endif

#ifdef AM_HAL_SPOTMGR_PROFILING

// Timer instance to be used as timestamp source
#define PROFILER_TIMESTAMP_TIMER_INST    0

// Whether to stream or accumulate log entry
// 1: Stream mode - log entry printed whenever received
// 0: Accumulate mode (Recommended)- Only print log entry when the dump function is called
//
#define PROFILER_STREAM_MODE             0

// Timer instance to be used as wakeup source
#define DEEPSLEEP_WAKEUP_TIMER_INST      1
#define GET_TIMER_IRQ_NUM(timer_num) ((IRQn_Type)((uint32_t)TIMER0_IRQn + timer_num))

//
// Take over the interrupt handler for wakeup timer we're using.
//
#define wakeup_timer_isr                                                \
    am_timer_isr1(DEEPSLEEP_WAKEUP_TIMER_INST)
#define am_timer_isr1(n)                                                \
    am_timer_isr(n)
#if (DEEPSLEEP_WAKEUP_TIMER_INST < 10)
#define am_timer_isr(n)                                                 \
    am_timer0 ## n ## _isr
#else
#define am_timer_isr(n)                                                 \
    am_timer ## n ## _isr
#endif

// Struture of spotmanager change log with timestamp
typedef struct
{
    uint32_t timestamp;
    uint32_t info;
    am_hal_spotmgr_changelog_t chgLog;
} log_entry_t;


// Global variable
#define MAX_LOG_ENTRY 1024
log_entry_t g_sLogBuffer[MAX_LOG_ENTRY];
uint32_t g_ui32LogIdx = 0;
bool g_bLogPause = false;

//*****************************************************************************
//
// Intialize Profiler variables
//
//*****************************************************************************
void
profiler_init(void)
{
    // Initialize profiler buffer with 0xFF for empty entry detection
    memset(g_sLogBuffer, 0xFF, sizeof(g_sLogBuffer));
    // Start Logging
    g_bLogPause = false;
}

//*****************************************************************************
//
// Helper function to print a single profiler log entry
//
//*****************************************************************************
static inline void
profiler_print_entry(uint32_t ui32Idx)
{
    am_util_stdio_printf("0x%08X: 0x%08X(%2d, %2d, %2d, %d), 0x%08X, %08X\n",
                        g_sLogBuffer[ui32Idx].timestamp,
                        g_sLogBuffer[ui32Idx].chgLog.u.u32,
                        g_sLogBuffer[ui32Idx].chgLog.u.s.pwrState,
                        g_sLogBuffer[ui32Idx].chgLog.u.s.tonState,
                        g_sLogBuffer[ui32Idx].chgLog.u.s.eStimulus,
                        g_sLogBuffer[ui32Idx].chgLog.u.s.bOn,
                        g_sLogBuffer[ui32Idx].chgLog.args,
                        g_sLogBuffer[ui32Idx].info);
}

//*****************************************************************************
//
// Override weak function for spotmgr log change handler
//
//*****************************************************************************
void
am_hal_spotmgr_log_change(am_hal_spotmgr_changelog_t *pChangeLog)
{
    if (!g_bLogPause)
    {
        // dump <timestamp> pChangeLog->u.u32 pChangeLog->args
        g_sLogBuffer[g_ui32LogIdx].timestamp = am_hal_timer_read(PROFILER_TIMESTAMP_TIMER_INST);
        g_sLogBuffer[g_ui32LogIdx].chgLog = *pChangeLog;
        g_sLogBuffer[g_ui32LogIdx].info = PWRCTRL->DEVPWRSTATUS;

#if (PROFILER_STREAM_MODE == 1)
        profiler_print_entry(g_ui32LogIdx);
#endif

        if (++g_ui32LogIdx == MAX_LOG_ENTRY)
        {
            // Wrap back to index zero
            g_ui32LogIdx = 0;
        }
    }
}

//*****************************************************************************
//
// Dump all entries in the log buffer
//
//*****************************************************************************
void
profiler_dump_entries(void)
{
#if (PROFILER_STREAM_MODE == 0)
    g_bLogPause = true;
    am_bsp_debug_printf_enable();
    #endif

    bool bLoopedBack = (g_sLogBuffer[g_ui32LogIdx].timestamp != 0xFFFFFFFF);
    uint32_t ui32CurIdx = g_ui32LogIdx;

    // If the buffer has looped back, print from index ui32Idx towards the
    // end of buffer, followed by index 0 to ui32Idx.
    if (bLoopedBack)
    {
        for ( uint32_t ui32Idx = ui32CurIdx; ui32Idx < MAX_LOG_ENTRY; ui32Idx ++ )
        {
            profiler_print_entry(ui32Idx);
        }
    }
    for ( uint32_t ui32Idx = 0; ui32Idx < ui32CurIdx; ui32Idx++ )
    {
        profiler_print_entry(ui32Idx);
    }

    #if (PROFILER_STREAM_MODE == 0)
    am_bsp_debug_printf_disable();
    g_bLogPause = false;
    #endif
}


//*****************************************************************************
//
// Configure Timer used for profiling timestamp
//
//*****************************************************************************
uint32_t
profiler_timestamp_timer_start(void)
{
    uint32_t ui32RetVal;
    uint32_t ui32TimerNum = PROFILER_TIMESTAMP_TIMER_INST;
    am_hal_timer_config_t timerConfig;

    am_hal_timer_default_config_set(&timerConfig);
    timerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4K;
    timerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;

    ui32RetVal = am_hal_timer_config(ui32TimerNum, &timerConfig);
    if (ui32RetVal != AM_HAL_STATUS_SUCCESS)
    {
        return ui32RetVal;
    }

    return am_hal_timer_start(ui32TimerNum);
}

//*****************************************************************************
// Wakeup timer ISR
//*****************************************************************************
void wakeup_timer_isr(void)
{
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(DEEPSLEEP_WAKEUP_TIMER_INST, AM_HAL_TIMER_COMPARE1));
    am_hal_timer_clear(DEEPSLEEP_WAKEUP_TIMER_INST);
    am_hal_timer_stop(DEEPSLEEP_WAKEUP_TIMER_INST);
}

//*****************************************************************************
//
// Wakeup Timer Initialization
//
//*****************************************************************************
static uint32_t setup_wakeup_timer(void)
{
    am_hal_timer_config_t TimerConfig;
    uint32_t ui32RetVal;

    //
    // Configure the timer
    //
    am_hal_timer_default_config_set(&TimerConfig);
    TimerConfig.eFunction        = AM_HAL_TIMER_FN_UPCOUNT;
    TimerConfig.ui32Compare1     = 6000*200;
    ui32RetVal = am_hal_timer_config(DEEPSLEEP_WAKEUP_TIMER_INST, &TimerConfig);
    if (ui32RetVal != AM_HAL_STATUS_SUCCESS)
    {
        return ui32RetVal;
    }

    //
    // Clear the timer and its interrupt
    //
    am_hal_timer_clear(DEEPSLEEP_WAKEUP_TIMER_INST);
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(DEEPSLEEP_WAKEUP_TIMER_INST, AM_HAL_TIMER_COMPARE1));

    //
    // Enable the timer Interrupt.
    //
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(DEEPSLEEP_WAKEUP_TIMER_INST, AM_HAL_TIMER_COMPARE1));

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_SetPriority(GET_TIMER_IRQ_NUM(DEEPSLEEP_WAKEUP_TIMER_INST), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(GET_TIMER_IRQ_NUM(DEEPSLEEP_WAKEUP_TIMER_INST));
    return am_hal_interrupt_master_enable();
}
#endif

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
#ifdef AM_HAL_SPOTMGR_PROFILING
    //
    // Initialize Profiler
    //
    profiler_init();

    //
    // Start timestamp timer
    //
    if (profiler_timestamp_timer_start())
    {
       while(1);
    }

#if (PROFILER_STREAM_MODE == 1)
    //
    // Initialize the printf interface for ITM output
    //
    if (am_bsp_debug_printf_enable())
    {
        // Cannot print - so no point proceeding
        while(1);
    }
#endif

    //
    // Print the banner.
    //
    am_bsp_itm_printf_enable();
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Power Profile Example\n");
    am_bsp_itm_printf_disable();

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // If Apollo510B device, turn off EM9305 to save power
    //
#ifdef AM_BSP_IS_SIP
    am_devices_em9305_shutdown();
#endif

#ifdef AM_PART_APOLLO330P_510L
    //
    // Power off the RSS
    //
    am_hal_pwrctrl_rss_pwroff();
#endif

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Initialize Wakeup Timer
    //
    if (setup_wakeup_timer())
    {
        while(1);
    }

    //
    // Test various event's logging
    //
    am_hal_pwrctrl_temp_thresh_t sTempThresh;
    am_hal_pwrctrl_temp_update(55.0, &sTempThresh);
    am_hal_pwrctrl_temp_update(20.0, &sTempThresh);
    am_hal_pwrctrl_temp_update(-10.0, &sTempThresh);
    am_hal_pwrctrl_temp_update(-30.0, &sTempThresh);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_GFX);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_MSPI0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_MSPI0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_GFX);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_MSPI0);
    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_I2S0);
    am_hal_timer_clear(DEEPSLEEP_WAKEUP_TIMER_INST);
    am_hal_timer_start(DEEPSLEEP_WAKEUP_TIMER_INST);
    am_hal_sysctrl_sleep(true);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_I2S0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_MSPI0);
    am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HPMAX);
    am_hal_timer_clear(DEEPSLEEP_WAKEUP_TIMER_INST);
    am_hal_timer_start(DEEPSLEEP_WAKEUP_TIMER_INST);
    am_hal_sysctrl_sleep(true);
    am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER);

    #if PROFILER_STREAM_MODE == 0
    profiler_dump_entries();
    #endif

    while (1)
    {

    }
#else
    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Print warning that the example is skipped as AM_HAL_SPOTMGR_PROFILING isn't enabled
    //
    am_bsp_debug_printf_enable();
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("WARNING: AM_HAL_SPOTMGR_PROFILING not defined.\nPower Profile Example skipped.\n");
    while(1);
#endif
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
