//*****************************************************************************
//
//! @file uart_example_timer_control.c
//!
//! @brief sets up and manages the periodic interrupt timer
//! small appends
//!
//! @addtogroup uart_stream_echo_rx_dma
//! @{
//! @defgroup uart_timer_control_rx_dma_c UART Timer Control
//! @{
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
#include "uart_example_timer_control.h"
#include "am_util.h"

#define TIMER_NUM 0
#if TIMER_NUM >= AM_REG_NUM_TIMERS
#error "Invalid Timer Number"
#endif

static pfTimerCallback_t pfTimerCallback;
void am_timer00_isr(void);

//*****************************************************************************
//
//! @brief Interrupt handler for the CTIMER
//! This provides a clock to gate the uart tx activities
//
//*****************************************************************************
void
am_timer00_isr(void)
{
    //
    // Clear Timer Interrupt.
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));
    am_hal_timer_clear(TIMER_NUM);

    if (pfTimerCallback)
    {
        pfTimerCallback(0);
    }
}


uint32_t uart_timer_init(uart_examp_timer_params_t *psTimerParams, pfTimerCallback_t pfTimerCallbacki )
{
    pfTimerCallback = pfTimerCallbacki;

    am_hal_timer_config_t sTimerConfig;

    //
    // Set up the desired TIMER.
    // The default config parameters include:
    //  eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV256
    //  eFunction = AM_HAL_TIMER_FN_EDGE
    //  Compare0 and Compare1 maxed at 0xFFFFFFFF
    //

    am_hal_timer_default_config_set(&sTimerConfig);
    //
    // Modify the default parameters.
    // Configure the timer to the requested interrupt rate
    //
    sTimerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;

    //
    // validate input, default to 2 second interrupt
    //
    if ( psTimerParams->ui32TimerPeriodMs == 0 )
    {
        psTimerParams->ui32TimerPeriodMs = 1000 * 2;        // 2 seconds default
    }

#if PRD_TIMER_FREQ == 8000000
    sTimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_XTAL_HS_DIV4;     // 8Mhz

#elif PRD_TIMER_FREQ == 375000

    sTimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV256;     // 375000
#else
    sTimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_XT;     // 32768
#endif

    uint64_t ui64Intermediate = (uint64_t) PRD_TIMER_FREQ * (uint64_t) psTimerParams->ui32TimerPeriodMs + (uint64_t)500;
    uint64_t ui64TimerVal     = ui64Intermediate / 1000;

    if (ui64TimerVal > 0x00000000FFFFFFFFuLL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    sTimerConfig.ui32Compare1 = (uint32_t) (ui64TimerVal);

    uint32_t status = am_hal_timer_config(TIMER_NUM, &sTimerConfig);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Clear the timer and its interrupt
    //
    am_hal_timer_clear(TIMER_NUM);
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));
    //
    // Clear the timer Interrupt
    //

    IRQn_Type eTimerIrq = (IRQn_Type) (TIMER0_IRQn + TIMER_NUM);
    NVIC_SetPriority(eTimerIrq, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(eTimerIrq);

    //
    // Enable the timer Interrupt.
    //
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));

    am_hal_timer_enable(TIMER_NUM);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
