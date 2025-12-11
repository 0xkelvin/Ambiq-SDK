//*****************************************************************************
//
//! @file uart_example_timer_control.c
//!
//! @brief sets up and manages the periodic interrupt timer
//! small appends
//!
//! @ingroup uart_fullduplex_common
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

//*****************************************************************************
//
//! @brief Timer Config struct
//
//*****************************************************************************
am_hal_timer_config_t TimerCfg =
{
    .eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16,
    .eFunction = AM_HAL_TIMER_FN_UPCOUNT,
    .ui32Compare0 = 0xFFFFFFFF,
    .ui32Compare1 = 0xFFFFFFFF,
    .bInvertOutput0 = false,
    .bInvertOutput1 = false,
    .eTriggerType = AM_HAL_TIMER_TRIGGER_DIS,
    .eTriggerSource = AM_HAL_TIMER_TRIGGER_TMR0_OUT1,
    .ui32PatternLimit = 0,
};

//*****************************************************************************
//
// starts timer with a timeout
//
//*****************************************************************************
uint32_t uart_timer_init(uint32_t ui32DelayUs )
{

#if PRD_TIMER_FREQ == 8000000
    sTimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_XTAL_HS_DIV4;     // 8Mhz
#elif PRD_TIMER_FREQ == 375000
    TimerCfg.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV256;     // 375000
#else
    sTimerConfig.eInputClock = AM_HAL_TIMER_CLOCK_XT;     // 32768
#endif

    uint64_t ui64Intermediate = (uint64_t) PRD_TIMER_FREQ * (uint64_t) ui32DelayUs + (uint64_t)500000;
    uint64_t ui64TimerVal     = ui64Intermediate / 1000000;

    if (ui64TimerVal > 0x00000000FFFFFFFFuLL )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    TimerCfg.ui32Compare1 = (uint32_t) (ui64TimerVal);

    uint32_t status = am_hal_timer_config(TIMER_NUM, &TimerCfg);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        return status;
    }

    //
    // Clear the timer and it's interrupt status
    //
    am_hal_timer_clear(TIMER_NUM);
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));

    am_hal_timer_enable(TIMER_NUM);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief returns true if timer has expired
//
//*****************************************************************************
bool uart_timer_isExpired(void)
{
    uint32_t intStatus;
    am_hal_timer_interrupt_status_get(false, &intStatus);
    return (intStatus & ( 1 << TIMER_NUM)) != 0;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
