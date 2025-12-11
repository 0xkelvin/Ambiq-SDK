//*****************************************************************************
//
//! @file fft_timer_control.c
//!
//! @brief configure free running upcounting timer running at max speed
//!
//! @defgroup pdm_fft PDM FFT Example
//! @ingroup audio_examples
//! @{
//
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
#include "fft_timer_control.h"
#include "am_util.h"

uint32_t
configAndStartTestTimer( am_hal_timer_clock_e clockSource )
{
    am_hal_timer_reset_config( TIMER_TEST_NUM );

    am_hal_timer_config_t sTTimerConfig;

    //
    // Set up the desired TIMER as free running up counting
    //
    am_hal_timer_default_config_set(&sTTimerConfig);

    //
    // Modify the default parameters.
    // Timer interrupt is not used here
    //
    sTTimerConfig.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    sTTimerConfig.eInputClock = clockSource;

    {
        uint32_t status = am_hal_timer_config(TIMER_TEST_NUM, &sTTimerConfig);
        if (status != AM_HAL_STATUS_SUCCESS)
        {
            return status;
        }
    }

    //
    // Clear the timer and its interrupt
    //
    am_hal_timer_clear(TIMER_TEST_NUM);

    am_hal_timer_enable(TIMER_TEST_NUM);

    // Start the TIMER.
    //
    am_hal_timer_start(TIMER_TEST_NUM);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
