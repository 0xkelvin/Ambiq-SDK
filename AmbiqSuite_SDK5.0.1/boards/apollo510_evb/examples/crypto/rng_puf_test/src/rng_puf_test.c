//*****************************************************************************
//
//! @file rng_puf_test.c
//!
//! @brief A simple example to demonstrate use of the PUF based RNG.
//!
//! @addtogroup crypto_examples Crypto Examples
//
//! @defgroup rng_puf_test RNG PUF Example
//! @ingroup crypto_examples
//! @{
//!
//! Purpose: This example demonstrates hardware-based random number generation
//! using a Physical Unclonable Function (PUF) with an RNG as the entropy source.
//! This is mapped to the One-Time Programmable (OTP) memory. The application
//! showcases secure random number generation leveraging hardware entropy sources
//! for unpredictable random numbers suitable for cryptographic
//! applications.
//!
//! @section rng_puf_test_features Key Features
//!
//! 1. @b PUF @b Based @b RNG: Utilizes Physical Unclonable Function based RNG
//!    where the outputlocated in One-Time Programmable memory for
//!    hardware-based random number generation
//!
//! 2. @b Power @b Management: Implements intelligent power control to enable
//!    OTP only during random number generation operations
//!
//! 3. @b Secure @b Random @b Numbers: Provides cryptographically secure
//!    random numbers suitable for security applications
//!
//! @section rng_puf_test_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes PUF-based hardware entropy source for random number generation
//! - Reads entropy from PUF-RNG register
//! - Generates cryptographically secure random numbers using hardware entropy
//! - Implements power management to enable OTP only during operations
//! - Transitions to sleep mode after random number generation
//! - Provides debug output for hardware entropy operations
//!
//! @section rng_puf_test_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM/SWO output for hardware random number generation results
//! 3. Verify successful entropy collection and RNG operation
//!
//! @section rng_puf_test_configuration Configuration
//!
//! - @b NUM_RANDOM_BYTES: Number of random bytes to generate (default: 128)
//! - @b AM_DEBUG_PRINTF: Enables debug output via ITM/SWO
//!
//! Additional Information:
//! To enable debug printing, add the following project-level macro definitions.<br>
//! When defined, debug messages will be sent over ITM/SWO at 1MHz.<br>
//! - #define AM_DEBUG_PRINTF<br>
//!
//! Note that when this macro is defined, the device will never achieve deep
//! sleep, only normal sleep, due to the ITM (and thus the HFRC) being enabled.<br>
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

#include <string.h>

//*****************************************************************************
//
// Globals.
//
//*****************************************************************************

//
// Local return codes.
//
#define SUCCESS    0
#define FAIL       1

//
// Output buffer for random numbers.
//
#define NUM_RANDOM_BYTES 128
uint8_t pui8RandomBytes[NUM_RANDOM_BYTES];

//
// Compiler portable MIN macro in C.
//
#define MIN(a, b) ((a) < (b) ? (a) : (b))

//
// Macro to print newline after n bytes per line.
//
#define NEWLINE_AFTER_N_BYTES(n) \
    if ((n + 1) % 8 == 0) \
    { \
        am_util_stdio_printf("\n"); \
    }

//
// The TRNG is implemented in the OTP memory
// and the TRNG address is memory mapped to the OTP.
//
__STATIC_FORCEINLINE uint32_t get_entropy_u32(void)
{
    return OTP->RNG;
}

static uint32_t get_entropy(uint8_t * buffer, uint16_t length);
static uint32_t entropy_init(void);
static uint32_t entropy_deinit(void);

int
main(void)
{
    //
    // This will also power down crypto.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM output
    //
    if (SUCCESS != am_bsp_debug_printf_enable())
    {
        //
        // Cannot print - skip to deinitialization and sleep.
        //
        goto pwr_down_and_sleep;
    }

    //
    // Clear and print entry banner message.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("RNG (PUF/OTP) Demo Example!\n");

    //
    // One time initialization - to be done at the beginning of program.
    //
    //
    // Initialize Entropy Agent.
    //
    if (SUCCESS != entropy_init())
    {
        am_util_debug_printf("\nEntropy Initialization Failure\n");
        am_util_debug_printf("Code will skip entropy and go to sleep\n");
        goto pwr_down_and_sleep;
    }
    else
    {
        am_util_debug_printf("\nOTP powered On!\n");
    }

    //
    // Get the entropy random bytes. If successful, print the
    // bytes to ITM. Otherwise, print the relevant error message.
    //
    if (SUCCESS != get_entropy(pui8RandomBytes, NUM_RANDOM_BYTES))
    {
        am_util_debug_printf("\nEntropy Retrieval Failure\n");
    }
    else
    {
        am_util_stdio_printf("\nRetrieved random bytes:\n");
        for (uint32_t i = 0; i < NUM_RANDOM_BYTES; i++)
        {
            am_util_stdio_printf("0x%02X, ", pui8RandomBytes[i]);
            NEWLINE_AFTER_N_BYTES(i);
        }
    }

pwr_down_and_sleep:
    //
    // Deinitialization of Entropy Agent.
    //
    if (SUCCESS != entropy_deinit())
    {
        am_util_debug_printf("\n\nEntropy Deinitialization Failure\n");
    }
    else
    {
        am_util_debug_printf("\n\nOTP powered down!\n");
    }

    //
    // Print a message indicating the end of the example.
    //
    am_util_stdio_printf("\nExample Done.\n");

    //
    // Disable printing.
    //
    am_bsp_debug_printf_disable();

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

static uint32_t
get_entropy(uint8_t * buffer, uint16_t length)
{
    //
    // Validate input parameters.
    //
    if (length == 0 || buffer == NULL)
    {
        return FAIL;
    }

    uint8_t *byte_buffer = buffer;
    uint8_t fail_cnt = 0;

    //
    // While the passed in length is greater than zero
    // grab data from RNG and save to output.
    //
    while ((length > 0) & (fail_cnt < 5))
    {
        uint32_t word = get_entropy_u32();
        size_t copy_length = MIN(sizeof(uint32_t), length);

        //
        // This is a failure mode where the RNG doean't have enough randomness
        //
        if (word == 0xdeaddead)
        {
            fail_cnt++;
            continue;
        }

        memcpy(byte_buffer, &word, copy_length);
        byte_buffer += copy_length;
        length -= copy_length;
    }

    if (fail_cnt == 6)
    {
        return FAIL;
    }

    return SUCCESS;
}

static uint32_t
entropy_init(void)
{
    uint32_t ui32Status = 0;
    bool bPeripheralEnabled = false;

    //
    // Check if OTP is already on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bPeripheralEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Error during read of OTP power status\n");
        return FAIL;
    }

    //
    // Power on OTP if it is not already on.
    //
    if (!bPeripheralEnabled)
    {
        ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Error in power down of OTP module\n");
            return FAIL;
        }
    }

    return SUCCESS;
}

static uint32_t
entropy_deinit(void)
{
    uint32_t ui32Status = 0;
    bool bPeripheralEnabled = false;

    //
    // Check if OTP is on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bPeripheralEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_debug_printf("Error during read of OTP power status\n");
        return FAIL;
    }

    //
    // If OTP is on disable it.
    //
    if (bPeripheralEnabled)
    {
        ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_util_debug_printf("Error in power down of OTP module\n");
            return FAIL;
        }
    }

    return SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
