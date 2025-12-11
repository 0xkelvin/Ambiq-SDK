//*****************************************************************************
//
//! @file rsa_sign_verify.c
//!
//! @brief A simple example to demonstrate using runtime crypto APIs.
//!
//! @addtogroup crypto_examples Crypto Examples
//
//! @defgroup rsa_sign_verify RSA Example
//! @ingroup crypto_examples
//! @{
//!
//! Purpose: This example demonstrates RSA digital signature
//! operations using the mbedTLS cryptographic library. The application
//! showcases RSA key generation, message signing, and signature verification
//! with SHA256 hashing for secure message authentication.
//!
//! This example requires 2048 words of stack space.<br>
//! The stack size is declared in the ini file and startup file.<br>
//! The user should specify this size in the apollo510-system-config.yaml file<br>
//!
//! @section rsa_sign_verify_features Key Features
//!
//! 1. @b RSA @b Digital @b Signatures: Implements RSA digital signature
//!    generation and verification for secure message authentication
//!
//! 2. @b SHA256 @b Hashing: Uses SHA256 cryptographic hash function for
//!    message digest generation and integrity verification
//!
//! 3. @b Power @b Management: Implements intelligent crypto power control
//!    to keep crypto module active only during operations
//!
//! 4. @b mbedTLS @b Integration: Utilizes mbedTLS cryptographic library
//!    for standards-compliant crypto operations
//!
//! 5. @b Memory @b Optimization: Uses dynamic memory allocation for
//!    efficient resource management during crypto operations
//!
//! @section rsa_sign_verify_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes mbedTLS cryptographic library and memory allocation
//! - Generates RSA key pair (2048-bit) or loads pre-generated keys
//! - Creates SHA256 hash of test message for digital signature
//! - Signs the message hash using RSA private key
//! - Verifies the signature using RSA public key
//! - Implements power management to enable crypto only during operations
//! - Provides debug output for cryptographic operations
//!
//! @section rsa_sign_verify_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM/SWO output for cryptographic operation results
//! 3. Verify successful signature generation and verification
//!
//! @section rsa_sign_verify_configuration Configuration
//!
//! - @b ENABLE_CRYPTO_ON_OFF: Enables intelligent crypto power management
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via ITM/SWO
//! - @b RSA_KEY_SIZE: Configurable RSA key size (default: 2048 bits)
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

//
//! @note MBEDTLS_CONFIG_FILE must be defined for crypto to run properly
//
#define MBEDTLS_CONFIG_FILE <config-cc312-apollo4-no-os.h>
#include MBEDTLS_CONFIG_FILE

#include "cc_lib.h"

#include "mbedtls/rsa.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/memory_buffer_alloc.h"
#include "mbedtls_cc_mng.h"
#include "mbedtls/platform.h"

#include "crypto_test_data.h"
#include "rsa_keys.h"

#define ENABLE_CRYPTO_ON_OFF

//*****************************************************************************
//
// Globals.
//
//*****************************************************************************

//
// Dynamic memory for mbedTLS stack.
//
#define AM_MBEDTLS_HEAP_SIZE (128*1024/4)
uint32_t g_ui32MbedTLSHeap[AM_MBEDTLS_HEAP_SIZE];

//
// Context variables for mbedTLS operations.
//
mbedtls_rsa_context rsa;
CCRndContext_t RndContex;
CCRndWorkBuff_t RndWorkBuff;
mbedtls_ctr_drbg_context RndState;
mbedtls_entropy_context MbedtlsEntropy;

int
main(void)
{
    bool bOTPEnabled;
    uint32_t ui32Status;

    /* init Rnd context's inner member */
    RndContex.rndState = &RndState;
    RndContex.entropyCtx = &MbedtlsEntropy;

    mbedtls_platform_set_printf((int (*)( const char *, ... ))am_util_stdio_printf);

#ifdef ENABLE_CRYPTO_ON_OFF
    //
    // This will also power down crypto
    //
    am_bsp_low_power_init();
#else
    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);
#endif

    //
    // Enable printing
    //
    am_bsp_debug_printf_enable();
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("RSA Sign and Verify Example!\n\n");

    //
    // One time library initialization - to be done at the beginning of program
    //
    am_util_stdio_printf("Doing one time Init!\n\n");
#ifdef ENABLE_CRYPTO_ON_OFF
    am_util_stdio_printf("Powering On Crypto\n");
    //
    // Check and Power on OTP if it is not already on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bOTPEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error during read of OTP power status\n");
    }
    if (!bOTPEnabled)
    {
        ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Error in power up of OTP module\n");
        }
    }

    //
    // Enable the Crypto module
    //
    ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power up of Crypto module\n");
    }
#endif

    //
    // Initiailize MbedTLS
    //
    mbedtls_memory_buffer_alloc_init((uint8_t*)g_ui32MbedTLSHeap, AM_MBEDTLS_HEAP_SIZE);
    CC_LibInit(&RndContex, &RndWorkBuff);

#ifdef ENABLE_CRYPTO_ON_OFF
    //
    // We can power off crypto now
    //
    mbedtls_mng_suspend(0, 0);
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power down of Crypto module\n");
    }
    //
    // Power down OTP as well.
    //
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power down of OTP module\n");
    }
    am_util_stdio_printf("Crypto powered down!\n");
#endif
    //
    // Placeholder for application doing other things unrelated to crypto
    //
    am_hal_delay_us(1000);
#ifdef ENABLE_CRYPTO_ON_OFF
    am_util_stdio_printf("Powering On Crypto\n");
    //
    // Check and Power on OTP if it is not already on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bOTPEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error during read of OTP power status\n");
    }
    if (!bOTPEnabled)
    {
        ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Error in power up of OTP module\n");
        }
    }
    //
    // Enable the Crypto module
    //
    ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power up of Crypto module\n");
    }
    mbedtls_mng_resume(0, 0);
#endif
    //
    // Initialize the RSA module, and import our keys.
    //
    mbedtls_rsa_init(&rsa, MBEDTLS_RSA_PKCS_V15, 0);
    mbedtls_mpi_read_string(&rsa.N, 16, PUBLIC_RSA_N);
    mbedtls_mpi_read_string(&rsa.E, 16, PUBLIC_RSA_E);

    rsa.len = ( mbedtls_mpi_bitlen( &rsa.N ) + 7 ) >> 3;

    if (mbedtls_rsa_check_pubkey(&rsa))
    {
        am_util_stdio_printf("mbedtls_rsa_check_pubkey failed\n");
        while (1);
    }

    //
    // Run SHA256 on our binary, and then compare against the previously calculated hash.
    //
    uint8_t digest[TEST_HASH_SIZE];
    mbedtls_sha256(pui8TestBinary, TEST_BIN_SIZE, digest, false);

    uint32_t i;
    uint32_t ui32HashErrors = 0;
    for (i = 0; i < TEST_HASH_SIZE; i++)
    {
        if (digest[i] != pui8TestHash[i])
        {
            ui32HashErrors++;
        }
    }

    //
    // Print our SHA256 results.
    //
    if (ui32HashErrors == 0)
    {
        am_util_stdio_printf("SHA256 values match.\n");
    }
    else
    {
        am_util_stdio_printf("Error, %d mismatches found in SHA256.\n", ui32HashErrors);
    }

    //
    // Attempt to verify the provided signature using our public key.
    //
    uint32_t result = mbedtls_rsa_pkcs1_verify(&rsa,
                                               NULL,
                                               NULL,
                                               MBEDTLS_RSA_PUBLIC,
                                               MBEDTLS_MD_SHA256,
                                               32,
                                               pui8TestHash,
                                               pui8TestSignature);

    //
    // Release the RSA context. We're done with RSA operations.
    //
    mbedtls_rsa_free(&rsa);

    //
    // Print our RSA results.
    //
    if (result == 0)
    {
        am_util_stdio_printf("RSA signature verified.\n", result);
    }
    else
    {
        am_util_stdio_printf("RSA Error: 0x%08X\n", result);
    }

#ifdef ENABLE_CRYPTO_ON_OFF
    //
    // We can power off crypto now
    //
    mbedtls_mng_suspend(0, 0);
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power down of Crypto module\n");
    }
    //
    // Power down OTP as well.
    //
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power down of OTP module\n");
    }
    am_util_stdio_printf("Crypto powered down!\n");
#endif
    am_util_stdio_printf("Example Done\n");
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

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

