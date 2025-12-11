//*****************************************************************************
//
//! @file ecdsa_sign_verify.c
//!
//! @brief A simple example to demonstrate using runtime crypto APIs.
//!
//! @addtogroup crypto_examples Crypto Examples
//
//! @defgroup ecdsa_sign_verify ECDSA Example
//! @ingroup crypto_examples
//! @{
//!
//! Purpose: This example demonstrates Elliptic Curve Digital
//! Signature Algorithm (ECDSA) operations using the mbedTLS cryptographic
//! library. The application showcases ECDSA key generation, message signing,
//! and signature verification with SHA256 hashing for secure message
//! authentication.
//!
//! This example requires 2048 words of stack space.<br>
//! The stack size is declared in the ini file and startup file.<br>
//! The user should specify this size in the apollo5x-system-config.yaml file<br>
//!
//! @section ecdsa_sign_verify_features Key Features
//!
//! 1. @b ECDSA @b Digital @b Signatures: Implements Elliptic Curve Digital
//!    Signature Algorithm for secure message authentication
//!
//! 2. @b SHA256 @b Hashing: Uses SHA256 cryptographic hash function for
//!    message digest generation and integrity verification
//!
//! 3. @b Elliptic @b Curve @b Cryptography: Leverages elliptic curve
//!    mathematics for efficient and secure cryptographic operations
//!
//! 4. @b Power @b Management: Implements intelligent crypto power control
//!    to keep crypto module active only during operations
//!
//! 5. @b mbedTLS @b Integration: Utilizes mbedTLS cryptographic library
//!    for standards-compliant ECDSA operations
//!
//! @section ecdsa_sign_verify_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes mbedTLS cryptographic library and memory allocation
//! - Generates ECDSA key pair (secp256r1 curve) or loads pre-generated keys
//! - Creates SHA256 hash of test message for digital signature
//! - Signs the message hash using ECDSA private key
//! - Verifies the signature using ECDSA public key
//! - Implements power management to enable crypto only during operations
//! - Provides debug output for cryptographic operations
//!
//! @section ecdsa_sign_verify_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM/SWO output for cryptographic operation results
//! 3. Verify successful signature generation and verification
//!
//! @section ecdsa_sign_verify_configuration Configuration
//!
//! - @b ENABLE_CRYPTO_ON_OFF: Enables intelligent crypto power management
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via ITM/SWO
//! - @b ECDSA_CURVE: Configurable elliptic curve (default: secp256r1)
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
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//
//! @note MBEDTLS_CONFIG_FILE must be defined for crypto to run properly
//
#define MBEDTLS_CONFIG_FILE <config-cc312-apollo4-no-os.h>
#include MBEDTLS_CONFIG_FILE

#include "cc_lib.h"

#include "mbedtls/ecdsa.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"
#include "mbedtls/memory_buffer_alloc.h"
#include "mbedtls_cc_mng.h"
#include "mbedtls/platform.h"

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
CCRndContext_t RndContex;
CCRndWorkBuff_t RndWorkBuff;
mbedtls_ctr_drbg_context RndState;
mbedtls_entropy_context MbedtlsEntropy;

static void dump_buf( const char *title, unsigned char *buf, size_t len );
static void dump_pubkey( const char *title, mbedtls_ecdsa_context *key );

int
main(void)
{
    bool bOTPEnabled = false;
    uint32_t ui32Status = 0;

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
    am_util_stdio_printf("Crypto EDCSA Sign and Verify Example!\n\n");

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
    am_util_stdio_printf("Crypto powered down!\n\n");
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

    int ret = 1;
    mbedtls_ecdsa_context ctx_sign, ctx_verify;
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctr_drbg;
    unsigned char message[100];
    unsigned char hash[32];
    unsigned char sig[MBEDTLS_ECDSA_MAX_LEN];
    size_t sig_len;
    const char *pers = "ecdsa";

    mbedtls_ecdsa_init( &ctx_sign );
    mbedtls_ecdsa_init( &ctx_verify );
    mbedtls_ctr_drbg_init( &ctr_drbg );

    memset( sig, 0, sizeof( sig ) );
    memset( message, 0x25, sizeof( message ) );

    //
    // Generate a key pair for signing
    //
    am_util_stdio_printf( "\n  . Seeding the random number generator..." );

    mbedtls_entropy_init( &entropy );
    if ( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen( pers ) ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_ctr_drbg_seed returned %d\n", ret );
        goto exit;
    }

    am_util_stdio_printf( " ok\n  . Generating key pair..." );

    if ( ( ret = mbedtls_ecdsa_genkey( &ctx_sign, MBEDTLS_ECP_DP_SECP192R1,
                              mbedtls_ctr_drbg_random, &ctr_drbg ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_ecdsa_genkey returned %d\n", ret );
        goto exit;
    }

    am_util_stdio_printf( " ok (key size: %d bits)\n", (int) ctx_sign.grp.pbits );

    dump_pubkey( "  + Public key: ", &ctx_sign );

    //
    // Compute message hash
    //
    am_util_stdio_printf( "  . Computing message hash..." );

    if ( ( ret = mbedtls_sha256_ret( message, sizeof( message ), hash, 0 ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_sha256_ret returned %d\n", ret );
        goto exit;
    }

    am_util_stdio_printf( " ok\n" );

    dump_buf( "  + Hash: ", hash, sizeof( hash ) );

    //
    // Sign message hash
    //
    am_util_stdio_printf( "  . Signing message hash..." );

    if ( ( ret = mbedtls_ecdsa_write_signature( &ctx_sign, MBEDTLS_MD_SHA256,
                                       hash, sizeof( hash ),
                                       sig, &sig_len,
                                       mbedtls_ctr_drbg_random, &ctr_drbg ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_ecdsa_genkey returned %d\n", ret );
        goto exit;
    }
    am_util_stdio_printf( " ok (signature length = %u)\n", (unsigned int) sig_len );

    dump_buf( "  + Signature: ", sig, sig_len );

    //
    // Transfer public information to verifying context
    //
    // We could use the same context for verification and signatures, but we
    // chose to use a new one in order to make it clear that the verifying
    // context only needs the public key (Q), and not the private key (d).
    //
    am_util_stdio_printf( "  . Preparing verification context..." );

    if ( ( ret = mbedtls_ecp_group_copy( &ctx_verify.grp, &ctx_sign.grp ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_ecp_group_copy returned %d\n", ret );
        goto exit;
    }

    if ( ( ret = mbedtls_ecp_copy( &ctx_verify.Q, &ctx_sign.Q ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_ecp_copy returned %d\n", ret );
        goto exit;
    }

    //
    // Verify signature
    //
    am_util_stdio_printf( " ok\n  . Verifying signature..." );

    if ( ( ret = mbedtls_ecdsa_read_signature( &ctx_verify,
                                      hash, sizeof( hash ),
                                      sig, sig_len ) ) != 0 )
    {
        am_util_stdio_printf( " failed\n  ! mbedtls_ecdsa_read_signature returned %d\n", ret );
        goto exit;
    }

    am_util_stdio_printf( " ok\n\n" );

exit:
    mbedtls_ecdsa_free( &ctx_verify );
    mbedtls_ecdsa_free( &ctx_sign );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );

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
    am_util_stdio_printf("Crypto powered down!\n\n");
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

static void dump_buf( const char *title, unsigned char *buf, size_t len )
{
    size_t i;

    am_util_stdio_printf( "%s", title );
    for ( i = 0; i < len; i++ )
    {
        am_util_stdio_printf("%c%c", "0123456789ABCDEF" [buf[i] / 16],
                       "0123456789ABCDEF" [buf[i] % 16] );
    }
    am_util_stdio_printf( "\n" );
}

static void dump_pubkey( const char *title, mbedtls_ecdsa_context *key )
{
    unsigned char buf[300];
    size_t len;

    if ( mbedtls_ecp_point_write_binary( &key->grp, &key->Q,
                MBEDTLS_ECP_PF_UNCOMPRESSED, &len, buf, sizeof buf ) != 0 )
    {
        am_util_stdio_printf("internal error\n");
        return;
    }

    dump_buf( title, buf, len );
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

