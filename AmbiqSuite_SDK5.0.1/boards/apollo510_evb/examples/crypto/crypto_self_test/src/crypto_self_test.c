//*****************************************************************************
//
//! @file crypto_self_test.c
//!
//! @brief A simple example to demonstrate using runtime crypto APIs.
//!
//! This example initializes the runtime crypto library at the beginning
//! and provides self-testing functionality for all mbedTLS
//! cryptographic algorithms to validate their correct operation and
//! implementation integrity.
//!
//! This example requires 20480 bytes of stack space.<br>
//! The stack size is declared in the ini file and startup file.<br>
//! The user should specify this size in the apollo510-system-config.yaml file<br>
//!
//! @section crypto_self_test_features Key Features
//!
//! 1. @b Self @b Testing: Tests all mbedTLS cryptographic
//!    algorithms to ensure correct implementation and operation
//!
//! 2. @b Multiple @b Algorithm @b Support: Includes AES, SHA, MD5, RSA, ECC,
//!    and other cryptographic algorithms for complete validation
//!
//! 3. @b Runtime @b Crypto @b Library: Initializes and validates the runtime
//!    cryptographic library for secure operations
//!
//! 4. @b Memory @b Buffer @b Allocation: Tests memory allocation and management
//!    for cryptographic operations
//!
//! 5. @b Entropy @b Source @b Validation: Verifies entropy sources and random
//!    number generation for cryptographic security
//!
//! @section crypto_self_test_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes mbedTLS cryptographic library and memory allocation
//! - Executes self-tests for all cryptographic algorithms
//! - Validates entropy sources and random number generation
//! - Tests memory buffer allocation and management
//! - Verifies implementation integrity of all crypto modules
//! - Provides detailed test results and validation status
//! - Implements power management for energy-efficient operation
//!
//! @section crypto_self_test_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM/SWO output for self-test results
//! 3. Verify successful completion of all cryptographic self-tests
//! 4. Review validation status and implementation integrity
//!
//! @section crypto_self_test_configuration Configuration
//!
//! - @b MBEDTLS_MODULE_PRINT: Enables detailed module feature reporting
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via ITM/SWO
//! - @b VERBOSE_TESTING: Enables verbose output for detailed test results
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
#include "string.h"
#include "cc_lib.h"

#define MBEDTLS_CONFIG_FILE <config-cc312-apollo4-no-os.h>
#include MBEDTLS_CONFIG_FILE

#include "am_util_crypto.h"

#include "mbedtls/entropy.h"
#include "mbedtls/entropy_poll.h"
#include "mbedtls/hmac_drbg.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/dhm.h"
#include "mbedtls/gcm.h"
#include "mbedtls/ccm.h"
#include "mbedtls/cmac.h"
#include "mbedtls/md2.h"
#include "mbedtls/md4.h"
#include "mbedtls/md5.h"
#include "mbedtls/ripemd160.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/arc4.h"
#include "mbedtls/des.h"
#include "mbedtls/aes.h"
#include "mbedtls/camellia.h"
#include "mbedtls/aria.h"
#include "mbedtls/chacha20.h"
#include "mbedtls/poly1305.h"
#include "mbedtls/chachapoly.h"
#include "mbedtls/base64.h"
#include "mbedtls/bignum.h"
#include "mbedtls/rsa.h"
#include "mbedtls/x509.h"
#include "mbedtls/xtea.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/ecp.h"
#include "mbedtls/ecjpake.h"
#include "mbedtls/timing.h"
#include "mbedtls/nist_kw.h"
#include "mbedtls/certs.h"
#include "mbedtls/x509.h"
#include "mbedtls/x509_crl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/x509_csr.h"
#include <string.h>

//#define MBEDTLS_MODULE_PRINT
#if defined(MBEDTLS_MODULE_PRINT)
#include "mbedtls/version.h"
static const char *features[] = {
#if defined(MBEDTLS_VERSION_FEATURES)
#if defined(MBEDTLS_HAVE_ASM)
    "MBEDTLS_HAVE_ASM",
#endif /* MBEDTLS_HAVE_ASM */
#if defined(MBEDTLS_NO_UDBL_DIVISION)
    "MBEDTLS_NO_UDBL_DIVISION",
#endif /* MBEDTLS_NO_UDBL_DIVISION */
#if defined(MBEDTLS_NO_64BIT_MULTIPLICATION)
    "MBEDTLS_NO_64BIT_MULTIPLICATION",
#endif /* MBEDTLS_NO_64BIT_MULTIPLICATION */
#if defined(MBEDTLS_HAVE_SSE2)
    "MBEDTLS_HAVE_SSE2",
#endif /* MBEDTLS_HAVE_SSE2 */
#if defined(MBEDTLS_HAVE_TIME)
    "MBEDTLS_HAVE_TIME",
#endif /* MBEDTLS_HAVE_TIME */
#if defined(MBEDTLS_HAVE_TIME_DATE)
    "MBEDTLS_HAVE_TIME_DATE",
#endif /* MBEDTLS_HAVE_TIME_DATE */
#if defined(MBEDTLS_PLATFORM_MEMORY)
    "MBEDTLS_PLATFORM_MEMORY",
#endif /* MBEDTLS_PLATFORM_MEMORY */
#if defined(MBEDTLS_PLATFORM_NO_STD_FUNCTIONS)
    "MBEDTLS_PLATFORM_NO_STD_FUNCTIONS",
#endif /* MBEDTLS_PLATFORM_NO_STD_FUNCTIONS */
#if defined(MBEDTLS_PLATFORM_EXIT_ALT)
    "MBEDTLS_PLATFORM_EXIT_ALT",
#endif /* MBEDTLS_PLATFORM_EXIT_ALT */
#if defined(MBEDTLS_PLATFORM_TIME_ALT)
    "MBEDTLS_PLATFORM_TIME_ALT",
#endif /* MBEDTLS_PLATFORM_TIME_ALT */
#if defined(MBEDTLS_PLATFORM_FPRINTF_ALT)
    "MBEDTLS_PLATFORM_FPRINTF_ALT",
#endif /* MBEDTLS_PLATFORM_FPRINTF_ALT */
#if defined(MBEDTLS_PLATFORM_PRINTF_ALT)
    "MBEDTLS_PLATFORM_PRINTF_ALT",
#endif /* MBEDTLS_PLATFORM_PRINTF_ALT */
#if defined(MBEDTLS_PLATFORM_SNPRINTF_ALT)
    "MBEDTLS_PLATFORM_SNPRINTF_ALT",
#endif /* MBEDTLS_PLATFORM_SNPRINTF_ALT */
#if defined(MBEDTLS_PLATFORM_NV_SEED_ALT)
    "MBEDTLS_PLATFORM_NV_SEED_ALT",
#endif /* MBEDTLS_PLATFORM_NV_SEED_ALT */
#if defined(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
    "MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT",
#endif /* MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT */
#if defined(MBEDTLS_DEPRECATED_WARNING)
    "MBEDTLS_DEPRECATED_WARNING",
#endif /* MBEDTLS_DEPRECATED_WARNING */
#if defined(MBEDTLS_DEPRECATED_REMOVED)
    "MBEDTLS_DEPRECATED_REMOVED",
#endif /* MBEDTLS_DEPRECATED_REMOVED */
#if defined(MBEDTLS_CHECK_PARAMS)
    "MBEDTLS_CHECK_PARAMS",
#endif /* MBEDTLS_CHECK_PARAMS */
#if defined(MBEDTLS_TIMING_ALT)
    "MBEDTLS_TIMING_ALT",
#endif /* MBEDTLS_TIMING_ALT */
#if defined(MBEDTLS_AES_ALT)
    "MBEDTLS_AES_ALT",
#endif /* MBEDTLS_AES_ALT */
#if defined(MBEDTLS_ARC4_ALT)
    "MBEDTLS_ARC4_ALT",
#endif /* MBEDTLS_ARC4_ALT */
#if defined(MBEDTLS_ARIA_ALT)
    "MBEDTLS_ARIA_ALT",
#endif /* MBEDTLS_ARIA_ALT */
#if defined(MBEDTLS_BLOWFISH_ALT)
    "MBEDTLS_BLOWFISH_ALT",
#endif /* MBEDTLS_BLOWFISH_ALT */
#if defined(MBEDTLS_CAMELLIA_ALT)
    "MBEDTLS_CAMELLIA_ALT",
#endif /* MBEDTLS_CAMELLIA_ALT */
#if defined(MBEDTLS_CCM_ALT)
    "MBEDTLS_CCM_ALT",
#endif /* MBEDTLS_CCM_ALT */
#if defined(MBEDTLS_CHACHA20_ALT)
    "MBEDTLS_CHACHA20_ALT",
#endif /* MBEDTLS_CHACHA20_ALT */
#if defined(MBEDTLS_CHACHAPOLY_ALT)
    "MBEDTLS_CHACHAPOLY_ALT",
#endif /* MBEDTLS_CHACHAPOLY_ALT */
#if defined(MBEDTLS_CMAC_ALT)
    "MBEDTLS_CMAC_ALT",
#endif /* MBEDTLS_CMAC_ALT */
#if defined(MBEDTLS_DES_ALT)
    "MBEDTLS_DES_ALT",
#endif /* MBEDTLS_DES_ALT */
#if defined(MBEDTLS_DHM_ALT)
    "MBEDTLS_DHM_ALT",
#endif /* MBEDTLS_DHM_ALT */
#if defined(MBEDTLS_ECJPAKE_ALT)
    "MBEDTLS_ECJPAKE_ALT",
#endif /* MBEDTLS_ECJPAKE_ALT */
#if defined(MBEDTLS_GCM_ALT)
    "MBEDTLS_GCM_ALT",
#endif /* MBEDTLS_GCM_ALT */
#if defined(MBEDTLS_NIST_KW_ALT)
    "MBEDTLS_NIST_KW_ALT",
#endif /* MBEDTLS_NIST_KW_ALT */
#if defined(MBEDTLS_MD2_ALT)
    "MBEDTLS_MD2_ALT",
#endif /* MBEDTLS_MD2_ALT */
#if defined(MBEDTLS_MD4_ALT)
    "MBEDTLS_MD4_ALT",
#endif /* MBEDTLS_MD4_ALT */
#if defined(MBEDTLS_MD5_ALT)
    "MBEDTLS_MD5_ALT",
#endif /* MBEDTLS_MD5_ALT */
#if defined(MBEDTLS_POLY1305_ALT)
    "MBEDTLS_POLY1305_ALT",
#endif /* MBEDTLS_POLY1305_ALT */
#if defined(MBEDTLS_RIPEMD160_ALT)
    "MBEDTLS_RIPEMD160_ALT",
#endif /* MBEDTLS_RIPEMD160_ALT */
#if defined(MBEDTLS_RSA_ALT)
    "MBEDTLS_RSA_ALT",
#endif /* MBEDTLS_RSA_ALT */
#if defined(MBEDTLS_SHA1_ALT)
    "MBEDTLS_SHA1_ALT",
#endif /* MBEDTLS_SHA1_ALT */
#if defined(MBEDTLS_SHA256_ALT)
    "MBEDTLS_SHA256_ALT",
#endif /* MBEDTLS_SHA256_ALT */
#if defined(MBEDTLS_SHA512_ALT)
    "MBEDTLS_SHA512_ALT",
#endif /* MBEDTLS_SHA512_ALT */
#if defined(MBEDTLS_XTEA_ALT)
    "MBEDTLS_XTEA_ALT",
#endif /* MBEDTLS_XTEA_ALT */
#if defined(MBEDTLS_ECP_ALT)
    "MBEDTLS_ECP_ALT",
#endif /* MBEDTLS_ECP_ALT */
#if defined(MBEDTLS_MD2_PROCESS_ALT)
    "MBEDTLS_MD2_PROCESS_ALT",
#endif /* MBEDTLS_MD2_PROCESS_ALT */
#if defined(MBEDTLS_MD4_PROCESS_ALT)
    "MBEDTLS_MD4_PROCESS_ALT",
#endif /* MBEDTLS_MD4_PROCESS_ALT */
#if defined(MBEDTLS_MD5_PROCESS_ALT)
    "MBEDTLS_MD5_PROCESS_ALT",
#endif /* MBEDTLS_MD5_PROCESS_ALT */
#if defined(MBEDTLS_RIPEMD160_PROCESS_ALT)
    "MBEDTLS_RIPEMD160_PROCESS_ALT",
#endif /* MBEDTLS_RIPEMD160_PROCESS_ALT */
#if defined(MBEDTLS_SHA1_PROCESS_ALT)
    "MBEDTLS_SHA1_PROCESS_ALT",
#endif /* MBEDTLS_SHA1_PROCESS_ALT */
#if defined(MBEDTLS_SHA256_PROCESS_ALT)
    "MBEDTLS_SHA256_PROCESS_ALT",
#endif /* MBEDTLS_SHA256_PROCESS_ALT */
#if defined(MBEDTLS_SHA512_PROCESS_ALT)
    "MBEDTLS_SHA512_PROCESS_ALT",
#endif /* MBEDTLS_SHA512_PROCESS_ALT */
#if defined(MBEDTLS_DES_SETKEY_ALT)
    "MBEDTLS_DES_SETKEY_ALT",
#endif /* MBEDTLS_DES_SETKEY_ALT */
#if defined(MBEDTLS_DES_CRYPT_ECB_ALT)
    "MBEDTLS_DES_CRYPT_ECB_ALT",
#endif /* MBEDTLS_DES_CRYPT_ECB_ALT */
#if defined(MBEDTLS_DES3_CRYPT_ECB_ALT)
    "MBEDTLS_DES3_CRYPT_ECB_ALT",
#endif /* MBEDTLS_DES3_CRYPT_ECB_ALT */
#if defined(MBEDTLS_AES_SETKEY_ENC_ALT)
    "MBEDTLS_AES_SETKEY_ENC_ALT",
#endif /* MBEDTLS_AES_SETKEY_ENC_ALT */
#if defined(MBEDTLS_AES_SETKEY_DEC_ALT)
    "MBEDTLS_AES_SETKEY_DEC_ALT",
#endif /* MBEDTLS_AES_SETKEY_DEC_ALT */
#if defined(MBEDTLS_AES_ENCRYPT_ALT)
    "MBEDTLS_AES_ENCRYPT_ALT",
#endif /* MBEDTLS_AES_ENCRYPT_ALT */
#if defined(MBEDTLS_AES_DECRYPT_ALT)
    "MBEDTLS_AES_DECRYPT_ALT",
#endif /* MBEDTLS_AES_DECRYPT_ALT */
#if defined(MBEDTLS_ECDH_GEN_PUBLIC_ALT)
    "MBEDTLS_ECDH_GEN_PUBLIC_ALT",
#endif /* MBEDTLS_ECDH_GEN_PUBLIC_ALT */
#if defined(MBEDTLS_ECDH_COMPUTE_SHARED_ALT)
    "MBEDTLS_ECDH_COMPUTE_SHARED_ALT",
#endif /* MBEDTLS_ECDH_COMPUTE_SHARED_ALT */
#if defined(MBEDTLS_ECDSA_VERIFY_ALT)
    "MBEDTLS_ECDSA_VERIFY_ALT",
#endif /* MBEDTLS_ECDSA_VERIFY_ALT */
#if defined(MBEDTLS_ECDSA_SIGN_ALT)
    "MBEDTLS_ECDSA_SIGN_ALT",
#endif /* MBEDTLS_ECDSA_SIGN_ALT */
#if defined(MBEDTLS_ECDSA_GENKEY_ALT)
    "MBEDTLS_ECDSA_GENKEY_ALT",
#endif /* MBEDTLS_ECDSA_GENKEY_ALT */
#if defined(MBEDTLS_ECP_INTERNAL_ALT)
    "MBEDTLS_ECP_INTERNAL_ALT",
#endif /* MBEDTLS_ECP_INTERNAL_ALT */
#if defined(MBEDTLS_ECP_RANDOMIZE_JAC_ALT)
    "MBEDTLS_ECP_RANDOMIZE_JAC_ALT",
#endif /* MBEDTLS_ECP_RANDOMIZE_JAC_ALT */
#if defined(MBEDTLS_ECP_ADD_MIXED_ALT)
    "MBEDTLS_ECP_ADD_MIXED_ALT",
#endif /* MBEDTLS_ECP_ADD_MIXED_ALT */
#if defined(MBEDTLS_ECP_DOUBLE_JAC_ALT)
    "MBEDTLS_ECP_DOUBLE_JAC_ALT",
#endif /* MBEDTLS_ECP_DOUBLE_JAC_ALT */
#if defined(MBEDTLS_ECP_NORMALIZE_JAC_MANY_ALT)
    "MBEDTLS_ECP_NORMALIZE_JAC_MANY_ALT",
#endif /* MBEDTLS_ECP_NORMALIZE_JAC_MANY_ALT */
#if defined(MBEDTLS_ECP_NORMALIZE_JAC_ALT)
    "MBEDTLS_ECP_NORMALIZE_JAC_ALT",
#endif /* MBEDTLS_ECP_NORMALIZE_JAC_ALT */
#if defined(MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT)
    "MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT",
#endif /* MBEDTLS_ECP_DOUBLE_ADD_MXZ_ALT */
#if defined(MBEDTLS_ECP_RANDOMIZE_MXZ_ALT)
    "MBEDTLS_ECP_RANDOMIZE_MXZ_ALT",
#endif /* MBEDTLS_ECP_RANDOMIZE_MXZ_ALT */
#if defined(MBEDTLS_ECP_NORMALIZE_MXZ_ALT)
    "MBEDTLS_ECP_NORMALIZE_MXZ_ALT",
#endif /* MBEDTLS_ECP_NORMALIZE_MXZ_ALT */
#if defined(MBEDTLS_TEST_NULL_ENTROPY)
    "MBEDTLS_TEST_NULL_ENTROPY",
#endif /* MBEDTLS_TEST_NULL_ENTROPY */
#if defined(MBEDTLS_ENTROPY_HARDWARE_ALT)
    "MBEDTLS_ENTROPY_HARDWARE_ALT",
#endif /* MBEDTLS_ENTROPY_HARDWARE_ALT */
#if defined(MBEDTLS_AES_ROM_TABLES)
    "MBEDTLS_AES_ROM_TABLES",
#endif /* MBEDTLS_AES_ROM_TABLES */
#if defined(MBEDTLS_AES_FEWER_TABLES)
    "MBEDTLS_AES_FEWER_TABLES",
#endif /* MBEDTLS_AES_FEWER_TABLES */
#if defined(MBEDTLS_CAMELLIA_SMALL_MEMORY)
    "MBEDTLS_CAMELLIA_SMALL_MEMORY",
#endif /* MBEDTLS_CAMELLIA_SMALL_MEMORY */
#if defined(MBEDTLS_CIPHER_MODE_CBC)
    "MBEDTLS_CIPHER_MODE_CBC",
#endif /* MBEDTLS_CIPHER_MODE_CBC */
#if defined(MBEDTLS_CIPHER_MODE_CFB)
    "MBEDTLS_CIPHER_MODE_CFB",
#endif /* MBEDTLS_CIPHER_MODE_CFB */
#if defined(MBEDTLS_CIPHER_MODE_CTR)
    "MBEDTLS_CIPHER_MODE_CTR",
#endif /* MBEDTLS_CIPHER_MODE_CTR */
#if defined(MBEDTLS_CIPHER_MODE_OFB)
    "MBEDTLS_CIPHER_MODE_OFB",
#endif /* MBEDTLS_CIPHER_MODE_OFB */
#if defined(MBEDTLS_CIPHER_MODE_XTS)
    "MBEDTLS_CIPHER_MODE_XTS",
#endif /* MBEDTLS_CIPHER_MODE_XTS */
#if defined(MBEDTLS_CIPHER_NULL_CIPHER)
    "MBEDTLS_CIPHER_NULL_CIPHER",
#endif /* MBEDTLS_CIPHER_NULL_CIPHER */
#if defined(MBEDTLS_CIPHER_PADDING_PKCS7)
    "MBEDTLS_CIPHER_PADDING_PKCS7",
#endif /* MBEDTLS_CIPHER_PADDING_PKCS7 */
#if defined(MBEDTLS_CIPHER_PADDING_ONE_AND_ZEROS)
    "MBEDTLS_CIPHER_PADDING_ONE_AND_ZEROS",
#endif /* MBEDTLS_CIPHER_PADDING_ONE_AND_ZEROS */
#if defined(MBEDTLS_CIPHER_PADDING_ZEROS_AND_LEN)
    "MBEDTLS_CIPHER_PADDING_ZEROS_AND_LEN",
#endif /* MBEDTLS_CIPHER_PADDING_ZEROS_AND_LEN */
#if defined(MBEDTLS_CIPHER_PADDING_ZEROS)
    "MBEDTLS_CIPHER_PADDING_ZEROS",
#endif /* MBEDTLS_CIPHER_PADDING_ZEROS */
#if defined(MBEDTLS_ENABLE_WEAK_CIPHERSUITES)
    "MBEDTLS_ENABLE_WEAK_CIPHERSUITES",
#endif /* MBEDTLS_ENABLE_WEAK_CIPHERSUITES */
#if defined(MBEDTLS_REMOVE_ARC4_CIPHERSUITES)
    "MBEDTLS_REMOVE_ARC4_CIPHERSUITES",
#endif /* MBEDTLS_REMOVE_ARC4_CIPHERSUITES */
#if defined(MBEDTLS_REMOVE_3DES_CIPHERSUITES)
    "MBEDTLS_REMOVE_3DES_CIPHERSUITES",
#endif /* MBEDTLS_REMOVE_3DES_CIPHERSUITES */
#if defined(MBEDTLS_ECP_DP_SECP192R1_ENABLED)
    "MBEDTLS_ECP_DP_SECP192R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP192R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP224R1_ENABLED)
    "MBEDTLS_ECP_DP_SECP224R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP224R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
    "MBEDTLS_ECP_DP_SECP256R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP256R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP384R1_ENABLED)
    "MBEDTLS_ECP_DP_SECP384R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP384R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP521R1_ENABLED)
    "MBEDTLS_ECP_DP_SECP521R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP521R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP192K1_ENABLED)
    "MBEDTLS_ECP_DP_SECP192K1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP192K1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP224K1_ENABLED)
    "MBEDTLS_ECP_DP_SECP224K1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP224K1_ENABLED */
#if defined(MBEDTLS_ECP_DP_SECP256K1_ENABLED)
    "MBEDTLS_ECP_DP_SECP256K1_ENABLED",
#endif /* MBEDTLS_ECP_DP_SECP256K1_ENABLED */
#if defined(MBEDTLS_ECP_DP_BP256R1_ENABLED)
    "MBEDTLS_ECP_DP_BP256R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_BP256R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_BP384R1_ENABLED)
    "MBEDTLS_ECP_DP_BP384R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_BP384R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_BP512R1_ENABLED)
    "MBEDTLS_ECP_DP_BP512R1_ENABLED",
#endif /* MBEDTLS_ECP_DP_BP512R1_ENABLED */
#if defined(MBEDTLS_ECP_DP_CURVE25519_ENABLED)
    "MBEDTLS_ECP_DP_CURVE25519_ENABLED",
#endif /* MBEDTLS_ECP_DP_CURVE25519_ENABLED */
#if defined(MBEDTLS_ECP_DP_CURVE448_ENABLED)
    "MBEDTLS_ECP_DP_CURVE448_ENABLED",
#endif /* MBEDTLS_ECP_DP_CURVE448_ENABLED */
#if defined(MBEDTLS_ECP_NIST_OPTIM)
    "MBEDTLS_ECP_NIST_OPTIM",
#endif /* MBEDTLS_ECP_NIST_OPTIM */
#if defined(MBEDTLS_ECP_RESTARTABLE)
    "MBEDTLS_ECP_RESTARTABLE",
#endif /* MBEDTLS_ECP_RESTARTABLE */
#if defined(MBEDTLS_ECDSA_DETERMINISTIC)
    "MBEDTLS_ECDSA_DETERMINISTIC",
#endif /* MBEDTLS_ECDSA_DETERMINISTIC */
#if defined(MBEDTLS_KEY_EXCHANGE_PSK_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_PSK_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_PSK_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_DHE_PSK_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_ECDHE_PSK_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_RSA_PSK_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_RSA_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_RSA_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_RSA_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_ECDH_RSA_ENABLED */
#if defined(MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED)
    "MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED",
#endif /* MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED */
#if defined(MBEDTLS_PK_PARSE_EC_EXTENDED)
    "MBEDTLS_PK_PARSE_EC_EXTENDED",
#endif /* MBEDTLS_PK_PARSE_EC_EXTENDED */
#if defined(MBEDTLS_ERROR_STRERROR_DUMMY)
    "MBEDTLS_ERROR_STRERROR_DUMMY",
#endif /* MBEDTLS_ERROR_STRERROR_DUMMY */
#if defined(MBEDTLS_GENPRIME)
    "MBEDTLS_GENPRIME",
#endif /* MBEDTLS_GENPRIME */
#if defined(MBEDTLS_FS_IO)
    "MBEDTLS_FS_IO",
#endif /* MBEDTLS_FS_IO */
#if defined(MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES)
    "MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES",
#endif /* MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES */
#if defined(MBEDTLS_NO_PLATFORM_ENTROPY)
    "MBEDTLS_NO_PLATFORM_ENTROPY",
#endif /* MBEDTLS_NO_PLATFORM_ENTROPY */
#if defined(MBEDTLS_ENTROPY_FORCE_SHA256)
    "MBEDTLS_ENTROPY_FORCE_SHA256",
#endif /* MBEDTLS_ENTROPY_FORCE_SHA256 */
#if defined(MBEDTLS_ENTROPY_NV_SEED)
    "MBEDTLS_ENTROPY_NV_SEED",
#endif /* MBEDTLS_ENTROPY_NV_SEED */
#if defined(MBEDTLS_MEMORY_DEBUG)
    "MBEDTLS_MEMORY_DEBUG",
#endif /* MBEDTLS_MEMORY_DEBUG */
#if defined(MBEDTLS_MEMORY_BACKTRACE)
    "MBEDTLS_MEMORY_BACKTRACE",
#endif /* MBEDTLS_MEMORY_BACKTRACE */
#if defined(MBEDTLS_PK_RSA_ALT_SUPPORT)
    "MBEDTLS_PK_RSA_ALT_SUPPORT",
#endif /* MBEDTLS_PK_RSA_ALT_SUPPORT */
#if defined(MBEDTLS_PKCS1_V15)
    "MBEDTLS_PKCS1_V15",
#endif /* MBEDTLS_PKCS1_V15 */
#if defined(MBEDTLS_PKCS1_V21)
    "MBEDTLS_PKCS1_V21",
#endif /* MBEDTLS_PKCS1_V21 */
#if defined(MBEDTLS_RSA_NO_CRT)
    "MBEDTLS_RSA_NO_CRT",
#endif /* MBEDTLS_RSA_NO_CRT */
#if defined(MBEDTLS_SELF_TEST)
    "MBEDTLS_SELF_TEST",
#endif /* MBEDTLS_SELF_TEST */
#if defined(MBEDTLS_SHA256_SMALLER)
    "MBEDTLS_SHA256_SMALLER",
#endif /* MBEDTLS_SHA256_SMALLER */
#if defined(MBEDTLS_SSL_ALL_ALERT_MESSAGES)
    "MBEDTLS_SSL_ALL_ALERT_MESSAGES",
#endif /* MBEDTLS_SSL_ALL_ALERT_MESSAGES */
#if defined(MBEDTLS_SSL_ASYNC_PRIVATE)
    "MBEDTLS_SSL_ASYNC_PRIVATE",
#endif /* MBEDTLS_SSL_ASYNC_PRIVATE */
#if defined(MBEDTLS_SSL_DEBUG_ALL)
    "MBEDTLS_SSL_DEBUG_ALL",
#endif /* MBEDTLS_SSL_DEBUG_ALL */
#if defined(MBEDTLS_SSL_ENCRYPT_THEN_MAC)
    "MBEDTLS_SSL_ENCRYPT_THEN_MAC",
#endif /* MBEDTLS_SSL_ENCRYPT_THEN_MAC */
#if defined(MBEDTLS_SSL_EXTENDED_MASTER_SECRET)
    "MBEDTLS_SSL_EXTENDED_MASTER_SECRET",
#endif /* MBEDTLS_SSL_EXTENDED_MASTER_SECRET */
#if defined(MBEDTLS_SSL_FALLBACK_SCSV)
    "MBEDTLS_SSL_FALLBACK_SCSV",
#endif /* MBEDTLS_SSL_FALLBACK_SCSV */
#if defined(MBEDTLS_SSL_HW_RECORD_ACCEL)
    "MBEDTLS_SSL_HW_RECORD_ACCEL",
#endif /* MBEDTLS_SSL_HW_RECORD_ACCEL */
#if defined(MBEDTLS_SSL_CBC_RECORD_SPLITTING)
    "MBEDTLS_SSL_CBC_RECORD_SPLITTING",
#endif /* MBEDTLS_SSL_CBC_RECORD_SPLITTING */
#if defined(MBEDTLS_SSL_RENEGOTIATION)
    "MBEDTLS_SSL_RENEGOTIATION",
#endif /* MBEDTLS_SSL_RENEGOTIATION */
#if defined(MBEDTLS_SSL_SRV_SUPPORT_SSLV2_CLIENT_HELLO)
    "MBEDTLS_SSL_SRV_SUPPORT_SSLV2_CLIENT_HELLO",
#endif /* MBEDTLS_SSL_SRV_SUPPORT_SSLV2_CLIENT_HELLO */
#if defined(MBEDTLS_SSL_SRV_RESPECT_CLIENT_PREFERENCE)
    "MBEDTLS_SSL_SRV_RESPECT_CLIENT_PREFERENCE",
#endif /* MBEDTLS_SSL_SRV_RESPECT_CLIENT_PREFERENCE */
#if defined(MBEDTLS_SSL_MAX_FRAGMENT_LENGTH)
    "MBEDTLS_SSL_MAX_FRAGMENT_LENGTH",
#endif /* MBEDTLS_SSL_MAX_FRAGMENT_LENGTH */
#if defined(MBEDTLS_SSL_PROTO_SSL3)
    "MBEDTLS_SSL_PROTO_SSL3",
#endif /* MBEDTLS_SSL_PROTO_SSL3 */
#if defined(MBEDTLS_SSL_PROTO_TLS1)
    "MBEDTLS_SSL_PROTO_TLS1",
#endif /* MBEDTLS_SSL_PROTO_TLS1 */
#if defined(MBEDTLS_SSL_PROTO_TLS1_1)
    "MBEDTLS_SSL_PROTO_TLS1_1",
#endif /* MBEDTLS_SSL_PROTO_TLS1_1 */
#if defined(MBEDTLS_SSL_PROTO_TLS1_2)
    "MBEDTLS_SSL_PROTO_TLS1_2",
#endif /* MBEDTLS_SSL_PROTO_TLS1_2 */
#if defined(MBEDTLS_SSL_PROTO_DTLS)
    "MBEDTLS_SSL_PROTO_DTLS",
#endif /* MBEDTLS_SSL_PROTO_DTLS */
#if defined(MBEDTLS_SSL_ALPN)
    "MBEDTLS_SSL_ALPN",
#endif /* MBEDTLS_SSL_ALPN */
#if defined(MBEDTLS_SSL_DTLS_ANTI_REPLAY)
    "MBEDTLS_SSL_DTLS_ANTI_REPLAY",
#endif /* MBEDTLS_SSL_DTLS_ANTI_REPLAY */
#if defined(MBEDTLS_SSL_DTLS_HELLO_VERIFY)
    "MBEDTLS_SSL_DTLS_HELLO_VERIFY",
#endif /* MBEDTLS_SSL_DTLS_HELLO_VERIFY */
#if defined(MBEDTLS_SSL_DTLS_CLIENT_PORT_REUSE)
    "MBEDTLS_SSL_DTLS_CLIENT_PORT_REUSE",
#endif /* MBEDTLS_SSL_DTLS_CLIENT_PORT_REUSE */
#if defined(MBEDTLS_SSL_DTLS_BADMAC_LIMIT)
    "MBEDTLS_SSL_DTLS_BADMAC_LIMIT",
#endif /* MBEDTLS_SSL_DTLS_BADMAC_LIMIT */
#if defined(MBEDTLS_SSL_SESSION_TICKETS)
    "MBEDTLS_SSL_SESSION_TICKETS",
#endif /* MBEDTLS_SSL_SESSION_TICKETS */
#if defined(MBEDTLS_SSL_EXPORT_KEYS)
    "MBEDTLS_SSL_EXPORT_KEYS",
#endif /* MBEDTLS_SSL_EXPORT_KEYS */
#if defined(MBEDTLS_SSL_SERVER_NAME_INDICATION)
    "MBEDTLS_SSL_SERVER_NAME_INDICATION",
#endif /* MBEDTLS_SSL_SERVER_NAME_INDICATION */
#if defined(MBEDTLS_SSL_TRUNCATED_HMAC)
    "MBEDTLS_SSL_TRUNCATED_HMAC",
#endif /* MBEDTLS_SSL_TRUNCATED_HMAC */
#if defined(MBEDTLS_SSL_TRUNCATED_HMAC_COMPAT)
    "MBEDTLS_SSL_TRUNCATED_HMAC_COMPAT",
#endif /* MBEDTLS_SSL_TRUNCATED_HMAC_COMPAT */
#if defined(MBEDTLS_THREADING_ALT)
    "MBEDTLS_THREADING_ALT",
#endif /* MBEDTLS_THREADING_ALT */
#if defined(MBEDTLS_THREADING_PTHREAD)
    "MBEDTLS_THREADING_PTHREAD",
#endif /* MBEDTLS_THREADING_PTHREAD */
#if defined(MBEDTLS_VERSION_FEATURES)
    "MBEDTLS_VERSION_FEATURES",
#endif /* MBEDTLS_VERSION_FEATURES */
#if defined(MBEDTLS_X509_ALLOW_EXTENSIONS_NON_V3)
    "MBEDTLS_X509_ALLOW_EXTENSIONS_NON_V3",
#endif /* MBEDTLS_X509_ALLOW_EXTENSIONS_NON_V3 */
#if defined(MBEDTLS_X509_ALLOW_UNSUPPORTED_CRITICAL_EXTENSION)
    "MBEDTLS_X509_ALLOW_UNSUPPORTED_CRITICAL_EXTENSION",
#endif /* MBEDTLS_X509_ALLOW_UNSUPPORTED_CRITICAL_EXTENSION */
#if defined(MBEDTLS_X509_CHECK_KEY_USAGE)
    "MBEDTLS_X509_CHECK_KEY_USAGE",
#endif /* MBEDTLS_X509_CHECK_KEY_USAGE */
#if defined(MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE)
    "MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE",
#endif /* MBEDTLS_X509_CHECK_EXTENDED_KEY_USAGE */
#if defined(MBEDTLS_X509_RSASSA_PSS_SUPPORT)
    "MBEDTLS_X509_RSASSA_PSS_SUPPORT",
#endif /* MBEDTLS_X509_RSASSA_PSS_SUPPORT */
#if defined(MBEDTLS_ZLIB_SUPPORT)
    "MBEDTLS_ZLIB_SUPPORT",
#endif /* MBEDTLS_ZLIB_SUPPORT */
#if defined(MBEDTLS_AESNI_C)
    "MBEDTLS_AESNI_C",
#endif /* MBEDTLS_AESNI_C */
#if defined(MBEDTLS_AES_C)
    "MBEDTLS_AES_C",
#endif /* MBEDTLS_AES_C */
#if defined(MBEDTLS_ARC4_C)
    "MBEDTLS_ARC4_C",
#endif /* MBEDTLS_ARC4_C */
#if defined(MBEDTLS_ASN1_PARSE_C)
    "MBEDTLS_ASN1_PARSE_C",
#endif /* MBEDTLS_ASN1_PARSE_C */
#if defined(MBEDTLS_ASN1_WRITE_C)
    "MBEDTLS_ASN1_WRITE_C",
#endif /* MBEDTLS_ASN1_WRITE_C */
#if defined(MBEDTLS_BASE64_C)
    "MBEDTLS_BASE64_C",
#endif /* MBEDTLS_BASE64_C */
#if defined(MBEDTLS_BIGNUM_C)
    "MBEDTLS_BIGNUM_C",
#endif /* MBEDTLS_BIGNUM_C */
#if defined(MBEDTLS_BLOWFISH_C)
    "MBEDTLS_BLOWFISH_C",
#endif /* MBEDTLS_BLOWFISH_C */
#if defined(MBEDTLS_CAMELLIA_C)
    "MBEDTLS_CAMELLIA_C",
#endif /* MBEDTLS_CAMELLIA_C */
#if defined(MBEDTLS_ARIA_C)
    "MBEDTLS_ARIA_C",
#endif /* MBEDTLS_ARIA_C */
#if defined(MBEDTLS_CCM_C)
    "MBEDTLS_CCM_C",
#endif /* MBEDTLS_CCM_C */
#if defined(MBEDTLS_CERTS_C)
    "MBEDTLS_CERTS_C",
#endif /* MBEDTLS_CERTS_C */
#if defined(MBEDTLS_CHACHA20_C)
    "MBEDTLS_CHACHA20_C",
#endif /* MBEDTLS_CHACHA20_C */
#if defined(MBEDTLS_CHACHAPOLY_C)
    "MBEDTLS_CHACHAPOLY_C",
#endif /* MBEDTLS_CHACHAPOLY_C */
#if defined(MBEDTLS_CIPHER_C)
    "MBEDTLS_CIPHER_C",
#endif /* MBEDTLS_CIPHER_C */
#if defined(MBEDTLS_CMAC_C)
    "MBEDTLS_CMAC_C",
#endif /* MBEDTLS_CMAC_C */
#if defined(MBEDTLS_CTR_DRBG_C)
    "MBEDTLS_CTR_DRBG_C",
#endif /* MBEDTLS_CTR_DRBG_C */
#if defined(MBEDTLS_DEBUG_C)
    "MBEDTLS_DEBUG_C",
#endif /* MBEDTLS_DEBUG_C */
#if defined(MBEDTLS_DES_C)
    "MBEDTLS_DES_C",
#endif /* MBEDTLS_DES_C */
#if defined(MBEDTLS_DHM_C)
    "MBEDTLS_DHM_C",
#endif /* MBEDTLS_DHM_C */
#if defined(MBEDTLS_ECDH_C)
    "MBEDTLS_ECDH_C",
#endif /* MBEDTLS_ECDH_C */
#if defined(MBEDTLS_ECDSA_C)
    "MBEDTLS_ECDSA_C",
#endif /* MBEDTLS_ECDSA_C */
#if defined(MBEDTLS_ECJPAKE_C)
    "MBEDTLS_ECJPAKE_C",
#endif /* MBEDTLS_ECJPAKE_C */
#if defined(MBEDTLS_ECP_C)
    "MBEDTLS_ECP_C",
#endif /* MBEDTLS_ECP_C */
#if defined(MBEDTLS_ENTROPY_C)
    "MBEDTLS_ENTROPY_C",
#endif /* MBEDTLS_ENTROPY_C */
#if defined(MBEDTLS_ERROR_C)
    "MBEDTLS_ERROR_C",
#endif /* MBEDTLS_ERROR_C */
#if defined(MBEDTLS_GCM_C)
    "MBEDTLS_GCM_C",
#endif /* MBEDTLS_GCM_C */
#if defined(MBEDTLS_HAVEGE_C)
    "MBEDTLS_HAVEGE_C",
#endif /* MBEDTLS_HAVEGE_C */
#if defined(MBEDTLS_HKDF_C)
    "MBEDTLS_HKDF_C",
#endif /* MBEDTLS_HKDF_C */
#if defined(MBEDTLS_HMAC_DRBG_C)
    "MBEDTLS_HMAC_DRBG_C",
#endif /* MBEDTLS_HMAC_DRBG_C */
#if defined(MBEDTLS_NIST_KW_C)
    "MBEDTLS_NIST_KW_C",
#endif /* MBEDTLS_NIST_KW_C */
#if defined(MBEDTLS_MD_C)
    "MBEDTLS_MD_C",
#endif /* MBEDTLS_MD_C */
#if defined(MBEDTLS_MD2_C)
    "MBEDTLS_MD2_C",
#endif /* MBEDTLS_MD2_C */
#if defined(MBEDTLS_MD4_C)
    "MBEDTLS_MD4_C",
#endif /* MBEDTLS_MD4_C */
#if defined(MBEDTLS_MD5_C)
    "MBEDTLS_MD5_C",
#endif /* MBEDTLS_MD5_C */
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    "MBEDTLS_MEMORY_BUFFER_ALLOC_C",
#endif /* MBEDTLS_MEMORY_BUFFER_ALLOC_C */
#if defined(MBEDTLS_NET_C)
    "MBEDTLS_NET_C",
#endif /* MBEDTLS_NET_C */
#if defined(MBEDTLS_OID_C)
    "MBEDTLS_OID_C",
#endif /* MBEDTLS_OID_C */
#if defined(MBEDTLS_PADLOCK_C)
    "MBEDTLS_PADLOCK_C",
#endif /* MBEDTLS_PADLOCK_C */
#if defined(MBEDTLS_PEM_PARSE_C)
    "MBEDTLS_PEM_PARSE_C",
#endif /* MBEDTLS_PEM_PARSE_C */
#if defined(MBEDTLS_PEM_WRITE_C)
    "MBEDTLS_PEM_WRITE_C",
#endif /* MBEDTLS_PEM_WRITE_C */
#if defined(MBEDTLS_PK_C)
    "MBEDTLS_PK_C",
#endif /* MBEDTLS_PK_C */
#if defined(MBEDTLS_PK_PARSE_C)
    "MBEDTLS_PK_PARSE_C",
#endif /* MBEDTLS_PK_PARSE_C */
#if defined(MBEDTLS_PK_WRITE_C)
    "MBEDTLS_PK_WRITE_C",
#endif /* MBEDTLS_PK_WRITE_C */
#if defined(MBEDTLS_PKCS5_C)
    "MBEDTLS_PKCS5_C",
#endif /* MBEDTLS_PKCS5_C */
#if defined(MBEDTLS_PKCS11_C)
    "MBEDTLS_PKCS11_C",
#endif /* MBEDTLS_PKCS11_C */
#if defined(MBEDTLS_PKCS12_C)
    "MBEDTLS_PKCS12_C",
#endif /* MBEDTLS_PKCS12_C */
#if defined(MBEDTLS_PLATFORM_C)
    "MBEDTLS_PLATFORM_C",
#endif /* MBEDTLS_PLATFORM_C */
#if defined(MBEDTLS_POLY1305_C)
    "MBEDTLS_POLY1305_C",
#endif /* MBEDTLS_POLY1305_C */
#if defined(MBEDTLS_RIPEMD160_C)
    "MBEDTLS_RIPEMD160_C",
#endif /* MBEDTLS_RIPEMD160_C */
#if defined(MBEDTLS_RSA_C)
    "MBEDTLS_RSA_C",
#endif /* MBEDTLS_RSA_C */
#if defined(MBEDTLS_SHA1_C)
    "MBEDTLS_SHA1_C",
#endif /* MBEDTLS_SHA1_C */
#if defined(MBEDTLS_SHA256_C)
    "MBEDTLS_SHA256_C",
#endif /* MBEDTLS_SHA256_C */
#if defined(MBEDTLS_SHA512_C)
    "MBEDTLS_SHA512_C",
#endif /* MBEDTLS_SHA512_C */
#if defined(MBEDTLS_SSL_CACHE_C)
    "MBEDTLS_SSL_CACHE_C",
#endif /* MBEDTLS_SSL_CACHE_C */
#if defined(MBEDTLS_SSL_COOKIE_C)
    "MBEDTLS_SSL_COOKIE_C",
#endif /* MBEDTLS_SSL_COOKIE_C */
#if defined(MBEDTLS_SSL_TICKET_C)
    "MBEDTLS_SSL_TICKET_C",
#endif /* MBEDTLS_SSL_TICKET_C */
#if defined(MBEDTLS_SSL_CLI_C)
    "MBEDTLS_SSL_CLI_C",
#endif /* MBEDTLS_SSL_CLI_C */
#if defined(MBEDTLS_SSL_SRV_C)
    "MBEDTLS_SSL_SRV_C",
#endif /* MBEDTLS_SSL_SRV_C */
#if defined(MBEDTLS_SSL_TLS_C)
    "MBEDTLS_SSL_TLS_C",
#endif /* MBEDTLS_SSL_TLS_C */
#if defined(MBEDTLS_THREADING_C)
    "MBEDTLS_THREADING_C",
#endif /* MBEDTLS_THREADING_C */
#if defined(MBEDTLS_TIMING_C)
    "MBEDTLS_TIMING_C",
#endif /* MBEDTLS_TIMING_C */
#if defined(MBEDTLS_VERSION_C)
    "MBEDTLS_VERSION_C",
#endif /* MBEDTLS_VERSION_C */
#if defined(MBEDTLS_X509_USE_C)
    "MBEDTLS_X509_USE_C",
#endif /* MBEDTLS_X509_USE_C */
#if defined(MBEDTLS_X509_CRT_PARSE_C)
    "MBEDTLS_X509_CRT_PARSE_C",
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#if defined(MBEDTLS_X509_CRL_PARSE_C)
    "MBEDTLS_X509_CRL_PARSE_C",
#endif /* MBEDTLS_X509_CRL_PARSE_C */
#if defined(MBEDTLS_X509_CSR_PARSE_C)
    "MBEDTLS_X509_CSR_PARSE_C",
#endif /* MBEDTLS_X509_CSR_PARSE_C */
#if defined(MBEDTLS_X509_CREATE_C)
    "MBEDTLS_X509_CREATE_C",
#endif /* MBEDTLS_X509_CREATE_C */
#if defined(MBEDTLS_X509_CRT_WRITE_C)
    "MBEDTLS_X509_CRT_WRITE_C",
#endif /* MBEDTLS_X509_CRT_WRITE_C */
#if defined(MBEDTLS_X509_CSR_WRITE_C)
    "MBEDTLS_X509_CSR_WRITE_C",
#endif /* MBEDTLS_X509_CSR_WRITE_C */
#if defined(MBEDTLS_XTEA_C)
    "MBEDTLS_XTEA_C",
#endif /* MBEDTLS_XTEA_C */
#endif /* MBEDTLS_VERSION_FEATURES */
    NULL
};
#endif

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_printf     printf
#define mbedtls_snprintf   snprintf
#define mbedtls_exit       exit
#define MBEDTLS_EXIT_SUCCESS EXIT_SUCCESS
#define MBEDTLS_EXIT_FAILURE EXIT_FAILURE
#endif

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

#define AM_MBEDTLS_HEAP_SIZE (256*1024/4)
uint32_t g_ui32MbedTLSHeap[AM_MBEDTLS_HEAP_SIZE];

#if defined(MBEDTLS_CHECK_PARAMS)
#include "mbedtls/platform_util.h"
void mbedtls_param_failed( const char *failure_condition,
                           const char *file,
                           int line )
{
    mbedtls_printf( "%s:%i: Input param failed - %s\n",
                    file, line, failure_condition );
    mbedtls_exit( MBEDTLS_EXIT_FAILURE );
}
#endif

#if defined(MBEDTLS_MODULE_PRINT)
int
mbedtls_version_check_features( const char *feature )
{
    const char **idx = features;
    char version_string[64];

    mbedtls_version_get_string_full(version_string);
    mbedtls_printf("%s\n", version_string);

    if ( *idx == NULL )
    {
        return( -2 );
    }

    if ( feature == NULL )
    {
        return( -1 );
    }

    while( *idx != NULL )
    {
        mbedtls_printf("    -> ");
        mbedtls_printf(*idx);
        mbedtls_printf("\n");
        idx++;
    }
    return( -1 );
}
#endif
/*
 * Check if a seed file is present, and if not create one for the entropy
 * self-test. If this fails, we attempt the test anyway, so no error is passed
 * back.
 */
#if defined(MBEDTLS_SELF_TEST) && defined(MBEDTLS_ENTROPY_C)
#if defined(MBEDTLS_ENTROPY_NV_SEED) && !defined(MBEDTLS_NO_PLATFORM_ENTROPY)
static void
create_entropy_seed_file( void )
{
    int result;
    size_t output_len = 0;
    unsigned char seed_value[MBEDTLS_ENTROPY_BLOCK_SIZE];

    /* Attempt to read the entropy seed file. If this fails - attempt to write
     * to the file to ensure one is present. */
    result = mbedtls_platform_std_nv_seed_read( seed_value,
                                                    MBEDTLS_ENTROPY_BLOCK_SIZE );
    if ( 0 == result )
    {
        return;
    }

    result = mbedtls_platform_entropy_poll( NULL,
                                            seed_value,
                                            MBEDTLS_ENTROPY_BLOCK_SIZE,
                                            &output_len );
    if ( 0 != result )
    {
        return;
    }

    if ( MBEDTLS_ENTROPY_BLOCK_SIZE != output_len )
    {
        return;
    }

    mbedtls_platform_std_nv_seed_write( seed_value, MBEDTLS_ENTROPY_BLOCK_SIZE );
}
#endif

int
mbedtls_entropy_self_test_wrapper( int verbose )
{
#if defined(MBEDTLS_ENTROPY_NV_SEED) && !defined(MBEDTLS_NO_PLATFORM_ENTROPY)
    create_entropy_seed_file( );
#endif
    return( mbedtls_entropy_self_test( verbose ) );
}
#endif

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
int
mbedtls_memory_buffer_alloc_free_and_self_test( int verbose )
{
    if ( verbose != 0 )
    {
#if defined(MBEDTLS_MEMORY_DEBUG)
        mbedtls_memory_buffer_alloc_status( );
#endif
    }
    mbedtls_memory_buffer_alloc_free( );
    return( mbedtls_memory_buffer_alloc_self_test( verbose ) );
}
#endif

typedef struct
{
    const char *name;
    int ( *function )( int );
} selftest_t;

const selftest_t selftests[] =
{
#if defined(MBEDTLS_MD2_C)
    {"md2", mbedtls_md2_self_test},
#endif
#if defined(MBEDTLS_MD4_C)
    {"md4", mbedtls_md4_self_test},
#endif
#if defined(MBEDTLS_MD5_C)
    {"md5", mbedtls_md5_self_test},
#endif
#if defined(MBEDTLS_RIPEMD160_C)
    {"ripemd160", mbedtls_ripemd160_self_test},
#endif
#if defined(MBEDTLS_SHA1_C)
    {"sha1", mbedtls_sha1_self_test},
#endif
#if defined(MBEDTLS_SHA256_C)
    {"sha256", mbedtls_sha256_self_test},
#endif
#if defined(MBEDTLS_SHA512_C)
    {"sha512", mbedtls_sha512_self_test},
#endif
#if defined(MBEDTLS_ARC4_C)
    {"arc4", mbedtls_arc4_self_test},
#endif
#if defined(MBEDTLS_DES_C)
    {"des", mbedtls_des_self_test},
#endif
#if defined(MBEDTLS_AES_C)
    {"aes", mbedtls_aes_self_test},
#endif
#if defined(MBEDTLS_GCM_C) && defined(MBEDTLS_AES_C)
    {"gcm", mbedtls_gcm_self_test},
#endif
#if defined(MBEDTLS_CCM_C) && defined(MBEDTLS_AES_C)
    {"ccm", mbedtls_ccm_self_test},
#endif
#if defined(MBEDTLS_NIST_KW_C) && defined(MBEDTLS_AES_C)
    {"nist_kw", mbedtls_nist_kw_self_test},
#endif
#if defined(MBEDTLS_CMAC_C)
    {"cmac", mbedtls_cmac_self_test},
#endif
#if defined(MBEDTLS_CHACHA20_C)
    {"chacha20", mbedtls_chacha20_self_test},
#endif
#if defined(MBEDTLS_POLY1305_C)
    {"poly1305", mbedtls_poly1305_self_test},
#endif
#if defined(MBEDTLS_CHACHAPOLY_C)
    {"chacha20-poly1305", mbedtls_chachapoly_self_test},
#endif
#if defined(MBEDTLS_BASE64_C)
    {"base64", mbedtls_base64_self_test},
#endif
#if defined(MBEDTLS_BIGNUM_C)
    {"mpi", mbedtls_mpi_self_test},
#endif
#if defined(MBEDTLS_RSA_C)
    {"rsa", mbedtls_rsa_self_test},
#endif
#if defined(MBEDTLS_X509_USE_C)
    {"x509", mbedtls_x509_self_test},
#endif
#if defined(MBEDTLS_XTEA_C)
    {"xtea", mbedtls_xtea_self_test},
#endif
#if defined(MBEDTLS_CAMELLIA_C)
    {"camellia", mbedtls_camellia_self_test},
#endif
#if defined(MBEDTLS_ARIA_C)
    {"aria", mbedtls_aria_self_test},
#endif
#if defined(MBEDTLS_CTR_DRBG_C)
    {"ctr_drbg", mbedtls_ctr_drbg_self_test},
#endif
#if defined(MBEDTLS_HMAC_DRBG_C)
    {"hmac_drbg", mbedtls_hmac_drbg_self_test},
#endif
#if defined(MBEDTLS_ECP_C)
    {"ecp", mbedtls_ecp_self_test},
#endif
#if defined(MBEDTLS_ECJPAKE_C)
    {"ecjpake", mbedtls_ecjpake_self_test},
#endif
#if defined(MBEDTLS_DHM_C)
    {"dhm", mbedtls_dhm_self_test},
#endif
#if defined(MBEDTLS_ENTROPY_C)
    {"entropy", mbedtls_entropy_self_test_wrapper},
#endif
#if defined(MBEDTLS_PKCS5_C)
    {"pkcs5", mbedtls_pkcs5_self_test},
#endif
/* Slower test after the faster ones */
#if defined(MBEDTLS_TIMING_C)
    {"timing", mbedtls_timing_self_test},
#endif
/* Heap test comes last */
#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
    {"memory_buffer_alloc", mbedtls_memory_buffer_alloc_free_and_self_test},
#endif
    {NULL, NULL}
};
#endif /* MBEDTLS_SELF_TEST */

mbedtls_ctr_drbg_context RndState;
mbedtls_entropy_context MbedtlsEntropy;
CCRndContext_t RndContex;
CCRndWorkBuff_t RndWorkBuff;

#if defined(MBEDTLS_X509_USE_C)
am_hal_rtc_time_t am_util_rtc_time;
struct tm gm_time;
#endif

int
main( int argc, char *argv[] )
{
    const selftest_t *test;
    int v = 1; /* v=1 for verbose mode */
    int suites_tested = 0, suites_failed = 0;
    void *pointer;
    bool bOTPEnabled;
    uint32_t ui32Status;

    /* init Rnd context's inner member */
    RndContex.rndState = &RndState;
    RndContex.entropyCtx = &MbedtlsEntropy;

    mbedtls_platform_set_printf((int (*)( const char *, ... ))am_util_stdio_printf);

    //
    // This will also power down crypto
    //
    am_bsp_low_power_init();

    //
    // Enable printing
    //
    am_bsp_debug_printf_enable();
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Crypto Self Test Example!\n\n");

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

    //
    // One time library initialization - to be done at the beginning of program
    //
    am_util_stdio_printf("Doing one time Init!\n\n");

#if defined(MBEDTLS_X509_USE_C)
    am_util_stdio_printf("Initializing RTC for x509!\n");

    //
    // The RTC is initialized from 2/28/2000
    //
    am_util_rtc_time.ui32Hour = 23;
    am_util_rtc_time.ui32Minute = 59;
    am_util_rtc_time.ui32Second = 55;
    am_util_rtc_time.ui32Hundredths = 0;
    am_util_rtc_time.ui32Weekday = 2;
    am_util_rtc_time.ui32DayOfMonth = 28;
    am_util_rtc_time.ui32Month = 2;
    am_util_rtc_time.ui32Year = 25;

    //
    // Century Bit is for tracking 2000s - 2100s
    //
    // 2000s  -> ui32CenturyBit == 1
    // 2100s  -> ui32CenturyBit == 0
    //
    am_util_rtc_time.ui32CenturyBit = RTC_CTRUP_CB_2000;

    //
    // Check the validity of the input values
    //
    if (am_hal_rtc_time_set(&am_util_rtc_time))
    {
        am_util_stdio_printf("Invalid Time Input Value\n");
        return 1;
    }

    am_util_crypto_ambiq_time_structure_set(&am_util_rtc_time);
    am_util_crypto_gm_time_structure_set(&gm_time);
    mbedtls_platform_set_time(&am_util_crypto_get_time);
#endif

    //
    // Initiailize MbedTLS
    //
    mbedtls_memory_buffer_alloc_init((uint8_t*)g_ui32MbedTLSHeap, AM_MBEDTLS_HEAP_SIZE);
    CC_LibInit(&RndContex, &RndWorkBuff);

    /*
     * The C standard doesn't guarantee that all-bits-0 is the representation
     * of a NULL pointer. We do however use that in our code for initializing
     * structures, which should work on every modern platform. Let's be sure.
     */
    memset( &pointer, 0, sizeof( void * ) );
    if ( pointer != NULL )
    {
        mbedtls_printf( "all-bits-zero is not a NULL pointer\n" );
        mbedtls_exit( MBEDTLS_EXIT_FAILURE );
    }

#if defined(MBEDTLS_MODULE_PRINT)
    mbedtls_printf("\nOutputting Version Information..\n\n");
    mbedtls_version_check_features(*features);
#endif

    mbedtls_printf("\nStarting Test Suite!\n\n");

#if defined(MBEDTLS_SELF_TEST)
    for ( test = selftests; test->name != NULL; test++ )
    {
        if ( test->function( v )  != 0 )
        {
            suites_failed++;
        }
        suites_tested++;
    }
#endif

    if ( v != 0 )
    {
        mbedtls_printf( "  Executed %d test suites\n\n", suites_tested );

        if ( suites_failed > 0 )
        {
            mbedtls_printf( "  [ %d tests FAIL ]\n\n", suites_failed );
        }
        else
        {
            mbedtls_printf( "  [ All tests PASS ]\n\n" );
        }
    }

    while(1);
}

