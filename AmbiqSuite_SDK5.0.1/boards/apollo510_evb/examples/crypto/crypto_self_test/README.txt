Name:
=====
 crypto_self_test


Description:
============
 A simple example to demonstrate using runtime crypto APIs.


 This example initializes the runtime crypto library at the beginning
 and provides self-testing functionality for all mbedTLS
 cryptographic algorithms to validate their correct operation and
 implementation integrity.

 This example requires 20480 bytes of stack space.
 The stack size is declared in the ini file and startup file.
 The user should specify this size in the apollo510-system-config.yaml file

Section: Key Features
=====================

 1. Self Testing: Tests all mbedTLS cryptographic
    algorithms to ensure correct implementation and operation

 2. Multiple Algorithm Support: Includes AES, SHA, MD5, RSA, ECC,
    and other cryptographic algorithms for complete validation

 3. Runtime Crypto Library: Initializes and validates the runtime
    cryptographic library for secure operations

 4. Memory Buffer Allocation: Tests memory allocation and management
    for cryptographic operations

 5. Entropy Source Validation: Verifies entropy sources and random
    number generation for cryptographic security

Section: Functionality
======================

 The application performs the following operations:
 - Initializes mbedTLS cryptographic library and memory allocation
 - Executes self-tests for all cryptographic algorithms
 - Validates entropy sources and random number generation
 - Tests memory buffer allocation and management
 - Verifies implementation integrity of all crypto modules
 - Provides detailed test results and validation status
 - Implements power management for energy-efficient operation

Section: Usage
==============

 1. Compile and download the application to target device
 2. Monitor ITM/SWO output for self-test results
 3. Verify successful completion of all cryptographic self-tests
 4. Review validation status and implementation integrity

Section: Configuration
======================

 - MBEDTLS_MODULE_PRINT: Enables detailed module feature reporting
 - AM_DEBUG_PRINTF: Enables detailed debug output via ITM/SWO
 - VERBOSE_TESTING: Enables verbose output for detailed test results


******************************************************************************


