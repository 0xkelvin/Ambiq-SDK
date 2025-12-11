Name:
=====
 coremark_best_performance


Description:
============
 EEMBC COREMARK test.




Purpose:
========
This example demonstrates EEMBC COREMARK benchmark
 testing for performance optimization. The application showcases
 performance benchmarking with multiple algorithms, self-checking
 mechanisms, and optimized compilation settings.

Section: Key Features
=====================

 1. EEMBC COREMARK Benchmark: Implements official EEMBC COREMARK
    benchmark for performance measurement

 2. Multiple Algorithm Testing: Provides testing of list processing,
    matrix manipulation, state machine, and CRC algorithms

 3. Self Checking Mechanism: Implements 16-bit CRC verification
    for correct benchmark operation validation

 4. Performance Optimization: Runs with ELP ON,
    and HP mode for maximum performance

 5. Compiler Optimization: Supports optimized compilation settings
    for best results

Section: Functionality
======================

 The application performs the following operations:
 - Initializes EEMBC COREMARK benchmark with best performance settings
 - Implements multiple algorithm testing (list, matrix, state machine, CRC)
 - Provides self-checking mechanism with 16-bit CRC verification
 - Runs 2250 ITERATIONS for performance measurement
 - Supports optimized compilation for performance
 - Implements performance benchmarking features

Section: Usage
==============

 1. Compile and download the application to target device
 2. Monitor UART output at 115,200 BAUD for benchmark results
 3. Observe performance measurement and CRC verification
 4. Test multiple algorithm performance capabilities
 5. Evaluate performance optimization results

Section: Configuration
======================

 - COREMARK_BEST_PERFORMANCE: Best performance configuration macro
 - ELP ON: Enhanced Low Power mode enabled
 - HP Mode: High Performance mode for 250MHz operation

 EEMBC’s CoreMark® is a benchmark that measures the performance of
 microcontrollers (MCUs) and central processing units (CPUs) used in
 embedded systems. Replacing the antiquated Dhrystone benchmark, Coremark
 contains implementations of the following algorithms: list processing
 (find and sort), matrix manipulation (common matrix operations), state
 machine (determine if an input stream contains valid numbers), and CRC
 (cyclic redundancy check). It is designed to run on devices from 8-bit
 microcontrollers to 64-bit microprocessors.

 The CRC algorithm serves a dual function; it provides a workload commonly
 seen in embedded applications and ensures correct operation of the CoreMark
 benchmark, essentially providing a self-checking mechanism. Specifically,
 to verify correct operation, a 16-bit CRC is performed on the data
 contained in elements of the linked-list.

 To ensure compilers cannot pre-compute the results at compile time, every
 operation in the benchmark derives a value that is not available at compile
 time. Furthermore, all code used within the timed portion of the benchmark
 is part of the benchmark itself (no library calls).

 More info may be found at the [EEMBC CoreMark website](https://www.eembc.org/coremark/).

 The Coremark implementation performs 2250 ITERATIONS (specified in
 ambiq_core_config.h), which is plenty of time for correct operation
 of the benchmark.

 Once the run has completed, the UART is reenabled and the results printed.

 Text is output to the UART at 115,200 BAUD, 8 bit, no parity.
 Please note that text end-of-line is a newline (LF) character only.
 Therefore, the UART terminal must be set to simulate a CR/LF.

 For Arm6 compilation of M55, add the following options in the MDK project.
  MDK Compiler option in 'Misc Controls': -Omax
  MDK Linker   option in 'Misc Controls': -Omax
  Non-MDK Compiler option: -Omax
  Non-MDK Linker option  : -Omax --cpu=Cortex-M55 --lto

  The below documentation details how each coremark example i.e. coremark,
  coremark_best_cm_per_mJ, coremark_best_performance differs based on
  pre-compile macro defined.

  These macros are used to select different version of coremark as listed
  in ambiq_core_config.h i.e.

  COREMARK_DEFAULT(best power) runs ELP in retention, LP mode.
  COREMARK_BEST_PERFORMANCE runs ELP On, HP mode.
  COREMARK_BEST_CM_PER_mJ runs ELP On, LP mode.
  If there are any different configs to be run, they need to be updated under
  the respective conditional macro definitions.There are no restrictions as
  to which combinations can be run under each individual conditional macro.




******************************************************************************


