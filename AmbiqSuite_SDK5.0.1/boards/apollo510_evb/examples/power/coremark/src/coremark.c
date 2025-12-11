//*****************************************************************************
//
//! @file coremark.c
//!
//! @brief EEMBC COREMARK test.
//!
//! @addtogroup power_examples Power Examples
//!
//! @defgroup coremark EEMBC COREMARK Example
//! @ingroup power_examples
//! @{
//!
//! Purpose: This example demonstrates EEMBC CoreMark® benchmark
//! functionality for microcontroller performance evaluation. The application
//! showcases benchmarking with list processing, matrix manipulation,
//! state machine operations, and CRC calculations.
//!
//! @section coremark_features Key Features
//!
//! 1. @b EEMBC @b CoreMark @b Benchmark: Implements official EEMBC CoreMark®
//!    benchmark for microcontroller performance evaluation
//!
//! 2. @b Algorithm @b Implementation: Provides list processing, matrix
//!    manipulation, state machine, and CRC algorithms
//!
//! 3. @b Self @b Checking @b Mechanism: Implements 16-bit CRC verification
//!    for benchmark correctness validation
//!
//! 4. @b Compiler @b Independent: Ensures operations derive values not
//!    available at compile time for accurate benchmarking
//!
//! 5. @b Performance @b Measurement: Provides performance
//!    analysis for system optimization
//!
//! @section coremark_functionality Functionality
//!
//! The application performs the following operations:
//! - Executes EEMBC CoreMark® benchmark with 2250 iterations
//! - Implements list processing (find and sort) algorithms
//! - Provides matrix manipulation and state machine operations
//! - Performs CRC calculations for self-checking validation
//! - Outputs results via UART at 115,200 BAUD
//! - Supports performance analysis
//!
//! @section coremark_usage Usage
//!
//! 1. Compile with optimization flags -O3
//! 2. Download the application to target device
//! 3. Monitor UART output for benchmark results
//! 4. Verify CRC calculations for correctness
//! 5. Analyze performance metrics and optimization
//!
//! @section coremark_configuration Configuration
//!
//! - @b ITERATIONS: 2250 benchmark iterations (specified in ambiq_core_config.h)
//! - @b UART @b Output: 115,200 BAUD, 8-bit, no parity
//! - @b Compiler @b Options: -O3 for optimal performance
//! - @b Linker @b Options: -Omax --cpu=Cortex-M55 --lto
//!
//!  The below documentation details how each coremark example i.e. coremark,
//!  coremark_best_cm_per_mJ, coremark_best_performance differs based on
//!  pre-compile macro defined.
//!
//!  These macros are used to select different version of coremark as listed
//!  in ambiq_core_config.h i.e.
//!
//!  COREMARK_DEFAULT(best power) runs ELP in retention, LP mode.
//!  COREMARK_BEST_PERFORMANCE runs ELP On, HP mode.
//!  COREMARK_BEST_CM_PER_mJ runs ELP On, LP mode.
//!  If there are any different configs to be run, they need to be updated under
//!  the respective conditional macro definitions.There are no restrictions as
//!  to which combinations can be run under each individual conditional macro.
//!
//!
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

//
// Include a dummy function just to avoid the pedantic error, "ISO C forbids
// an empty translation unit".
//
void
am_coremark_avoidpedanticerror(void)
{
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

