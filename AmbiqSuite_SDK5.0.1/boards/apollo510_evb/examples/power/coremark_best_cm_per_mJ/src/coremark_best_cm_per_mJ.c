//*****************************************************************************
//
//! @file coremark_best_cm_per_mJ.c
//!
//! @brief EEMBC COREMARK test optimized for CoreMark/mJ metric.
//!
//! @addtogroup power_examples Power Examples
//!
//! @defgroup coremark_best_cm_per_mJ EEMBC COREMARK Best CM per mJ Example
//! @ingroup power_examples
//! @{
//!
//! Purpose: This example implements the EEMBC CoreMark benchmark optimized
//!          for maximum CoreMark score per millijoule (CM/mJ). It balances
//!          processing performance with power consumption to achieve optimal
//!          energy efficiency while executing industry-standard benchmark
//!          algorithms.
//!
//! @section coremark_cm_mj_features Key Features
//!
//! 1. @b Algorithm @b Suite: List processing, matrix operations, state machines
//! 2. @b Power @b Optimization: Balanced performance vs. power consumption
//! 3. @b Validation: Built-in CRC-based result verification
//! 4. @b Performance @b Metrics: CoreMark/MHz and CoreMark/mJ measurements
//! 5. @b Compiler @b Options: Optimized build settings for efficiency
//!
//! @section coremark_cm_mj_functionality Functionality
//!
//! - List find and sort operations
//! - Matrix manipulation algorithms
//! - State machine implementations
//! - CRC result verification
//! - Power consumption monitoring
//!
//! @section coremark_cm_mj_usage Usage
//!
//! 1. Build with recommended compiler options
//! 2. Load and run on target device
//! 3. Monitor performance metrics via UART
//! 4. Analyze CoreMark/mJ results
//! 5. Compare with standard CoreMark results
//!
//! @section coremark_cm_mj_configuration Configuration
//!
//! - Compiler settings (see MDK options)
//! - Iteration count in ambiq_core_config.h
//! - UART output at 115,200 baud
//! - ELP and power mode selections
//!
//! For M55 compilation with Arm6:
//! - MDK: -Omax in compiler/linker
//! - Non-MDK: -Omax --cpu=Cortex-M55 --lto
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

