Name:
=====
 coremark_best_cm_per_mJ


Description:
============
 EEMBC COREMARK test optimized for CoreMark/mJ metric.




Purpose:
========
This example implements the EEMBC CoreMark benchmark optimized
          for maximum CoreMark score per millijoule (CM/mJ). It balances
          processing performance with power consumption to achieve optimal
          energy efficiency while executing industry-standard benchmark
          algorithms.

Section: Key Features
=====================

 1. Algorithm Suite: List processing, matrix operations, state machines
 2. Power Optimization: Balanced performance vs. power consumption
 3. Validation: Built-in CRC-based result verification
 4. Performance Metrics: CoreMark/MHz and CoreMark/mJ measurements
 5. Compiler Options: Optimized build settings for efficiency

Section: Functionality
======================

 - List find and sort operations
 - Matrix manipulation algorithms
 - State machine implementations
 - CRC result verification
 - Power consumption monitoring

Section: Usage
==============

 1. Build with recommended compiler options
 2. Load and run on target device
 3. Monitor performance metrics via UART
 4. Analyze CoreMark/mJ results
 5. Compare with standard CoreMark results

Section: Configuration
======================

 - Compiler settings (see MDK options)
 - Iteration count in ambiq_core_config.h
 - UART output at 115,200 baud
 - ELP and power mode selections

 For M55 compilation with Arm6:
 - MDK: -Omax in compiler/linker
 - Non-MDK: -Omax --cpu=Cortex-M55 --lto

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


