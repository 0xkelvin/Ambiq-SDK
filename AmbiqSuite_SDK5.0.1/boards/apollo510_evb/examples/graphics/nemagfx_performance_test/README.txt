Name:
=====
 nemagfx_performance_test


Description:
============
 NemaGFX Performance Testing Suite




Purpose:
========
This example provides GPU performance testing for
 various NemaGFX operations including fill, copy, blend, scale, rotate,
 scale+rotate, perspective transformation, SVG image rendering, and TTF
 font rendering. The test suite generates detailed performance metrics
 for optimization and benchmarking purposes.

Section: Key Features
=====================

 1. Testing: Covers all major GPU operations including
    basic graphics, transformations, and rendering

 2. Configurable Test Suites: Multiple configuration files for different
    performance scenarios and optimization targets

 3. Detailed Metrics: Provides precise performance measurements for
    each operation with timing and throughput data

 4. CSV Output: Generates CSV-formatted results for easy analysis
    in spreadsheet applications

 5. PSRAM Integration: Tests GPU performance with external memory
    for realistic system performance evaluation

Section: Test Configurations
============================

 - config_base.h: Complete test suite with all operations
 - config_morton.h: Rotation performance with Morton/tiling settings
 - config_rotate.h: Rotation performance with various parameters
 - config_scale.h: Scaling performance optimization
 - config_scale_rotate.h: Affine transformation performance
 - config_write_param.h: GPU PSRAM write performance

Section: Usage Instructions
===========================

 1. Select desired test configuration in test_common.c (lines 54-59)
 2. Compile and download to target device
 3. Collect SWO output during test execution
 4. Save results as CSV file and import to Excel for analysis

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - PSRAM for realistic memory performance testing
 - Display panel for visual verification
 - SWO interface for performance data collection

 NOTE: PSRAM must be connected before running performance tests



******************************************************************************


