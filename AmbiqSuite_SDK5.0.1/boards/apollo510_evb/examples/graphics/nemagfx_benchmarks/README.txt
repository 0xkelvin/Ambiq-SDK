Name:
=====
 nemagfx_benchmarks


Description:
============
 NemaGFX Performance Benchmarking Suite




Purpose:
========
This example provides performance benchmarking for
 the NemaGFX GPU and CPU operations. The benchmark suite measures FPS (Frames
 Per Second) for various graphics operations including shape rendering,
 texture operations, and complex graphics transformations.

Section: Key Features
=====================

 1. GPU Performance Measurement: benchmarking of GPU operations
    including triangle filling, rectangle rendering, and texture blitting

 2. CPU-GPU Coordination: Tests both CPU-bound and GPU-bound execution modes
    to evaluate system performance under different workloads

 3. Memory Bandwidth Testing: Evaluates burst length configurations and
    memory access patterns for optimal performance tuning

 4. Real-time FPS Monitoring: Provides accurate timing measurements using
    hardware timers for precise performance analysis

 5. Multiple Test Categories: Includes tests for:
    - Basic shape rendering (triangles, rectangles, quads)
    - Line and circle drawing operations
    - Texture blitting with various blend modes
    - String rendering and text operations
    - Rotation and transformation operations

Section: Usage
==============

 The benchmark automatically runs through all test categories and reports
 FPS results for each operation. Results are displayed via ITM debug output
 when AM_DEBUG_PRINTF is enabled.

 NOTE: Performance results depend on hardware configuration and clock settings



******************************************************************************


