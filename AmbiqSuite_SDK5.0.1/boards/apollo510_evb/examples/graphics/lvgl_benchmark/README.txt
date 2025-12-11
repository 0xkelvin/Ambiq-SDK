Name:
=====
 lvgl_benchmark


Description:
============
 LVGL Performance Benchmarking Suite




Purpose:
========
This example provides performance benchmarking for
 LVGL (Light and Versatile Graphics Library) operations. The benchmark tests
 various graphics operations including rectangles, borders, shadows, text,
 image blending, transformations, and blending modes with consistent metrics.

Section: Key Features
=====================

 1. Testing: Tests performance across multiple graphics
    operations including rectangles, borders, shadows, text, and images

 2. Opacity Testing: All tests are repeated with 50% opacity to evaluate
    alpha blending performance

 3. Repeatable Results: Uses pseudo-random number generation for consistent
    object positioning and sizing across test runs

 4. Real-time Display: Shows current test step title and previous step
    results on screen for immediate feedback

 5. PSRAM Integration: Optional PSRAM support for large graphics buffers
    and asset storage with timing optimization

Section: Configuration Options
==============================

 - USE_NON_PSRAM_MODE: Disables PSRAM initialization for systems without
   external memory
 - MSPI_PSRAM_TIMING_CHECK: Enables DDR timing scan for optimal PSRAM
   performance when PSRAM is used

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for benchmark visualization
 - PSRAM (optional) for large graphics buffers

Section: Usage
==============

 The benchmark automatically runs through all test categories and displays
 performance results in real-time. Results are shown on screen with test
 step information and timing metrics.

 NOTE: Benchmark results depend on hardware configuration and clock settings



******************************************************************************


