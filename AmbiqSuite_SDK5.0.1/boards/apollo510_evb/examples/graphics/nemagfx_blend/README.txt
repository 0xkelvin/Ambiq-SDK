Name:
=====
 nemagfx_blend


Description:
============
 NemaGFX Blending and Alpha Compositing Example




Purpose:
========
This example demonstrates blending capabilities
 using NemaGFX GPU acceleration. The application showcases various
 blending modes for combining source (foreground) and destination
 (background) color fragments to produce final colors. The example
 cycles through different blending modes every second, using a red
 destination (50% alpha) and blue source (50% alpha) to demonstrate
 the visual effects of each blending operation.

Section: Key Features
=====================

 1. Multiple Blend Modes: Demonstrates all supported blending
    operations including SIMPLE, CLEAR, SRC, SRC_OVER, DST_OVER,
    SRC_IN, DST_IN, SRC_OUT, DST_OUT, SRC_ATOP, DST_ATOP, ADD, XOR

 2. Alpha Blending: Shows how alpha values affect color mixing
    and transparency effects

 3. Real-Time Demonstration: Cycles through blend modes every
    second with RTC timer for consistent timing

 4. Hardware Acceleration: Utilizes NemaGFX GPU for efficient
    blending calculations

 5. Color Fragment Processing: Demonstrates the mathematical
    calculations between source and destination colors

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for blend visualization
 - Sufficient memory for frame buffers and textures

Section: Usage
==============

 The application automatically cycles through different blending modes
 every second, showing how red (50% alpha) and blue (50% alpha)
 colors combine. Output is available via ITM at 1MHz for detailed
 analysis of blending operations.

 NOTE: Debug output is available via ITM for blend mode analysis



******************************************************************************


