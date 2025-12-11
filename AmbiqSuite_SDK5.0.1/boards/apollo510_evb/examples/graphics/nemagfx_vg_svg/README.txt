Name:
=====
 nemagfx_vg_svg


Description:
============
 NemaGFX Vector Graphics SVG Rendering Example




Purpose:
========
This example demonstrates SVG (Scalable Vector Graphics)
 rendering capabilities using NemaGFX vector graphics library. The example
 showcases hardware-accelerated SVG rendering with custom memory management
 and MPU configuration for optimal performance.

Section: Key Features
=====================

 1. SVG Rendering: Supports full SVG specification rendering with
    hardware acceleration for complex vector graphics

 2. Custom Memory Management: Implements custom graphics heap allocation
    with configurable cache behavior for optimal GPU performance

 3. MPU Configuration: Demonstrates Memory Protection Unit setup for
    non-cacheable graphics memory regions

 4. High Performance Mode: Supports high-performance mode for optimal
    rendering performance on compatible hardware

 5. Scalable Graphics: Renders scalable vector graphics that maintain
    quality at any resolution without pixelation

Section: Configuration Options
==============================

 - NON_CACHEABLE_CL_RB_HEAP: Enables non-cacheable graphics heap for
   command list and ring buffer operations
 - NEMA_USE_CUSTOM_MALLOC: Enables custom memory allocation for graphics
   operations
 - RUN_IN_HP_MODE: Enables high-performance mode for optimal rendering

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for SVG rendering visualization
 - Sufficient memory for graphics heap and SVG processing

Section: Usage
==============

 The application automatically loads and renders SVG graphics using hardware
 acceleration. The example demonstrates various SVG features including
 complex shapes, gradients, and transformations.

 NOTE: Custom memory management may be required for optimal performance



******************************************************************************


