Name:
=====
 lvgl_test


Description:
============
 LVGL Widget Testing and GPU Acceleration Example




Purpose:
========
This example demonstrates testing of LVGL widgets
 with GPU acceleration using NemaGFX. The application showcases fundamental
 widget rendering with hardware acceleration and supports various texture
 formats. The example provides a flexible testing framework where different
 widgets and texture formats can be selected via macro definitions in
 gui_tasks.c for validation and performance testing.

Section: Key Features
=====================

 1. Widget Testing: testing of fundamental LVGL widgets
    with GPU acceleration for performance validation

 2. GPU Acceleration: Utilizes NemaGFX GPU for improved widget
    rendering performance and visual quality

 3. Multiple Texture Formats: Supports various texture formats for
    flexible graphics testing and optimization

 4. Configurable Testing: Allows switching between different widgets
    and texture formats via macro definitions

 5. PSRAM Integration: Utilizes external PSRAM for large texture
    assets and frame buffers

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for widget visualization
 - PSRAM for texture storage and frame buffers

Section: Usage
==============

 The application automatically tests LVGL widgets with GPU acceleration.
 Modify macro definitions in gui_tasks.c to switch between different
 widgets and texture formats for testing.

 NOTE: Different widgets and texture formats can be selected via macros



******************************************************************************


