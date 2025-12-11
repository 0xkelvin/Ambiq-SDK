Name:
=====
 nemagfx_vg_test


Description:
============
 NemaGFX Vector Graphics Test Suite




Purpose:
========
This example provides a test suite for all built-in
 NemaGFX Vector Graphics (VG) demos. The application includes multiple
 demonstration modules that showcase various VG capabilities including
 masking, painting, font rendering, shapes, transformations, and SVG operations.

Section: Available Demos
========================

 1. Masking Example (RUN_MASKING_EXAMPLE): Demonstrates masking features
    with NemaVG for complex clipping and overlay operations

 2. Paint Example (RUN_PAINT_EXAMPLE): Shows different paint features
    including gradients, patterns, and color fills

 3. Paint LUT Example (RUN_PAINT_LUT_EXAMPLE): Demonstrates LUT format
    texture usage in VG paint objects for color mapping

 4. Font Rendering (RUN_RENDER_VG_FONT): Shows TTF font rendering
    capabilities with NemaVG vector graphics

 5. Shapes (RUN_SHAPE): Displays various pre-defined shapes with
    different paint settings and transformations

 6. Text Transformation (RUN_TEXT_TRANSFORMATION): Demonstrates text
    object manipulation using different transform matrices

 7. TSVG Benchmark (RUN_TSVG_BENCHMARK): Performance benchmark using
    rotating tiger head SVG image

 8. TSVG Render (RUN_TSVG_RENDER_EXAMPLE): Renders SVG images containing
    both shapes and fonts

 9. TSVG Measure (RUN_TSVG_MEASURE): SVG measurement and analysis tools

 10. Raster Arc (RUN_RASTER_ARC): Arc rendering and rasterization

 11. Raw TTF (RUN_RENDER_RAW_TTF): Raw True Type Font rendering

 12. Joins and Caps (RUN_JOINS_CAPS): Line join and cap style rendering

 13. Dash Example (RUN_DASH_EXAMPLE): Stroke dash pattern rendering

Section: Configuration
======================

 Modify macro definitions in nemagfx_vg_test.h to select the desired demo.
 Only one demo can be enabled at a time for proper operation.

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - PSRAM for graphics buffer management
 - Display panel for VG demo visualization

Section: Usage
==============

 Configure the desired demo in the header file, then run the application
 to see the selected vector graphics demonstration in action.

 NOTE: Only one demo macro can be enabled at a time



******************************************************************************


