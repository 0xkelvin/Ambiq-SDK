Name:
=====
 nemagfx_font_render


Description:
============
 NemaGFX Font Rendering and Layout Example




Purpose:
========
This example demonstrates font rendering capabilities
 using NemaGFX GPU acceleration. The application showcases the display
 of fonts in multiple bit depths (1b, 2b, 4b, 8b), the impact of enabling
 font kerning, and various text layout options supported by nema_print.
 It also provides guidance for integrating custom fonts into projects.

Section: Key Features
=====================

 1. Multi-Bit Depth Fonts: Renders fonts in 1-bit, 2-bit, 4-bit, and
    8-bit formats to demonstrate quality and memory trade-offs

 2. Font Kerning: Shows the visual impact of enabling/disabling kerning
    for improved text appearance

 3. Text Layout: Demonstrates different text alignment and layout
    options available in the NemaGFX SDK

 4. Custom Font Integration: Provides instructions for converting and
    integrating custom fonts using the provided font conversion utility

 5. Hardware Acceleration: Utilizes GPU for efficient font rendering
    and layout calculations

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for font visualization
 - Sufficient memory for font bitmaps and frame buffers

Section: Usage
==============

 The application automatically renders sample text using different font
 bit depths, kerning settings, and layout options. For custom fonts,
 refer to the how_to_use.md in third_party/ThinkSi/NemaGFX_SDK/utils/fontConvert.

 NOTE: Debug output is available via ITM when AM_DEBUG_PRINTF is enabled



******************************************************************************


