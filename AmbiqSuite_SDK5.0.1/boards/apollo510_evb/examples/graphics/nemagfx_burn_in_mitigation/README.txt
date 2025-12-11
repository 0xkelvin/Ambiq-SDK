Name:
=====
 nemagfx_burn_in_mitigation


Description:
============
 NemaGFX Burn-In Mitigation and Mask Blending Example




Purpose:
========
This example demonstrates software-based burn-in mitigation
 techniques using NemaGFX GPU acceleration. The application blends a
 mask pattern over the display content to reduce the risk of image
 retention (burn-in) on OLED and other sensitive displays. Two blending
 approaches are shown: GPU-based blending with NEMA_TEX_REPEAT, and
 display controller (DC) blending using multiple hardware layers.

Section: Key Features
=====================

 1. Burn-In Mitigation: Applies a moving or static mask to prevent
    persistent image retention on displays

 2. GPU Mask Blending: Uses GPU to blend a small mask buffer across
    the framebuffer with NEMA_TEX_REPEAT for efficient coverage

 3. DC Layer Blending: Demonstrates blending a full mask using the
    display controller with at least two hardware layers

 4. Configurable Modes: Select between GPU and DC blending via macro
    definition for flexible demonstration

 5. Multiple Blend Modes: Supports various blend modes for mask
    application and visual effect tuning

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel susceptible to burn-in (e.g., OLED)
 - Sufficient memory for frame buffers and mask patterns

Section: Usage
==============

 The application automatically applies a mask over the display content
 using the selected blending method. Adjust macros to switch between GPU
 and DC blending. The effect demonstrates practical burn-in mitigation
 for embedded displays.

 NOTE: GPU_BLEND_MASK macro selects GPU or DC blending method.



******************************************************************************


