Name:
=====
 nemadc_scrolling_effect


Description:
============
 NemaDC Dual-Layer Scrolling Effect Example




Purpose:
========
This example demonstrates smooth scrolling effects using
 NemaDC display controller with dual-layer rendering. The application
 creates a seamless transition between two images by adjusting layer
 positions and blending modes. The scrolling effect is optimized for
 MIPI DSI interface and showcases the display controller's ability
 to handle complex layer management and real-time visual transitions.

Section: Key Features
=====================

 1. Dual-Layer Rendering: Uses two hardware layers to create
    smooth scrolling transitions between images

 2. Seamless Scrolling: Implements continuous scrolling with
    proper layer positioning and blending

 3. MIPI DSI Support: Optimized for MIPI DSI interface with
    efficient data transfer and timing

 4. Multiple Formats: Supports both TSC6 and RGB565 image formats
    for flexible content rendering

 5. Real-Time Animation: Continuously updates layer positions
    for smooth visual effects

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - MIPI DSI-compatible display panel
 - Sufficient memory for dual-layer frame buffers

Section: Usage
==============

 The application automatically creates a scrolling effect between
 two images using dual-layer rendering.

 NOTE: Layer size must be a multiple of 4 for Apollo5 compatibility



******************************************************************************


