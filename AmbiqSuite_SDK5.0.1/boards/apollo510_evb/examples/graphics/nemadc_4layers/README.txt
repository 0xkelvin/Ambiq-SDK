Name:
=====
 nemadc_4layers


Description:
============
 NemaDC 4-Layer Overlay and Global Alpha Blending Example




Purpose:
========
This example demonstrates 4-layer overlay capabilities
 using NemaDC display controller with global alpha blending. The
 application showcases different color formats across layers: Layer 0
 (RGBA8888 red), Layer 1 (RGB24 green), Layer 2 (RGB565 blue), and
 Layer 3 (A8 pure alpha). Global alpha values are applied when
 NEMADC_FORCE_A or NEMADC_MODULATE_A features are enabled.

Section: Key Features
=====================

 1. 4-Layer Overlay: Implements simultaneous rendering of four
    independent layers with different color formats

 2. Global Alpha Blending: Demonstrates global alpha control
    with NEMADC_FORCE_A and NEMADC_MODULATE_A features

 3. Multiple Color Formats: Supports RGBA8888, RGB24, RGB565,
    and A8 formats for flexible content rendering

 4. Layer Positioning: Implements offset positioning for visual
    separation and depth perception

 5. Blending: Showcases complex blending operations
    with different alpha values and blend modes

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel supporting 4-layer overlay
 - Sufficient memory for multiple frame buffers

Section: Usage
==============

 The application automatically demonstrates 4-layer overlay with
 different color formats and global alpha blending. The example
 shows how to create complex visual effects using multiple layers.

 NOTE: Layer size must be a multiple of 4 for Apollo5 compatibility



******************************************************************************


