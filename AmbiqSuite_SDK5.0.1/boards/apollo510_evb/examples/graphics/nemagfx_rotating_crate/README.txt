Name:
=====
 nemagfx_rotating_crate


Description:
============
 NemaGFX 3D Rotating Crate with Texture Rendering Example




Purpose:
========
This example demonstrates 3D graphics rendering with texture mapping
 using NemaGFX GPU acceleration. The application renders a rotating 3D crate
 with realistic texture mapping, showcasing hardware-accelerated 3D graphics
 capabilities that significantly reduce CPU load for frame buffer calculations.

Section: Key Features
=====================

 1. 3D Graphics Rendering: Implements full 3D cube rendering with
    perspective projection and matrix transformations

 2. Texture Mapping: Applies realistic textures to 3D surfaces using
    hardware-accelerated texture sampling and filtering

 3. Real-time Animation: Continuously rotates the crate on multiple axes
    with smooth, real-time updates

 4. Hardware Acceleration: Uses GPU for efficient 3D calculations and
    rendering, reducing CPU workload

 5. Dual Buffer Support: Implements frame buffer switching for smooth
    animation without tearing

Section: Technical Details
==========================

 - Uses 4x4 transformation matrices for 3D operations
 - Implements perspective projection with configurable field of view
 - Supports texture filtering and blending modes
 - Demonstrates triangle culling for performance optimization

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for 3D visualization
 - Sufficient memory for textures and frame buffers

Section: Usage
==============

 The application automatically renders a continuously rotating 3D crate
 with texture mapping. The crate rotates on multiple axes demonstrating
 smooth 3D graphics performance.

 NOTE: Debug output is available via ITM when AM_DEBUG_PRINTF is enabled



******************************************************************************


