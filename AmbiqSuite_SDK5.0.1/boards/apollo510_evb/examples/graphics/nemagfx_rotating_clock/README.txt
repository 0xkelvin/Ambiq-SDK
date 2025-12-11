Name:
=====
 nemagfx_rotating_clock


Description:
============
 NemaGFX Rotating Clock - 3D Digital Clock Animation




Purpose:
========
This example demonstrates 3D graphics rendering using
 NemaGFX GPU with dual frame buffers for smooth animation. The application
 creates a digital rotating clock with continuous shader effects and
 real-time time updates.

Section: Key Features
=====================

 1. 3D Graphics Rendering: Implements 3D transformation matrices for
    perspective projection and model-view transformations

 2. Dual Frame Buffer: Uses two frame buffers for smooth animation
    without tearing or flickering

 3. Real-time Animation: Continuously updates clock display with
    smooth rotation and shader effects

 4. GPU Acceleration: Leverages NemaGFX GPU for hardware-accelerated
    3D graphics rendering and texture operations

 5. Mathematical Transformations: Implements complex 3D coordinate
    transformations including projection, rotation, and viewport mapping

Section: Technical Details
==========================

 - Uses 4x4 transformation matrices for 3D graphics operations
 - Implements perspective projection with configurable viewport
 - Supports real-time rotation with smooth interpolation
 - Uses texture mapping for number rendering

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for 3D clock visualization
 - Sufficient memory for dual frame buffers and textures

Section: Usage
==============

 The application automatically starts the rotating clock animation with
 real-time updates. The clock displays current time with smooth 3D rotation
 effects and continuous shader animations.

 NOTE: Debug output is available via ITM when AM_DEBUG_PRINTF is enabled



******************************************************************************


