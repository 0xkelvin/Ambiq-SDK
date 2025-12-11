Name:
=====
 nemagfx_watchface


Description:
============
 NemaGFX Watchface Example - Real-time Clock Display




Purpose:
========
This example demonstrates the creation of a watch face
 application using NemaGFX graphics library. The watch face features:

 - Real-time clock display with rotating hour, minute, and second hands
 - Background image rendering with proper scaling and positioning
 - SVG format weather icon display with dynamic updates
 - Power-optimized rendering with GPU and display power management

Section: Key Features
=====================

 1. Rotating Clock Hands: Uses raw NemaGFX APIs for smooth hand rotation
    with proper pivot point calculations and matrix transformations

 2. GPU Power Management: Implements intelligent GPU power cycling where
    the GPU powers off after command list execution and restarts for next frame

 3. Display Power Optimization: DC and DSI are powered up only during
    frame transfer operations and immediately shut down after completion

 4. Multi-threaded Architecture: Uses FreeRTOS tasks for rendering and
    display management with proper synchronization

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel with DSI interface
 - External power supply for stable operation

Section: Usage
==============

 The application automatically starts displaying the watch face upon boot.
 The clock hands update in real-time, and weather icons cycle periodically.

 NOTE: This example requires proper display configuration and external power
 for optimal performance and power management features.



******************************************************************************


