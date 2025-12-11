Name:
=====
 nemagfx_transition_effects


Description:
============
 NemaGFX Transition Effects and Animation Example




Purpose:
========
This example demonstrates transition effects between two
 images using NemaGFX GPU acceleration. The application showcases various
 hardware-accelerated transition animations with smooth visual effects
 suitable for modern user interfaces.

Section: Key Features
=====================

 1. Horizontal Transitions: Linear, cube, inner cube, and stack effects
    for horizontal image transitions

 2. Vertical Transitions: Linear, cube, inner cube, and stack effects
    for vertical image transitions

 3. Fade Effects: Smooth fade and fade-zoom transitions for elegant
    image blending

 4. Hardware Acceleration: GPU-accelerated transition effects for
    smooth, real-time animations

 5. Dual Buffer Support: Implements both single and dual buffer modes
    for flexible rendering strategies

Section: Available Transitions
==============================

 - NEMA_TRANS_LINEAR_H/V: Simple linear slide transitions
 - NEMA_TRANS_CUBE_H/V: 3D cube rotation effects
 - NEMA_TRANS_INNERCUBE_H/V: Inner cube perspective effects
 - NEMA_TRANS_STACK_H/V: Stacked card transition effects
 - NEMA_TRANS_FADE: Smooth cross-fade between images
 - NEMA_TRANS_FADE_ZOOM: Fade with zoom effect
 - NEMA_TRANS_MAX/NONE: Special transition modes

Section: Configuration Options
==============================

 - BAREMETAL: Comment out to enable FreeRTOS support
 - AM_DEBUG_PRINTF: Enable for ITM debug output
 - FRAME_BUFFERS: Configure for single or dual buffer mode

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel for transition effect visualization
 - Sufficient memory for dual image buffers and effects

Section: Usage
==============

 The application automatically cycles through various transition effects
 between two sample images. Each transition demonstrates different
 animation techniques and visual effects.

 NOTE: Transition effects are hardware-accelerated for optimal performance



******************************************************************************


