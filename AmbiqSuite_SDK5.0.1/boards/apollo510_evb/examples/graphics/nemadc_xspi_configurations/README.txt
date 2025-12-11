Name:
=====
 nemadc_xspi_configurations


Description:
============
 NemaDC XSPI Interface Configuration Example




Purpose:
========
This example demonstrates how to configure and drive various
 SPI-based display panels using NemaDC. The application supports SPI4,
 DSPI (Dual-SPI), and QSPI (Quad-SPI) interfaces with configurable
 clock polarity and phase settings. It also showcases dynamic power
 control of the display controller for power management.

Section: Key Features
=====================

 1. SPI4 Interface: 4-wire SPI with CSX, CLK, DATA, and DCX signals
    for standard SPI display communication

 2. DSPI Interface: 1P1T 2-wire Dual-SPI with CSX, CLK, DATA0,
    and DATA1 for enhanced data throughput

 3. QSPI Interface: Quad-SPI with CSX, CLK, and DATA0-3 for
    maximum data transfer rates

 4. Clock Configuration: Configurable clock polarity and phase
    settings for compatibility with various display panels

 5. Dynamic Power Control: Demonstrates power management of the
    display controller for energy efficiency

Section: Operation Modes
========================

 - Test Pattern Mode (TESTMODE_EN=1): Displays test patterns for
   interface validation and debugging
 - Image Display Mode (TESTMODE_EN=0): Shows actual images for
   normal operation demonstration

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - SPI-compatible display panel (SPI4/DSPI/QSPI)
 - Proper signal routing for SPI interface signals

Section: Usage
==============

 The application automatically configures the selected SPI interface
 and displays content based on the mode setting. Use TESTMODE_EN to
 switch between test patterns and image display modes.

 NOTE: Layer size must be a multiple of 4 for Apollo5 compatibility



******************************************************************************


