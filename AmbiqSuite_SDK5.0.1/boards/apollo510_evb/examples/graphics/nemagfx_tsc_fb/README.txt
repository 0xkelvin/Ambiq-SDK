Name:
=====
 nemagfx_tsc_fb


Description:
============
 NemaGFX TSC Frame Buffer Compression Example




Purpose:
========
This example demonstrates TSC (ThinkSilicon Compression) frame buffer
 compression technology. The program uses TSC4-compressed frame buffers during
 runtime, achieving significant memory savings with a compression ratio of 1:4.
 The demo displays a 400x400 TSC4 image while dramatically reducing RAM usage.

Section: Key Features
=====================

 1. TSC Compression: Implements ThinkSilicon compression technology
    for efficient frame buffer storage with 1:4 compression ratio

 2. Multiple TSC Formats: Supports various TSC formats including
    TSC4, TSC6, TSC6A, TSC12, and TSC12A for different quality/size trade-offs

 3. Memory Optimization: Significantly reduces frame buffer memory
    requirements while maintaining visual quality

 4. Hardware Acceleration: Uses GPU hardware acceleration for
    efficient compression and decompression operations

 5. Display Integration: Seamlessly integrates with display controller
    for real-time frame buffer management

Section: Supported TSC Formats
==============================

 - TSC4: 4-bit compression for maximum memory savings
 - TSC6: 6-bit compression for balanced quality and size
 - TSC6A: 6-bit compression with alpha support
 - TSC12: 12-bit compression for higher quality
 - TSC12A: 12-bit compression with alpha support

Section: Requirements
=====================

 - Frame buffer width and height must be 4-pixel aligned
 - MSPI display control is not supported with TSC format
 - Requires sufficient memory for compression/decompression operations

Section: Hardware Requirements
==============================

 - Compatible Development Board
 - Display panel supporting TSC formats
 - Sufficient memory for compressed frame buffer operations

Section: Usage
==============

 The application automatically loads TSC-compressed images and displays
 them using hardware-accelerated decompression. Memory usage is significantly
 reduced compared to uncompressed formats.

 NOTE: Debug output is available via ITM when AM_DEBUG_PRINTF is enabled



******************************************************************************


