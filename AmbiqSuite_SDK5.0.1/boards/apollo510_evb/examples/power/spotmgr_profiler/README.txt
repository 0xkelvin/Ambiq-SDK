Name:
=====
 spotmgr_profiler


Description:
============
 Example demonstrating the operation of spotmgr_profiler.



Purpose:
========
This example demonstrates spotmgr (Spot Manager)
 profiler functionality for power management analysis and optimization.
 The application showcases power profiling capabilities with
 timestamp logging, change tracking, and power state analysis.

Section: Key Features
=====================

 1. Power Profiling: Implements power management
    profiling for power analysis and optimization

 2. Timestamp Logging: Provides timestamp-based logging for
    accurate power state change tracking and analysis

 3. Change Tracking: Implements change tracking functionality
    for power state transitions and stimulus monitoring

 4. Stream Accumulate Modes: Supports both stream and accumulate
    modes for flexible logging and analysis capabilities

 5. Power State Analysis: Provides detailed power state analysis
    for optimization and debugging purposes

Section: Functionality
======================

 The application performs the following operations:
 - Initializes spotmgr profiler with timestamp logging capabilities
 - Implements power state change tracking and analysis
 - Provides stream and accumulate logging modes
 - Manages power state transition monitoring
 - Supports power profiling functionality
 - Implements power management analysis features

Section: Usage
==============

 1. Compile and download the application to target device
 2. Ensure HAL is compiled with AM_HAL_SPOTMGR_PROFILING macro
 3. Monitor power state changes and profiling data
 4. Analyze power state transitions and stimulus events
 5. Optimize power management based on profiling results

Section: Configuration
======================

 - AM_HAL_SPOTMGR_PROFILING: Enable spotmgr profiling (required)
 - PROFILER_STREAM_MODE: Stream or accumulate logging mode
 - MAX_LOG_ENTRY: Maximum log entries for power state tracking
 - PROFILER_TIMESTAMP_TIMER: Timer instance for timestamp source

 Note: Make sure that HAL is compiled with macro AM_HAL_SPOTMGR_PROFILING
       defined.


******************************************************************************


