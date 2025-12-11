Name:
=====
 mspi_hex_ddr_psram_bandwidth_example


Description:
============
 Example of the MSPI bandwidth test with DDR HEX SPI PSRAM.



Purpose:
========
This example demonstrates MSPI (Multi-Serial Peripheral
 Interface) bandwidth testing with DDR HEX SPI PSRAM for high-performance
 memory applications. The application showcases bandwidth measurement
 capabilities, cache enhancement features, and performance optimization.

Section: Key Features
=====================

 1. Bandwidth Testing: Implements MSPI bandwidth
    testing with DDR HEX SPI PSRAM for performance evaluation

 2. Cache Enhancement: Leverages ARM CM55 cache and MSPI CPU read
    combine features for enhanced bandwidth performance

 3. Multiple Test Modes: Provides DMA write/read, XIP memory copy,
    word access, and GPU bandwidth testing capabilities

 4. Performance Measurement: Implements accurate bandwidth measurement
    with timer-based performance evaluation

 5. Configurable Test Parameters: Supports configurable block sizes,
    burst sizes, and test loops for bandwidth analysis

Section: Functionality
======================

 The application performs the following operations:
 - Initializes MSPI bandwidth testing with DDR HEX SPI PSRAM
 - Implements DMA write/read bandwidth testing
 - Provides XIP memory copy bandwidth testing
 - Supports word access bandwidth testing
 - Manages GPU bandwidth testing capabilities
 - Implements bandwidth measurement and analysis

Section: Usage
==============

 1. Compile and download the application to target device
 2. Optionally enable debug printing with AM_DEBUG_PRINTF
 3. Monitor ITM/SWO output for bandwidth test results
 4. Test different bandwidth measurement modes
 5. Analyze performance results and optimization opportunities

Section: Configuration
======================

 - AM_DEBUG_PRINTF: Enable debug printing (optional)
 - ITM/SWO: Output for debug messages (1MHz)
 - MSPI_BURST_SIZE: Configurable burst size (32 bytes default)
 - BANDWIDTH_TEST_LOOP: Number of test loops for analysis

 Starting from Apollo510, the MSPI XIP read bandwidth is boosted by the
 ARM CM55 cache and the MSPI CPU read combine feature. By default,
 the CPU read queue is on(CPURQEN). Cache prefetch(RID3) and cache miss(RID2)
 requests deemed appropriate by MSPI hardware are combined and processed
 with a 2:1 ratio(GQARB) between general queue and CPU read queue.

 am_hal_mspi_cpu_read_burst_t default =
 {
     .eGQArbBais                         = AM_HAL_MSPI_GQARB_2_1_BAIS,
     .bCombineRID2CacheMissAccess        = true,
     .bCombineRID3CachePrefetchAccess    = true,
     .bCombineRID4ICacheAccess           = false,
     .bCPUReadQueueEnable                = true,
 }

Additional Information:
=======================
 To enable debug printing, add the following project-level macro definitions.

 AM_DEBUG_PRINTF

 When defined, debug messages will be sent over ITM/SWO at 1MHz.


******************************************************************************


