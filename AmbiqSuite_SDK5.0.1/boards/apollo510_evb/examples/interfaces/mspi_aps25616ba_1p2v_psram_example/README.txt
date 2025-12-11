Name:
=====
 mspi_hex_ddr_aps25616ba_psram_example


Description:
============
 Example of the MSPI operation with DDR HEX SPI PSRAM APS25616BA.



Purpose:
========
This example demonstrates MSPI (Multi-Serial Peripheral
 Interface) DDR HEX operation with APS25616BA PSRAM device for high-performance
 memory applications. The application showcases DDR (Double Data Rate)
 memory operations, XIP (Execute In Place) functionality, and cache enhancement
 features.

Section: Key Features
=====================

 1. DDR HEX PSRAM: Implements DDR HEX operation with APS25616BA
    PSRAM device for high-performance memory applications

 2. XIP Functionality: Provides Execute In Place functionality for
    direct code execution from external PSRAM

 3. DMA Operations: Supports DMA read/write operations for efficient
    data transfer and memory management

 4. Cache Enhancement: Leverages ARM CM55 cache and MSPI CPU read
    combine features for enhanced performance

 5. Power Management: Implements power saving features with
    configurable sleep modes for 1.2V operation

Section: Functionality
======================

 The application performs the following operations:
 - Initializes MSPI with DDR HEX PSRAM configuration for 1.2V operation
 - Implements DDR DMA read/write operations with verification
 - Provides XIP read/write functionality for external memory
 - Supports function execution from external PSRAM
 - Manages power saving and sleep mode operations
 - Implements external memory management

Section: Usage
==============

 1. Compile and download the application to target device
 2. Monitor ITM/SWO output for PSRAM operations and status
 3. Observe DDR DMA and XIP read/write operations
 4. Test function execution from external PSRAM
 5. Verify high-performance memory functionality with 1.2V operation

Section: Configuration
======================

 - AM_DEBUG_PRINTF: Enable debug printing (optional)
 - ITM/SWO: Output for debug messages (1MHz)
 - MSPI_BUFFER_SIZE: Configurable buffer size (4KB default)
 - HALFSLEEP_TEST: Enable half-sleep testing (optional)

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


