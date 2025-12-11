Name:
=====
 mspi_octal_example


Description:
============
 Example of the MSPI operation with Octal SPI Flash.



Purpose:
========
This example demonstrates MSPI (Multi-Serial Peripheral
 Interface) octal SPI flash functionality for high-performance memory operations.
 The application showcases MSPI configuration with Octal DDR mode,
 timing optimization, and memory operations.

Section: Key Features
=====================

 1. Octal SPI Flash: Implements Octal DDR mode for high-speed
    flash memory operations with timing optimization

 2. Memory Operations: Provides erase, write, read,
    and XIP (Execute In Place) operations with verification

 3. Timing Optimization: Implements timing scan for optimal MSPI
    clock and chipset configuration

 4. DMA Support: Utilizes DMA for efficient data transfer during
    read/write operations

 5. XIP Functionality: Supports Execute In Place operations for
    direct code execution from flash memory

Section: Functionality
======================

 The application performs the following operations:
 - Initializes MSPI with timing optimization and configuration
 - Implements SDR DMA read/write operations with verification
 - Provides SDR XIP read/write functionality
 - Supports function call after XIP writes to FLASH
 - Manages flash memory operations
 - Implements MSPI configuration and optimization

Section: Usage
==============

 1. Compile and download the application to target device
 2. Monitor ITM/SWO output for MSPI operations and status
 3. Observe timing optimization and memory operations
 4. Test SDR DMA and XIP read/write functionality
 5. Verify flash memory operations and performance

Section: Configuration
======================

 - AM_DEBUG_PRINTF: Enable debug printing (optional)
 - ITM/SWO: Output for debug messages (1MHz)
 - MSPI_BUFFER_SIZE: Configurable buffer size (4KB default)
 - Cache Optimization: CPU read queue and cache combine features

 Initialize the test:
 1. Run a timing check to find the best timing for the MSPI clock and chipset
 2. Initialize the MSPI instance
 3. Apply the timing scan results

 Test SDR DMA R/W:
 1. Write known data to a buffer using DMA
 2. Read the data back into another buffer using DMA
 3. Compare the results from 1 and 2 immediately above

 Test SDR XIP R/W:
 1. Enable XIP
 2. Write known data to a buffer using XIP
 3. Read the data back into another buffer using XIP
 4. Compare the results from 2 and 3 immediately above

 Test Function Call after XIP writes function to FLASH:
 1. Place MSPI in Scrambling Mode
 2. Write test function to External FLASH
 3. Call function located in FLASH

 Deinitialize the MSPI and go to sleep

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


