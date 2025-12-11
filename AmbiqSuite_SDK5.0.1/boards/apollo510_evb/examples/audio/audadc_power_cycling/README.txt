Name:
=====
 audadc_power_cycling


Description:
============
 This example tests AUDADC power cycling sequence.




Purpose:
========
This example demonstrates AUDADC power cycling
 functionality for audio data acquisition. The application showcases
 power management techniques for AUDADC, implementing proper
 power cycling sequences to ensure reliable operation and optimal power
 efficiency.

Section: Key Features
=====================

 1. AUDADC Power Cycling: Demonstrates proper power cycling sequence
    for AUDADC to ensure reliable operation and power management

 2. DMA Transfer Testing: Tests DMA operations with random duration
    to validate data transfer reliability under various conditions

 3. Error Detection: Monitors for DMA errors and power cycling issues
    to ensure operation and fault tolerance

 4. Continuous Operation: Runs indefinitely to stress test the AUDADC
    power cycling and DMA transfer mechanisms

 5. Hardware Validation: Provides testing of AUDADC
    hardware functionality and power management features

Section: Functionality
======================

 The application performs the following operations:
 - Implements proper AUDADC power cycling sequence
 - Configures DMA for efficient audio data transfer
 - Tests DMA operations with random duration periods
 - Monitors for errors and provides status reporting
 - Runs continuous stress testing for hardware validation
 - Reports status and results via SWO/ITM

Section: Usage
==============

 1. Compile and download the application to target device
 2. Monitor SWO output for power cycling status and results
 3. Observe continuous operation and error detection
 4. Validate AUDADC hardware functionality

Section: Configuration
======================

 - AUDADC_SAMPLE_BUF_SIZE: Configurable sample buffer size (default: 2400)
 - AUDADC_DATA_BUFFER_SIZE: Configurable data buffer size
 - SWO/ITM: Output for status and results (1MHz)

 Printing takes place over the SWO at 1MHz.



******************************************************************************


