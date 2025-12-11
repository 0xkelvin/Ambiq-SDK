Name:
=====
 uart_stream_echo_rxDma


Description:
============
 A uart example that demonstrates the stream driver using RX DMA




Purpose:
========
This example demonstrates UART streaming driver
 functionality with RX DMA (Direct Memory Access) for high-performance
 serial communication. The application showcases UART reception
 using DMA, interrupt-driven data handling, and real-time echo functionality.

Section: Key Features
=====================

 1. RX DMA Streaming: Demonstrates high-performance UART reception
    using DMA for efficient data transfer and low CPU utilization

 2. Interrupt Driven Operation: Implements interrupt-based data
    handling for real-time reception and transmission

 3. Buffer Management: Provides buffer management with
    double buffering and circular queue support

 4. Callback Based Reception: Implements callback-based reception
    for efficient and responsive data handling

 5. Real Time Echo: Demonstrates real-time echo functionality for
    bidirectional UART communication

Section: Functionality
======================

 The application performs the following operations:
 - Initializes UART with RX DMA configuration and buffer management
 - Implements interrupt-driven data reception and transmission
 - Provides callback-based reception for efficient data handling
 - Demonstrates real-time echo functionality for bidirectional communication
 - Monitors UART status and provides debug output via SWO
 - Supports interaction with PC using UART/USB cable and terminal application

Section: Usage
==============

 1. Connect UART pins to PC using UART/USB cable (1.8V logic)
 2. Compile and download the application to target device
 3. Use provided Python script or serial terminal for testing
 4. Send data to device and observe real-time echo and status output
 5. Monitor SWO output for UART status and error information

Section: Configuration
======================

 - UART_BAUDRATE: Configurable UART communication rate (default: 3,000,000 baud)
 - RX_DMA_QUEUE_TYPE: DMA queue type (double buffering or circular queue)
 - RX_CALLBACK_THRESHOLD: Threshold for callback-based reception
 - AM_DEBUG_PRINTF: Enables detailed debug output via SWO

 This driver expects the uart fifos be enabled.

 Default Configuration:
 By default, this example uses UART2. The I/O pins used are defined in the BSP
 file as AM_BSP_GPIO_UART2_TX and AM_BSP_GPIO_UART2_RX

 Configuration and Operation:
 - These examples require enabling Tx and Rx fifos and queues.
 - It operates in a non-blocking manner using rx threshold callbacks and no tx callbacks.

 To interact with these pins from a PC, the user should obtain a 1.8v uart/usb
 cable (FTDI recommended especially for high-speed data).

 The user can test with the provided python file ser_echo_test.py or can
 test by manually typing a large block of characters via a uart terminal and
 observing the echo

 The SWO output can send Rx/Tx status and error information.
 Swo output can be enabled disabled using the booleans in streamErrorMessageMap[]
 SWO Printing takes place over the ITM at 1MHz.


******************************************************************************


