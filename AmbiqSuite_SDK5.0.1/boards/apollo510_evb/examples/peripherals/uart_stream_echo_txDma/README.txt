Name:
=====
 uart_stream_echo_tx_Dma


Description:
============
 A uart example that demonstrates the stream driver using TX DMA




Purpose:
========
This example demonstrates UART streaming driver
 functionality with TX DMA (Direct Memory Access) for high-performance
 serial communication. The application showcases UART transmission
 using DMA, interrupt-driven data handling, and real-time echo functionality.

Section: Key Features
=====================

 1. TX DMA Streaming: Demonstrates high-performance UART transmission
    using DMA for efficient data transfer and low CPU utilization

 2. Interrupt Driven Operation: Implements interrupt-based data
    handling for real-time transmission and reception

 3. Buffer Management: Provides buffer management with
    double buffering and circular queue support

 4. Callback Based Reception: Implements callback-based reception
    for efficient and responsive data handling

 5. Real Time Echo: Demonstrates real-time echo functionality for
    bidirectional UART communication and testing

Section: Functionality
======================

 The application performs the following operations:
 - Initializes UART with TX DMA configuration and buffer management
 - Implements interrupt-driven data transmission and reception
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
 - TX_DMA_QUEUE_TYPE: DMA queue type (double buffering or circular queue)
 - RX_CALLBACK_THRESHOLD: Threshold for callback-based reception
 - AM_DEBUG_PRINTF: Enables detailed debug output via SWO

 This driver expects the uart FIFOs be enabled.

 Similarly, the interrupt code will move received data into the Rx queue
 and the application periodically reads from the Rx queue.

 The Rx callback has been enabled here.
 Once the number of bytes in the Rx buffer
 exceeds the threshold, the callback is made (from the ISR). Optionally, the callback
 can be disabled and the rx buffer read via polling.
 This example will immediately retransmit the received data via TX DMA.

 The associated ISR handler am_hal_uart_interrupt_queue_service() will return
 status in a bitfield, suitable for use as a callback or polling.

 Default Configuration:
 By default, this example uses UART2. The I/O pins used are defined in the BSP
 file as AM_BSP_GPIO_UART2_TX and AM_BSP_GPIO_UART2_RX

 Configuration and Operation:
 - These examples require enabling Tx and Rx fifos and queues.

 To interact with these pins from a PC, the user should obtain a 1.8v uart/usb
 cable (FTDI, etc.).

 to test use the provided python program ser_echo_test.py
 or test with a serial terminal by manually typing a block of data 64 to 256 bytes
 and observe the echo once the internal RX callback size has been reached.
 The swo output will report status during example operation.<

 The SWO output will send Rx/Tx status and error information.
 SWO Printing takes place over the ITM at 1MHz.


******************************************************************************


