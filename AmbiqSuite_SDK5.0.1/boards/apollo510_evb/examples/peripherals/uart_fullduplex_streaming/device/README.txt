Name:
=====
 uart_fullduplex


Description:
============
 UART full-duplex device example for Apollo5 microcontrollers.




Purpose:
========
This example demonstrates full-duplex UART DMA driven communication.
          for Apollo5 microcontrollers. It enables reliable, bidirectional
          serial communication between two boards, with robust handshaking
          and error handling mechanisms.

Section: Key Features
=====================

 1. Full Duplex Communication: Simultaneous transmit and receive
 2. DMA Driven: Efficient real-time data handling
 3. Handshaking: Device ready signaling and coordination
 5. Multi-board Support: Pin mapping for multiple Apollo510 boards

Section: Functionality
======================

 - UART initialization with full-duplex and flow control
 - GPIO configuration for handshaking and device ready
 - DMA / Interrupt-driven data transmission and reception
 - Real-time bidirectional data exchange
 - Communication status monitoring and debug output

Section: Usage
==============

 1. Flash uart_fullduplex_controller to one Apollo510 board
 2. Flash uart_fullduplex to another Apollo510 board
 3. Connect boards according to pin mapping specifications (see SWO printout)
 4. Power on controller board first, then device board
 5. Monitor communication and data exchange between boards
 6. Since multiple boards are support the SWO output is the pin connection reference

Section: Configuration
======================

 - UART_BAUD_RATE: Configurable UART communication rate
 - GPIO_PIN_MAPPING: Board-specific pin configuration
 - INTERRUPT_PRIORITY: Configurable interrupt priority levels



 Apollo510_evb mapping

     CONTROLLER (uart_fullduplex_controller) DEVICE (uart_fullduplex)
     --------------------                    ----------------
     GPIO[61]  UART3 TX                      GPIO[62]  UART0 RX
     GPIO[62]  UART0 RX                      GPIO[61]  UART3 TX
     GPIO[05]  Device Ready INT (controller) GPIO[05]  Device Ready INT (device)
     GND                                     GND

 Apollo510b_evb mapping

     CONTROLLER (uart_fullduplex_controller) DEVICE (uart_fullduplex)
     --------------------                    ----------------
     GPIO[00]  UART0 TX                      GPIO[02]  UART1 RX
     GPIO[02]  UART1 RX                      GPIO[00]  UART0 TX
     GPIO[05]  Device Ready INT (controller) GPIO[05]  Device Ready INT (device)
     GND                                     GND




******************************************************************************


