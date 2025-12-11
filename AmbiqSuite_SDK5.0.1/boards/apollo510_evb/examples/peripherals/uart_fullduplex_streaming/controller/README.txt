Name:
=====
 uart_fullduplex_controller


Description:
============
 This example demonstrates how apollo5 UART work in fullduplex mode.




Purpose:
========
This example demonstrates how apollo5 UART work in fullduplex mode.
 To run this example, the user needs two apollo510 boards and flash
 uart_fullduplex_controller and uart_fullduplex respectively for the controller and
 the device, then power on the controller first, then the device.


 Apollo510_evb mapping

     CONTROLLER (uart_fullduplex_controller) DEVICE (uart_fullduplex)
     --------------------                    ----------------
     GPIO[61]  UART3 TX                      GPIO[62]  UART0 RX
     GPIO[62]  UART0 RX                      GPIO[61]  UART3 TX
     GPIO[5]   Device Ready INT (controller) GPIO[5]   Device Ready INT (device)
     GND                                     GND

 Apollo510b_evb mapping

     CONTROLLER (uart_fullduplex_controller) DEVICE (uart_fullduplex)
     --------------------                    ----------------
     GPIO[00]  UART0 TX                      GPIO[02]  UART1 RX
     GPIO[02]  UART1 RX                      GPIO[00]  UART0 TX
     GPIO[5]   Device Ready INT (controller) GPIO[5]   Device Ready INT (device)
     GND                                     GND


******************************************************************************


