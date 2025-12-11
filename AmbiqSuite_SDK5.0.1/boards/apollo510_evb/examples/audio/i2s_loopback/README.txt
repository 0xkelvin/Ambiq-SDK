Name:
=====
 i2s_loopback


Description:
============
 An example to show basic I2S operation.




Purpose:
========
This example demonstrates I2S loopback functionality for audio
 signal integrity testing and validation. The application showcases I2S
 interface configuration with dual-channel support, audio processing,
 and buffer management.

Section: Key Features
=====================

 1. I2S Loopback: Enables I2S interfaces to loop back data between
    TX and RX for signal integrity testing and validation

 2. Dual Interface Support: Supports I2S0 and I2S1 with configurable
    controller/device roles for flexible audio routing

 3. Data Verification: Compares transmitted and received data to ensure
    accurate loopback operation and signal integrity

 4. Audio Sampling: Supports 48 kHz sample rate, 32-bit word width,
    and 24-bit bit-depth

 5. Ping Pong Buffering: Implements software-managed ping-pong buffer
    mechanism for continuous data streaming without interruption

Section: Functionality
======================

 The application performs the following operations:
 - Configures I2S interfaces with audio parameters
 - Implements loopback between I2S TX and RX interfaces
 - Provides data verification and integrity checking
 - Implements ping-pong buffering for continuous streaming
 - Supports dual I2S interface configuration
 - Monitors and reports I2S operation status via SWO

Section: Usage
==============

 1. Connect I2S pins according to the specified pin configuration
 2. Compile and download the application to target device
 3. Monitor SWO output for I2S operation status and results
 4. Verify loopback data integrity and signal quality
 5. Test continuous operation and buffer management

Section: Configuration
======================

 - Sample Rate: 48 kHz audio sampling rate
 - Audio Format: 32-bit word width, 24-bit bit-depth, 2 channels
 - I2S Format: Standard I2S format
 - Buffer Management: Ping-pong buffering for continuous streaming


 I2S configurations:
  - 2 channels
  - 48 kHz sample rate
  - Standard I2S format
  - 32 bits word width
  - 24 bits bit-depth

 NOTE: Usage of software-implemented ping pong machine
 step 1: Prepare 2 blocks of buffer.
  - sTransfer0.ui32RxTargetAddr = addr1;
  - sTransfer0.ui32RxTargetAddrReverse = addr2;
  - am_hal_i2s_dma_configure(pI2S0Handle, &g_sI2S0Config, &sTransfer0);
 step 2: Call am_hal_i2s_interrupt_service() in the ISR to restart DMA operation,
 the ISR helps automatically switch to the reverse buffer.
 step 3: Fetch the valid data by calling am_hal_i2s_dma_get_buffer().

 The required pin connections are as follows:

 - No peripheral card is connected.
 - mikrobus level shifter is set to 1.8v

 Apollo510 EVB

     I2S1                                I2S0
     --------------------                ----------------
     GPIO[16] CLK (mikrobus RST)         GPIO[ 5]  CLK
     GPIO[19] DIN                        GPIO[ 4]  DIN
     GPIO[17] DOUT (mikrobus AN)         GPIO[ 6]  DOUT (mikrobus MOSI)
     GPIO[18] WS                         GPIO[ 7]  WS (mikrobus MISO)
     GND                                 GND

 Apollo510B EVB

     I2S1                                I2S0
     --------------------                ----------------
     GPIO[16] CLK (mikrobus RST)         GPIO[ 5]  CLK
     GPIO[ 3] DIN                        GPIO[ 4]  DIN
     GPIO[17] DOUT (mikrobus AN)         GPIO[ 6]  DOUT (mikrobus MOSI)
     GPIO[10] WS (mikrobus CS)           GPIO[ 7]  WS (mikrobus MISO)
     GND                                 GND

 Printing takes place over the SWO at 1MHz.



******************************************************************************


