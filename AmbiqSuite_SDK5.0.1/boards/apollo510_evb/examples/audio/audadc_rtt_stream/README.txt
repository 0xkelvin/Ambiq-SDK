Name:
=====
 audadc_rtt_stream


Description:
============
 This example uses AUDADC to capture and send audio data to PC via SEGGER RTT.




Purpose:
========
This example demonstrates AUDADC audio capture and
 real-time streaming functionality using SEGGER RTT. The application showcases
 audio data acquisition from external analog microphones with
 timing control using INTTRIGTIMER. The example implements dual-channel
 audio capture at 16 kHz sample rate, providing PCM data output
 for real-time monitoring, analysis, and debugging of audio applications.

Section: Key Features
=====================

 1. AUDADC Interface: Captures audio samples from external analog microphones
    at 16 kHz sample rate using INTTRIGTIMER

 2. RTT Data Streaming: Uploads captured audio data to PC via SEGGER
    RTT for real-time monitoring and analysis

 3. Dual Channel Support: Captures audio from two analog microphones
    simultaneously for stereo or multi-microphone applications

 4. PCM Format Output: Provides 16-bit PCM data in little-endian format
    compatible with standard audio analysis tools

 5. Real Time Processing: Enables continuous audio streaming with
    minimal latency for live audio applications and debugging

Section: Functionality
======================

 The application performs the following operations:
 - Configures AUDADC with INTTRIGTIMER for audio sampling
 - Captures dual-channel audio data from external microphones
 - Implements real-time data streaming via SEGGER RTT
 - Provides PCM format output for audio analysis tools
 - Supports continuous audio streaming with minimal latency
 - Enables real-time monitoring and debugging capabilities

Section: Usage
==============

 1. Connect external analog microphones to the device
 2. Compile and download the application to target device
 3. Update RTT address and device type in rtt_logger.py
 4. Run 'python3 rtt_logger.py' to capture PCM data via RTT
 5. Use Audacity to import and analyze the PCM audio data
 6. Convert PCM to WAV format using 'python3 pcm_to_wav.py'

Section: Configuration
======================

 - Sample Rate: 16 kHz audio sampling rate
 - Audio Format: 16-bit PCM, little-endian, 2 channels
 - Clock Source: Configurable (XTHS, HFRC, HFRC2ADJ, EXTCLK, PLL)
 - RTT Interface: Real-time data streaming to PC

 Run 'python3 rtt_logger.py' to capture PCM raw data via SEGGER RTT, you can check
 the PCM file in Audacity https://www.audacityteam.org/:
 -    File -> Import -> Raw data...
 -    Default import format is:
 -        Signed 16-bit PCM
 -        Little-endian
 -        2 Channels
 -        Start offset: 0 bytes
 -        Amount to import: 100%
 -        Sample rate: 16000 Hz
 These two channels are data of mic1 and data of mic2.
 You can convert this PCM file to wav file by running 'python3 pcm_to_wav.py', but
 only the first channel will be kept.

 NOTE:
 In this example, RTT control block is mapped to a fixed address to facilitate
 the searching process. If the address is changed, make sure to modify the
 rtt_logger.py script to match the address, which can be get from SWO prints.



******************************************************************************


