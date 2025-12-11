//*****************************************************************************
//
//! @file tdm_loopback.c
//!
//! @brief An example to show basic TDM operation.
//!
//! @addtogroup audio_examples Audio Examples
//!
//! @defgroup tdm_loopback TDM Loopback Example
//! @ingroup audio_examples
//! @{
//!
//! Purpose: This example demonstrates TDM (Time Division Multiplexing)
//! loopback functionality for audio signal integrity testing and validation. The
//! application showcases I2S interface configuration in TDM mode, dual
//! interface support, DMA-based data transfer, and buffer management.
//!
//! @section tdm_loopback_features Key Features
//!
//! 1. @b TDM @b Mode @b Support: Demonstrates Time Division Multiplexing with
//!    two phases of data format in a single frame
//!
//! 2. @b Dual @b I2S @b Interface: Supports I2S0 and I2S1 with configurable
//!    master/slave roles for flexible audio routing
//!
//! 3. @b Data @b Verification: Compares transmitted and received data to ensure
//!    accurate loopback operation and signal integrity
//!
//! 4. @b Ping @b Pong @b Buffering: Implements software-managed ping-pong buffer
//!    mechanism for continuous data streaming
//!
//! 5. @b Hardware @b Acceleration: Utilizes DMA for efficient data transfer
//!    between I2S interfaces with minimal CPU overhead
//!
//! @section tdm_loopback_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures I2S interfaces in TDM mode with dual-phase support
//! - Implements loopback between I2S TX and RX interfaces
//! - Provides data verification and integrity checking
//! - Implements ping-pong buffering for continuous streaming
//! - Utilizes DMA for efficient data transfer
//! - Monitors and reports TDM operation status via SWO/ITM
//!
//! @section tdm_loopback_usage Usage
//!
//! 1. Connect I2S pins according to the specified pin configuration
//! 2. Compile and download the application to target device
//! 3. Monitor SWO/ITM output for TDM operation status and results
//! 4. Verify loopback data integrity and signal quality
//! 5. Test continuous operation and buffer management
//!
//! @section tdm_loopback_configuration Configuration
//!
//! - @b TDM @b Mode: Dual-phase data format in a single frame
//! - @b DMA: Hardware-accelerated data transfer
//! - @b Buffer @b Management: Ping-pong buffering for continuous streaming
//! - @b SWO/ITM: Output for status and results (1MHz)
//!
//! @verbatim
//! In TDM mode, I2S supports two kinds of data format in one frame which were
//! called phase 1 and phase 2. The number of channels can be different in both
//! phases but the total number shouldn't be greater than 8.
//!
//! NOTE: Usage of software-implemented ping pong machine
//! step 1: Prepare 2 blocks of buffer.
//!  - sTransfer0.ui32RxTargetAddr = addr1;
//!  - sTransfer0.ui32RxTargetAddrReverse = addr2;
//!  - am_hal_i2s_dma_configure(pI2S0Handle, &g_sI2S0Config, &sTransfer0);
//! step 2: Call am_hal_i2s_interrupt_service() in the ISR to restart DMA operation,
//! the ISR helps automatically switch to the reverse buffer.
//! step 3: Fetch the valid data by calling am_hal_i2s_dma_get_buffer().
//!
//! The required pin connections are as follows:
//!
//! - No peripheral card is connected.
//! - mikrobus level shifter is set to 1.8v
//!
//! Apollo510 EVB
//!
//!     I2S1                                I2S0
//!     --------------------                ----------------
//!     GPIO[16] CLK (mikrobus RST)         GPIO[ 5]  CLK
//!     GPIO[19] DIN                        GPIO[ 4]  DIN
//!     GPIO[17] DOUT (mikrobus AN)         GPIO[ 6]  DOUT (mikrobus MOSI)
//!     GPIO[18] WS                         GPIO[ 7]  WS (mikrobus MISO)
//!     GND                                 GND
//!
//! Apollo510B EVB
//!
//!     I2S1                                I2S0
//!     --------------------                ----------------
//!     GPIO[16] CLK (mikrobus RST)         GPIO[ 5]  CLK
//!     GPIO[ 3] DIN                        GPIO[ 4]  DIN
//!     GPIO[17] DOUT (mikrobus AN)         GPIO[ 6]  DOUT (mikrobus MOSI)
//!     GPIO[10] WS (mikrobus CS)           GPIO[ 7]  WS (mikrobus MISO)
//!     GND                                 GND
//!
//!
//! Printing takes place over the ITM at 1MHz.
//! @endverbatim
//!
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2025, Ambiq Micro, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// Third party software included in this distribution is subject to the
// additional license terms as defined in the /docs/licenses directory.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision release_sdk5p0p1-61912905f0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include <stdlib.h>

//*****************************************************************************
//
// Example parameters.
//
//*****************************************************************************
#define CLOCK_SOURCE                (PLL)
    #define HFRC                    (0)
    #define PLL                     (1)
    #define NCO                     (2)
    #define HF2ADJ                  (3)
#define I2S_MODULE_0                (0)
#define I2S_MODULE_1                (1)
#define USE_I2S_CONTROLLER          I2S_MODULE_0 // macro name updated per user request
#define SIZE_SAMPLES                (4096 - 32)     // padded to 32 samples, align with DMA Threshold.
#define DMACPL_TIMEOUT_MS           (2000)

#if defined(AM_PART_APOLLO510)
#define NUM_OF_I2S_INSTANCES   (2)
#elif defined(AM_PART_APOLLO330P_510L)
#define NUM_OF_I2S_INSTANCES   (1)
#endif

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
//
// Record the number of test loops.
//
static uint32_t g_ui32TestLoop = 0;

//
// I2S handlers.
//
void *pI2S0Handle;
#if NUM_OF_I2S_INSTANCES > 1
void *pI2S1Handle;
#endif

//
// I2S interrupts number.
//
static const IRQn_Type i2s_interrupts[] =
{
    I2S0_IRQn,
    #if NUM_OF_I2S_INSTANCES > 1
    I2S1_IRQn,
    #endif
};

//
// Flags of DMA complete.
//
volatile uint32_t g_ui32I2SDmaCpl[] =
{
    0, //I2S0 TX.
    0, //I2S0 RX.
    #if NUM_OF_I2S_INSTANCES > 1
    0, //I2S1 Tx.
    0, //I2S1 Rx.
    #endif
};

//
// Used as the ping-pong buffer of DMA.
// Aligned to 32 bytes to meet data cache requirements.
//
AM_SHARED_RW uint32_t g_ui32I2s0TxDmaBufPing[SIZE_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2s0TxDmaBufPong[SIZE_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2s0RxDmaBufPing[SIZE_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2s0RxDmaBufPong[SIZE_SAMPLES] __attribute__((aligned(32)));
#if NUM_OF_I2S_INSTANCES > 1
AM_SHARED_RW uint32_t g_ui32I2s1TxDmaBufPing[SIZE_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2s1TxDmaBufPong[SIZE_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2s1RxDmaBufPing[SIZE_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2s1RxDmaBufPong[SIZE_SAMPLES] __attribute__((aligned(32)));
#endif

//*****************************************************************************
//
// I2S configurations:
//  - 16 kHz sample rate in common
//  - Total 8 channles
//  - Variable word width
//  - Variable bit-depth
//
//*****************************************************************************
static am_hal_i2s_io_signal_t g_sI2SIOConfig =
{
    .sFsyncPulseCfg = {
        .eFsyncPulseType = AM_HAL_I2S_FSYNC_PULSE_ONE_BIT_CLOCK,
    },
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_HIGH,
    .eTxCpol = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol = AM_HAL_I2S_IO_RX_CPOL_RISING
};

static am_hal_i2s_data_format_t g_sI2SDataConfig[] =
{
    {
        .ePhase = AM_HAL_I2S_DATA_PHASE_SINGLE,
        .eDataDelay = 0x1,
        .ui32ChannelNumbersPhase1 = 8,
        .ui32ChannelNumbersPhase2 = 0,
        .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
        .eChannelLenPhase1 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eChannelLenPhase2 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
        .eSampleLenPhase2 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS
    },
    {
        .ePhase = AM_HAL_I2S_DATA_PHASE_DUAL,
        .eDataDelay = 0x1,
        .ui32ChannelNumbersPhase1 = 4,
        .ui32ChannelNumbersPhase2 = 4,
        .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
        .eChannelLenPhase1 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eChannelLenPhase2 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
        .eSampleLenPhase2 = AM_HAL_I2S_SAMPLE_LENGTH_16BITS
    },
    {
        .ePhase = AM_HAL_I2S_DATA_PHASE_DUAL,
        .eDataDelay = 0x1,
        .ui32ChannelNumbersPhase1 = 4,
        .ui32ChannelNumbersPhase2 = 4,
        .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
        .eChannelLenPhase1 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eChannelLenPhase2 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_32BITS,
        .eSampleLenPhase2 = AM_HAL_I2S_SAMPLE_LENGTH_24BITS
    },
    {
        .ePhase = AM_HAL_I2S_DATA_PHASE_DUAL,
        .eDataDelay = 0x1,
        .ui32ChannelNumbersPhase1 = 4,
        .ui32ChannelNumbersPhase2 = 4,
        .eDataJust = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
        .eChannelLenPhase1 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eChannelLenPhase2 = AM_HAL_I2S_FRAME_WDLEN_32BITS,
        .eSampleLenPhase1 = AM_HAL_I2S_SAMPLE_LENGTH_32BITS,
        .eSampleLenPhase2 = AM_HAL_I2S_SAMPLE_LENGTH_16BITS
    },
};

static am_hal_i2s_config_t g_sI2S0Config =
{
// Controller/Device role selection
#if (USE_I2S_CONTROLLER == I2S_MODULE_0)
    .eMode  = AM_HAL_I2S_IO_MODE_MASTER,
#else
    .eMode  = AM_HAL_I2S_IO_MODE_SLAVE,
#endif
    .eXfer  = AM_HAL_I2S_XFER_RXTX,
#if defined (AM_PART_APOLLO510)
#if (CLOCK_SOURCE == HFRC)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC_12MHz,
    .eDiv3  = 1,
#elif (CLOCK_SOURCE == PLL)
    .eClock = eAM_HAL_I2S_CLKSEL_PLL_FOUT3,
    .eDiv3  = 0,
#elif (CLOCK_SOURCE == NCO)
    .eClock    = eAM_HAL_I2S_CLKSEL_NCO_HFRC_48MHz,
    .f64NcoDiv = 48.0 / 4.096,
#elif (CLOCK_SOURCE == HF2ADJ)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC2_APPROX_16MHz,
    .eDiv3  = 1,
#endif
#elif defined (AM_PART_APOLLO330P_510L)
#if (CLOCK_SOURCE == HFRC)
    .eClock = AM_HAL_I2S_CLKSEL_HFRC_48MHz,
    .ui32ClockDivideRatio = 12,
#elif (CLOCK_SOURCE == PLL)
    .eClock = AM_HAL_I2S_CLKSEL_PLL_FOUT3,
    .ui32ClockDivideRatio = 1,
#elif (CLOCK_SOURCE == NCO)
    .eClock = AM_HAL_I2S_CLKSEL_NCO_HFRC_96MHz,
    .ui32ClockDivideRatio = 1,
    .f64NcoDiv = 96.0 / 4.096,
#endif
    .eMclkout = AM_HAL_I2S_MCLKOUT_SEL_OFF,
    .ui32MclkoutDiv = 1,
#endif
    .eASRC  = 0,
    .eData  = &g_sI2SDataConfig[0],
    .eIO    = &g_sI2SIOConfig
};

// Device/Controller role selection
#if NUM_OF_I2S_INSTANCES > 1
static am_hal_i2s_config_t g_sI2S1Config =
{
    #if (USE_I2S_CONTROLLER == I2S_MODULE_0)
    .eMode  = AM_HAL_I2S_IO_MODE_SLAVE,
    #else
    .eMode  = AM_HAL_I2S_IO_MODE_MASTER,
    #endif
    .eXfer  = AM_HAL_I2S_XFER_RXTX,
    #if (CLOCK_SOURCE == HFRC)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC_12MHz,
    .eDiv3  = 1,
    #elif (CLOCK_SOURCE == PLL)
    .eClock = eAM_HAL_I2S_CLKSEL_PLL_FOUT4,
    .eDiv3  = 1,
    #elif (CLOCK_SOURCE == NCO)
    .eClock    = eAM_HAL_I2S_CLKSEL_NCO_HFRC_48MHz,
    .f64NcoDiv = 48.0 / 4.096,
    #elif (CLOCK_SOURCE == HF2ADJ)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC2_APPROX_16MHz,
    .eDiv3  = 1,
    #endif
    .eASRC  = 0,
    .eData  = &g_sI2SDataConfig[0],
    .eIO    = &g_sI2SIOConfig,
};
#endif

//
// Ping pong buffer settings.
//
static am_hal_i2s_transfer_t sTransfer0 =
{
    .ui32RxTotalCount         = SIZE_SAMPLES,
    .ui32RxTargetAddr         = (uint32_t)(&g_ui32I2s0RxDmaBufPing[0]),
    .ui32RxTargetAddrReverse  = (uint32_t)(&g_ui32I2s0RxDmaBufPong[0]),
    .ui32TxTotalCount         = SIZE_SAMPLES,
    .ui32TxTargetAddr         = (uint32_t)(&g_ui32I2s0TxDmaBufPing[0]),
    .ui32TxTargetAddrReverse  = (uint32_t)(&g_ui32I2s0TxDmaBufPong[0]),
};

#if NUM_OF_I2S_INSTANCES > 1
static am_hal_i2s_transfer_t sTransfer1 =
{
    .ui32RxTotalCount         = SIZE_SAMPLES,
    .ui32RxTargetAddr         = (uint32_t)(&g_ui32I2s1RxDmaBufPing[0]),
    .ui32RxTargetAddrReverse  = (uint32_t)(&g_ui32I2s1RxDmaBufPong[0]),
    .ui32TxTotalCount         = SIZE_SAMPLES,
    .ui32TxTargetAddr         = (uint32_t)(&g_ui32I2s1TxDmaBufPing[0]),
    .ui32TxTargetAddrReverse  = (uint32_t)(&g_ui32I2s1TxDmaBufPong[0]),
};
#endif

//*****************************************************************************
//
// I2S data checking function.
//
// Compare data in the RX buffer and data in the TX buffer, return true if data
// match, otherwise return false.
//
//*****************************************************************************
static bool
check_i2s_data(uint32_t ui32BufSize, uint32_t* pui32RxBuf, uint32_t* pui32TxBuf, am_hal_i2s_data_format_t* pI2SDataConfig, bool bRxIsMaster)
{
    uint32_t i, ui32Mask, ui32Mask1, ui32Mask2, ui32WordBytes;

    uint32_t ui32ChNumPh1 = pI2SDataConfig->ui32ChannelNumbersPhase1;
    uint32_t ui32ChNumPh2 = pI2SDataConfig->ui32ChannelNumbersPhase2;
    uint32_t ui32SampleLenPh1 = ui32I2sWordLength[pI2SDataConfig->eSampleLenPhase1];
    uint32_t ui32SampleLenPh2 = ui32I2sWordLength[pI2SDataConfig->eSampleLenPhase2];
    uint32_t ui32FrameChNum = ui32ChNumPh1 + ui32ChNumPh2;

    if (ui32SampleLenPh1 < ui32SampleLenPh2)
    {
        ui32SampleLenPh2 = ui32SampleLenPh1;
    }

    if (ui32SampleLenPh1 == 16)
    {
        ui32BufSize -= 2;
    }

    if (ui32SampleLenPh1 == 8)
    {
        ui32BufSize -= 3;
    }

    if (ui32SampleLenPh1 == 32)
    {
        ui32WordBytes = 4;
        ui32Mask1 = 0xFFFFFFFF;
        switch (ui32SampleLenPh2)
        {
            case 32: ui32Mask2 = 0xFFFFFFFF; break;
            case 24: ui32Mask2 = 0xFFFFFF00; break;
            case 16: ui32Mask2 = 0xFFFF0000; break;
            case 8:  ui32Mask2 = 0xFF000000; break;
            default: am_util_stdio_printf("ERROR! Invalidate sample length!"); return false;
        }
    }
    else if (ui32SampleLenPh1 == 24)
    {
        ui32WordBytes = 4;
        ui32Mask1 = 0x00FFFFFF;
        switch (ui32SampleLenPh2)
        {
            case 24: ui32Mask2 = 0x00FFFFFF; break;
            case 16: ui32Mask2 = 0x00FFFF00; break;
            case 8:  ui32Mask2 = 0x00FF0000; break;
            default: am_util_stdio_printf("ERROR! Invalidate sample length!"); return false;
        }
    }
    else if (ui32SampleLenPh1 == 16)
    {
        ui32WordBytes = 2;
        ui32Mask1 = 0xFFFF;
        switch (ui32SampleLenPh2)
        {
            case 16: ui32Mask2 = 0xFFFF; break;
            case 8:  ui32Mask2 = 0xFF00; break;
            default: am_util_stdio_printf("ERROR! Invalidate sample length!"); return false;
        }
    }
    else if (ui32SampleLenPh1 == 8)
    {
        ui32WordBytes = 1;
        ui32Mask1 = 0xFF;
        switch (ui32SampleLenPh2)
        {
            case 8:  ui32Mask2 = 0xFF; break;
            default: am_util_stdio_printf("ERROR! Invalidate sample length!"); return false;
        }
    }
    else
    {
        am_util_stdio_printf("ERROR! Invalidate sample length!");
        return false;
    }

    //
    // Find the beginning sample.
    //
    uint32_t ui32StartPoint = 0xFFFFFFFF;
    uint32_t ui32MatchCount = 0;
    uint32_t ui32RxIndex, ui32TxIndex;
    bool bMatchTrue = false;

    //
    // The maximum allowed lag between controller TX and device TX is 500 us.
    // At 12.288 MHz SCLK, there will be 12.288 MHz * 0.0005 s / 8 bits = 768 bytes of data which
    // won't be received by the device RX, or 768 bytes of zero will be received by the controller RX.
    //
    uint32_t ui32StartTolerance = (ui32BufSize > 768) ? 768 : ui32BufSize;

    for (i = 0; i < ui32StartTolerance / ui32WordBytes; i++)
    {
        ui32TxIndex = (bRxIsMaster) ? ui32MatchCount : i;
        ui32RxIndex = (bRxIsMaster) ? i : ui32MatchCount;

        ui32Mask = ((ui32RxIndex % ui32FrameChNum) < ui32ChNumPh1) ? ui32Mask1 : ui32Mask2;

        switch (ui32WordBytes)
        {
            case 1: bMatchTrue = ( ((uint8_t*)pui32RxBuf)[ui32RxIndex] ==   ((uint8_t*)pui32TxBuf)[ui32TxIndex]); break;
            case 2: bMatchTrue = (((uint16_t*)pui32RxBuf)[ui32RxIndex] == (((uint16_t*)pui32TxBuf)[ui32TxIndex] & (uint16_t)ui32Mask)); break;
            case 4: bMatchTrue = (((uint32_t*)pui32RxBuf)[ui32RxIndex] == (((uint32_t*)pui32TxBuf)[ui32TxIndex] & ui32Mask)); break;
        }

        //
        // Until we match 8 bytes of data, we find the start point.
        //
        if (bMatchTrue)
        {
            ui32MatchCount++;
            if ((ui32MatchCount * ui32WordBytes) >= 8)
            {
                ui32StartPoint = i - ui32MatchCount + 1;
                break;
            }
        }
        else
        {
            ui32MatchCount = 0;
        }
    }

    if (ui32StartPoint == 0xFFFFFFFF)
    {
        am_util_stdio_printf("Did not find the beginning word in the buffer!\n");
        return false;
    }

    //
    // Compare the remaining data.
    //
    for (i = ui32StartPoint; i < ui32BufSize / ui32WordBytes; i++)
    {
        ui32TxIndex = (bRxIsMaster) ? (i - ui32StartPoint) : i;
        ui32RxIndex = (bRxIsMaster) ? i : (i - ui32StartPoint);

        ui32Mask = ((ui32RxIndex % ui32FrameChNum) < ui32ChNumPh1) ? ui32Mask1 : ui32Mask2;

        //
        // These two values should be equal, otherwise the transmitter mask is different from the receiver mask.
        //
        if ((ui32RxIndex % ui32FrameChNum) != (ui32TxIndex % ui32FrameChNum))
        {
            am_util_stdio_printf("ERROR: Channel is out of order.\n");
            return false;
        }

        switch (ui32WordBytes)
        {
            case 1: bMatchTrue = ( ((uint8_t*)pui32RxBuf)[ui32RxIndex] ==   ((uint8_t*)pui32TxBuf)[ui32TxIndex]); break;
            case 2: bMatchTrue = (((uint16_t*)pui32RxBuf)[ui32RxIndex] == (((uint16_t*)pui32TxBuf)[ui32TxIndex] & (uint16_t)ui32Mask)); break;
            case 4: bMatchTrue = (((uint32_t*)pui32RxBuf)[ui32RxIndex] == (((uint32_t*)pui32TxBuf)[ui32TxIndex] & ui32Mask)); break;
        }

        if (bMatchTrue == false)
        {
            am_util_stdio_printf("Data checking faild: Started from: %d, failed at %d, Word bytes %d\n", ui32StartPoint, i, ui32WordBytes);
            return false;
        }
    }

    return true;
}

//*****************************************************************************
//
// I2S0 interrupt handler.
//
//*****************************************************************************
void
am_dspi2s0_isr(void)
{
    uint32_t ui32Status;

    am_hal_i2s_interrupt_status_get(pI2S0Handle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(pI2S0Handle, ui32Status);

    //
    // Switch ping pong buffer.
    //
    am_hal_i2s_interrupt_service(pI2S0Handle, ui32Status, &g_sI2S0Config);

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        g_ui32I2SDmaCpl[0] = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        g_ui32I2SDmaCpl[1] = 1;
    }
}

#if NUM_OF_I2S_INSTANCES > 1
//*****************************************************************************
//
// I2S1 interrupt handler.
//
//*****************************************************************************
void
am_dspi2s1_isr(void)
{
    uint32_t ui32Status;

    am_hal_i2s_interrupt_status_get(pI2S1Handle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(pI2S1Handle, ui32Status);

    //
    // Switch ping pong buffer.
    //
    am_hal_i2s_interrupt_service(pI2S1Handle, ui32Status, &g_sI2S1Config);

    if (ui32Status & AM_HAL_I2S_INT_TXDMACPL)
    {
        g_ui32I2SDmaCpl[2] = 1;
    }

    if (ui32Status & AM_HAL_I2S_INT_RXDMACPL)
    {
        g_ui32I2SDmaCpl[3] = 1;
    }
}
#endif

//*****************************************************************************
//
// I2S initialization.
//
//*****************************************************************************
void
i2s_init(void)
{
    #if (CLOCK_SOURCE == PLL)
    {
        #if defined(AM_PART_APOLLO510)
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_SYSPLL, 24576000, NULL);
        #elif defined(AM_PART_APOLLO330P_510L)
        //
        // Set XTAL_HS as highest priority for SYSPLL FREF.
        //
        am_hal_clkmgr_syspll_fref_priority_t sSysPllFrefPrio =
        {
            .high = AM_HAL_SYSPLL_FREFSEL_XTAL48MHz,
            .mid = AM_HAL_SYSPLL_FREFSEL_EXTREFCLK,
            .low = AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4,
        };
        am_hal_clkmgr_control(AM_HAL_CLKMGR_SYPLL_FREF_PRIORITY_SET, (void*)&sSysPllFrefPrio);

        //
        // Config PLLPOSTDIV to output 24.576 MHz, hence PLLOUT3 = 4.096 MHz.
        //
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLVCO, 245760000, NULL);
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, 24576000, NULL);
        #endif
    }
    #elif (CLOCK_SOURCE == HF2ADJ)
    {
        #if defined(AM_PART_APOLLO510)
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC2, AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ, NULL);
        #endif
    }
    #endif

    //
    // Configure the necessary pins.
    //
    am_bsp_i2s_pins_enable(I2S_MODULE_0, false);
    #if NUM_OF_I2S_INSTANCES > 1
    am_bsp_i2s_pins_enable(I2S_MODULE_1, false);
    #endif

    //
    // Configure I2S0 and I2S1.
    //
    am_hal_i2s_initialize(I2S_MODULE_0, &pI2S0Handle);
    am_hal_i2s_power_control(pI2S0Handle, AM_HAL_I2S_POWER_ON, false);
    #if NUM_OF_I2S_INSTANCES > 1
    am_hal_i2s_initialize(I2S_MODULE_1, &pI2S1Handle);
    am_hal_i2s_power_control(pI2S1Handle, AM_HAL_I2S_POWER_ON, false);
    #endif

    if (AM_HAL_STATUS_SUCCESS != am_hal_i2s_configure(pI2S0Handle, &g_sI2S0Config))
    {
        am_util_stdio_printf("ERROR: Invalid I2S0 configuration.\nNote: For Apollo5 Rev.B0, I2S can only use PLL as the clock source.\n");
    }

    #if NUM_OF_I2S_INSTANCES > 1
    if (AM_HAL_STATUS_SUCCESS != am_hal_i2s_configure(pI2S1Handle, &g_sI2S1Config))
    {
        am_util_stdio_printf("ERROR: Invalid I2S1 configuration.\nNote: For Apollo5 Rev.B0, I2S can only use PLL as the clock source.\n");
    }
    #endif

    am_hal_i2s_enable(pI2S0Handle);
    #if NUM_OF_I2S_INSTANCES > 1
    am_hal_i2s_enable(pI2S1Handle);
    #endif
}

//*****************************************************************************
//
// I2S deinitialization.
//
//*****************************************************************************
void
i2s_deinit(void)
{
    //
    // I2S0
    //
    am_hal_i2s_disable(pI2S0Handle);
    am_hal_i2s_power_control(pI2S0Handle, AM_HAL_I2S_POWER_OFF, false);
    am_hal_i2s_deinitialize(pI2S0Handle);
    am_bsp_i2s_pins_disable(I2S_MODULE_0, false);

    #if NUM_OF_I2S_INSTANCES > 1
    //
    // I2S1
    //
    am_hal_i2s_disable(pI2S1Handle);
    am_hal_i2s_power_control(pI2S1Handle, AM_HAL_I2S_POWER_OFF, false);
    am_hal_i2s_deinitialize(pI2S1Handle);
    am_bsp_i2s_pins_disable(I2S_MODULE_1, false);
    #endif
}


//*****************************************************************************
//
// Print I2S0 channel and data information.
//
//*****************************************************************************
void
print_i2s0_configuration(void)
{
    if (I2Sn(0)->I2SDATACFG_b.PH == 0)
    {
        am_util_stdio_printf("-- Single Phase\n");
        am_util_stdio_printf("-- Channel:    %2d\n", I2Sn(0)->I2SDATACFG_b.FRLEN1 + 1);
        am_util_stdio_printf("-- Word Width: %2d\n", ui32I2sWordLength[I2Sn(0)->I2SDATACFG_b.WDLEN1]);
        am_util_stdio_printf("-- Bit Depth:  %2d\n", ui32I2sWordLength[I2Sn(0)->I2SDATACFG_b.SSZ1]);
    }
    else
    {
        am_util_stdio_printf("-- Dual Phase\n");
        am_util_stdio_printf("-- Channel:    %2d, %2d\n", I2Sn(0)->I2SDATACFG_b.FRLEN1 + 1, I2Sn(0)->I2SDATACFG_b.FRLEN2 + 1);
        am_util_stdio_printf("-- Word Width: %2d, %2d\n", ui32I2sWordLength[I2Sn(0)->I2SDATACFG_b.WDLEN1], ui32I2sWordLength[I2Sn(0)->I2SDATACFG_b.WDLEN2]);
        am_util_stdio_printf("-- Bit Depth:  %2d, %2d\n", ui32I2sWordLength[I2Sn(0)->I2SDATACFG_b.SSZ1], ui32I2sWordLength[I2Sn(0)->I2SDATACFG_b.SSZ2]);
    }
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Initialize the printf interface for ITM output.
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();

    #if defined(AM_PART_APOLLO510)
    #if (USE_I2S_CONTROLLER == I2S_MODULE_0)
    am_util_stdio_printf("TDM Full Duplex Loopback Test: Controller = I2S0, Device = I2S1.\n\n");
    #else
    am_util_stdio_printf("TDM Full Duplex Loopback Test: Controller = I2S1, Device = I2S0.\n\n");
    #endif
    #elif defined(AM_PART_APOLLO330P_510L)
    am_util_stdio_printf("TDM Self-loopback Test: Controller = I2S0.\n\n");
    #endif

    //
    // I2S DMACPL interrupts timeout value.
    //
    uint32_t ui32Timeout0P1Ms = DMACPL_TIMEOUT_MS * 10;

    for (uint32_t i1 = 0; i1 < sizeof(g_sI2SDataConfig) / sizeof(am_hal_i2s_data_format_t); i1++)
    {
        am_util_stdio_printf("\nTDM Loopback Test with Configuration %d...\n", i1);

        g_ui32TestLoop = 0;
        g_ui32I2SDmaCpl[0] = g_ui32I2SDmaCpl[1] = 0;
        g_sI2S0Config.eData = &g_sI2SDataConfig[i1];
        #if NUM_OF_I2S_INSTANCES > 1
        g_ui32I2SDmaCpl[2] = g_ui32I2SDmaCpl[3] = 0;
        g_sI2S1Config.eData = &g_sI2SDataConfig[i1];
        #endif

        i2s_init();
        print_i2s0_configuration();
        am_hal_i2s_dma_configure(pI2S0Handle, &g_sI2S0Config, &sTransfer0);
        NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE_0]);
        #if NUM_OF_I2S_INSTANCES > 1
        am_hal_i2s_dma_configure(pI2S1Handle, &g_sI2S1Config, &sTransfer1);
        NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE_1]);
        #endif
        am_hal_interrupt_master_enable();

        //
        // Prepare the 1st and the 2nd block of data.
        //
        for (uint32_t i2 = 0; i2 < SIZE_SAMPLES; i2++)
        {
            g_ui32I2s0TxDmaBufPing[i2] = rand();
            g_ui32I2s0TxDmaBufPong[i2] = rand();
            #if NUM_OF_I2S_INSTANCES > 1
            g_ui32I2s1TxDmaBufPing[i2] = rand();
            g_ui32I2s1TxDmaBufPong[i2] = rand();
            #endif
        }

        //
        // Write the contents in the data cache to the external memory.
        //
        am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32I2s0TxDmaBufPing, sizeof(g_ui32I2s0TxDmaBufPing)});
        am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32I2s0TxDmaBufPong, sizeof(g_ui32I2s0TxDmaBufPong)});
        #if NUM_OF_I2S_INSTANCES > 1
        am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32I2s1TxDmaBufPing, sizeof(g_ui32I2s1TxDmaBufPing)});
        am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32I2s1TxDmaBufPong, sizeof(g_ui32I2s1TxDmaBufPong)});
        #endif

        //
        // Start DMA transaction.
        //
        #if NUM_OF_I2S_INSTANCES == 2
        #if (USE_I2S_MASTER == I2S_MODULE_0)
        am_hal_i2s_dma_transfer_start(pI2S1Handle, &g_sI2S1Config);
        am_hal_i2s_dma_transfer_start(pI2S0Handle, &g_sI2S0Config);
        #else
        am_hal_i2s_dma_transfer_start(pI2S0Handle, &g_sI2S0Config);
        am_hal_i2s_dma_transfer_start(pI2S1Handle, &g_sI2S1Config);
        #endif
        #elif NUM_OF_I2S_INSTANCES == 1
        am_hal_i2s_dma_transfer_start(pI2S0Handle, &g_sI2S0Config);
        #endif

        while (1)
        {
            #if NUM_OF_I2S_INSTANCES == 2
            if (g_ui32I2SDmaCpl[0] && g_ui32I2SDmaCpl[1] && g_ui32I2SDmaCpl[2] && g_ui32I2SDmaCpl[3])
            {
                g_ui32TestLoop++;
                g_ui32I2SDmaCpl[0] = g_ui32I2SDmaCpl[1] = g_ui32I2SDmaCpl[2] = g_ui32I2SDmaCpl[3] = 0;
                ui32Timeout0P1Ms = DMACPL_TIMEOUT_MS * 10;

                //
                // RX DMA buffer that has completed reception.
                //
                uint32_t* i2s0RxBuf = (uint32_t*)am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_RX);
                uint32_t* i2s1RxBuf = (uint32_t*)am_hal_i2s_dma_get_buffer(pI2S1Handle, AM_HAL_I2S_XFER_RX);

                //
                // Ensure that what the CPU sees is consistent with the contents in SSRAM.
                //
                am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){(uint32_t)i2s0RxBuf, sizeof(uint32_t) * SIZE_SAMPLES}, false);
                am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){(uint32_t)i2s1RxBuf, sizeof(uint32_t) * SIZE_SAMPLES}, false);

                //
                // TX DMA buffer that has completed sending.
                //
                uint32_t* i2s0TxBuf = ((g_ui32TestLoop % 2) == 1) ? g_ui32I2s0TxDmaBufPing : g_ui32I2s0TxDmaBufPong;
                uint32_t* i2s1TxBuf = ((g_ui32TestLoop % 2) == 1) ? g_ui32I2s1TxDmaBufPing : g_ui32I2s1TxDmaBufPong;

                //
                // Compare the TX DMA buffer and the RX DMA buffer to check if the data matches.
                //
                bool bI2S0to1Transfer = check_i2s_data(SIZE_SAMPLES * 4, i2s1RxBuf, i2s0TxBuf, &g_sI2SDataConfig[i1], (g_sI2S1Config.eMode == AM_HAL_I2S_IO_MODE_MASTER));
                bool bI2S1to0Transfer = check_i2s_data(SIZE_SAMPLES * 4, i2s0RxBuf, i2s1TxBuf, &g_sI2SDataConfig[i1], (g_sI2S0Config.eMode == AM_HAL_I2S_IO_MODE_MASTER));

                if (bI2S0to1Transfer && bI2S1to0Transfer)
                {
                    am_util_stdio_printf("TDM Loopback Iteration %2d PASSED!\n", g_ui32TestLoop);
                    if (g_ui32TestLoop >= 10)
                    {
                        am_hal_interrupt_master_disable();
                        NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE_0]);
                        NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE_1]);
                        am_hal_i2s_dma_transfer_complete(pI2S0Handle);
                        am_hal_i2s_dma_transfer_complete(pI2S1Handle);
                        i2s_deinit();
                        break;
                    }
                }
                else
                {
                    am_util_stdio_printf("---ERROR--- TDM Loopback Iteration %2d FAILED!\n", g_ui32TestLoop);
                    break;
                }

                //
                // The (n)th block of the buffer is completed; the upper code is for comparing them.
                // The (n+1)th block of the buffer is ongoing.
                // The (n+2)th block of the buffer will be prepared in the code below.
                //
                uint32_t* i2s0TxBufNext = (uint32_t*)am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_TX);
                uint32_t* i2s1TxBufNext = (uint32_t*)am_hal_i2s_dma_get_buffer(pI2S1Handle, AM_HAL_I2S_XFER_TX);
                for (uint32_t i3 = 0; i3 < SIZE_SAMPLES; i3++)
                {
                    i2s0TxBufNext[i3] = rand();
                    i2s1TxBufNext[i3] = rand();
                }

                //
                // Write the contents in the data cache to the external memory.
                //
                am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)i2s0TxBufNext, sizeof(uint32_t) * SIZE_SAMPLES});
                am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)i2s1TxBufNext, sizeof(uint32_t) * SIZE_SAMPLES});
            }
            #elif NUM_OF_I2S_INSTANCES == 1
            if (g_ui32I2SDmaCpl[0] && g_ui32I2SDmaCpl[1])
            {
                g_ui32TestLoop++;
                g_ui32I2SDmaCpl[0] = g_ui32I2SDmaCpl[1] = 0;
                ui32Timeout0P1Ms = DMACPL_TIMEOUT_MS * 10;

                //
                // RX DMA buffer that has completed reception.
                //
                uint32_t* i2s0RxBuf = (uint32_t*)am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_RX);

                //
                // Ensure that what the CPU sees is consistent with the contents in SSRAM.
                //
                am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){(uint32_t)i2s0RxBuf, sizeof(uint32_t) * SIZE_SAMPLES}, false);

                //
                // TX DMA buffer that has completed sending.
                //
                uint32_t* i2s0TxBuf = ((g_ui32TestLoop % 2) == 1) ? g_ui32I2s0TxDmaBufPing : g_ui32I2s0TxDmaBufPong;

                //
                // Compare the TX DMA buffer and the RX DMA buffer to check if the data matches.
                //
                bool bI2S0to0Transfer = check_i2s_data(SIZE_SAMPLES * 4, i2s0RxBuf, i2s0TxBuf, &g_sI2SDataConfig[i1], true);

                if (bI2S0to0Transfer)
                {
                    am_util_stdio_printf("TDM Loopback Iteration %2d PASSED!\n", g_ui32TestLoop);
                    if (g_ui32TestLoop >= 10)
                    {
                        am_hal_interrupt_master_disable();
                        NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE_0]);
                        am_hal_i2s_dma_transfer_complete(pI2S0Handle);
                        i2s_deinit();
                        break;
                    }
                }
                else
                {
                    am_util_stdio_printf("---ERROR--- TDM Loopback Iteration %2d FAILED!\n", g_ui32TestLoop);
                    break;
                }

                //
                // The (n)th block of the buffer is completed; the upper code is for comparing them.
                // The (n+1)th block of the buffer is ongoing.
                // The (n+2)th block of the buffer will be prepared in the code below.
                //
                uint32_t* i2s0TxBufNext = (uint32_t*)am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_TX);
                for (uint32_t i3 = 0; i3 < SIZE_SAMPLES; i3++)
                {
                    i2s0TxBufNext[i3] = rand();
                }

                //
                // Write the contents in the data cache to the external memory.
                //
                am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)i2s0TxBufNext, sizeof(uint32_t) * SIZE_SAMPLES});
            }
            #endif

            am_util_delay_us(100);
            ui32Timeout0P1Ms--;
            if (ui32Timeout0P1Ms == 0)
            {
                am_util_stdio_printf("---ERROR--- TDM Loopback Timeout.\n");
                break;
            }
        }

        am_util_delay_ms(100);
    }
    am_util_stdio_printf("Ran to End!\n");
    while (1)
    {
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
