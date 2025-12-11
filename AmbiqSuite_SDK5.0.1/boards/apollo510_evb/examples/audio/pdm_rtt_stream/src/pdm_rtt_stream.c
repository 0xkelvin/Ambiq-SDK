//*****************************************************************************
//
//! @file pdm_rtt_stream.c
//!
//! @brief An example to show PDM audio streaming to PC over RTT data logger.
//!
//! @addtogroup audio_examples Audio Examples
//!
//! @defgroup pdm_rtt_stream PDM RTT Stream Example
//! @ingroup audio_examples
//! @{
//!
//! Purpose: This example demonstrates PDM audio capture and
//! real-time streaming functionality using SEGGER RTT. The application showcases
//! digital microphone audio acquisition at 16 kHz sample rate, PCM
//! conversion with 24-bit resolution, and stereo channel support.
//!
//! @section pdm_rtt_stream_features Key Features
//!
//! 1. @b PDM @b Audio @b Capture: Receives audio signals from external digital
//!    microphone at 16 kHz sample rate for audio recording
//!
//! 2. @b RTT @b Data @b Streaming: Uploads captured audio data to PC via SEGGER
//!    RTT for real-time monitoring and analysis
//!
//! 3. @b PCM @b Format @b Support: Converts PDM data to PCM format with 24-bit
//!    resolution and stereo channel support
//!
//! 4. @b File @b Logging: Automatically saves captured audio to timestamped
//!    PCM files for offline analysis and playback
//!
//! 5. @b Real @b Time @b Monitoring: Provides continuous audio streaming with
//!    minimal latency for live audio applications and debugging
//!
//! @section pdm_rtt_stream_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures PDM interface for digital microphone audio capture
//! - Captures audio data at 16 kHz sample rate
//! - Converts PDM data to 24-bit PCM format with stereo support
//! - Streams audio data to PC via SEGGER RTT
//! - Logs audio data to timestamped PCM files for offline analysis
//! - Provides real-time monitoring and status reporting via SWO
//!
//! @section pdm_rtt_stream_usage Usage
//!
//! 1. Connect digital microphone to specified GPIO pins (PDM0_CLK, PDM0_DATA)
//! 2. Compile and download the application to target device
//! 3. Run 'python3 rtt_logger.py' to capture PCM data via RTT
//! 4. Use Audacity to import and analyze the PCM audio data
//! 5. Monitor SWO output for status and configuration information
//!
//! @section pdm_rtt_stream_configuration Configuration
//!
//! - @b Sample @b Rate: 16 kHz audio sampling rate
//! - @b PCM @b Format: 24-bit resolution, stereo channels
//! - @b RTT @b Interface: Real-time data streaming to PC
//! - @b SWO/ITM: Output for status and results (1MHz)
//!
//! General pin connections:
//! GPIO_50 PDM0_CLK  to CLK_IN of digital microphone
//! GPIO_51 PDM0_DATA to DATA_OUT of digital microphone
//!
//! Known Issue: this was tested and worked with Segger Jlink_v764e tools
//! It wasn't working with Segger Jlink_v844a
//!
//! NOTE:
//! In this example, RTT control block is mapped to a fixed address to facilitate
//! the searching process. If the address is changed, make sure to modify the
//! rtt_logger.py script to match the address, which can be get from SWO prints.
//!
//! Printing takes place over the SWO at 1MHz.
//! RTT logger takes place over the SWD at 4M Baud.
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
#include "SEGGER_RTT.h"

//*****************************************************************************
//
// Example option parameters.
//
//*****************************************************************************

typedef enum
{
    eCS_HFRC,
    eCS_PLL,
    eCS_HF2ADJ,
    eCS_EXTREF,
    eCS_EXTREF_PLL,
    eCS_HFXTAL,
    eCS_HFXTAL_DIV2,
    eCS_HFXTAL_DIV4,
    eCS_HFRC_DED,
    eCS_MAX_SOURCES,
}
clock_source_t;

const clock_source_t eClockSource = eCS_PLL;

typedef struct
{
    uint32_t  ui32PdmFreq;
    uint32_t  ui32DecimationRate;
    am_hal_pdm_clkspd_e eClkSpd;
    bool bValid;
}
clock_info_table_t;


const clock_info_table_t clkInfoTable[eCS_MAX_SOURCES] =
{
    [eCS_HFRC]       = {.ui32PdmFreq = 24000000, .eClkSpd = AM_HAL_PDM_CLK_HFRC_24MHZ, .bValid = true },
    [eCS_PLL]        = {.ui32PdmFreq = 24576000, .eClkSpd = AM_HAL_PDM_CLK_PLL, .bValid = true },
    //[eCS_HF2ADJ]    = {.ui32PdmFreq = 31250000, .eClkSpd = AM_HAL_PDM_CLK_HFRC2_31MHZ, .bValid = true },
    [eCS_HF2ADJ]     = {.ui32PdmFreq = AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ, .eClkSpd = AM_HAL_PDM_CLK_HFRC2_31MHZ, .bValid = true },
    [eCS_EXTREF]     = {.ui32PdmFreq = 12000000, .ui32DecimationRate = 64, .eClkSpd = AM_HAL_PDM_CLK_EXTREF, .bValid = true },
    [eCS_EXTREF_PLL] = {.ui32PdmFreq = 24576000, .ui32DecimationRate = 64, .eClkSpd = AM_HAL_PDM_CLK_PLL, .bValid = true },
};


#define PDM_MODULE              (0)
#define FIFO_THRESHOLD_CNT      (16)
#define NUM_OF_SAMPLES          (256)
#define PCM_CHANNEL_NUM         (2)
#define PCM_WORD_WIDTH          (PCM_24_BITS)
    #define PCM_24_BITS         (24)
    #define PCM_16_BITS         (16)
#define SIZE_OF_SAMPLES         (NUM_OF_SAMPLES * (PCM_WORD_WIDTH > 16 ? 4 : 2))

#if defined(AM_PART_APOLLO510) && (PCM_WORD_WIDTH == PCM_16_BITS)
#error "PCM_WORD_WIDTH must be 24 bits on Apollo510."
#endif

//
// Exercise DMA start and DMA stop APIs.
//
#define EXERCISE_DMA_ONOFF_API  (0)

//
// Enable the ping-pong mechanism so that the PDM HAL uses these two buffers to
// reload DMA alternately in the am_hal_pdm_interrupt_service() function.
// If this mechanism is disabled, the user must reload the DMA by calling
// am_hal_pdm_dma_transfer_continue() to start a new transfer when the previous
// DMA transfer is complete.
//
#define ENABLE_PING_PONG_DMA    (1)

#if (EXERCISE_DMA_ONOFF_API == 1)
#define DMA_ONOFF_INTERVAL_MS   (3000)
#define XT_PERIOD               (32768)
#define WAKE_INTERVAL           (XT_PERIOD * DMA_ONOFF_INTERVAL_MS / 1000)
#endif

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

typedef struct
{
    void *PDMHandle;
#define RTT_BUFFER_LENGTH       (256*3*200)
    uint8_t rttRecorderBuffer[RTT_BUFFER_LENGTH];
    //
    // PDM handler.
    //
    volatile bool bPDMDataReady;

}
rtt_stream_vars_t;


static rtt_stream_vars_t g_sRttStrVars;

//
// PDM buffer with or without DMA
// Aligned to 32 bytes to meet data cache requirements.
//
AM_SHARED_RW uint32_t g_ui32PDMDataBuffer[2*NUM_OF_SAMPLES] __attribute__((aligned(32)));

//
// RTT buffer
//


//
// PDM interrupt number.
//
static const IRQn_Type pdm_interrupts[] =
{
    PDM0_IRQn,
    PDM0_IRQn,
    PDM0_IRQn,
    PDM0_IRQn,
};


//*****************************************************************************
//
// PDM configuration information.
//
//*****************************************************************************
am_hal_pdm_config_t g_sPdmConfig =
{
    //
    // Example setting:
    //  2.048 MHz PDM CLK OUT:
    //      PDM_CLK_OUT = ePDMClkSpeed / (eClkDivider + 1) / (ePDMAClkOutDivder + 1)
    //  16 kHz 24bit Sampling:
    //      DecimationRate = 64
    //      SAMPLEING_FREQ = PDM_CLK_OUT / (ui32DecimationRate * 2)
    //

#if defined(AM_PART_APOLLO330P_510L)

 #if (PCM_WORD_WIDTH == 16)
    .eWordWidth         = AM_HAL_PDM_DATA_WORD_WIDTH_16BITS,
 #elif (PCM_WORD_WIDTH == 24)
    .eWordWidth         = AM_HAL_PDM_DATA_WORD_WIDTH_24BITS,
 #else
  #error "PCM_WORD_WIDTH must be 16 or 24"
 #endif

    .eDataFlowDirection = AM_HAL_PDM_DATA_FLOW_TO_MEMORY,
    .bI2sMaster         = false,
#endif

#if (PCM_CHANNEL_NUM == 1)
    .ePCMChannels        = AM_HAL_PDM_CHANNEL_LEFT,
#elif (PCM_CHANNEL_NUM == 2)
    .ePCMChannels        = AM_HAL_PDM_CHANNEL_STEREO,
#else
#error "PCM_CHANNEL_NUM must be 1 or 2"
#endif

    .eClkDivider         = AM_HAL_PDM_MCLKDIV_1,
    .ePDMAClkOutDivder   = AM_HAL_PDM_PDMA_CLKO_DIV5,
    .ui32DecimationRate  = 64,
    .eLeftGain           = AM_HAL_PDM_GAIN_0DB,
    .eRightGain          = AM_HAL_PDM_GAIN_0DB,
    .eStepSize           = AM_HAL_PDM_GAIN_STEP_0_13DB,
    .bHighPassEnable     = AM_HAL_PDM_HIGH_PASS_ENABLE,
    .ui32HighPassCutoff  = 0x3,
    .bDataPacking        = 1,
    .bPDMSampleDelay     = AM_HAL_PDM_CLKOUT_PHSDLY_NONE,
    .ui32GainChangeDelay = AM_HAL_PDM_CLKOUT_DELAY_NONE,
    .bSoftMute           = 0,
    .bLRSwap             = 0,
};

am_hal_pdm_transfer_t sTransfer =
{
    .ui32TargetAddr        = (uint32_t)(&g_ui32PDMDataBuffer[0]),
#if ENABLE_PING_PONG_DMA
    .ui32TargetAddrReverse = (uint32_t)(&g_ui32PDMDataBuffer[NUM_OF_SAMPLES]),
#else
    //
    // Set .ui32TargetAddrReverse to 0xFFFFFFFF to indicate to
    // the PDM HAL not to enable the ping-pong mechanism.
    //
    .ui32TargetAddrReverse = 0xFFFFFFFF,
#endif
    .ui32TotalCount        = SIZE_OF_SAMPLES,
};

static uint32_t pdm_init(void);
static void pdm_config_print(void);
void am_pdm0_isr(void);

//*****************************************************************************
//
// PDM initialization.
//
//*****************************************************************************
static uint32_t
pdm_init(void)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    if (eClockSource >= eCS_MAX_SOURCES)
    {
        ui32Status = AM_HAL_STATUS_INVALID_ARG;
        return ui32Status;
    }

    const clock_info_table_t *pClockInfo = &clkInfoTable[eClockSource];
    if (!pClockInfo->bValid)
    {
        ui32Status = AM_HAL_STATUS_INVALID_OPERATION;
        return ui32Status;
    }

    g_sPdmConfig.ePDMClkSpeed = pClockInfo->eClkSpd;

    uint32_t ui32Decimation = pClockInfo->ui32DecimationRate;
    if (ui32Decimation)
    {
        //
        // default decimation override by const table
        //
        g_sPdmConfig.ui32DecimationRate = ui32Decimation;
    }

#if defined(AM_PART_APOLLO510)
    switch ( eClockSource )
    {
        case eCS_PLL:
        case eCS_EXTREF_PLL:
            if ( !pClockInfo->bValid)
            {
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
                break;
            }
            ui32Status  = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_SYSPLL, pClockInfo->ui32PdmFreq, NULL);

            break;

        case eCS_HF2ADJ:
            if ( !pClockInfo->bValid)
            {
                ui32Status = AM_HAL_STATUS_INVALID_ARG;
                break;
            }
            ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC2, pClockInfo->ui32PdmFreq, NULL);
            break;
            break;

        default:
            break;
    }
#else //defined(AM_PART_APOLLO510)
    switch ( eClockSource )
    {
        case eCS_PLL:
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
            // Config PLLPOSTDIV to output 24.576 MHz.
            //
            ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLVCO, 245760000, NULL);
            ui32Status |= am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, 24576000, NULL);
            break;

        case eCS_HF2ADJ:
            // not allowed
            return 1 ;
        default:
            break;
    }

#endif // !defined(AM_PART_APOLLO510)

    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("PDM Clock config error\n");
        return ui32Status;
    }

    //
    // Configure the necessary pins.
    //
    am_bsp_pdm_pins_enable(PDM_MODULE);

    //
    // Initialize, power-up, and configure the PDM.
    //
    ui32Status = am_hal_pdm_initialize(PDM_MODULE, &g_sRttStrVars.PDMHandle);
    if (ui32Status)
    {
        am_util_stdio_printf("am_hal_pdm_initialize error\n");
        return ui32Status;
    }
    ui32Status = am_hal_pdm_power_control(g_sRttStrVars.PDMHandle, AM_HAL_PDM_POWER_ON, false);
    if (ui32Status)
    {
        am_util_stdio_printf("am_hal_pdm_power_control error\n");
        return ui32Status;
    }
    ui32Status = am_hal_pdm_configure(g_sRttStrVars.PDMHandle, &g_sPdmConfig);
    if (ui32Status)
    {
        am_util_stdio_printf("am_hal_pdm_power_control error\n");
        return ui32Status;
    }

    //
    // Setup the FIFO threshold.
    //
    am_hal_pdm_fifo_threshold_setup(g_sRttStrVars.PDMHandle, FIFO_THRESHOLD_CNT);

    //
    // Configure and enable PDM interrupts.
    //
    am_hal_pdm_interrupt_enable(g_sRttStrVars.PDMHandle, (AM_HAL_PDM_INT_DCMP | AM_HAL_PDM_INT_DERR));
    NVIC_SetPriority(pdm_interrupts[PDM_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(pdm_interrupts[PDM_MODULE]);

    //
    // Enable PDM, CLK_OUT pad starts outputting clock.
    //
    return am_hal_pdm_enable(g_sRttStrVars.PDMHandle);

}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
static void
pdm_config_print(void)
{
    uint32_t ui32PDMClk, ui32PDMClkOut;
    uint32_t ui32MClkDiv;
    uint32_t ui32DivClkQ;
    uint32_t ui32SampleFreq;

    //
    // Read the config structure to figure out what our internal clock is set
    // to.
    //
    switch (g_sPdmConfig.eClkDivider)
    {
        case AM_HAL_PDM_MCLKDIV_3: ui32MClkDiv = 3; break;
        case AM_HAL_PDM_MCLKDIV_2: ui32MClkDiv = 2; break;
        case AM_HAL_PDM_MCLKDIV_1: ui32MClkDiv = 1; break;
        default:
            ui32MClkDiv = 1;
    }

    switch (g_sPdmConfig.ePDMAClkOutDivder)
    {
        case AM_HAL_PDM_PDMA_CLKO_DIV1:  ui32DivClkQ =  1; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV2:  ui32DivClkQ =  2; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV3:  ui32DivClkQ =  3; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV4:  ui32DivClkQ =  4; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV5:  ui32DivClkQ =  5; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV6:  ui32DivClkQ =  6; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV7:  ui32DivClkQ =  7; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV8:  ui32DivClkQ =  8; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV9:  ui32DivClkQ =  9; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV10: ui32DivClkQ = 10; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV11: ui32DivClkQ = 11; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV12: ui32DivClkQ = 12; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV13: ui32DivClkQ = 13; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV14: ui32DivClkQ = 14; break;
        case AM_HAL_PDM_PDMA_CLKO_DIV15: ui32DivClkQ = 15; break;
        default:
            ui32DivClkQ = 1;
    }

    switch (g_sPdmConfig.ePDMClkSpeed)
    {
        case AM_HAL_PDM_CLK_HFRC_24MHZ:  ui32PDMClk = 24000000; break;
#if defined(AM_PART_APOLLO510)
        // Since HF2ADJ was enabled, HFRC2_31MHZ frequency changed.
        case AM_HAL_PDM_CLK_HFRC2_31MHZ: ui32PDMClk = 24576000; break;
#endif
        case AM_HAL_PDM_CLK_PLL:         ui32PDMClk = 24576000; break;
        default:                         ui32PDMClk = 24000000; break;
    }

    ui32SampleFreq = (ui32PDMClk / ((ui32MClkDiv + 1) * (ui32DivClkQ + 1) * 2 * g_sPdmConfig.ui32DecimationRate));
    ui32PDMClkOut  = (ui32PDMClk / ((ui32MClkDiv + 1) * (ui32DivClkQ + 1)));

    am_util_stdio_printf("PDM Settings:\n");
    am_util_stdio_printf("PDM Clock Out(Hz):      %12d\n", ui32PDMClkOut);
    am_util_stdio_printf("Decimation Rate:        %12d\n", g_sPdmConfig.ui32DecimationRate);
    am_util_stdio_printf("Effective Sample Freq.: %12d\n", ui32SampleFreq);
}


//*****************************************************************************
//
// PDM interrupt handler.
//
//*****************************************************************************
void
am_pdm0_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and clear the interrupt status.
    //
    am_hal_pdm_interrupt_status_get(g_sRttStrVars.PDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(g_sRttStrVars.PDMHandle, ui32Status);

    //
    // DMA ping-pong buffer switch, or workaround for the race condition issue of DMATIP bit.
    //
    am_hal_pdm_interrupt_service(g_sRttStrVars.PDMHandle, ui32Status, &sTransfer);

    //
    // Check if DMA buffer is ready.
    //
    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {
        g_sRttStrVars.bPDMDataReady = true;
    }
}

#if (EXERCISE_DMA_ONOFF_API == 1)
//*****************************************************************************
//
// Init a stimer, ISR will trigger every {WAKE_INTERVAL}.
//
//*****************************************************************************
void stimer_init(void)
{
    //
    // Enable compare A interrupt in STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);

    //
    // Configure the STIMER and run
    //
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);
}

//*****************************************************************************
//
// Stimer ISR, start or stop DMA transfer in this ISR.
//
//*****************************************************************************
void am_stimer_cmpr0_isr(void)
{
    static bool dmaRunning = true;

    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);

    //
    // Stop DMA if it's running, vice versa.
    //
    if (dmaRunning)
    {
        am_util_stdio_printf("Stop DMA transfer.\n");
        am_hal_pdm_dma_stop(PDMHandle);
        dmaRunning = false;
    }
    else
    {
        //
        // PDM filter needs to settle before starting the DMA.
        //
        am_util_delay_ms(60);
        am_util_stdio_printf("Start DMA transfer.\n");
        am_hal_pdm_dma_start(PDMHandle, &sTransfer);
        dmaRunning = true;
    }
}
#endif

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
    am_util_stdio_printf("==============================================\n");
    am_util_stdio_printf("PDM RTT streaming example.\n\n");

    //
    // Disable interrupt and clear the possible pending interrupt.
    //
    am_hal_interrupt_master_disable();
    NVIC_ClearPendingIRQ(pdm_interrupts[PDM_MODULE]);

    //
    // Initialize RTT.
    //
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1, "DataLogger", g_sRttStrVars.rttRecorderBuffer, sizeof(g_sRttStrVars.rttRecorderBuffer), SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    //
    // Initialize PDM-to-PCM module.
    //
    uint32_t ui32Status = pdm_init();
    if (ui32Status)
    {
        am_util_stdio_printf("Error:  pdm_init %d\nexample will hanh\n", ui32Status);
        while (1);
    }


    //
    // Print compiler version, RTT control block address, and PDM configuration.
    //
    am_util_stdio_printf("App Compiler:    %s\n", COMPILER_VERSION);
    am_util_stdio_printf("RTT Control Block Address:  0x%08X\n", (uint32_t)&_SEGGER_RTT);
    pdm_config_print();
    am_util_stdio_printf("==============================================\n");
    am_util_stdio_printf("Run 'python3 rtt_logger.py' command to dump audio data.\n");

    //
    // Start DMA transfer.
    //
    // PDM filter needs to settle before starting the DMA.
    //
    am_util_delay_ms(60);
    ui32Status = am_hal_pdm_dma_start(g_sRttStrVars.PDMHandle, &sTransfer);
    if (ui32Status)
    {
        am_util_stdio_printf("Error:  am_hal_pdm_dma_start %d\nexample will hanh\n", ui32Status);
        while (1);
    }
    //
    // Enable interrupts.
    //
    am_hal_interrupt_master_enable();

#if (EXERCISE_DMA_ONOFF_API == 1)
    stimer_init();
#endif

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // Record the DMA buffer if it's ready.
        //
        if (g_sRttStrVars.bPDMDataReady)
        {
            g_sRttStrVars.bPDMDataReady = false;

#if ENABLE_PING_PONG_DMA
            uint32_t* ui32PDMDatabuffer = (uint32_t*)am_hal_pdm_dma_get_buffer(g_sRttStrVars.PDMHandle);
#else
            uint32_t* ui32PDMDatabuffer = (uint32_t*)sTransfer.ui32TargetAddr;
            sTransfer.ui32TargetAddr = (ui32PDMDatabuffer == &g_ui32PDMDataBuffer[0]) ? (uint32_t)(&g_ui32PDMDataBuffer[NUM_OF_SAMPLES]) : (uint32_t)(&g_ui32PDMDataBuffer[0]);
            am_hal_pdm_dma_transfer_continue(PDMHandle, &sTransfer);
#endif
            am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){(uint32_t)ui32PDMDatabuffer, SIZE_OF_SAMPLES}, false);

            //
            // Re-arrange data.
            //
#if (PCM_WORD_WIDTH == PCM_24_BITS)
            uint8_t pByte[NUM_OF_SAMPLES * 3];
            for (uint32_t i = 0; i < NUM_OF_SAMPLES; i++ )
            {
                pByte[3 * i + 0] = (ui32PDMDatabuffer[i] & 0x0000FF);
                pByte[3 * i + 1] = (ui32PDMDatabuffer[i] & 0x00FF00) >>  8U;
                pByte[3 * i + 2] = (ui32PDMDatabuffer[i] & 0xFF0000) >> 16U;
            }

            //
            // Recording 24-bit data via RTT.
            //
            SEGGER_RTT_Write(1, pByte, NUM_OF_SAMPLES * 3);
#elif (PCM_WORD_WIDTH == PCM_16_BITS)
            uint8_t pByte[NUM_OF_SAMPLES * 2];
            uint16_t* pi16DataPtr = (uint16_t*)ui32PDMDatabuffer;
            for (uint32_t i = 0; i < NUM_OF_SAMPLES; i++ )
            {
                pByte[2 * i + 0] = (pi16DataPtr[i] & 0x00FF);
                pByte[2 * i + 1] = (pi16DataPtr[i] & 0xFF00) >> 8U;
            }

            //
            // Recording 16-bit data via RTT.
            //
            SEGGER_RTT_Write(1, pByte, NUM_OF_SAMPLES * 2);
#endif
        }
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
