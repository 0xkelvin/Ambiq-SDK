//*****************************************************************************
//
//! @file pdm_to_i2s.c
//!
//! @brief An example to show PDM to I2S(controller) operation.
//!
//! @addtogroup audio_examples Audio Examples
//!
//! @defgroup pdm_to_i2s PDM to I2S Example
//! @ingroup audio_examples
//! @{
//!
//! Purpose: This example demonstrates PDM to I2S audio conversion
//! functionality for digital microphone integration with external audio devices.
//! The application showcases audio signal processing, converting PDM
//! data from digital microphones to PCM format and forwarding it via I2S
//! interface.
//!
//! @section pdm_to_i2s_features Key Features
//!
//! 1. @b PDM @b Interface: Receives audio signals from external digital microphone
//!    at 16 kHz sample rate with 2.048 MHz PDM clock
//!
//! 2. @b PCM @b Conversion: Converts PDM data to PCM format for standard
//!    audio processing and transmission
//!
//! 3. @b I2S @b Output: Forwards converted PCM data to external I2S devices
//!    for audio playback or further processing
//!
//! 4. @b Digital @b Microphone @b Support: Compatible with external digital
//!    microphones requiring PDM interface
//!
//! 5. @b Real @b Time @b Processing: Provides continuous audio streaming with
//!    minimal latency for live audio applications
//!
//! @section pdm_to_i2s_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures PDM interface for digital microphone audio capture
//! - Captures audio data at 16 kHz sample rate with 2.048 MHz PDM clock
//! - Converts PDM data to PCM format for standard audio processing
//! - Forwards PCM data to external I2S devices via I2S interface
//! - Implements real-time audio streaming with minimal latency
//! - Provides status reporting via ITM/SWO
//!
//! @section pdm_to_i2s_usage Usage
//!
//! 1. Connect digital microphone to PDM interface pins (PDM0_CLK, PDM0_DATA)
//! 2. Connect external I2S device to I2S interface pins (SCLK, WS, DATA_IN)
//! 3. Compile and download the application to target device
//! 4. Monitor ITM output for status and configuration information
//! 5. Observe real-time audio conversion and streaming
//!
//! @section pdm_to_i2s_configuration Configuration
//!
//! - @b Sample @b Rate: 16 kHz audio sampling rate
//! - @b PDM @b Clock: 2.048 MHz PDM clock frequency
//! - @b PCM @b Format: Standard PCM audio format
//! - @b I2S @b Interface: Controller mode I2S output
//! - @b ITM/SWO: Output for status and results (1MHz)
//!
//! General pin connections:
//! GPIO_50 PDM0_CLK      to CLK_IN of digital microphone
//! GPIO_51 PDM0_DATA     to DATA_OUT of digital microphone
//! I2S_CLK_GPIO_PIN      to SCLK or BIT_CLK of external I2S device
//! I2S_WS_GPIO_PIN       to WS or FRAM_CLK of external I2S device
//! I2S_DATA_OUT_GPIO_PIN to DATA_IN of external I2S device
//!
//! Printing takes place over the ITM at 1MHz.
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

//*****************************************************************************
//
// Example parameters.
//
//*****************************************************************************

//
//! list of pdm clock options
//
typedef enum
{
    eCS_HFRC,
    eCS_PLL,
    eCS_HF2ADJ,
    eCS_EXRREF,
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

const clock_info_table_t clkInfoTable[eCS_MAX_SOURCES] = {
    [eCS_HFRC]      = {.ui32PdmFreq = 24000000, .eClkSpd = AM_HAL_PDM_CLK_HFRC_24MHZ, .bValid = true },
    [eCS_PLL]       = {.ui32PdmFreq = 24576000, .eClkSpd = AM_HAL_PDM_CLK_PLL, .bValid = true  },
    [eCS_HF2ADJ]    = {.ui32PdmFreq = 31250000, .eClkSpd = AM_HAL_PDM_CLK_HFRC2_31MHZ, .bValid = true },
    [eCS_EXRREF]     = {.ui32PdmFreq = 12000000, .ui32DecimationRate = 64, .eClkSpd = AM_HAL_PDM_CLK_EXTREF,  .bValid = true},
    [eCS_EXTREF_PLL] = {.ui32PdmFreq = 24576000, .ui32DecimationRate = 64, .eClkSpd = AM_HAL_PDM_CLK_PLL,  .bValid = true  },
};



typedef struct
{
    am_hal_i2s_clksel_e eI2SClkSel;
    union
    {
        uint32_t eDiv3;
        uint32_t ui32ClockDivideRatio;
    };
    bool bValid;

}
i2s_clock_info_table_t;



const i2s_clock_info_table_t g_i2sClkInfoTable[eCS_MAX_SOURCES] = {
#if defined (AM_PART_APOLLO510)
    [eCS_HFRC]      = {   .eI2SClkSel = eAM_HAL_I2S_CLKSEL_HFRC_3MHz,       .eDiv3 = 1, .bValid = true },
    [eCS_PLL]       = {   .eI2SClkSel = eAM_HAL_I2S_CLKSEL_PLL_FOUT4,       .eDiv3 = 1, .bValid = true },
    [eCS_HF2ADJ]    = {   .eI2SClkSel = eAM_HAL_I2S_CLKSEL_HFRC2_APPROX_4MHz, .eDiv3 = 1, .bValid = true },
    [eCS_EXRREF]     = {   .eI2SClkSel = eAM_HAL_I2S_CLKSEL_XTHS_EXTREF_CLK, .eDiv3 = 1, .bValid = true },
    [eCS_EXTREF_PLL] = {   .eI2SClkSel = eAM_HAL_I2S_CLKSEL_PLL_FOUT4,       .eDiv3 = 1, .bValid = true },
#elif defined(AM_PART_APOLLO330P_510L)
    [eCS_HFRC]      = {  .eI2SClkSel = AM_HAL_I2S_CLKSEL_HFRC_48MHz, .ui32ui32ClockDivideRatio = 48, .bValid = true },
    [eCS_PLL]       = {  .eI2SClkSel = eAM_HAL_I2S_CLKSEL_PLL_FOUT4, .ui32ui32ClockDivideRatio = 3,  .bValid = true },
    [eCS_EXTREF]     = {  .eI2SClkSel = AM_HAL_I2S_CLKSEL_EXTREF,     .ui32ui32ClockDivideRatio = 16, .bValid = true },  // there are two options here AM_HAL_I2S_CLKSEL_NCO_EXTREF
    [eCS_EXTREF_PLL] = {  .eI2SClkSel = eAM_HAL_I2S_CLKSEL_PLL_FOUT4, .ui32ui32ClockDivideRatio = 3,  .bValid = true },

#else
#error "unknown part"
#endif
};

#define PDM_MODULE              0
#define I2S_MODULE              0
#define FIFO_THRESHOLD_CNT      16
#define NUM_OF_PCM_SAMPLES      320
#define DATA_VERIFY             0
#define PDM_ISR_TEST_PAD        2
#define I2S_ISR_TEST_PAD        3


//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

typedef struct
{
    uint32_t ui32FifoOVFCount;
    void    *PDMHandle;
    void    *I2SHandle;
    volatile bool bPDMDataReady;
}
pdm_to_i2s_vars_t;

static pdm_to_i2s_vars_t g_sPdmI2sVars;

//
//! PDM and I2S interrupt number.
//
static const IRQn_Type pdm_interrupts[] =
{
    PDM0_IRQn,
    PDM0_IRQn,
    PDM0_IRQn,
    PDM0_IRQn,
};

static const IRQn_Type i2s_interrupts[] =
{
#if defined(AM_PART_APOLLO510)
    I2S0_IRQn,
    I2S1_IRQn
#elif defined(AM_PART_APOLLO330P_510L)
    I2S0_IRQn
#endif
};

//
//! Used as the ping-pong buffer of PDM DMA.
//! Aligned to 32 bytes to meet data cache requirements.
//
AM_SHARED_RW uint32_t g_ui32PingPongBuffer[2*NUM_OF_PCM_SAMPLES] __attribute__((aligned(32)));

//*****************************************************************************
//
// PDM configuration information.
//
//  2.048 MHz PDM CLK OUT:
//      PDM_CLK_OUT = ePDMClkSpeed / (eClkDivider + 1) / (ePDMAClkOutDivder + 1)
//  16 kHz 24bit Sampling:
//      DecimationRate = 64
//      SAMPLEING_FREQ = PDM_CLK_OUT / (ui32DecimationRate * 2)
//
//*****************************************************************************
am_hal_pdm_config_t g_sPdmConfig =
{
#if defined(AM_PART_APOLLO330P_510L)
    .eWordWidth          = AM_HAL_PDM_DATA_WORD_WIDTH_24BITS,
    .eDataFlowDirection  = AM_HAL_PDM_DATA_FLOW_TO_MEMORY,
    .bI2sMaster          = false,
#endif
    .eClkDivider         = AM_HAL_PDM_MCLKDIV_1,
    .ePDMAClkOutDivder   = AM_HAL_PDM_PDMA_CLKO_DIV5,
    .ui32DecimationRate  = 64,
    .eLeftGain           = AM_HAL_PDM_GAIN_0DB,
    .eRightGain          = AM_HAL_PDM_GAIN_0DB,
    .eStepSize           = AM_HAL_PDM_GAIN_STEP_0_13DB,
    .bHighPassEnable     = AM_HAL_PDM_HIGH_PASS_ENABLE,
    .ui32HighPassCutoff  = 10,
    .bDataPacking        = 1,
    .ePCMChannels        = AM_HAL_PDM_CHANNEL_STEREO,
    .bPDMSampleDelay     = AM_HAL_PDM_CLKOUT_PHSDLY_NONE,
    .ui32GainChangeDelay = AM_HAL_PDM_CLKOUT_DELAY_NONE,
    .bSoftMute           = 0,
    .bLRSwap             = 0,
};

//
//! Ping pong buffer settings.
//
am_hal_pdm_transfer_t g_sTransferPDM =
{
    .ui32TargetAddr        = (uint32_t)(&g_ui32PingPongBuffer[0]),
    .ui32TargetAddrReverse = (uint32_t)(&g_ui32PingPongBuffer[NUM_OF_PCM_SAMPLES]),
    .ui32TotalCount        = NUM_OF_PCM_SAMPLES * sizeof(uint32_t),
};

//*****************************************************************************
//
//! I2S configurations:
//!  - 2 channels
//!  - 16 kHz sample rate
//!  - Standard I2S format
//!  - 32 bits word width
//!  - 24 bits bit-depth
//!
//! SCLK = 16000 * 2 * 32 = 1.024 MHz
//
//*****************************************************************************
static am_hal_i2s_io_signal_t g_sI2SIOConfig =
{
    .sFsyncPulseCfg = {
        .eFsyncPulseType = AM_HAL_I2S_FSYNC_PULSE_ONE_SUBFRAME,
    },
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_LOW,
    .eTxCpol   = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol   = AM_HAL_I2S_IO_RX_CPOL_RISING,
};

static am_hal_i2s_data_format_t g_sI2SDataConfig =
{
    .ePhase                   = AM_HAL_I2S_DATA_PHASE_SINGLE,
    .eChannelLenPhase1        = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .eChannelLenPhase2        = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .ui32ChannelNumbersPhase1 = 2,
    .ui32ChannelNumbersPhase2 = 0,
    .eDataDelay               = 0x1,
    .eSampleLenPhase1         = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
    .eSampleLenPhase2         = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
    .eDataJust                = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
};

static am_hal_i2s_config_t g_sI2SConfig =
{
    .eMode  = AM_HAL_I2S_IO_MODE_MASTER,
    .eXfer  = AM_HAL_I2S_XFER_TX,
    .eASRC  = 0,
    .eData  = &g_sI2SDataConfig,
    .eIO    = &g_sI2SIOConfig,

#if defined (AM_PART_APOLLO510)
    .eDiv3  = 1,

#elif defined(AM_PART_APOLLO330P_510L)
    .eMclkout = AM_HAL_I2S_MCLKOUT_SEL_OFF,
    .ui32MclkoutDiv = 1,
#endif
};

//
// Ping pong buffer settings.
//
static am_hal_i2s_transfer_t g_sTransferI2S =
{
    .ui32TxTotalCount        = NUM_OF_PCM_SAMPLES,
    .ui32TxTargetAddr        = 0x0,
    .ui32TxTargetAddrReverse = 0x0,
};



static uint32_t pdm_init(void);
void am_pdm0_isr(void);
static uint32_t i2s_init(void);
static void pdm_deinit(void *pHandle) __attribute__((unused));
static void i2s_deinit(void *pHandle) __attribute__((unused));

//*****************************************************************************
//
//! @brief PDM initialization.
//
//*****************************************************************************
static uint32_t
pdm_init(void)
{

    if (eClockSource >= eCS_MAX_SOURCES)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    const clock_info_table_t *pClockInfo = &clkInfoTable[eClockSource];
    if (!pClockInfo->bValid)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    g_sPdmConfig.ePDMClkSpeed = pClockInfo->eClkSpd;

    uint32_t ui32Decimation = pClockInfo->ui32DecimationRate;
    if (ui32Decimation)
    {
        g_sPdmConfig.ui32DecimationRate = ui32Decimation;
    }

    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;


#if defined(AM_PART_APOLLO510)
    switch ( eClockSource )
    {
        case eCS_PLL:
        case eCS_EXTREF_PLL:
            ui32Status  = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_SYSPLL, 24576000, NULL);
            break;

        case eCS_HF2ADJ:
            ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC2, AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ, NULL);
            break;

        default:
            break;
    }
#else //defined(AM_PART_APOLLO510)
    switch ( eClockSource )
    {
        case eCS_PLL:
        case eCS_EXTREF_PLL:
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
            am_util_stdio_printf("ERROR: eCS_HF2ADJ not allowed here\n");
            ui32Status = 1;
        default:
            break;
    }

#endif // !defined(AM_PART_APOLLO510)


    do
    {
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("PDM Clock config error\n");
            break;
        }

        //
        // Configure the necessary pins.
        //
        am_bsp_pdm_pins_enable(PDM_MODULE);
        //
        // Initialize, power-up, and configure the PDM.
        //
        ui32Status = am_hal_pdm_initialize(PDM_MODULE, &g_sPdmI2sVars.PDMHandle);
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("am_hal_pdm_initialize error\n");
            break;
        }
        ui32Status = am_hal_pdm_power_control(g_sPdmI2sVars.PDMHandle, AM_HAL_PDM_POWER_ON, false);
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("am_hal_pdm_power_control error\n");
            break;
        }
        ui32Status = am_hal_pdm_configure(g_sPdmI2sVars.PDMHandle, &g_sPdmConfig);
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("am_hal_pdm_configure error\n");
            break;
        }

        //
        // Setup the FIFO threshold.
        //
        ui32Status = am_hal_pdm_fifo_threshold_setup(g_sPdmI2sVars.PDMHandle, FIFO_THRESHOLD_CNT);
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("am_hal_pdm_fifo_threshold_setup error\n");
            break;
        }

        //
        // Configure and enable PDM interrupts (set up to trigger on DMA completion).
        //
        ui32Status = am_hal_pdm_interrupt_enable(g_sPdmI2sVars.PDMHandle, (AM_HAL_PDM_INT_DERR  |\
                                                AM_HAL_PDM_INT_DCMP  |\
                                                AM_HAL_PDM_INT_UNDFL |\
                                                AM_HAL_PDM_INT_OVF));
        if (ui32Status != AM_HAL_STATUS_SUCCESS)
        {
            am_util_stdio_printf("am_hal_pdm_interrupt_enable error\n");
            break;
        }

        NVIC_SetPriority(pdm_interrupts[PDM_MODULE], AM_IRQ_PRIORITY_DEFAULT);
        NVIC_EnableIRQ(pdm_interrupts[PDM_MODULE]);

    } while ( false );

    return ui32Status;
}

//*****************************************************************************
//
//! @brief PDM interrupt handler.
//
//*****************************************************************************
void
am_pdm0_isr(void)
{
    uint32_t ui32Status;

#if DATA_VERIFY
    am_hal_gpio_output_set(PDM_ISR_TEST_PAD);
#endif

    //
    // Read the interrupt status.
    //
    am_hal_pdm_interrupt_status_get(g_sPdmI2sVars.PDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(g_sPdmI2sVars.PDMHandle, ui32Status);

#if DATA_VERIFY
    am_hal_pdm_state_t *pState = (am_hal_pdm_state_t *) PDMHandle;
    static uint32_t ui32Switch = 0;
    if ( ui32Switch )
    {
        ui32Switch = 0;
        for (int i = 0; i < DMA_COUNT; i++)
        {
            ((uint32_t*)pState->ui32BufferPtr)[i] = (i & 0xFF) | 0xAB0000;
        }
    }
    else
    {
        ui32Switch = 1;
        for (int i = 0; i < DMA_COUNT; i++)
        {
            ((uint32_t*)pState->ui32BufferPtr)[i] = (i & 0xFF) | 0xCD0000;
        }
    }

    //am_util_stdio_printf("pdm isr addr = %x.\n", pState->ui32BufferPtr);
#endif

    //
    // Switch ping pong buffer.
    //
    am_hal_pdm_interrupt_service(g_sPdmI2sVars.PDMHandle, ui32Status, &g_sTransferPDM);

    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {
        g_sPdmI2sVars.bPDMDataReady = true;
    }

     if (ui32Status & AM_HAL_PDM_INT_OVF)
     {
         am_hal_pdm_fifo_count_get(g_sPdmI2sVars.PDMHandle);
         am_hal_pdm_fifo_flush(g_sPdmI2sVars.PDMHandle);
         g_sPdmI2sVars.ui32FifoOVFCount++;
     }

#if DATA_VERIFY
    am_hal_gpio_output_clear(PDM_ISR_TEST_PAD);
#endif
}

//*****************************************************************************
//
//! @brief initialization.
//
//*****************************************************************************
static uint32_t
i2s_init(void)
{

    if (eClockSource >= eCS_MAX_SOURCES)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    const i2s_clock_info_table_t *pI2CClkInfo  = &g_i2sClkInfoTable[eClockSource];
    if (!pI2CClkInfo->bValid)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    g_sI2SConfig.eClock = pI2CClkInfo->eI2SClkSel;
    #if defined (AM_PART_APOLLO510)
    g_sI2SConfig.eDiv3 = pI2CClkInfo->eDiv3;
    #elif defined(AM_PART_APOLLO330P_510L)
    g_sI2SConfig.ui32ClockDivideRatio = pI2CClkInfo->ui32ClockDivideRatio;
    #endif
    //
    // Configure the necessary pins.
    //
    am_bsp_i2s_pins_enable(I2S_MODULE, false);

    //
    // Configure the I2S.
    //
    uint32_t ui32Status = am_hal_i2s_initialize(I2S_MODULE, &g_sPdmI2sVars.I2SHandle);
    if (ui32Status)
    {
        am_util_stdio_printf("ERROR: am_hal_i2s_initialize\n");
        return ui32Status;
    }
    ui32Status = am_hal_i2s_power_control(g_sPdmI2sVars.I2SHandle, AM_HAL_I2S_POWER_ON, false);
    if (ui32Status)
    {
        am_util_stdio_printf("ERROR: am_hal_i2s_power_control\n");
        return ui32Status;
    }

    ui32Status = am_hal_i2s_configure(g_sPdmI2sVars.I2SHandle, &g_sI2SConfig);

    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("ERROR: Invalid I2S configuration.\nNote: For Apollo5 Rev.B0, I2S can only use PLL as the clock source.\n");
        return ui32Status;
    }

    return am_hal_i2s_enable(g_sPdmI2sVars.I2SHandle);
}

#define i2s_isr                 am_dspi2s_isrx(I2S_MODULE)
#define am_dspi2s_isrx(n)       am_dspi2s_isr(n)
#define am_dspi2s_isr(n)        am_dspi2s ## n ## _isr
//*****************************************************************************
//
//! @brief I2S interrupt handler.
//
//*****************************************************************************
void
i2s_isr(void)
{
    uint32_t ui32Status;

#if DATA_VERIFY
    am_hal_gpio_output_set(I2S_ISR_TEST_PAD);
#endif

    am_hal_i2s_interrupt_status_get(g_sPdmI2sVars.I2SHandle, &ui32Status, true);
    am_hal_i2s_interrupt_clear(g_sPdmI2sVars.I2SHandle, ui32Status);

    //
    // Swich ping pong buffer.
    //
    am_hal_i2s_interrupt_service(g_sPdmI2sVars.I2SHandle, ui32Status, &g_sI2SConfig);

#if DATA_VERIFY
    am_hal_gpio_output_clear(I2S_ISR_TEST_PAD);
#endif
}

//*****************************************************************************
//
//! @brief deinitialization.
//
//*****************************************************************************
static void pdm_deinit(void *pHandle)
{
    am_hal_pdm_interrupt_clear(pHandle, (AM_HAL_PDM_INT_DERR  |\
                                         AM_HAL_PDM_INT_DCMP  |\
                                         AM_HAL_PDM_INT_UNDFL |\
                                         AM_HAL_PDM_INT_OVF));

    am_hal_pdm_interrupt_disable(pHandle, (AM_HAL_PDM_INT_DERR  |\
                                           AM_HAL_PDM_INT_DCMP  |\
                                           AM_HAL_PDM_INT_UNDFL |\
                                           AM_HAL_PDM_INT_OVF));

    NVIC_DisableIRQ(pdm_interrupts[PDM_MODULE]);

    am_bsp_pdm_pins_disable(PDM_MODULE);

    am_hal_pdm_disable(pHandle);
    am_hal_pdm_power_control(pHandle, AM_HAL_PDM_POWER_OFF, false);
    am_hal_pdm_deinitialize(pHandle);
}

//*****************************************************************************
//
//! @brief I2S deinitialization.
//
//*****************************************************************************
static void i2s_deinit(void *pHandle)
{
    am_hal_i2s_dma_transfer_complete(pHandle);

    am_hal_i2s_interrupt_clear(pHandle, (AM_HAL_I2S_INT_RXDMACPL | AM_HAL_I2S_INT_TXDMACPL));
    am_hal_i2s_interrupt_disable(pHandle, (AM_HAL_I2S_INT_RXDMACPL | AM_HAL_I2S_INT_TXDMACPL));

    NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE]);

    am_bsp_i2s_pins_disable(I2S_MODULE, false);

    am_hal_i2s_disable(pHandle);
    am_hal_i2s_power_control(pHandle, AM_HAL_I2S_POWER_OFF, false);
    am_hal_i2s_deinitialize(pHandle);
}

//*****************************************************************************
//
//! @brief  Main
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
    // Enable the I-Cache and D-Cache., this is done in am_bsp_low_power_init
    //
    //am_hal_cachectrl_icache_enable();
    //am_hal_cachectrl_dcache_enable(true);

    //
    // Print the banner.
    //
    am_bsp_itm_printf_enable();
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("==============================================\n");
    am_util_stdio_printf("PDM_I2S streaming example.\n\n");

#if DATA_VERIFY
    am_hal_gpio_pinconfig(PDM_ISR_TEST_PAD, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(I2S_ISR_TEST_PAD, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(PDM_ISR_TEST_PAD);
    am_hal_gpio_output_clear(I2S_ISR_TEST_PAD);
#endif

    //
    // Initialize PDM-to-PCM module
    //
    uint32_t ui32Status = pdm_init();
    if (ui32Status)
    {
        am_util_stdio_printf("pmd_init error\n..This example will hang");
        while ( 1 );
    }
    ui32Status = am_hal_pdm_enable(g_sPdmI2sVars.PDMHandle);
    if (ui32Status)
    {
        am_util_stdio_printf("am_hal_pdm_enable error: %d\n..This example will hang", ui32Status);
        while ( 1 );
    }

    //
    // Initialize I2S.
    //
    ui32Status = i2s_init();
    if (ui32Status)
    {
        am_util_stdio_printf("i2s_init error: %d\n..This example will hang", ui32Status);
        while ( 1 );
    }

    NVIC_SetPriority(i2s_interrupts[I2S_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE]);
    am_hal_interrupt_master_enable();

    //
    // Start DMA transfer.
    //
    // PDM filter needs to settle before starting the DMA.
    //
    am_util_delay_ms(60);
    ui32Status = am_hal_pdm_dma_start(g_sPdmI2sVars.PDMHandle, &g_sTransferPDM);
    if (ui32Status)
    {
        am_util_stdio_printf("am_hal_pdm_dma_start error: %d\n..This example will hang", ui32Status);
        while ( 1 );
    }

    //
    // Avoid interrupt coming simultaneously.
    //
    am_util_delay_ms(5);

    //
    // Use the reverser buffer of PDM
    //
    g_sTransferI2S.ui32TxTargetAddr = am_hal_pdm_dma_get_buffer(g_sPdmI2sVars.PDMHandle);
    g_sTransferI2S.ui32TxTargetAddrReverse = (g_sTransferI2S.ui32TxTargetAddr == g_sTransferPDM.ui32TargetAddr)? g_sTransferPDM.ui32TargetAddrReverse: g_sTransferPDM.ui32TargetAddr;

    //
    // Start I2S data transaction.
    //
    am_hal_i2s_dma_configure(g_sPdmI2sVars.I2SHandle, &g_sI2SConfig, &g_sTransferI2S);
    am_hal_i2s_dma_transfer_start(g_sPdmI2sVars.I2SHandle, &g_sI2SConfig);

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        //
        // Prepare for deepsleep while SWO is still enabled
        //
        am_bsp_debug_printf_deepsleep_prepare(true);

        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        //
        // Restore from deepsleep
        //
        am_bsp_debug_printf_deepsleep_prepare(false);
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
