//*****************************************************************************
//
//! @file pdm_fft.c
//!
//! @brief An example to show basic PDM operation.
//!
//! @addtogroup audio_examples Audio Examples
//!
//! @defgroup pdm_fft PDM FFT Example
//! @ingroup audio_examples
//! @{
//!
//! Purpose: This example demonstrates PDM interface operation
//! for digital microphone audio capture and real-time frequency analysis using
//! multiple FFT algorithms. The application showcases audio signal
//! acquisition at 16 kHz sample rate, implements various FFT techniques for
//! spectral analysis, and identifies dominant frequency components in the
//! captured audio.
//!
//! @section pdm_fft_features Key Features
//!
//! 1. @b PDM @b Interface: Receives audio signals from digital microphone at
//!    16 kHz sample rate for audio input
//!
//! 2. @b FFT @b Analysis: Performs various FFT algorithms including Complex FFT,
//!    Real FFT, and Q31 FFT for frequency domain analysis
//!
//! 3. @b Frequency @b Detection: Calculates and identifies the dominant frequency
//!    components in the audio signal for spectral analysis
//!
//! 4. @b Multiple @b FFT @b Algorithms: Supports cfft, rfft, rfft_optim_input,
//!    q31fft, and q31_optimized input for different processing requirements
//!
//! 5. @b Real @b Time @b Processing: Provides continuous audio analysis with
//!    minimal latency for live frequency monitoring applications
//!
//! @section pdm_fft_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures PDM interface for digital microphone audio capture
//! - Captures audio data at 16 kHz sample rate
//! - Implements multiple FFT algorithms for spectral analysis
//! - Identifies and reports dominant frequency components
//! - Provides real-time audio analysis and status reporting via SWO
//!
//! @section pdm_fft_usage Usage
//!
//! 1. Connect digital microphone to specified GPIO pins (CLK_IN, DATA_OUT)
//! 2. Compile and download the application to target device
//! 3. Monitor SWO output for frequency analysis results
//! 4. Observe real-time dominant frequency detection
//!
//! @section pdm_fft_configuration Configuration
//!
//! - @b Sample @b Rate: 16 kHz audio sampling rate
//! - @b FFT @b Size: Configurable FFT size (default: 1024)
//! - @b FFT @b Algorithms: cfft, rfft, rfft_optim_input, q31fft, q31_optimized
//! - @b SWO/ITM: Output for status and results (1MHz)
//!
//! The data is captured from a digital microphone with the following pin connections:
//!
//! - **GPIO 50** to **CLK_IN** of the digital microphone
//! - **GPIO 51** to **DATA_OUT** of the digital microphone
//!
//! Printing is done over the **SWO** at 1 Mbps baud rate.
//!
//! @note The system configuration, such as clock source and PDM settings, is highly
//!       specific to the Apollo5 hardware platform, so adjust the configuration accordingly.
//!
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
#include <arm_math.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "fft_timer_control.h"
#include <arm_math.h>
#include <arm_math_types.h>

//*****************************************************************************
// Define constants and example parameters
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
    uint32_t  ui32PdmFreq ;
    uint32_t  ui32DecimationRate;
    am_hal_pdm_clkspd_e eClkSpd ;
    bool bValid;

}
clock_info_table_t;

const clock_info_table_t clkInfoTable[eCS_MAX_SOURCES] = {
    [eCS_HFRC]       = {.ui32PdmFreq = 24000000, .eClkSpd = AM_HAL_PDM_CLK_HFRC_24MHZ, .bValid = true },
    [eCS_PLL]        = {.ui32PdmFreq = 24576000, .eClkSpd = AM_HAL_PDM_CLK_PLL, .bValid = true },
    [eCS_HF2ADJ]     = {.ui32PdmFreq = 31250000, .eClkSpd = AM_HAL_PDM_CLK_HFRC2_31MHZ, .bValid = true },
    [eCS_EXTREF]     = {.ui32PdmFreq = 12000000, .ui32DecimationRate = 64, .eClkSpd = AM_HAL_PDM_CLK_EXTREF, .bValid = true },
    [eCS_EXTREF_PLL] = {.ui32PdmFreq = 24576000, .ui32DecimationRate = 64, .eClkSpd = AM_HAL_PDM_CLK_PLL, .bValid = true },
};

#define PDM_MODULE          0
#define PDM_FFT_SIZE        1024
#define PDM_FFT_BYTES       (PDM_FFT_SIZE * 4)
#define PRINT_PDM_DATA      0
#define PRINT_FFT_DATA      0
#define FIFO_THRESHOLD_CNT  16
#define DMA_BYTES           PDM_FFT_BYTES

//*****************************************************************************
//
// Insert compiler version at compile time.
//
//*****************************************************************************
#define STRINGIZE_VAL(n)                    STRINGIZE_VAL2(n)
#define STRINGIZE_VAL2(n)                   #n

#ifdef __GNUC__
#define COMPILER_VERSION                    ("GCC " __VERSION__)
#elif defined(__ARMCC_VERSION)
#define COMPILER_VERSION                    ("ARMCC " STRINGIZE_VAL(__ARMCC_VERSION))
#elif defined(__KEIL__)
#define COMPILER_VERSION                    "KEIL_CARM " STRINGIZE_VAL(__CA__)
#elif defined(__IAR_SYSTEMS_ICC__)
#define COMPILER_VERSION                    __VERSION__
#else
#define COMPILER_VERSION                    "Compiler unknown"
#endif

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************

//
// Define Buffers used for FFT.
//
typedef union
{
    q31_t q[PDM_FFT_SIZE + 4];
    float f[PDM_FFT_SIZE + 4];
}
fq_data_ut;

typedef union
{
    q31_t q[PDM_FFT_SIZE * 2 + 4];
    float f[PDM_FFT_SIZE * 2 + 4];
}
fq_ut;

typedef struct
{
    float fPDMTimeDomain[PDM_FFT_SIZE * 2 + 4]; //!< used in CFFT
    float fPDMMagnitudes[PDM_FFT_SIZE * 2 + 4]; //!< used in CFFT

    fq_data_ut fftInput;        //!< holds scaled fft input data
    fq_ut fftout;               //!< holds fft output
    fq_ut fftmag;               //!< holds fft magnitude output

    void *PDMHandle;    //!< pdm handle

    volatile bool bPDMDataReady;
    uint32_t ui32SampleFreq;       //!< computed frequency resolution
    uint32_t ui32FifoOVFCount;     //!< overflow count
}
global_vars_t;

global_vars_t g_globv; //!< globals used in this module

//
// Used as the ping-pong buffer of DMA.
// Aligned to 32 bytes to meet data cache requirements.
//
AM_SHARED_RW uint32_t g_ui32PDMDataBuffer[PDM_FFT_SIZE*2] __attribute__((aligned(32)));

//
// PDM interrupt number.
//
static const IRQn_Type pdm_interrupts[] =
{
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
    .eWordWidth          = AM_HAL_PDM_DATA_WORD_WIDTH_24BITS,
    .eDataFlowDirection  = AM_HAL_PDM_DATA_FLOW_TO_MEMORY,
    .bI2sMaster          = false,
    #endif

    .eClkDivider         = AM_HAL_PDM_MCLKDIV_1,
    .ePDMAClkOutDivder   = AM_HAL_PDM_PDMA_CLKO_DIV5,
    .ui32DecimationRate  = 64,  // this is the default, it can be overriden
    .eLeftGain           = AM_HAL_PDM_GAIN_0DB,
    .eRightGain          = AM_HAL_PDM_GAIN_0DB,
    .eStepSize           = AM_HAL_PDM_GAIN_STEP_0_13DB,
    .bHighPassEnable     = AM_HAL_PDM_HIGH_PASS_ENABLE,
    .ui32HighPassCutoff  = 10,
    .ePCMChannels        = AM_HAL_PDM_CHANNEL_LEFT,
    .bPDMSampleDelay     = AM_HAL_PDM_CLKOUT_PHSDLY_NONE,
    .ui32GainChangeDelay = AM_HAL_PDM_CLKOUT_DELAY_NONE,
    .bSoftMute           = 0,
    .bLRSwap             = 0,
};

am_hal_pdm_transfer_t sTransfer =
{
    //
    // DMA ping-pong buffer.
    //
    .ui32TargetAddr        = (uint32_t)(&g_ui32PDMDataBuffer[0]),
    .ui32TargetAddrReverse = (uint32_t)(&g_ui32PDMDataBuffer[PDM_FFT_SIZE]),
    .ui32TotalCount        = DMA_BYTES,
};

typedef union
{
    q31_t maxq31;
    float maxF32;
    int32_t maxI32;

} MaxValType;

//
//! defines return values from each fft instance
//
typedef struct
{
    uint32_t t_setup;
    uint32_t t_fft;
    uint32_t t_conv;
    uint32_t index;
    uint32_t loudFreq;
    MaxValType maxValue;

} fft_return_info_t;

const float odivx = 1.0f / 65536.0f;

static uint32_t pdm_init(void);
static uint32_t pdm_config_print(void);
static uint32_t pcm_q31_opt(int32_t *pi32PDMData, fft_return_info_t *ri);
static uint32_t pcm_q31(int32_t *pi32PDMData, fft_return_info_t *ri);
static uint32_t pcm_rfft_optim(int32_t *pi32PDMData, fft_return_info_t *ri);
static uint32_t pcm_rfft(int32_t *pi32PDMData, fft_return_info_t *ri);
static uint32_t pcm_cfft(int32_t *pi32PDMData, fft_return_info_t *ri);

static void printAssist( char *str, fft_return_info_t *pri);

static void pcm_fft_print(int32_t *pi32PDMData);

//*****************************************************************************
// Function to initialize the PDM (Pulse Density Modulation) module
//*****************************************************************************
static uint32_t
pdm_init(void)
{

    if ( eClockSource >= eCS_MAX_SOURCES)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    const clock_info_table_t *pClockInfo = &clkInfoTable[eClockSource];
    if ( !pClockInfo->bValid)
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
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
            break;

         default:
            break;
    }
#else
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

#endif

    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        return ui32Status;
    }

    //
    // Configure the necessary pins.
    //
    am_bsp_pdm_pins_enable(PDM_MODULE);

    //
    // Initialize, power-up, and configure the PDM.
    //
    am_hal_pdm_initialize(PDM_MODULE, &g_globv.PDMHandle);
    am_hal_pdm_power_control(g_globv.PDMHandle, AM_HAL_PDM_POWER_ON, false);
    am_hal_pdm_configure(g_globv.PDMHandle, &g_sPdmConfig);

    //
    // Setup the FIFO threshold.
    //
    am_hal_pdm_fifo_threshold_setup(g_globv.PDMHandle, FIFO_THRESHOLD_CNT);

    //
    // Configure and enable PDM interrupts.
    //
    am_hal_pdm_interrupt_enable(g_globv.PDMHandle, (AM_HAL_PDM_INT_DCMP | AM_HAL_PDM_INT_DERR));
    NVIC_SetPriority(pdm_interrupts[PDM_MODULE], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(pdm_interrupts[PDM_MODULE]);

    //
    // Enable PDM, CLK_OUT pad starts outputting clock.
    //
    return am_hal_pdm_enable(g_globv.PDMHandle);
}

//*****************************************************************************
//
// Print PDM configuration data.
//
//*****************************************************************************
static uint32_t
pdm_config_print(void)
{
    //
    // Read the config structure to figure out what our internal clock is set
    // to.
    //

    uint32_t ui32MClkDiv = g_sPdmConfig.eClkDivider;
    if (ui32MClkDiv == 0 || ui32MClkDiv > AM_HAL_PDM_MCLKDIV_3 )
    {
        ui32MClkDiv = 1;
    }

    uint32_t ui32DivClkQ = g_sPdmConfig.ePDMAClkOutDivder;
    if (ui32DivClkQ == 0 || ui32DivClkQ > AM_HAL_PDM_PDMA_CLKO_DIV15 )
    {
        ui32DivClkQ = 1;
    }

    //
    // Record the effective sample frequency. We'll need it later to print the
    // loudest frequency from the sample.
    //
    uint32_t ui32PDMClk = clkInfoTable[eClockSource].ui32PdmFreq;
    if ( ui32PDMClk == 0 )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    uint32_t ui32Denom_A        = (ui32MClkDiv + 1) * (ui32DivClkQ + 1);
    uint32_t ui32Denom_B        = ui32Denom_A *  2 * g_sPdmConfig.ui32DecimationRate;

    g_globv.ui32SampleFreq    = ui32PDMClk / ui32Denom_B;
    uint32_t ui32PDMClkOut    = ui32PDMClk / ui32Denom_A;

    float fFrequencyResolution = (float) g_globv.ui32SampleFreq / (float) PDM_FFT_SIZE;

    am_util_stdio_printf("PDM Settings:\n");
    am_util_stdio_printf("PDM Clock Out(Hz):      %12d\n", ui32PDMClkOut);
    am_util_stdio_printf("Decimation Rate:        %12d\n", g_sPdmConfig.ui32DecimationRate);
    am_util_stdio_printf("Effective Sample Freq.: %12d\n", g_globv.ui32SampleFreq);
    am_util_stdio_printf("FFT Length:             %12d\n\n", PDM_FFT_SIZE);
    am_util_stdio_printf("FFT Resolution: %15.3f Hz\n", fFrequencyResolution);

    return AM_HAL_STATUS_SUCCESS;
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
    am_hal_pdm_interrupt_status_get(g_globv.PDMHandle, &ui32Status, true);
    am_hal_pdm_interrupt_clear(g_globv.PDMHandle, ui32Status);

    //
    // Swich ping pong buffer.
    //
    am_hal_pdm_interrupt_service(g_globv.PDMHandle, ui32Status, &sTransfer);

    if (ui32Status & AM_HAL_PDM_INT_DCMP)
    {
        g_globv.bPDMDataReady = true;
    }

     if (ui32Status & AM_HAL_PDM_INT_OVF)
     {
        uint32_t count = am_hal_pdm_fifo_count_get(g_globv.PDMHandle);
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
        volatile uint32_t ui32FifoDummy;
        for ( uint8_t i = 0; i < count; i++ )
        {
            ui32FifoDummy = PDMn(PDM_MODULE)->FIFOREAD;
        }
#pragma GCC diagnostic pop
#endif
        am_hal_pdm_fifo_flush(g_globv.PDMHandle);

        g_globv.ui32FifoOVFCount++;
     }
}
//*****************************************************************************
//
//! @brief  Function to process the PDM data and perform Q31 FFT (optimized version)
//!
//! @param pi32PDMData  pointer to input data
//! @param ri pointer to output data struct for this function
//!
//! @return dominant freq
//
//*****************************************************************************
static uint32_t
pcm_q31_opt(int32_t *pi32PDMData, fft_return_info_t *ri )
{
    static bool first = false;
    static arm_rfft_instance_q31 QS;
    if ( !first )
    {
        first = true;
        arm_rfft_init_q31( &QS, PDM_FFT_SIZE, 0, 1 );
    }
    uint32_t t0 = am_hal_timer_read(TIMER_TEST_NUM);

    arm_shift_q31(pi32PDMData, 8, g_globv.fftInput.q, PDM_FFT_SIZE);

    uint32_t t1 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_rfft_q31( &QS, g_globv.fftInput.q, g_globv.fftout.q );
    uint32_t t2 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_cmplx_mag_squared_q31( g_globv.fftout.q, g_globv.fftmag.q, PDM_FFT_SIZE / 2);
    uint32_t t3 = am_hal_timer_read(TIMER_TEST_NUM);

    ri->t_setup = t1 - t0;
    ri->t_fft = t2 - t1;
    ri->t_conv = t3 - t2;

    uint32_t ui32JMaxIndex;
    arm_max_q31(&g_globv.fftmag.q[1], (PDM_FFT_SIZE / 2) - 1, &ri->maxValue.maxq31, &ui32JMaxIndex);

    ui32JMaxIndex++;

    ri->index = ui32JMaxIndex;
    ri->loudFreq = (g_globv.ui32SampleFreq * ui32JMaxIndex) / PDM_FFT_SIZE;

    return ri->loudFreq;
}

//*****************************************************************************
//
//! @brief Function to process the PDM data and perform Q31 FFT (non-optimized version)
//!
//! @note this is an example and if efficent and not recommended. Use the
//! optimized version
//!
//! @param pi32PDMData  pointer to input data
//! @param ri pointer to output data struct for this function
//!
//! @return dominant freq
//
//*****************************************************************************
static uint32_t
pcm_q31(int32_t *pi32PDMData, fft_return_info_t *ri)
{
    static bool first = false;
    static arm_rfft_instance_q31 QS;
    if ( !first )
    {
        first = true;
        arm_rfft_init_q31( &QS, PDM_FFT_SIZE, 0, 1 );
    }

    uint32_t t0 = am_hal_timer_read(TIMER_TEST_NUM);
    for (uint32_t i = 0; i < PDM_FFT_SIZE; i++)
    {
        // simpler version of this does not create correct q31 data
        // that is
        // fftInput.q[i][i] =  (q31_t) (pi32PDMData[i] << 8);
        // this is now very inefficient, so use the vector call, in the optim example

        int64_t temp = (int64_t)pi32PDMData[i] << 8;
        if (temp > INT32_MAX)
        {
            temp = INT32_MAX;
        }
        else if (temp < INT32_MIN)
        {
            temp = INT32_MIN;
        }

        g_globv.fftInput.q[i]       = (q31_t) temp;  // covnert to Q31 format
    }

    uint32_t t1 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_rfft_q31( &QS, g_globv.fftInput.q, g_globv.fftout.q );
    uint32_t t2 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_cmplx_mag_squared_q31( g_globv.fftout.q, g_globv.fftmag.q, PDM_FFT_SIZE / 2);
    uint32_t t3 = am_hal_timer_read(TIMER_TEST_NUM);

    ri->t_setup = t1 - t0;
    ri->t_fft = t2 - t1;
    ri->t_conv = t3 - t2;

    uint32_t ui32JMaxIndex;
    arm_max_q31(&g_globv.fftmag.q[1], (PDM_FFT_SIZE / 2) - 1, &ri->maxValue.maxq31, &ui32JMaxIndex);

    ui32JMaxIndex++;

    ri->index = ui32JMaxIndex;
    ri->loudFreq = (g_globv.ui32SampleFreq * ui32JMaxIndex) / PDM_FFT_SIZE;

    return ri->loudFreq;
}
//*****************************************************************************
//
//! @brief Function to perform Real FFT (optimized) on PDM data
//! Optimized real FFT processing using ARM's fast RFFT functions
//!
//! @param pi32PDMData  pointer to input data
//! @param ri pointer to output data struct for this function
//!
//! @return dominant freq
//
//*****************************************************************************
static uint32_t
pcm_rfft_optim(int32_t *pi32PDMData, fft_return_info_t *ri )
{
    static bool first = false;
    static arm_rfft_fast_instance_f32  RFS;
    if ( !first )
    {
        first = true;
        arm_rfft_fast_init_f32( &RFS, PDM_FFT_SIZE );
    }
    uint32_t t0 = am_hal_timer_read(TIMER_TEST_NUM);
    // Vectorized processing (4 elements per iteration)
    float32x4_t scale_vect = vdupq_n_f32(odivx);
    for (uint32_t i = 0; i < (PDM_FFT_SIZE & ~3U); i += 4)
    {
        //
        // Load 4 elements (int32)
        //
        int32x4_t pdm_data = vldrwq_s32(&pi32PDMData[i]);

        //
        // Left shift by 8 (multiply by 256)
        //
        pdm_data = vshlq_n_s32(pdm_data, 8);

        //
        // Convert to float32 and multiply by odivx
        //
        float32x4_t float_data = vcvtq_n_f32_s32(pdm_data, 8); // Converts int32 to float
        float_data = vmulq_f32(float_data, scale_vect); // Scale

        //
        // Store result
        //
        vst1q_f32(&g_globv.fftInput.f[i], float_data);
    }

    uint32_t t1 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_rfft_fast_f32( &RFS, g_globv.fftInput.f, g_globv.fftout.f, 0 );
    uint32_t t2 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_cmplx_mag_squared_f32(g_globv.fftout.f, g_globv.fftmag.f, PDM_FFT_SIZE / 2);
    uint32_t t3 = am_hal_timer_read(TIMER_TEST_NUM);

    ri->t_setup = t1 - t0;
    ri->t_fft = t2 - t1;
    ri->t_conv = t3 - t2;

    uint32_t ui32FMaxIndex;
    arm_max_f32(&g_globv.fftmag.f[1], (PDM_FFT_SIZE / 2) - 1, &ri->maxValue.maxF32, &ui32FMaxIndex);

    ui32FMaxIndex++;
    ri->index = ui32FMaxIndex;

    ri->loudFreq = (g_globv.ui32SampleFreq * ui32FMaxIndex) / PDM_FFT_SIZE;

    return ri->loudFreq;
}

//*****************************************************************************
//
//! @brief Function to perform Real FFT on PDM data
//! Standard real FFT processing using ARM's RFFT functions
//!
//! @param pi32PDMData  pointer to input data
//! @param ri pointer to output data struct for this function
//!
//! @return dominant freq
//
//*****************************************************************************
static uint32_t
pcm_rfft(int32_t *pi32PDMData, fft_return_info_t *ri )
{
    static bool first = false;
    static arm_rfft_fast_instance_f32  RFS;
    if ( !first )
    {
        first = true;
        arm_rfft_fast_init_f32( &RFS, PDM_FFT_SIZE );
    }
    uint32_t t0 = am_hal_timer_read(TIMER_TEST_NUM);
    for (uint32_t i = 0; i < PDM_FFT_SIZE; i++)
    {
        if (PRINT_PDM_DATA)
        {
            am_util_stdio_printf("%d\n", pi32PDMData[i]);
        }
        // The PDM output data here is signed 24 bit,
        // Need to left-shift 8 to create a usable signed 32-bit number
        int32_t s32Data = pi32PDMData[i] << 8;

        float fltCvt    = (float)s32Data * odivx;
        g_globv.fftInput.f[i] = fltCvt;
    }

    uint32_t t1 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_rfft_fast_f32( &RFS, g_globv.fftInput.f, g_globv.fftout.f, 0 );
    uint32_t t2 = am_hal_timer_read(TIMER_TEST_NUM);

    arm_cmplx_mag_squared_f32(g_globv.fftout.f, g_globv.fftmag.f, PDM_FFT_SIZE / 2);
    uint32_t t3 = am_hal_timer_read(TIMER_TEST_NUM);

    ri->t_setup = t1 - t0;
    ri->t_fft = t2 - t1;
    ri->t_conv = t3 - t2;

    uint32_t ui32FMaxIndex;
    arm_max_f32(&g_globv.fftmag.f[1], (PDM_FFT_SIZE / 2) - 1, &ri->maxValue.maxF32, &ui32FMaxIndex);

    ui32FMaxIndex++;
    ri->index = ui32FMaxIndex;

    ri->loudFreq = (g_globv.ui32SampleFreq * ui32FMaxIndex) / PDM_FFT_SIZE;

    return ri->loudFreq;
}

//*****************************************************************************
//
//! @brief Function to perform Complex FFT on PDM data
//! Complex FFT processing using ARM's complex FFT functions
//!
//! @param pi32PDMData  pointer to input data
//! @param ri pointer to output data struct for this function
//!
//! @return dominant freq
//
//*****************************************************************************
static uint32_t
pcm_cfft(int32_t *pi32PDMData, fft_return_info_t *ri )
{
    static bool first = false;
    static arm_cfft_instance_f32 S;
    if ( !first )
    {
        first = true;
        arm_cfft_init_f32( &S, PDM_FFT_SIZE );
    }
    uint32_t t0 = am_hal_timer_read(TIMER_TEST_NUM);
    for (uint32_t i = 0; i < PDM_FFT_SIZE; i++)
    {
        if (PRINT_PDM_DATA)
        {
            am_util_stdio_printf("%d\n", pi32PDMData[i]);
        }
        // The PDM output data here is signed 24 bit,
        // Need to left-shift 8 to create a usable signed 32-bit number
        int32_t s32Data = pi32PDMData[i] << 8;

        float fltCvt    = (float)s32Data * odivx;
        g_globv.fPDMTimeDomain[2 * i]  = fltCvt;
        g_globv.fPDMTimeDomain[2 * i + 1] = 0.0f;
    }

    uint32_t t1 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_cfft_f32( &S, g_globv.fPDMTimeDomain, 0, 1);
    uint32_t t2 = am_hal_timer_read(TIMER_TEST_NUM);
    arm_cmplx_mag_squared_f32(g_globv.fPDMTimeDomain, g_globv.fPDMMagnitudes, PDM_FFT_SIZE);
    uint32_t t3 = am_hal_timer_read(TIMER_TEST_NUM);

    ri->t_setup = t1 - t0;
    ri->t_fft = t2 - t1;
    ri->t_conv = t3 - t2;

    uint32_t ui32MaxIndex;
    arm_max_f32(g_globv.fPDMMagnitudes, PDM_FFT_SIZE / 2, &ri->maxValue.maxF32, &ui32MaxIndex);
    ri->index = ui32MaxIndex;
    ri->loudFreq = (g_globv.ui32SampleFreq * ui32MaxIndex) / PDM_FFT_SIZE;

    return ri->loudFreq;
}

//*****************************************************************************
//
//! @brief Helper function to print FFT results
//
//*****************************************************************************
static void
printAssist( char *str, fft_return_info_t *pri)
{
    am_util_stdio_printf( "%s   %5d, %8d %8d %8d %8d\n", str,
        pri->loudFreq, pri->index, pri->t_setup, pri->t_fft, pri->t_conv);
}
//*****************************************************************************
//
//! @brief Helper function to compute FFT results
//!
//! @param pi32PDMData  pointer to input data
//!
//
//*****************************************************************************
static void
pcm_fft_print(int32_t *pi32PDMData)
{
    fft_return_info_t q31_fft;
    fft_return_info_t q31_fft_opt;
    fft_return_info_t rfft;
    fft_return_info_t rfft_opt;
    fft_return_info_t cfft;

    __DMB();

    pcm_q31(pi32PDMData, &q31_fft);
    pcm_q31_opt(pi32PDMData, &q31_fft_opt);
    pcm_rfft(pi32PDMData, &rfft);
    pcm_rfft_optim(pi32PDMData, &rfft_opt);
    pcm_cfft(pi32PDMData, &cfft);

    if (PRINT_FFT_DATA)
    {
        am_util_stdio_printf("Loudest frequency bin: %d %d %d\n",
            q31_fft.loudFreq, rfft.loudFreq, cfft.loudFreq );
    }
    printAssist( "\nq31   ", &q31_fft);
    printAssist( "q31o  ", &q31_fft_opt);
    printAssist( "rfft  ", &rfft);
    printAssist( "rffto ", &rfft_opt);
    printAssist( "cfft  ", &cfft);

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
    // Initialize the printf interface for ITM output
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("==============================================\n");
    am_util_stdio_printf("PDM FFT example.\n\n");

    #if defined(AM_PART_APOLLO510)
    configAndStartTestTimer(AM_HAL_TIMER_CLOCK_XTAL_HS);
    #else
    if (eClockSource == eCS_HF2ADJ)
    {
        am_util_stdio_printf("HFRC2 is not supported on Apollo510L\n This example will hang\n");
        while (1);
    }
    #if defined(AM_PART_APOLLO330P_510L)
    configAndStartTestTimer(AM_HAL_TIMER_CLOCK_RF_XTAL_DIV2);

    #endif
    #endif

    //
    // Initialize PDM-to-PCM module
    //
    uint32_t ui32Stat = pdm_init();
    if (ui32Stat)
    {
        am_util_stdio_printf("Pdm init failure %d\n This example will hang\n", ui32Stat);
        while (true) ;
    }

    //
    // Print compiler version and PDM configuration.
    //
    am_util_stdio_printf("App Compiler:    %s\n", COMPILER_VERSION);
    pdm_config_print();

    //
    // Start DMA transfer.
    //
    // PDM filter needs to settle before starting the DMA.
    //
    am_util_delay_ms(60);
    ui32Stat = am_hal_pdm_dma_start(g_globv.PDMHandle, &sTransfer);
    if ( ui32Stat )
    {
        am_util_stdio_printf("am_hal_pdm_dma_start failure %d\n This example will hang\n", ui32Stat);
        while (true) ;
    }
    //
    // Enable interrupts.
    //
    am_hal_interrupt_master_enable();

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        if (g_globv.bPDMDataReady)
        {
            g_globv.bPDMDataReady = false;
            int32_t *pPDMData = (int32_t *) am_hal_pdm_dma_get_buffer(g_globv.PDMHandle);
            am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){(uint32_t)pPDMData, DMA_BYTES}, false);
            pcm_fft_print(pPDMData);
        }

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
