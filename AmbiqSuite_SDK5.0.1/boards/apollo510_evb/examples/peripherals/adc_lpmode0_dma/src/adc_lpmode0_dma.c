//*****************************************************************************
//
//! @file adc_lpmode0_dma.c
//!
//! @brief This example takes samples with the ADC at high-speed using DMA.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup adc_lpmode0_dma ADC Low Power Mode 0 DMA Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates high-speed ADC sampling using DMA in
//! Low Power Mode 0 (LPMODE0). The application showcases ADC
//! configuration with internal timer triggering, efficient data transfer
//! using DMA, and low-power operation for high-throughput data acquisition.
//! The example implements circular buffer management and real-time data
//! processing for analog signal capture.
//!
//! @section adc_lpmode0_dma_features Key Features
//!
//! 1. @b High @b Speed @b ADC @b Sampling: Achieves 1.2Msps sampling rate
//!    using internal timer and HFRC clock
//!
//! 2. @b DMA @b Data @b Transfer: Utilizes DMA to efficiently move ADC data
//!    from FIFO to SRAM buffer with minimal CPU intervention
//!
//! 3. @b Low @b Power @b Operation: Operates ADC in LPMODE0 for reduced
//!    power consumption during high-speed sampling
//!
//! 4. @b Circular @b Buffer @b Management: Implements circular buffer for
//!    continuous data acquisition and processing
//!
//! 5. @b Real @b Time @b Data @b Processing: Supports real-time data
//!    processing and status reporting via SWO
//!
//! @section adc_lpmode0_dma_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures ADC with internal timer and HFRC clock
//! - Sets up DMA for efficient data transfer to SRAM buffer
//! - Implements circular buffer management for continuous sampling
//! - Processes and averages ADC samples in real time
//! - Reports status and results via SWO/ITM
//!
//! @section adc_lpmode0_dma_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Connect analog input to ADC SE0 pin (Pin 19)
//! 3. Monitor SWO output for sampling status and results
//! 4. Observe high-speed, low-power ADC operation
//!
//! @section adc_lpmode0_dma_configuration Configuration
//!
//! - @b ADC_SAMPLE_BUF_SIZE: Configurable ADC sample buffer size (default: 128)
//! - @b ADC @b clock: Internal timer clocked by HFRC
//! - @b SWO/ITM: Output for status and results (1MHz)
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
// Define a circular buffer to hold the ADC samples
//
//*****************************************************************************
//
// ADC Sample buffer.
//
#define ADC_SAMPLE_BUF_SIZE 128

AM_SHARED_RW uint32_t g_ui32ADCSampleBuffer[ADC_SAMPLE_BUF_SIZE]  __attribute__ ((aligned (32)));

am_hal_adc_sample_t SampleBuffer[ADC_SAMPLE_BUF_SIZE];

//
// ADC Device Handle.
//
static void *g_ADCHandle;

//
// ADC DMA complete flag.
//
volatile bool                   g_bADCDMAComplete;

//
// ADC DMA error flag.
//
volatile bool                   g_bADCDMAError;

volatile uint32_t g_ui32AdcFifoCount = 0;

//
// Define the ADC SE0 pin to be used.
//
const am_hal_gpio_pincfg_t g_AM_PIN_19_ADCSE0 =
{
    .GP.cfg_b.uFuncSel       = AM_HAL_PIN_19_ADCSE0,
};

//*****************************************************************************
//
// Interrupt handler for the ADC.
//
//*****************************************************************************
void
am_adc_isr(void)
{
    uint32_t ui32IntMask;
    //
    // Read the interrupt status.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_status(g_ADCHandle, &ui32IntMask, false))
    {
        am_util_stdio_printf("Error reading ADC interrupt status\n");
    }

    //
    // Clear the ADC interrupt.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_clear(g_ADCHandle, ui32IntMask))
    {
        am_util_stdio_printf("Error clearing ADC interrupt status\n");
    }

    //
    // If we got a DMA complete, set the flag.
    //
    if (ui32IntMask & AM_HAL_ADC_INT_FIFOOVR1)
    {
        if ( ADCn(0)->DMASTAT_b.DMACPL )
        {
            g_bADCDMAComplete = true;
        }
    }

    //
    // If we got a DMA error, set the flag.
    //
    if ( ui32IntMask & AM_HAL_ADC_INT_DERR )
    {
        g_bADCDMAError = true;
    }
}

//*****************************************************************************
//
// Set up the core for sleeping, and then go to sleep.
//
//*****************************************************************************
void
sleep(void)
{
    //
    // Disable things that can't run in sleep mode.
    //
    am_bsp_debug_printf_disable();

    //
    // Go to Deep Sleep.
    //
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

    //
    // Re-enable peripherals for run mode.
    //
    am_bsp_debug_printf_enable();
}

//*****************************************************************************
//
// Configure the ADC.
//
//*****************************************************************************
void
adc_config_dma(void)
{
    am_hal_adc_dma_config_t       ADCDMAConfig;

    //
    // Configure the ADC to use DMA for the sample transfer.
    //
    ADCDMAConfig.bDynamicPriority = true;
    ADCDMAConfig.ePriority = AM_HAL_ADC_PRIOR_SERVICE_IMMED;
    ADCDMAConfig.bDMAEnable = true;
    ADCDMAConfig.ui32SampleCount = ADC_SAMPLE_BUF_SIZE;
    ADCDMAConfig.ui32TargetAddress = (uint32_t)g_ui32ADCSampleBuffer;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_dma(g_ADCHandle, &ADCDMAConfig))
    {
        am_util_stdio_printf("Error - configuring ADC DMA failed.\n");
    }

    //
    // Reset the ADC DMA flags.
    //
    g_bADCDMAComplete = false;
    g_bADCDMAError = false;
}

//*****************************************************************************
//
// Configure the ADC.
//
//*****************************************************************************
void
adc_config(void)
{
    am_hal_adc_config_t           ADCConfig;
    am_hal_adc_slot_config_t      ADCSlotConfig;

    //
    // Initialize the ADC and get the handle.
    //
    if ( AM_HAL_STATUS_SUCCESS != am_hal_adc_initialize(0, &g_ADCHandle) )
    {
        am_util_stdio_printf("Error - reservation of the ADC instance failed.\n");
    }

    //
    // Power on the ADC.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_power_control(g_ADCHandle,
                                                          AM_HAL_SYSCTRL_WAKE,
                                                          false) )
    {
        am_util_stdio_printf("Error - ADC power on failed.\n");
    }

    //
    // Set up internal repeat trigger timer
    //
    am_hal_adc_irtt_config_t      ADCIrttConfig =
    {
        .bIrttEnable        = true,
        .eClkDiv            = AM_HAL_ADC_RPTT_CLK_DIV16,
        .ui32IrttCountMax   = 30,
    };

    am_hal_adc_configure_irtt(g_ADCHandle, &ADCIrttConfig);

    //
    // Set up the ADC configuration parameters. These settings are reasonable
    // for accurate measurements at a low sample rate.
    //
    ADCConfig.eClock             = AM_HAL_ADC_CLKSEL_HFRC_24MHZ;
    ADCConfig.ePolarity          = AM_HAL_ADC_TRIGPOL_RISING;
    ADCConfig.eTrigger           = AM_HAL_ADC_TRIGSEL_SOFTWARE;
    ADCConfig.eClockMode         = AM_HAL_ADC_CLKMODE_LOW_LATENCY;
    ADCConfig.ePowerMode         = AM_HAL_ADC_LPMODE0;
    ADCConfig.eRepeat            = AM_HAL_ADC_REPEATING_SCAN;
    ADCConfig.eRepeatTrigger     = AM_HAL_ADC_RPTTRIGSEL_INT;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure(g_ADCHandle, &ADCConfig))
    {
        am_util_stdio_printf("Error - configuring ADC failed.\n");
    }

    //
    // Set up an ADC slot
    //
    ADCSlotConfig.eMeasToAvg      = AM_HAL_ADC_SLOT_AVG_128;
    ADCSlotConfig.ui32TrkCyc      = AM_HAL_ADC_MIN_TRKCYC;
    ADCSlotConfig.ePrecisionMode  = AM_HAL_ADC_SLOT_12BIT;
    ADCSlotConfig.eChannel        = AM_HAL_ADC_SLOT_CHSEL_SE0;
    ADCSlotConfig.bWindowCompare  = false;
    ADCSlotConfig.bEnabled        = true;
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_configure_slot(g_ADCHandle, 0, &ADCSlotConfig))
    {
        am_util_stdio_printf("Error - configuring ADC Slot 0 failed.\n");
    }

    //
    // Configure the ADC to use DMA for the sample transfer.
    //
    adc_config_dma();

    //
    // For this example, the samples will be coming in slowly. This means we
    // can afford to wake up for every conversion.
    //
    am_hal_adc_interrupt_enable(g_ADCHandle, AM_HAL_ADC_INT_FIFOOVR1 | AM_HAL_ADC_INT_DERR | AM_HAL_ADC_INT_DCMP);

    //
    // Enable the ADC.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_enable(g_ADCHandle))
    {
        am_util_stdio_printf("Error - enabling ADC failed.\n");
    }

    //
    // Enable internal repeat trigger timer
    //
    am_hal_adc_irtt_enable(g_ADCHandle);
}

//*****************************************************************************
//
// Main function.
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
    // Start the ITM interface.
    //
    am_bsp_debug_printf_enable();

    //
    // Set a pin to act as our ADC input
    //
    am_hal_gpio_pinconfig(19, g_AM_PIN_19_ADCSE0);

    //
    // Configure the ADC
    //
    adc_config();

    //
    // Enable interrupts.
    //
    NVIC_SetPriority(ADC_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(ADC_IRQn);
    am_hal_interrupt_master_enable();

    //
    // Trigger the ADC sampling for the first time manually.
    //
    if (AM_HAL_STATUS_SUCCESS != am_hal_adc_sw_trigger(g_ADCHandle))
    {
        am_util_stdio_printf("Error - triggering the ADC failed.\n");
    }

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("ADC Example with 2.4Msps and LPMODE=0\n");

    //
    // Allow time for all printing to finish.
    //
    am_util_delay_ms(10);

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
    am_bsp_debug_printf_disable();

    //
    // Loop forever.
    //
    while(1)
    {
        //
        // Go to Deep Sleep.
        //
        if (!g_bADCDMAComplete)
        {
            sleep();
        }

        //
        // Check for DMA errors.
        //
        if (g_bADCDMAError)
        {
            am_util_stdio_printf("DMA Error occured\n");
            while(1);
        }

        //
        // Check if the ADC DMA completion interrupt occurred.
        //
        if (g_bADCDMAComplete)
        {
            {
                am_util_stdio_printf("DMA Complete\n");

                uint32_t ui32SampleCount;
                ui32SampleCount = ADC_SAMPLE_BUF_SIZE;

                //
                // If DMA buffer is given, invalidate it before read samples from it.
                //
                am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32ADCSampleBuffer, sizeof(uint32_t) * ADC_SAMPLE_BUF_SIZE}, false);

                if (AM_HAL_STATUS_SUCCESS != am_hal_adc_samples_read(g_ADCHandle, false,
                                                                     g_ui32ADCSampleBuffer,
                                                                     &ui32SampleCount,
                                                                     SampleBuffer))
                {
                    am_util_stdio_printf("Error - failed to process samples.\n");
                }
            }
            //
            // Reset the DMA completion and error flags.
            //
            g_bADCDMAComplete = false;

            //
            // Re-configure the ADC DMA.
            //
            adc_config_dma();

            //
            // Clear the ADC interrupts.
            //
            if (AM_HAL_STATUS_SUCCESS != am_hal_adc_interrupt_clear(g_ADCHandle, 0xFFFFFFFF))
            {
                am_util_stdio_printf("Error - clearing the ADC interrupts failed.\n");
            }
        }
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
