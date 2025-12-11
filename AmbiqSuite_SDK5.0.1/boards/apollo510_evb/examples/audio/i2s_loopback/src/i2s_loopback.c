//*****************************************************************************
//
//! @file i2s_loopback.c
//!
//! @brief An example to show basic I2S operation.
//!
//! @addtogroup audio_examples Audio Examples
//!
//! @defgroup i2s_loopback I2S Loopback Example
//! @ingroup audio_examples
//! @{
//!
//! Purpose: This example demonstrates I2S loopback functionality for audio
//! signal integrity testing and validation. The application showcases I2S
//! interface configuration with dual-channel support, audio processing,
//! and buffer management.
//!
//! @section i2s_loopback_features Key Features
//!
//! 1. @b I2S @b Loopback: Enables I2S interfaces to loop back data between
//!    TX and RX for signal integrity testing and validation
//!
//! 2. @b Dual @b Interface @b Support: Supports I2S0 and I2S1 with configurable
//!    controller/device roles for flexible audio routing
//!
//! 3. @b Data @b Verification: Compares transmitted and received data to ensure
//!    accurate loopback operation and signal integrity
//!
//! 4. @b Audio @b Sampling: Supports 48 kHz sample rate, 32-bit word width,
//!    and 24-bit bit-depth
//!
//! 5. @b Ping @b Pong @b Buffering: Implements software-managed ping-pong buffer
//!    mechanism for continuous data streaming without interruption
//!
//! @section i2s_loopback_functionality Functionality
//!
//! The application performs the following operations:
//! - Configures I2S interfaces with audio parameters
//! - Implements loopback between I2S TX and RX interfaces
//! - Provides data verification and integrity checking
//! - Implements ping-pong buffering for continuous streaming
//! - Supports dual I2S interface configuration
//! - Monitors and reports I2S operation status via SWO
//!
//! @section i2s_loopback_usage Usage
//!
//! 1. Connect I2S pins according to the specified pin configuration
//! 2. Compile and download the application to target device
//! 3. Monitor SWO output for I2S operation status and results
//! 4. Verify loopback data integrity and signal quality
//! 5. Test continuous operation and buffer management
//!
//! @section i2s_loopback_configuration Configuration
//!
//! - @b Sample @b Rate: 48 kHz audio sampling rate
//! - @b Audio @b Format: 32-bit word width, 24-bit bit-depth, 2 channels
//! - @b I2S @b Format: Standard I2S format
//! - @b Buffer @b Management: Ping-pong buffering for continuous streaming
//!
//! @verbatim
//! I2S configurations:
//!  - 2 channels
//!  - 48 kHz sample rate
//!  - Standard I2S format
//!  - 32 bits word width
//!  - 24 bits bit-depth
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
//! Printing takes place over the SWO at 1MHz.
//! @endverbatim
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
#define CLOCK_SOURCE                (PLL)
    #define HFRC                    (0)
    #define PLL                     (1)
    #define NCO                     (2)
    #define HF2ADJ                  (3)
#define I2S_MODULE_0                (0)
#define I2S_MODULE_1                (1)
#define NUM_OF_SAMPLES              (256)
#define USE_I2S_CONTROLLER          I2S_MODULE_0
#define USE_SLEEP_MODE              (USE_SLEEP_MODE_DEEP)
    #define USE_SLEEP_MODE_NONE     (0)
    #define USE_SLEEP_MODE_NORMAL   (1)
    #define USE_SLEEP_MODE_DEEP     (2)
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
volatile uint32_t g_ui32I2SDmaCpl[5] =
{
    0, //I2S0 TX.
    0, //I2S0 RX.
    #if NUM_OF_I2S_INSTANCES > 1
    0, //I2S1 Tx.
    0, //I2S1 Rx.
    #endif
    0  //Success or Fail.
};

//
// Used as the ping-pong buffer of DMA.
// Aligned to 32 bytes to meet data cache requirements.
//
AM_SHARED_RW uint32_t g_ui32I2S0RxDataBuffer[2 * NUM_OF_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2S0TxDataBuffer[2 * NUM_OF_SAMPLES] __attribute__((aligned(32)));
#if NUM_OF_I2S_INSTANCES > 1
AM_SHARED_RW uint32_t g_ui32I2S1RxDataBuffer[2 * NUM_OF_SAMPLES] __attribute__((aligned(32)));
AM_SHARED_RW uint32_t g_ui32I2S1TxDataBuffer[2 * NUM_OF_SAMPLES] __attribute__((aligned(32)));
#endif

//*****************************************************************************
//
// I2S configurations:
//  - 2 channels
//  - Standard I2S format
//  - 32 bits word width
//  - 24 bits bit-depth
//  - sample rate:
//    * HFRC   46875Hz
//    * PLL    48000Hz
//    * NCO    48000Hz
//    * HF2ADJ 48000Hz (Apollo510 only)
//
//*****************************************************************************
static am_hal_i2s_io_signal_t g_sI2SIOConfig =
{
    .sFsyncPulseCfg =
    {
        .eFsyncPulseType = AM_HAL_I2S_FSYNC_PULSE_ONE_SUBFRAME,
    },
    .eFyncCpol = AM_HAL_I2S_IO_FSYNC_CPOL_LOW,
    .eTxCpol   = AM_HAL_I2S_IO_TX_CPOL_FALLING,
    .eRxCpol   = AM_HAL_I2S_IO_RX_CPOL_RISING
};

static am_hal_i2s_data_format_t g_sI2SDataConfig =
{
    .ePhase                   = AM_HAL_I2S_DATA_PHASE_SINGLE,
    .eChannelLenPhase1        = AM_HAL_I2S_FRAME_WDLEN_32BITS,
    .ui32ChannelNumbersPhase1 = 2,
    .eDataDelay               = 0x1,
    .eSampleLenPhase1         = AM_HAL_I2S_SAMPLE_LENGTH_24BITS,
    .eDataJust                = AM_HAL_I2S_DATA_JUSTIFIED_LEFT,
};

static am_hal_i2s_config_t g_sI2S0Config =
{
    #if (USE_I2S_CONTROLLER == I2S_MODULE_0)
    .eMode  = AM_HAL_I2S_IO_MODE_MASTER,
    #else
    .eMode  = AM_HAL_I2S_IO_MODE_SLAVE,
    #endif
    .eXfer  = AM_HAL_I2S_XFER_RXTX,
    #if defined (AM_PART_APOLLO510)
    #if (CLOCK_SOURCE == HFRC)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC_3MHz,
    .eDiv3  = 0,
    #elif (CLOCK_SOURCE == PLL)
    .eClock = eAM_HAL_I2S_CLKSEL_PLL_FOUT4,
    .eDiv3  = 0,
    #elif (CLOCK_SOURCE == NCO)
    .eClock = eAM_HAL_I2S_CLKSEL_NCO_HFRC_48MHz,
    .f64NcoDiv = 48.0 / 3.072,
    #elif (CLOCK_SOURCE == HF2ADJ)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC2_APPROX_4MHz,
    .eDiv3  = 0,
    #endif
    #elif defined (AM_PART_APOLLO330P_510L)
    #if (CLOCK_SOURCE == HFRC)
    .eClock = AM_HAL_I2S_CLKSEL_HFRC_48MHz,
    .ui32ClockDivideRatio = 16,
    #elif (CLOCK_SOURCE == PLL)
    .eClock = AM_HAL_I2S_CLKSEL_PLL_FOUT4,
    .ui32ClockDivideRatio = 1,
    #elif (CLOCK_SOURCE == NCO)
    .eClock = AM_HAL_I2S_CLKSEL_NCO_HFRC_96MHz,
    .ui32ClockDivideRatio = 1,
    .f64NcoDiv = 96.0 / 3.072,
    #else
    #error "Invalid I2S clock source for Apollo510L."
    #endif
    .eMclkout = AM_HAL_I2S_MCLKOUT_SEL_OFF,
    .ui32MclkoutDiv = 1,
    #endif
    .eASRC  = 0,
    .eData  = &g_sI2SDataConfig,
    .eIO    = &g_sI2SIOConfig,
};

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
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC_3MHz,
    .eDiv3  = 0,
    #elif (CLOCK_SOURCE == PLL)
    .eClock = eAM_HAL_I2S_CLKSEL_PLL_FOUT4,
    .eDiv3  = 0,
    #elif (CLOCK_SOURCE == NCO)
    .eClock    = eAM_HAL_I2S_CLKSEL_NCO_HFRC_48MHz,
    .f64NcoDiv = 48.0 / 3.072,
    #elif (CLOCK_SOURCE == HF2ADJ)
    .eClock = eAM_HAL_I2S_CLKSEL_HFRC2_APPROX_4MHz,
    .eDiv3  = 0,
    #endif
    .eASRC  = 0,
    .eData  = &g_sI2SDataConfig,
    .eIO    = &g_sI2SIOConfig,
};
#endif

//
// Ping pong buffer settings.
//
static am_hal_i2s_transfer_t sTransfer0 =
{
    .ui32RxTotalCount        = NUM_OF_SAMPLES,
    .ui32RxTargetAddr        = (uint32_t)(&g_ui32I2S0RxDataBuffer[0]),
    .ui32RxTargetAddrReverse = (uint32_t)(&g_ui32I2S0RxDataBuffer[NUM_OF_SAMPLES]),
    .ui32TxTotalCount        = NUM_OF_SAMPLES,
    .ui32TxTargetAddr        = (uint32_t)(&g_ui32I2S0TxDataBuffer[0]),
    .ui32TxTargetAddrReverse = (uint32_t)(&g_ui32I2S0TxDataBuffer[NUM_OF_SAMPLES]),
};

#if NUM_OF_I2S_INSTANCES > 1
static am_hal_i2s_transfer_t sTransfer1 =
{
    .ui32RxTotalCount        = NUM_OF_SAMPLES,
    .ui32RxTargetAddr        = (uint32_t)(&g_ui32I2S1RxDataBuffer[0]),
    .ui32RxTargetAddrReverse = (uint32_t)(&g_ui32I2S1RxDataBuffer[NUM_OF_SAMPLES]),
    .ui32TxTotalCount        = NUM_OF_SAMPLES,
    .ui32TxTargetAddr        = (uint32_t)(&g_ui32I2S1TxDataBuffer[0]),
    .ui32TxTargetAddrReverse = (uint32_t)(&g_ui32I2S1TxDataBuffer[NUM_OF_SAMPLES]),
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
check_i2s_data(uint32_t rxtx_size, uint32_t* rx_databuf, uint32_t* tx_databuf)
{
    int i, index_0 = 0;

    //
    // Find the first element of Tx buffer in Rx buffer, and return the index.
    // Rx will delay N samples in full duplex mode.
    //
    for (i = 0; i < rxtx_size; i++)
    {
        if (rx_databuf[i] == tx_databuf[0])
        {
            index_0 = i;
            break;
        }
    }

    for (i = 0; i < (rxtx_size-index_0); i++)
    {
        if ( rx_databuf[i + index_0] != tx_databuf[i] )
        {
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
        // Config PLLPOSTDIV to output 24.576 MHz, hence PLLOUT4 = 3.072 MHz.
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
        am_util_stdio_printf("ERROR: Invalid I2S0 configuration.\nNote: For Apollo510 Rev.B0, I2S can only use PLL as the clock source.\n");
    }

    #if NUM_OF_I2S_INSTANCES > 1
    if (AM_HAL_STATUS_SUCCESS != am_hal_i2s_configure(pI2S1Handle, &g_sI2S1Config))
    {
        am_util_stdio_printf("ERROR: Invalid I2S1 configuration.\nNote: For Apollo510 Rev.B0, I2S can only use PLL as the clock source.\n");
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
// Helper function to enter sleep
//
//*****************************************************************************
void helper_sleep(void)
{
    #if (USE_SLEEP_MODE > USE_SLEEP_MODE_NONE)

    // Disable Interrupt and check whether all expected interrupt has been
    // received. Only enter sleep if we are still expecting interrupt. This is
    // to prevent entering sleep after all expected interrupt has been received.
    uint32_t ui32Critical = am_hal_interrupt_master_disable();

    // Check for all expected interrupt
    #if NUM_OF_I2S_INSTANCES == 2
    if (g_ui32I2SDmaCpl[3] && g_ui32I2SDmaCpl[2] && g_ui32I2SDmaCpl[1] && g_ui32I2SDmaCpl[0])
    {
        // Recover master interrupt
        am_hal_interrupt_master_set(ui32Critical);

        // All expected interrupt is received. Return without sleep
        return;
    }
    #elif NUM_OF_I2S_INSTANCES == 1
    if (g_ui32I2SDmaCpl[1] && g_ui32I2SDmaCpl[0])
    {
        // Recover master interrupt
        am_hal_interrupt_master_set(ui32Critical);

        // All expected interrupt is received. Return without sleep
        return;
    }
    #endif

    #if (USE_SLEEP_MODE == USE_SLEEP_MODE_DEEP)
    //Wait for ITM to be idle and turn it off
    while (!am_hal_itm_print_not_busy() || !am_hal_itm_not_busy());
    am_bsp_itm_printf_disable();

    //Power Off ROM and OTP so that MCU can enter Deep Sleep
    am_hal_pwrctrl_rom_disable();
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);

    // Enter deep sleep
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

    // MCU Wakeup: re-enable itm printf
    am_bsp_itm_printf_enable();
    #else
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
    #endif

    // Recover master interrupt
    am_hal_interrupt_master_set(ui32Critical);

    #endif //(USE_SLEEP_MODE > USE_SLEEP_MODE_NONE)
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

    #if defined(AM_PART_APOLLO330P_510L)
    am_util_stdio_printf("I2S Self-loopback Test: Controller = I2S0.\n\n");
    #else
    #if (USE_I2S_CONTROLLER == I2S_MODULE_0)
    am_util_stdio_printf("I2S Full Duplex Loopback Test: Controller = I2S0, Device = I2S1.\n\n");
    #else
    am_util_stdio_printf("I2S Full Duplex Loopback Test: Controller = I2S1, Device = I2S0.\n\n");
    #endif
    #endif


    //
    // Initialize data.
    //
    for (int i = 0; i < NUM_OF_SAMPLES; i++)
    {
        g_ui32I2S0TxDataBuffer[i]                  = (i & 0xFF) | 0xF50000;
        g_ui32I2S0TxDataBuffer[i + NUM_OF_SAMPLES] = (i & 0xFF) | 0x5F0000;
        #if NUM_OF_I2S_INSTANCES > 1
        g_ui32I2S1TxDataBuffer[i]                  = (i & 0xFF) | 0xF50000;
        g_ui32I2S1TxDataBuffer[i + NUM_OF_SAMPLES] = (i & 0xFF) | 0x5F0000;
        #endif
    }

    //
    // Write the contents in the data cache to the external memory.
    //
    am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32I2S0TxDataBuffer, sizeof(g_ui32I2S0TxDataBuffer)});
    #if NUM_OF_I2S_INSTANCES > 1
    am_hal_cachectrl_dcache_clean(&(am_hal_cachectrl_range_t){(uint32_t)g_ui32I2S1TxDataBuffer, sizeof(g_ui32I2S1TxDataBuffer)});
    #endif

    i2s_init();
    am_hal_i2s_dma_configure(pI2S0Handle, &g_sI2S0Config, &sTransfer0);
    #if NUM_OF_I2S_INSTANCES > 1
    am_hal_i2s_dma_configure(pI2S1Handle, &g_sI2S1Config, &sTransfer1);
    #endif

    NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE_0]);
    #if NUM_OF_I2S_INSTANCES > 1
    NVIC_EnableIRQ(i2s_interrupts[I2S_MODULE_1]);
    #endif
    am_hal_interrupt_master_enable();

    //
    // Start DMA transaction.
    //
    #if NUM_OF_I2S_INSTANCES == 2
    #if (USE_I2S_CONTROLLER == I2S_MODULE_0)
    am_hal_i2s_dma_transfer_start(pI2S1Handle, &g_sI2S1Config);
    am_hal_i2s_dma_transfer_start(pI2S0Handle, &g_sI2S0Config);
    #else
    am_hal_i2s_dma_transfer_start(pI2S0Handle, &g_sI2S0Config);
    am_hal_i2s_dma_transfer_start(pI2S1Handle, &g_sI2S1Config);
    #endif
    #elif NUM_OF_I2S_INSTANCES == 1
    am_hal_i2s_dma_transfer_start(pI2S0Handle, &g_sI2S0Config);
    #endif

    //
    // I2S DMACPL interrupts timeout value.
    //
    uint32_t ui32Timeout0P1Ms = DMACPL_TIMEOUT_MS / (0.1);

    //
    // Loop forever
    //
    while (1)
    {
        #if NUM_OF_I2S_INSTANCES == 2
        if (g_ui32I2SDmaCpl[0] && g_ui32I2SDmaCpl[1] && g_ui32I2SDmaCpl[2] && g_ui32I2SDmaCpl[3])
        {
            g_ui32TestLoop++;
            g_ui32I2SDmaCpl[0] = g_ui32I2SDmaCpl[1] = g_ui32I2SDmaCpl[2] = g_ui32I2SDmaCpl[3] = 0;

            uint32_t i2s0_rx_buffer = am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_RX);
            uint32_t i2s1_rx_buffer = am_hal_i2s_dma_get_buffer(pI2S1Handle, AM_HAL_I2S_XFER_RX);
            uint32_t i2s0_tx_buffer = am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_TX);
            uint32_t i2s1_tx_buffer = am_hal_i2s_dma_get_buffer(pI2S1Handle, AM_HAL_I2S_XFER_TX);

            //
            // Ensure that what the CPU sees is consistent with the contents of external memory.
            //
            am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){i2s0_rx_buffer, sizeof(uint32_t) * NUM_OF_SAMPLES}, false);
            am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){i2s1_rx_buffer, sizeof(uint32_t) * NUM_OF_SAMPLES}, false);

            if (check_i2s_data(NUM_OF_SAMPLES, (uint32_t*)i2s1_rx_buffer, (uint32_t*)i2s0_tx_buffer) && \
                check_i2s_data(NUM_OF_SAMPLES, (uint32_t*)i2s0_rx_buffer, (uint32_t*)i2s1_tx_buffer))
            {
                g_ui32I2SDmaCpl[4] = 1;
                am_util_stdio_printf("I2S Loopback Iteration %d PASSED!\n", g_ui32TestLoop);
            }
            else
            {
                am_util_stdio_printf("I2S Loopback Iteration %d FAILED!\n", g_ui32TestLoop);
            }

            ui32Timeout0P1Ms = DMACPL_TIMEOUT_MS / (0.1);
        }
        #elif NUM_OF_I2S_INSTANCES == 1
        if (g_ui32I2SDmaCpl[0] && g_ui32I2SDmaCpl[1])
        {
            g_ui32TestLoop++;
            g_ui32I2SDmaCpl[0] = g_ui32I2SDmaCpl[1] = 0;

            uint32_t i2s0_rx_buffer = am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_RX);
            uint32_t i2s0_tx_buffer = am_hal_i2s_dma_get_buffer(pI2S0Handle, AM_HAL_I2S_XFER_TX);

            //
            // Ensure that what the CPU sees is consistent with the contents of external memory.
            //
            am_hal_cachectrl_dcache_invalidate(&(am_hal_cachectrl_range_t){i2s0_rx_buffer, sizeof(uint32_t) * NUM_OF_SAMPLES}, false);

            if (check_i2s_data(NUM_OF_SAMPLES, (uint32_t*)i2s0_rx_buffer, (uint32_t*)i2s0_tx_buffer))
            {
                g_ui32I2SDmaCpl[2] = 1;
                am_util_stdio_printf("I2S Loopback Iteration %d PASSED!\n", g_ui32TestLoop);
            }
            else
            {
                am_util_stdio_printf("I2S Loopback Iteration %d FAILED!\n", g_ui32TestLoop);
            }

            ui32Timeout0P1Ms = DMACPL_TIMEOUT_MS / (0.1);
        }
        #endif

        am_util_delay_us(100);
        ui32Timeout0P1Ms--;
        if (ui32Timeout0P1Ms == 0)
        {
            //
            // Disable interrupt.
            //
            NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE_0]);
            #if NUM_OF_I2S_INSTANCES > 1
            NVIC_DisableIRQ(i2s_interrupts[I2S_MODULE_1]);
            #endif
            am_hal_interrupt_master_disable();

            //
            // Disable DMA and deinitialize I2S.
            //
            am_hal_i2s_dma_transfer_complete(pI2S0Handle);
            #if NUM_OF_I2S_INSTANCES > 1
            am_hal_i2s_dma_transfer_complete(pI2S1Handle);
            #endif
            i2s_deinit();

            am_util_stdio_printf("---ERROR--- I2S Loopback Timeout.\n");
            return 0;
        }

        // Enter Sleep Mode selected
        helper_sleep();
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
