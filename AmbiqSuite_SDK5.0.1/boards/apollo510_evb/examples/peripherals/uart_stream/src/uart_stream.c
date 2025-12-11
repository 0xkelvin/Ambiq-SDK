//*****************************************************************************
//
//! @file uart_stream.c
//!
//! @brief A uart example that demonstrates the stream driver.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup uart_stream UART Streaming Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates UART streaming driver
//! functionality with DMA and interrupt management capabilities.
//! The application showcases UART communication using FIFO,
//! interrupt-driven, and DMA-based transmission modes. The driver supports
//! multiple DMA queue types including double buffering and circular queues,
//! enabling efficient handling of arbitrarily large transmit buffers.
//!
//! @section uart_stream_features Key Features
//!
//! 1. @b Multiple @b Test @b Scenarios: Provides five different test scenarios
//!    for UART streaming validation and demonstration
//!
//! 2. @b DMA @b Based @b Transmission: Implements DMA-based UART transmission
//!    for high-performance data transfer with low CPU overhead
//!
//! 3. @b Interrupt @b Driven @b Operation: Implements interrupt-based data
//!    handling for real-time transmission and reception
//!
//! 4. @b Buffer @b Management: Provides buffer management with
//!    double buffering and circular queue support
//!
//! 5. @b Non @b Blocking @b Operation: Implements non-blocking UART operations
//!    for efficient system resource utilization
//!
//! @section uart_stream_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes UART with streaming configuration
//! - Implements multiple test scenarios for validation
//! - Provides DMA-based transmission with buffer management
//! - Implements interrupt-driven data handling
//! - Demonstrates bidirectional communication capabilities
//! - Monitors UART status and provides debug output via SWO
//!
//! @section uart_stream_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Connect UART pins to PC using UART/USB cable (1.8V logic)
//! 3. Use provided Python scripts for testing
//! 4. Monitor SWO output for UART status and error information
//! 5. Test different scenarios using the various test modes
//!
//! @section uart_stream_configuration Configuration
//!
//! - @b UART_BAUDRATE: Configurable UART communication rate (default: 115200)
//! - @b DMA_QUEUE_TYPE: DMA queue type (double buffering or circular queue)
//! - @b TX_BUFFER_SIZE: Configurable transmit buffer size
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via SWO
//!
//! This example consists of five different test scenarios:
//! - uart_stream_test1: Simple alternating data buffer transmission
//! - uart_stream_test2: Transmit buffer overflow testing
//! - uart_stream_test3: Large DMA buffer transmit overflow testing
//! - uart_stream_test4: Maximum transmit output with full buffer
//! - uart_echo_test: Bidirectional communication with Python script
//! - uart_echo_rx_dma_test: Rx DMA with echo via Tx FIFO transfers
//!
//! Default Configuration:<br>
//! By default, this example uses UART2. The I/O pins used are defined in the BSP
//! file as AM_BSP_GPIO_UART2_TX and AM_BSP_GPIO_UART2_RX<br><br>
//!
//! The SWO output will send Rx/Tx status and error information.
//! SWO Printing takes place over the ITM at 1MHz.
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
#include "am_util.h"
#include "uart_stream_common.h"
#include "uart_stream_test1.h"
#include "uart_stream_test2.h"
#include "uart_stream_test3.h"
#include "uart_stream_test4.h"
#include "uart_echo_test.h"
#include "uart_echo_rx_dma_test.h"
#include <string.h>

#define NUM_UARTS_USED 1

//
// define global ram used in this example
//
typedef struct
{
    bool bTimerIntOccurred;
}
uart_example_ramGlobals_t;

//
//! allocate global RAM
//
uart_example_ramGlobals_t g_localv;

//
//! allocate tx buffer RAM
//! this will be sent to the various examples/tests to minimize ram use
//
uint8_t g_txDataBuffer[MAX_TX_DATA_BUFFER_SIZE];

//***************************** local prototypes *******************************

static void uartPeriodicTimerCallback( uint32_t x);
//***************************** code below *************************************

//*****************************************************************************
//
//! @brief Called from timer ISR
//!
//! @param x unused
//
//*****************************************************************************
static void uartPeriodicTimerCallback( uint32_t x)
{
    (void) x;

    g_localv.bTimerIntOccurred = true;  // inform background loop
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Status = mcu_setup();
    if (ui32Status)
    {
        while(1);
    }

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Uart Streaming Example!\n\n");

    ui32Status = uart_stream_cmn_mpuConfig();
    if ( ui32Status )
    {
        am_util_stdio_printf("MpuConfig error! %d\nExample will hang\n", ui32Status);

        while(1);
    }

    //
    // Choose one example/test here
    //
    //const uart_cmn_fcn_t *sTestFcnMap = &g_sUartFcnEchoTest;
    const uart_cmn_fcn_t *sTestFcnMap = &g_sUartFcnArrayTest1;
    //const uart_cmn_fcn_t *sTestFcnMap = &g_sUartFcnArrayTest2;
    //const uart_cmn_fcn_t *sTestFcnMap = &g_sUartFcnArrayTest3;
    //const uart_cmn_fcn_t *sTestFcnMap = &g_sUartFcnArrayTest4;
    //const uart_cmn_fcn_t *sTestFcnMap = &g_sUartFcnEchoRxDMATest;

    uart_examp_stream_vars_t *psStreamLocalVars = sTestFcnMap->psStreamLocalVars;

    psStreamLocalVars->pfTimerCallback = uartPeriodicTimerCallback; // null to disable timer, needs to be set before init call
    psStreamLocalVars->stestBuffInfo.pui8Testbuff = g_txDataBuffer;
    psStreamLocalVars->stestBuffInfo.ui32TestBuffMax = MAX_TX_DATA_BUFFER_SIZE;

    //
    // Choose transfer method (uart tx/rx mode)
    // This has to be appropriate for the test chosen above
    //
    //AM_HAL_UART_DMA_NONE,
    //AM_HAL_UART_DMA_TX_DOUBLE_BUFFER,
    //AM_HAL_UART_DMA_TX_SINGLE_BUFFER,
    //AM_HAL_UART_DMA_RX_SINGLE,
    //AM_HAL_UART_DMA_RX_DOUBLE,
    //

    //psStreamLocalVars->euartEaxampleStreamMode = AM_HAL_UART_DMA_NONE;
    //psStreamLocalVars->euartEaxampleStreamMode = AM_HAL_UART_DMA_TX_DOUBLE_BUFFER;
    psStreamLocalVars->euartEaxampleStreamMode = AM_HAL_UART_DMA_TX_SINGLE_BUFFER;

    //
    // call example/test setup
    //
    ui32Status = sTestFcnMap->pfSerialIfInit(psStreamLocalVars);
    if ( ui32Status )
    {
        am_util_stdio_printf("pfSerialIfInit error! %d\nExample will hang\n", ui32Status);

        while(1);
    }

    //
    // Enable all ISRs
    //
    am_hal_interrupt_master_enable();

    uint32_t tx_stat = 0;
    uint32_t rx_stat = 0;
    am_util_stdio_printf("%s started, tx_start_stat: %d, rx_start_stat %d\n", sTestFcnMap->testName, tx_stat, rx_stat);

    //
    // enable uart interrupts (final setup call)
    //
    sTestFcnMap->pfSerInterruptEnable(psStreamLocalVars);

    while (1)
    {
        // loop through all the uart descriptors (there is only one in the base example)

        // check through the uart status
        // read data
        // look at input status

        am_hal_uart_stream_status_t isrStatus;
        AM_CRITICAL_BEGIN
            //
            // clear off the accumulated status for this module
            //
            isrStatus = psStreamLocalVars->e32Status;
            psStreamLocalVars->e32Status = AM_HAL_UART_STREAM_STATUS_SUCCESS;
        AM_CRITICAL_END

        if (isrStatus)
        {
            sTestFcnMap->pfSerProcessIsrStatus(psStreamLocalVars, isrStatus);
        }

        if (g_localv.bTimerIntOccurred)
        {
            //
            // timer interrupt, this is used to send data periodically
            //
            g_localv.bTimerIntOccurred = false;

            sTestFcnMap->pfSerProcessTimerInterrupt(psStreamLocalVars);
        } // g_localv.bTimerIntOccurred
    } // while(1)
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
