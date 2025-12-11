//*****************************************************************************
//
//! @file uart_stream_echo_rxDma.c
//!
//! @brief A uart example that demonstrates the stream driver using RX DMA
//!
//! @addtogroup peripheral_examples Peripheral Examples
//! @{
//!
//! @defgroup uart_stream_echo_rx_dma UART RX DMA Streaming Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates UART streaming driver
//! functionality with RX DMA (Direct Memory Access) for high-performance
//! serial communication. The application showcases UART reception
//! using DMA, interrupt-driven data handling, and real-time echo functionality.
//!
//! @section uart_stream_echo_rx_dma_features Key Features
//!
//! 1. @b RX @b DMA @b Streaming: Demonstrates high-performance UART reception
//!    using DMA for efficient data transfer and low CPU utilization
//!
//! 2. @b Interrupt @b Driven @b Operation: Implements interrupt-based data
//!    handling for real-time reception and transmission
//!
//! 3. @b Buffer @b Management: Provides buffer management with
//!    double buffering and circular queue support
//!
//! 4. @b Callback @b Based @b Reception: Implements callback-based reception
//!    for efficient and responsive data handling
//!
//! 5. @b Real @b Time @b Echo: Demonstrates real-time echo functionality for
//!    bidirectional UART communication
//!
//! @section uart_stream_echo_rx_dma_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes UART with RX DMA configuration and buffer management
//! - Implements interrupt-driven data reception and transmission
//! - Provides callback-based reception for efficient data handling
//! - Demonstrates real-time echo functionality for bidirectional communication
//! - Monitors UART status and provides debug output via SWO
//! - Supports interaction with PC using UART/USB cable and terminal application
//!
//! @section uart_stream_echo_rx_dma_usage Usage
//!
//! 1. Connect UART pins to PC using UART/USB cable (1.8V logic)
//! 2. Compile and download the application to target device
//! 3. Use provided Python script or serial terminal for testing
//! 4. Send data to device and observe real-time echo and status output
//! 5. Monitor SWO output for UART status and error information
//!
//! @section uart_stream_echo_rx_dma_configuration Configuration
//!
//! - @b UART_BAUDRATE: Configurable UART communication rate (default: 3,000,000 baud)
//! - @b RX_DMA_QUEUE_TYPE: DMA queue type (double buffering or circular queue)
//! - @b RX_CALLBACK_THRESHOLD: Threshold for callback-based reception
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via SWO
//!
//! This driver expects the uart fifos be enabled.
//!
//! Default Configuration:<br>
//! By default, this example uses UART2. The I/O pins used are defined in the BSP
//! file as AM_BSP_GPIO_UART2_TX and AM_BSP_GPIO_UART2_RX<br><br>
//!
//! Configuration and Operation:<br>
//! - These examples require enabling Tx and Rx fifos and queues.<br>
//! - It operates in a non-blocking manner using rx threshold callbacks and no tx callbacks.<br>
//!
//! To interact with these pins from a PC, the user should obtain a 1.8v uart/usb
//! cable (FTDI recommended especially for high-speed data).<br>
//!
//! The user can test with the provided python file ser_echo_test.py or can
//! test by manually typing a large block of characters via a uart terminal and
//! observing the echo
//!
//! The SWO output can send Rx/Tx status and error information.
//! Swo output can be enabled disabled using the booleans in streamErrorMessageMap[]
//! SWO Printing takes place over the ITM at 1MHz.

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

static const am_hal_uart_config_t g_sUartConfigEchoRxDMA =
{
    //
    //! Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate    = UART_BAUDRATE,
    //
    //! Set TX and RX FIFOs to interrupt at three-quarters full.
    //! Rx input will also use the timeout feature to trigger an RX ISR
    //
    .eDataBits    = AM_HAL_UART_DATA_BITS_8,
    .eParity      = AM_HAL_UART_PARITY_NONE,
#if UART_BAUDRATE >= 1500000
    .eStopBits    = AM_HAL_UART_TWO_STOP_BITS,
#else
    .eStopBits    = AM_HAL_UART_ONE_STOP_BIT,
#endif
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_4,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_24,
};

//
//! define RAM vars used locally in this example
//
typedef struct
{
    //
    //! Global holding uart streaming config and parameters
    //! this is for use in this example
    //
    uart_examp_stream_vars_t sStreamTestRxDma;
    uint32_t ui32DataRxed;
    uint32_t ui32maxDataBuffSize;
    //
    //! a copy of data sent in the rx dma callback
    //
    volatile am_hal_uart_stream_callback_data_t sLocalRxDmaCbData;

    //
    //! This is set in the rx dma callback, it is checked in cleared in the background loop.
    //
    volatile bool rxData;   //!<

    volatile bool bTimerIntOccurred;
}
test_local_vars_t;

static test_local_vars_t g_tstLocVar;

static void RxCompleteCallBack(am_hal_uart_stream_callback_data_t *ptCallbackData);
static uint32_t serial_interface_init(void);
static am_hal_uart_errors_t serial_interrupt_enable_rxDma(void);
static uint32_t processRxDmaComplete(uart_examp_stream_vars_t *psAsyncLVars);

//
//! Configure RxDma parameters
//
static const am_hal_uart_stream_rx_config_t g_rxConfig =
{
    .ui32NumBytesToRead =  RX_BUFF_THRESHOLD,
    .pfRxCallback       = RxCompleteCallBack,
    .ui32TimeoutMs      = 0,
    .bClearRxFifo       = false,
    .sRxBuffer = { .pui8Buff = NULL, .ui32BufferSize = MAX_TX_DATA_BUFFER_SIZE > DATABUFF_SIZE ? DATABUFF_SIZE : MAX_TX_DATA_BUFFER_SIZE  },
};

//
//! Configure uart Tx setup parameters
//
static const am_hal_uart_stream_tx_config_t g_txConfig =
{
    .pfTxCallback       = NULL,     //!< no tx callbacks used
    .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE, //!< from uart_stream_example_config.h
    .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
    .sTxBuffer = { .pui8Buff = NULL, .ui32BufferSize = TX_BUFFER_SRAM_USED  }, //!< null: force use of SRAM
};

//
//! @brief This is a helper table used to (efficiently) print statuses returned from the uart streaming isr call
//! @note arrange the most likely statuses first
//
static const stream_error_message_action_map_t streamErrorMessageMap[] =
{
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "uart_rx_dma_complete" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_BUSY           , "tx busy" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_BUSY, "rx_dma_busy" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_COMPLETE       , "tx_complete" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_ERROR, "rx_dma_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_TIMEOUT, "rx_dma_timeout" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_OVERFLOW, "rx_dma_overflow" },

    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_RX_QUEUE_FULL,   "rx_queue_full" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_RX_DATA_AVAIL     , "rx_data_avail" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_QUEUE_FULL     , "tx_queue_full" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_DMA_BUSY       , "tx_dma_busy" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_DMA_COMPLETE   , "tx_dma_complete" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_DMA_ERROR         , "dma_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_INTERNAL_DMA_ERROR, "intern_dma_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_FRM_ERROR, "framing_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_PRTY_ERROR, "parity_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_BRK_ERROR , "break_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_OVRN_ERROR, "overrun error" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = (am_hal_uart_stream_status_t)0, NULL },  // zero entry must be last
};

//******************************************************************************
//! @brief RxCompleteCallBack: rxDma Complete callback
//!
//! @note This is called from a UART ISR, when RX DMA is processed.
//!
//! @param ptCallbackData pointer to callback data
//!
//******************************************************************************
static void RxCompleteCallBack(am_hal_uart_stream_callback_data_t *ptCallbackData)
{
    //
    // copy struct
    //
    g_tstLocVar.sLocalRxDmaCbData  = *ptCallbackData;
    g_tstLocVar.rxData            = true;
}
//*****************************************************************************
//
//! @brief This is the init function called to enable interrupts for the rxDMA example
//! It calls am_hal_uart_stream_start_rxdma to get rxDMA running
//! It is included in the standard example function table
//!
//! @return am_hal_uart_errors_t status
//
//*****************************************************************************
static am_hal_uart_errors_t serial_interrupt_enable_rxDma(void)
{
    //
    //! start rx dma transfers
    //
    //am_hal_uart_errors_t rxStartStat =  am_hal_uart_stream_configure_rx( &g_tstLocVar.sStreamTestRxDma, &rxDmaConfig );
    bool bEnableRxDMA = true;
    bool bClearFifo = false;
    am_hal_uart_errors_t eStartState = am_hal_uart_stream_enable_rxDma(g_tstLocVar.sStreamTestRxDma.pUartHandle,
        bEnableRxDMA,
        bClearFifo);

    if (eStartState)
    {
        const char *pcErrorString = printErrorString( eStartState, false );
        am_util_stdio_printf("error: rx dma startup error: %s\r\n", pcErrorString );
    }

    return eStartState;
}
//*****************************************************************************
//
//! @brief will process returned ISR data, called in background loop
//!
//! @note This function is designed to run in the background, using data that was sent
//! and saved from the uart isr when a DMA buffer was filled, and the double buffer switch occurred
//!
//! @param psAsyncLVars
//! @return
//
//*****************************************************************************
static uint32_t processRxDmaComplete( uart_examp_stream_vars_t *psAsyncLVars)
{
    if (g_tstLocVar.rxData)
    {
        //
        // Echo data. Cannot use dma to TX (since DMA is used on RX)
        // The address and size were sent from the ISR via a callback
        // this function is not that callback
        //
        am_hal_uart_errors_t ui32TxStat = am_hal_uart_append_tx_fifo( psAsyncLVars->pUartHandle,
                                                                      g_tstLocVar.sLocalRxDmaCbData.pui8Buffer,
                                                                      g_tstLocVar.sLocalRxDmaCbData.ui32BufferSize);

        if ( ui32TxStat )
        {
            am_util_stdio_printf( "tx resend error %x %d\n", ui32TxStat, psAsyncLVars->uartId );
        }
        __DMB();
        *g_tstLocVar.sLocalRxDmaCbData.hasData = 0; // used the data, clear the flag
        g_tstLocVar.rxData = false;
    }

    return 0;
}

//******************************************************************************
//
// inits the rx dma echo
//
//*****************************************************************************
static uint32_t serial_interface_init( void )
{
    //
    // copy pointer to test buff and its size
    //

    //
    // configure the uart driver
    //
    uart_examp_stream_vars_t *psStreamTestRxDma = &g_tstLocVar.sStreamTestRxDma;

    //
    // Grab the pointer to the uart config table.
    // This is the uart config parameters like baud and stop bits.
    //
    psStreamTestRxDma->psUartConfig        = &g_sUartConfigEchoRxDMA;

    //
    // if this timer callback function isn't provided, the timer won't be used
    // This is so the timer can run in the background for poling activities (if wanted)
    //
    psStreamTestRxDma->pfTimerCallback     = NULL;

    //
    // set timer interrupt to 20msec
    // not used since callback is null
    //
    psStreamTestRxDma->sUartTimerParams.ui32TimerPeriodMs = TIMER_PERIOD_MS;


    psStreamTestRxDma->uart_hardware_instance = UART_ID;

    //
    // located dma buffers in SRAM, cache disabled for DMA (disabled via MPU)
    // this will do uart_init, after setting up buffers
    //


    return uart_cmn_serial_interface_init(psStreamTestRxDma,
        &g_rxConfig,
        &g_txConfig );

}

//******************************************************************************
//!
//! @brief Main
//!
//******************************************************************************
int
main(void)
{
    uint32_t ui32Status = mcu_setup();
    if ( ui32Status )
    {
        while(1);
    }

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Uart Streaming RxDma Echo Example!\n\n");

    //
    // Configure the SRAM memory containing DMA buffers as non-cacheable
    //
    ui32Status = uart_stream_cmn_mpuConfig();
    if ( ui32Status )
    {
        am_util_stdio_printf("MpuConfig error! %d\nExample will hang\n", ui32Status);
        while(1);
    }

    //
    // preform serial init
    //
    ui32Status =  serial_interface_init();
    if ( ui32Status )
    {
        am_util_stdio_printf("serial_interface_init error! %d\nExample will hang\n", ui32Status);
        while(1);
    }

    //
    // Enable interrupt service routines.
    //
    am_hal_interrupt_master_enable();

    //
    // At this point, the uart should be configured for rx dma.
    // The first test is to send a small string out the serial port
    //


    //
    // start rx dma
    //
    ui32Status = serial_interrupt_enable_rxDma();
    if ( ui32Status )
    {
        am_util_stdio_printf("error starting RX DMA %d\nExample will hang\n", ui32Status);
        while (true);
    }

    char testString[] = "starting rxdma test\r\n";
    am_hal_uart_errors_t ui32TxStat = am_hal_uart_append_tx_fifo( g_tstLocVar.sStreamTestRxDma.pUartHandle,
                                                                  (uint8_t *) testString, sizeof(testString)-1);
    if ( ui32TxStat )
    {
        const char *pstr  = printErrorString( ui32TxStat, false );
        am_util_stdio_printf("Serial Rx dma start error! %s : %d\nProgram will hang\n", pstr, ui32TxStat);
        while(1);
    }

    am_util_stdio_printf("echo_rxDma started\n");

    while (1)
    {
        //
        //! collect and processes returned uart isr status
        //
        am_hal_uart_stream_status_t isrStatus;
        AM_CRITICAL_BEGIN
            //
            // save and clear the accumulated returned isr status for this module
            //
            isrStatus = g_tstLocVar.sStreamTestRxDma.e32Status;
            g_tstLocVar.sStreamTestRxDma.e32Status = AM_HAL_UART_STREAM_STATUS_SUCCESS;
        AM_CRITICAL_END

        if (isrStatus)
        {
            //
            // print and otherwise process returned isr service status messages
            //
            processStreamError(streamErrorMessageMap,
                &g_tstLocVar.sStreamTestRxDma,
                isrStatus);
        }

        //
        //! Check rx dma complete callback
        //
        if ( g_tstLocVar.rxData )
        {
            processRxDmaComplete(&g_tstLocVar.sStreamTestRxDma);
        }

    } // while(1)
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
