//*****************************************************************************
//
//! @file uart_stream_echo_tx_Dma.c
//!
//! @brief A uart example that demonstrates the stream driver using TX DMA
//!
//! @addtogroup peripheral_examples Peripheral Examples
//! @{
//!
//! @defgroup uart_stream_echo_tx_dma UART TX DMA Streaming Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates UART streaming driver
//! functionality with TX DMA (Direct Memory Access) for high-performance
//! serial communication. The application showcases UART transmission
//! using DMA, interrupt-driven data handling, and real-time echo functionality.
//!
//! @section uart_stream_echo_tx_dma_features Key Features
//!
//! 1. @b TX @b DMA @b Streaming: Demonstrates high-performance UART transmission
//!    using DMA for efficient data transfer and low CPU utilization
//!
//! 2. @b Interrupt @b Driven @b Operation: Implements interrupt-based data
//!    handling for real-time transmission and reception
//!
//! 3. @b Buffer @b Management: Provides buffer management with
//!    double buffering and circular queue support
//!
//! 4. @b Callback @b Based @b Reception: Implements callback-based reception
//!    for efficient and responsive data handling
//!
//! 5. @b Real @b Time @b Echo: Demonstrates real-time echo functionality for
//!    bidirectional UART communication and testing
//!
//! @section uart_stream_echo_tx_dma_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes UART with TX DMA configuration and buffer management
//! - Implements interrupt-driven data transmission and reception
//! - Provides callback-based reception for efficient data handling
//! - Demonstrates real-time echo functionality for bidirectional communication
//! - Monitors UART status and provides debug output via SWO
//! - Supports interaction with PC using UART/USB cable and terminal application
//!
//! @section uart_stream_echo_tx_dma_usage Usage
//!
//! 1. Connect UART pins to PC using UART/USB cable (1.8V logic)
//! 2. Compile and download the application to target device
//! 3. Use provided Python script or serial terminal for testing
//! 4. Send data to device and observe real-time echo and status output
//! 5. Monitor SWO output for UART status and error information
//!
//! @section uart_stream_echo_tx_dma_configuration Configuration
//!
//! - @b UART_BAUDRATE: Configurable UART communication rate (default: 3,000,000 baud)
//! - @b TX_DMA_QUEUE_TYPE: DMA queue type (double buffering or circular queue)
//! - @b RX_CALLBACK_THRESHOLD: Threshold for callback-based reception
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via SWO
//!
//! This driver expects the uart FIFOs be enabled.
//!
//! Similarly, the interrupt code will move received data into the Rx queue
//! and the application periodically reads from the Rx queue.<br><br>
//!
//! The Rx callback has been enabled here.
//! Once the number of bytes in the Rx buffer
//! exceeds the threshold, the callback is made (from the ISR). Optionally, the callback
//! can be disabled and the rx buffer read via polling.<br><br>
//! This example will immediately retransmit the received data via TX DMA.
//!
//! The associated ISR handler am_hal_uart_interrupt_queue_service() will return
//! status in a bitfield, suitable for use as a callback or polling.<br><br>
//!
//! Default Configuration:<br>
//! By default, this example uses UART2. The I/O pins used are defined in the BSP
//! file as AM_BSP_GPIO_UART2_TX and AM_BSP_GPIO_UART2_RX<br><br>
//!
//! Configuration and Operation:<br>
//! - These examples require enabling Tx and Rx fifos and queues.<br>
//!
//! To interact with these pins from a PC, the user should obtain a 1.8v uart/usb
//! cable (FTDI, etc.).<br>
//!
//! to test use the provided python program ser_echo_test.py
//! or test with a serial terminal by manually typing a block of data 64 to 256 bytes
//! and observe the echo once the internal RX callback size has been reached.
//! The swo output will report status during example operation.<<br><br>
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

//
// define global ram used in this example
//
typedef struct
{
    volatile bool bTimerIntOccurred;
    volatile bool bDataAvailable;
}
uart_example_ramGlobals_t;

//
//! allocate global RAM
//
uart_example_ramGlobals_t g_localv;

static const am_hal_uart_config_t g_sUartConfigEchoTxDMA =
{
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

#if USE_RX_BUFFER_DTCM
    uint8_t ucRxBuffer[RX_BUFFER_DTCM_SIZE];
#endif
    uart_examp_stream_vars_t sStreamTestTxDma;      //!< variable for passing parameters to common init
    uint32_t ui32DataRxed;
    uint32_t ui32maxDataBuffSize;
    uint32_t ui32TestDbuffSize;

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
static uint32_t echoData( test_local_vars_t *psAsyncLVars );
static void RxCompleteCallBack(am_hal_uart_stream_callback_data_t *ptCallbackData);
static uint32_t serial_interface_init_txEcho(void);

//
//! Configure uart Rx setup parameters
//
static const am_hal_uart_stream_rx_config_t g_rxConfig =
{
    .ui32NumBytesToRead =  RX_BUFF_THRESHOLD,
    .pfRxCallback       = RxCompleteCallBack,
    .ui32RxCallbackThreshold =  RX_BUFF_THRESHOLD,
    .ui32TimeoutMs      = 0,
    .bClearRxFifo       = false,
#if USE_RX_BUFFER_DTCM
    .sRxBuffer = { .pui8Buff = g_tstLocVar.ucRxBuffer, .ui32BufferSize = RX_BUFFER_DTCM_SIZE  }, //!< use DTCM buffer
#else
    .sRxBuffer = { .pui8Buff = NULL, .ui32BufferSize = RX_BUFFER_DTCM_SIZE  }, //!< force use of SRAM
#endif
};

//
//! Configure uart Tx setup parameters
//
static const am_hal_uart_stream_tx_config_t g_txConfig =
{
    .pfTxCallback       = NULL,
    .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE, //!< from uart_stream_example_config.h
    .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
    .sTxBuffer = { .pui8Buff = NULL, .ui32BufferSize = TX_BUFFER_SRAM_USED  }, //!< force use of SRAM
};

//
//! @brief This is a helper table used to (efficiently) print statuses returned from the uart streaming isr call
//! @note arrange the most likely statuses first
//
static const stream_error_message_action_map_t streamErrorMessageMap[] =
{
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "uart_rx_dma_complete" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_BUSY           , "tx busy" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_DMA_BUSY       , "tx_dma_busy" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_RX_DATA_AVAIL     , "rx_data_avail" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_COMPLETE       , "tx_complete" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_ERROR, "rx_dma_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_TIMEOUT, "rx_dma_timeout" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_OVERFLOW, "rx_dma_overflow" },

    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_RX_QUEUE_FULL,   "rx_queue_full" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_QUEUE_FULL     , "tx_queue_full" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_DMA_COMPLETE   , "tx_dma_complete" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_DMA_ERROR         , "dma_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_INTERNAL_DMA_ERROR, "intern_dma_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_FRM_ERROR, "framing_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_PRTY_ERROR, "parity_error" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_BRK_ERROR , "break_error" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_BUSY, "rx_dma_busy" },
    {.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_OVRN_ERROR, "overrun error" },
    {.bEnabled = false, .pfStreamAction = 0, .streamID = (am_hal_uart_stream_status_t)0, NULL },  // zero entry must be last
};

const char cStartupTestString[] = "Starting TxDma test\r\n";

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
    if (*(ptCallbackData->hasData))
    {
        *(ptCallbackData->hasData) = 0;
        g_localv.bDataAvailable = true;
    }
}

//*****************************************************************************
//
//! @breif will read rx data and transmit the data
//!
//! @param psAsyncLVars pointer test specific variables
//!
//! @return std hal status
//
//*****************************************************************************
static uint32_t echoData(test_local_vars_t *psAsyncLVars)
{
    (void) psAsyncLVars;

    uint8_t  rxBuf[512];  // 20 msec @ 115200 = 256 bytes
    uint32_t loopLimit = 0;

    while (true)
    {
        //
        // read uart after interrupt
        // here the uart will accumulate data until the fifo is full or a rx timeout occurs
        // Note: the rx-timeout interrupt must be enabled for the rx timeout to occur
        //
        if (++loopLimit > 8)
        {
            am_util_stdio_printf("serial_process_isr_status_echo loop limit\r\n");
            break;
        }

        uint32_t numBytesRead   = am_hal_uart_stream_get_rx_data(g_tstLocVar.sStreamTestTxDma.pUartHandle,
                                                              rxBuf, sizeof(rxBuf), false);
        if (numBytesRead == 0)
        {
            break;
        }

        am_hal_uart_errors_t ui32TxStat = am_hal_stream_uart_append_tx(g_tstLocVar.sStreamTestTxDma.pUartHandle, rxBuf, numBytesRead);

        if (ui32TxStat != AM_HAL_UART_STATUS_SUCCESS )
        {
            if (ui32TxStat == AM_HAL_UART_ERR_BUFFER_OVERFILL)
            {
                am_util_stdio_printf( "tx buff overfill\n" );
            }
            else
            {
                am_util_stdio_printf( "uart tx error %d \n", ui32TxStat );
            }
            break;
        }

        if (numBytesRead != sizeof(rxBuf))
        {
            //
            // drained all the data from queue, since read buffer isn't full
            //
            break;
        }
    } // while

    return AM_HAL_STATUS_SUCCESS;
}

//******************************************************************************
//
//! @brief init tx dma echo uart settings
//!
//! @return  standard hal status
//
//*****************************************************************************
static uint32_t serial_interface_init_txEcho(void)
{
    uart_examp_stream_vars_t *psStreamTestTxDma = &g_tstLocVar.sStreamTestTxDma;

    // Grab the pointer to the uart config table.
    // This is the uart config parameters like baud and stop bits.
    //
    psStreamTestTxDma->psUartConfig = &g_sUartConfigEchoTxDMA;

    //
    // if this timer callback function isn't provided, the timer won't be used
    // This is so the timer can run in the background for poling activities (if wanted)
    //
    g_tstLocVar.sStreamTestTxDma.pfTimerCallback = NULL;

    //
    // set timer interrupt
    // not used since callback is null
    //
    g_tstLocVar.sStreamTestTxDma.sUartTimerParams.ui32TimerPeriodMs = TIMER_PERIOD_MS; // @todo is this set in the hal

    psStreamTestTxDma->uart_hardware_instance = UART_ID;

    return uart_cmn_serial_interface_init(psStreamTestTxDma,
        &g_rxConfig,
        &g_txConfig );
}

//******************************************************************************
//
//! @brief periodic test call, used to queue tx data
//! called from background loop due to expiring timer (200msec)
//!
//! @param psLocalVars  -- unused
//!
//! @return
//
//*****************************************************************************
static uint32_t serial_process_timer_interrupt_echo(uart_examp_stream_vars_t *psLocalVars)
{
    (void) psLocalVars;
    //if ( g_eEchoMode == eECHO_MODE_POLLING)
    {
       echoData(0);
    }

    return AM_HAL_STATUS_SUCCESS;
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
    if ( ui32Status )
    {
        while(1);
    }

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Uart Streaming TxDma Echo Example!\n\n");

    //
    // Configure the SRAM memory containing DMA buffers as non-cacheable
    //
    ui32Status = uart_stream_cmn_mpuConfig();
    if ( ui32Status )
    {
        while(1);
    }

    //
    // preform serial init
    //
    serial_interface_init_txEcho();

    //
    // Enable interrupt service routines.
    //
    am_hal_interrupt_master_enable();

    //
    // last thing to do is enable RX interrupts
    //

    am_hal_uart_stream_interrupt_clr_set(g_tstLocVar.sStreamTestTxDma.pUartHandle,
        0, UART0_IER_RTIM_Msk | UART0_IER_RXIM_Msk);

    //
    // At this point, the uart should be configured for Tx dma.
    // The first test is to send a small string out the serial port
    //
    am_hal_uart_errors_t ui32TxStat = am_hal_stream_uart_append_tx (g_tstLocVar.sStreamTestTxDma.pUartHandle,
                                                                    (uint8_t *) cStartupTestString,
                                                                    sizeof(cStartupTestString)-1);

    if ( ui32TxStat )
    {
        const char *pstr  = printErrorString( ui32TxStat, false );
        am_util_stdio_printf("Serial tx dma start error! %s : %d\nProgram will hang\n", pstr, ui32TxStat);
        while(1);
    }

    am_util_stdio_printf("Tx DMA Echo example started\n");

    while (1)
    {
        // check through the uart status
        // read data
        // look at input status
        // could mask interrupts or disable ISR interrupts

        am_hal_uart_stream_status_t isrStatus;
        AM_CRITICAL_BEGIN
            //
            // read then clear the accumulated status for this module
            //
            isrStatus = g_tstLocVar.sStreamTestTxDma.e32Status;
            g_tstLocVar.sStreamTestTxDma.e32Status = AM_HAL_UART_STREAM_STATUS_SUCCESS;

        AM_CRITICAL_END

        if (isrStatus != AM_HAL_UART_STREAM_STATUS_SUCCESS)
        {
            processStreamError(streamErrorMessageMap, &g_tstLocVar.sStreamTestTxDma, isrStatus);
        }

        if (g_localv.bDataAvailable)
        {
            //
            // The rx callback occurred
            //
            g_localv.bDataAvailable = false;
            echoData(&g_tstLocVar);

        }

        if (g_localv.bTimerIntOccurred)
        {
            //
            // timer interrupt, this is used to send data periodically
            //
            g_localv.bTimerIntOccurred = false;

            serial_process_timer_interrupt_echo(&g_tstLocVar.sStreamTestTxDma);

        } // g_localv.bTimerIntOccurred
    } // while(1)
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
