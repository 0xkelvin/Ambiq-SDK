//*****************************************************************************
//
//! @file uart_echo_rx_dma_test.c
//!
//! @brief This uart async test is designed to read in serial data and
//! immediately send it back out (echo)
//!
//! Use ser_echo_rx_dma_test.py to test this
//!
//! @addtogroup uart_stream
//! @{
//! @defgroup uart_echo_rx_dma_rx_dma_test_c UART Streaming Receive DMA Echo Test
//! @{
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
#include "uart_echo_rx_dma_test.h"
#include "am_util.h"

#define RX_USED_DATA_BUFF_SIZE 2048
#define TX_USED_DATA_BUFF_SIZE 2048

#define UART_DMA_MODE  AM_HAL_UART_DMA_RX_DOUBLE
//#define UART_DMA_MODE  AM_HAL_UART_DMA_TX_DOUBLE_BUFFER
//#define UART_DMA_MODE  AM_HAL_UART_DMA_NONE

#define TIMER_PERIOD_MS 10  //!< call the timer function at 100Hz for this test
#define USE_DTCM_RX_BUFF 0  //!< don't use DTCM, use sram for RX DMA

//
//! table of functions and data used for this test
//
static uart_examp_stream_vars_t streamTestRxDma;

static uint32_t serial_interface_init_echo_rxDma(uart_examp_stream_vars_t *psStreamLVars);
static uint32_t serial_process_timer_interrupt_echo_rxDma(uart_examp_stream_vars_t *psStreamLVars);
static uint32_t serial_process_isr_status_echo_rxDma(uart_examp_stream_vars_t *psAsyncLVars, am_hal_uart_stream_status_t ui32IsrStatus);
static uint32_t serial_interrupt_enable_rxDma(uart_examp_stream_vars_t *psStreamLVars);


const uart_cmn_fcn_t g_sUartFcnEchoRxDMATest =
{
    .pfSerialIfInit             = serial_interface_init_echo_rxDma,
    .pfSerInterruptEnable       = serial_interrupt_enable_rxDma,      // local
    .pfSerProcessIsrStatus      = serial_process_isr_status_echo_rxDma,  // local
    .pfSerProcessTimerInterrupt = serial_process_timer_interrupt_echo_rxDma, // local
    .psStreamLocalVars           = &streamTestRxDma,
    .testName                   = "echo rxDma test",
};

//
//! Standard UART config settings
//
static const am_hal_uart_config_t sUartConfigEchoRxDMA =
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
//! Configure uart Tx setup parameters
//
static const am_hal_uart_stream_tx_config_t g_txConfig =
{
    .pfTxCallback       = NULL,
    .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE, //!< from uart_stream_example_config.h
    .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
    .sTxBuffer = { .pui8Buff = NULL, .ui32BufferSize = TX_USED_DATA_BUFF_SIZE}, //!< force use of SRAM
};

typedef struct
{
    uart_examp_stream_init_args_t sDataBuff;

    uint32_t ui32DataRxed;
    uint32_t ui32TestDbuffSize;

    //
    //! a copy of data sent in the rx dma callback
    //
    am_hal_uart_stream_callback_data_t sLocalRxDmaCbData;
    //
    //! This is set in the rx dma callback, it is checked in cleared in the background loop.
    //
    volatile bool rxData;   //!<
}
test_local_vars_t;

//
//! define a function that can be called (optional) on receipt of a specific stream error
//
typedef uint32_t (*pfStreamAction_t)(uart_examp_stream_vars_t *psAsyncLVars);

static void setRxBufferSize(void);
static void dmaCompleteCallBack( am_hal_uart_stream_callback_data_t *ptCallbackData);
static uint32_t processDmaComplete( uart_examp_stream_vars_t *psAsyncLVars);

//
//! @brief This is a helper table used to (efficiently) print statuses returned from the uart streaming isr call
//! @note arrange the most likely statuses first
//
static const stream_error_message_action_map_t streamErrorMessageMap[] =
{
{.bEnabled = true, .pfStreamAction = processDmaComplete, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "uart_rx_dma_complete" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_BUSY           , "tx busy" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_BUSY, "rx_dma_busy" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_COMPLETE       , "tx_complete" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "rx_dma_complete" },
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

//
//! Configure RxDma parameters
//
static const am_hal_uart_stream_rx_config_t g_rxConfig =
{
    .ui32NumBytesToRead =  RX_BUFF_THRESHOLD,
    .pfRxCallback       = dmaCompleteCallBack,
    .ui32TimeoutMs      = 0,
    .bClearRxFifo       = false,
    .sRxBuffer = {.pui8Buff = NULL, .ui32BufferSize = MAX_RX_DATA_BUFFER_SIZE > RX_USED_DATA_BUFF_SIZE ? RX_USED_DATA_BUFF_SIZE : MAX_RX_DATA_BUFFER_SIZE},
};


static test_local_vars_t g_tstLocVar;
#if USE_DTCM_RX_BUFF
static uint8_t localRxBuff[2048];
#endif

//*****************************************************************************
//
//! @brief will process returned ISR data, called in background loop
//!
//! @param psAsyncLVars pointer to test globals
//!
//! @return
//
//*****************************************************************************
static uint32_t processDmaComplete( uart_examp_stream_vars_t *psAsyncLVars)
{
    if (g_tstLocVar.rxData)
    {
        //
        // echo data, cannot use dma (if using the same uart)
        //
        am_hal_uart_errors_t ui32TxStat = am_hal_uart_append_tx_fifo( psAsyncLVars->pUartHandle,
            g_tstLocVar.sLocalRxDmaCbData.pui8Buffer,
            g_tstLocVar.sLocalRxDmaCbData.ui32BufferSize );
        if ( ui32TxStat )
        {
            am_util_stdio_printf( "tx resend error %x %d\n", ui32TxStat, psAsyncLVars->uartId );
        }
        __DMB();
        *g_tstLocVar.sLocalRxDmaCbData.hasData = 0; // used the data
        g_tstLocVar.rxData = false;
    }

    return 0;
}

//******************************************************************************
//
//! @brief This is used in a function call table
//! This is called when a non-trivial return happens in the isr service call.
//! This is called from the background loop (not from an ISR).
//! The background executive calls it quickly after the ISR service is called.
//!
//! @param psAsyncLVars pointer to test globals
//! @param ui32IsrStatus value returned from isr service call
//!
//! @return hal status
//
//******************************************************************************
//!
static uint32_t serial_process_isr_status_echo_rxDma(uart_examp_stream_vars_t *psAsyncLVars,
                                                     am_hal_uart_stream_status_t ui32IsrStatus)
{
    //
    // print isr activity/status messages
    //
    return processStreamError( streamErrorMessageMap,
                              psAsyncLVars,
                              (am_hal_uart_stream_status_t) ui32IsrStatus );
}

//******************************************************************************
//
//! @brief fill test output buffer
//! not important here(echo example), because test output buffer is RXed
//
//******************************************************************************
static void setRxBufferSize(void)
{
    uint32_t ui32DBuffSize           = g_tstLocVar.sDataBuff.ui32TestBuffMax;
    g_tstLocVar.ui32TestDbuffSize = ui32DBuffSize > RX_USED_DATA_BUFF_SIZE ? RX_USED_DATA_BUFF_SIZE : ui32DBuffSize;
}
//******************************************************************************
//! @brief rxDma Complete callback
//!
//! @note This is called from a UART ISR, when RX DMA is processed.
//!
//! @param ptCallbackData pointer to callback data
//!
//******************************************************************************
static void dmaCompleteCallBack( am_hal_uart_stream_callback_data_t *ptCallbackData)
{
    //
    // copy struct
    //
    g_tstLocVar.sLocalRxDmaCbData  = *ptCallbackData;
    //
    // notify background a new buffer is ready
    //
    g_tstLocVar.rxData            = true;
}
//******************************************************************************
//
//! @brief inits the rx dma echo
//! called when starting
//!
//! @param psStreamLVars  contains test globals
//!
//! @return  standard hal status
//
//*****************************************************************************
static uint32_t serial_interface_init_echo_rxDma( uart_examp_stream_vars_t *psStreamLVars )
{
    //
    // copy pointer to test buff and its size
    //
    g_tstLocVar.sDataBuff = psStreamLVars->stestBuffInfo;  // struct copy

    setRxBufferSize();

    //
    // set timer interrupt to 10msec
    //
    psStreamLVars->sUartTimerParams.ui32TimerPeriodMs = TIMER_PERIOD_MS;
    psStreamLVars->psUartConfig    = &sUartConfigEchoRxDMA;
    psStreamLVars->uart_hardware_instance = UART_ID;
    psStreamLVars->euartEaxampleStreamMode = AM_HAL_UART_DMA_RX_DOUBLE;

    return uart_cmn_serial_interface_init(&streamTestRxDma, &g_rxConfig, &g_txConfig);

}
//******************************************************************************
//
//! @brief enable rx DMA for this test
//!
//! @param psStreamLVars  contains init params
//!
//! @return  standard hal status
//
//*****************************************************************************
static uint32_t serial_interrupt_enable_rxDma(uart_examp_stream_vars_t *psStreamLVars)
{
    //
    //! start rx dma transfers
    //
    //am_hal_uart_errors_t rxStartStat =  am_hal_uart_stream_configure_rx( &g_tstLocVar.sStreamTestRxDma, &rxDmaConfig );
    bool bEnableRxDMA = true;
    bool bClearFifo = false;
    am_hal_uart_errors_t eStartState = am_hal_uart_stream_enable_rxDma(psStreamLVars->pUartHandle,
        bEnableRxDMA,
        bClearFifo);

    if (eStartState)
    {
        const char *pcErrorString = printErrorString( eStartState, false );
        am_util_stdio_printf("error: rx dma startup error: %s\r\n", pcErrorString );
    }
    else
    {
        am_util_stdio_printf("RX dma startup OK\r\n" );
    }

    return eStartState;
}

//******************************************************************************
//
//! @brief periodic test call, used to queue tx data
//! called from background loop due to expiring timer (200 msec)
//! @note This Rx DMA example uses polled ISR events, not this periodic function
//!
//! @param psStreamLVars  -- unused
//!
//! @return
//
//*****************************************************************************
static uint32_t serial_process_timer_interrupt_echo_rxDma(uart_examp_stream_vars_t *psStreamLVars)
{

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
