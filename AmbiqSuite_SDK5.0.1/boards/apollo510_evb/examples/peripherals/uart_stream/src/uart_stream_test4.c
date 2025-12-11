//*****************************************************************************
//
//! @file uart_stream_test4.c
//!
//! @brief This uart stream test is designed to keep the tx buffer full using
//! small appends
//!
//! Use serial1_test.py to test this
//!
//! @addtogroup uart_stream
//! @{
//! @defgroup uart_stream_test4_c UART Streaming Common Test4
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
#include "uart_stream_test4.h"
#include "am_util.h"

#define RX_USED_DATA_BUFF_SIZE 2048
#define TX_USED_DATA_BUFF_SIZE 8192

static uint32_t serial_interface_init_t4(uart_examp_stream_vars_t *psStreamLVars);
static uint32_t serial_process_timer_interrupt_t4(uart_examp_stream_vars_t *psStreamLVars);
static uint32_t serial_process_isr_status(uart_examp_stream_vars_t *psStreamLVars, am_hal_uart_stream_status_t eIsrStatus);
static uint32_t uart_test4_interrupt_enable(uart_examp_stream_vars_t *psStreamLVars);

static uart_examp_stream_vars_t streamTest;

const uart_cmn_fcn_t g_sUartFcnArrayTest4 =
{
    .pfSerialIfInit             = serial_interface_init_t4,
    .pfSerInterruptEnable       = uart_test4_interrupt_enable,
    .pfSerProcessIsrStatus      = serial_process_isr_status,
    .pfSerProcessTimerInterrupt = serial_process_timer_interrupt_t4,
    .psStreamLocalVars           = &streamTest,
    .testName                   = "Streaming test 4",

};

//
//! Uart configs used.
//! Modify and add additional configs as needed.
//! Use this in gs_UartPinDefs below.
//
static const am_hal_uart_config_t sUartConfigTest4 =
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
//! Configure RxDma parameters
//
static const am_hal_uart_stream_rx_config_t g_rxConfig =
{
    .ui32NumBytesToRead =  RX_BUFF_THRESHOLD,
    .pfRxCallback       = 0,
    .ui32TimeoutMs      = 0,
    .bClearRxFifo       = false,
    .sRxBuffer = { .pui8Buff = NULL, .ui32BufferSize = MAX_RX_DATA_BUFFER_SIZE > RX_USED_DATA_BUFF_SIZE ? RX_USED_DATA_BUFF_SIZE : MAX_RX_DATA_BUFFER_SIZE},
};

//
//! Configure uart Tx setup parameters
//
static const am_hal_uart_stream_tx_config_t g_txConfig =
{
    .pfTxCallback       = NULL,
    .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE, //!< from uart_stream_example_config.h
    .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
    .sTxBuffer = { .pui8Buff = NULL, .ui32BufferSize = TX_USED_DATA_BUFF_SIZE }, //!< force use of SRAM
};

typedef struct
{
    uart_examp_stream_init_args_t sDataBuff;
    uint32_t ui32TestDbuffSize;         //!< Size of data buffer used here
    uint8_t *pui8Start;
    uint8_t *pui8End;
    uint8_t *pui8Curr;      //!< active pointer to output buffer
}
test_local_vars_t;

//
//! local variables for this test example
//
static test_local_vars_t g_tstLocVar;


static uint32_t fillBuffer(void);

//******************************************************************************
//
//! @brief This is a helper table used to (efficiently) print statuses returned from the uart streaming isr call
//! @note arrange the most likely statuses first
//
static const stream_error_message_action_map_t streamErrorMessageMap[] =
{
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "uart_rx_dma_complete" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_BUSY           , "tx busy" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_BUSY, "rx_dma_busy" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_RX_DATA_AVAIL     , "rx_data_avail" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_COMPLETE       , "tx_complete" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "rx_dma_complete" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_ERROR, "rx_dma_error" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_TIMEOUT, "rx_dma_timeout" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_OVERFLOW, "rx_dma_overflow" },

{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_RX_QUEUE_FULL,   "rx_queue_full" },
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
//*****************************************************************************
//
//*****************************************************************************
static uint32_t uart_test4_interrupt_enable(uart_examp_stream_vars_t *psStreamLVars)
{
    serial_print_test_name(&g_sUartFcnArrayTest4);
    return serial_interrupt_enable(psStreamLVars);
}
//*****************************************************************************
//
//*****************************************************************************
static uint32_t fillBuffer(void)
{
    uint32_t ui32DuffSize           = g_tstLocVar.sDataBuff.ui32TestBuffMax;
    if (ui32DuffSize < 256)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    g_tstLocVar.ui32TestDbuffSize = ui32DuffSize > TX_USED_DATA_BUFF_SIZE ? TX_USED_DATA_BUFF_SIZE : ui32DuffSize;

    g_tstLocVar.pui8Start = g_tstLocVar.sDataBuff.pui8Testbuff;
    g_tstLocVar.pui8Curr  = g_tstLocVar.sDataBuff.pui8Testbuff;
    g_tstLocVar.pui8End   = (uint8_t *) ((uint32_t) g_tstLocVar.sDataBuff.pui8Testbuff + g_tstLocVar.ui32TestDbuffSize);

    uint32_t size = g_tstLocVar.ui32TestDbuffSize / 2;
    uint16_t *p16 = (uint16_t *) g_tstLocVar.sDataBuff.pui8Testbuff;

    for ( uint32_t i = 0; i < size; i++ )
    {
        *p16++ = i;
    }
    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
// called in the background due to an isr event
//*****************************************************************************
static uint32_t serial_process_isr_status(uart_examp_stream_vars_t *psStreamLVars,
                                          am_hal_uart_stream_status_t eIsrStatus)
{
    return processStreamError( streamErrorMessageMap, psStreamLVars, eIsrStatus );
}

//******************************************************************************
//
//! @brief periodic test call, used to queue tx data
//! called from background loop due to expiring timer
//!
//! @param psStreamLVars  contains init params
//!
//! @return  standard hal status
//
//*****************************************************************************
static uint32_t serial_interface_init_t4( uart_examp_stream_vars_t *psStreamLVars )
{
    g_tstLocVar.sDataBuff = psStreamLVars->stestBuffInfo;  // struct copy

    uint32_t ui32Status = fillBuffer();
    if ( ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf( "serial_interface_init_t1: fill buffer error %d\n", ui32Status);
        return ui32Status;
    }
    psStreamLVars->psUartConfig = &sUartConfigTest4;
    // psAsyncLVars->pfTimerCallback = 0;  // This is setup in the main function
    psStreamLVars->sUartTimerParams.ui32TimerPeriodMs = 1000;
    psStreamLVars->uart_hardware_instance = UART_ID;
    return uart_cmn_serial_interface_init(psStreamLVars, &g_rxConfig, &g_txConfig);
}

static const uint32_t txRefSize = 83 * 2;  // this is 83 shorts

//******************************************************************************
//
//! @brief periodic test call, used to queue tx data
//! called from background loop due to expiring timer (200msec)
//!
//! @param psStreamLVars  - pointer stream vars
//!
//! @return
//
//*****************************************************************************
static uint32_t serial_process_timer_interrupt_t4(uart_examp_stream_vars_t *psStreamLVars)
{
    uint32_t ui32TxCount = ++streamTest.ui32TxCount;

    uint32_t qCount = 0;
    uint32_t bout = 0;

    //
    // queue small blocks of data until overflow, then exit function
    //
    while(true)
    {

        uint8_t *p = g_tstLocVar.pui8Curr;  // grab pointer

        //
        // number of bytes left till end of buffer
        //
        uint32_t ui32Room = (uint32_t) g_tstLocVar.pui8End - (uint32_t) p;


        uint32_t txSize = txRefSize;
        bool addTxSize = true;

        if (txSize >= ui32Room)
        {
            // not enough room towards the end of buffer to send full buffer
            if ( ui32Room < 2 )
            {
                //
                // at the end of buffer now, start from the beginning of buffer
                //
                p = g_tstLocVar.pui8Start;
            }
            else
            {
                //
                // some data left in buffer, start from the beginning of buffer next time
                // so don't increment pointer
                //
                txSize = ui32Room;  // lower the tx size to what is available
                addTxSize = false;  // next time starting with fresh buffer and no offset
            }

            // next time will need to start from beginning or near beginning of buffer
            g_tstLocVar.pui8Curr = g_tstLocVar.pui8Start;

            am_util_stdio_printf( "room %d %d %d %08x\n", qCount, ui32Room, txSize, (uint32_t) p);
        }
        else
        {
            // everything fits
        }


        am_hal_uart_errors_t ui32TxStat = am_hal_stream_uart_append_tx(psStreamLVars->pUartHandle, (uint8_t *) p, txSize);
        if ( ui32TxStat == AM_HAL_UART_ERR_BUFFER_OVERFILL )
        {
            am_util_stdio_printf( "overfill %d %d %d\n", qCount, bout, txSize );
            break;
        }
        if (addTxSize)
        {
            g_tstLocVar.pui8Curr += txSize;
        }
        qCount++;
        errorCheck(ui32TxStat, ui32TxCount, qCount);
        bout += txSize;
    }

    streamTest.bytesOut += bout;

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
