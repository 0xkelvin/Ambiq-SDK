//*****************************************************************************
//
//! @file uart_stream_test1.c
//!
//! @brief contains simple buffer dma and non dma test
//!
//! Use a serial terminal program to test by watching the output pattern
//! Also contains example of isr service return processing for rx data
//!
//! @addtogroup uart_stream
//! @{
//! @defgroup uart_stream_test1_c UART Streaming Common Test1
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
#include "uart_stream_test1.h"
#include "am_util.h"
#include <string.h>

#define RX_USED_DATA_BUFF_SIZE 2048
#define TX_USED_DATA_BUFF_SIZE 2048

static uint32_t serial_interface_init_t1(uart_examp_stream_vars_t *psStreamLVars);
static uint32_t serial_process_timer_interrupt_t1(uart_examp_stream_vars_t *psStreamLVars);
static uint32_t serial_process_isr_status(uart_examp_stream_vars_t *psStreamLVars,
                                          am_hal_uart_stream_status_t eIsrStatus);
static uint32_t uart_test1_interrupt_enable(uart_examp_stream_vars_t *psStreamLVars);

static uart_examp_stream_vars_t streamTest1;

//
// array of function and data needed to run this example
//
const uart_cmn_fcn_t g_sUartFcnArrayTest1 =
{
    .pfSerialIfInit             = serial_interface_init_t1,
    .pfSerInterruptEnable       = uart_test1_interrupt_enable,
    .pfSerProcessIsrStatus      = serial_process_isr_status,
    .pfSerProcessTimerInterrupt = serial_process_timer_interrupt_t1,
    .psStreamLocalVars          = &streamTest1,
    .testName                   = "streaming test 1",
};

//
// create a buffer that will cause at least two fifo refills
//
static const uint8_t ui8cTxBuf[74] =
{
    'a', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'z', '\r', '\n', ' ',
};

//
// create a buffer that will not require a fifo refill
// (so no tx interrupt, just tx complete)
//
const static uint8_t ui8cTxBuf2[14] =
{
    'a', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L',
    'y', '\r', '\n', ' ',
};

//
//! Uart configs used.
//! Modify and add additional configs as needed.
//! Use this in gs_UartPinDefs below.
//
static const am_hal_uart_config_t sUartConfigT1 =
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
    .sTxBuffer = { .pui8Buff = NULL, .ui32BufferSize = TX_USED_DATA_BUFF_SIZE  }, //!< force use of SRAM
};

//
//! data buff params
//
typedef struct
{
    uart_examp_stream_init_args_t sTxBuff;
    const uint8_t *pui8cSourceBuff;
}
tx_buff_params_t;

typedef struct
{
    uart_examp_stream_init_args_t sDataBuff;

    uint32_t ui32TxCount;
    uint32_t bytesOut;
    tx_buff_params_t txBuffParams[2];
}
test_local_vars_t;

//
//! local to this function RAM vars
//
static test_local_vars_t g_tstLocVar;

static void copyTestBuff(tx_buff_params_t *psParam);
static uint32_t fillBuffer(void);
static uint32_t rxInputDataProcess(uart_examp_stream_vars_t *psAsyncLVars);

//*****************************************************************************
//
//! @brief This is a helper table used to (efficiently) print statuses returnrf
//! from the uart streaming isr call
//! @note arrange the most likely statuses first
//
//*****************************************************************************
static const stream_error_message_action_map_t streamErrorMessageMap[] =
{
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_DMA_BUSY       , "tx_dma_busy" },
{.bEnabled = true, .pfStreamAction = rxInputDataProcess, .streamID = AM_HAL_UART_STREAM_STATUS_RX_DATA_AVAIL     , "rx_data_avail" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_COMPLETE       , "tx_complete" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "uart_rx_dma_complete" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "rx_dma_complete" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_BUSY           , "tx busy" },
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_ERROR, "rx_dma_error" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_BUSY, "rx_dma_busy" },
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
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_OVRN_ERROR, "overrun error" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = (am_hal_uart_stream_status_t)0, NULL },  // zero entry must be last

};

//*****************************************************************************
//
//! @brief  This is called (in the background when the uart interrupt service returns rx bytes available
//! check the streamErrorMessageMap table to see where it is included
//!
//! @param psAsyncLVars
//! @return
//
//*****************************************************************************
static uint32_t rxInputDataProcess(uart_examp_stream_vars_t *psAsyncLVars)
{
    void *pHandle = psAsyncLVars->pUartHandle;
    uint32_t num_bytes_get = am_hal_uart_stream_get_num_rx_bytes_in_buffer(pHandle, false );
    uint8_t inBuff[64];
    uint32_t num_b_read = am_hal_uart_stream_get_rx_data(pHandle, inBuff, sizeof(inBuff), true );

    am_util_stdio_printf( "data read, num in buff, num read %d, %d\n", num_bytes_get, num_b_read );
    return 0;
}

//*****************************************************************************
//
//! @brief This is the example startup call from the function array
//!
//! @param psStreamLVars     pointer to local array
//
//*****************************************************************************
static uint32_t uart_test1_interrupt_enable(uart_examp_stream_vars_t *psStreamLVars)
{
    serial_print_test_name(&g_sUartFcnArrayTest1);
    return serial_interrupt_enable(psStreamLVars);
}

static void copyTestBuff(tx_buff_params_t *psParam)
{
    memcpy(psParam->sTxBuff.pui8Testbuff, psParam->pui8cSourceBuff, psParam->sTxBuff.ui32TestBuffMax);
}
//*****************************************************************************
//
// data init function
//
//*****************************************************************************
static uint32_t fillBuffer(void)
{
    const uint32_t buff2Offset = 512;
    uint32_t workingBuffSize = g_tstLocVar.sDataBuff.ui32TestBuffMax;
    if (workingBuffSize <= (buff2Offset + sizeof(ui8cTxBuf2)))
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    uint32_t constBuffSize = sizeof(ui8cTxBuf);
    g_tstLocVar.txBuffParams[0].pui8cSourceBuff          = ui8cTxBuf;

    g_tstLocVar.txBuffParams[0].sTxBuff.ui32TestBuffMax  = constBuffSize;
    g_tstLocVar.txBuffParams[0].sTxBuff.pui8Testbuff     = g_tstLocVar.sDataBuff.pui8Testbuff;

    constBuffSize = sizeof(ui8cTxBuf2);
    g_tstLocVar.txBuffParams[1].pui8cSourceBuff = ui8cTxBuf2;

    g_tstLocVar.txBuffParams[1].sTxBuff.ui32TestBuffMax = constBuffSize;
    g_tstLocVar.txBuffParams[1].sTxBuff.pui8Testbuff     = &g_tstLocVar.sDataBuff.pui8Testbuff[buff2Offset];

    copyTestBuff( &g_tstLocVar.txBuffParams[0]);
    copyTestBuff( &g_tstLocVar.txBuffParams[1]);

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief called in background with isr return status
//!
//! @param psStreamLVars pointer to this test's global array
//! @param eIsrStatus ISR status returned from isr_service
//!
//! @return hal status
//
//*****************************************************************************
static uint32_t serial_process_isr_status(uart_examp_stream_vars_t *psStreamLVars,
                                          am_hal_uart_stream_status_t eIsrStatus)
{
    return processStreamError( streamErrorMessageMap, psStreamLVars, eIsrStatus );
}

//*****************************************************************************
//
//! @brief test init function
//!
//! @param psStreamLVars pointer to test global params
//!
//! @return standard hal status
//
//*****************************************************************************
static uint32_t serial_interface_init_t1( uart_examp_stream_vars_t *psStreamLVars )
{
    g_tstLocVar.sDataBuff = psStreamLVars->stestBuffInfo;  // // struct copy

    uint32_t ui32Status  = fillBuffer();
    if (ui32Status)
    {
        am_util_stdio_printf( "serial_interface_init_t1: fill buffer error %d\n", ui32Status);
        return ui32Status;
    }
    psStreamLVars->psUartConfig = &sUartConfigT1;
    // psAsyncLVars->pfTimerCallback = 0;  // This is setup in the main function
    psStreamLVars->sUartTimerParams.ui32TimerPeriodMs = 1000;  // set this 0 to disable timer, want 1 a second
    psStreamLVars->uart_hardware_instance = UART_ID;
    return uart_cmn_serial_interface_init(psStreamLVars, &g_rxConfig, &g_txConfig);
}

//*****************************************************************************
//
//! @brief periodic timer interrupt
//!
//! @param psStreamLVars  pointer to local variable (not used
//!
//! @return standard hal status
//
//*****************************************************************************
static uint32_t serial_process_timer_interrupt_t1(uart_examp_stream_vars_t *psStreamLVars)
{
    (void) psStreamLVars;
    uint32_t ui32TxCount    = ++g_tstLocVar.ui32TxCount;
    uint32_t txBuffIdx      = ui32TxCount & 0x01;

    uint8_t *p              = g_tstLocVar.txBuffParams[txBuffIdx].sTxBuff.pui8Testbuff;
    uint32_t txSize         = g_tstLocVar.txBuffParams[txBuffIdx].sTxBuff.ui32TestBuffMax;

    void *pUartHandle = psStreamLVars->pUartHandle;

    uint32_t ax = (ui32TxCount % 26) + 'a';
    p[0] = ax;
    p[1] = 'A';
    am_util_stdio_printf( "Call A\n" );
    am_hal_uart_errors_t ui32TxStat = am_hal_stream_uart_append_tx(pUartHandle, (uint8_t *) p, txSize);
    uint32_t bout = txSize;
    errorCheck(ui32TxStat, ui32TxCount, 1);

    p[1] = 'B';
    am_util_stdio_printf( "Call B\n" );
    ui32TxStat = am_hal_stream_uart_append_tx(pUartHandle, (uint8_t *) p, txSize);
    bout += txSize;
    errorCheck(ui32TxStat, ui32TxCount, 2);

    p[1] = 'C';
    am_util_stdio_printf( "Call C\n" );
    ui32TxStat = am_hal_stream_uart_append_tx(pUartHandle, (uint8_t *) p, txSize);
    bout += txSize;
    errorCheck(ui32TxStat, ui32TxCount, 3);

    g_tstLocVar.bytesOut += bout;

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
