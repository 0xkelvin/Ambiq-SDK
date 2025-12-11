//*****************************************************************************
//
//! @file uart_echo_test.c
//!
//! @brief This uart async test is designed to read in serial data and
//! immediately send it back out (echo)
//!
//! Use ser_echo_test.py to test this
//!
//! @addtogroup uart_stream
//! @{
//! @defgroup uart_echo_test_c UART Streaming Echo Test
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
#include "uart_echo_test.h"
#include "am_util.h"

#define RX_USED_DATA_BUFF_SIZE 2048
#define TX_USED_DATA_BUFF_SIZE 2048

#define TIMER_PERIOD_MS 10  //!< call the timer function at 100Hz for this test
#define USE_DTCM_RX_BUFF 0

//
//! Defines what mode to use to read serial data
//
typedef enum
{
    //
    //! this is called in the background loop due to an event from
    //! the uart ISR
    //
    eECHO_MODE_INTERRUPT_TRIGGERED,
    //
    //! This is called at a periodic rate to collect the serial data
    //! that has arrived since the last call.
    //!
    eECHO_MODE_POLLING,
}
echo_mode_t;


static const echo_mode_t g_eEchoMode = eECHO_MODE_POLLING;

static uint32_t serial_interface_init_echo(uart_examp_stream_vars_t *psAsyncLVars);
static uint32_t serial_process_timer_interrupt_echo(uart_examp_stream_vars_t *psAsyncLVars);
static uint32_t serial_process_isr_status_echo(uart_examp_stream_vars_t *psAsyncLVars,
                                               am_hal_uart_stream_status_t eIsrStatus);
static uint32_t uart_echo_interrupt_enable(uart_examp_stream_vars_t *psStreamLVars);


static uart_examp_stream_vars_t streamTest;

const uart_cmn_fcn_t g_sUartFcnEchoTest =
{
    .pfSerialIfInit             = serial_interface_init_echo,
    .pfSerInterruptEnable       = uart_echo_interrupt_enable,
    .pfSerProcessIsrStatus      = serial_process_isr_status_echo,
    .pfSerProcessTimerInterrupt = serial_process_timer_interrupt_echo,
    .psStreamLocalVars           = &streamTest,
    .testName                   = "echo test",
};

static const am_hal_uart_config_t g_sUartConfig4 =
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
    .ui32NumBytesToRead =  0,
    .pfRxCallback       = 0,
    .ui32TimeoutMs      = 0,
    .bClearRxFifo       = false,
    .sRxBuffer = { .pui8Buff = NULL, .ui32BufferSize = MAX_RX_DATA_BUFFER_SIZE > RX_USED_DATA_BUFF_SIZE ? RX_USED_DATA_BUFF_SIZE : MAX_RX_DATA_BUFFER_SIZE  },
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

typedef struct
{
    uart_examp_stream_init_args_t sDataBuff;
    //uint32_t ui32DataRxed;
    //uint32_t ui32maxDataBuffSize;
    uint32_t ui32TestDbuffSize;
}
test_local_vars_t;

static test_local_vars_t g_tstLocVar;

static void setRxBufferSize(void);

#if USE_DTCM_RX_BUFF
static uint8_t localRxBuff[2048];
#endif


static uint32_t handleRxDataAvail(uart_examp_stream_vars_t *psAsyncLVars);
//
//! @brief This is a helper table used to (efficiently) print statuses returned from the uart streaming isr call
//! @note arrange the most likely statuses first
//
static const stream_error_message_action_map_t streamErrorMessageMap[] =
{
{.bEnabled = true, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_COMPLETE, "uart_rx_dma_complete" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_STREAM_STATUS_TX_BUSY           , "tx busy" },
{.bEnabled = false, .pfStreamAction = 0, .streamID = AM_HAL_UART_RX_DMA_BUSY, "rx_dma_busy" },
{.bEnabled = true, .pfStreamAction = handleRxDataAvail, .streamID = AM_HAL_UART_STREAM_STATUS_RX_DATA_AVAIL     , "rx_data_avail" },
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

//! @brief  this is called to enable and start interrupts for the test
//! @param psStreamLVars pointer to
static uint32_t uart_echo_interrupt_enable(uart_examp_stream_vars_t *psStreamLVars)
{
    serial_print_test_name(&g_sUartFcnEchoTest);
    return serial_interrupt_enable(psStreamLVars);  // call default function in common
}

//! @brief called from the action map when rx data is available, and enabled in the action-map

static uint32_t handleRxDataAvail(uart_examp_stream_vars_t *psAsyncLVars)
{
    am_hal_uart_errors_t ui32TxStat = AM_HAL_UART_STATUS_SUCCESS;
    if ( g_eEchoMode == eECHO_MODE_INTERRUPT_TRIGGERED)
    {
        uint8_t  rxBuf[128];
        uint32_t loopLimit = 0;

        while (true)
        {
            //
            // read uart after interrupt
            // here the uart will accumulate data until the fifo is full or a rx timeout occurs
            // Note: the rx-timeout interrupt must be enabled for the rx timeout to occur
            //

            if (++loopLimit > 10)
            {
                am_util_stdio_printf("serial_process_isr_status_echo loop limit\r\n");
                break;
            }
            uint32_t numBytesRead   = am_hal_uart_stream_get_rx_data(psAsyncLVars->pUartHandle,
                                                                  rxBuf,
                                                                  sizeof(rxBuf),
                                                                  false);

            if (numBytesRead == 0)
            {
                break;
            }

            am_util_stdio_printf("num bytes rxed on uart %d : %d\r\n",
                psAsyncLVars->uart_hardware_instance,
                numBytesRead);
            ui32TxStat = am_hal_stream_uart_append_tx(psAsyncLVars->pUartHandle, rxBuf, numBytesRead);
            if (ui32TxStat == AM_HAL_UART_ERR_BUFFER_OVERFILL)
            {
                //am_util_stdio_printf( "overfill %d %d %d\n", qCount, bout, txSize );
                break;
            }
            if (numBytesRead != sizeof(rxBuf))
            {
                break;
            }

        } // while
    }
    return (uint32_t) ui32TxStat;

}

//*****************************************************************************
//
// will process returned ISR data, called in background loop
//
//*****************************************************************************
static uint32_t serial_process_isr_status_echo(uart_examp_stream_vars_t *psAsyncLVars,
                                               am_hal_uart_stream_status_t eIsrStatus)
{
    return processStreamError( streamErrorMessageMap, psAsyncLVars, eIsrStatus );
}

//******************************************************************************
//
//! @brief fill test output buffer
//! not important here(echo example), because test output buffer is rxed
//
//******************************************************************************
static void setRxBufferSize(void)
{
    uint32_t ui32DuffSize           = g_tstLocVar.sDataBuff.ui32TestBuffMax;
    g_tstLocVar.ui32TestDbuffSize = ui32DuffSize > RX_USED_DATA_BUFF_SIZE ? RX_USED_DATA_BUFF_SIZE : ui32DuffSize;
}
//******************************************************************************
//
//! @brief periodic test call, used to queue tx data
//! called from background loop due to expiring timer
//!
//! @param psAsyncLVars  contains init params
//!
//! @return  standard hal status
//
//*****************************************************************************
static uint32_t serial_interface_init_echo( uart_examp_stream_vars_t *psAsyncLVars )
{
    g_tstLocVar.sDataBuff = psAsyncLVars->stestBuffInfo; // struct copy

    setRxBufferSize();

    psAsyncLVars->psUartConfig = &g_sUartConfig4;
    // psAsyncLVars->pfTimerCallback = 0;  // This is setup in the main function
    psAsyncLVars->sUartTimerParams.ui32TimerPeriodMs = TIMER_PERIOD_MS;  // set this 0 to disable timer
    psAsyncLVars->uart_hardware_instance = UART_ID;

    return uart_cmn_serial_interface_init(psAsyncLVars, &g_rxConfig, &g_txConfig);
}

//******************************************************************************
//
//! @brief periodic test call, used to queue tx data
//! called from background loop due to expiring timer (200msec)
//!
//! @param psAsyncLVars
//!
//! @return
//
//*****************************************************************************
static uint32_t serial_process_timer_interrupt_echo(uart_examp_stream_vars_t *psAsyncLVars)
{
    if ( g_eEchoMode == eECHO_MODE_POLLING)
    {
        uint8_t  rxBuf[512];  // 20 msec @ 115200 = 256 bytes
        uint32_t loopLimit = 0;

        while (true)
        {
            //
            // read uart after interrupt
            // here the uart will accumulate data until the fifo is full or a rx timeout occurs
            // Note: the rx-timeout interrupt must be enabled for the rx timeout to occur
            //
            if (++loopLimit > 4)
            {
                am_util_stdio_printf("serial_process_isr_status_echo loop limit\r\n");
                break;
            }

            uint32_t numBytesRead   = am_hal_uart_stream_get_rx_data(psAsyncLVars->pUartHandle,
                                                                  rxBuf,
                                                                  sizeof(rxBuf),
                                                                  false);

            if (numBytesRead == 0)
            {
                break;
            }

            am_util_stdio_printf("rxed on uart %d %08x\r\n", numBytesRead, __REV(*((uint32_t *) rxBuf)));
            am_hal_uart_errors_t ui32TxStat = am_hal_stream_uart_append_tx(psAsyncLVars->pUartHandle, rxBuf, numBytesRead);

            if (ui32TxStat == AM_HAL_UART_ERR_BUFFER_OVERFILL)
            {
                am_util_stdio_printf( "overfill %d %d %d\n", 0, 0, 0 );
                break;
            }
            if (numBytesRead != sizeof(rxBuf))
            {
                //
                // got all the data
                //
                break;
            }

        } // while
    }

    return AM_HAL_STATUS_SUCCESS;
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
