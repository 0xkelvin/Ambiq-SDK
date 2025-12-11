//*****************************************************************************
//
//! @file uart_streaming_utils.c
//!
//! @brief This contains error display routines and the uart ISRs
//!
//! @addtogroup peripheral_examples Peripheral Examples
//!
//! @ingroup uart_fullduplex_common
//! @{
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

#include "uart_fullduplex_common.h"
#include "uart_streaming_utils.h"
#include "am_util.h"

//
//! decode standard error names for printing
//
static const char * const uartCmnErrStatus[] =
{
    "SUCCESS",
    "FAIL",
    "INVALID_HANDLE",
    "IN_USE",
    "TIMEOUT",
    "OUT_OF_RANGE",
    "INVALID_ARG",
    "INVALID_OPERATION",
    "MEM_ERR",
    "HW_ERR",
};

//
//! decode extended error names for printing
//
static const char * const uartDriverErrorStrings[] =
{
    "US_BUS_ERROR",
    "US_RX_QUEUE_FULL",
    "US_CLOCK_NOT_CONFIGURED",
    "US_BAUDRATE_NOT_POSSIBLE",
    "US_TX_CHANNEL_BUSY",
    "US_RX_CHANNEL_BUSY",
    "US_RX_TIMEOUT",
    "US_FRAME_ERROR",
    "US_PARITY_ERROR",
    "US_BREAK_ERROR",
    "US_OVERRUN_ERROR",
    "US_DMA_ERROR",
    "DMA_NO_INIT",
    "BUFFER_OVERFILL",
    "MEMORY_ERROR_01",
    "MEMORY_ERROR_02",
    "MEMORY_ERROR_03",
    "DMA_BUSY_ERROR",
    "RX_BUFF_TOO_SMALL",
    "DMA SETUP ERROR",
    "DMA INIT ERROR",
};


static uart_vars_t *psUartTxParams;
static uart_vars_t *psUartRxParams;

static void am_isr_common(uart_vars_t *pUartDescriptor);

//*****************************************************************************
//
// This will format an error string for either standard hal errors or uart
// specific errors
//
//*****************************************************************************
const char *printErrorString( am_hal_uart_errors_t eErrx, bool printError )
{
    const char *pcErrorInfoStr;
    if ((am_hal_status_e) eErrx <= AM_HAL_STATUS_HW_ERR )
    {
        pcErrorInfoStr = uartCmnErrStatus[eErrx];
    }
    else if ((eErrx >= (am_hal_uart_errors_t) AM_HAL_STATUS_MODULE_SPECIFIC_START) && (eErrx < AM_HAL_UART_ERR_END))
    {
        eErrx -= AM_HAL_STATUS_MODULE_SPECIFIC_START;
        pcErrorInfoStr = uartDriverErrorStrings[eErrx];
    }
    else
    {
        pcErrorInfoStr = "Unknown Error";
    }

    if ( printError )
    {
        am_util_stdio_printf("streaming call error status: %s\r\n", pcErrorInfoStr );
    }

    return pcErrorInfoStr;
}


//*****************************************************************************
//
//  Register pointers to data structs used in ISR setup and procession
//
//*****************************************************************************
void set_uart_tx_rx_params(uart_vars_t *psUartTxParamsi, uart_vars_t *psUartRxParamsi)
{
    psUartTxParams = psUartTxParamsi;
    psUartRxParams = psUartRxParamsi;
}

//*****************************************************************************
//
//! @brief Uart isr handlers
//! @note For example, purposes capture all uart ISRs here and process them in the same way.
//
//*****************************************************************************
void am_uart_isr(void)
{
#if  AM_REG_UART_NUM_MODULES > 0
#if TEST_UART_TX_MODULE == 0
    am_isr_common(psUartTxParams);
#elif TEST_UART_RX_MODULE == 0
    am_isr_common(psUartRxParams);
#endif
#endif
}

void am_uart1_isr(void)
{
#if  AM_REG_UART_NUM_MODULES > 1
#if TEST_UART_TX_MODULE == 1
    am_isr_common(psUartTxParams);
#elif TEST_UART_RX_MODULE == 1
    am_isr_common(psUartRxParams);
#endif
#endif
}

void am_uart2_isr(void)
{
#if AM_REG_UART_NUM_MODULES > 2
#if TEST_UART_TX_MODULE == 2
    am_isr_common(psUartTxParams);
#elif TEST_UART_RX_MODULE == 2
    am_isr_common(psUartRxParams);
#endif
#endif
}

void am_uart3_isr(void)
{
#if AM_REG_UART_NUM_MODULES > 3
#if TEST_UART_TX_MODULE == 3
    am_isr_common(psUartTxParams);
#elif TEST_UART_RX_MODULE == 3
    am_isr_common(psUartRxParams);
#endif
#endif
}

//*****************************************************************************
//
//! @brief Common uart isr handler
//!
//! @param pUartDescriptor
//
//*****************************************************************************
static void am_isr_common(uart_vars_t *pUartDescriptor)
{
    pUartDescriptor->e32Status |= am_hal_uart_interrupt_stream_service(pUartDescriptor->pUartHandle);

    if (pUartDescriptor->e32Status != AM_HAL_UART_STREAM_STATUS_SUCCESS)
    {
        pUartDescriptor->ui32ISrErrorCount++;
    }
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
