//*****************************************************************************
//
//! @file uart_streaming_utils.h
//!
//! @brief Common code and definitions for uart_stream examples
//!
//! @ingroup uart_fullduplex_common
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

#ifndef UART_STREAMING_UTILS_H
#define UART_STREAMING_UTILS_H

#include "am_mcu_apollo.h"
#include "uart_fullduplex_common.h"

typedef uint32_t (*pfStreamAction_t)(uart_vars_t *psAsyncLVars);

//*****************************************************************************
//
//! @brief Register pointers to data structs used in ISR setup and procession
//!
//! @param psUartTxParamsi pointer to uart Tx buffer
//! @param psUartRxParamsi pointer to uart Rx buffer
//
//*****************************************************************************
extern void set_uart_tx_rx_params(uart_vars_t *psUartTxParamsi, uart_vars_t *psUartRxParamsi);


//*****************************************************************************
//
//! @brief Lookup and return/print UART error string
//!
//! @param eErrx Uart Error
//! @param will print error string when true
//!
//! @return pointer to const error string
//
//*****************************************************************************
extern const char *printErrorString(am_hal_uart_errors_t eErrx, bool printError);

#endif //UART_STREAMING_UTILS_H
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
