//*****************************************************************************
//
//! @file uart_fullduplex_common.h
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
#ifndef UART_FULL_DUPLEX_CMN_H
#define UART_FULL_DUPLEX_CMN_H

#include "am_mcu_apollo.h"
#include "am_bsp.h"

#define UART_DMA_ENABLED        1

#define UART_QUEUE_SIZE         2048
#define UART_BUFFER_SIZE        2048
#define UART_DMA_SIZE           (UART_BUFFER_SIZE/2)

#define UART_TEST_PACKET_SIZE   2048
#define UART_TEST_PACKET_CNT    50

#define UART_DEVICE_READY_IN_PIN 5
#define UART_DEVICE_READY_OUT_PIN 5

typedef enum
{
    eTrxTypeRx,
    eTrxTypeTx,
    eTrxMAX,
}
transfer_type_t;

typedef enum
{
    eRX,
    eTX,
    eRTS,
    eCTS,
    eNUM_PIN_TYPES,
}
pin_type_e;

typedef struct
{
    uint32_t             pinNUmber;
    am_hal_gpio_pincfg_t *pinCfg;
    bool                 enabled;
    pin_type_e           ePinType;
}
uart_pin_descr_t;

//
//! defines the uart hardware config and use.
//! @note match the order here to what has been defined in pin_type_e
//
typedef struct
{
    uart_pin_descr_t           pinDescr[eNUM_PIN_TYPES];
    bool                       bValid;
    uint8_t                    uartHwInstance;
}
uart_hw_defs_t;

typedef struct
{
    const uart_hw_defs_t *psUartHwDefs;
    const am_hal_uart_config_t *psUartConfig;
    const am_hal_uart_stream_data_config_t *psUartStreamDataConfig;
    void *pUartHandle;
    am_hal_uart_stream_status_t e32Status;
    uint32_t ui32ISrErrorCount;

    transfer_type_t eTransferType;          // is this tx or rx dma mode
    uint8_t align[3];

}
uart_vars_t;

//*****************************************************************************
//
//! @brief Sets up the UART configuration for full-duplex communication.
//!
//! @details This function performs initialization and configuration for UART transmit
//! and receive modules. It verifies that the transmit and receive UART modules
//! are not the same, initializes the UART pins, sets up the RX and TX UART
//! modules, and prints the pin configuration. Any errors during this process
//! are logged and the corresponding error code is returned.
//!
//! @return Returns AM_HAL_STATUS_SUCCESS on success, or an appropriate error
//!         code from am_hal_status_e if a failure occurs during initialization.
//
//*****************************************************************************
uint32_t fd_common_setup(void);

//*****************************************************************************
//
//! @brief Initializes a UART instance for the specified transfer type (RX or TX).
//!
//! @param eTxType The transfer type, either RX (eTrxTypeRx) or TX (eTrxTypeTx).
//! @param uartInstance The UART instance number to initialize. Valid values are less than UART_MAX.
//! @return Returns AM_HAL_STATUS_SUCCESS on successful initialization, or appropriate error codes:
//!         - AM_HAL_STATUS_INVALID_ARG if uartInstance or eTxType is invalid.
//
//*****************************************************************************
uint32_t init_uart( transfer_type_t eTxType, uint32_t uartInstance );


//*****************************************************************************
//
//! @brief Executes a full-duplex UART test, which involves transmitting and
//! receiving data using UART DMA and verifying the data integrity.
//!
//! The method performs the following actions:
//! - Initializes the UART receive and transmit DMA parameters.
//! - Transmits a test packet and waits for both the transmit and receive to complete.
//! - Handles timeouts and checks for errors in received data.
//!
//! @return AM_HAL_STATUS_SUCCESS on successful execution of the test and data validation;
//!         AM_HAL_STATUS_FAIL on errors such as DMA timeout or data verification failures.
//
//*****************************************************************************
int32_t uart_run_fullduplex_test(void);

//*****************************************************************************
//
//! @brief Initializes the MCU for low-power operation, configures clocks, and enables debugging interfaces.
//!
//! This function sets up the board for low power consumption, enables the ITM print interface,
//! and configures the clocking mechanisms depending on the specific hardware configuration.
//! On certain hardware, it enables HFADJ and configures the SYSPLL or PLLPOSTDIV as required.
//!
//! @return A status code indicating the success or failure of the initialization process.
//!         - 0: Success
//!         - Non-zero: Error code indicating the specific failure.
//
//*****************************************************************************
uint32_t mcu_setup(void);

//*****************************************************************************
//
//! @brief Initializes the UART hardware with the appropriate configuration based on the provided UART variables.
//!
//! This function sets up the GPIO pins, initializes the UART hardware instance, powers on the UART,
//! configures UART parameters (such as baud rate and stop bits), and enables required interrupts.
//! The function ensures that the operation is only performed if the specific pin and instance are valid
//! and operational. It elevates the interrupt priority for high-speed testing.
//!
//! @param pUartVar Pointer to a uart_vars_t structure containing configuration details and hardware definitions for the UART.
//! @return AM_HAL_STATUS_SUCCESS on successful initialization. On failure, returns an appropriate error
//!         code such as AM_HAL_STATUS_INVALID_OPERATION, AM_HAL_STATUS_INVALID_ARG, or any other relevant
//!         error indicating the stage of failure.
//
//*****************************************************************************
uint32_t uart_fd_init(uart_vars_t *pUartVar);

//*****************************************************************************
//
// @brief disconnects the UARTSs
//
//*****************************************************************************
void common_teardown(void);

#endif // UART_FULL_DUPLEX_CMN_H
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
