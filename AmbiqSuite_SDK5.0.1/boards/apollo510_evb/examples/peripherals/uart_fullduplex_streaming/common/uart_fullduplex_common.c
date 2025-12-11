//*****************************************************************************
//
//! @file uart_fullduplex_common.c
//!
//! @brief Common utilities for UART full-duplex examples on Apollo5.
//!
//! @defgroup uart_fullduplex_common UART FullDuplex Example Common
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This module provides shared configuration, pin mapping, and utility
//!          functions for UART full-duplex controller and device examples. It
//!          enables flexible hardware setup, buffer management, and unified
//!          initialization for reliable bidirectional UART communication.
//!
//! @section uart_fullduplex_common_features Key Features
//!
//! 1. @b Pin @b Mapping: Centralized pin configuration for all UART modules
//! 2. @b Buffer @b Management: Shared transmit/receive buffer definitions
//! 3. @b Hardware @b Abstraction: Unified hardware setup for controller/device
//! 4. @b Interrupt @b Handling: Common ISR and callback utilities
//! 5. @b Power @b Optimization: Low-power initialization routines
//!
//! @section uart_fullduplex_common_functionality Functionality
//!
//! - Pin mapping and configuration for all UART modules
//! - Buffer allocation and alignment for DMA and FIFO
//! - Common setup/teardown routines for power management
//! - Interrupt handler and callback registration
//! - Data validation and checksum utilities
//!
//! @section uart_fullduplex_common_usage Usage
//!
//! 1. Include this module in both controller and device examples
//! 2. Use provided pin mapping and buffer definitions
//! 3. Call common_setup() and common_teardown() for board management
//! 4. Use utility functions for UART configuration and data handling
//!
//! @section uart_fullduplex_common_configuration Configuration
//!
//! - UART module selection via macros
//! - Pin mapping for controller/device boards
//! - Buffer size and alignment settings
//! - DMA and FIFO configuration options
//!
//! This is part of revision release_sdk5p0p1-61912905f0 of the AmbiqSuite Development Package.
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
#include "uart_fullduplex_common.h"

#include <string.h>

#include "uart_stream_example_config.h"
#include "uart_example_timer_control.h"
#include "am_util.h"
#include "uart_streaming_utils.h"
#include "mpu_config.h"

#define NULL_BUFF_SIZE 64

typedef struct
{
    uint8_t ui8TxBuffer[UART_BUFFER_SIZE];
    uint8_t ui8RxBuffer[UART_BUFFER_SIZE];
    uint8_t ui8TxRxBuffer[NULL_BUFF_SIZE];
    uint8_t ui8RxTxBuffer[NULL_BUFF_SIZE];

} dma_buiffers_t;

AM_SHARED_RW dma_buiffers_t g_dma_buffers __attribute__((aligned(32)));

#define UART_MAX AM_REG_UART_NUM_MODULES

//
//! variable definitions used in this file
//
typedef struct
{
    uart_vars_t uartVars[eTrxMAX];      //!< contains info for each uart used
    uint32_t ui32UsedPinNUms[eNUM_PIN_TYPES]; //!< pin map for printing pin connection diagram
    volatile uint32_t ui32RxDmaError ;
    volatile am_hal_uart_stream_callback_data_t savedRxBuffParams;   //!< data returned in rx callback struct
    volatile bool bUartTxDone;
    volatile bool bUartRxDone;
}
uart_cmn_vars_t;

//
//! UART variables used in this file
//
static uart_cmn_vars_t uartCmnVars;

const uint8_t ui8SeedVal = 0;

//
//! UART buffer sizes for unused buffers
//
#define NULL_BUFF_SIZE 64

//
//! rxdma setup parameters
//
const am_hal_uart_rxdma_params_t RxDmaParams =
{
    .bEnableRxDMA = true,   //<! use rxdma
    .bClearFifo = true,     //!< clear data from fifo when starting rxdam
    .bEnableRxTimeout = false,  //!< do not enable rx fifo timeout
    .bForceDmaRestart = true,
};

//
//! Uart pin definitions table
//
#ifdef APOLLO510B_EVB
    static const uart_hw_defs_t uartDefs[UART_MAX] =
    {
        [0] =
     {
            .pinDescr[eTX] = {.pinNUmber = AM_BSP_GPIO_HDR_UART0_TX, .pinCfg = &g_AM_BSP_GPIO_HDR_UART0_TX, .enabled = true, .ePinType = eTX},
            .pinDescr[eCTS] = {.pinNUmber = AM_BSP_GPIO_HDR_UART0_CTS, .pinCfg = &g_AM_BSP_GPIO_HDR_UART0_CTS, .enabled = true, .ePinType = eCTS },
            .pinDescr[eRX] = {.pinNUmber = AM_BSP_GPIO_HDR_UART0_RX, .pinCfg = &g_AM_BSP_GPIO_HDR_UART0_RX, .enabled = true, .ePinType = eRX },
            .pinDescr[eRTS] = {.pinNUmber = AM_BSP_GPIO_HDR_UART0_RTS, .pinCfg = &g_AM_BSP_GPIO_HDR_UART0_RTS, .enabled = true, .ePinType = eRTS },
            .bValid  = true,
            .uartHwInstance = 0,
        },
        [1] =
     {
            .pinDescr[eTX] = {.pinNUmber = AM_BSP_GPIO_HDR_UART1_TX, .pinCfg = &g_AM_BSP_GPIO_HDR_UART1_TX, .enabled = true, .ePinType = eTX},
            .pinDescr[eCTS] = {.pinNUmber = AM_BSP_GPIO_HDR_UART1_CTS, .pinCfg = &g_AM_BSP_GPIO_HDR_UART1_CTS, .enabled = true, .ePinType = eCTS },
            .pinDescr[eRX] = {.pinNUmber = AM_BSP_GPIO_HDR_UART1_RX, .pinCfg = &g_AM_BSP_GPIO_HDR_UART1_RX, .enabled = true, .ePinType = eRX },
            .pinDescr[eRTS] = {.pinNUmber = AM_BSP_GPIO_HDR_UART1_RTS, .pinCfg = &g_AM_BSP_GPIO_HDR_UART1_RTS, .enabled = true, .ePinType = eRTS },
            .bValid  = true,
            .uartHwInstance = 1,
        },
    };

#else

static const uart_hw_defs_t uartDefs[UART_MAX] =
    {
#if defined(AM_BSP_GPIO_UART0_TX) && defined(AM_BSP_GPIO_UART0_CTS)
    [0].pinDescr[eTX] = {.pinNUmber = AM_BSP_GPIO_UART0_TX, .pinCfg = &g_AM_BSP_GPIO_UART0_TX, .enabled = true, .ePinType = eTX},
    [0].pinDescr[eCTS] = {.pinNUmber = AM_BSP_GPIO_UART0_CTS, .pinCfg = &g_AM_BSP_GPIO_UART0_CTS, .enabled = true, .ePinType = eCTS },
    [0].bValid  = true,
    [0].uartHwInstance = 0,

#endif

#if defined(AM_BSP_GPIO_UART1_TX) && defined(AM_BSP_GPIO_UART1_CTS)
    [1].pinDescr[eTX] = {.pinNUmber = AM_BSP_GPIO_UART1_TX, .pinCfg = &g_AM_BSP_GPIO_UART1_TX, .enabled = true, .ePinType = eTX },
    [1].pinDescr[eCTS] = {.pinNUmber = AM_BSP_GPIO_UART1_CTS, .pinCfg = &g_AM_BSP_GPIO_UART1_CTS, .enabled = true, .ePinType = eCTS },
    [1].bValid  = true,
    [1].uartHwInstance = 1,
#endif

#if defined(AM_BSP_GPIO_UART2_TX) && defined(AM_BSP_GPIO_UART2_CTS)
    [2].pinDescr[eTX] = {.pinNUmber = AM_BSP_GPIO_UART2_TX, .pinCfg = &g_AM_BSP_GPIO_UART2_TX, .enabled = true, .ePinType = eTX },
    [2].pinDescr[eCTS] = {.pinNUmber = AM_BSP_GPIO_UART2_CTS, .pinCfg = &g_AM_BSP_GPIO_UART2_CTS, .enabled = true, .ePinType = eCTS },
    [2].bValid  = true,
    [2].uartHwInstance = 2,
#endif

#if defined(AM_BSP_GPIO_UART3_TX) && defined(AM_BSP_GPIO_UART3_CTS)
    [3].pinDescr[eTX] = {.pinNUmber = AM_BSP_GPIO_UART3_TX, .pinCfg = &g_AM_BSP_GPIO_UART3_TX, .enabled = true, .ePinType = eTX },
    [3].pinDescr[eCTS] = {.pinNUmber = AM_BSP_GPIO_UART3_CTS, .pinCfg = &g_AM_BSP_GPIO_UART3_CTS, .enabled = true, .ePinType = eCTS },
    [3].bValid  = true,
    [3].uartHwInstance = 3,
#endif

#if (defined(AM_BSP_GPIO_UART0_FD_RX) && defined(AM_BSP_GPIO_UART0_FD_RTS))
    [0].pinDescr[eRX] = {.pinNUmber = AM_BSP_GPIO_UART0_FD_RX, .pinCfg = &g_AM_BSP_GPIO_UART0_FD_RX, .enabled = true, .ePinType = eRX },
    [0].pinDescr[eRTS] = {.pinNUmber = AM_BSP_GPIO_UART0_FD_RTS, .pinCfg = &g_AM_BSP_GPIO_UART0_FD_RTS, .enabled = true, .ePinType = eRTS },
#endif

#if defined(AM_BSP_GPIO_UART1_RX) && defined(AM_BSP_GPIO_UART1_RTS)
    [1].pinDescr[eRX] = {.pinNUmber = AM_BSP_GPIO_UART1_RX, .pinCfg = &g_AM_BSP_GPIO_UART1_RX, .enabled = true, .ePinType = eRX },
    [1].pinDescr[eRTS] = {.pinNUmber = AM_BSP_GPIO_UART1_RTS, .pinCfg = &g_AM_BSP_GPIO_UART1_RTS, .enabled = true, .ePinType = eRTS },
#endif

#if defined(AM_BSP_GPIO_UART2_RX) && defined(AM_BSP_GPIO_UART2_RTS)
    [2].pinDescr[eRX] = {.pinNUmber = AM_BSP_GPIO_UART2_RX, .pinCfg = &g_AM_BSP_GPIO_UART2_RX, .enabled = true, .ePinType = eRX },
    [2].pinDescr[eRTS] = {.pinNUmber = AM_BSP_GPIO_UART2_RTS, .pinCfg = &g_AM_BSP_GPIO_UART2_RTS, .enabled = true, .ePinType = eRTS },
#endif

#if defined(AM_BSP_GPIO_UART3_RX) && defined(AM_BSP_GPIO_UART3_RTS)
    [3].pinDescr[eRX] = {.pinNUmber = AM_BSP_GPIO_UART3_RX, .pinCfg = &g_AM_BSP_GPIO_UART3_RX, .enabled = true, .ePinType = eRX },
    [3].pinDescr[eRTS] = {.pinNUmber = AM_BSP_GPIO_UART3_RTS, .pinCfg = &g_AM_BSP_GPIO_UART3_RTS, .enabled = true, .ePinType = eRTS },
#endif
};

#endif

//
//! uart driver config parameters
//
static const am_hal_uart_config_t g_sUartConfig =
{
    //
    //! at normal speeds: Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate    = UART_BAUDRATE,
    .eDataBits    = AM_HAL_UART_DATA_BITS_8,
    .eParity      = AM_HAL_UART_PARITY_NONE,
#if UART_BAUDRATE >= 1500000
    //
    //! two stops bits works a bit better at high speeds
    //
    .eStopBits    = AM_HAL_UART_TWO_STOP_BITS,
    .eClockSrc    = AM_HAL_UART_CLOCK_SRC_SYSPLL,
#else
    .eStopBits    = AM_HAL_UART_ONE_STOP_BIT,
    .eClockSrc    = AM_HAL_UART_CLOCK_SRC_HFRC,
#endif
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
    //
    //! Set TX and RX FIFOs to trigger DMA at 1/2 full
    //
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
};

static void tx_complete_callback( am_hal_uart_stream_callback_data_t *psCBData);
static void rx_callback( am_hal_uart_stream_callback_data_t *psCBData);

//
//! Configure uart streaming setup parameters
//! This is the tx config, the rx part of this uart isn't used in this example
//
const am_hal_uart_stream_data_config_t sTxDataCfg =
{
    //
    //! specify dma mode
    //
    //
    .eStreamingDmaMode = AM_HAL_UART_DMA_TX_DOUBLE_BUFFER,

    //
    //! setup rx parameters
    //
    // need rx turned off here
    .sRxStreamConfig =
    {
        .ui32NumBytesToRead         =  32,
        .pfRxCallback               = 0,
        .ui32RxCallbackThreshold    =  32,
        .ui32TimeoutMs              = 0,
        .bClearRxFifo               = false,
        .sRxBuffer  = { .pui8Buff = g_dma_buffers.ui8TxRxBuffer, .ui32BufferSize = NULL_BUFF_SIZE  }, // use DTCM buffer
    },

    //
    //! set up tx parameters
    //
    .sTxStreamConfig =
    {
        .pfTxCallback       = tx_complete_callback,
        .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE,
        .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
        .sTxBuffer = { .pui8Buff = g_dma_buffers.ui8TxBuffer, .ui32BufferSize = UART_BUFFER_SIZE  }, // use DTCM buffer
    },
};

//
//! Configure uart streaming setup parameters
//! This is the rx config, the tx part of this uart isn't used in this example
//

const am_hal_uart_stream_data_config_t sRxDataCfg =
{
    //
    //! specify dma mode
    //
    .eStreamingDmaMode = AM_HAL_UART_DMA_RX_DOUBLE,

    //
    //! setup rx parameters
    //
    // need rx turned off here
    .sRxStreamConfig =
    {
        .ui32NumBytesToRead         =  UART_DMA_SIZE,
        .pfRxCallback               = rx_callback,
        .ui32RxCallbackThreshold    =  UART_BUFFER_SIZE,
        .ui32TimeoutMs              = 0,
        .bClearRxFifo               = false,
        .sRxBuffer  = { .pui8Buff = g_dma_buffers.ui8RxBuffer, .ui32BufferSize = UART_BUFFER_SIZE  }, // use DTCM buffer
    },

    //
    //! set up tx parameters
    //
    .sTxStreamConfig =
    {
        .pfTxCallback       = NULL,
        .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE,
        .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
        .sTxBuffer = { .pui8Buff = g_dma_buffers.ui8RxTxBuffer, .ui32BufferSize = NULL_BUFF_SIZE  }, // use DTCM buffer
    },
} ;


static uint32_t uart_print_pin_config(void);
static uint32_t uart_fd_test_test_pkt(uint8_t seed, uint8_t *p, uint32_t ui32Len);
static void uart_fd_generate_test_pkt(uint8_t seed, uint8_t *p, uint32_t ui32Len);

//*****************************************************************************
//
//! @brief called from uart ISR for rx errors and completion
//!
//! @note this is called from an ISR
//!
//! @param psCBData  pointer to stack variable populated in ISR
//
//*****************************************************************************
#define RX_DMA_ERROR_MASK (AM_HAL_UART_RX_DMA_ERROR | AM_HAL_UART_RX_DMA_TIMEOUT | AM_HAL_UART_RX_DMA_OVERFLOW | AM_HAL_UART_STREAM_DMA_CFG_ERROR)
static void
rx_callback( am_hal_uart_stream_callback_data_t *psCBData)
{
    uartCmnVars.savedRxBuffParams = *psCBData;  // copy params from the RX part of ISR, this is a struct copy
    *(psCBData->hasData) = 0;                   // clear data flag, this should be done in the background

    am_hal_uart_stream_status_t eStatus = uartCmnVars.savedRxBuffParams.eStatus;

    if (eStatus & AM_HAL_UART_RX_DMA_COMPLETE )
    {
        uartCmnVars.bUartRxDone = true;             // signal background process/loop
    }
    if (eStatus & RX_DMA_ERROR_MASK)
    {
        uartCmnVars.ui32RxDmaError |= eStatus;
    }

    __DMB();
}
//*****************************************************************************
//
//! @brief called from uart ISR for tx errors and completion
//!
//! @note this is called from an ISR
//!
//! @param psCBData  pointer to stack variable populated in ISR
//
//*****************************************************************************
static void
tx_complete_callback( am_hal_uart_stream_callback_data_t *psCBData)
{
    uartCmnVars.bUartTxDone = true;
}

//*****************************************************************************
//
// Sets up the UART configuration for full-duplex communication.
//
//
//*****************************************************************************
uint32_t
fd_common_setup(void)
{
    if ( TEST_UART_TX_MODULE == TEST_UART_RX_MODULE)
    {
        am_util_stdio_printf("ERROR: Rx and TX uarts are the same: %d\r\n", TEST_UART_TX_MODULE);
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    // set up the isr service
    set_uart_tx_rx_params( &uartCmnVars.uartVars[eTrxTypeTx], &uartCmnVars.uartVars[eTrxTypeRx] );

    uint32_t *pui32Pins = uartCmnVars.ui32UsedPinNUms;
    for ( uint32_t i = 0; i < eNUM_PIN_TYPES; i++)
    {
        *pui32Pins++ = 0x7FFFFFFF;
    }
    // setup rx and tx
    uint32_t ui32Status = init_uart(eTrxTypeRx, TEST_UART_RX_MODULE );
    if (ui32Status)
    {
        am_util_stdio_printf("ERROR: Rx config: am_hal_uart_stream_configure error: %d\r\n", ui32Status );
        return ui32Status;
    }

    ui32Status = init_uart(eTrxTypeTx, TEST_UART_TX_MODULE );
    if (ui32Status)
    {
        am_util_stdio_printf("ERROR: Tx config: am_hal_uart_stream_configure error: %d\r\n", ui32Status );
    }

    uart_print_pin_config() ;

    return ui32Status;
}

//*****************************************************************************
//
// Initializes the MCU for low-power operation, configures clocks, and enables debugging interfaces.
//
//
//*****************************************************************************
uint32_t
mcu_setup(void)
{
    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Enable the ITM print interface.
    //
    uint32_t ui32Status =  (uint32_t) am_bsp_itm_printf_enable();
    if (ui32Status)
    {
        return ui32Status;
    }

#ifndef AM_PART_APOLLO330P_510L
    //
    // Enable HFADJ
    //
    ui32Status = am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFADJ_ENABLE, 0);
    if (ui32Status)
    {
        am_util_stdio_printf(" am_hal_clkgen_control error: %d\n", ui32Status);
        return ui32Status;
    }
    if ( g_sUartConfig.eClockSrc == AM_HAL_UART_CLOCK_SRC_SYSPLL)
    {
        //
        // Enable SYSPLL.
        //
        ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_SYSPLL, AM_HAL_UART_PLLCLK_FREQ, NULL);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
            am_util_stdio_printf("ERROR: am_hal_clkmgr_clock_config: %d\n", ui32Status);
            return ui32Status;
        }
    }
#else

    //
    // Set XTAL_HS as highest priority for SYSPLL FREF.
    //
    am_hal_clkmgr_syspll_fref_priority_t sSysPllFrefPrio =
    {
        .high = AM_HAL_SYSPLL_FREFSEL_XTAL48MHz,
        .mid = AM_HAL_SYSPLL_FREFSEL_EXTREFCLK,
        .low = AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4,
    };
    am_hal_clkmgr_control(AM_HAL_CLKMGR_SYPLL_FREF_PRIORITY_SET, (void*)&sSysPllFrefPrio);

    //
    am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLVCO, 0, NULL);
    ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, AM_HAL_UART_PLLCLK_FREQ, NULL);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf(" PLLPOSTDIV clock source error: %d\n", ui32Status);
        return ui32Status;
    }
#endif  // AM_PART_APOLLO330P_510L

    //
    // turn off cache for DMA buffs
    //

    ui32Status = uart_stream_cmn_mpuConfig( &g_dma_buffers, sizeof(g_dma_buffers));
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("ERROR: Mpu Config Error %d\n", ui32Status);
        return ui32Status;
    }

    return ui32Status;
}

//*****************************************************************************
//
//! @brief Prints the UART pin configuration and related information
//! @details This includes baud rate,
//! transmit (TX) and receive (RX) pin details, and optional RTS/CTS handshake information.
//! Displays pin jumper mapping between controller and device, power sequencing,
//! and compiler compatibility note.
//!
//! @return AM_HAL_STATUS_SUCCESS upon completion.
//
//*****************************************************************************
static uint32_t
uart_print_pin_config(void)
{
    uint32_t *uart_pin_info = uartCmnVars.ui32UsedPinNUms;

    am_util_stdio_printf("The Baud Rate is: %d, tx is %d, rx is %d,  %d n\n",
        g_sUartConfig.ui32BaudRate,
        uartCmnVars.uartVars[eTrxTypeTx].psUartHwDefs->uartHwInstance,
        uartCmnVars.uartVars[eTrxTypeRx].psUartHwDefs->uartHwInstance, (eTX << 8) | eRX);

    am_util_stdio_printf("The pin jumpers are as follows:\n...(assuming both controller and device are the same EVBs):\n");
    am_util_stdio_printf("\n");
    am_util_stdio_printf("\t\t+--------------------+                    +--------------------+\n");
    am_util_stdio_printf("\t\t|Board#1 (CONTROLLER)|                    |   Board#2 (DEVICE) |\n");
    am_util_stdio_printf("\t\t|       UART         |                    |       UART         |\n");
    am_util_stdio_printf("\t\t|--------------------|                    |--------------------|\n");
    am_util_stdio_printf("\t\t|   %3d (TX)         |--------TX----->----|   %3d (RX)         |\n", uart_pin_info[eTX], uart_pin_info[eRX]);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
    am_util_stdio_printf("\t\t|   %3d (RX)         |----<---RX----------|   %3d (TX)         |\n", uart_pin_info[eRX], uart_pin_info[eTX]);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
   if (uart_pin_info[eRTS] < 0xFFFF && uart_pin_info[eCTS] < 0xFFFF )
   {
    am_util_stdio_printf("\t\t|   %3d (RTS)        |--------RTS---->----|   %3d (CTS)        |\n", uart_pin_info[eRTS], uart_pin_info[eCTS]);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
    am_util_stdio_printf("\t\t|   %3d (CTS)        |----<---CTS---------|   %3d (RTS)        |\n", uart_pin_info[eCTS], uart_pin_info[eRTS]);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
   }
    am_util_stdio_printf("\t\t|   %3d (INT)        |----<---HANDSHAKE---|   %3d (HANDSHAKE)  |\n", UART_DEVICE_READY_OUT_PIN , UART_DEVICE_READY_OUT_PIN);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
    am_util_stdio_printf("\t\t|       GND          |-------GND----->----|       GND          |\n");
    am_util_stdio_printf("\t\t+--------------------+                    +--------------------+\n");
    am_util_stdio_printf("\n");
    am_util_stdio_printf("Power Sequence for Board#1 (CONTROLLER) and Board#2 (DEVICE):\n");
    am_util_stdio_printf("\tStep 1: Reset Board#1 (CONTROLLER) \n");
    am_util_stdio_printf("\tStep 2: Reset Board#2 (DEVICE) \n");
    am_util_stdio_printf("\n");
    am_util_stdio_printf("Note 1: Controller and device need to use the same compiler %s\n", COMPILER_VERSION);
    am_util_stdio_printf("\n");

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// Initializes a UART instance for the specified transfer type (RX or TX).
//
//*****************************************************************************
uint32_t
init_uart( transfer_type_t eTxType, uint32_t uartInstance )
{
    //uart_fd_cmn_module_init() ;

    if ( uartInstance >= UART_MAX )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    if ( eTxType >= eTrxMAX )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }
    const uart_hw_defs_t *uartHWdefs = &uartDefs[uartInstance];
    if ( !uartHWdefs->bValid )
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    uart_vars_t *pUv = &uartCmnVars.uartVars[eTxType];
    pUv->eTransferType = eTxType;
    pUv->pUartHandle   = 0;
    pUv->psUartHwDefs  = uartHWdefs;
    if ( eTxType == eTrxTypeRx )
    {
        pUv->psUartConfig = &g_sUartConfig ;
        pUv->psUartStreamDataConfig = &sRxDataCfg ;
    }
    else
    {
        pUv->psUartConfig = &g_sUartConfig ;
        pUv->psUartStreamDataConfig = &sTxDataCfg ;
    }

    return uart_fd_init( pUv ) ;
}

//*****************************************************************************
//
// Initializes the UART hardware with the appropriate configuration based on the provided UART variables.
//
//*****************************************************************************
uint32_t
uart_fd_init( uart_vars_t *pUartVar )
{
    //
    // init the gpio
    //
    pin_type_e ePinType = pUartVar->eTransferType == eTrxTypeTx ? eTX : eRX;
    const uart_pin_descr_t *pPinDesc = &pUartVar->psUartHwDefs->pinDescr[ePinType];
    if ( !pPinDesc->enabled )
    {
        return AM_HAL_STATUS_INVALID_OPERATION;
    }

    uint32_t uiStatus = am_hal_gpio_pinconfig(pPinDesc->pinNUmber, *(pPinDesc->pinCfg));
    if (uiStatus)
    {
        am_util_stdio_printf("pin config error: %d %d\r\n", pPinDesc->pinNUmber,  uiStatus );
        return uiStatus;
    }
    uartCmnVars.ui32UsedPinNUms[ePinType] = pPinDesc->pinNUmber;

    uiStatus = am_hal_uart_stream_initialize(pUartVar->psUartHwDefs->uartHwInstance, &pUartVar->pUartHandle);
    if (uiStatus)
    {
        am_util_stdio_printf("am_hal_uart_stream_initialize error: %d\r\n", uiStatus );
        return uiStatus;
    }
    uiStatus = am_hal_uart_stream_power_control(pUartVar->pUartHandle, AM_HAL_SYSCTRL_WAKE, false);
    if (uiStatus)
    {
        am_util_stdio_printf("am_hal_uart_stream_power_control error: %d\r\n", uiStatus );
        return uiStatus;
    }
    //! Config the low-level uart params (baud rate, stop bits)
    uiStatus = am_hal_uart_stream_configure( pUartVar->pUartHandle, pUartVar->psUartConfig );
    if (uiStatus)
    {
        am_util_stdio_printf("am_hal_uart_stream_configure error: %d\r\n", uiStatus );
        return uiStatus;
    }
    am_hal_uart_errors_t uiStreamStatus = am_hal_uart_stream_data_configure(pUartVar->pUartHandle, pUartVar->psUartStreamDataConfig);
    if (uiStreamStatus)
    {
        const char *str = printErrorString(uiStreamStatus, false);
        am_util_stdio_printf("am_hal_uart_stream_data_configure error: %s %x\r\n", str, uiStreamStatus);
        uiStatus = (uint32_t) uiStreamStatus;
        return uiStatus;
    }
    {
        //
        // no errors above, enable the uart ISR, elevate the priority for high-speed testing
        //
        IRQn_Type eIrqType = (IRQn_Type) (pUartVar->psUartHwDefs->uartHwInstance + UART0_IRQn);

        NVIC_ClearPendingIRQ(eIrqType);
        //
        // raise the uart priority above normal
        //
        uint32_t ui32InterruptPriority = (AM_IRQ_PRIORITY_DEFAULT > 1)? (AM_IRQ_PRIORITY_DEFAULT-1) : AM_IRQ_PRIORITY_DEFAULT;
        NVIC_SetPriority(eIrqType, ui32InterruptPriority);
        NVIC_EnableIRQ(eIrqType);
    }

    //
    // at this point rx and/or tx are ready to run, but not transmitting
    //
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Validates a test packet for UART by comparing its contents against a sequence
//! generated from a given seed. Counts the number of mismatched bytes.
//!
//! @param seed The initial value used to generate the sequence to compare against.
//! @param p Pointer to the buffer containing the test packet data.
//! @param ui32Len The length of the buffer to process. Note that this method processes
//!                ui32Len - 1 bytes.
//!
//! @return The number of errors (mismatches) found in the test packet.
//
//*****************************************************************************
static uint32_t
uart_fd_test_test_pkt(uint8_t seed, uint8_t *p, uint32_t ui32Len)
{
    //
    // compare input and sent. ui32Len includes final checksum byte.
    //
    uint32_t errorCount = 0;
    uint8_t checksum = 0;
    uint8_t val = seed;

    //
    // validate data bytes
    //
    {
        uint8_t *q = p; // use local pointer so we don't change the caller's pointer
        for ( uint32_t i = 0; i < ui32Len - 1; i++ )
        {
            uint8_t bb = *q++;
            if ( bb != val )
            {
                //
                // print index and expected/actual to aid debugging
                //
                am_util_stdio_printf("ERROR: data mismatch at index %u: got 0x%02X expected 0x%02X\n", i, bb, val);
                errorCount++;
            }
            checksum += bb;
            val++ ;
        }
    }

    //
    // validate checksum byte
    //
    uint8_t recv_crc = p[ui32Len - 1];
    if ( recv_crc != checksum )
    {
        am_util_stdio_printf("ERROR: checksum mismatch: got 0x%02X expected 0x%02X\n", recv_crc, checksum);
        errorCount++;
    }

    return errorCount;
}

//*****************************************************************************
//
//! @brief Generate a test packet for UART transmission.
//!
//! The function creates a test packet based on a given seed value by filling the buffer
//! with incrementing values starting from the seed. The last byte in the buffer is set as
//! the checksum, which is the sum of all preceding bytes.
//!
//! @param seed The initial value to start generating the packet data.
//! @param p Pointer to the buffer where the generated packet will be stored.
//! @param ui32Len Length of the packet to be generated, including the checksum byte.
 //
 //*****************************************************************************
static void
uart_fd_generate_test_pkt(uint8_t seed, uint8_t *p, uint32_t ui32Len)
{
    // Generate tx data
    uint8_t checksum = 0;
    uint8_t val = seed;
    for ( uint32_t i = 0; i < ui32Len - 1; i++ )
    {
        *p++ = val;
        checksum += val;
        val++ ;
    }
    *p = checksum;
}

//
//! compute a farily unforgiving timeout for the DMA transfer
//
const uint64_t delay_numerator_usec = UART_DMA_SIZE * 16ull * 1000000ull;

//*****************************************************************************
//
// Executes a full-duplex UART test, which involves transmitting and receiving data
// using UART DMA and verifying the data integrity.
//
//*****************************************************************************
int32_t
uart_run_fullduplex_test(void)
{
    static bool rxDmaStarted = false;
    uint8_t *txBuffer = g_dma_buffers.ui8TxBuffer;

    // get rxDma started
    if ( !rxDmaStarted )
    {
        //
        //! using ping pong double buffer, only need to start this once
        //
        am_hal_uart_errors_t eRxDmaStatus = am_hal_uart_stream_enable_rxDmaParms(uartCmnVars.uartVars[eTrxTypeRx].pUartHandle,
                                                                                 &RxDmaParams);

        if ( eRxDmaStatus != AM_HAL_UART_STATUS_SUCCESS )
        {
            printErrorString( eRxDmaStatus, true);
            return AM_HAL_STATUS_FAIL;
        }
        rxDmaStarted = true;
    }
    // now send the tx data

    uart_fd_generate_test_pkt( ui8SeedVal, txBuffer, UART_DMA_SIZE );

#ifdef DEBUG
    //
    // Debug: print the last two data bytes and checksum to help diagnose mismatches
    //
    if (UART_DMA_SIZE >= 2)
    {
        uint32_t idx_last = UART_DMA_SIZE - 1;
        uint32_t idx_prev = UART_DMA_SIZE - 2;
        am_util_stdio_printf("DEBUG: tx last data byte idx %u = 0x%02X, prev idx %u = 0x%02X, checksum idx %u = 0x%02X\n",
            idx_prev, txBuffer[idx_prev], idx_last - 1, txBuffer[idx_last - 1], idx_last, txBuffer[idx_last]);
    }
#endif

    am_hal_uart_errors_t eTxDmaStatus = am_hal_uart_append_tx_double(uartCmnVars.uartVars[eTrxTypeTx].pUartHandle,
                                            txBuffer, UART_DMA_SIZE);

    if ( eTxDmaStatus != AM_HAL_UART_STATUS_SUCCESS )
    {
        printErrorString( eTxDmaStatus, true);
        return AM_HAL_STATUS_FAIL;
    }

    uint32_t delay_usec = (uint32_t) (delay_numerator_usec / (uint64_t) g_sUartConfig.ui32BaudRate) ;
    if ( delay_usec < 400 )
    {
        delay_usec = 400;
    }
    uart_timer_init( delay_usec );

    // start time
    // wait for the rx data

    bool timeout = false;

    while ( !(uartCmnVars.bUartRxDone && uartCmnVars.bUartTxDone) )
    {
        // wait for a timeout
        timeout = uart_timer_isExpired() ;
        if ( timeout )
        {
            break ;
        }
    }

    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    if ( timeout )
    {
        am_util_stdio_printf("ERROR: timeout\n");

        if ( !uartCmnVars.bUartRxDone )
        {
            am_util_stdio_printf("ERROR: timeout waiting for RxDma\n");
        }

        if ( !uartCmnVars.bUartTxDone )
        {
            am_util_stdio_printf("ERROR: timeout waiting for TxDma\n");
        }
        ui32Status = AM_HAL_STATUS_FAIL;
    }
    else
    {
        if ( uartCmnVars.savedRxBuffParams.eStatus & AM_HAL_UART_RX_DMA_COMPLETE)
        {
            // process incoming data
            uint32_t ui32BytesToRead    = uartCmnVars.savedRxBuffParams.ui32BufferSize;
            if ( ui32BytesToRead != UART_DMA_SIZE)
            {
                am_util_stdio_printf("ERROR: invalid rx size: %d, expected %d\r\n", ui32BytesToRead, UART_DMA_SIZE);
                ui32Status = AM_HAL_STATUS_FAIL;
            }
            else
            {
                uint32_t ui32NumErrors = uart_fd_test_test_pkt(ui8SeedVal, uartCmnVars.savedRxBuffParams.pui8Buffer, UART_DMA_SIZE );
                memset( uartCmnVars.savedRxBuffParams.pui8Buffer, 0, UART_DMA_SIZE );
                if (ui32NumErrors)
                {
                    am_util_stdio_printf("ERROR: invalid rx buff contents: %d\r\n", ui32NumErrors);
                    ui32Status = AM_HAL_STATUS_FAIL;
                }
            }
        }
        else
        {
            am_util_stdio_printf("ERROR: RX DMA not Complete rx buff contents: 0x%08X\n", uartCmnVars.savedRxBuffParams.eStatus);
            ui32Status = AM_HAL_STATUS_FAIL;
        }
    }

    if ( uartCmnVars.ui32RxDmaError )
    {
        uint32_t ui32RxDMAERR = uartCmnVars.ui32RxDmaError;
        am_util_stdio_printf("ERROR: RX DMA ERROR: 0x%08X\n", ui32RxDMAERR);
    }

    AM_CRITICAL_BEGIN
    uartCmnVars.ui32RxDmaError = 0 ;
    uartCmnVars.bUartRxDone = false;
    uartCmnVars.bUartTxDone = false;

    uartCmnVars.savedRxBuffParams.eStatus = AM_HAL_UART_STREAM_STATUS_SUCCESS;
    uartCmnVars.savedRxBuffParams.ui32BufferSize = 0;
    AM_CRITICAL_END


    return ui32Status;
}

//*****************************************************************************
//
// disconnects uart
//
//*****************************************************************************
void common_teardown(void)
{
    //
    // Shut everything back down.
    //
    am_hal_uart_stream_power_control(uartCmnVars.uartVars[eTrxTypeTx].pUartHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    am_hal_uart_stream_power_control(uartCmnVars.uartVars[eTrxTypeRx].pUartHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false);
}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
