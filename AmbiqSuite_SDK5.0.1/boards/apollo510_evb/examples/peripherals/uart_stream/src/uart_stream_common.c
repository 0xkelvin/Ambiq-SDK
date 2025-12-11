//*****************************************************************************
//
//! @file uart_stream_common.c
//!
//! @brief Common code and definitions for uart_stream example
//!
//! @addtogroup uart_stream
//! @{
//! @defgroup uart_stream_common_c UART Streaming Common Example
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
#include "uart_stream_common.h"
#include "am_util.h"

//
//! Uart pin definitions
//! This defaults with standard BSP pin definitions.
//
static const uart_hw_defs_t gs_UartPinDefs[eMAX_UARTS] =
{
#if UART0_AVAIL
    [eUART0] =
    {
    .uartId = eUART0,
    .rxPin = {AM_BSP_GPIO_COM_UART_RX, &g_AM_BSP_GPIO_COM_UART_RX},
    .txPin = {AM_BSP_GPIO_COM_UART_TX, &g_AM_BSP_GPIO_COM_UART_TX},
    .uartHwIndex = 0,

    },
#endif
#if UART1_AVAIL
    [eUART1] =
    {
        .uartId = eUART1,
        .rxPin = {AM_BSP_GPIO_UART1_RX, &g_AM_BSP_GPIO_UART1_RX},
        .txPin = {AM_BSP_GPIO_UART1_TX, &g_AM_BSP_GPIO_UART1_TX},
        .uartHwIndex = 1,
    },
#endif
#if UART2_AVAIL
    [eUART2] =
    {
        .uartId = eUART2,
        .rxPin = {AM_BSP_GPIO_UART2_RX, &g_AM_BSP_GPIO_UART2_RX},
        .txPin = {AM_BSP_GPIO_UART2_TX, &g_AM_BSP_GPIO_UART2_TX},
        .uartHwIndex = 2,
    },
#endif
#if UART3_AVAIL
    [eUART3] =
    {
        .uartId = eUART3,
        .rxPin = {AM_BSP_GPIO_UART3_RX, &g_AM_BSP_GPIO_UART3_RX},
        .txPin = {AM_BSP_GPIO_UART3_TX, &g_AM_BSP_GPIO_UART3_TX},
        .uartHwIndex = 3,
    },
#endif
};

//
//! uart error capture struct
//
typedef struct
{
    uint32_t loopCountErr;
    uint32_t posx;
    am_hal_uart_errors_t lastError;
}
error_cap_t;

//
//! this is to look at with debugger if printing is disabled
//
volatile error_cap_t ec;

static const char * const uartStreamingMode[] = {
    "DMA_NONE",
    "TX_DOUBLE_BUFFER",
    "TX_SINGLE_BUFFER",
    "RX_SINGLE",
    "RX_DOUBLE",
};

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

static const stream_error_message_action_map_t sStreamErrorMessageDefaultMap[] =
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

static uart_examp_stream_vars_t *g_uartIsrMap[4];  //<! one for each uart
//
//! Place DMA ram in SRAM and align the region for better MPU access
//! The mpu will be used to disable the cache in this region
//
__attribute__ ((section ("SHARED_RW")))
__attribute__((aligned(32))) uart_dma_rx_tx_DMA_buffers_t g_dma_buff[NUM_UARTS_USED];

void am_uart_isr(void);
void am_uart1_isr(void);
void am_uart2_isr(void);
void am_uart3_isr(void);
static void am_isr_common(uart_examp_stream_vars_t *pUartDescriptor);


static uint32_t uart_cmn_buffAssignmentAssistant(
    am_hal_uart_stream_buff_t  *psDestBuffer,
    const am_hal_uart_stream_buff_t  *psSrcBuffer,
    uint32_t ui32MaxBufferSize,
    uint8_t *pui8DmaBuffer);

//*****************************************************************************
//
//! @brief Uart isr handlers
//! @note For example purposes capture all uart ISRs here and process them in the same way.
//
//*****************************************************************************
void am_uart_isr(void)
{
#if UART0_AVAIL
    am_isr_common(g_uartIsrMap[0]);
#endif
}

void am_uart1_isr(void)
{
#if UART1_AVAIL
    am_isr_common(g_uartIsrMap[1]);
#endif
}

void am_uart2_isr(void)
{
#if UART2_AVAIL
    am_isr_common(g_uartIsrMap[2]);
#endif
}

void am_uart3_isr(void)
{
#if UART3_AVAIL
    am_isr_common(g_uartIsrMap[3]);
#endif
}

//*****************************************************************************
//
//! @brief Common uart isr handler
//!
//! @param pUartDescriptor
//
//*****************************************************************************
static void am_isr_common(uart_examp_stream_vars_t *pUartDescriptor)
{
    pUartDescriptor->e32Status |= am_hal_uart_interrupt_stream_service(pUartDescriptor->pUartHandle);

    if (pUartDescriptor->e32Status != AM_HAL_UART_STREAM_STATUS_SUCCESS)
    {
        pUartDescriptor->ui32ISrErrorCount++;
    }
}

//*****************************************************************************
//
// called to configure MPU for dma data
//
//*****************************************************************************
uint32_t uart_stream_cmn_mpuConfig(void)
{
    uint32_t dmaBufAddr = (uint32_t) g_dma_buff;

    if ( dmaBufAddr >= AM_HAL_MRAM_SRAM_ADDR)
    {
        // want region set at non-cacheable if it is in SRAM
        // is data already non-cacheable if it's in DTCM?
        return mpuConfig( dmaBufAddr, sizeof(g_dma_buff) );
    }

    return AM_HAL_STATUS_SUCCESS;
}

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
//  will check and log run-time errors
//
//*****************************************************************************
void errorCheck(am_hal_uart_errors_t eErrx, uint32_t ui32LoopCount, uint32_t ui32ErrorPos)
{
    //
    // if there is an error, then print an error message and hang
    //
    if (eErrx)
    {
        //
        // Extract the error string
        //
        const char *pcErrorInfoStr = printErrorString(eErrx, false);

        ec.lastError    = eErrx;
        ec.loopCountErr = ui32LoopCount;
        ec.posx = ui32ErrorPos;
        am_util_stdio_printf("error capture: %s 0x%08x %d %d\r\n Exampling hanging\n", pcErrorInfoStr, eErrx, ui32LoopCount, ui32ErrorPos);
        while (1);
    }
}

//*****************************************************************************
//
//! @brief Initialize a UART instance
//!
//! @param uartId  the uart number (0<->(N-1))
//! @param pUartLocalVar ram allocated for this uart
//!
//! @return standard hal status
//
//*****************************************************************************
static uint32_t init_uart(uart_examp_stream_vars_t *psUartInfo,
                          am_hal_uart_stream_data_config_t *psDataFcg)
{
    //
    // get and check the uart number (not always identical  to the hardware instance in this example).
    //
    uart_id_e uartId = psUartInfo->uartId;

    if (uartId >= eMAX_UARTS)
    {
        return AM_HAL_STATUS_INVALID_ARG;
    }

    if ( psDataFcg->eStreamingDmaMode >= AM_HAL_UART_DMA_MODE_ENTRIES )
    {
        am_util_stdio_printf("invalid streaming mode: %d\r\n", psDataFcg->eStreamingDmaMode );
        return AM_HAL_STATUS_INVALID_OPERATION;
    }
    //
    // print the streaming mode
    //
    const char *pstr = uartStreamingMode[psDataFcg->eStreamingDmaMode];
    am_util_stdio_printf("The streaming mode is: %s\r\n", pstr);

    //
    // get the uart hardware instance
    // This will ensure the uart ID is mapped to the correct set of uart data
    //
    const uart_hw_defs_t *pUartDefs = psUartInfo->psUartPinDefs;
    g_uartIsrMap[pUartDefs->uartHwIndex] = psUartInfo;

    uint32_t uiStatus;
    do
    {
        //
        // setup rx and tx pins
        //
        uiStatus = am_hal_gpio_pinconfig(pUartDefs->txPin.pinNUmber, *(pUartDefs->txPin.pinCfg));
        if (uiStatus)
        {
            am_util_stdio_printf("tx ping config error: %d\r\n", uiStatus );
            break;
        }
        uiStatus = am_hal_gpio_pinconfig(pUartDefs->rxPin.pinNUmber, *(pUartDefs->rxPin.pinCfg));
        if (uiStatus)
        {
            am_util_stdio_printf("rx ping config error: %d\r\n", uiStatus );
            break;
        }

        am_util_stdio_printf("rx and tx pin cfg success: rx: %d tx: %d\r\n",
            pUartDefs->rxPin.pinNUmber, pUartDefs->txPin.pinNUmber);

        //
        // init the uart
        // first get the uart handle
        //
        uiStatus = am_hal_uart_stream_initialize(pUartDefs->uartHwIndex, &psUartInfo->pUartHandle);
        if (uiStatus)
        {
            am_util_stdio_printf("am_hal_uart_stream_initialize error: %d\r\n", uiStatus );
            break;
        }
        void *pUartHandle = psUartInfo->pUartHandle;

        uiStatus = am_hal_uart_stream_power_control(pUartHandle, AM_HAL_SYSCTRL_WAKE, false);
        if (uiStatus)
        {
            am_util_stdio_printf("am_hal_uart_stream_power_control error: %d\r\n", uiStatus );
            break;
        }

        //! Config the low-level uart params (baud rate, stop bits)
        uiStatus = am_hal_uart_stream_configure( pUartHandle, psUartInfo->psUartConfig);
        if (uiStatus)
        {
            am_util_stdio_printf("am_hal_uart_stream_configure error: %d\r\n", uiStatus );
            break;
        }

        //
        // Configure the uart data
        //
        am_hal_uart_errors_t uiStreamStatus = am_hal_uart_stream_data_configure(pUartHandle, psDataFcg);
        if (uiStreamStatus)
        {
            const char *str = printErrorString(uiStreamStatus, false);
            am_util_stdio_printf("am_hal_uart_stream_data_configure error: %s %x\r\n", str, uiStreamStatus);
            uiStatus = (uint32_t) uiStreamStatus;
            break;
        }

        //
        // Make sure to enable the interrupts for RX, since the HAL doesn't already
        // know we intend to use them., this is done later in the startup sequence
        //
    }
    while( false );

    if (uiStatus == AM_HAL_STATUS_SUCCESS)
    {
        //
        // no errors above, enable the uart ISR, elevate the priority for high-speed testing
        //
        IRQn_Type eIrqType = (IRQn_Type) (pUartDefs->uartHwIndex + UART0_IRQn);

        NVIC_ClearPendingIRQ(eIrqType);
        //
        // raise the uart priority above normal
        //
        uint32_t ui32InterruptPriority = (AM_IRQ_PRIORITY_DEFAULT > 1)? (AM_IRQ_PRIORITY_DEFAULT-1) : AM_IRQ_PRIORITY_DEFAULT;
        NVIC_SetPriority(eIrqType, ui32InterruptPriority);
        NVIC_EnableIRQ(eIrqType);
    }

    return uiStatus;
}

//*****************************************************************************
//
//! @brief Computes buffer assignments based on inputs
//!
//! if source buffer isn't provided (null) the dma buffer is used
//!
//! @param psDestBuffer pointer to dest buffer struct
//! @param psSrcBuffer pointer to src buffer struct
//! @param ui32MaxBufferSize
//! @param pui8DmaBuffer  pointer to dma buffer
//!
//! @return  standard hal status (success)
//
//*****************************************************************************
static uint32_t uart_cmn_buffAssignmentAssistant(
    am_hal_uart_stream_buff_t  *psDestBuffer,
    const am_hal_uart_stream_buff_t  *psSrcBuffer,
    uint32_t ui32MaxBufferSize,
    uint8_t *pui8DmaBuffer)
{
    uint32_t ui32SourceBufferSize = psSrcBuffer->ui32BufferSize;
    psDestBuffer->ui32BufferSize = ui32SourceBufferSize;

    //
    // if the user supplied pointer is null, then the
    // SRAM buffer will be used (the region with cache disabled)
    //
    if (psSrcBuffer->pui8Buff)
    {
        psDestBuffer->pui8Buff = psSrcBuffer->pui8Buff;
    }
    else
    {
        psDestBuffer->pui8Buff = pui8DmaBuffer;
        if ((ui32SourceBufferSize == 0) || (ui32SourceBufferSize > ui32MaxBufferSize))
        {
            psDestBuffer->ui32BufferSize = ui32MaxBufferSize;
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief Initialize the serial module and associated tx and tx pins
//!
//! @param pUartDesc  pointer to local struct that holds uart params
//! @param psRxStreamConfig pointer to const struct that defines rx params
//! @param sTxStreamConfig  pointer to const struct that defines tx params
//!
//! @return standard hal status
//
//*****************************************************************************
uint32_t
uart_cmn_serial_interface_init(uart_examp_stream_vars_t *pUartDesc,
                               const am_hal_uart_stream_rx_config_t *psRxStreamConfig,
                               const am_hal_uart_stream_tx_config_t *sTxStreamConfig )
{
    uint8_t uartHwInstance = pUartDesc->uart_hardware_instance;
    for (uart_id_e uartId = (uart_id_e) 0; uartId< eMAX_UARTS; uartId++)
    {
        if (gs_UartPinDefs[uartId].uartHwIndex == uartHwInstance)
        {
            pUartDesc->uartId = uartId;
        }
    }

    //
    // Start the UART that is enabled here
    // if fifo buffers aren't provided use the dma buffers
    //

    am_hal_uart_stream_data_config_t sDataCfg;

    sDataCfg.eStreamingDmaMode = pUartDesc->euartEaxampleStreamMode;

    sDataCfg.sRxStreamConfig = *psRxStreamConfig;
    sDataCfg.sTxStreamConfig = *sTxStreamConfig;

    uint32_t uartIndexOffset = 0;      // This is the only uart used, the index is zero, this is not the uart hardware number

    //
    // assign uart tx and rx buffers
    //
    uart_cmn_buffAssignmentAssistant( &sDataCfg.sRxStreamConfig.sRxBuffer,
        &psRxStreamConfig->sRxBuffer,
        RX_DMA_BUFFER_SIZE,
        g_dma_buff[uartIndexOffset].pui8UARTRXDMABuffer);

    uart_cmn_buffAssignmentAssistant( &sDataCfg.sTxStreamConfig.sTxBuffer,
        &sTxStreamConfig->sTxBuffer,
        TX_DMA_BUFFER_SIZE,
        g_dma_buff[uartIndexOffset].pui8UARTTXDMABuffer);

    if (pUartDesc->pfTimerCallback)
    {
        //
        // if the function is defined, the timed callback will be used
        //
        uint32_t ui32Status = uart_timer_init( &pUartDesc->sUartTimerParams, pUartDesc->pfTimerCallback, true);
        if ( ui32Status )
        {
            am_util_stdio_printf("%invalid periodic timer config %d\nThe program will hang\n", ui32Status);
            while(1);
        }
    }

    //
    // get and save the pointer to the uart pin defs for this uart
    //
    pUartDesc->psUartPinDefs = &gs_UartPinDefs[pUartDesc->uartId];

    uint32_t ui32status      = init_uart(pUartDesc, &sDataCfg);

    return ui32status;

} //serial_interface_init

//*****************************************************************************
//
// MPU config. Configure DMA region for as non-cacheable
//
//*****************************************************************************
uint32_t mpuConfig(uint32_t ui32BaseAddress, uint32_t ui32DmaSize)
{
    ARM_MPU_Disable();

    //
    // Randomly choose an MPU slot, sometimes 0,1 are used for other things
    //
    uint8_t ui32AttrIndex = 3;
    uint8_t ui32CachePolicy = (ARM_MPU_ATTR_NON_CACHEABLE << 4) | ARM_MPU_ATTR_NON_CACHEABLE;

    //
    // define mpu attributes for no cache
    //
    ARM_MPU_SetMemAttr(ui32AttrIndex, ui32CachePolicy);

    uint32_t endAddr     = ui32BaseAddress + ui32DmaSize -1;

    //
    // define MPU region
    //
    am_hal_mpu_region_config_t mpuRegion =
    {
        .ui32RegionNumber = 3,             // 3 is a random choice, has nothing to do with ui32AttrIndex
        .ui32BaseAddress  = ui32BaseAddress,
        .eShareable = ARM_MPU_SH_INNER,
        .eAccessPermission = RW_NONPRIV,
        .bExecuteNever = true,
        .ui32LimitAddress = endAddr,
        .ui32AttrIndex = ui32AttrIndex,
        .bEnable = true,
    };

    uint32_t  ui32Status = am_hal_mpu_region_configure( &mpuRegion, 1 );
    if (ui32Status)
    {
        return ui32Status;
    }

    return am_hal_mpu_enable(true, true);
}

//*****************************************************************************
// Improved strcpy function
//*****************************************************************************
static char *strcpy_p(char *dest, const char *str, bool term)
{
    while ( 1 )
    {
        char ch = *str++;
        if (ch == 0)
        {
            break;
        }
        *dest++ = ch;
    }
    if (term)
    {
        *dest++  = '\r';
        *dest++  = '\n';
    }
    *dest = 0;
    return dest;
}
//*****************************************************************************
//
// print the name of the test out through the serial port
//
//*****************************************************************************
uint32_t serial_print_test_name( const uart_cmn_fcn_t *psCommonFcn )
{
    const char *pstr = psCommonFcn->testName;
    if ( pstr != NULL )
    {
        char outBuff[128];
        char *p = strcpy_p(outBuff, "starting subexample: ", false );
        p = strcpy_p(p, pstr, true );

        uint32_t len = (uint32_t) p - (uint32_t)outBuff;

        void *pUartHandle = psCommonFcn->psStreamLocalVars->pUartHandle;
        am_hal_uart_errors_t eErr = am_hal_stream_uart_append_tx( pUartHandle, (uint8_t *) outBuff, len);
        if (eErr)
        {
            printErrorString( eErr, true);
        }
    }
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// enable standard interrupts for uart streaming modes
//
//*****************************************************************************
uint32_t serial_interrupt_enable( uart_examp_stream_vars_t *psStreamLVars )
{
    return am_hal_uart_stream_interrupt_clr_set( psStreamLVars->pUartHandle,
        0, (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT));
}
//*****************************************************************************
//
// enable standard interrupts for uart streaming modes
//
//*****************************************************************************
uint32_t mcu_setup(void)
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
        return ui32Status;
    }

    //
    // Enable SYSPLL.
    //
    ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_SYSPLL, AM_HAL_UART_PLLCLK_FREQ, NULL);
    if (ui32Status)
    {
        return ui32Status;
    }


#else
    //
    am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLVCO, 0, NULL);
    ui32Status = am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, AM_HAL_UART_PLLCLK_FREQ, NULL);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf(" PLLPOSTDIV clock source error: %d\n", ui32Status);
    }
#endif  // AM_PART_APOLLO330P_510L

    return ui32Status;
}
//*****************************************************************************
//
//! @brief utility function that'll print error and status messages from the stream isr service
//!
//! @param psAsyncLVars  pointer to stream variables
//! @param streamStatus  the isr return status
//!
//! @return accumulated status from any function calls, non-zero is an error
//
//*****************************************************************************
uint32_t processStreamErrorDefMap(uart_examp_stream_vars_t *psAsyncLVars,
                                  am_hal_uart_stream_status_t streamStatus)
{
    return processStreamError( sStreamErrorMessageDefaultMap, psAsyncLVars, streamStatus);
} // processStreamErrorDefMap


//*****************************************************************************
//
//! @brief utility function that'll print error and status messages from the stream isr service
//!
//! @param pSEM          uses an external status action map
//! @param psAsyncLVars  pointer to stream variables
//! @param streamStatus  the isr return status
//!
//! @return accumulated status from any function calls, non-zero is an error
//
//*****************************************************************************
uint32_t processStreamError(const stream_error_message_action_map_t *pSEM,
                            uart_examp_stream_vars_t *psAsyncLVars,
                            am_hal_uart_stream_status_t streamStatus)
{
    uint32_t status = 0;
    //
    // loop through all the bits that can be set in the return from the ISR
    // if any are set, print a message or take action
    //
    while ( streamStatus && pSEM->streamID )
    {
        //
        // there is at least one-bit set
        //
        if (pSEM->bEnabled && (streamStatus & pSEM->streamID))
        {
            //
            // processing is enabled for this bit, and
            // the bit was set in the isr return
            //
            if (pSEM->pfStreamAction)
            {
                //
                // A function has been defined to process the output
                //
                status |= pSEM->pfStreamAction(psAsyncLVars);
            }
            else
            {
                //
                // print the associated message
                //
                const char *msg = pSEM->stream_message;
                am_util_stdio_printf( "uart %d: %s\r\n", psAsyncLVars->uartId, msg );
            }
        } // enabled

        //
        //! clear this bit, this allows early termination of the loop
        //
        streamStatus &= ~pSEM->streamID;

        //
        // advance to the next candidate bit
        //
        pSEM++;
    } // while

    return status;
} // processStreamError

//*****************************************************************************
//
// End Doxygen group.
//! @}
//! @}
//
//*****************************************************************************
