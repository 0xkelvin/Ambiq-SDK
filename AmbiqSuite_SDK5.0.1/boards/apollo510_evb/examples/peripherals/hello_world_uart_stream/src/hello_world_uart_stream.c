//*****************************************************************************
//
//! @file hello_world_uart_stream.c
//!
//! @brief A simple "Hello World" example using the UART Stream peripheral.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup hello_world_uart_stream Hello World UART Stream Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates basic UART streaming driver functionality
//! for simple serial communication. The application showcases fundamental
//! UART transmission capabilities using the streaming driver, providing
//! a foundation for more complex UART applications. The example implements
//! basic "Hello World" functionality with device information output.
//!
//! @section hello_world_uart_stream_features Key Features
//!
//! 1. @b Simple @b UART @b Communication: Demonstrates basic UART streaming
//!    driver functionality for serial communication
//!
//! 2. @b Device @b Information @b Output: Provides device information and
//!    status messages via UART interface
//!
//! 3. @b Configurable @b UART @b Interface: Supports both VCOM (debugger
//!    interface) and external UART (FTDI) configurations
//!
//! 4. @b Interrupt @b Driven @b Operation: Implements interrupt-based data
//!    handling for reliable transmission
//!
//! 5. @b Callback @b Based @b Transmission: Implements callback-based
//!    transmission for efficient data handling
//!
//! @section hello_world_uart_stream_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes UART with streaming driver configuration
//! - Implements basic "Hello World" message transmission
//! - Provides device information and status output
//! - Implements interrupt-driven data handling
//! - Demonstrates callback-based transmission capabilities
//! - Supports both VCOM and external UART configurations
//!
//! @section hello_world_uart_stream_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Connect UART pins to PC using UART/USB cable (1.8V logic)
//! 3. Use terminal application (Tera Term, PuTTY) to view output
//! 4. Observe "Hello World" message and device information
//! 5. Monitor UART status and transmission completion
//!
//! @section hello_world_uart_stream_configuration Configuration
//!
//! - @b UART_BAUDRATE: Configurable UART communication rate (default: 115200)
//! - @b USE_VCOMM: Enable VCOM interface via debugger
//! - @b TX_BUFFER_SIZE: Configurable transmit buffer size
//! - @b RX_BUFFER_SIZE: Configurable receive buffer size
// The default is to use the built-in uart:VCOM uart
//!
//! to use VCOM (via debugger interface) that is use AM_BSP_UART_PRINT_INST:
//! #define USE_VCOMM
//!
//! If the user uses an alternate pin with an external uart (FTDI recommended),
//! Don't define USE_VCOMM and define the UART_ID that is desired ensuring the RX and TX
//! pins are accessible.
//! The user will have to find (or set) the correct pins in the pin mapping table.
//! Uart tx and rx pins are used and defined in the table.
//!
//! To see the output of this program, run a terminal application such as
//! Tera Term or PuTTY, and configure the console for UART
//!
//! The example sleeps after it is done printing.
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

#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"

//*****************************************************************************
//
// Insert compiler version at compile time.
//
// Note - COMPILER_VERSION is defined in am_hal_mcu.h.
//
//*****************************************************************************
#define ENABLE_DEBUGGER
#define UART_RX_EXAMPLE

//
// DCU controls needed for debugger
//
#define DCU_DEBUGGER (AM_HAL_DCU_SWD | AM_HAL_DCU_CPUDBG_INVASIVE | AM_HAL_DCU_CPUDBG_NON_INVASIVE | AM_HAL_DCU_CPUDBG_S_INVASIVE | AM_HAL_DCU_CPUDBG_S_NON_INVASIVE)


//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************

#define USE_TX_COMPLETE_CALLBACK 1

#define RX_BUFF_THRESHOLD 64
#define RX_BUFFER_SIZE (256)
#define TX_BUFFER_SIZE (4096)

uint8_t g_uartRxBuffer[RX_BUFFER_SIZE];                //!< uart rx buffer, used in hal
uint8_t g_uartTxBuffer[TX_BUFFER_SIZE];                //!< uart tx buffer, used in hal

volatile bool g_txComplete = false;                    //!< set when tx complete callback is called
void *phUART;                                          //!< Uart device handle
volatile am_hal_uart_stream_status_t g_UartIsrStatus;  //!< The return value from the uart isr call
volatile uint32_t ui32LastError;

#define UART_BAUDRATE 115200

#define USE_VCOMM


typedef struct
{
    uint32_t ui32PinNum;
    am_hal_gpio_pincfg_t *pui32PinCfg;
}
uart_pin_mapping_t;


#ifdef USE_VCOMM
const uart_pin_mapping_t g_uart_tx_pin = {AM_BSP_GPIO_COM_UART_TX, &g_AM_BSP_GPIO_COM_UART_TX  };
const uart_pin_mapping_t g_uart_rx_pin = {AM_BSP_GPIO_COM_UART_RX, &g_AM_BSP_GPIO_COM_UART_RX  };
#define UART_ID AM_BSP_UART_PRINT_INST

#else // !USE_VCOMM

#define UART_ID 2


#if UART_ID == 0
const uart_pin_mapping_t g_uart_tx_pin = {AM_BSP_GPIO_UART0_TX, &g_AM_BSP_GPIO_UART0_TX  };
const uart_pin_mapping_t g_uart_rx_pin = {AM_BSP_GPIO_UART0_RX, &g_AM_BSP_GPIO_UART0_RX  };
#elif UART_ID == 1
const uart_pin_mapping_t g_uart_tx_pin = {AM_BSP_GPIO_UART1_TX, &g_AM_BSP_GPIO_UART1_TX  };
const uart_pin_mapping_t g_uart_rx_pin = {AM_BSP_GPIO_UART1_RX, &g_AM_BSP_GPIO_UART1_RX  };
#elif UART_ID == 2
const uart_pin_mapping_t g_uart_tx_pin = {AM_BSP_GPIO_UART2_TX, &g_AM_BSP_GPIO_UART2_TX  };
const uart_pin_mapping_t g_uart_rx_pin = {AM_BSP_GPIO_UART2_RX, &g_AM_BSP_GPIO_UART2_RX  };
#elif UART_ID == 3
const uart_pin_mapping_t g_uart_tx_pin = {AM_BSP_GPIO_UART3_TX, &g_AM_BSP_GPIO_UART3_TX  };
const uart_pin_mapping_t g_uart_rx_pin = {AM_BSP_GPIO_UART3_RX, &g_AM_BSP_GPIO_UART3_RX  };
#else
#warning "invalid uart selection"
#endif
#endif // !USE_VCOMM

//*****************************************************************************
//
//! UART configuration.
//
//*****************************************************************************
static const am_hal_uart_config_t g_sUartConfig =
{
    //
    //! at normal speeds: Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate    = UART_BAUDRATE,
    //
    //! Set TX and RX FIFOs to interrupt at three-quarters full.
    //! Rx input will also use the timeout feature to trigger an RX ISR
    //
    .eDataBits    = AM_HAL_UART_DATA_BITS_8,
    .eParity      = AM_HAL_UART_PARITY_NONE,
#if UART_BAUDRATE >= 1500000
    //
    //! two stops bits has been seen to work a bit better at high speeds
    //
    .eStopBits    = AM_HAL_UART_TWO_STOP_BITS,
#else
    .eStopBits    = AM_HAL_UART_ONE_STOP_BIT,
#endif
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_4,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_24,
    .eClockSrc    = AM_HAL_UART_CLOCK_SRC_HFRC,
};

static void tx_complete_callback( am_hal_uart_stream_callback_data_t *psCBData);

//
//! Configure uart streaming setup parameters
//
const am_hal_uart_stream_data_config_t sDataCfg =
{
    //
    //! specify dma mode
    //
    .eStreamingDmaMode = AM_HAL_UART_DMA_NONE,

    //
    //! setup rx parameters
    //
    .sRxStreamConfig =
    {
        .ui32NumBytesToRead         =  RX_BUFF_THRESHOLD,
        .pfRxCallback               = 0,
        .ui32RxCallbackThreshold    =  RX_BUFF_THRESHOLD,
        .ui32TimeoutMs              = 0,
        .bClearRxFifo               = false,
        .sRxBuffer  = { .pui8Buff = g_uartRxBuffer, .ui32BufferSize = RX_BUFFER_SIZE  }, // use DTCM buffer
    },

    //
    //! set up tx parameters
    //
    .sTxStreamConfig =
    {
#if USE_TX_COMPLETE_CALLBACK == 1
        .pfTxCallback       = tx_complete_callback,
    #else
        .pfTxCallback       = NULL,
    #endif
        .eTxCompleteMode    = eAM_HAL_TX_COMPL_TX_COMPLETE,
        .eTxCompleteNotificationAction = eAM_HAL_TX_COMPL_TX_COMPLETE,
        .sTxBuffer = { .pui8Buff = g_uartTxBuffer, .ui32BufferSize = TX_BUFFER_SIZE  }, // use DTCM buffer
    },
} ;

//*****************************************************************************
//
// Prototypes
//
//*****************************************************************************

static uint32_t uart_printf_init(void) ;
void uart_print(char *pcStr);


//*****************************************************************************
//
//! Called at ISR level from uart ISR
//! This is called (when enabled) when the UART transmit is complete
//
//*****************************************************************************
static void tx_complete_callback( am_hal_uart_stream_callback_data_t *psCBData)
{
    g_txComplete = true ;
}

//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************


#if UART_ID == 0
void am_uart_isr(void)
#elif UART_ID == 1
void am_uart1_isr(void)
#elif UART_ID == 2
void am_uart2_isr(void)
#elif UART_ID == 3
void am_uart3_isr(void)
#endif
{
    //
    // This uart streaming ISR handles all the register setups and control
    // It returns a status word that the use can check.
    //
    g_UartIsrStatus |= am_hal_uart_interrupt_stream_service(phUART);
}

//*****************************************************************************
//
//! @brief Uart print string
//! This is a callback from the print subsystem: am_util_stdio_printf()
//!
//! @param pcStr   The string that will be set via uart driver
//
//*****************************************************************************
void
uart_print(char *pcStr)
{
    uint32_t ui32TryCount = 4 ;

    am_hal_uart_errors_t eErrors = AM_HAL_UART_STATUS_SUCCESS;

    //
    // Compute string length.
    //
    uint32_t ui32StrLen =  strlen(pcStr);

    while (ui32TryCount--)
    {
        //
        //! queue the output data and start transmission if it isn't running
        //
        eErrors = am_hal_stream_uart_append_tx( phUART, (uint8_t *) pcStr, ui32StrLen);
        //
        // For this example, if the buffer is full, delay and try again several times.
        // One could also wait for tx complete from the ISR or the callback.
        // This is an example here, because the uart output buffer size is large enough to hold
        // everything this program will send.
        //
        if (eErrors != AM_HAL_UART_ERR_BUFFER_OVERFILL)
        {
            //
            // either success or a more severe error, exit this loop
            //
            break;
        }
        am_hal_delay_us(100000);  // 100 msec
    }
}

//*****************************************************************************
//
//! @brief  UART initialization and configuration
//
//*****************************************************************************

static uint32_t
uart_printf_init(void)
{
    //
    // Initialize the printf interface for UART output.
    //
    uint32_t uiStatus;

    do
    {
        //
        // setup rx and tx pins
        //
        uiStatus = am_hal_gpio_pinconfig(g_uart_tx_pin.ui32PinNum, *g_uart_tx_pin.pui32PinCfg);
        if (uiStatus)
        {
            break;
        }
        uiStatus = am_hal_gpio_pinconfig(g_uart_rx_pin.ui32PinNum, *g_uart_rx_pin.pui32PinCfg);
        if (uiStatus)
        {
            break;
        }

        //
        // init the uart
        // first get the uart handle
        //
        uiStatus = am_hal_uart_stream_initialize(UART_ID, &phUART);
        if (uiStatus)
        {
            break;
        }

        uiStatus = am_hal_uart_stream_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false);
        if (uiStatus)
        {
            break;
        }

        //
        // Config the low-level uart params (baud rate, stop bits)
        //
        uiStatus = am_hal_uart_stream_configure( phUART, &g_sUartConfig);
        if (uiStatus)
        {
            am_util_stdio_printf("am_hal_uart_stream_configure error: %d\r\n", uiStatus );
            break;
        }

        //
        // Configure the uart Tx and Tx setups, callbacks, buffers, DMA mode, etc.
        //
        am_hal_uart_errors_t uiStreamStatus = am_hal_uart_stream_data_configure(phUART, &sDataCfg);
        if (uiStreamStatus)
        {
            uiStatus = (uint32_t) uiStreamStatus;
            break;
        }

        // enable rx and rx-timeout interrupts
        uint32_t ui32DisableTheseInts = 0 ;
        uint32_t ui32EnableTheseInts = UART0_IER_RTIM_Msk | UART0_IER_RXIM_Msk ;

        am_hal_uart_stream_interrupt_clr_set( phUART, ui32DisableTheseInts, ui32EnableTheseInts) ;

        //
        // @todo verify if this is still true
        // Make sure to enable the interrupts for RX, since the HAL doesn't already
        // know we intend to use them., this is done later in the startup sequence
        //
    }
    while( false );


    if (uiStatus == AM_HAL_STATUS_SUCCESS)
    {
        IRQn_Type eIrqType = (IRQn_Type) (UART_ID + UART0_IRQn);

        NVIC_ClearPendingIRQ(eIrqType);
        //
        // Optional: raise the uart priority above normal in an attempt to prevent DMA overruns or starvation
        //
        uint32_t ui32InterruptPriority = (AM_IRQ_PRIORITY_DEFAULT > 1)? (AM_IRQ_PRIORITY_DEFAULT-1) : AM_IRQ_PRIORITY_DEFAULT;
        NVIC_SetPriority(eIrqType, ui32InterruptPriority);
        NVIC_EnableIRQ(eIrqType);

        am_util_stdio_printf_init(uart_print);
    }

    return uiStatus;

} // UART_printf_init

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    am_util_id_t sIdDevice;
    uint32_t ui32StrBuf;
    uint32_t ui32Ret;
    am_hal_reset_status_t sResetStatus;
    uint8_t ui8OTPstatus;
    bool bOTPEnabled;
    uint32_t ui32Status;
    uint32_t ui32ErrorsWaitForPrintInit;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Check and Power on OTP if it is not already on.
    //
    ui32ErrorsWaitForPrintInit = 0;
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bOTPEnabled);
    ui32ErrorsWaitForPrintInit |= ( ui32Status == AM_HAL_STATUS_SUCCESS ) ? 0 : 0x1;

    if ( !bOTPEnabled )
    {
        ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        ui32ErrorsWaitForPrintInit |= ( ui32Status == AM_HAL_STATUS_SUCCESS ) ? 0 : 0x2;
    }

    //
    // Enable the Crypto module
    //
    ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    ui32ErrorsWaitForPrintInit |= ( ui32Status == AM_HAL_STATUS_SUCCESS ) ? 0 : 0x4;

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#ifdef ENABLE_DEBUGGER
    //
    // Enable the DCU for SWO.
    //
    MCUCTRL->DEBUGGER &= ~AM_HAL_DCU_SWO;
#endif //  ENABLE_DEBUGGER

    //
    // Initialize the printf interface for UART output
    //
    ui32Status = uart_printf_init();
    if (ui32Status)
    {
        // can't print an error message so just hang here
        while (1);
    }

    //
    // Testing indicated a small delay is needed here for the clock to stabilize
    //
    am_hal_delay_us(10000);

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Hello World!\n\n");

    //
    // Now that printing has been enabled, check for any OTP enable errors.
    //
    if ( ui32ErrorsWaitForPrintInit != 0 )
    {
        am_util_stdio_printf("OTP errors occurred 0x%X:\n", ui32ErrorsWaitForPrintInit);

        if ( ui32ErrorsWaitForPrintInit & 0x1 )
        {
            am_util_stdio_printf("  Error during read of OTP power status\n");
        }

        if ( ui32ErrorsWaitForPrintInit & 0x2 )
        {
            am_util_stdio_printf("  Error in power up of OTP module\n");
        }

        if ( ui32ErrorsWaitForPrintInit & 0x4 )
        {
            am_util_stdio_printf("  Error in power up of Crypto module\n");
        }
    }

    //
    // Print the device info.
    //
    am_util_id_device(&sIdDevice);
    am_util_stdio_printf("Vendor Name: %s\n", sIdDevice.pui8VendorName);
    am_util_stdio_printf("Device type: %s\n", sIdDevice.pui8DeviceName);
    am_util_stdio_printf("Device Info:\n"
                         "\tPart number: 0x%08X\n"
                         "\tChip ID0:    0x%08X\n"
                         "\tChip ID1:    0x%08X\n"
                         "\tRevision:    0x%08X (Rev%c%c)\n",
                         sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID0,
                         sIdDevice.sMcuCtrlDevice.ui32ChipID1,
                         sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                         sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );

    //
    // If not a multiple of 1024 bytes, append a plus sign to the KB.
    //
    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32MRAMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tMRAM size:   %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32MRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32MRAMSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32DTCMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tDTCM size:   %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32DTCMSize,
                         sIdDevice.sMcuCtrlDevice.ui32DTCMSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32SSRAMSize % 1024 ) ? '+' : 0;
    am_util_stdio_printf("\tSSRAM size:  %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32SSRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32SSRAMSize / 1024,
                         &ui32StrBuf);

    //
    // If INFO1 is OTP, OTP must first be enabled.
    //
    ui8OTPstatus = 0;
    if ( MCUCTRL->SHADOWVALID_b.INFO1SELOTP )
    {
        //
        // bit0=INFO1 OTP, bit1=OTP enabled. Enable/disable if == 0x1.
        //
        am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bOTPEnabled);
        ui8OTPstatus = bOTPEnabled ? 0x3 : 0x1;
        if ( ui8OTPstatus == 0x1 )
        {
            am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        }
    }

    if ( sIdDevice.sMcuCtrlFeature.trimver_b.bTrimVerValid )
    {
        if ( sIdDevice.sMcuCtrlFeature.trimver_b.bTrimVerPCM )
        {
            am_util_stdio_printf("\tTrim Rev     PCM %d.%d\n",
                sIdDevice.sMcuCtrlFeature.trimver_b.ui8TrimVerMaj,
                sIdDevice.sMcuCtrlFeature.trimver_b.ui8TrimVerMin);
        }
        else
        {
            am_util_stdio_printf("\tTrim Rev     %d\n",
                sIdDevice.sMcuCtrlFeature.trimver_b.ui8TrimVerMin);
        }
    }
    else
    {
        am_util_stdio_printf("\tTrim Rev Unknown (0x%08X).\n",
                sIdDevice.sMcuCtrlFeature.trimver_b.ui8TrimVerMin);
    }

    //
    // Print the compiler version.
    //
    //////am_hal_uart_tx_flush(phUART);
    am_util_stdio_printf("App Compiler:    %s\n", COMPILER_VERSION);
    am_util_stdio_printf("HAL Compiler:    %s\n", g_ui8HALcompiler);
    am_util_stdio_printf("HAL SDK version: %d.%d.%d\n",
                         g_ui32HALversion.s.Major,
                         g_ui32HALversion.s.Minor,
                         g_ui32HALversion.s.Revision);
    am_util_stdio_printf("\n");

    am_util_stdio_printf("SECURITY INFO\n");
    am_util_stdio_printf("=============\n");

    am_hal_reset_status_get(&sResetStatus);
    am_util_stdio_printf("Reset Status:    0x%X\n", sResetStatus.eStatus);

    bool bInfo0Valid = am_hal_info0_valid();
    am_hal_security_socid_t socId;
    uint32_t ui32Var;
    uint32_t ui32dcuVal;
    uint32_t lcs = CRYPTO->LCSREG_b.LCSREG;
    am_util_stdio_printf("Device LCS: %s\n",
                         ((lcs == 0) ? "CM" :       \
                         ((lcs == 1) ? "DM" :       \
                         ((lcs == 5) ? "Secure" :   \
                         ((lcs == 7) ? "RMA" : "Undefined")))));
    am_hal_dcu_get(&ui32dcuVal);
    am_hal_security_get_socid(&socId);
    am_util_stdio_printf("\tSOC Id:\n\t0x%08X : 0x%08X : 0x%08X : 0x%08X\n\t0x%08X : 0x%08X : 0x%08X : 0x%08X\n",
                             socId.socid[0], socId.socid[1], socId.socid[2], socId.socid[3],
                             socId.socid[4], socId.socid[5], socId.socid[6], socId.socid[7] );
    if ( bInfo0Valid )
    {
        am_hal_info0_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO0, AM_REG_OTP_INFO0_SBR_SDCERT_ADDR_O / 4, 1, &ui32Var);
    }
    else
    {
        ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, AM_REG_OTP_INFO1_SBR_SDCERT_ADDR_O / 4, 1, &ui32Var);
        if ( (ui32Var != 0x00000000)        &&
             (ui32Ret == AM_HAL_STATUS_SUCCESS) )
        {
            am_util_stdio_printf("Secure Debug Certificate Location: 0x%08X\n", ui32Var);
        }
        else
        {
            am_util_stdio_printf("Error - could not retrieve Secure Debug Certificate Location from INFO1.\n");
        }
    }

    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, AM_REG_OTP_INFO1_SBR_OPT_ADDR_O / 4, 1, &ui32Var);
    if ( (ui32Var != 0x00000000)        &&
         (ui32Ret == AM_HAL_STATUS_SUCCESS) )
    {
        am_util_stdio_printf("SBR OPT Address: 0x%08X\n", ui32Var);
    }
    else
    {
        am_util_stdio_printf("Error - could not retrieve OTP Address from INFO1.\n");
    }

    //
    // Restore OTP to its original state.
    //
    if ( ui8OTPstatus == 0x1 )
    {
        am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
    }

#ifdef ENABLE_DEBUGGER
    {
        //
        // Enable Debugger & handle Reset-Halt if needed.
        //
        am_hal_bootloader_exit(NULL, true);
    }
#endif // ENABLE_DEBUGGER

    g_UartIsrStatus = AM_HAL_UART_STREAM_STATUS_SUCCESS;
    am_util_stdio_printf("\n");

    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 1) && (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 1))
    {
        am_util_stdio_printf("DEBUG INFO\n");
        am_util_stdio_printf("==========\n");
        am_util_stdio_printf("Original Qualified DCU Val   0x%x\n", ui32dcuVal);
        am_hal_dcu_get(&ui32dcuVal);
        am_util_stdio_printf("Current  Qualified DCU Val   0x%x\n", ui32dcuVal);
        am_hal_dcu_lock_status_get(&ui32dcuVal);
        am_util_stdio_printf("Qualified DCU Lock Val       0x%x\n", ui32dcuVal);
        DIAG_SUPPRESS_VOLATILE_ORDER()
        am_util_stdio_printf("\tRaw DCU Enable: 0x%08X : 0x%08X : 0x%08X : 0x%08X\n",
                             CRYPTO->HOSTDCUEN0, CRYPTO->HOSTDCUEN1, CRYPTO->HOSTDCUEN2, CRYPTO->HOSTDCUEN3);
        am_util_stdio_printf("\tRaw DCU Lock  : 0x%08X : 0x%08X : 0x%08X : 0x%08X\n",
                             CRYPTO->HOSTDCULOCK0, CRYPTO->HOSTDCULOCK1, CRYPTO->HOSTDCULOCK2, CRYPTO->HOSTDCULOCK3);
        DIAG_DEFAULT_VOLATILE_ORDER()
        am_util_stdio_printf("MSPLIM:  0x%08X\n", __get_MSPLIM());
        am_util_stdio_printf("PSPLIM:  0x%08X\n", __get_PSPLIM());
        am_util_stdio_printf("\n");
    }

    //
    //! wait for isr to indicate tx complete, see g_UartIsrStatus = 0; above
    //
    while ( !(g_UartIsrStatus & AM_HAL_UART_STREAM_STATUS_TX_COMPLETE)) ;
    //
    // We can power off crypto now
    //
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power down of Crypto module\n");
    }
    //
    // Power down OTP as well.
    //
    ui32Status = am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_OTP);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power down of OTP module\n");
    }

    //
    // Done printing.
    // Disable the UART and interrupts
    //
#if USE_TX_COMPLETE_CALLBACK == 1
    g_txComplete = false; // clear tx callback status in case it's already been set
#else
    g_UartIsrStatus = AM_HAL_UART_STREAM_STATUS_SUCCESS;
#endif
    // add a large amount of spaces to overcome whatever is happening in the jlink uart
    am_util_stdio_printf("Done with prints. Entering While loop\n"
         "                                                                      ");
    //
    // wait for tx to finish
    //
#if USE_TX_COMPLETE_CALLBACK == 1
    //
    // wait for tx callback
    //
    while ( g_txComplete == false );
#else
    //
    // wait for isr to indicate tx complete
    //
    while ( !(g_UartIsrStatus & AM_HAL_UART_STREAM_STATUS_TX_COMPLETE)) ;
#endif

#ifdef UART_RX_EXAMPLE
    while(1)
    {
        uint8_t rxBuff[128];

        uint32_t  ui32NumBytes = am_hal_uart_stream_get_rx_data(phUART, rxBuff, sizeof(rxBuff), false ) ;

        if (ui32NumBytes > 0)
        {
            am_hal_stream_uart_append_tx( phUART, rxBuff, ui32NumBytes);
        }
    }

#else
    ui32Status = am_hal_uart_stream_power_control(phUART, AM_HAL_SYSCTRL_DEEPSLEEP, false);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        error_handler(ui32Status);
    }

    //
    // Loop forever while looking for rx input
    //
    while (1)
    {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
#endif
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

