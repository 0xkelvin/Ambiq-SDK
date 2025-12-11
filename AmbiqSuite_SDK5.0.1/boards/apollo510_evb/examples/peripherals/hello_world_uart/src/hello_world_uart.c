//*****************************************************************************
//
//! @file hello_world_uart.c
//!
//! @brief A simple "Hello World" example using the UART peripheral.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup hello_world_uart Hello World UART Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates basic UART driver functionality for
//! simple serial communication. The application showcases fundamental UART
//! transmission capabilities, providing a foundation for more complex UART
//! applications. The example implements basic "Hello World" functionality
//! with device information output.
//!
//! @section hello_world_uart_features Key Features
//!
//! 1. @b Simple @b UART @b Communication: Demonstrates basic UART driver
//!    functionality for serial communication
//!
//! 2. @b Device @b Information @b Output: Provides device information and
//!    status messages via UART interface
//!
//! 3. @b Configurable @b UART @b Interface: Supports flexible UART configuration
//!
//! 4. @b Interrupt @b Driven @b Operation: Implements interrupt-based data
//!    handling for reliable transmission
//!
//! 5. @b Non @b Blocking @b Operation: Supports non-blocking UART operation
//!
//! @section hello_world_uart_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes UART with standard configuration
//! - Implements basic "Hello World" message transmission
//! - Provides device information and status output
//! - Implements interrupt-driven and non-blocking data handling
//!
//! @section hello_world_uart_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Connect UART pins to PC using UART/USB cable (1.8V logic)
//! 3. Use terminal application (Tera Term, PuTTY) to view output
//! 4. Observe "Hello World" message and device information
//! 5. Monitor UART status and transmission completion
//!
//! @section hello_world_uart_configuration Configuration
//!
//! - @b UART_BAUDRATE: Configurable UART communication rate (default: 115200)
//! - @b USE_NONBLOCKING: Enable non-blocking UART operation
//! - @b ENABLE_DEBUGGER: Enable debugger support
//!
//! To see the output of this program, run a terminal application such as
//! Tera Term or PuTTY, and configure the console for UART.
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
#define USE_NONBLOCKING
#define ENABLE_DEBUGGER

//
// DCU controls needed for debugger
//
#define DCU_DEBUGGER (AM_HAL_DCU_SWD | AM_HAL_DCU_CPUDBG_INVASIVE | AM_HAL_DCU_CPUDBG_NON_INVASIVE | AM_HAL_DCU_CPUDBG_S_INVASIVE | AM_HAL_DCU_CPUDBG_S_NON_INVASIVE)

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;

#define CHECK_ERRORS(x)                                                       \
    if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
    {                                                                         \
        error_handler(x);                                                     \
    }

volatile uint32_t ui32LastError;

//*****************************************************************************
//
// Catch HAL errors.
//
//*****************************************************************************
void
error_handler(uint32_t ui32ErrorStatus)
{
    ui32LastError = ui32ErrorStatus;

    while (1);
}

//*****************************************************************************
//
// UART buffers.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[256];
uint8_t g_pui8RxBuffer[2];

#ifdef USE_NONBLOCKING
volatile bool g_bUARTdone = false;
#endif
//*****************************************************************************
//
// UART configuration.
//
//*****************************************************************************
const am_hal_uart_config_t g_sUartConfig =
{
    //
    // Standard UART settings: 115200-8-N-1
    //
    .ui32BaudRate = 115200,
    .eDataBits = AM_HAL_UART_DATA_BITS_8,
    .eParity = AM_HAL_UART_PARITY_NONE,
    .eStopBits = AM_HAL_UART_ONE_STOP_BIT,
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,

    //
    // Set TX and RX FIFOs to interrupt at half-full.
    //
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
};

//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
#if AM_BSP_UART_PRINT_INST == 0
void am_uart_isr(void)
#elif AM_BSP_UART_PRINT_INST == 1
void am_uart1_isr(void)
#elif AM_BSP_UART_PRINT_INST == 2
void am_uart2_isr(void)
#elif AM_BSP_UART_PRINT_INST == 3
void am_uart3_isr(void)
#endif
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32Status;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    am_hal_uart_interrupt_service(phUART, ui32Status);

#ifdef USE_NONBLOCKING
    if ( ui32Status & AM_HAL_UART_INT_DMACPRIS )
    {
        g_bUARTdone = true;
    }
#endif
}

//*****************************************************************************
//
// UART print string
//
//*****************************************************************************
void
uart_print(char *pcStr)
{
    uint32_t ui32StrLen = 0;
    uint32_t ui32BytesWritten = 0;

    //
    // Measure the length of the string.
    //
    while (pcStr[ui32StrLen] != 0)
    {
        ui32StrLen++;
    }

    //
    // Print the string via the UART.
    //
    am_hal_uart_transfer_t sUartWrite =
    {
#ifdef USE_NONBLOCKING
        .eDirection = AM_HAL_UART_TX,
        .pui32TxBuffer = (uint32_t *) pcStr,
#else
        .eType = AM_HAL_UART_BLOCKING_WRITE,
        .pui8Data = (uint8_t *) pcStr,
#endif
        .ui32NumBytes = ui32StrLen,
        .pui32BytesTransferred = &ui32BytesWritten,
        .ui32TimeoutMs = 100,
        .pfnCallback = NULL,
        .pvContext = NULL,
        .ui32ErrorStatus = 0
    };

#ifdef USE_NONBLOCKING
    CHECK_ERRORS(am_hal_uart_dma_transfer(phUART, &sUartWrite));
    while (!g_bUARTdone);
    g_bUARTdone = false;
#else
    CHECK_ERRORS(am_hal_uart_transfer(phUART, &sUartWrite));
    if (ui32BytesWritten != ui32StrLen)
    {
        //
        // Couldn't send the whole string!!
        //
        while(1);
    }
#endif
}

//*****************************************************************************
//
// UART initialization and configuration
//
//*****************************************************************************
void
UART_printf_init(void)
{
    //
    // Initialize the printf interface for UART output.
    //
    CHECK_ERRORS(am_hal_uart_initialize(AM_BSP_UART_PRINT_INST, &phUART));
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_WAKE, false));
    CHECK_ERRORS(am_hal_uart_configure(phUART, &g_sUartConfig));

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);

#ifdef USE_NONBLOCKING
    am_hal_uart_interrupt_enable(phUART, (AM_HAL_UART_INT_OVER_RUN |
                                           AM_HAL_UART_INT_DMAERIS  |
                                           AM_HAL_UART_INT_DMACPRIS));
#endif

    //
    // Enable interrupts.
    //
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST), AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));
    am_hal_interrupt_master_enable();

    //
    // Set the main print interface to use the UART print function we defined.
    //
    am_util_stdio_printf_init(uart_print);

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

#ifdef AM_PART_APOLLO330P_510L
    //
    // Power off the RSS
    //
    am_hal_pwrctrl_rss_pwroff();
#endif

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
    UART_printf_init();

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
    am_hal_uart_tx_flush(phUART);
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
    // We are done printing.
    // Disable the UART and interrupts
    //
    am_util_stdio_printf("Done with prints. Entering While loop\n");
    am_hal_uart_tx_flush(phUART);
    CHECK_ERRORS(am_hal_uart_power_control(phUART, AM_HAL_SYSCTRL_DEEPSLEEP, false));

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

