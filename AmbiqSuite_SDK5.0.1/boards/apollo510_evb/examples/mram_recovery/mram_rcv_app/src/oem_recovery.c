//*****************************************************************************
//
//! @file oem_recovery_image.c
//!
//! @brief OEM Recovery Image
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

#include "hello_world_oem_recovery.h"
#include "am_bsp.h"
#include "am_util.h"
#include "cmdline.h"
#include <stdint.h>

#define WAKE_INTERVAL_IN_MS     1700
#define CRC_CLK_PERIOD          375 * 1000
#define WAKE_INTERVAL           (CRC_CLK_PERIOD * WAKE_INTERVAL_IN_MS / 1000)
uint32_t g_ui32WakeInterval = WAKE_INTERVAL;
bool g_WDTMramMsg = false;
//*****************************************************************************
//
// Insert compiler version at compile time.
//
// Note - COMPILER_VERSION is defined in am_hal_mcu.h.
//
//*****************************************************************************
//#define USE_NONBLOCKING
#define ENABLE_DEBUGGER
#define UART_BUFFER_SIZE    128

//
// DCU controls needed for debugger
//
#define DCU_DEBUGGER (AM_HAL_DCU_SWD | AM_HAL_DCU_CPUDBG_INVASIVE | AM_HAL_DCU_CPUDBG_NON_INVASIVE | AM_HAL_DCU_CPUDBG_S_INVASIVE | AM_HAL_DCU_CPUDBG_S_NON_INVASIVE)

//*****************************************************************************
//
// Global variables
//
//*****************************************************************************
volatile uint32_t g_ui32CRCCnt = 0;
volatile uint32_t g_ui32WDTCnt = 0;
volatile uint32_t g_ui32CRCFlag = 0;
volatile uint32_t g_ui32WDTFlag = 0;
volatile uint32_t g_bUartRxDone = 0;
volatile bool g_btriggerWdtRst = 0;
bool g_bCRCFail = false;

//*****************************************************************************
//
// UART handle.
//
//*****************************************************************************
void *phUART;

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
AM_SHARED_RW uint8_t g_ui8TxBuffer[UART_BUFFER_SIZE] __attribute__((aligned(32)));
AM_SHARED_RW uint8_t g_ui8RxBuffer[UART_BUFFER_SIZE] __attribute__((aligned(32)));

#ifdef USE_NONBLOCKING
bool g_bUARTdone = false;
#endif

//*****************************************************************************
//
//! Watchdog Timer Configuration
//
//*****************************************************************************
const am_hal_wdt_config_t g_sWatchdogConfig =
{
    // Set the clock for ~16HZ
    .eClockSource = AM_HAL_WDT_LFRC_DIV64,
    .bInterruptEnable = true,
    .ui32InterruptValue = 3 * 16,
    .bResetEnable = true,
    .ui32ResetValue = 4 * 16
};

//*****************************************************************************
//
//! Watchdog Timer Initialization
//
//*****************************************************************************
void am_watchdog_init(void)
{
    //
    // Setup the Watchdog Timer
    //
    am_hal_wdt_stop(AM_HAL_WDT_MCU);
    am_hal_wdt_config(AM_HAL_WDT_MCU, (void*)&g_sWatchdogConfig);
    am_hal_wdt_interrupt_enable(AM_HAL_WDT_MCU, AM_HAL_WDT_INTERRUPT_MCU);

    NVIC_SetPriority((IRQn_Type)(WDT_IRQn), 0);
    NVIC_EnableIRQ((IRQn_Type)(WDT_IRQn));

    //
    // Start the Watchdog Timer
    //
    am_hal_wdt_start(AM_HAL_WDT_MCU, false);
}

//*****************************************************************************
//
//! Watchdog interrupt handler.
//
//*****************************************************************************
void __attribute__((section("RAMFUNC")))
am_watchdog_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and clear the interrupt status.
    //
    am_hal_wdt_interrupt_status_get(AM_HAL_WDT_MCU, &ui32Status, true);
    am_hal_wdt_interrupt_clear(AM_HAL_WDT_MCU, ui32Status);

    //
    // Restart the watchdog.
    //
    g_ui32WDTFlag = 1;
    g_ui32WDTCnt++;
    if (g_WDTMramMsg)
    {
        am_hal_wdt_restart(AM_HAL_WDT_MCU);
    }

    //
    // Check for CRC and recover if necessary
    //
    if ( g_bCRCFail )
    {
        //
        //Trigger the Application Recovery
        //
        am_hal_mram_recovery_init_app_recovery(AM_HAL_MRAM_RECOVERY_KEY, false);
        //
        // Application is corrupted.
        //
        am_print_swo_terminal("CRC Check failed.\nStopping Watchdog Timer and initiating Application MRAM Recovery.\n");
        //
        // POI
        //
        RSTGEN->SWPOI = RSTGEN_SWPOI_SWPOIKEY_KEYVALUE;
    }
    //
    // Trigger a WDT reset if the flag is set
    //
    if (g_btriggerWdtRst)
    {
        while(1);
    }
}

//*****************************************************************************
//
//! STIMER Initialization
//
//*****************************************************************************
void am_stimer_init(void)
{
    //
    // Enable the STIMER
    //
    am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_config(AM_HAL_STIMER_HFRC_375KHZ |
                         AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);

    //
    // Set the compare A value
    //
    am_hal_stimer_compare_delta_set(0, WAKE_INTERVAL);

    //
    // Enable the STIMER interrupt
    //
    NVIC_SetPriority(STIMER_CMPR0_IRQn, 2);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);
}

void am_change_crc_trigger_time(uint32_t ui32Time)
{
    if (ui32Time == 0)
    {
        am_hal_stimer_int_disable(AM_HAL_STIMER_INT_COMPAREA);
        am_print_swo_uart_terminal("Stopping the CRC\n");
    }
    else
    {
        am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
        uint32_t ui32StimerInterval;
        am_print_swo_uart_terminal("Setting CRC Period : %d mSec\n", ui32Time);
        ui32StimerInterval = (ui32Time * 375);
        g_ui32WakeInterval = ui32StimerInterval;
    }
}

void am_change_wdt_trigger_time(uint32_t ui32Time)
{

    //
    // Stop the WDT if the time is 0
    //
    if (ui32Time != 0)
    {
        am_print_swo_uart_terminal("Setting WDT Period : %d mSec\n", ui32Time);

        am_hal_wdt_stop(AM_HAL_WDT_MCU);

        am_hal_wdt_config_t sWatchdogConfig =
        {
            // Set the clock for ~16HZ
            .eClockSource = AM_HAL_WDT_LFRC_DIV64,
            .bInterruptEnable = true,
            .ui32InterruptValue = ((ui32Time - 1000) * 16) / 1000 ,
            .bResetEnable = true,
            .ui32ResetValue = ((ui32Time) * 16) / 1000
        };
        am_hal_wdt_config(AM_HAL_WDT_MCU, &sWatchdogConfig);
        //
        // Start the Watchdog Timer
        //
        am_hal_wdt_start(AM_HAL_WDT_MCU, false);
        am_hal_wdt_restart(AM_HAL_WDT_MCU);

    }
    else
    {
        am_print_swo_uart_terminal("Stopping the WatchDog Timer\n");
        WDT->CFG_b.WDTEN = 0x0;
    }
}

//*****************************************************************************
//
//! STIMER interrupt handler.
//
//*****************************************************************************
void
am_stimer_cmpr0_isr(void)
{
    //
    // Check the timer interrupt status.
    //
    am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
    am_hal_stimer_compare_delta_set(0, g_ui32WakeInterval);

    g_ui32CRCFlag = 1;
    g_ui32CRCCnt++;

}

//*****************************************************************************
//
// Corrupt the Stored CRC Value to trigger the MRAM Recovery
//
//*****************************************************************************
void am_corrupt_crc(void)
{
    uint32_t ui32CorruptCRC[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
    am_print_swo_uart_terminal("Corrupting CRC to force MRAM Recovery\n")
    //
    // Corrupt the CRC
    //
    am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, ui32CorruptCRC, (uint32_t*)(__pPatchable + 1), 4);
}

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
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_28,
    .eRXFifoLevel = 0,
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
    uint32_t ui32Module = AM_BSP_UART_PRINT_INST;
    am_hal_uart_interrupt_status_get(phUART, &ui32Status, true);
    am_hal_uart_interrupt_clear(phUART, ui32Status);
    //am_hal_uart_interrupt_service(phUART, ui32Status);

#ifdef USE_NONBLOCKING
    if ( ui32Status & AM_HAL_UART_INT_DMACPRIS )
    {
        g_bUARTdone = true;
    }
#endif

    if ( ui32Status & AM_HAL_UART_INT_RX )
    {
        while ( !UARTn(ui32Module)->FR_b.RXFE )
        {
            cmdLine(ui32Module);
        }
    }
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
                                           AM_HAL_UART_INT_DMACPRIS |
                                           AM_HAL_UART_INT_RX));
#else
    am_hal_uart_interrupt_enable(phUART, (AM_HAL_UART_INT_TX |
                                          AM_HAL_UART_INT_TXCMP));

#endif
    //
    // Enable interrupts.
    //
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST), 1);
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + AM_BSP_UART_PRINT_INST));

} // UART_printf_init

//*****************************************************************************
//
//! Get the reset status and print it out.
//
//*****************************************************************************
void am_print_reset_status(void)
{
    am_hal_reset_status_t sResetStatus;

    //
    // Get the reset status
    //
    am_hal_reset_status_get(&sResetStatus);

    //
    // Print the reset status
    //
    am_print_swo_terminal("  Device Reset Status: 0x%08X\n", sResetStatus.eStatus);

    //
    // If reset caused by Watchdog perform a CRC
    //
    if (sResetStatus.bWDTStat)
    {
        //
        // Clear the STAT register
        //
        RSTGEN->STAT &= ~(AM_HAL_RESET_STATUS_WDT);

        am_print_swo_terminal("  Device Reset Caused by Watch Dog Timer.\n    Performing CRC Check.\n")

        //AM_CRITICAL_BEGIN
        if (!am_crc_check_app(false))
        {
            //
            // Application is corrupted.
            //
            am_print_swo_terminal("  CRC Check failed after a WDT Reset.\nInitiating Application MRAM Recovery.\n\n");
            am_hal_mram_recovery_init_app_recovery(AM_HAL_MRAM_RECOVERY_KEY, true);
        }
        else
        {
            am_print_swo_terminal("  CRC Check passed after a WDT Reset.\n\n");
        }
        //AM_CRITICAL_END
    }

} // am_print_reset_status

//*****************************************************************************
//
//! Get the MRAM Recovery Status and print it out.
//
//*****************************************************************************
void am_mram_recovery_status(am_mram_recovery_status_t* sMRAMRecoveryStatus )
{
    uint32_t ui32ResetStatus = AM_REGVAL(0x4000885C);
    uint32_t ui32Info0[20];

    sMRAMRecoveryStatus->eStatus = (am_mram_recovery_status_e)(ui32ResetStatus & 0xFFFF8000);
    sMRAMRecoveryStatus->bRecoveryInProgress = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_RCVYINPROGRESS;
    sMRAMRecoveryStatus->bRecoveryUARTLoad = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_RCVYUARTLOAD;
    sMRAMRecoveryStatus->bRecoverySPILoad = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_RCVYSPILOAD;
    sMRAMRecoveryStatus->bRecoveryInitiated = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_RCVYINITIATED;
    sMRAMRecoveryStatus->bSBLWiredOTA = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_SBLWIREDOTA;
    sMRAMRecoveryStatus->bSBLWiredLoad = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_SBLWIREDLOAD;
    sMRAMRecoveryStatus->bSBLNVOTA = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_SBLNVOTA;
    sMRAMRecoveryStatus->bSBLNVImageLoad = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_SBLNVIMAGELOAD;
    sMRAMRecoveryStatus->bOEMWiredOTA = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_OEMWIREDOTA;
    sMRAMRecoveryStatus->bOEMWiredLoad = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_OEMWIREDLOAD;
    sMRAMRecoveryStatus->bOEMNVOTA = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_OEMNVOTA;
    sMRAMRecoveryStatus->bOEMNVImageLoad = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_OEMNVIMAGELOAD;
    sMRAMRecoveryStatus->bOEMRecoveryFail = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_OEMRCVYFAIL;
    sMRAMRecoveryStatus->bOEMRecoverySuccess = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_OEMRCVYSUCCESS;
    sMRAMRecoveryStatus->bSBLRecoveryFail = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_SBLRCVYFAIL;
    sMRAMRecoveryStatus->bSBLRecoverySuccess = ui32ResetStatus & AM_MRAM_RECOVERY_STATUS_SBLRCVYSUCCESS;

    //
    // Clear the MRAM Recovery Status Registers bit fields
    //
    RSTGEN->STAT &= AM_MRAM_RECOVERY_MASK;

    //
    // Read INFO0
    //
    am_hal_info0_read(AM_INFO0_SEL(MCUCTRL->SHADOWVALID_b.INFO0SELOTP), 0x15, 0x14, &ui32Info0[0]);

    //
    // Print the MRAM Recovery Status.
    //
    am_print_swo_terminal("  MRAM Recovery Status: 0x%08X\n", *sMRAMRecoveryStatus);

    //
    // Print if the reset happened due to MRAM Reocvery
    //
    if ( sMRAMRecoveryStatus->bSBLRecoverySuccess || sMRAMRecoveryStatus->bOEMRecoverySuccess )
    {
        am_print_swo_terminal("  Device reset caused by MRAM Recovery\n");

        if ( sMRAMRecoveryStatus->bSBLRecoverySuccess )
        {
            am_print_swo_terminal("Successful Ambiq MRAM Recovery\n");
        }
        if ( sMRAMRecoveryStatus->bSBLNVImageLoad )
        {
            if ( (ui32Info0[4] & AM_REG_INFO0_MRAM_RCVY_CTRL_NV_RCVY_TYPE_Msk) >> AM_REG_INFO0_MRAM_RCVY_CTRL_NV_RCVY_TYPE_Pos ==       \
                  AM_ENUM_INFO0_MRAM_RCVY_CTRL_NV_RCVY_TYPE_NV_MSPI )
            {
                am_print_swo_terminal("Ambiq MRAM Recovery using MSPI\n");
            }
            else
            {
                am_print_swo_terminal("Ambiq MRAM Recovery using eMMC\n");
            }
        }
        if ( sMRAMRecoveryStatus->bSBLWiredLoad )
        {
            if (sMRAMRecoveryStatus->bRecoveryUARTLoad)
            {
                am_print_swo_terminal("Ambiq MRAM Recovery using UART\n");
            }
            else if (sMRAMRecoveryStatus->bRecoverySPILoad)
            {
                am_print_swo_terminal("Ambiq MRAM Recovery using SPI\n");
            }
        }
        if ( sMRAMRecoveryStatus->bOEMNVImageLoad )
        {
            if ( (ui32Info0[4] & AM_REG_INFO0_MRAM_RCVY_CTRL_NV_RCVY_TYPE_Msk) >> AM_REG_INFO0_MRAM_RCVY_CTRL_NV_RCVY_TYPE_Pos ==       \
                  AM_ENUM_INFO0_MRAM_RCVY_CTRL_NV_RCVY_TYPE_NV_MSPI )
            {
                am_print_swo_terminal("OEM MRAM Recovery using MSPI\n");
            }
            else
            {
                am_print_swo_terminal("OEM MRAM Recovery using eMMC\n");
            }
        }
        if ( sMRAMRecoveryStatus->bOEMWiredLoad )
        {
            if (sMRAMRecoveryStatus->bRecoveryUARTLoad)
            {
                am_print_swo_terminal("OEM MRAM Recovery using UART\n");
            }
            else if (sMRAMRecoveryStatus->bRecoverySPILoad)
            {
                am_print_swo_terminal("OEM MRAM Recovery using UART\n");
            }
        }
    }
    else
    {
        am_print_swo_terminal("  Normal Boot -- No MRAM Recovery Occurred\n\n");
    }
}

//*****************************************************************************
//
//! Print the INFO0 settings
//
//*****************************************************************************
void am_print_info0(void)
{
    uint32_t ui32Info0[20];
    uint32_t ui32Info0Uart[6];
    uint32_t ui32InfoC;
    uint32_t ui32Ret;

    //
    // Read the INFO0 settings
    //
    ui32Ret = am_hal_info0_read(AM_INFO0_SEL(MCUCTRL->SHADOWVALID_b.INFO0SELOTP), 0x15, 0x14, &ui32Info0[0]);
    ui32Ret = am_hal_info0_read(AM_INFO0_SEL(MCUCTRL->SHADOWVALID_b.INFO0SELOTP), 0xA, 0x6, &ui32Info0Uart[0]);
    ui32Ret = am_hal_infoc_read_word(0x95, &ui32InfoC);

    if ( !ui32Ret )
    {
        am_print_swo_uart_terminal("\nError Reading INFO0\n\n");
        return;
    }
    //
    // Print the INFO0 settings
    //
    if (MCUCTRL->SHADOWVALID_b.INFO0SELOTP)
    {
        am_print_swo_uart_terminal("\nINFO0-OTP Settings\n==============\n\n");
    }
    else
    {
        am_print_swo_uart_terminal("\nINFO0-MRAM Settings\n==============\n\n");
    }

    am_print_swo_uart_terminal("OFFSET  VALUE         DESCRIPTION\n");
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Main Pointer\n", AM_REG_INFO0_MAINPTR_O , ui32Info0[3]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Configuration Setting 0\n", AM_REG_INFO0_SECURITY_WIRED_IFC_CFG0_O, ui32Info0Uart[0]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Configuration Setting 1\n", AM_REG_INFO0_SECURITY_WIRED_IFC_CFG1_O, ui32Info0Uart[1]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Configuration Setting 2\n", AM_REG_INFO0_SECURITY_WIRED_IFC_CFG2_O, ui32Info0Uart[2]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Configuration Setting 3\n", AM_REG_INFO0_SECURITY_WIRED_IFC_CFG3_O, ui32Info0Uart[3]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Configuration Setting 4\n", AM_REG_INFO0_SECURITY_WIRED_IFC_CFG4_O, ui32Info0Uart[4]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Configuration Setting 5\n", AM_REG_INFO0_SECURITY_WIRED_IFC_CFG5_O, ui32Info0Uart[5]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 Wired Timeout\n", AM_REG_INFO0_WIRED_TIMEOUT_O, ui32Info0[0]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 MRAM Recovery Control\n", AM_REG_INFO0_MRAM_RCVY_CTRL_O, ui32Info0[5]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Meta Data Offset\n", AM_REG_INFO0_NV_METADATA_OFFSET_O, ui32Info0[6]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Power & Reset Config\n", AM_REG_INFO0_NV_PWR_RESET_CFG_O, ui32Info0[7]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Pin Numbers\n", AM_REG_INFO0_NV_PIN_NUMS_O, ui32Info0[8]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV CE/CMD Config\n", AM_REG_INFO0_NV_CE_CMD_PINCFG_O, ui32Info0[9]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Clock Config\n", AM_REG_INFO0_NV_CLK_PINCFG_O, ui32Info0[10]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Data Config\n", AM_REG_INFO0_NV_DATA_PINCFG_O, ui32Info0[11]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV DQS Config\n", AM_REG_INFO0_NV_DQS_PINCFG_O, ui32Info0[12]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Config0\n", AM_REG_INFO0_NV_CONFIG0_O, ui32Info0[13]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Config1\n", AM_REG_INFO0_NV_CONFIG1_O, ui32Info0[14]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Config2\n", AM_REG_INFO0_NV_CONFIG2_O, ui32Info0[15]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Config3\n", AM_REG_INFO0_NV_CONFIG3_O, ui32Info0[16]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV Options\n", AM_REG_INFO0_NV_OPTIONS_O, ui32Info0[17]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 NV MSPI Pre Commands\n", AM_REG_INFO0_NV_MSPI_PRECMDS_O, ui32Info0[18]);
    am_print_swo_uart_terminal("0x%02X    0x%08X    INFO0 MRAM Recovery Retries\n", AM_REG_INFO0_MRAM_RCV_RETRIES_TIMES_O, ui32Info0[19]);

}

//*****************************************************************************
//
//! Print MRAM Recovery Configurations
//
//*****************************************************************************
void am_print_mram_recovery_config(am_mram_recovery_cfg_t* sMRCfg)
{
    //
    // Check if Application based MRAM Recovery is enabled
    //
    if (sMRCfg->appRcvyEnable)
    {
        am_print_swo_uart_terminal("\tApplication MRAM Recovery  : Enabled.\n");
    }
    else
    {
        am_print_swo_uart_terminal("\tApplication MRAM Recovery  : Disabled.\n");
    }

    //
    // Check if Wired MRAM Recovery is enabled
    //
    if (sMRCfg->wiredRcvyEnable)
    {
        am_print_swo_uart_terminal("\tWired MRAM Recovery        : Enabled.\n");
    }
    else
    {
        am_print_swo_uart_terminal("\tWired MRAM Recovery        : Disabled.\n");
    }

    //
    // Check if NV MRAM Recovery is enabled
    //
    switch(sMRCfg->nvRcvyType)
    {
        case 0x0:
            am_print_swo_uart_terminal("\tNV Recovery Type           : None\n");
            break;
        case 0x1:
            am_print_swo_uart_terminal("\tNV Recovery Type           : MSPI %d\n", sMRCfg->nvModuleNum);
            break;
        case 0x2:
            am_print_swo_uart_terminal("\tNV Recovery Type           : eMMC %d\n", sMRCfg->nvModuleNum);

            switch(sMRCfg->eMMCpartition)
            {
                case 0x0:
                    am_print_swo_uart_terminal("\teMMC Partition             : USER\n");
                    break;
                case 0x1:
                    am_print_swo_uart_terminal("\teMMC Partition             : BOOT1\n");
                    break;
                case 0x2:
                    am_print_swo_uart_terminal("\teMMC Partition             : BOOT2\n");
                    break;
                default:
                    am_print_swo_uart_terminal("\teMMC Partition             : Invalid\n");
                    break;
            }
            break;
        default:
            am_print_swo_uart_terminal("\tNV Recovery Type       : Invalid\n");
            break;
    }

    //
    // Check if GPIO initiated MRAM Recovery is enabled
    if ( sMRCfg->pinNumGpioRcvy != INVALID_PIN )
    {
        am_print_swo_uart_terminal("\tGPIO Recovery              : Enabled.\n");
        am_print_swo_uart_terminal("\tGPIO Recovery PIN          : %d\n", sMRCfg->pinNumGpioRcvy);
        am_print_swo_uart_terminal("\tGPIO Recovery Polarity     : %d\n", sMRCfg->gpioRcvyPol);
    }
    else
    {
        am_print_swo_uart_terminal("\tGPIO MRAM Recovery         : Disabled.\n");
    }

    //
    // Check if Watchdog Timer is enabled
    //
    if (sMRCfg->enWDT)
    {
        am_print_swo_uart_terminal("\tWatchdog Timer             : Enabled with a period of 2 seconds.\n");
    }
    else
    {
        am_print_swo_uart_terminal("\tWatchdog Timer             : Disabled.\n");
    }

    //
    // Check if MRAM Recovery in progress output pin is configured
    //
    if (sMRCfg->gpioRcvyInprog != INVALID_PIN)
    {
        am_print_swo_uart_terminal("\tMRAM Recovery Progress     : Enabled.\n");
        am_print_swo_uart_terminal("\tMRAM Recovery Progress PIN : %d\n", sMRCfg->gpioRcvyInprog);
    }
    else
    {
        am_print_swo_uart_terminal("\tMRAM Recovery Progress     : Disbaled.\n");
    }

    if ( sMRCfg->rebootOnAppRecFail )
    {
        am_print_swo_uart_terminal("\tReboot on Recovery Fail    : Enabled.\n");
    }
    else
    {
        am_print_swo_uart_terminal("\tReboot on Recovery Fail    : Disabled.\n");
    }
}

//*****************************************************************************
//
//! Perform the initial CRC check of the application and store in the MRAM
//
//*****************************************************************************
bool __attribute__((section("RAMFUNC")))
am_crc_check_app(bool bStoreCRC)
{
    uint32_t ui32ComputedCRC;
    volatile uint32_t ui32CRCStored;
    am_hal_cachectrl_range_t cacheRange;
    //
    // MRAM changed so invalidate the cache range we just wrote
    //
    cacheRange.ui32StartAddr = (uint32_t) __pPatchable;
    cacheRange.ui32Size = CRC_PATCH_WORDS_NUM * 4;
    am_hal_cachectrl_dcache_invalidate(&cacheRange, false);

    uint32_t ui32ImageSize = (uint32_t) *((uint32_t*)(__pPatchable + 1));
    uint32_t ui32ImageStartOffSet = (uint32_t) *((uint32_t*)(__pPatchable));
    uint32_t ui32ImageStart;
    //
    // Fetch the start address of the image from the INFO0 if configured
    //
    if (am_hal_info0_valid())
    {
        am_hal_info0_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO0, AM_REG_INFO0_MAINPTR_O / 4, 1, &ui32ImageStart);
    }
    else
    {
        ui32ImageStart = 0x410000;
    }
    //
    // If the CRC is already running then turn it off since we need a new CRC run
    //
    if ( SECURITY->CTRL_b.ENABLE )
    {
        SECURITY->CTRL_b.ENABLE = 0x00;
        SECURITY->CTRL = 0x00;
    }

    //
    // Setup the CRC registers
    //
    am_hal_crc32(ui32ImageStart + ui32ImageStartOffSet , ui32ImageSize, &ui32ComputedCRC);

    //
    // MRAM changed so invalidate the cache range we just wrote
    //
    cacheRange.ui32StartAddr = (uint32_t) __pPatchable;
    cacheRange.ui32Size = CRC_PATCH_WORDS_NUM * 4;
    am_hal_cachectrl_dcache_invalidate(&cacheRange, false);

    //
    // Read the stored CRC from the MRAM
    //
    ui32CRCStored = (volatile uint32_t) *((uint32_t*)(__pPatchable + 2));
#ifdef AM_DEBUG_PRINTF
    am_print_swo_terminal("Calculated CRC = 0x%08X\n", ui32ComputedCRC);
    am_print_swo_terminal("Stored CRC = 0x%08X\n", ui32CRCStored);
#endif
    //
    // Compare the computed CRC with the stored CRC
    //
    return (ui32ComputedCRC == ui32CRCStored);
}

//*****************************************************************************
//
//! Print the WDT and CRC Count
//
//*****************************************************************************
void am_wdt_crc_status_print(void)
{
    am_print_swo_uart_terminal("Watchdog Timer Pet Count: %d\n", g_ui32WDTCnt);
    am_print_swo_uart_terminal("Application CRC Count: %d\n\n\n", g_ui32CRCCnt);
}

//*****************************************************************************
//
//! Trigger WDT to simulate a watchdog reset
//
//*****************************************************************************
void am_trigger_wdt_reset(void)
{
    //
    // Force a watchdog reset
    //
    am_print_swo_uart_terminal("Triggering Watchdog Reset.\n");
    g_btriggerWdtRst = 1;
    while(1);
}

//*****************************************************************************
//
//! Get MRAM Recovery Count
//
//*****************************************************************************
uint32_t am_mram_rcvy_count_get(bool bAmbiq)
{
    //
    // Update MRAM Recovery Count
    //
    uint32_t ui32MramRcvCount[2];
    uint32_t ui32Count = 0;

    if ( bAmbiq )
    {
        am_hal_info1_read(AM_HAL_INFO_INFOSPACE_OTP_INFO1, AM_OTP_INFO1_MRAM_RCVY_CNT_O, 2, ui32MramRcvCount);
    }
    else
    {
        am_hal_info0_read(AM_HAL_INFO_INFOSPACE_OTP_INFO0, AM_OTP_INFO0_MRAM_RCVY_CNT_O, 2, ui32MramRcvCount);
    }

    if (ui32MramRcvCount[0] == 0xFFFFFFFF)
    {
        ui32Count = 31;
        while (ui32MramRcvCount[1])
        {
            ui32Count++;
            ui32MramRcvCount[1] = ui32MramRcvCount[1] >> 1;
        }
    }
    else
    {
        while (ui32MramRcvCount[0])
        {
            ui32Count++;
            ui32MramRcvCount[0] = ui32MramRcvCount[0] >> 1;
        }
    }

    return ui32Count;
}

//*****************************************************************************
//
//! @brief  Updates OTP INFO1 MRAM Recovery Count registers
//!
//! @param  None
//!
//!
//! @return None
//
//*****************************************************************************
void am_update_mram_rcvy_count(void)
{
    //
    // Update MRAM Recovery Count
    //
    uint32_t ui32MramRcvCount[2];
    uint32_t ui32Status;
    am_hal_info0_read(AM_HAL_INFO_INFOSPACE_OTP_INFO0, AM_OTP_INFO0_MRAM_RCVY_CNT_O, 2, ui32MramRcvCount);

    //
    // Only Update the MRAM Recovery Count if we have space in OTP register i.e. Count < 63
    //
    if ( ui32MramRcvCount[1] != 0xFFFFFFFF )
    {
        uint32_t *targetRegister = (ui32MramRcvCount[0] != 0xFFFFFFFF) ? &ui32MramRcvCount[0] : &ui32MramRcvCount[1];

        //
        // Update the appropriate count register
        //
        *targetRegister = (*targetRegister << 1) | 0x1;

        ui32Status = am_hal_info0_program(AM_HAL_INFO_INFOSPACE_OTP_INFO0, AM_HAL_MRAM_INFO_KEY, (uint32_t*)ui32MramRcvCount, AM_OTP_INFO0_MRAM_RCVY_CNT_O, 2);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
            am_print_swo_terminal("Unsuccessful MRAM Recovery Count Write to OTP\n");
        }

    }
}

//*****************************************************************************
//
//! Force a hardfault (No MRAM Corruption)
//
//*****************************************************************************
void am_hardfault_no_corruption(void)
{
    am_print_swo_uart_terminal("Forcing a Hardfault\n");

    volatile uint32_t* pui32Addr = (uint32_t*)(0xFFFFFFFF);
    *pui32Addr = 0;
}
//*****************************************************************************
//
//! Force a hardfault (Force MRAM Corruption)
//
//*****************************************************************************
void am_hardfault_corruption(void)
{
    uint32_t ui32Status;
    am_print_swo_uart_terminal("Forcing Hardfault by corrupting MRAM\n");

    uint32_t ui32CorruptCRC[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

    //
    // Corrupt the CRC
    //
    ui32Status = am_hal_mram_main_program(AM_HAL_MRAM_PROGRAM_KEY, ui32CorruptCRC, (uint32_t*)((uint32_t)(&am_uart_activity) & ~0xF), 4);

    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_print_swo_uart_terminal("Programming fail\n");
    }
    //
    // Force hardfault
    //
    volatile uint32_t* pui32Addr = (uint32_t*)(0xFFFFFFFF);
    *pui32Addr = 0;
}

//*****************************************************************************
//
//! Print the MRAM Recovery Message
//
//*****************************************************************************
void am_print_mram_recovery_message(bool bReset)
{
    am_util_id_t sIdDevice;
    am_mram_recovery_status_t sMRAMRecoveryStatus;
    am_mram_recovery_cfg_t sMRCfg;

    //
    // Protection for MRAM recovery message
    //
    g_WDTMramMsg = true;
    {

        //
        // Clear out OTA pointer
        //
        MCUCTRL->OTAPOINTER = 0x00;

        //
        // Print the banner.
        //
        am_util_stdio_terminal_clear();
        am_print_swo_uart_terminal("\n**********************************************\n\n");
        am_print_swo_uart_terminal(" MRAM Recovery Application\n\n");
        am_print_swo_uart_terminal("**********************************************\n\n");

        //
        // Print MRAM Recovery happened
        //
        if ((RSTGEN->STAT & AM_AMBIQ_MRAM_RECOVERY_MASK) == AM_AMBIQ_MRAM_RECOVERY_MASK)
        {
            am_print_swo_uart_terminal("Ambiq MRAM Recovery completed successfully\n");
        }
        //
        // Update the OEM MRAM Recovery Count
        //
        if ( (RSTGEN->STAT & AM_OEM_MRAM_RECOVERY_MASK) == AM_OEM_MRAM_RECOVERY_MASK)
        {
            am_update_mram_rcvy_count();
            am_print_swo_uart_terminal("OEM MRAM Recovery completed successfully\n\n");
        }

        //
        // Print the Apollo device information.
        //
        am_print_swo_uart_terminal("DEVICE INFO\n");
        am_print_swo_uart_terminal("====================\n");
        am_util_id_device(&sIdDevice);
        am_print_swo_uart_terminal("  Vendor Name: %s\n", sIdDevice.pui8VendorName);
        am_print_swo_uart_terminal("  Device type: %s\n", sIdDevice.pui8DeviceName);
        am_print_swo_uart_terminal("  Device Info:\n"
                            "  \tPart number: 0x%08X\n"
                            "  \tChip ID0:    0x%08X\n"
                            "  \tChip ID1:    0x%08X\n"
                            "  \tRevision:    0x%08X (Rev%c%c)\r\n",
                            sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                            sIdDevice.sMcuCtrlDevice.ui32ChipID0,
                            sIdDevice.sMcuCtrlDevice.ui32ChipID1,
                            sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                            sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );

        am_print_swo_uart_terminal("MRAM RECOVERY STATUS\n");
        am_print_swo_uart_terminal("====================\n");

        //
        // Get the reset status and print it out.
        //
        am_print_reset_status();

        //
        // Get and print the MRAM Recovery Status.
        //
        am_mram_recovery_status(&sMRAMRecoveryStatus);

        //
        // Print the MRAM Recovery Count
        //
        am_print_swo_uart_terminal("Successful Ambiq MRAM Recovery Count = %d\n", am_mram_rcvy_count_get(true));
        am_print_swo_uart_terminal("Successful OEM MRAM Recovery Count = %d\n\n", am_mram_rcvy_count_get(false));

        //
        // INFO 0 setting banner
        //
        am_print_swo_uart_terminal("INFO0 SETTINGS\n");
        am_print_swo_uart_terminal("==============\n");

        //
        // Grab the INFO0 MRAM settings
        //
        if (am_hal_info0_valid())
        {
            am_print_swo_uart_terminal("INFO0 : Valid (");
            if ( MCUCTRL->SHADOWVALID_b.INFO0SELOTP )
            {
                am_print_swo_uart_terminal("OTP INFO0 Selected)\n\n");
            }
            else
            {
                am_print_swo_uart_terminal("MRAM INFO0 Selected)\n\n");
            }

            //
            // Read the current INFO0
            //
            am_hal_info0_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO0,
                                        AM_REG_INFO0_MRAM_RCVY_CTRL_O / 4,
                                        0xF,
                                        (uint32_t*)& sMRCfg);

            //
            // Check if main MRAM recovery is enabled
            //
            if (sMRCfg.mstrRcvyEnable == 0x6)
            {
                am_print_swo_uart_terminal("\tMain MRAM Recovery       : Enabled.\n");
                am_print_mram_recovery_config(&sMRCfg);
            }
            else
            {
                am_print_swo_uart_terminal("\tMain MRAM Recovery       : Disabled.\n");
            }
        }
        else
        {
            if ( !( MCUCTRL->SHADOWVALID_b.INFO0SELOTP ) )
            {
                am_print_swo_uart_terminal("INFO0 MRAM : Invalid (MRAM Recovery not enabled).\n");
            }
            else
            {
                am_print_swo_uart_terminal("INFO0 OTP  : Invalid (MRAM Recovery not enabled).\n");
            }
        }

        //
        // POR in 15 secs
        //
        am_print_uart_terminal("\n\nCountdown:");
        for ( int32_t i = 15; i >= 0; i-- )
        {
            am_print_uart_terminal(" %d", i);
            am_util_delay_ms(800);
            // For numbers >= 10 (two digits), erase both digits
            if (i >= 10)
            {
                am_print_uart_terminal("\b\b\b   \b\b\b");
            }
            // For numbers < 10 (one digit), erase only the single digit
            else
            {
                am_print_uart_terminal("\b\b  \b\b");
            }
        }
        am_util_delay_us(100);
        g_WDTMramMsg = false;
        if ( bReset )
        {
            RSTGEN->SWPOR = RSTGEN_SWPOR_SWPORKEY_KEYVALUE;
        }
    }
}
