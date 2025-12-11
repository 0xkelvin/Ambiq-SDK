//*****************************************************************************
//
//! @file hello_world_oem_recovery.c
//!
//! @brief The OEM_recovery application
//!
//! @addtogroup mram_recovery_examples OEM Recovery Image Examples
//!
//! @defgroup hello_world_oem_recovery OEM Recovery Image Example
//! @ingroup mram_recovery_examples
//! @{
//!
//! Purpose: This example demonstrates MRAM (Magnetic Random
//! Access Memory) recovery functionality with fault tolerance and
//! recovery capabilities. The application showcases recovery
//! mechanisms, WDT (Watchdog Timer) integration, active CRC
//! verification, and relocated fault handlers.
//!
//! @section hello_world_oem_recovery_features Key Features
//!
//! 1. @b MRAM @b Recovery: Implements MRAM recovery
//!    functionality for fault-tolerant applications
//!
//! 2. @b WDT @b Integration: Provides Watchdog Timer integration for
//!    reliable fault detection and recovery initiation
//!
//! 3. @b Active @b CRC @b Verification: Implements active CRC verification
//!    for data integrity and recovery validation
//!
//! 4. @b Relocated @b Fault @b Handlers: Provides relocated fault handlers
//!    for enhanced fault detection and recovery management
//!
//! 5. @b Recovery @b Information @b Display: Implements recovery information
//!    display for system status monitoring and debugging
//!
//! @section hello_world_oem_recovery_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes MRAM recovery functionality with WDT integration
//! - Implements active CRC verification for data integrity
//! - Provides relocated fault handlers for enhanced fault detection
//! - Manages recovery information display and status monitoring
//! - Supports recovery mechanisms
//! - Implements MRAM recovery features
//!
//! @section hello_world_oem_recovery_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor UART output for recovery operations and status
//! 3. Test fault detection and recovery initiation
//! 4. Verify CRC verification and data integrity
//! 5. Observe recovery information display functionality
//!
//! @section hello_world_oem_recovery_configuration Configuration
//!
//! - @b WDT: Watchdog Timer configuration for fault detection
//! - @b CRC @b Verification: Active CRC verification setup
//! - @b Fault @b Handlers: Relocated fault handler configuration
//! - @b Recovery @b Display: Recovery information display settings
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

#define AM_MRAM_FILL_CRC2_ADDRESS       (0x4D0000)
#define AM_MRAM_FILL_CRC2_SIZE_BYTES    (MRAM_MAX_SIZE - (AM_MRAM_FILL_CRC2_ADDRESS - MRAM_BASEADDR))

int32_t g_UARTActCount = 0;
uint32_t *g_pOriginalVTOR;
uint32_t gSramVT[256] __attribute__((aligned (1024) ));
uint32_t g_CRCStartAddr;
uint32_t g_CRCSize;
uint32_t g_ExpectedCRC;
uint32_t g_ExpectedCRC2;

//*****************************************************************************
//
// Print the hello world message
//
//*****************************************************************************
void am_print_hello_world(void)
{
    am_util_id_t sIdDevice;
    uint32_t ui32StrBuf;
    am_hal_reset_status_t sResetStatus;
    uint32_t ui32Ret;

    //
    // Print the banner.
    //
    am_print_swo_uart_terminal("\n\n\n\n\n\n\n\n\n\n\n");
    am_print_swo_uart_terminal("Hello World! { MRAM Recovery Example }\n\n");
    am_print_uart_terminal("Press H [Enter] for Help.\n\n======================================\n\n");

    //
    // Print the device info.
    //
    am_util_id_device(&sIdDevice);
    am_print_swo_terminal("  Vendor Name: %s\n", sIdDevice.pui8VendorName);
    am_print_swo_terminal("  Device type: %s\n", sIdDevice.pui8DeviceName);
    am_print_swo_terminal("  Device Info:\n"
                        "  \tPart number: 0x%08X\n"
                        "  \tChip ID0:    0x%08X\n"
                        "  \tChip ID1:    0x%08X\n"
                        "  \tRevision:    0x%08X (Rev%c%c)\r\n",
                        sIdDevice.sMcuCtrlDevice.ui32ChipPN,
                        sIdDevice.sMcuCtrlDevice.ui32ChipID0,
                        sIdDevice.sMcuCtrlDevice.ui32ChipID1,
                        sIdDevice.sMcuCtrlDevice.ui32ChipRev,
                        sIdDevice.ui8ChipRevMaj, sIdDevice.ui8ChipRevMin );

    //
    // If not a multiple of 1024 bytes, append a plus sign to the KB.
    //
    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32MRAMSize % 1024 ) ? '+' : 0;
    am_print_swo_terminal("\tMRAM size:   %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32MRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32MRAMSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32DTCMSize % 1024 ) ? '+' : 0;
    am_print_swo_terminal("\tDTCM size:   %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32DTCMSize,
                         sIdDevice.sMcuCtrlDevice.ui32DTCMSize / 1024,
                         &ui32StrBuf);

    ui32StrBuf = ( sIdDevice.sMcuCtrlDevice.ui32SSRAMSize % 1024 ) ? '+' : 0;
    am_print_swo_terminal("\tSSRAM size:  %7d (%d KB%s)\n",
                         sIdDevice.sMcuCtrlDevice.ui32SSRAMSize,
                         sIdDevice.sMcuCtrlDevice.ui32SSRAMSize / 1024,
                         &ui32StrBuf);

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
    am_print_swo_terminal("App Compiler:    %s\n", COMPILER_VERSION);
    am_print_swo_terminal("HAL Compiler:    %s\n", g_ui8HALcompiler);
    am_print_swo_terminal("HAL SDK version: %d.%d.%d\n",
                         g_ui32HALversion.s.Major,
                         g_ui32HALversion.s.Minor,
                         g_ui32HALversion.s.Revision);
    am_print_swo_terminal("HAL compiled with %s-style registers\n",
                         g_ui32HALversion.s.bAMREGS ? "AM_REG" : "CMSIS");
    am_print_swo_terminal("\n");

    am_print_swo_terminal("SECURITY INFO\n");
    am_print_swo_terminal("=============\n");

    am_hal_reset_status_get(&sResetStatus);
    am_print_swo_terminal("Reset Status:    0x%X\n", sResetStatus.eStatus);

    bool bInfo0Valid = am_hal_info0_valid();
    am_hal_security_socid_t socId;
    uint32_t ui32Var;
    uint32_t ui32dcuVal;
    uint32_t lcs = CRYPTO->LCSREG_b.LCSREG;
    am_print_swo_terminal("Device LCS: %s\n",
                         ((lcs == 0) ? "CM" :       \
                         ((lcs == 1) ? "DM" :       \
                         ((lcs == 5) ? "Secure" :   \
                         ((lcs == 7) ? "RMA" : "Undefined")))));
    am_hal_dcu_get(&ui32dcuVal);
    am_hal_security_get_socid(&socId);
    am_print_swo_terminal("\tSOC Id:\n\t0x%08X : 0x%08X : 0x%08X : 0x%08X\n\t0x%08X : 0x%08X : 0x%08X : 0x%08X\n",
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
            am_print_swo_terminal("Secure Debug Certificate Location: 0x%08X\n", ui32Var);
        }
        else
        {
            am_print_swo_terminal("Error - could not retrieve Secure Debug Certificate Location from INFO1.\n");
        }
    }

    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_CURRENT_INFO1, AM_REG_OTP_INFO1_SBR_OPT_ADDR_O / 4, 1, &ui32Var);
    if ( (ui32Var != 0x00000000)        &&
         (ui32Ret == AM_HAL_STATUS_SUCCESS) )
    {
        am_print_swo_terminal("SBR OPT Address: 0x%08X\n", ui32Var);
    }
    else
    {
        am_print_swo_terminal("Error - could not retrieve OTP Address from INFO1.\n");
    }
    am_print_swo_terminal("\n");

#ifdef ENABLE_DEBUGGER
    {
        // Enable Debugger
        if (am_hal_dcu_update(true, DCU_DEBUGGER) != AM_HAL_STATUS_SUCCESS)
        {
            // Cannot enable Debugger
            am_print_swo_terminal("Could not enable debugger using DCU\n");
        }
        else
        {
            am_print_swo_terminal("Forcibly enabling debugger using DCU\n\n");
        }
    }
    MCUCTRL->DEBUGGER = 0;
#endif // ENABLE_DEBUGGER
    if ((PWRCTRL->DEVPWRSTATUS_b.PWRSTCRYPTO == 1) && (CRYPTO->HOSTCCISIDLE_b.HOSTCCISIDLE == 1))
    {
        am_print_swo_terminal("DEBUG INFO\n");
        am_print_swo_terminal("==========\n");
        am_print_swo_terminal("Original Qualified DCU Val   0x%x\n", ui32dcuVal);
        am_hal_dcu_get(&ui32dcuVal);
        am_print_swo_terminal("Current  Qualified DCU Val   0x%x\n", ui32dcuVal);
        am_hal_dcu_lock_status_get(&ui32dcuVal);
        am_print_swo_terminal("Qualified DCU Lock Val       0x%x\n", ui32dcuVal);
        DIAG_SUPPRESS_VOLATILE_ORDER()
        am_print_swo_terminal("\tRaw DCU Enable: 0x%08X : 0x%08X : 0x%08X : 0x%08X\n",
                             CRYPTO->HOSTDCUEN0, CRYPTO->HOSTDCUEN1, CRYPTO->HOSTDCUEN2, CRYPTO->HOSTDCUEN3);
        am_print_swo_terminal("\tRaw DCU Lock  : 0x%08X : 0x%08X : 0x%08X : 0x%08X\n",
                             CRYPTO->HOSTDCULOCK0, CRYPTO->HOSTDCULOCK1, CRYPTO->HOSTDCULOCK2, CRYPTO->HOSTDCULOCK3);
        DIAG_DEFAULT_VOLATILE_ORDER()
        am_print_swo_terminal("MSPLIM:  0x%08X\n", __get_MSPLIM());
        am_print_swo_terminal("PSPLIM:  0x%08X\n", __get_PSPLIM());
        am_print_swo_terminal("\n");
    }

    //
    // Display Ambiq & OEM MRAM Recovery Count
    //
    am_print_swo_terminal("MRAM RECOVERY INFO\n==================\n");
    am_print_swo_terminal("Successful Ambiq MRAM Recovery Count = %d\n", am_mram_rcvy_count_get(true));
    am_print_swo_terminal("Successful OEM MRAM Recovery Count = %d\n", am_mram_rcvy_count_get(false));
}
#define ACTIVITY_INTERVAL       960000 * 2
void am_uart_activity(void)
{
    if (g_UARTActCount > ACTIVITY_INTERVAL)
    {
        am_print_uart_terminal("WDT:%4d, CRC:%4d > ", g_ui32WDTCnt, g_ui32CRCCnt);
        g_UARTActCount = 0;
    }
    g_UARTActCount++;
}

//*****************************************************************************
//
// Fault Handler
//
//*****************************************************************************
void __attribute__((section("RAMFUNC")))
am_secboot_mram_fault_handler( void )
{
    uint32_t ui32Exception;

    //
    // Stop the WDT
    //
    am_hal_wdt_stop(AM_HAL_WDT_MCU);

    //
    // Print
    //
    am_print_swo_terminal("Hardfault occured, Performing a CRC.\n\n");
    //
    // Check for CRC and recover if necessary
    //
    if ( !am_crc_check_app(false) || !crc2_check() )
    {
        //
        //Trigger the Application Recovery
        //
        am_hal_mram_recovery_init_app_recovery(AM_HAL_MRAM_RECOVERY_KEY, false);

        //
        // Application is corrupted.
        //
        am_print_swo_terminal("CRC Check failed.\nInitiating Application MRAM Recovery.\n");

        //
        // POI
        //
        RSTGEN->SWPOI = RSTGEN_SWPOI_SWPOIKEY_KEYVALUE;
    }

    //
    // CRC check passed. Get the IPSR
    //
    ui32Exception = __get_IPSR();

    //
    // Call the resp. exception handler
    //
    ((VECTOR_TABLE_Type) *(g_pOriginalVTOR + ui32Exception))();
}

//*****************************************************************************
//
// Print the hello world message
//
//*****************************************************************************
void am_relocate_vector_table( void )
{
    //
    // Initialize the CRC variables
    //
    uint32_t* pPatch = (uint32_t*)(__pPatchable);
    g_CRCStartAddr  = (uint32_t)*pPatch;
    g_CRCSize       = (uint32_t)*(pPatch + 1);
    g_ExpectedCRC   = (uint32_t)*(pPatch + 2);

    //
    // Read the original VTOR value
    //
    g_pOriginalVTOR = (uint32_t*)(AM_REGVAL(ARM_VTOR_ADDRESS));

    //
    // Copy the original vector table
    //
    for ( uint32_t itr = 0; itr < (sizeof(__VECTOR_TABLE)) / 4; itr++ )
    {
        gSramVT[itr] = (uint32_t)*(g_pOriginalVTOR + itr);
    }

    //
    // Initialize the fault handlers to the MRAM recovery fault handler
    // Following handlers are initialized:
    // HARDFAULT
    //
    gSramVT[3] = (uint32_t)am_secboot_mram_fault_handler;

    //
    // Relocate the VTOR
    //
    AM_REGVAL(ARM_VTOR_ADDRESS) = (uint32_t)&gSramVT;
}

//*****************************************************************************
//
//! CRC2 Check Function
//
//*****************************************************************************
bool __attribute__((section("RAMFUNC")))
crc2_check(void)
{
    uint32_t ui32ComputedCRC;
        //
    // If the CRC is already running then turn it off since we need a new CRC run
    //
    if ( SECURITY->CTRL_b.ENABLE )
    {
        SECURITY->CTRL_b.ENABLE = 0x00;
        SECURITY->CTRL = 0x00;
    }
    //
    // Service the Watchdog since we are sweeping the whole MRAM to generate the CRC
    //
    am_hal_crc32(AM_MRAM_FILL_CRC2_ADDRESS, AM_MRAM_FILL_CRC2_SIZE_BYTES, &ui32ComputedCRC);

    return (ui32ComputedCRC == g_ExpectedCRC2);
}
//*****************************************************************************
//
//! Main
//
//*****************************************************************************
int main(void)
{
    uint32_t ui32Status;
    bool bOTPEnabled;
    uint32_t activityCount = 0;

    //
    // Relocate Vector Tables
    //
    am_relocate_vector_table();

    //
    // Initialize the Watchdog Timer
    //
    if ( !(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) )
    {
        am_watchdog_init();
    }

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Check and Power on OTP if it is not already on.
    //
    ui32Status = am_hal_pwrctrl_periph_enabled(AM_HAL_PWRCTRL_PERIPH_OTP, &bOTPEnabled);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error during read of OTP power status\n");
    }

    if (!bOTPEnabled)
    {
        ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_OTP);
        if (AM_HAL_STATUS_SUCCESS != ui32Status)
        {
            am_util_stdio_printf("Error in power up of OTP module\n");
        }
    }

    //
    // Enable the Crypto module
    //
    ui32Status = am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_CRYPTO);
    if (AM_HAL_STATUS_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Error in power up of Crypto module\n");
    }

    //
    // Fill the whole MRAM for the secondary CRC
    //
    am_hal_mram_main_fill(AM_HAL_MRAM_PROGRAM_KEY,
                          0xA5A5A5A5,
                          (uint32_t*)AM_MRAM_FILL_CRC2_ADDRESS,
                          AM_MRAM_FILL_CRC2_SIZE_BYTES / 4);

    //
    // Run an initial CRC to store the reference value
    //
    am_hal_crc32(AM_MRAM_FILL_CRC2_ADDRESS, AM_MRAM_FILL_CRC2_SIZE_BYTES, &g_ExpectedCRC2);

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Initialize the printf interface for ITM output
    //
    if (am_bsp_debug_printf_enable())
    {
        // Cannot print - so no point proceeding
        while(1);
    }

    //
    // Initialize the printf interface for UART output
    //
    UART_printf_init();

    //
    // Initialize the STIMER to trigger CRC check every 10 seconds
    //
    am_stimer_init();
    //
    // Print hello world message if no recovery
    //
    //
    if ( (RSTGEN->STAT & AM_MRAM_RECOVERY_MASK) != 0x0 || RSTGEN->STAT_b.WDRSTAT )
    {
        //
        // Their has been a recovery, so print the recovery message
        // Note: Does not return
        //
        am_print_mram_recovery_message(true);

    }
    else
    {
        //
        // Print the hello world message
        //
        am_print_hello_world();

        //
        // SWO wait banner
        //
        am_print_swo_terminal("\nWaiting for UART command\n\n");
    }

    //
    // Enable the master Interrupt
    //
    am_hal_interrupt_master_enable();

    //
    // Handle the ISR flags for WDT, UART, and STIMER
    //
    bool bCmdDone = true;
    while(true)
    {
        //
        // We are expecting a command from the user. Call the command handler
        //
        if ( !UARTn(AM_BSP_UART_PRINT_INST)->FR_b.RXFE )
        {
            bCmdDone = cmdLine(AM_BSP_UART_PRINT_INST);
            g_UARTActCount = 0;
        }
        if (bCmdDone)
        {
            am_uart_activity();
            activityCount++;
            if ( activityCount > ACTIVITY_INTERVAL / 2 )
            {
                int32_t i32Temp = 21;
                while( i32Temp )
                {
                    am_print_uart_terminal("\b \b");
                    i32Temp--;
                }
                g_UARTActCount = ACTIVITY_INTERVAL;
                activityCount = 0;
            }
        }
        //
        // Check if the WDT or CRC flag is set
        //
        if (g_ui32WDTFlag)
        {
            g_ui32WDTFlag = 0;
            am_hal_wdt_restart(AM_HAL_WDT_MCU);
        }

        //
        // Check if the CRC flag is set
        //
        if ( g_ui32CRCFlag )
        {
            g_ui32CRCFlag = 0;
            //
            // Perform the CRC check of the application
            //
            if ( !am_crc_check_app(false) || !crc2_check() )
            {
                //
                // CRC failure encountered
                // Set the flag and let the WDT perform an Application triggered
                // MRAM recovery
                //
                g_bCRCFail = true;
                while(1);
            }

        }
    }

    //
    // We are done printing.
    // Disable the UART and interrupts
    //
    am_print_swo_terminal("Done with prints. Entering While loop\n");
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
