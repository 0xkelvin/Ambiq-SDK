//*****************************************************************************
//
//! @file hello_world.c
//!
//! @brief A simple "Hello World" example.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup hello_world Hello World Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates fundamental device initialization and
//! configuration for Apollo microcontrollers. The application showcases basic
//! power management, peripheral setup, and debug interface configuration.
//! The example prints device information including device ID,
//! reset status, and configuration details over SWO at 1MHz for real-time
//! monitoring and debugging. After completing the initialization sequence,
//! the device transitions to sleep mode for power efficiency.
//!
//! @section hello_world_features Key Features
//!
//! 1. @b Basic @b Device @b Initialization: Demonstrates fundamental device
//!    setup including power management and peripheral configuration
//!
//! 2. @b SWO @b Communication: Uses SWO interface for debug output at 1MHz
//!    for real-time monitoring and debugging
//!
//! 3. @b Device @b Information: Displays device information
//!    including ID, reset status, and configuration details
//!
//! 4. @b Power @b Management: Implements low power initialization and
//!    demonstrates power management features
//!
//! 5. @b Debug @b Interface: Provides debugger support with SWO configuration
//!    for development and testing environments
//!
//! @section hello_world_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes low power configuration and board setup
//! - Configures OTP (One-Time Programmable) memory access
//! - Retrieves and displays device information
//! - Reads and reports device ID and reset status
//! - Configures SWO debug interface for real-time output
//! - Prints detailed device configuration and status information
//! - Transitions to sleep mode for power efficiency
//!
//! @section hello_world_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Configure SWO viewer with 1MHz rate and SWOClock = 1000
//! 3. Monitor SWO output for device information and status
//! 4. Verify successful device initialization and configuration
//!
//! @section hello_world_configuration Configuration
//!
//! - @b ENABLE_DEBUGGER: Enables debugger support and SWO interface
//! - @b AM_DEBUG_PRINTF: Enables detailed debug output via SWO
//! - @b SWO_BAUD_RATE: Configurable SWO communication rate (default: 1MHz)
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
#define ENABLE_DEBUGGER

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
    // Initialize the printf interface for ITM output
    //
    if ( am_bsp_debug_printf_enable() != 0 )
    {
        //
        // Cannot print - so no point proceeding
        //
        while(1);
    }

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

#ifdef AM_PART_APOLLO330P_510L
    //
    // Print information about the Ambiq CM4 image
    //
    uint32_t ui32ImgInfo[4];
    ui32Ret = am_hal_info1_read(AM_HAL_INFO_INFOSPACE_MRAM_INFO1, AM_REG_INFO1_EXTIMGTYPE0_O / 4, 4, &ui32ImgInfo[0]);
    if ( ui32Ret == AM_HAL_STATUS_SUCCESS )
    {
        if ( AM_INFO_FLD2VAL(1, EXTIMGTYPE0_CM4IMAGE, ui32ImgInfo[0]) )
        {
            char *psImgType;
            if ( AM_INFO_FLD2VAL(1, EXTIMGTYPE0_TYPE, ui32ImgInfo[0]) == 0 )
            {
                psImgType = "None";
            }
            else if ( AM_INFO_FLD2VAL(1, EXTIMGTYPE0_TYPE, ui32ImgInfo[0]) == 1 )
            {
                psImgType = "XtalHF_only";
            }
            else if ( AM_INFO_FLD2VAL(1, EXTIMGTYPE0_TYPE, ui32ImgInfo[0]) == 2 )
            {
                psImgType = "BLE";
            }
            else
            {
                psImgType = "Unknown";
            }

            am_util_stdio_printf("\nAmbiq CM4 image type: %d (%s)\n",
                AM_INFO_FLD2VAL(1, EXTIMGTYPE0_TYPE, ui32ImgInfo[0]), psImgType);

            am_util_stdio_printf("Ambiq CM4 image version %d.%d.%d\n",
                AM_INFO_FLD2VAL(1, EXTIMGVERSION0_VERMAJOR, ui32ImgInfo[1]),
                AM_INFO_FLD2VAL(1, EXTIMGVERSION0_VERMINOR, ui32ImgInfo[1]),
                AM_INFO_FLD2VAL(1, EXTIMGVERSION0_VERBLD, ui32ImgInfo[1]) );

            am_util_stdio_printf("Ambiq CM4 image installed at: 0x%08X - 0x%08X\n",
                ui32ImgInfo[2], ui32ImgInfo[3]);
        }
    }
    else
    {
        am_util_stdio_printf("Failed to get Ambiq CM4 IMG info\n");
    }
#endif

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
    // Disable debug printf messages on ITM.
    //
    am_util_stdio_printf("Done with prints. Entering While loop\n");
    am_bsp_debug_printf_disable();

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

