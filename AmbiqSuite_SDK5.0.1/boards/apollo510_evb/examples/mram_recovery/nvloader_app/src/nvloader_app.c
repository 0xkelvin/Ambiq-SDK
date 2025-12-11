//*****************************************************************************
//
//! @file nvloader_app.c
//!
//! @brief Loads preloaded buffer into the MPSI/eMMC memory
//!
//! @addtogroup mram_recovery_examples MRAM Recovery Examples
//!
//! @defgroup nvloader_app NV Loader Example
//! @ingroup mram_recovery_examples
//! @{
//!
//! Purpose: This example demonstrates NV Loader (Non-Volatile
//! Loader) functionality for loading preloaded buffer into MSPI/eMMC memory.
//! The application showcases memory loading capabilities, MSPI
//! flash operations, eMMC memory management, and recovery image handling.
//!
//! @section nvloader_app_features Key Features
//!
//! 1. @b NV @b Loader @b Functionality: Implements NV Loader
//!    functionality for external memory loading operations
//!
//! 2. @b MSPI @b Flash @b Support: Provides MSPI flash memory operations
//!    for reliable external memory management
//!
//! 3. @b eMMC @b Memory @b Support: Implements eMMC memory management
//!    for high-capacity external storage operations
//!
//! 4. @b Recovery @b Image @b Loading: Handles recovery image loading
//!    for system recovery and restoration capabilities
//!
//! 5. @b Memory @b Verification: Implements memory verification capabilities
//!    for reliable data loading and validation
//!
//! @section nvloader_app_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes NV Loader functionality for external memory operations
//! - Implements MSPI flash memory loading and verification
//! - Provides eMMC memory management and partition handling
//! - Manages recovery image loading and validation
//! - Supports memory verification capabilities
//! - Implements NV Loader features
//!
//! @section nvloader_app_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor ITM/SWO output for NV Loader operations and status
//! 3. Test MSPI flash and eMMC memory loading operations
//! 4. Verify recovery image loading and validation
//! 5. Evaluate memory verification and recovery capabilities
//!
//! @section nvloader_app_configuration Configuration
//!
//! - @b ITM/SWO: Output for debug messages (1MHz)
//! - @b MSPI @b Flash: MSPI flash memory configuration and operations
//! - @b eMMC @b Memory: eMMC memory management and partition setup
//! - @b Recovery @b Image: Recovery image loading and validation settings
//!
//! Additional Information:
//! Debug messages will be sent over ITM/SWO at 1MHz.<br>
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

#ifdef AM_BSP_MSPI_FLASH_PRESENT
#define AM_BSP_MSPI_FLASH_DEVICE_IS25WX064
#endif

#include "am_util.h"
#include "am_mcu_apollo.h"
#include "nvloader_app.h"
//
// Undefine AM_DEBUG_PRINTF to disable all printing.
//
//#define AM_DEBUG_PRINTF

//*****************************************************************************
//
//! Define the location of the recovery information structure (defined below).
//
//*****************************************************************************
#define PRGM_IMG_DATA_ADDR             (0x20100000)
#define CFGDATA_META_SIZE              16
#define DEFAULT_TIMEOUT                10000
AM_SHARED_RW uint8_t    g_MetadataBuffer[CFGDATA_META_SIZE] __attribute__((aligned(32)));

//*****************************************************************************
//
//! Macro for BKPT instruction
//
//*****************************************************************************
#if (defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)) || defined(__GNUC_STDC_INLINE__)
#define     AM_ASM_BKPT(n)  __asm("    bkpt "#n)
#elif defined(__IAR_SYSTEMS_ICC__)
#define     AM_ASM_BKPT(n)  asm("    bkpt "#n)
#else
#error Compiler unknown, cannot define BKPT instruction.
#endif

#ifdef AM_BSP_MSPI_FLASH_PRESENT
#define MSPI_TEST_FREQ          AM_HAL_MSPI_CLK_96MHZ
#endif

//*****************************************************************************
//
//! eMMC Global Variables & Defines
//
//*****************************************************************************
am_hal_card_host_t *pSdhcCardHost = NULL;

#ifdef AM_BSP_MSPI_FLASH_PRESENT
//*****************************************************************************
//
//! MSPI Global Variables & Defines
//
//*****************************************************************************
#define MSPI_TARGET_SECTOR      (0)
#define MSPI_BUFFER_SIZE        (1 << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT)
AM_SHARED_RW uint32_t       DMATCBBuffer[2560];
AM_SHARED_RW uint8_t        g_SectorBuffer[MSPI_BUFFER_SIZE] __attribute__((aligned(32)));
void            *g_FlashHdl;
void            *g_MSPIHdl;
extern am_hal_mspi_xip_config_t gXipConfig[];
const am_hal_mspi_clock_e eClockFreq = MSPI_TEST_FREQ;
am_abstract_mspi_devices_timing_config_t MSPISdrTimingConfig;

//*****************************************************************************
//
//! MSPI Configuration Lookup Table
//
//*****************************************************************************
am_hal_mspi_device_e deviceConfigLookup[] = {
    AM_HAL_MSPI_FLASH_SERIAL_CE0,        // AM_CHIPSEL_MSPI_CE0_MODE0
    AM_HAL_MSPI_FLASH_DUAL_CE0_1_1_2,    // AM_CHIPSEL_MSPI_CE0_MODE1
    AM_HAL_MSPI_FLASH_DUAL_CE0_1_2_2,    // AM_CHIPSEL_MSPI_CE0_MODE2
    AM_HAL_MSPI_FLASH_QUAD_CE0_1_1_4,    // AM_CHIPSEL_MSPI_CE0_MODE3
    AM_HAL_MSPI_FLASH_QUAD_CE0_1_4_4,    // AM_CHIPSEL_MSPI_CE0_MODE4
    AM_HAL_MSPI_FLASH_OCTAL_CE0_1_1_8,   // AM_CHIPSEL_MSPI_CE0_MODE5
    AM_HAL_MSPI_FLASH_OCTAL_CE0_1_8_8,   // AM_CHIPSEL_MSPI_CE0_MODE6
    AM_HAL_MSPI_FLASH_DUAL_CE0,          // AM_CHIPSEL_MSPI_CE0_MODE7
    AM_HAL_MSPI_FLASH_QUAD_CE0,          // AM_CHIPSEL_MSPI_CE0_MODE8
    AM_HAL_MSPI_FLASH_OCTAL_DDR_CE0,     // AM_CHIPSEL_MSPI_CE0_MODE9
    AM_HAL_MSPI_FLASH_MAX,               // Not used
    AM_HAL_MSPI_FLASH_MAX,
    AM_HAL_MSPI_FLASH_MAX,
    AM_HAL_MSPI_FLASH_MAX,
    AM_HAL_MSPI_FLASH_MAX,
    AM_HAL_MSPI_FLASH_MAX,
    AM_HAL_MSPI_FLASH_SERIAL_CE1,        // AM_CHIPSEL_MSPI_CE1_MODE0
    AM_HAL_MSPI_FLASH_DUAL_CE1_1_1_2,    // AM_CHIPSEL_MSPI_CE1_MODE1
    AM_HAL_MSPI_FLASH_DUAL_CE1_1_2_2,    // AM_CHIPSEL_MSPI_CE1_MODE2
    AM_HAL_MSPI_FLASH_QUAD_CE1_1_1_4,    // AM_CHIPSEL_MSPI_CE1_MODE3
    AM_HAL_MSPI_FLASH_QUAD_CE1_1_4_4,    // AM_CHIPSEL_MSPI_CE1_MODE4
    AM_HAL_MSPI_FLASH_OCTAL_CE1_1_1_8,   // AM_CHIPSEL_MSPI_CE1_MODE5
    AM_HAL_MSPI_FLASH_OCTAL_CE1_1_8_8,   // AM_CHIPSEL_MSPI_CE1_MODE6
    AM_HAL_MSPI_FLASH_DUAL_CE1,          // AM_CHIPSEL_MSPI_CE1_MODE7
    AM_HAL_MSPI_FLASH_QUAD_CE1,          // AM_CHIPSEL_MSPI_CE1_MODE8
    AM_HAL_MSPI_FLASH_OCTAL_DDR_CE1,     // AM_CHIPSEL_MSPI_CE1_MODE9
};

//*****************************************************************************
//
//! MSPI Device Function Table
//
//*****************************************************************************
mspi_device_func_t mspi_device_func =
{
#if defined(AM_BSP_MSPI_FLASH_DEVICE_IS25WX064)
    .devName = "MSPI FLASH IS25WX064",
    .mspi_init = am_devices_mspi_is25wx064_init,
    .mspi_init_timing_check = am_devices_mspi_is25wx064_init_timing_check,
    .mspi_init_timing_apply = am_devices_mspi_is25wx064_apply_ddr_timing,
    .mspi_term = am_devices_mspi_is25wx064_deinit,
    .mspi_read_id = am_devices_mspi_is25wx064_id,
    .mspi_read = am_devices_mspi_is25wx064_read,
    .mspi_read_adv = am_devices_mspi_is25wx064_read_adv,
    .mspi_write = am_devices_mspi_is25wx064_write,
    .mspi_mass_erase = am_devices_mspi_is25wx064_mass_erase,
    .mspi_sector_erase = am_devices_mspi_is25wx064_sector_erase,
    .mspi_xip_enable = am_devices_mspi_is25wx064_enable_xip,
    .mspi_xip_disable = am_devices_mspi_is25wx064_disable_xip,
    .mspi_scrambling_enable = am_devices_mspi_is25wx064_enable_scrambling,
    .mspi_scrambling_disable = am_devices_mspi_is25wx064_disable_scrambling,
    .mspi_change_nv_cfg_default = am_devices_mspi_is25wx064_change_nvconfig_default,
    .mspi_change_nv_iomode = am_devices_mspi_is25wx064_change_nv_iomode,
#else
    .devName = "MSPI FLASH ATXP032",
    .mspi_init = am_devices_mspi_atxp032_init,
    .mspi_init_timing_check = am_devices_mspi_atxp032_sdr_init_timing_check,
    .mspi_init_timing_apply = am_devices_mspi_atxp032_apply_sdr_timing,
    .mspi_term = am_devices_mspi_atxp032_deinit,
    .mspi_read_id = am_devices_mspi_atxp032_id,
    .mspi_read = am_devices_mspi_atxp032_read,
    .mspi_read_adv = am_devices_mspi_atxp032_read_adv,
    .mspi_write = am_devices_mspi_atxp032_write,
    .mspi_mass_erase = am_devices_mspi_atxp032_mass_erase,
    .mspi_sector_erase = am_devices_mspi_atxp032_sector_erase,
    .mspi_xip_enable = am_devices_mspi_atxp032_enable_xip,
    .mspi_xip_disable = am_devices_mspi_atxp032_disable_xip,
    .mspi_scrambling_enable = am_devices_mspi_atxp032_enable_scrambling,
    .mspi_scrambling_disable = am_devices_mspi_atxp032_disable_scrambling,
#endif
};

//*****************************************************************************
//
//! MSPI Device Configuration
//
//*****************************************************************************
am_abstract_mspi_devices_config_t MSPI_Flash_Config =
{
#if defined(AM_BSP_MSPI_FLASH_DEVICE_IS25WX064)
    .eDeviceConfig = AM_HAL_MSPI_FLASH_SERIAL_CE0,
#else
    .eDeviceConfig = AM_HAL_MSPI_FLASH_OCTAL_CE0,
#endif

    .eClockFreq = MSPI_TEST_FREQ,
    .pNBTxnBuf = DMATCBBuffer,
    .ui32NBTxnBufLength = (sizeof(DMATCBBuffer) / sizeof(uint32_t)),
    .ui32ScramblingStartAddr = 0,
    .ui32ScramblingEndAddr = 0x10000000,
};

//*****************************************************************************
//
//! MSPI interrupts.
//
//*****************************************************************************
static const IRQn_Type mspi_interrupts[] =
{
    MSPI0_IRQn,
    MSPI1_IRQn,
    MSPI2_IRQn,

#if !defined(AM_PART_APOLLO330P_510L) && !defined(AM_PART_APOLLO510)
    MSPI3_IRQn,
#endif

};

//*****************************************************************************
//
// MSPI ISRs. Sure, map all of them because we choose 1 at runtime
//
//*****************************************************************************
void am_mspi0_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIHdl, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIHdl, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIHdl, ui32Status);
}

void am_mspi1_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIHdl, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIHdl, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIHdl, ui32Status);
}

void am_mspi2_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIHdl, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIHdl, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIHdl, ui32Status);
}


#if !defined(AM_PART_APOLLO330P_510L) && !defined(AM_PART_APOLLO510)

void am_mspi3_isr(void)
{
    uint32_t      ui32Status;

    am_hal_mspi_interrupt_status_get(g_MSPIHdl, &ui32Status, false);

    am_hal_mspi_interrupt_clear(g_MSPIHdl, ui32Status);

    am_hal_mspi_interrupt_service(g_MSPIHdl, ui32Status);
}
#endif

/*
 * Erase, program, verify MSPI.
 *
 * If the contents of the buffer match what's already in flash, do nothing. If the buffer
 * does not span the entire sector, the remainder of the sector is preserved and written
 * back along with the buffer after erasing.
 *
 * buffer: pointer to a buffer to write to MSPI
 * flash_start: byte offset into the MSPI flash for the buffer contents
 * length: length of buffer in bytes
 * verify_only: If True, only reads the sector and compares against the buffer. No erase or write is performed
 *
 * Returns  0 if the write and verify (or verify if verify_only) are successful
 *          Number of bytes that failed to verify if not
 *         -1 if there was an error communicating with the MSPI device
 */
int
write_verify(uint8_t* buffer, uint32_t flash_start, uint32_t length, bool verify_only)
{
    uint32_t bad_bytes = 0;
    uint32_t sector, sector_start_offset, sector_length;
    uint8_t* local_buffer = buffer;
    uint32_t remaining = length;
    uint32_t flash_index = flash_start;
    uint32_t ui32Status;
    uint32_t compare_error = false;
    uint32_t already_programmed = true;

    am_hal_cachectrl_range_t sRange;

    while(remaining > 0)
    {
        sector = flash_index >> AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT;
        sector_start_offset = flash_index - (sector << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT);
        sector_length = (remaining > (1 << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT) - sector_start_offset) ?
                        (1 << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT) - sector_start_offset:
            remaining;

        if ( !verify_only )
        {
            sRange.ui32StartAddr = (uint32_t)g_SectorBuffer;
            sRange.ui32Size = MSPI_BUFFER_SIZE;
            am_hal_cachectrl_dcache_clean(&sRange);
            // First check that the sector doesn't already contain the contents we expect
            ui32Status = mspi_device_func.mspi_read(g_FlashHdl,
                                                    g_SectorBuffer,
                                                    sector << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT,
                                                    1 << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT,
                                                    true);
            if ( AM_ABSTRACT_MSPI_SUCCESS != ui32Status )
            {
                am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
                return -1;
            }
            already_programmed = true;
            for (uint32_t i = 0; i < sector_length; i++)
            {
                if (g_SectorBuffer[i + sector_start_offset] != local_buffer[i])
                {
                    already_programmed = false;
                    break;
                }
            }
            if (!already_programmed)
            {
                // sector erase
                ui32Status = mspi_device_func.mspi_sector_erase(g_FlashHdl, sector << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT);
                if (AM_ABSTRACT_MSPI_SUCCESS != ui32Status)
                {
                    am_util_stdio_printf("Failed to erase Flash Device sector %d!\n", sector);
                    return -1;
                }

                // Union previous sector contents with buffer
                for (uint32_t i = 0; i < sector_length; i++)
                {
                    g_SectorBuffer[i + sector_start_offset] = local_buffer[i];
                }

                sRange.ui32StartAddr = (uint32_t)g_SectorBuffer;
                sRange.ui32Size = MSPI_BUFFER_SIZE;
                am_hal_cachectrl_dcache_clean(&sRange);
                // write the sector
                ui32Status = mspi_device_func.mspi_write(g_FlashHdl,
                                                         g_SectorBuffer,
                                                         sector << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT,
                                                         1 << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT,
                                                         true);
                if (AM_ABSTRACT_MSPI_SUCCESS != ui32Status)
                {
                    am_util_stdio_printf("Failed to write buffer to Flash Device sector %d!\n", sector);
                    return -1;
                }
            }
        }

        if (!verify_only && already_programmed)
        {
            // don't repeat the read if we didn't write
            am_util_stdio_printf(".");
        }
        else
        {
            // verify
            ui32Status = mspi_device_func.mspi_read(g_FlashHdl,
                                                    g_SectorBuffer,
                                                    sector << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT,
                                                    1 << AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT,
                                                    true);
            if ( AM_ABSTRACT_MSPI_SUCCESS != ui32Status )
            {
                am_util_stdio_printf("Failed to read buffer to Flash Device!\n");
                return -1;
            }
            for (uint32_t i = 0; i < sector_length; i++)
            {
                if (g_SectorBuffer[i + sector_start_offset] != local_buffer[i])
                {
                    compare_error = true;
                    bad_bytes++;
                }
            }
            if ( compare_error )
            {
                am_util_stdio_printf("E");
            }
            else
            {
                am_util_stdio_printf("#");
            }
        }
        local_buffer += sector_length;
        flash_index += sector_length;
        remaining -= sector_length;
    }
    return bad_bytes;
}

//*****************************************************************************
//
//! MSPI Image Loader
//
//*****************************************************************************
uint32_t am_mspi_image_loader(am_common_img_ldr_cgf_t *pCommonCfg)
{
    am_mspi_img_ldr_cfg_t* pMspiCfg = (am_mspi_img_ldr_cfg_t*)pCommonCfg;

    uint32_t ui32Module = pMspiCfg->ui32Cfg_b.ui8DeviceNumber;
    uint32_t ui32Status;
    uint32_t ui32ResetGpioNum;
    bool bUseScrambling = pMspiCfg->ui32Cfg_b.ui8Scramble > 0;

    //
    // Initialize the MSPI Configuration based on Chip select and Device number
    //
    if ( pMspiCfg->ui32Cfg_b.ui8ChipSelect == AM_MSPI_CHIP_SELECT_CE0 )
    {
        am_util_stdio_printf("  Targeting MSPI%01d CS0\n", ui32Module);
    }
    else
    {
        am_util_stdio_printf("  Targeting MSPI%01d CS1\n", ui32Module);
    }

    //
    // Configure the mdoe and chip select
    //
    uint8_t ui8CsMode = (uint8_t)((pMspiCfg->ui32Cfg_b.ui8ChipSelect << 4) | pMspiCfg->ui32Mode);

    // Ensure that the lookup table covers all possible values
    const uint8_t lookupTableSize = sizeof(deviceConfigLookup) / sizeof(deviceConfigLookup[0]);

    if (ui8CsMode < lookupTableSize)
    {
        MSPI_Flash_Config.eDeviceConfig = deviceConfigLookup[ui8CsMode];
    }
    //
    // Hard reset MSPI device via GPIO
    //
    switch(ui32Module)
    {
        case 1:
#if defined(AM_PART_APOLLO510)
            ui32ResetGpioNum = AM_BSP_GPIO_MSPI1_RST;
#elif defined(AM_PART_APOLLO330P_510L)
            am_util_stdio_printf("MSPI module%d does not have a dedicated rst pin. Please use MSPI2.\n", ui32Module);
            return -1;
#endif
            break;
        case 2:
#if defined(AM_PART_APOLLO510)
            ui32ResetGpioNum = AM_BSP_GPIO_MSPI2_RST;
#elif defined(AM_PART_APOLLO330P_510L)
            ui32ResetGpioNum = AM_BSP_GPIO_MSPI2_X8_NOR_RST ;
#endif
            break;

#if !defined(AM_PART_APOLLO330P_510L)

#if !defined(AM_PART_APOLLO510)
        case 3:
#if defined(AM_PART_APOLLO510)
            ui32ResetGpioNum = AM_BSP_GPIO_MSPI3_RST;
#elif defined(AM_PART_APOLLO330P_510L)
            am_util_stdio_printf("MSPI module%d does not have a dedicated rst pin. Please use MSPI2.\n", ui32Module);
            return -1;
#endif
            break;
#endif

#endif

        default:
        case 0:
#if defined(AM_PART_APOLLO510)
            ui32ResetGpioNum = AM_BSP_GPIO_MSPI0_RST;
#elif defined(AM_PART_APOLLO330P_510L)
            am_util_stdio_printf("MSPI module%d does not have a dedicated rst pin. Please use MSPI2.\n", ui32Module);
            return -1;
#endif
            break;
    }

    //
    // Execute a GPIO reset of the MSPI device
    //
    am_hal_gpio_state_write(ui32ResetGpioNum, AM_HAL_GPIO_OUTPUT_SET);
    am_hal_gpio_pinconfig(ui32ResetGpioNum, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(ui32ResetGpioNum, AM_HAL_GPIO_OUTPUT_SET);
    am_util_delay_ms(100);
    am_hal_gpio_state_write(ui32ResetGpioNum, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_util_delay_ms(100);
    am_hal_gpio_state_write(ui32ResetGpioNum, AM_HAL_GPIO_OUTPUT_SET);
    am_util_debug_printf("Starting MSPI Timing Scan: \n");
    if ( AM_ABSTRACT_MSPI_SUCCESS == mspi_device_func.mspi_init_timing_check(ui32Module, (void*)&MSPI_Flash_Config, &MSPISdrTimingConfig) )
    {
        am_util_debug_printf("Scan Result: TXNEG = %d \n", MSPISdrTimingConfig.bTxNeg);
        am_util_debug_printf("             RXNEG = %d \n", MSPISdrTimingConfig.bRxNeg);
        am_util_debug_printf("             RXCAP = %d \n", MSPISdrTimingConfig.bRxCap);
        am_util_debug_printf("             TURNAROUND = %d \n", MSPISdrTimingConfig.ui8Turnaround);
        am_util_debug_printf("             TXDQSDELAY = %d \n", MSPISdrTimingConfig.ui8TxDQSDelay);
        am_util_debug_printf("             RXDQSDELAY = %d \n", MSPISdrTimingConfig.ui8RxDQSDelay);
    }
    else
    {
        am_util_stdio_printf("Scan Result: Failed, no valid setting.  \n");
        return -1;
    }

    //
    // Enable the MSPI interrupts.
    //
    NVIC_SetPriority(mspi_interrupts[ui32Module], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(mspi_interrupts[ui32Module]);
    am_hal_interrupt_master_enable();
    //
    //
    // Initialize the MSPI Module
    //
    ui32Status = mspi_device_func.mspi_init(ui32Module, (void*)&MSPI_Flash_Config, &g_FlashHdl, &g_MSPIHdl);

    if (AM_ABSTRACT_MSPI_SUCCESS != ui32Status)
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 1: Could not configure the device.\n");
#endif // AM_DEBUG_PRINTF
        pMspiCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF002;
        pMspiCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF002;
        pMspiCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF002;
        AM_ASM_BKPT(2);
        while(1);
    }

    //
    //  Set the timing from previous scan.
    //
    mspi_device_func.mspi_init_timing_apply(g_FlashHdl, &MSPISdrTimingConfig);

    mspi_device_func.mspi_read_id(g_FlashHdl);

    if (bUseScrambling)
    {
        ui32Status = mspi_device_func.mspi_scrambling_enable(g_FlashHdl);
        am_util_stdio_printf("  Scrambling enabled\n");
    }

    //
    // Fill the Metadata buffer
    // The sequence is as follows:
    //  0-3: Ambiq Image offset
    //  4-7: Ambiq Image size
    //  8-11: OEM Image offset
    // 12-15: OEM Image size
    //
    g_MetadataBuffer[0] = pMspiCfg->sCommonImgLdrCfg.ui32AmImgOffset & 0xFF;
    g_MetadataBuffer[1] = (pMspiCfg->sCommonImgLdrCfg.ui32AmImgOffset >> 8) & 0xFF;
    g_MetadataBuffer[2] = (pMspiCfg->sCommonImgLdrCfg.ui32AmImgOffset >> 16) & 0xFF;
    g_MetadataBuffer[3] = (pMspiCfg->sCommonImgLdrCfg.ui32AmImgOffset >> 24) & 0xFF;
    g_MetadataBuffer[4] = pMspiCfg->sCommonImgLdrCfg.ui32AmImgSize & 0xFF;
    g_MetadataBuffer[5] = (pMspiCfg->sCommonImgLdrCfg.ui32AmImgSize >> 8) & 0xFF;
    g_MetadataBuffer[6] = (pMspiCfg->sCommonImgLdrCfg.ui32AmImgSize >> 16) & 0xFF;
    g_MetadataBuffer[7] = (pMspiCfg->sCommonImgLdrCfg.ui32AmImgSize >> 24) & 0xFF;
    g_MetadataBuffer[8] = pMspiCfg->sCommonImgLdrCfg.ui32OemImgOffset & 0xFF;
    g_MetadataBuffer[9] = (pMspiCfg->sCommonImgLdrCfg.ui32OemImgOffset >> 8) & 0xFF;
    g_MetadataBuffer[10] = (pMspiCfg->sCommonImgLdrCfg.ui32OemImgOffset >> 16) & 0xFF;
    g_MetadataBuffer[11] = (pMspiCfg->sCommonImgLdrCfg.ui32OemImgOffset >> 24) & 0xFF;
    g_MetadataBuffer[12] = pMspiCfg->sCommonImgLdrCfg.ui32OemImgSize & 0xFF;
    g_MetadataBuffer[13] = (pMspiCfg->sCommonImgLdrCfg.ui32OemImgSize >> 8) & 0xFF;
    g_MetadataBuffer[14] = (pMspiCfg->sCommonImgLdrCfg.ui32OemImgSize >> 16) & 0xFF;
    g_MetadataBuffer[15] = (pMspiCfg->sCommonImgLdrCfg.ui32OemImgSize >> 24) & 0xFF;

    //
    // Write the Metadata to the MSPI flash
    //
    am_util_stdio_printf("Write metadata @ flash offset 0x%08x\n", pMspiCfg->ui32MetadataFlashAddr);
    pMspiCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = write_verify(g_MetadataBuffer, pMspiCfg->ui32MetadataFlashAddr,
                                                                CFGDATA_META_SIZE, (pMspiCfg->sCommonImgLdrCfg.ui32ReadWrite == AM_DATA_READ_VERIFY) ? true: false);
    if (pMspiCfg->sCommonImgLdrCfg.ui32MetadataErrCnt > 0)
    {
        am_util_stdio_printf("\nError 2: Failed to write metadata - %d bytes, %d errors\n", CFGDATA_META_SIZE, pMspiCfg->sCommonImgLdrCfg.ui32MetadataErrCnt);
        AM_ASM_BKPT(3);
        while(1);
    }

    if (pMspiCfg->sCommonImgLdrCfg.ui32MetadataErrCnt < 0)
    {
        am_util_stdio_printf("\nError 3: Failed to write metadata\n");
        AM_ASM_BKPT(4);
        while(1);
    }

    //
    // Write the Ambiq image
    //
    am_util_stdio_printf("\nWrite Ambiq Recovery Image @ flash offset 0x%08x from 0x%08x\n", pMspiCfg->sCommonImgLdrCfg.ui32AmImgOffset, pMspiCfg->sCommonImgLdrCfg.ui32AmImgAddr);
    pMspiCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = write_verify((uint8_t*)(pMspiCfg->sCommonImgLdrCfg.ui32AmImgAddr), pMspiCfg->sCommonImgLdrCfg.ui32AmImgOffset,
                                                             pMspiCfg->sCommonImgLdrCfg.ui32AmImgSize, (pMspiCfg->sCommonImgLdrCfg.ui32ReadWrite == AM_DATA_READ_VERIFY) ? true: false);

    if (pMspiCfg->sCommonImgLdrCfg.ui32AmImgErrCnt > 0)
    {
        am_util_stdio_printf("\nError 4: Errors writing recovery image - %d bytes, %d errors\n", pMspiCfg->sCommonImgLdrCfg.ui32AmImgSize, pMspiCfg->sCommonImgLdrCfg.ui32AmImgErrCnt);
        AM_ASM_BKPT(5);
        while(1);
    }

    if (pMspiCfg->sCommonImgLdrCfg.ui32AmImgErrCnt < 0)
    {
        am_util_stdio_printf("\nError 5: MSPI failure when writing Ambiq recovery image\n");
        AM_ASM_BKPT(6);
        while(1);
    }

    //
    // Write the OEM image
    //
    am_util_stdio_printf("\nWrite OEM Recovery Image @ flash offset 0x%08x from 0x%08x\n", pMspiCfg->sCommonImgLdrCfg.ui32OemImgOffset, pMspiCfg->sCommonImgLdrCfg.ui32OemImgAddr);
    pMspiCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = write_verify((uint8_t*)(pMspiCfg->sCommonImgLdrCfg.ui32OemImgAddr), pMspiCfg->sCommonImgLdrCfg.ui32OemImgOffset,
                                                             pMspiCfg->sCommonImgLdrCfg.ui32OemImgSize, (pMspiCfg->sCommonImgLdrCfg.ui32ReadWrite == AM_DATA_READ_VERIFY) ? true: false);

    if (pMspiCfg->sCommonImgLdrCfg.ui32OemImgErrCnt > 0)
    {
        am_util_stdio_printf("\nError 4: Errors writing recovery image - %d bytes, %d errors\n", pMspiCfg->sCommonImgLdrCfg.ui32OemImgSize, pMspiCfg->sCommonImgLdrCfg.ui32OemImgErrCnt);
        AM_ASM_BKPT(5);
        while(1);
    }

    if (pMspiCfg->sCommonImgLdrCfg.ui32OemImgErrCnt < 0)
    {
        am_util_stdio_printf("\nError 5: MSPI failure when writing recovery image\n");
        AM_ASM_BKPT(6);
        while(1);
    }

    if (bUseScrambling)
    {
        ui32Status = mspi_device_func.mspi_scrambling_disable(g_FlashHdl);
        am_util_stdio_printf("  Scrambling disabled\n");
    }

    am_util_stdio_printf("\nMSPI Loader successful\n");
    // instrumentation - view MSPI contents through XIP in the debugger
#ifdef XIP_AFTER_FLASH
    // Enable XIP and breakpoint to allow inspection
    am_util_stdio_printf("  XIP enabled, break for inspection\n");
    ui32Status = mspi_device_func.mspi_xip_enable(g_FlashHdl);
    AM_ASM_BKPT(0);
    ui32Status = mspi_device_func.mspi_xip_disable(g_FlashHdl);
#endif

#if defined(AM_BSP_MSPI_FLASH_DEVICE_IS25WX064)
    //
    // Change NV Config if needed
    //
    if (pMspiCfg->ui32Mode == 9)
    {
        mspi_device_func.mspi_change_nv_iomode(g_FlashHdl, true);
    }
#endif // AM_BSP_MSPI_FLASH_DEVICE_IS25WX064

    am_hal_interrupt_master_disable();
    NVIC_DisableIRQ(mspi_interrupts[ui32Module]);
    //
    // Clean up the MSPI before exit.
    //
    ui32Status = mspi_device_func.mspi_term(g_FlashHdl);
    if (AM_ABSTRACT_MSPI_SUCCESS != ui32Status)
    {
        am_util_stdio_printf("Failed to shutdown the MSPI and Flash Device!\n");
    }

    return ui32Status;
}
#endif

//*****************************************************************************
//
// eMMC partition handling
//
//*****************************************************************************
static uint32_t
emmc_set_partition(am_hal_card_t *pCard,
                   am_hal_card_emmc_partition_e eNewPartition)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;

    if ( pCard->eCurrentPartition != eNewPartition )
    {
        if ( eNewPartition <= AM_HAL_EMMC_BOOT2_PARTITION )
        {
            ui32Status = am_hal_card_emmc_partition_switch(pCard, eNewPartition);
        }
        else
        {
            ui32Status = AM_HAL_STATUS_OUT_OF_RANGE;
        }
    }

    return ui32Status;

} // emmc_set_partition()

//*****************************************************************************
//
//! eMMC Image Loader
//
//*****************************************************************************
uint32_t am_emmc_image_loader (am_common_img_ldr_cgf_t *pCommonCfg)
{
    am_emmc_img_ldr_cfg_t* pEmmcCfg = (am_emmc_img_ldr_cfg_t*)pCommonCfg;
    am_hal_card_t eMMCard;
    uint32_t ui32buffer[512 / 4];   // Buffer to contain 1 block of data
    uint32_t ui32MetaErrcnt, ui32RecvErrcnt, *pui32src, *pui32dat;
    uint32_t ui32Status, ux;
    uint32_t ui32NumBlksAmImg, ui32NumBlksOemImg;
    am_hal_host_inst_index_e eMMCDevNum = pEmmcCfg->ui32SDIODevNum;

    //
    // Configure SDIO PINs.
    //
    am_bsp_sdio_pins_enable(pEmmcCfg->ui32SDIODevNum, AM_HAL_HOST_BUS_WIDTH_8);

    //
    // Global interrupt enable
    //
    am_hal_interrupt_master_enable();

#ifdef AM_DEBUG_PRINTF
    //
    // Enable printing to the console.
    //
    am_bsp_itm_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_printf("Utility to write/read/verify eMMC%d MRAM recovery data\n\n", pEmmcCfg->ui32SDIODevNum);

    if ( pEmmcCfg->sCommonImgLdrCfg.ui32ReadWrite == AM_DATA_READ_VERIFY )
    {
        am_util_stdio_printf("!!! No writing -- read and verify existing data only. !!!\n");
    }
#endif // AM_DEBUG_PRINTF

    //
    // Get the underlying SDHC card host instance
    //
    pSdhcCardHost = am_hal_get_card_host(eMMCDevNum, true);

    if (pSdhcCardHost == NULL)
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 1: No card host found.\n");
#endif // AM_DEBUG_PRINTF
        pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF002;
        pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF002;
        pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF002;
        AM_ASM_BKPT(2);
        while(1);
    }
#ifdef AM_DEBUG_PRINTF
    else
    {
        am_util_stdio_printf("Card host found.\n");
    }
#endif // AM_DEBUG_PRINTF

    //
    // check if card is present
    //
    if (am_hal_card_host_find_card(pSdhcCardHost, &eMMCard) != AM_HAL_STATUS_SUCCESS)
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 2: No card is present.\n");
#endif // AM_DEBUG_PRINTF
        pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF003;
        pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF003;
        pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF003;
        AM_ASM_BKPT(2);
        while(1);
    }

    if (am_hal_card_init(&eMMCard, AM_HAL_CARD_TYPE_EMMC, NULL, AM_HAL_CARD_PWR_CTRL_SDHC_OFF) != AM_HAL_STATUS_SUCCESS)
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 3: Card init failed.\n");
#endif // AM_DEBUG_PRINTF
        pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF004;
        pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF004;
        pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF004;
        AM_ASM_BKPT(4);
        while(1);
    }

    //
    // Copy the metadata to a buffer.
    //
    for (ux = 0; ux < (512 / 4); ux++ )
    {
        ui32buffer[ux] = 0x0;
    }

    ui32buffer[0] = pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset;
    ui32buffer[1] = pEmmcCfg->sCommonImgLdrCfg.ui32AmImgSize;
    ui32buffer[2] = pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset;
    ui32buffer[3] = pEmmcCfg->sCommonImgLdrCfg.ui32OemImgSize;

    //
    // Configure for 48MHz, 8-bit DDR mode for read and write
    //
    ui32Status = am_hal_card_cfg_set(&eMMCard,
                                     AM_HAL_CARD_TYPE_EMMC,
                                     AM_HAL_HOST_BUS_WIDTH_8,
                                     48000000,
                                     AM_HAL_HOST_BUS_VOLTAGE_1_8,
                                     AM_HAL_HOST_UHS_DDR50);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 5 configuring eMMC device, rc=0x%08X.\n", ui32Status);
#endif // AM_DEBUG_PRINTF
        pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF005;
        pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF005;
        pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF005;
        AM_ASM_BKPT(5);
        while(1);
    }

    //
    // Set the desired partition
    //
    ui32Status = emmc_set_partition( &eMMCard, pEmmcCfg->ui32Partition & 0x3 );
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 5.2 setting partition, rc=0x%08X.\n", ui32Status);
#endif // AM_DEBUG_PRINTF
        pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF006;
        pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF006;
        pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF006;
        AM_ASM_BKPT(6);
        while(1);
    }
#ifdef AM_DEBUG_PRINTF
    else
    {
        am_util_stdio_printf("Partition %d set (", pEmmcCfg->ui32Partition & 0x3);
        switch ( pEmmcCfg->ui32Partition & 0x3 )
        {
            case 0:
                am_util_stdio_printf("USER_PARTITION).\n");
                break;
            case 1:
                am_util_stdio_printf("BOOT1_PARTITION).\n");
                break;
            case 2:
                am_util_stdio_printf("BOOT2_PARTITION).\n");
                break;
            default:
                am_util_stdio_printf("Invalid PARTITION!!).\n");
                break;
        }
    }
#endif // AM_DEBUG_PRINTF

    //
    // Compute the number of 512 blocks required for programming Ambiq Image.
    //
    ui32NumBlksAmImg  = pEmmcCfg->sCommonImgLdrCfg.ui32AmImgSize;  // data_size
    ui32NumBlksAmImg  = ui32NumBlksAmImg / 512;
    ui32NumBlksAmImg += (pEmmcCfg->sCommonImgLdrCfg.ui32AmImgSize % 512) ? 1 : 0;

    //
    // Compute the number of 512 blocks required for programming Ambiq Image.
    //
    ui32NumBlksOemImg  = pEmmcCfg->sCommonImgLdrCfg.ui32OemImgSize;  // data_size
    ui32NumBlksOemImg  = ui32NumBlksOemImg / 512;
    ui32NumBlksOemImg += (pEmmcCfg->sCommonImgLdrCfg.ui32OemImgSize % 512) ? 1 : 0;

    if ( pEmmcCfg->sCommonImgLdrCfg.ui32ReadWrite == AM_DATA_WRITE_VERIFY )
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("\n**********************************************************************\n");
#endif // AM_DEBUG_PRINTF

        //
        // Erase the Metadata, Ambiq Image and OEM Image Blocks.
        //
        ui32Status = am_hal_card_block_erase(&eMMCard, pEmmcCfg->ui32MetadataBlkNum, 1, AM_HAL_ERASE, 100);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
#ifdef AM_DEBUG_PRINTF
            am_util_stdio_printf("Error 6 erasing the metadata block %d (rc=0x%08X) .\n", pEmmcCfg->ui32MetadataBlkNum, ui32Status);
#endif // AM_DEBUG_PRINTF
            pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF007;
            pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF007;
            pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF007;
            AM_ASM_BKPT(7);
            while(1);
        }

        //
        // Erase the blocks for the Ambiq recovery image.
        //
        ui32Status = am_hal_card_block_erase(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset, ui32NumBlksAmImg, AM_HAL_ERASE, 100);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
#ifdef AM_DEBUG_PRINTF
            am_util_stdio_printf("Error 8 erasing the Ambiq recovery image blocks beginning at block %d (rc=0x%08X) .\n",
                                 pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset, ui32Status);
#endif // AM_DEBUG_PRINTF
            pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF008;
            pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF008;
            pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF008;
            AM_ASM_BKPT(8);
            while(1);
        }

        //
        // Erase the blocks for the OEM recovery image.
        //
        ui32Status = am_hal_card_block_erase(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset, ui32NumBlksOemImg, AM_HAL_ERASE, 100);
        if ( ui32Status != AM_HAL_STATUS_SUCCESS )
        {
#ifdef AM_DEBUG_PRINTF
            am_util_stdio_printf("Error 8 erasing the recovery image blocks beginning at block %d (rc=0x%08X) .\n",
                                 pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset, ui32Status);
#endif // AM_DEBUG_PRINTF
            pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF009;
            pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF009;
            pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF009;
            AM_ASM_BKPT(9);
            while(1);
        }

        //
        // Write the block of metadata. We'll write 1 block of 512 bytes.
        // Returns upper 16 bits: Number blocks written, low 16-bit is actual status.
        //
        ui32Status = am_hal_card_block_write_sync(&eMMCard, pEmmcCfg->ui32MetadataBlkNum, 1, (uint8_t *)ui32buffer);
        if ( ((ui32Status & 0xFFFF) != AM_HAL_STATUS_SUCCESS) || ((ui32Status >> 16) != 1) )
        {
#ifdef AM_DEBUG_PRINTF
            am_util_stdio_printf("Error 7 writing the metadata block %d (rc=0x%08X) .\n", pEmmcCfg->ui32MetadataBlkNum, ui32Status);
#endif // AM_DEBUG_PRINTF
            pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF00A;
            pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF00A;
            pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF00A;
            AM_ASM_BKPT(10);
            while(1);
        }
#ifdef AM_DEBUG_PRINTF
        else
        {
            am_util_stdio_printf("Successful write of metadata.       Start block=%5d. Num blocks written=%d.\n",
                                 pEmmcCfg->ui32MetadataBlkNum, ui32Status >> 16);
        }
#endif // AM_DEBUG_PRINTF

        //
        // Write the Ambiq recovery image to eMMC.
        //
        ui32Status = am_hal_card_block_write_sync(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset, ui32NumBlksAmImg, (uint8_t *)pEmmcCfg->sCommonImgLdrCfg.ui32AmImgAddr);
        if ( ((ui32Status & 0xFFFF) != AM_HAL_STATUS_SUCCESS) || ((ui32Status >> 16) != ui32NumBlksAmImg) )
        {
#ifdef AM_DEBUG_PRINTF
            am_util_stdio_printf("Error 9 writing the recovery image blocks beginning at block %d (rc=0x%08X) .\n",
                                 pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset, ui32Status);
#endif // AM_DEBUG_PRINTF
            pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF00B;
            pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF00B;
            pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF00B;
            AM_ASM_BKPT(11);
            while(1);
        }
#ifdef AM_DEBUG_PRINTF
        else
        {
            am_util_stdio_printf("Successful write of Ambiq recovery image. Start block=%5d. Num blocks written=%d.\n",
                                 pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset, ui32Status >> 16);
        }
#endif // AM_DEBUG_PRINTF

        //
        // Write the OEM recovery image to eMMC.
        //
        ui32Status = am_hal_card_block_write_sync(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset, ui32NumBlksOemImg, (uint8_t *)pEmmcCfg->sCommonImgLdrCfg.ui32OemImgAddr);
        if ( ((ui32Status & 0xFFFF) != AM_HAL_STATUS_SUCCESS) || ((ui32Status >> 16) != ui32NumBlksOemImg) )
        {
#ifdef AM_DEBUG_PRINTF
            am_util_stdio_printf("Error 9 writing the recovery image blocks beginning at block %d (rc=0x%08X) .\n",
                                 pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset, ui32Status);
#endif // AM_DEBUG_PRINTF
            pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF00C;
            pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF00C;
            pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF00C;
            AM_ASM_BKPT(12);
            while(1);
        }
#ifdef AM_DEBUG_PRINTF
        else
        {
            am_util_stdio_printf("Successful write of recovery image. Start block=%5d. Num blocks written=%d.\n",
                                 pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset, ui32Status >> 16);
        }

        am_util_stdio_printf("**********************************************************************\n");
#endif // AM_DEBUG_PRINTF
    }

    //
    // Read back and verify the eMMC data (Ambiq/OEM recovery image) just written.
    //
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("\n**********************************************************************\n");
    am_util_stdio_printf("Verify the OEM data on the eMMC device.\n");
#endif // AM_DEBUG_PRINTF
    ui32RecvErrcnt = 0;
    pui32src = (uint32_t*)(pEmmcCfg->sCommonImgLdrCfg.ui32OemImgAddr);
    for ( ux = 0; ux < ui32NumBlksOemImg; ux++ )
    {
        ui32Status = am_hal_card_block_read_sync(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset + ux, 1, (uint8_t *)ui32buffer);
        pui32dat = ui32buffer;
        for ( uint32_t uy = 0; uy < 512 / 4; uy++ )
        {
            if ( ui32buffer[uy] != *pui32src )
            {
                // If this is the first error, print a debug message
                if ( ui32RecvErrcnt == 0 )
                {
#ifdef AM_DEBUG_PRINTF
                    am_util_stdio_printf("MISMATCH - First error at source address 0x%08X:\n"
                                         "  Read from eMMC: 0x%08X, original/expected value 0x%08X.\n",
                                         pui32src, ui32buffer[uy], *pui32src);
#endif // AM_DEBUG_PRINTF
                }
                // Increment the error count
                ui32RecvErrcnt++;
                // Break after the first mismatch is found
                break;
            }
            pui32src++;
            pui32dat++;
        }
    }

    //
    // Save the results of writing the recovery image to the predefined location
    //
    pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = ui32RecvErrcnt;

    //
    // Read back and verify the eMMC data (OEM recovery image) just written.
    //
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("\n**********************************************************************\n");
    am_util_stdio_printf("Verify the Ambiq data on the eMMC device.\n");
#endif // AM_DEBUG_PRINTF
    ui32RecvErrcnt = 0;
    pui32src = (uint32_t*)(pEmmcCfg->sCommonImgLdrCfg.ui32AmImgAddr);
    for ( ux = 0; ux < ui32NumBlksAmImg; ux++ )
    {
        ui32Status = am_hal_card_block_read_sync(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset + ux, 1, (uint8_t *)ui32buffer);
        pui32dat = ui32buffer;
        for ( uint32_t uy = 0; uy < 512 / 4; uy++ )
        {
            if ( ui32buffer[uy] != *pui32src )
            {
                if ( ui32RecvErrcnt == 0 )
                {
#ifdef AM_DEBUG_PRINTF
                    am_util_stdio_printf("MISMATCH - First error at source address 0x%08X:\n"
                                         "  Read from eMMC: 0x%08X, original/expected value 0x%08X.\n",
                                         pui32src, ui32buffer[uy], *pui32src);
#endif // AM_DEBUG_PRINTF
                }
                ui32RecvErrcnt++;
                break;
            }
            pui32src++;
            pui32dat++;
        }
    }

    //
    // Save the results of writing the recovery image to the predefined location
    //
    pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = ui32RecvErrcnt;

    //
    // Check the metadata written to the device.
    //
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("\nDisplay to the user some of the data read from the eMMC device:\n");
#endif // AM_DEBUG_PRINTF
    ui32MetaErrcnt = 0;
    ui32Status = am_hal_card_block_read_sync(&eMMCard, pEmmcCfg->ui32MetadataBlkNum, 1, (uint8_t *)ui32buffer);
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("Display both words of the metadata as read from the eMMC device.\n");
    am_util_stdio_printf("  Block %d: WD0=0x%08X (%d), WD1=0x%08X (%d).\n",
                         pEmmcCfg->ui32MetadataBlkNum, ui32buffer[0], ui32buffer[0], ui32buffer[1], ui32buffer[1]);
#endif // AM_DEBUG_PRINTF

    if ( (ui32buffer[0] != pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset) )
    {
        ui32MetaErrcnt++;
    }
    if ( (ui32buffer[1] != pEmmcCfg->sCommonImgLdrCfg.ui32AmImgSize) )
    {
        ui32MetaErrcnt++;
    }
    if ( (ui32buffer[2] != pEmmcCfg->sCommonImgLdrCfg.ui32OemImgOffset) )
    {
        ui32MetaErrcnt++;
    }
    if ( (ui32buffer[3] != pEmmcCfg->sCommonImgLdrCfg.ui32OemImgSize) )
    {
        ui32MetaErrcnt++;
    }

    //
    // Save the results of writing the metadata at the predefined location
    //
    pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = ui32MetaErrcnt;

    //
    // Now display first and last words of the recovery image.
    //
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("Display selected words of the recovery image as read from the eMMC device.\n");
#endif // AM_DEBUG_PRINTF
    ui32Status = am_hal_card_block_read_sync(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset, 1, (uint8_t *)ui32buffer);
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("  First block of programmed image: WD0 =0x%08X, WD1 =0x%08X (blk=%d).\n",
                         ui32buffer[0], ui32buffer[1], pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset);
#endif // AM_DEBUG_PRINTF

    for (ux = 0; ux < (512 / 4); ux++ )
    {
        ui32buffer[ux] = 0x0;
    }

    ui32Status = am_hal_card_block_read_sync(&eMMCard, pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset + ui32NumBlksAmImg - 1, 1, (uint8_t *)ui32buffer);
    ux = pEmmcCfg->sCommonImgLdrCfg.ui32AmImgOffset + ui32NumBlksAmImg - 1;
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("  Last  block of programmed image: WD0 =0x%08X, WD1 =0x%08X (blk=%d).\n",
                         ui32buffer[0], ui32buffer[1], ux);
#endif // AM_DEBUG_PRINTF

    ux = ((pEmmcCfg->sCommonImgLdrCfg.ui32AmImgSize - (ui32NumBlksAmImg - 1) * 512) / 4) - 2;
#ifdef AM_DEBUG_PRINTF
    am_util_stdio_printf("  Last 2 wrds of programmed image: WD%2d=0x%08X, WD%2d=0x%08X.\n",
                         ux, ui32buffer[ux], ux + 1, ui32buffer[ux + 1]);

    am_util_stdio_printf("**********************************************************************\n");
#endif // AM_DEBUG_PRINTF

    //
    // Put the results at a known location
    //
    pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = ui32MetaErrcnt;
    pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = ui32RecvErrcnt;

#ifdef AM_DEBUG_PRINTF
    if ( (ui32RecvErrcnt == 0)  &&  (ui32MetaErrcnt == 0) )
    {
        am_util_stdio_printf("\nSuccess!\n"
                             "No programming or verification errors encountered with either metadata or the recovery image.\n");
    }
    else
    {
        if ( ui32MetaErrcnt == 0 )
        {
            am_util_stdio_printf("Metadata programmed correctly.\n");
        }
        else
        {
            am_util_stdio_printf("FAIL!! Metadata did not program correctly.\n", ui32RecvErrcnt);
        }

        if ( ui32RecvErrcnt == 0 )
        {
            am_util_stdio_printf("Recovery data programmed correctly.\n");
        }
        else
        {
            am_util_stdio_printf("FAIL!! %d mismatches found.\n", ui32RecvErrcnt);
        }
    }
#endif // AM_DEBUG_PRINTF

    //
    // Exit with the partition restored to USER partition (if a different partition was used)
    //
    ui32Status = emmc_set_partition( &eMMCard, 0 );
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
#ifdef AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 5.2 setting partition, rc=0x%08X.\n", ui32Status);
#endif // AM_DEBUG_PRINTF
        pEmmcCfg->sCommonImgLdrCfg.ui32MetadataErrCnt = 0xF00D;
        pEmmcCfg->sCommonImgLdrCfg.ui32AmImgErrCnt = 0xF00D;
        pEmmcCfg->sCommonImgLdrCfg.ui32OemImgErrCnt = 0xF00D;
        AM_ASM_BKPT(13);
        while(1);
    }
    return ui32Status;
}

#ifdef AM_BSP_MSPI_FLASH_PRESENT
am_hal_mpu_region_config_t sMPUCfg =
{
    .ui32RegionNumber = 6,
    .ui32BaseAddress = (uint32_t)DMATCBBuffer,
    .eShareable = NON_SHARE,
    .eAccessPermission = RW_NONPRIV,
    .bExecuteNever = true,
    .ui32LimitAddress = (uint32_t)DMATCBBuffer + sizeof(DMATCBBuffer) - 1,
    .ui32AttrIndex = 0,
    .bEnable = true,
};
am_hal_mpu_attr_t sMPUAttr =
{
    .ui8AttrIndex = 0,
    .bNormalMem = true,
    .sOuterAttr = {
                    .bNonTransient = false,
                    .bWriteBack = true,
                    .bReadAllocate = false,
                    .bWriteAllocate = false
                  },
    .sInnerAttr = {
                    .bNonTransient = false,
                    .bWriteBack = true,
                    .bReadAllocate = false,
                    .bWriteAllocate = false
                  },
    .eDeviceAttr = 0,
};
#endif

//*****************************************************************************
//
//! Main function for the MRAM Recovery Image Loader example
//
//*****************************************************************************
int main(void)
{
    am_common_img_ldr_cgf_t*     pCommonCfg = (am_common_img_ldr_cgf_t*)(PRGM_IMG_DATA_ADDR);
    uint32_t ui32Status;
    bool bOTPEnabled;

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
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#ifdef AM_BSP_MSPI_FLASH_PRESENT
    //
    // Set up the attributes.
    //
    am_hal_mpu_attr_configure(&sMPUAttr, 1);

    //
    // Clear the MPU regions.
    //
    am_hal_mpu_region_clear();

    //
    // Set up the regions.
    //
    am_hal_mpu_region_configure(&sMPUCfg, 1);

    //
    // Invalidate and clear DCACHE, this is required by CM55 TRF.
    //
    am_hal_cachectrl_dcache_invalidate(NULL, true);

    //
    // MPU enable
    //
    am_hal_mpu_enable(true, true);
#endif

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    if (am_bsp_itm_printf_enable())
    {
        while(1);
    }

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("MRAM Recovery Image Loader Example!\n");

    //
    // Check the signature
    //
    if (pCommonCfg->ui32Signature != AM_IMAGE_LOAD_SIGNATURE)
    {
#if defined AM_DEBUG_PRINTF
        am_util_stdio_printf("Error 1: Invalid Signature : %08X, expected 0xABCD1234. \n", pCommonCfg->ui32Signature);
        am_util_stdio_printf("The data consists of 15 words and is expected to be located at 0x%08X.\n", PRGM_IMG_DATA_ADDR);
        am_util_stdio_printf("  WD0: Signature. Must be 0xABCD1234.\n");
        am_util_stdio_printf("  WD1: Read-Verify/Write.\n");
        am_util_stdio_printf("  WD2: Ambiq Image Offset in flash.\n");
        am_util_stdio_printf("  WD3: Ambiq Image Size.\n");
        am_util_stdio_printf("  WD4: OEM Image Offset in flash.\n");
        am_util_stdio_printf("  WD5: OEM Image Size.\n");
        am_util_stdio_printf("  WD6: Ambiq Image Address in memory.\n");
        am_util_stdio_printf("  WD7: OEM Image Address in memory.\n");
        am_util_stdio_printf("  WD8: Result of writing the metadata to the device.\n");
        am_util_stdio_printf("  WD9: Result of writing the Ambiq image to the device.\n");
        am_util_stdio_printf("  WD10: Result of writing the OEM image to the device.\n");
        am_util_stdio_printf("  WD11: Device selection (0=MSPI, 1=eMMC).\n");
        am_util_stdio_printf("  WD12: Metadata block number/MSPI device Configurations.\n");
        am_util_stdio_printf("  WD13: eMMC Partition Selection/ MSPI Metadata Address in Flash.\n");
        am_util_stdio_printf("  WD14: eMMC Module Number.\n");
        am_util_stdio_printf("");
#endif // AM_DEBUG_PRINTF
        pCommonCfg->ui32MetadataErrCnt = 0xF001;
        pCommonCfg->ui32AmImgErrCnt = 0xF001;
        pCommonCfg->ui32OemImgErrCnt = 0xF001;
        AM_ASM_BKPT(1);
        while(1);
    }

    //
    // Clear signature so we don't try to load image in flash again
    //
    pCommonCfg->ui32Signature = 0x00000000;

#ifdef AM_BSP_MSPI_FLASH_PRESENT
    //
    // Load the image to the selected device
    //
    if (pCommonCfg->ui32DeviceSel == AM_DEVICE_SEL_MSPI)
    {
        //
        // Load the image to the MSPI device
        //
        am_mspi_image_loader(pCommonCfg);
    }
#endif
    if (pCommonCfg->ui32DeviceSel == AM_DEVICE_SEL_EMMC)
    {
        //
        // Load the image to the eMMC device
        //
        am_emmc_image_loader(pCommonCfg);
    }

    //
    //  End banner.
    //
    am_util_stdio_printf("MRAM Recovery Image Loader Example Complete\n");

    //
    // Return control to debugger
    //
    AM_ASM_BKPT(0);

    //
    // Loop forever while sleeping.
    //
    while (1)
    {
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
