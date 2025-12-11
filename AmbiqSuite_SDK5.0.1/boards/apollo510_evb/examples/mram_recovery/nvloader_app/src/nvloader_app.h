//*****************************************************************************
//
//! @file nvloader_app.h
//!
//! @brief Headers file for MSPI/eMMC flash loader.
//!
//!
//!
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

#ifndef MR_IMAGE_LOADER_H
#define MR_IMAGE_LOADER_H

#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! Defines
//
//*****************************************************************************
#define AM_DEVICE_SEL_MSPI          0
#define AM_DEVICE_SEL_EMMC          1
#define AM_IMAGE_LOAD_SIGNATURE     (0xABCD1234)

#define AM_DATA_READ_VERIFY         0
#define AM_DATA_WRITE_VERIFY        0xF

#define AM_MSPI_CHIP_SELECT_CE0     0
#define AM_MSPI_CHIP_SELECT_CE1     1

typedef enum
{
    AM_CHIPSEL_MSPI_CE0_MODE0 = 0x00,
    AM_CHIPSEL_MSPI_CE1_MODE0 = 0x10,
    AM_CHIPSEL_MSPI_CE0_MODE1 = 0x01,
    AM_CHIPSEL_MSPI_CE1_MODE1 = 0x11,
    AM_CHIPSEL_MSPI_CE0_MODE2 = 0x02,
    AM_CHIPSEL_MSPI_CE1_MODE2 = 0x12,
    AM_CHIPSEL_MSPI_CE0_MODE3 = 0x03,
    AM_CHIPSEL_MSPI_CE1_MODE3 = 0x13,
    AM_CHIPSEL_MSPI_CE0_MODE4 = 0x04,
    AM_CHIPSEL_MSPI_CE1_MODE4 = 0x14,
    AM_CHIPSEL_MSPI_CE0_MODE5 = 0x05,
    AM_CHIPSEL_MSPI_CE1_MODE5 = 0x15,
    AM_CHIPSEL_MSPI_CE0_MODE6 = 0x06,
    AM_CHIPSEL_MSPI_CE1_MODE6 = 0x16,
    AM_CHIPSEL_MSPI_CE0_MODE7 = 0x07,
    AM_CHIPSEL_MSPI_CE1_MODE7 = 0x17,
    AM_CHIPSEL_MSPI_CE0_MODE8 = 0x08,
    AM_CHIPSEL_MSPI_CE1_MODE8 = 0x18,
    AM_CHIPSEL_MSPI_CE0_MODE9 = 0x09,
    AM_CHIPSEL_MSPI_CE1_MODE9 = 0x19,
}am_mspi_chipsel_mode_e;

//*****************************************************************************
//
//! Common Image Loader Structure
//
//*****************************************************************************
typedef struct
{
    uint32_t    ui32Signature;          // Signature for the image loader
    uint32_t    ui32ReadWrite;          // Read/Write flag (0x0 for read/verify, 0xF for write/verify)
    uint32_t    ui32AmImgOffset;        // Offset of the Ambiq image in the Flash Device
    uint32_t    ui32AmImgSize;          // Size of the Ambiq image
    uint32_t    ui32OemImgOffset;       // Offset of the OEM image in the Flash Device
    uint32_t    ui32OemImgSize;         // Size of the OEM image
    uint32_t    ui32AmImgAddr;          // Address of the Ambiq image in the DUT
    uint32_t    ui32OemImgAddr;         // Address of the OEM image in the DUT
    uint32_t    ui32MetadataErrCnt;     // Error count for the metadata
    uint32_t    ui32AmImgErrCnt;        // Error count for the Ambiq image
    uint32_t    ui32OemImgErrCnt;       // Error count for the OEM image
    uint32_t    ui32DeviceSel;          // Device selection (0 for MSPI, 1 for eMMC)
}am_common_img_ldr_cgf_t;

//*****************************************************************************
//
//! eMMC Image Loader Structure
//
//*****************************************************************************
typedef struct
{
    am_common_img_ldr_cgf_t sCommonImgLdrCfg;
    uint32_t                ui32MetadataBlkNum;         // Block Number of the Metadata address in the Flash
    uint32_t                ui32Partition;              // Partition number of the eMMC ( 0x0->USER, 0x1->BOOT1, 0x2->BOOT2 )
    uint32_t                ui32SDIODevNum;             // SDIO Device Number (0x0 - 0x1)
}am_emmc_img_ldr_cfg_t;

//*****************************************************************************
//
//! MSPI Image Loader Structure
//
//*****************************************************************************
typedef struct
{
    am_common_img_ldr_cgf_t sCommonImgLdrCfg;
    union                                           // Configuration for the MSPI device
    {
        struct
        {
            uint8_t     ui8DeviceNumber : 8;        // Device Number (0x0 - 0x3)
            uint8_t     ui8ChipSelect   : 8;        // Chip Select (0x0 for CE0, 0x1 for CE1)
            uint8_t     ui8Scramble     : 8;        // Scrambling enable (0x0 for disable, 0x1 for enable)
            uint8_t     rsvd            : 8;        // Reserved
        }ui32Cfg_b;
        uint32_t ui32Cfg;
    };
    uint32_t                ui32MetadataFlashAddr;  // Metadata address in the Flash
    uint32_t                ui32Mode;               // Mode of operation (1-1-1 = 0, 1-1-2 = 1, 1-2-2 = 2, 1-1-4 = 3, 1-4-4 = 4, 1-1-8 = 5, 1-8-8 = 6, 2-2-2 = 7, 4-4-4 = 8, 8-8-8 = 9)
}am_mspi_img_ldr_cfg_t;

//*****************************************************************************
//
//! abstracted macros for the various mspi devices
//
//*****************************************************************************
#if defined(AM_BSP_MSPI_FLASH_DEVICE_ATXP032)
    #include "am_devices_mspi_atxp032.h"
    #define am_abstract_mspi_devices_config_t         am_devices_mspi_atxp032_config_t
    #define am_abstract_mspi_devices_timing_config_t  am_devices_mspi_atxp032_sdr_timing_config_t
    #define AM_ABSTRACT_MSPI_SUCCESS                  AM_DEVICES_MSPI_ATXP032_STATUS_SUCCESS
    #define AM_ABSTRACT_MSPI_ERROR                    AM_DEVICES_MSPI_ATXP032_STATUS_ERROR
    #define AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT        16
#elif defined(AM_BSP_MSPI_FLASH_DEVICE_IS25WX064)
    #include "am_devices_mspi_is25wx064.h"
    #define am_abstract_mspi_devices_config_t         am_devices_mspi_is25wx064_config_t
    #define am_abstract_mspi_devices_timing_config_t  am_devices_mspi_is25wx064_timing_config_t
    #define AM_ABSTRACT_MSPI_SUCCESS                  AM_DEVICES_MSPI_IS25WX064_STATUS_SUCCESS
    #define AM_ABSTRACT_MSPI_ERROR                    AM_DEVICES_MSPI_IS25WX064_STATUS_ERROR
    #define AM_DEVICES_MSPI_FLASH_SECTOR_SHIFT        17
#endif

#ifdef AM_BSP_MSPI_FLASH_PRESENT
//*****************************************************************************
//
//! the universal struct for the various mspi devices struct
//! each internal function pointer members refer to their different drivers
//
//*****************************************************************************
typedef struct
{
    uint8_t  devName[30];
    uint32_t (*mspi_init)(uint32_t ui32Module, am_abstract_mspi_devices_config_t *psMSPISettings, void **ppHandle, void **ppMspiHandle);
    uint32_t (*mspi_term)(void *pHandle);

    uint32_t (*mspi_read_id)(void *pHandle);

    uint32_t (*mspi_read)(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion);

    uint32_t (*mspi_read_adv)(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes,
                           uint32_t ui32PauseCondition,
                           uint32_t ui32StatusSetClr,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);

    uint32_t (*mspi_read_callback)(void *pHandle, uint8_t *pui8RxBuffer,
                           uint32_t ui32ReadAddress,
                           uint32_t ui32NumBytes);

    uint32_t (*mspi_write)(void *pHandle, uint8_t *ui8TxBuffer,
                           uint32_t ui32WriteAddress,
                           uint32_t ui32NumBytes,
                           bool bWaitForCompletion);

    uint32_t (*mspi_write_adv)(void *pHandle,
                           uint8_t *puiTxBuffer,
                           uint32_t ui32WriteAddress,
                           uint32_t ui32NumBytes,
                           uint32_t ui32PauseCondition,
                           uint32_t ui32StatusSetClr,
                           am_hal_mspi_callback_t pfnCallback,
                           void *pCallbackCtxt);

    uint32_t (*mspi_mass_erase)(void *pHandle);
    uint32_t (*mspi_sector_erase)(void *pHandle, uint32_t ui32SectorAddress);

    uint32_t (*mspi_xip_enable)(void *pHandle);
    uint32_t (*mspi_xip_disable)(void *pHandle);
    uint32_t (*mspi_scrambling_enable)(void *pHandle);
    uint32_t (*mspi_scrambling_disable)(void *pHandle);
    uint32_t (*mspi_init_timing_check)(uint32_t ui32Module,
                                        am_abstract_mspi_devices_config_t *pDevCfg,
                                        am_abstract_mspi_devices_timing_config_t *pDevSdrCfg);
    uint32_t (*mspi_init_timing_apply)(void *pHandle,
                                        am_abstract_mspi_devices_timing_config_t *pDevSdrCfg);
    void (*mspi_change_nv_cfg_default)( uint32_t ui32Module,
                                        void **ppHandle,
                                        void **ppMspiHandle);
    void (*mspi_change_nv_iomode)(void *pHandle, bool bOctal);
} mspi_device_func_t;
#endif

#ifdef __cplusplus
}
#endif

#endif // MR_IMAGE_LOADER_H
