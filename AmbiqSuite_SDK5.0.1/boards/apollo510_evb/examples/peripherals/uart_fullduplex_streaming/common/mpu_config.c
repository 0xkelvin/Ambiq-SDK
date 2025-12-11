//*****************************************************************************
//
//! @file mpu_config.c
//!
//! @brief disabled caching in an SRAM region
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
#include "mpu_config.h"

//
//! --- Tune these two if you already use certain slots elsewhere ---
//

#define MPU_ATTR_INDEX_NC   (3u)   //!< MAIR slot to program as Normal, Non-cacheable,
#define MPU_REGION_FOR_DMA  (3u)   //!< MPU region number reserved for DMA buffers


#define SSRAM_END (SSRAM_BASEADDR + SSRAM_MAX_SIZE - 1)
#define DCACHE_LINE 32u

static uint32_t mpu_map_normal_nc(uint32_t ui32BaseAddress,
                                  uint32_t ui32DmaSize,
                                  uint32_t ui32RegionNum,
                                  uint32_t attr_index);

//*****************************************************************************
//
//  Alignment helper
//
//*****************************************************************************
static inline uint32_t align_down(uint32_t a, uint32_t align)
{
    return a & ~(align - 1u);
}
//*****************************************************************************
//
//  Alignment helper
//
//*****************************************************************************
static inline uint32_t align_up  (uint32_t a, uint32_t align)
{
    return (a + (align - 1u)) & ~(align - 1u);
}

//*****************************************************************************
//
//  Configure MPU
//
//*****************************************************************************
static uint32_t mpu_map_normal_nc(uint32_t ui32BaseAddress,
                                  uint32_t ui32DmaSize,
                                  uint32_t ui32RegionNum,
                                  uint32_t ui32AttrIndex)
{

    //
    // Clean/invalidate before changing attributes so cache can't retain stale lines
    //
    SCB_CleanDCache_by_Addr((void *) ui32BaseAddress, (int32_t)ui32DmaSize);
    SCB_InvalidateDCache_by_Addr((void *) ui32BaseAddress, (int32_t)ui32DmaSize);

    ARM_MPU_Disable();

    uint8_t ui8CachePolicy = (ARM_MPU_ATTR_NON_CACHEABLE << 4) | ARM_MPU_ATTR_NON_CACHEABLE;

    //
    // define mpu attributes for no cache
    //
    ARM_MPU_SetMemAttr((uint8_t) ui32AttrIndex, ui8CachePolicy);

    uint32_t ui32EndAddr     = ui32BaseAddress + ui32DmaSize -1;

    //
    // define MPU region
    //
    am_hal_mpu_region_config_t mpuRegion =
    {
        .ui32RegionNumber = ui32RegionNum,             // 3 is a random choice, has nothing to do with ui32AttrIndex
        .ui32BaseAddress  = ui32BaseAddress,
        .eShareable = ARM_MPU_SH_INNER,
        .eAccessPermission = RW_NONPRIV,
        .bExecuteNever = true,
        .ui32LimitAddress = ui32EndAddr,
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
//
// Ensures addresses are properly aligned
//
//*****************************************************************************
uint32_t mpuConfig(uint32_t ui32BaseAddress, uint32_t ui32DmaSize)
{
    //
    // route through the aligned helper
    //
    uint32_t start = ui32BaseAddress;
    uint32_t end   = start + ui32DmaSize - 1u;

    // Align the window to cache lines
    uint32_t addr_start = align_down(start, DCACHE_LINE);
    uint32_t addr_end   = align_up(end + 1u, DCACHE_LINE) - 1u;

    return mpu_map_normal_nc(addr_start, addr_end, MPU_REGION_FOR_DMA, MPU_ATTR_INDEX_NC);
}

//*****************************************************************************
//
//  Configure MPU as non-cacheable if (and only if) the buffer lives in system SRAM.
//
//*****************************************************************************
uint32_t uart_stream_cmn_mpuConfig(void *bufferAddress, uint32_t bufferSize)
{
    uint32_t start = (uint32_t) bufferAddress;
    uint32_t end   = start + bufferSize - 1u;

    // Must be entirely within system SRAM and not in DTCM
    if (start >= SSRAM_BASEADDR && end <= SSRAM_END)
    {
        return mpuConfig((uint32_t)start, (uint32_t)bufferSize);
    }
    // If it landed in DTCM or crosses a boundary, skip (DMA can't see DTCM anyway)
    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

