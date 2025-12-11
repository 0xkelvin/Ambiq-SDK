//*****************************************************************************
//
//! @file nemagfx_vg_svg.c
//!
//! @brief NemaGFX Vector Graphics SVG Rendering Example
//!
//! @addtogroup graphics_examples Graphics Examples
//!
//! @defgroup nemagfx_vg_svg NemaGFX Vector Graphics SVG Example
//! @ingroup graphics_examples
//! @{
//!
//! Purpose: This example demonstrates SVG (Scalable Vector Graphics)
//! rendering capabilities using NemaGFX vector graphics library. The example
//! showcases hardware-accelerated SVG rendering with custom memory management
//! and MPU configuration for optimal performance.
//!
//! @section nemagfx_vg_svg_features Key Features
//!
//! 1. @b SVG @b Rendering: Supports full SVG specification rendering with
//!    hardware acceleration for complex vector graphics
//!
//! 2. @b Custom @b Memory @b Management: Implements custom graphics heap allocation
//!    with configurable cache behavior for optimal GPU performance
//!
//! 3. @b MPU @b Configuration: Demonstrates Memory Protection Unit setup for
//!    non-cacheable graphics memory regions
//!
//! 4. @b High @b Performance @b Mode: Supports high-performance mode for optimal
//!    rendering performance on compatible hardware
//!
//! 5. @b Scalable @b Graphics: Renders scalable vector graphics that maintain
//!    quality at any resolution without pixelation
//!
//! @section nemagfx_vg_svg_configuration Configuration Options
//!
//! - @b NON_CACHEABLE_CL_RB_HEAP: Enables non-cacheable graphics heap for
//!   command list and ring buffer operations
//! - @b NEMA_USE_CUSTOM_MALLOC: Enables custom memory allocation for graphics
//!   operations
//! - @b RUN_IN_HP_MODE: Enables high-performance mode for optimal rendering
//!
//! @section nemagfx_vg_svg_hardware Hardware Requirements
//!
//! - Compatible Development Board
//! - Display panel for SVG rendering visualization
//! - Sufficient memory for graphics heap and SVG processing
//!
//! @section nemagfx_vg_svg_usage Usage
//!
//! The application automatically loads and renders SVG graphics using hardware
//! acceleration. The example demonstrates various SVG features including
//! complex shapes, gradients, and transformations.
//!
//! @note Custom memory management may be required for optimal performance
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

#include "nemagfx_vg_svg.h"
#include "nemagfx_buffer_customize.h"

//*****************************************************************************
//
//! @brief Macro definitions for graphics configuration
//!
//! @details Configuration macros for custom memory management and cache behavior
//!
//*****************************************************************************
#ifndef NON_CACHEABLE_CL_RB_HEAP
        #define NON_CACHEABLE_CL_RB_HEAP 0
#endif

//*****************************************************************************
//
//! @brief Global variables for SVG rendering configuration
//!
//! @details Global variables and configuration structures for SVG rendering
//! and custom memory management
//!
//*****************************************************************************

//*****************************************************************************
//
//! @brief Custom graphics heap configuration
//!
//! @details Custom memory allocation for graphics operations with configurable
//! heap size and cache behavior for optimal GPU performance
//!
//*****************************************************************************

#ifdef NEMA_USE_CUSTOM_MALLOC
    #define GRAPHIC_HEAP_SIZE 0x180000UL

    AM_SHARED_RW uint8_t graphic_heap_ssram[GRAPHIC_HEAP_SIZE];
#endif


//*****************************************************************************
//
//! @brief Memory Protection Unit (MPU) configuration
//!
//! @details MPU setup for non-cacheable graphics memory regions to ensure
//! proper memory access patterns for GPU command lists and ring buffers
//!
//*****************************************************************************

#if (NON_CACHEABLE_CL_RB_HEAP == 1) && defined(NEMA_USE_CUSTOM_MALLOC)

#define MPU_REGION_NUMBER 6
#define MPU_ATTRIBUTE_ENTRY 0

/*
 * Set the graphics heap to noncacheable,
 * the GPU command list buffers and global ring buffer will be allocated from this heap.
 */
void set_graphic_heap_noncacheable(void)
{

    am_hal_mpu_region_config_t sMPUCfg =
    {
        .ui32RegionNumber = MPU_REGION_NUMBER,
        .ui32BaseAddress = 0x20080000U,
        .eShareable = NON_SHARE,
        .eAccessPermission = RW_NONPRIV,
        .bExecuteNever = true,
        .ui32LimitAddress = 0,
        .ui32AttrIndex = MPU_ATTRIBUTE_ENTRY,
        .bEnable = true,
    };

    //MPU non-cacheable attribute setting.
    //OuterAttr and InnerAttr is set to 0b0100 to follow the requirement in ArmV8-m Architecture Reference Manual
    am_hal_mpu_attr_t sMPUAttr =
    {
        .ui8AttrIndex = MPU_ATTRIBUTE_ENTRY,
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
    sMPUCfg.ui32BaseAddress = (uintptr_t)graphic_heap_ssram;
    sMPUCfg.ui32LimitAddress = (uintptr_t)graphic_heap_ssram + GRAPHIC_HEAP_SIZE - 1;
    am_hal_mpu_region_configure(&sMPUCfg, 1);

    //
    // Invalidate and clear DCACHE, this is required by CM55 TRF.
    //
    am_hal_cachectrl_dcache_invalidate(NULL, true);

    //
    // MPU enable
    //
    am_hal_mpu_enable(true, true);
}

#endif

bool graphic_heap_region_need_flush(uint32_t region_start, uint32_t region_size)
{
#if NON_CACHEABLE_CL_RB_HEAP == 1
    if ( (region_start >= (uint32_t)graphic_heap_ssram) &&
       (region_start < (uint32_t)graphic_heap_ssram + GRAPHIC_HEAP_SIZE) )
    {
        return false;
    }
    else
#endif
    {
        return true;
    }
}

//*****************************************************************************
//
//! @brief Main application entry point for SVG rendering example
//!
//! @details Initializes the system, configures custom memory management,
//! sets up MPU regions if needed, and starts the SVG rendering application.
//!
//! @return int Returns 0 on successful execution (never returns)
//!
//*****************************************************************************
int
main(void)
{
    uint32_t ui32Status;

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Configure the SEGGER SystemView Interface.
    //
#ifdef SYSTEM_VIEW
    SEGGER_SYSVIEW_Conf();
#endif

    //
    // Enable global IRQ.
    //
    am_hal_interrupt_master_enable();

    //
    // Initialize plotting interface.
    //
    am_util_stdio_printf("nemagfx_vg_svg Example\n");

#ifdef RUN_IN_HP_MODE
#ifdef AM_PART_APOLLO330P_510L
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("HP_LP:Enter HP mode failed!\n");
    }
#else
    if ( am_hal_pwrctrl_mcu_mode_select(AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE) != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("HP_LP:Enter HP mode failed!\n");
    }
#endif
#endif

    //
    // Set the memory region used for graphics buffer to non-cacheable.
    //
#if defined(NEMA_USE_CUSTOM_MALLOC) && (NON_CACHEABLE_CL_RB_HEAP == 1)
    set_graphic_heap_noncacheable();
#endif

    //
    // Customize graphics .
    //
#ifdef NEMA_USE_CUSTOM_MALLOC
    graphic_heap_init((void*)graphic_heap_ssram, GRAPHIC_HEAP_SIZE);
#endif


    //
    // Run the application.
    //
    run_tasks();

    //
    // We shouldn't ever get here.
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

