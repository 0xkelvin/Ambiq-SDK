//*****************************************************************************
//
//! @file ios_fifo_common.c
//!
//! @brief Common utilities for IOS FIFO communication examples.
//!
//! @addtogroup interface_examples Interface Examples
//!
//! @defgroup ios_fifo_common IOS FIFO Common Utilities
//! @ingroup interface_examples
//! @{
//!
//! Purpose: This module provides shared functionality for IOS FIFO mode
//! examples, implementing common initialization, board setup, and pin
//! configuration display routines used by both controller and device components.
//!
//! @section ios_fifo_common_features Key Features
//!
//! 1. @b System @b Configuration: Implements low-power and cache settings
//!    for optimal performance
//!
//! 2. @b Debug @b Interface: Sets up ITM printf capability for status
//!    and debug output
//!
//! 3. @b Pin @b Documentation: Provides detailed pin connection diagrams
//!    for hardware setup
//!
//! @section ios_fifo_common_functions Functions
//!
//! - Board initialization and power configuration
//! - Cache system setup and optimization
//! - Pin connection visualization and documentation
//! - Debug interface configuration
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

#include "ios_fifo_common.h"

//*****************************************************************************
//
// Common function.
//
//*****************************************************************************
void common_setup(void)
{
    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

    //
    // Enable the ITM print interface.
    //
    am_bsp_itm_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();

    am_util_stdio_printf("The pin jumpers are as follows:\n");
    am_util_stdio_printf("\n");
    am_util_stdio_printf("\t\t+--------------------+                    +--------------------+\n");
    am_util_stdio_printf("\t\t|Board#1 (CONTROLLER)|                    |   Board#2 (DEVICE) |\n");
    am_util_stdio_printf("\t\t|      (IOM)         |                    |        (IOS)       |\n");
    am_util_stdio_printf("\t\t|--------------------|                    |--------------------|\n");
    am_util_stdio_printf("\t\t|   %3d (SCK/SCL)    |-----SCK------->----|   %3d (SCK/SCL)    |\n", AM_BSP_GPIO_TEST_IOM_SCK, AM_BSP_GPIO_TEST_IOS_SCK);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
    am_util_stdio_printf("\t\t|   %3d (MOSI/SDA)   |-----MOSI------>----|   %3d (MOSI/SDA)   |\n", AM_BSP_GPIO_TEST_IOM_MOSI, AM_BSP_GPIO_TEST_IOS_MOSI);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
#if (USE_SPI == 1)
    am_util_stdio_printf("\t\t|   %3d (MISO)       |----<----MISO-------|   %3d (MISO)       |\n", AM_BSP_GPIO_TEST_IOM_MISO, AM_BSP_GPIO_TEST_IOS_MISO);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
    am_util_stdio_printf("\t\t|   %3d (nCE)        |-----nCE------->----|   %3d (nCS)        |\n", AM_BSP_GPIO_TEST_IOM_CE, AM_BSP_GPIO_TEST_IOS_CE);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
#endif
    am_util_stdio_printf("\t\t|   %3d (INT)        |----<---HANDSHAKE---|   %3d (HANDSHAKE)  |\n", HANDSHAKE_PIN , HANDSHAKE_PIN);
    am_util_stdio_printf("\t\t|                    |                    |                    |\n");
    am_util_stdio_printf("\t\t|       GND          |-------GND----->----|       GND          |\n");
    am_util_stdio_printf("\t\t+--------------------+                    +--------------------+\n");
    am_util_stdio_printf("\n");
    am_util_stdio_printf("Power Sequence for Board#1 (CONTROLLER) and Board#2 (DEVICE):\n");
    am_util_stdio_printf("\tStep 1: Reset Board#2 (DEVICE) \n");
    am_util_stdio_printf("\tStep 2: Reset Board#1 (CONTROLLER) \n");
    am_util_stdio_printf("\n");

}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
