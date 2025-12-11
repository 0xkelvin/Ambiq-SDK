//*****************************************************************************
//
//! @file uart_fullduplex_controller.c
//!
//! @brief This example demonstrates how apollo5 UART work in fullduplex mode.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//!
//! @defgroup uart_fullduplex_controller UART FullDuplex Controller Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates how apollo5 UART work in fullduplex mode.
//! To run this example, the user needs two apollo510 boards and flash
//! uart_fullduplex_controller and uart_fullduplex respectively for the controller and
//! the device, then power on the controller first, then the device.
//!
//!
//! Apollo510_evb mapping
//!
//!     CONTROLLER (uart_fullduplex_controller) DEVICE (uart_fullduplex)
//!     --------------------                    ----------------
//!     GPIO[61]  UART3 TX                      GPIO[62]  UART0 RX
//!     GPIO[62]  UART0 RX                      GPIO[61]  UART3 TX
//!     GPIO[5]   Device Ready INT (controller) GPIO[5]   Device Ready INT (device)
//!     GND                                     GND
//!
//! Apollo510b_evb mapping
//!
//!     CONTROLLER (uart_fullduplex_controller) DEVICE (uart_fullduplex)
//!     --------------------                    ----------------
//!     GPIO[00]  UART0 TX                      GPIO[02]  UART1 RX
//!     GPIO[02]  UART1 RX                      GPIO[00]  UART0 TX
//!     GPIO[5]   Device Ready INT (controller) GPIO[5]   Device Ready INT (device)
//!     GND                                     GND
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

#include "uart_fullduplex_common.h"
#include "uart_example_timer_control.h"
#include "handshake_gpio.h"
#include "am_util.h"

uint32_t g_ui32TestCount = 0;

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int main(void)
{
    uint32_t ui32Status = mcu_setup();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("UART Fullduplex [CONTROLLER] Example\n");
    am_util_stdio_printf("\n");

    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("ERROR: common_setup\n example will hang\n");
        while (1);
    }

    bool isRx = true ;
    ui32Status = handshake_gpio_init(isRx) ;

    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("ERROR: handshake setup\n example will hang\n");
        while (1);
    }
    am_hal_interrupt_master_enable();

    ui32Status = fd_common_setup();
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("ERROR: common uart setup\n example will hang\n");
        while (1);
    }

    uint32_t ui32TestSuccess = 0;

    while(1)
    {
        while (handshake_gpio_flagged(false) == false );

        am_util_stdio_printf("start %d\n ", g_ui32TestCount);
        
        am_util_delay_ms( 50 ) ;



        if ( uart_run_fullduplex_test() != 0 )
        {
            am_util_stdio_printf("ERROR: uart_run_fullduplex_test %d\n ", ui32Status);
        }
        else
        {
            ui32TestSuccess++;
        }

        if ( ++g_ui32TestCount == UART_TEST_PACKET_CNT )
        {
            break;
        }
        am_util_stdio_printf(" Test %d\n ", g_ui32TestCount);
    }

    am_util_stdio_printf(" Test complete: Tests Passed: %d, Total Tests %d\n ",
        ui32TestSuccess, g_ui32TestCount);

    common_teardown();
    while (1);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
