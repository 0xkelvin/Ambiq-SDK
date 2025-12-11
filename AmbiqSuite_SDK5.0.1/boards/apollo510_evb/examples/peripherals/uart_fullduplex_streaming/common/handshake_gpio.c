//*****************************************************************************
//
//! @file handshake_gpio.c
//!
//! @brief sets up and manages the periodic interrupt timer
//! small appends
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
#include "handshake_gpio.h"
#include "uart_fullduplex_common.h"
#include "uart_example_timer_control.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_bsp.h"

static bool g_handshake_is_rx = false;
static volatile bool g_handshake_flag = false;

//
// because this only uses am_gpio0_001f_isr, the pin number must be less than 32
//
#if UART_DEVICE_READY_IN_PIN > 31
#error "invalid pin number for this code"
#endif

//*****************************************************************************
//
// This will setup the GPIO for tx or interrupt rx depending on the isRx flag
//
//*****************************************************************************
uint32_t handshake_gpio_init(bool isRx)
{
    g_handshake_is_rx = isRx;
    g_handshake_flag = false;

    if (isRx)
    {
        uint32_t ui32IntPin = UART_DEVICE_READY_IN_PIN;
        if (ui32IntPin > 31 )
        {
            //
            // this pin will not work here, need to use a different ISR
            //
            return AM_HAL_STATUS_FAIL;
        }
        am_hal_gpio_pinconfig(UART_DEVICE_READY_IN_PIN, am_hal_gpio_pincfg_input);

        uint32_t ui32IntStatus = 1 << (ui32IntPin & 0x1F);

        am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, ui32IntStatus);
        am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                      AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                      (void *)&ui32IntPin);

        //
        // Clear and enable interrupt (but don't allow ISR to act unless in RX mode)
        //
        uint32_t irq_idx = ui32IntPin >> 5;

        IRQn_Type eGpioIRQn = (IRQn_Type)(GPIO0_001F_IRQn + irq_idx);

        NVIC_ClearPendingIRQ(eGpioIRQn);
        NVIC_SetPriority(eGpioIRQn, AM_IRQ_PRIORITY_DEFAULT);
        NVIC_EnableIRQ(eGpioIRQn);
    }
    else
    {
        am_hal_gpio_pinconfig(UART_DEVICE_READY_OUT_PIN, am_hal_gpio_pincfg_output);
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// This will return true if a GPIO interrupt was received and then clear the rx flag
//
//*****************************************************************************
bool handshake_gpio_flagged( bool clearFlag)
{
    bool flag = g_handshake_flag;
    if ( clearFlag )
    {
        g_handshake_flag = false;
    }
    return flag;
}

//*****************************************************************************
//
// this will wait for the GPIO signal, with timeout
//
//*****************************************************************************
uint32_t handshake_gpio_wait( bool state, uint32_t timeoutUsec)
{
    if (g_handshake_flag != state)
    {
        uart_timer_init(timeoutUsec );
        while (g_handshake_flag != state)
        {
            if ( uart_timer_isExpired())
            {
                return AM_HAL_STATUS_TIMEOUT;
            }
        }
    }

    return AM_HAL_STATUS_SUCCESS;
}

//*****************************************************************************
//
// This will create/send a pulse on the previously defined GPIO pin
//
//*****************************************************************************
void handshake_gpio_send_pulse(void)
{
    if ( g_handshake_is_rx == false )
    {
        am_hal_gpio_output_set(UART_DEVICE_READY_OUT_PIN);
        am_hal_delay_us(50);
        am_hal_gpio_output_clear(UART_DEVICE_READY_OUT_PIN);
    }
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio0_001f_isr(void)
{
    uint32_t    ui32IntStatus;

    am_hal_gpio_interrupt_irq_status_get(GPIO0_001F_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, ui32IntStatus);
    if (ui32IntStatus & ( 1 << (UART_DEVICE_READY_IN_PIN & 0x1F) ))
    {
        g_handshake_flag = true;
    }

    __NOP();       // sometimes need a delay after clearing ISR to prevent ISR doubling.
    __ISB();       // usually putting the ISR clear first, then doing processing is all that's needed.
    __NOP();
    __ISB();
    __NOP();
    __ISB();

}
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
