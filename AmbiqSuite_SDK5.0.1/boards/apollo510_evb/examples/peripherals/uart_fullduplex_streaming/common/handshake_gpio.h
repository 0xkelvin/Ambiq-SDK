//*****************************************************************************
//
//! @file handshake_gpio.h
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
#ifndef HANDSHAKE_GPIO_H
#define HANDSHAKE_GPIO_H

#include "am_mcu_apollo.h"
#include "am_util.h"

//*****************************************************************************
//
//! @brief This will setup the GPIO for tx or interrupt rx depending on the isRx flag
//!
//! @note this simplified code will only accept GPIO pins betwen 0 and 31
//!
//! @param when true, this will setup for RX (using the GPIO isr)
//!
//! @return standard hal status (fail if internally #defined pin number > 31)
//
//*****************************************************************************
uint32_t handshake_gpio_init(bool isRx);

//*****************************************************************************
//
//! @brief This will create/send a pulse on the previously defined GPIO pin
//
//*****************************************************************************
void handshake_gpio_send_pulse(void);

//*****************************************************************************
//
//
//! @brief return GPIO pulse detect staus (true of puls was detected)
//!
//! @param clearFlag  will clear flag( reset ) the GPIO flag
//
//! @return  will return true if the pulse ISR was triggered
//
//*****************************************************************************
bool handshake_gpio_flagged( bool clearFlag);

//*****************************************************************************
//
//! @brief This will wait for the GPIO signal, with timeout
//!
//! @param state  will return when the ISR written flag matches this state
//! @param timeoutUsec  return error timeout after this many microseconds with no flag change
//!
//! @return  will return success (0) or timeout error
//
//*****************************************************************************
uint32_t handshake_gpio_wait( bool state, uint32_t timeoutUsec);

#endif //HANDSHAKE_GPIO_H
//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
