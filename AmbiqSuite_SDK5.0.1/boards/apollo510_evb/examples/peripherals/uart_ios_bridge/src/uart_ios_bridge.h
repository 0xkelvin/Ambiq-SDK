//*****************************************************************************
//
//! @file uart_ios_bridge.h
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

#ifndef UART_IOS_BRIDGE_H
#define UART_IOS_BRIDGE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"


//*****************************************************************************
//
// Configuration options
//
//*****************************************************************************

//
// The following PP's device take care of the target device.
// The default takes the same target as the host
// To override the TARGET device update DESIGNATE_TARGET to the desired device type
//

#define DESIGNATE_TARGET AM_PART_APOLLO510

#ifdef DESIGNATE_TARGET
#define TARGET_DEVICE DESIGNATE_TARGET
#else
#ifdef AM_PART_APOLLO510
#define TARGET_DEVICE AM_PART_APOLLO510
#else
#ifdef AM_PART_APOLLO510L
#define TARGET_DEVICE AM_PART_APOLLO510L
#endif
#endif
#endif



#if (AM_BSP_NUM_BUTTONS > 0)
#define INITIATE_MRAM_RCV_PIN    AM_BSP_GPIO_BUTTON1
#else
#define INITIATE_MRAM_RCV_PIN      0
#endif


#define TIMEOUT_TIMER_INSTANCE      2

//
// Define the host pins that will be used for connecting to
// corresponding pins on the target.
//
// Platform specific pin:
// TARGET_OVERRIDE_MRAM_RCV_PIN is connected to the corresponding
// 'Override' or 'MRAM Recovery GPIO Control' pin on the slave device.
// This signal is used to force the target SBL/SBR resp to enter SBL
// WIRED update/MRAM recovery resp.
//
#define TARGET_OVERRIDE_MRAM_RCV_PIN       18

//
// Platform specific pin:
// Slave interrupt (SLINT) pin is connected to this host pin.
// This interrupt is an input from the slave device.
//
#define BOOTLOADER_HANDSHAKE_PIN            2
#define SLINT_TIMEOUT_MILLI_SECS            10000

//
// Platform specific pin:
// DRIVE_SLAVE_RESET_PIN is the pin connected to the nRst pin of the slave.
//
#define DRIVE_SLAVE_RESET_PIN               1

//
// Gpio In progress pin is the MRAM recovery progress pin coming from the
// DUT
//
#define GPIO_IN_PROGRESS_PIN                35

//
// Define the host UART module (0 or 1) to be used.
// Also define the max packet size
//
#define UART_HOST                       0
#define MAX_UART_PACKET_SIZE            (8192)
#define IOM_MODULE                      0
#define SLAVE_ADDRESS                   0x20
#define MAX_SPI_SIZE                    1023
#define MAX_I2C_SIZE                    255
#define MAX_IOS_LRAM_SIZE               120     // LRAM can only accept 120 bytes at a time.
//#define MAX_IOS_LRAM_SIZE             16      // LRAM can only accept 120 bytes at a time.

//
// Host IOS parameters.
//
#if (TARGET_DEVICE == AM_PART_APOLLO510L)
#define IOSOFFSET_WRITE_CMD             0xFF
#elif (TARGET_DEVICE == AM_PART_APOLLO510)
#define IOSOFFSET_WRITE_CMD             0x80
#endif
#define IOSOFFSET_READ_FIFO             0x7F
#define IOSOFFSET_READ_FIFOCTR          0x7C

//#define PRT_INFO        am_util_stdio_printf
#define PRT_INFO        no_print

//
// Define PRT_DATA if additional pkt data and other information is desired.
// PRT_INFO must also be defined.
//
#if PRT_INFO == am_util_stdio_printf
#define PRT_DATA        no_print
//#define PRT_DATA        am_util_stdio_printf
#endif


//*****************************************************************************
//
// Custom data type.
// Note - am_uart_buffer was simply derived from the am_hal_iom_buffer macro.
//
//*****************************************************************************
#define am_uart_buffer(A)                                           \
union                                                               \
{                                                                   \
    uint32_t words[(A + 3) >> 2];                                   \
    uint8_t bytes[A];                                               \
}

typedef struct
{
    uint32_t                     crc32;     // First word
    uint16_t                     msgType;   // am_secboot_wired_msgtype_e
    uint16_t                     length;
} am_secboot_wired_msghdr_t;

typedef struct
{
    uint32_t                      length  : 16;
    uint32_t                      resv    : 14;
    uint32_t                      bEnd    : 1;
    uint32_t                      bStart  : 1;
} am_secboot_ios_pkthdr_t;

typedef enum
{
    AM_SECBOOT_WIRED_MSGTYPE_HELLO,
    AM_SECBOOT_WIRED_MSGTYPE_STATUS,
    AM_SECBOOT_WIRED_MSGTYPE_OTADESC,
    AM_SECBOOT_WIRED_MSGTYPE_UPDATE,
    AM_SECBOOT_WIRED_MSGTYPE_ABORT,
    AM_SECBOOT_WIRED_MSGTYPE_RECOVER,
    AM_SECBOOT_WIRED_MSGTYPE_RESET,
    AM_SECBOOT_WIRED_MSGTYPE_ACK,
    AM_SECBOOT_WIRED_MSGTYPE_DATA,
} am_secboot_wired_msgtype_e;

typedef enum
{
    BOOTROM_UPDATE,
    SBL_WIRED_UPDATE,
}uart_ios_bridge_mode_e;


//
// IOM pins for Apollo5 EVB
//
uint32_t g_IOMPins[2][4] =
{
    // IOM0 PINS
    // SCK      MOSI,   MISO,   CE
    {   5,      6,      7,      10},
    // IOM1 Pins
    {   8,      9,      10,     92}

};

struct
{
    am_secboot_ios_pkthdr_t       header;
    uint8_t                       data[MAX_IOS_LRAM_SIZE - sizeof(am_secboot_ios_pkthdr_t)];
} g_IosPktData;


#endif // UART_IOS_BRIDGE_H
