//*****************************************************************************
//
//! @file ios_fullduplex_controller.c
//!
//! @brief Full-duplex IOM controller demonstrating bidirectional data transfer.
//!
//! @addtogroup interface_examples Interface Examples
//!
//! @defgroup ios_fullduplex_controller IOS FullDuplex Controller Example
//! @ingroup interface_examples
//! @{
//!
//! Purpose: This example demonstrates full-duplex communication between
//! IOM and IOS interfaces, implementing bidirectional data transfer with
//! pattern verification. It showcases high-speed, simultaneous transmit
//! and receive operations with comprehensive data validation.
//!
//! @section ios_fullduplex_controller_features Key Features
//!
//! 1. @b Full @b Duplex @b Operation: Implements simultaneous
//!    bidirectional data transfer between IOM and IOS
//!
//! 2. @b Pattern @b Verification: Validates data accuracy with
//!    predefined test patterns and verification
//!
//! 3. @b Real-time @b Monitoring: Provides status updates and
//!    verification results via SWO interface
//!
//! @section ios_fullduplex_controller_functionality Functionality
//!
//! The application provides:
//! - Bidirectional data transfer using FIFO (address 0x7F)
//! - Pattern generation and verification
//! - Status monitoring via SWO at 1MHz
//! - Error detection and reporting
//!
//! @section ios_fullduplex_controller_usage Usage
//!
//! 1. Configure two EVBs with controller and device firmware
//! 2. Connect EVBs using pin jumper connections
//! 3. Monitor operation via SWO (1MHz, 8-n-1)
//! 4. Verify pattern accuracy and transfer completion
//!
//! Hardware Configuration:
//! @verbatim
//! The pin jumpers should be connected as follows, as defined in bsp_pins.src:
//!
//! Apollo510 EVB
//!
//! SPI:
//!     CONTROLLER (ios_fullduplex_controller)  DEVICE (ios_fullduplex)
//!     --------------------                    ----------------
//!     GPIO[47] IOM5 SPI SCK                   GPIO[0]  IOS IOSFD_SCK
//!     GPIO[48] IOM5 SPI MOSI                  GPIO[1]  IOS IOSFD_MOSI
//!     GPIO[49] IOM5 SPI MISO                  GPIO[2]  IOS IOSFD_MISO
//!     GPIO[60] IOM5 SPI nCE                   GPIO[3]  IOS IOSFD_CE
//!     GPIO[4]  REQ_ACK (device to controller) GPIO[4]  IOS IOSFD_INT
//!     GND                                     GND
//!
//! Pin 0, 1, 2, 3, 4, 47, 48, 49 : J8
//! Pin 60                        : J9
//!
//! Apollo510B EVB
//!
//! SPI:
//!     CONTROLLER (ios_fullduplex_controller)   DEVICE (ios_fullduplex)
//!     --------------------                     ----------------
//!     GPIO[22]  IOM7 SPI SCK                   GPIO[0]  IOS IOSFD_SCK
//!     GPIO[23]  IOM7 SPI MOSI                  GPIO[1]  IOS IOSFD_MOSI
//!     GPIO[24]  IOM7 SPI MISO                  GPIO[2]  IOS IOSFD_MISO
//!     GPIO[5]   IOM7 SPI nCE                   GPIO[3]  IOS IOSFD_CE
//!     GPIO[4]   REQ_ACK (device to controller) GPIO[4]  IOS IOSFD_INT
//!     GND                                      GND
//!
//! Pin 0, 1, 2, 3, 4, 5, 22, 23, 24   : J8
//! @endverbatim
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
#include "ios_fullduplex_common.h"

void *g_IOMHandle;

am_hal_iom_config_t     g_sIOMSpiConfig =
{
    .eInterfaceMode       = AM_HAL_IOM_SPI_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_1MHZ,
    .eSpiMode             = AM_HAL_IOM_SPI_MODE_0,
    .ui32NBTxnBufLength   = 0,
    .pNBTxnBuf = NULL,
#if defined(AM_PART_APOLLO330P_510L) || defined(AM_PART_APOLLO510)
    .eClockSelPriority  = AM_HAL_IOM_PREFER_SYMMETRICAL_DC,
#endif
};

const am_hal_gpio_pincfg_t g_AM_BSP_IOM_GPIO_ENABLE =
{
    .GP.cfg_b.uFuncSel            = AM_HAL_PIN_4_GPIO,
    .GP.cfg_b.eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.eIntDir             = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPInput            = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

bool bHandshake = false;

#define IOSOFFSET_WRITE_INTEN       (AM_HAL_IOS_IOINTEN_OFFSET | 0x80)
#define IOSOFFSET_WRITE_INTCLR      (AM_HAL_IOS_IOINTCLR_OFFSET | 0x80)
#define IOSOFFSET_WRITE_CMD         0x80
#define IOSOFFSET_READ_INTSTAT      AM_HAL_IOS_IOINT_OFFSET
#define IOSOFFSET_READ_FIFO         AM_HAL_IOS_FIFO_OFFSET
#define IOSOFFSET_READ_FIFOCTR      AM_HAL_IOS_FIFOCTRLO_OFFSET

AM_SHARED_RW uint8_t g_pucInBuffer[AM_TX_BUFSIZE_MAX] __attribute__((aligned(32)));
AM_SHARED_RW uint8_t g_pucOutBuffer[AM_TX_BUFSIZE_MAX] __attribute__((aligned(32)));

//*****************************************************************************
//
// Pseudo random number generator.
//
//*****************************************************************************
#define DEFAULT_VL_M_W 0x12345678
#define DEFAULT_VL_M_Z 0x90abcdef
static          unsigned vl_m_w = DEFAULT_VL_M_W;   // Default seeds
static          unsigned vl_m_z = DEFAULT_VL_M_Z;   //  "
static volatile unsigned prng_seeded = 0;

void vl_SetSeed(unsigned newseed)
{
    if ( newseed != 0 )
    {
        vl_m_w = newseed;
        vl_m_z = (newseed ^ 0xFFFFFFFF) ^ DEFAULT_VL_M_Z;
        prng_seeded = 1;
    }
} // vl_SetSeed()

unsigned vl_RandVal(void)
{
    //
    //  These functions adapted from code found at the following URL on 5/25/14:
    //  http://www.codeproject.com/Articles/25172/Simple-Random-Number-Generation
    //
    vl_m_z = 36969 * (vl_m_z & 65535) + (vl_m_z >> 16);
    vl_m_w = 18000 * (vl_m_w & 65535) + (vl_m_w >> 16);
    return (vl_m_z << 16) + vl_m_w;
} // vl_RandVal()

unsigned vl_RandValRange(unsigned uMin, unsigned uMax)
{
    unsigned uDelta, uVal;

    // Returns a random value between min and max (inclusive).
    uDelta = uMax - uMin;
    if (uDelta == 0)
    {
        if (uMin == 0)
        {
            return vl_RandVal();    // Return a 32-bit random value
        }
        else
        {
            return uMin;            // Return the given value
        }
    }

    //
    // To get a random value within the given range, we'll simply take a random
    //  value and scale it into the range we need.
    //
    uVal = vl_RandVal() % (uDelta + 1);

    return uVal + uMin;
} // vl_RandValRange()

//*****************************************************************************
//
// IOM Generic Command Read function.
//
//*****************************************************************************
void iom_device_read(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;
    uint32_t ui32Status = 0;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui64Instr = offset;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32SpiChipSelect = 0;
    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("IOM Blocking read failed: %d\n", ui32Status);
    }
}

//*****************************************************************************
//
// IOM Generic Command Write function.
//
//*****************************************************************************
void iom_device_write(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;
    uint32_t ui32Status = 0;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui64Instr = offset;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32SpiChipSelect = 0;
    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("IOM Blocking write failed: %d\n", ui32Status);
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
    am_hal_gpio_mask_t IntStatus;
    uint32_t    ui32IntStatus;

    am_hal_gpio_interrupt_status_get(AM_HAL_GPIO_INT_CHANNEL_0,
                                     false,
                                     &IntStatus);
    am_hal_gpio_interrupt_irq_status_get(GPIO0_001F_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_service(GPIO0_001F_IRQn, ui32IntStatus);
}

// ISR callback for the controller IOINT
static void cntlr_int_handler(void *pArg)
{
    bHandshake = true;
}

//! Compare input and output buffers.
//! This checks that the received data matches transmitted data.
//! It also checks that any of the buffer memory beyond what is needed is not corrupted
static bool compare_buffers(int num_addr)
{
    // Compare the output buffer with input buffer
    for (uint32_t i = 0; i < sizeof(g_pucOutBuffer); i++)
    {
        if (g_pucInBuffer[i] != g_pucOutBuffer[i])
        {
            am_util_stdio_printf("Buffer miscompare at location %d\n", i);
            am_util_stdio_printf("TX Value = %2X | RX Value = %2X\n", g_pucOutBuffer[i], g_pucInBuffer[i]);
            return false;
        }
    }
    return true;
}

static void iom_set_up(void)
{
    uint32_t data;

    //
    // Initialize the IOM.
    //
    am_hal_iom_initialize(TEST_IOS_CNTLR_MODULE, &g_IOMHandle);

    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(TEST_IOS_CNTLR_MODULE, AM_HAL_IOM_SPI_MODE);

    //
    // Enable the IOM.
    //
    am_hal_iom_enable(g_IOMHandle);
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, g_AM_BSP_IOM_GPIO_ENABLE);

    uint32_t IntNum = HANDSHAKE_PIN;
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
    // Set up the controller IO interrupt
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, (am_hal_gpio_mask_t*)&IntNum);
    // Register handler for IOS => IOM interrupt
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, HANDSHAKE_PIN,
                                    (am_hal_gpio_handler_t)cntlr_int_handler, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
    NVIC_SetPriority(GPIO0_001F_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(GPIO0_001F_IRQn);
    am_hal_interrupt_master_enable();

    data = HANDSHAKE_IOS_TO_IOM;
    iom_device_write(IOSOFFSET_WRITE_INTEN, &data, 1);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int main(void)
{
    uint32_t                ui32Status;
    am_hal_iom_transfer_t         Transaction;
    uint32_t data;
    int ix = 0;
    uint32_t ui32Rand;

    common_setup();

    am_util_stdio_printf("IOS *FullDuplex* [CONTROLLER] Example\n");

    // First initialize random pattern in whole buffer - out and in same
    for ( ix = 0; ix < sizeof(g_pucOutBuffer); ix++ )
    {
        if ( ((ix & 0x3) == 0) )
        {
            ui32Rand = vl_RandValRange(0, 0);
        }

        //
        // Store 1 byte to the output buffer, 1 to the input buffer.
        //
        g_pucOutBuffer[ix] = g_pucInBuffer[ix] = ui32Rand & 0xff;
        ui32Rand >>= 8;
    }
    iom_set_up();

    for ( uint32_t sizeIdx = 0; sizeIdx < TEST_PACKET_SIZE_MAX; sizeIdx++ )
    {
        // Initialize the buffer section which will be used - to initialize out and in different
        for (ix = 0; ix < PacketSizes[sizeIdx]; ix++)
        {
            g_pucOutBuffer[sizeIdx + ix] = ix;
        }

        //
        // Write the buffer to the loop device.
        //
        Transaction.ui8RepeatCount  = 0;
        Transaction.ui32PauseCondition = 0;
        Transaction.ui32StatusSetClr = 0;
        Transaction.eDirection      = AM_HAL_IOM_FULLDUPLEX;
        Transaction.ui32InstrLen    = 1;
        Transaction.ui64Instr       = IOSOFFSET_READ_FIFO;
        Transaction.ui32NumBytes    = PacketSizes[sizeIdx];
        Transaction.pui32TxBuffer   = (uint32_t *)(&g_pucOutBuffer[sizeIdx]);
        Transaction.pui32RxBuffer   = (uint32_t *)(&g_pucInBuffer[sizeIdx]);
        Transaction.bContinue       = false;
        Transaction.uPeerInfo.ui32SpiChipSelect = 0;
        am_util_stdio_printf("\nTest size = %d \n", PacketSizes[sizeIdx]);

        while(!bHandshake);
        // Read the IOINT status
        iom_device_read(IOSOFFSET_READ_INTSTAT, &data, 1);
        if ( data & HANDSHAKE_IOS_TO_IOM )
        {
            bHandshake = false;
        }
        else
        {
            am_util_stdio_printf("Handshake failed!\n");
            continue;
        }

        //
        // Start the transaction.
        //
        ui32Status = am_hal_iom_spi_blocking_fullduplex(g_IOMHandle, &Transaction);
        // Clear IOINT status after transaction completes
        data = 0xFF;
        iom_device_write(IOSOFFSET_WRITE_INTCLR, &data, 1);
        if ( ui32Status )
        {
            am_util_stdio_printf("Transaction failed!\n");
            continue;
        }
        //
        // Compare the receive buffer to the transmit buffer.
        //
        compare_buffers(PacketSizes[sizeIdx]);
    }
    am_util_stdio_printf("\nExample completes\n");
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

