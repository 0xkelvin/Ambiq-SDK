//*****************************************************************************
//
//! @file ios_fifo_controller.c
//!
//! @brief Advanced IOS FIFO controller demonstrating data exchange protocols.
//!
//! @addtogroup interface_examples Interface Examples
//!
//! @defgroup ios_fifo_controller IOM FIFO Controller Example
//! @ingroup interface_examples
//! @{
//!
//! Purpose: This example implements a sophisticated data exchange protocol
//! between two EVBs using IOM-IOS FIFO communication. The controller manages
//! data flow with command-based control, implementing a state machine for
//! synchronized data transfer with simulated sensor data handling on the
//! device side.
//!
//! @section ios_fifo_controller_features Key Features
//!
//! 1. @b Command @b Protocol: Implements one-byte command protocol for
//!    controlling data accumulation and transfer operations
//!
//! 2. @b State @b Machine: Manages synchronized data exchange with
//!    comprehensive state tracking and flow control
//!
//! 3. @b Sensor @b Data @b Handling: Supports simulated sensor data
//!    accumulation with CTimer-based event generation
//!
//! @section ios_fifo_controller_functionality Functionality
//!
//! The application provides:
//! - Command-based control for Start/Stop operations
//! - Block data transfer acknowledgment handling
//! - Real-time status monitoring via ITM SWO
//! - Synchronized data exchange with device
//!
//! @section ios_fifo_controller_usage Usage
//!
//! 1. Configure two EVBs with controller and device firmware
//! 2. Connect EVBs using appropriate SPI/I2C connections
//! 3. Monitor operation via ITM SWO at 1MHz
//! 4. Observe data exchange and state transitions
//!
//! Additional Information:
//! Requires a companion device running ios_fifo example.
//! The device implements sensor simulation with CTimer events.
//!
//! The IOS interrupts the controller to indicate data availability. The controller then
//! reads the available data (as indicated by FIFOCTR) by READing using IOS FIFO
//! (at address 0x7F).  The IOS keeps accumulating any new data coming in the
//! background.
//!
//! Controller sends an acknowledgement to IOS once it has finished reading a block
//! of data initiated by IOS (partitally or complete). IOS interrupts the controller
//! again if and when it has more data for the controller to read, and the cycle
//! repeats - till controller indicates that it is no longer interested in receiving
//! data by sending STOP command.
//!
//! Additional Information:
//! In order to run this example, a device (e.g. a second EVB) must be set
//! up to run the companion example, ios_fifo. The two EVBs can be connected
//! using fly leads between the two EVBs as follows.
//!
//! @verbatim
//! The pin jumpers should be connected as follows, as defined in bsp_pins.src:
//!
//! Apollo510 EVB
//!
//! SPI:
//!     CONTROLLER (ios_fifo_controller)             DEVICE (ios_fifo)
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
//!     CONTROLLER (ios_fifo_controller)         DEVICE (ios_fifo)
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
#include <stdint.h>
#include <stdbool.h>
#include "ios_fifo_common.h"

// How much data to read from device before ending the test
#define MAX_SIZE                    10000

#define IOSOFFSET_WRITE_INTEN       (AM_HAL_IOS_IOINTEN_OFFSET | (USE_SPI << 7))
#define IOSOFFSET_WRITE_INTCLR      (AM_HAL_IOS_IOINTCLR_OFFSET | (USE_SPI << 7))
#define IOSOFFSET_WRITE_CMD         (0x0 | (USE_SPI << 7))

#define IOSOFFSET_READ_INTSTAT      AM_HAL_IOS_IOINT_OFFSET
#define IOSOFFSET_READ_FIFO         AM_HAL_IOS_FIFO_OFFSET
#define IOSOFFSET_READ_FIFOCTR      AM_HAL_IOS_FIFOCTRLO_OFFSET

//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************
#define AM_TEST_RCV_BUF_SIZE    1024 // Max Size we can receive is 1023
uint8_t g_pui8RcvBuf[AM_TEST_RCV_BUF_SIZE];
volatile uint32_t g_startIdx = 0;
volatile bool bIosInt = false;
void *g_IOMHandle;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
static am_hal_iom_config_t g_sIOMSpiConfig =
{
    .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
    .ui32ClockFreq = AM_HAL_IOM_1MHZ,
    .eSpiMode = AM_HAL_IOM_SPI_MODE_0,
#if defined(AM_PART_APOLLO330P_510L) || defined(AM_PART_APOLLO510)
    .eClockSelPriority  = AM_HAL_IOM_PREFER_SYMMETRICAL_DC,
#endif

};

#define MAX_SPI_SIZE    1023

static am_hal_iom_config_t g_sIOMI2cConfig =
{
    .eInterfaceMode = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq  = AM_HAL_IOM_1MHZ,
};

#define MAX_I2C_SIZE   255

const am_hal_gpio_pincfg_t g_AM_BSP_GPIO_HANDSHAKE =
{
    .GP.cfg_b.uFuncSel       = AM_HAL_PIN_4_GPIO,
    .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
    .GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
};

//*****************************************************************************
//
// Clear Rx Buffer for comparison
//
//*****************************************************************************
void clear_rx_buf(void)
{
    uint32_t i;
    for ( i = 0; i < AM_TEST_RCV_BUF_SIZE; i++ )
    {
        g_pui8RcvBuf[i] = EMPTY_BYTE;
    }
}

//*****************************************************************************
//
// Validate Rx Buffer
// Returns 0 for success case
//
//*****************************************************************************
uint32_t validate_rx_buf(uint32_t rxSize)
{
    uint32_t i;
    for ( i = 0; i < rxSize; i++ )
    {
        if ( g_pui8RcvBuf[i] != (((g_startIdx + i) & 0xFF) ^ XOR_BYTE) )
        {
            am_util_stdio_printf("Failed to compare buffers at index %d \n", i);
            break;
        }
    }
    // Set the reference for next chunk
    g_startIdx += rxSize;
    return (i == rxSize);
}

// ISR callback for the controller IOINT
void cntlr_int_handler(void)
{
    bIosInt = true;
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio0_001f_isr(void)
{
    //
    // Read and clear the GPIO interrupt status.
    //
    uint32_t    ui32IntStatus;
    AM_CRITICAL_BEGIN
    am_hal_gpio_interrupt_irq_status_get(GPIO0_001F_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, ui32IntStatus);
    AM_CRITICAL_END
    am_hal_gpio_interrupt_service(GPIO0_001F_IRQn, ui32IntStatus);
}

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

    if ( USE_SPI )
    {
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
    }
    else
    {
        Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
    }
    ui32Status = am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("IOM Blocking read failed: %d\n", ui32Status);
    }
}

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

    if ( USE_SPI )
    {
        Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;
    }
    else
    {
        Transaction.uPeerInfo.ui32I2CDevAddr = I2C_ADDR;
    }
    ui32Status = am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
    if (ui32Status != AM_HAL_STATUS_SUCCESS)
    {
        am_util_stdio_printf("IOM Blocking write failed: %d\n", ui32Status);
    }
}

static void iom_set_up(void)
{
    uint32_t ioIntEnable = AM_IOSTEST_IOSTOIOM_DATAAVAIL_INTMASK;

    //
    // Initialize the IOM.
    //
    am_hal_iom_initialize(TEST_IOS_CNTLR_MODULE, &g_IOMHandle);

    am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);

    if ( USE_SPI )
    {
        //
        // Set the required configuration settings for the IOM.
        //
        am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);

        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(TEST_IOS_CNTLR_MODULE, AM_HAL_IOM_SPI_MODE);
    }
    else
    {
        //
        // Set the required configuration settings for the IOM.
        //
        am_hal_iom_configure(g_IOMHandle, &g_sIOMI2cConfig);

        //
        // Configure the IOM pins.
        //
        am_bsp_iom_pins_enable(TEST_IOS_CNTLR_MODULE, AM_HAL_IOM_I2C_MODE);
    }

    //
    // Enable the IOM.
    //
    am_hal_iom_enable(g_IOMHandle);
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, g_AM_BSP_GPIO_HANDSHAKE);

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

    // Set up IOCTL interrupts
    // IOS ==> IOM
    iom_device_write(IOSOFFSET_WRITE_INTEN, &ioIntEnable, 1);
}

uint32_t g_ui32LastUpdate = 0;

//*****************************************************************************
//
// Print a progress message.
//
//*****************************************************************************
void update_progress(uint32_t ui32NumBytes)
{
    //
    // Print a dot every 1000 bytes.
    //
    if ( (ui32NumBytes - g_ui32LastUpdate) > 1000 )
    {
        am_util_stdio_printf(".");
        g_ui32LastUpdate = ui32NumBytes;
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int main(void)
{
    bool bReadIosData = false;
    bool bDone = false;
    uint32_t data;

    common_setup();

    am_util_stdio_printf("IOS *FIFO* [CONTROLLER] Example\n");
    am_util_stdio_printf("Note : IOS Test controller: Waiting for at least %d bytes from the device.\n", MAX_SIZE);

    //
    // Allow time for all printing to finish.
    //
    am_util_delay_ms(10);

    //
    // Enable Interrupts.
    //
    am_hal_interrupt_master_enable();

    iom_set_up();

    // Make sure the print is complete
    am_util_delay_ms(100);

    // Send the START
    data = AM_IOSTEST_CMD_START_DATA;
    iom_device_write(IOSOFFSET_WRITE_CMD, &data, 1);

    //
    // Loop forever.
    //
    while ( !bDone )
    {
        if ( bIosInt == true )
        {
            bIosInt = false;
            // Read & Clear the IOINT status
            iom_device_read(IOSOFFSET_READ_INTSTAT, &data, 1);
            // We need to clear the bit by writing to IOS
            if ( data & AM_IOSTEST_IOSTOIOM_DATAAVAIL_INTMASK )
            {
                data = AM_IOSTEST_IOSTOIOM_DATAAVAIL_INTMASK;
                iom_device_write(IOSOFFSET_WRITE_INTCLR, &data, 1);
                // Set bReadIosData
                bReadIosData = true;
            }
            if ( bReadIosData )
            {
                uint32_t iosSize = 0;

                bReadIosData = false;
#if (ENABLE_DMA == 0)
                // Read the Data Size
                uint32_t maxSize = (USE_SPI) ? MAX_SPI_SIZE: MAX_I2C_SIZE;
                iom_device_read(IOSOFFSET_READ_FIFOCTR, &iosSize, 2);
                iosSize = (iosSize > maxSize)? maxSize: iosSize;
#else
                iom_device_read(4, &iosSize, 2);
#endif
                // Initialize Rx Buffer for later comparison
                clear_rx_buf();
#if (ENABLE_DMA == 1)
                data = AM_IOSTEST_CMD_ACK_DATA;
                iom_device_write(IOSOFFSET_WRITE_CMD, &data, 1);
                am_util_delay_us(500);
#endif
                // Read the data
                iom_device_read(IOSOFFSET_READ_FIFO,
                    (uint32_t *)g_pui8RcvBuf, iosSize);
                // Validate Content
                if ( !validate_rx_buf(iosSize) )
                {
                    am_util_stdio_printf("\nData Verification failed Accum:%lu rx=%d\n",
                        g_startIdx, iosSize);
                }
                update_progress(g_startIdx);
#if (ENABLE_DMA == 0)
                // Send the ACK/STOP
                data = AM_IOSTEST_CMD_ACK_DATA;

                if ( g_startIdx >= MAX_SIZE )
                {
                    bDone = true;
                    data = AM_IOSTEST_CMD_STOP_DATA;
                }
                iom_device_write(IOSOFFSET_WRITE_CMD, &data, 1);
#else
                am_util_delay_us(100);
                data = AM_IOSTEST_CMD_START_DATA;
                if ( g_startIdx >= MAX_SIZE )
                {
                    bDone = true;
                    data = AM_IOSTEST_CMD_STOP_DATA;
                }
                iom_device_write(IOSOFFSET_WRITE_CMD, &data, 1);
#endif
            }
        }
        else
        {
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        }
    }
    am_util_stdio_printf("\nTest Done - Total Received = =%d\n", g_startIdx);
    while (1);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

