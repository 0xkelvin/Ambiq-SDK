//*****************************************************************************
//
//! @file ios_fullduplex.c
//!
//! @brief Example to demonstrate the use of IOSFD
//!
//! @addtogroup interface_examples Interface Examples
//!
//! @defgroup ios_fullduplex IOS FullDuplex Example
//! @ingroup interface_examples
//! @{
//!
//! Purpose: This example demonstrates IOS full duplex functionality
//! for bidirectional data transfer with an Apollo IO Master (IOM) controller.
//! The application implements full duplex communication using IOSFD,
//! enabling simultaneous data transmission and reception.
//!
//! @section ios_fullduplex_features Key Features
//!
//! 1. @b Full @b Duplex @b Communication: Implements bidirectional data transfer
//!    between IOM and IOS for simultaneous transmission and reception
//!
//! 2. @b Pattern @b Verification: Sends and verifies data patterns for accuracy
//!    and data integrity validation
//!
//! 3. @b FIFO @b Based @b Transfer: Utilizes FIFO (address 0x7F) for efficient
//!    data exchange between device and controller
//!
//! 4. @b Interrupt @b Driven @b Operation: Uses interrupts for synchronization
//!    and efficient data flow control
//!
//! 5. @b Multi-Platform @b Support: Provides pin configuration for various
//!    Apollo EVB platforms (Apollo5, Apollo510, Apollo510B)
//!
//! @section ios_fullduplex_functionality Functionality
//!
//! The application performs the following operations:
//! - Implements full duplex communication between IOM and IOS
//! - Sends data patterns from IOS to IOM for verification
//! - Verifies data accuracy and integrity
//! - Utilizes FIFO-based data transfer at address 0x7F
//! - Provides interrupt-driven synchronization
//! - Supports multiple Apollo EVB platforms
//!
//! @section ios_fullduplex_usage Usage
//!
//! 1. Connect two EVBs as device and controller using specified pin configuration
//! 2. Compile and download the application to the device EVB
//! 3. Run the companion controller example on the second EVB
//! 4. Observe full duplex data transfer and pattern verification
//! 5. Monitor SWO output for status and verification results
//!
//! @section ios_fullduplex_configuration Configuration
//!
//! - @b FIFO @b Address: Data transfer at address 0x7F
//! - @b SPI @b Interface: Full duplex SPI communication
//! - @b SWO: Output for status and results (1MHz, 8-n-1 mode)
//! - @b Pin @b Mapping: Platform-specific pin configuration for Apollo EVBs
//!
//! SWO is configured in 1MHz, 8-n-1 mode.
//!
//! The example utilizes an IOM<->IOS transfer between two EVBs. Running them
//! requires jumpering pins in order to connect the IOM to the IOS.
//!
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

static void *g_pIOSFD0Handle;
static void *g_pIOSFD1Handle;
static bool g_bIOSFDWR, g_bIOSFDRD = false;
AM_SHARED_RW uint8_t g_pIosSendBuf[AM_TX_BUFSIZE_MAX] __attribute__((aligned(32)));
AM_SHARED_RW uint8_t g_pIosRecvBuf[AM_TX_BUFSIZE_MAX] __attribute__((aligned(32)));

static am_hal_ios_config_t g_sIOSFDConfig =
{
    .ui32InterfaceSelect = AM_HAL_IOS_USE_SPI,
    .ui32ROBase = 0x0,
    .ui32FIFOBase = 0x08,
    .ui32RAMBase = AM_HAL_IOSFD_FIFO_MAX_SIZE,
    .ui32FIFOThreshold = IOS_FIFO_THRESHOLD,
    .ui8WrapEnabled = 1,
};

#define AM_HAL_IOS_INT_ERR  (AM_HAL_IOS_INT_FOVFL | AM_HAL_IOS_INT_FUNDFL | AM_HAL_IOS_INT_FRDERR | AM_HAL_IOS_INT_GENAD)
#define AM_HAL_IOS_XCMP_INT (AM_HAL_IOS_INT_XCMPWR | AM_HAL_IOS_INT_XCMPWF | AM_HAL_IOS_INT_XCMPRR | AM_HAL_IOS_INT_XCMPRF)
#define AM_HAL_IOS_DMA_INT  (AM_HAL_IOS_INT_DCMP | AM_HAL_IOS_INT_DERR)

#define AM_TEST_TIMER       0
#define AM_TEST_TIMEOUT     1 // 1 Seconds

//*****************************************************************************
//
// Timer handling to implement timeout
//
//*****************************************************************************
static volatile bool g_bTimeOut = 0;

// Function to initialize Timer A0 to interrupt every X second.
static void
timeout_check_init(uint32_t seconds)
{
    am_hal_timer_config_t TimerCfg;

    //
    // Configure the test timer.
    //
    am_hal_timer_reset_config(AM_TEST_TIMER);
    am_hal_timer_default_config_set(&TimerCfg);
    TimerCfg.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    TimerCfg.ui32Compare0 = 6000000 * seconds;

    am_hal_timer_config(AM_TEST_TIMER, &TimerCfg);

    //
    // Clear the timer and its interrupt
    //
    am_hal_timer_clear(AM_TEST_TIMER);
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(AM_TEST_TIMER, AM_HAL_TIMER_COMPARE0));

    //
    // Enable the timer interrupt.
    //
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(AM_TEST_TIMER, AM_HAL_TIMER_COMPARE0));

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_EnableIRQ(TIMER0_IRQn);
}

//*****************************************************************************
//
// Inform controller of new data available to read
//
//*****************************************************************************
void inform_controller(void)
{
    uint32_t ui32Arg = HANDSHAKE_IOS_TO_IOM;
    // Notify the controller
    am_hal_ios_control(g_pIOSFD0Handle, AM_HAL_IOS_REQ_CNTLR_INTSET, &ui32Arg);
}

// Timer Interrupt Service Routine (ISR)
void
am_timer00_isr(void)
{
    //
    // Clear the test timer interrupt (write to clear).
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(AM_TEST_TIMER, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_clear(AM_TEST_TIMER);

    // Set Timeout flag
    g_bTimeOut = 1;
}

// IOSFD0 Interrupt Service Routine (ISR)
void am_ioslave_fd0_isr(void)
{
    uint32_t ui32Status;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    am_hal_ios_interrupt_status_get(g_pIOSFD0Handle, true, &ui32Status);

    am_hal_ios_interrupt_clear(g_pIOSFD0Handle, ui32Status);

    if (ui32Status & AM_HAL_IOS_INT_XCMPRF)
    {
        am_hal_ios_interrupt_service(g_pIOSFD0Handle, ui32Status);
    }
}

// IOSFD1 Interrupt Service Routine (ISR)
void am_ioslave_fd1_isr(void)
{
    uint32_t ui32Status;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    am_hal_ios_interrupt_status_get(g_pIOSFD1Handle, true, &ui32Status);

    am_hal_ios_interrupt_clear(g_pIOSFD1Handle, ui32Status);

    if (ui32Status & AM_HAL_IOS_INT_DCMP)
    {
        am_hal_ios_interrupt_service(g_pIOSFD1Handle, ui32Status);
    }
}

static void ios_send_complete(uint32_t transactionStatus, void *pCallbackCtxt)
{
    g_bIOSFDWR = true;
}

static void ios_recv_complete(uint32_t transactionStatus, void *pCallbackCtxt)
{
    am_hal_cachectrl_range_t sRange;
    g_bIOSFDRD = true;
    sRange.ui32StartAddr = (uint32_t)g_pIosRecvBuf;
    sRange.ui32Size = AM_TX_BUFSIZE_MAX;
    am_hal_cachectrl_dcache_invalidate(&sRange, false);
}

//*****************************************************************************
//
// Setup IOSFD modules
//
//*****************************************************************************
uint32_t iosfd_set_up(void)
{
    uint8_t roSize = g_sIOSFDConfig.ui32FIFOBase - g_sIOSFDConfig.ui32ROBase;
    uint8_t ahbRamSize = AM_HAL_IOSFD_FIFO_MAX_SIZE - g_sIOSFDConfig.ui32RAMBase;
    uint32_t i;

    // Configure SPI interface
    am_bsp_ios_pins_enable(AM_HAL_IOSFD_WR, AM_HAL_IOS_USE_SPI);

    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_initialize(AM_HAL_IOSFD_WR, &g_pIOSFD0Handle);
    am_hal_ios_power_ctrl(g_pIOSFD0Handle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_ios_configure(g_pIOSFD0Handle, &g_sIOSFDConfig);

    //
    // Configure the IOS interface and LRAM structure.
    //
    am_hal_ios_initialize(AM_HAL_IOSFD_RD, &g_pIOSFD1Handle);
    am_hal_ios_power_ctrl(g_pIOSFD1Handle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_ios_configure(g_pIOSFD1Handle, &g_sIOSFDConfig);

    // Initialize RO & AHB-RAM data with pattern
    for (i = 0; i < roSize; i++)
    {
        am_hal_iosfd0_pui8LRAM[g_sIOSFDConfig.ui32ROBase + i] = ROBUFFER_INIT;
        am_hal_iosfd1_pui8LRAM[g_sIOSFDConfig.ui32ROBase + i] = ROBUFFER_INIT;
    }
    for (i = 0; i < ahbRamSize; i++)
    {
        am_hal_iosfd0_pui8LRAM[g_sIOSFDConfig.ui32RAMBase + i] = AHBBUF_INIT;
        am_hal_iosfd1_pui8LRAM[g_sIOSFDConfig.ui32RAMBase + i] = AHBBUF_INIT;
    }

    //
    // Enable interrupts for the registers we're interested in.
    //
    am_hal_ios_interrupt_enable(g_pIOSFD0Handle, AM_HAL_IOS_INT_XCMPRF);
    am_hal_ios_interrupt_enable(g_pIOSFD1Handle, AM_HAL_IOS_DMA_INT);

    //
    // Set the bit in the NVIC to accept access interrupts from the IOSFD.
    //
    NVIC_EnableIRQ(IOSLAVEFD0_IRQn);
    NVIC_EnableIRQ(IOSLAVEFD1_IRQn);

    am_hal_interrupt_master_enable();

    // Set up the IOSINT interrupt pin
    am_hal_gpio_pinconfig(AM_BSP_GPIO_IOSFD0_INT, g_AM_BSP_GPIO_IOSFD0_INT);
    am_hal_gpio_state_write(AM_BSP_GPIO_IOSFD0_INT, AM_HAL_GPIO_OUTPUT_CLEAR);

    return 0;
}

//*****************************************************************************
//
// Cleanup IOSFD modules
//
//*****************************************************************************
uint32_t iosfd_clean_up(void)
{
    uint32_t ui32Arg = AM_HAL_IOS_ACCESS_INT_ALL;

    // Configure SPI interface
    am_bsp_ios_pins_disable(AM_HAL_IOSFD_WR, AM_HAL_IOS_USE_SPI);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // disable interrupts.
    //
    am_hal_ios_control(g_pIOSFD0Handle, AM_HAL_IOS_REQ_ACC_INTDIS, &ui32Arg);
    am_hal_ios_interrupt_disable(g_pIOSFD0Handle, AM_HAL_IOS_INT_ALL);
    am_hal_ios_control(g_pIOSFD1Handle, AM_HAL_IOS_REQ_ACC_INTDIS, &ui32Arg);
    am_hal_ios_interrupt_disable(g_pIOSFD1Handle, AM_HAL_IOS_INT_ALL);

    //
    // Clearance of FIFO
    //
    am_hal_ios_control(g_pIOSFD0Handle, AM_HAL_IOS_REQ_FIFO_BUF_CLR, NULL);
    am_hal_ios_control(g_pIOSFD1Handle, AM_HAL_IOS_REQ_FIFO_BUF_CLR, NULL);

    am_hal_ios_disable(g_pIOSFD0Handle);
    am_hal_ios_disable(g_pIOSFD1Handle);

    //
    // Clear the bit in the NVIC to accept access interrupts from the IOSFD.
    //
    NVIC_DisableIRQ(IOSLAVEFD0_IRQn);
    NVIC_DisableIRQ(IOSLAVEFD1_IRQn);

    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOSFD0);
    am_hal_pwrctrl_periph_disable(AM_HAL_PWRCTRL_PERIPH_IOSFD1);

    am_hal_ios_uninitialize(g_pIOSFD0Handle);
    am_hal_ios_uninitialize(g_pIOSFD1Handle);

    am_hal_interrupt_master_disable();
    return 0;
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int main(void)
{
    am_hal_ios_transfer_t Transaction;
    uint32_t ui32ReadSize, ui32SentSize = 0;
    uint32_t ui32Status = 0;
    uint32_t ui32Arg;

    common_setup();

    am_util_stdio_printf("IOS *FullDuplex* [DEVICE] Example\n");

    iosfd_set_up();

    for (uint32_t i = 0; i < AM_TX_BUFSIZE_MAX; i++)
    {
        g_pIosSendBuf[i] = i;
    }
    am_hal_cachectrl_range_t sRange;
    sRange.ui32StartAddr = (uint32_t)g_pIosSendBuf;
    sRange.ui32Size = AM_TX_BUFSIZE_MAX;
    am_hal_cachectrl_dcache_clean(&sRange);
    memset(g_pIosRecvBuf, 0, AM_TX_BUFSIZE_MAX);

    // Wait until the controller is ready
    while(1)
    {
        am_hal_ios_control(g_pIOSFD0Handle, AM_HAL_IOS_REQ_CNTLR_INTEN_GET, &ui32Arg);
        if ( ui32Arg == HANDSHAKE_IOS_TO_IOM )
        {
            break;
        }
        else
        {
            am_util_delay_ms(1);
        }
    }

    if (AM_TEST_TIMEOUT)
    {
        timeout_check_init(AM_TEST_TIMEOUT);
    }

    for ( uint32_t sizeIdx = 0; sizeIdx < TEST_PACKET_SIZE_MAX; sizeIdx++ )
    {
        Transaction.eDirection = AM_HAL_IOS_FD;
        Transaction.ui8Priority = 0;
        Transaction.pui32RxBuffer = (uint32_t *)g_pIosRecvBuf;
        Transaction.ui32NumBytes[AM_HAL_IOS_READ] = PacketSizes[sizeIdx];
        Transaction.pui32BytesTransferred[AM_HAL_IOS_READ] = &ui32ReadSize;
        Transaction.pfnCallback[AM_HAL_IOS_READ] = ios_recv_complete;
        Transaction.pui32TxBuffer = (uint32_t *)g_pIosSendBuf;
        Transaction.ui32NumBytes[AM_HAL_IOS_WRITE] = PacketSizes[sizeIdx];
        Transaction.pui32BytesTransferred[AM_HAL_IOS_WRITE] = &ui32SentSize;
        Transaction.pfnCallback[AM_HAL_IOS_WRITE] = ios_send_complete;
        am_util_stdio_printf("\nTest size = %d \n", PacketSizes[sizeIdx]);

        ui32Status = am_hal_ios_dma_fullduplex_transfer(g_pIOSFD0Handle, g_pIOSFD1Handle, &Transaction);
        if ( ui32Status )
        {
            am_util_stdio_printf("Transaction failed, error code 0x%x\n", ui32Status);
            continue;
        }
        inform_controller();

        g_bTimeOut = 0;
        //
        // Start test timer
        //
        am_hal_timer_start(AM_TEST_TIMER);

        while (1)
        {
            if (g_bIOSFDRD || g_bTimeOut)
            {
                break;
            }
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        }
        g_bIOSFDWR = false;
        g_bIOSFDRD = false;
        if (g_bTimeOut)
        {
            // Timeout!
            am_util_stdio_printf("Transaction timed out!\n");
            am_hal_ios_dma_fullduplex_transfer_abort(g_pIOSFD0Handle, g_pIOSFD1Handle);
            continue;
        }
        if (ui32ReadSize != PacketSizes[sizeIdx])
        {
            am_util_stdio_printf("Transaction incompleted!\n");
            continue;
        }
        for ( uint32_t i = 0; i < PacketSizes[sizeIdx]; i++ )
        {
            if ( g_pIosRecvBuf[i] != g_pIosSendBuf[i] )
            {
                am_util_stdio_printf("Buffer Validation failed @i=%d Rcvd 0x%x Expected 0x%x\n",
                    i, g_pIosRecvBuf[i], g_pIosSendBuf[i]);
                continue;
            }
        }
    }

    iosfd_clean_up();

    am_util_stdio_printf("\nExample completes\n");
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

