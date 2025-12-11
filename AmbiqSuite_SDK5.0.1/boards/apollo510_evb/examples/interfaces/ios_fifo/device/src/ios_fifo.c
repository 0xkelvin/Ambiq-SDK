//*****************************************************************************
//
//! @file ios_fifo.c
//!
//! @brief Example device used for demonstrating the use of the IOS FIFO.
//!
//! @addtogroup interface_examples Interface Examples
//!
//! @defgroup ios_fifo IOM FIFO Host Example
//! @ingroup interface_examples
//! @{
//!
//! Purpose: This example demonstrates IOS FIFO device functionality
//! for data exchange with an Apollo IO Master (IOM) controller. The application
//! implements a protocol for data accumulation, command handling, and synchronized
//! data transfer using FIFO and interrupt-driven communication.
//!
//! @section ios_fifo_features Key Features
//!
//! 1. @b IOS @b FIFO @b Data @b Exchange: Implements FIFO-based data transfer
//!    protocol for efficient communication with IOM controller
//!
//! 2. @b Command @b Handling: Supports start/stop data accumulation and block
//!    acknowledgment commands from controller
//!
//! 3. @b Interrupt @b Driven @b Operation: Uses interrupts to synchronize data
//!    availability and transfer between device and controller
//!
//! 4. @b Sensor @b Emulation: Emulates sensor data input using CTimer events
//!    for demonstration and testing
//!
//! 5. @b Multi-Protocol @b Support: Operates over both SPI and I2C interfaces
//!    for flexible connectivity
//!
//! @section ios_fifo_functionality Functionality
//!
//! The application performs the following operations:
//! - Implements FIFO-based data accumulation and transfer protocol
//! - Handles commands from controller for data flow control
//! - Emulates sensor data input using timers
//! - Synchronizes data transfer using interrupts
//! - Supports both SPI and I2C communication modes
//! - Provides pin configuration for various Apollo EVB platforms
//!
//! @section ios_fifo_usage Usage
//!
//! 1. Connect two EVBs as device and controller using specified pin configuration
//! 2. Compile and download the application to the device EVB
//! 3. Run the companion controller example on the second EVB
//! 4. Observe synchronized data exchange between device and controller
//! 5. Monitor ITM output for status and progress (controller side)
//!
//! @section ios_fifo_configuration Configuration
//!
//! - @b Communication @b Mode: SPI or I2C selectable
//! - @b FIFO @b Address: Data transfer at address 0x7F, command at 0x80
//! - @b Interrupts: Used for data availability and synchronization
//! - @b Pin @b Mapping: Platform-specific pin configuration for Apollo EVBs
//!
//! The ios_fifo example has no print output.
//! The controller example does use the ITM SWO to let the user know progress and
//! status of the demonstration.
//!
//! This example implements the device part of a protocol for data exchange with
//! an Apollo IO Master (IOM).  The controller sends one byte commands on SPI/I2C by
//! writing to offset 0x80.
//!
//! The command is issued by the controller to Start/Stop Data accumulation, and also
//! to acknowledge read-complete of a block of data.
//!
//! On the IOS side, once it is asked to start accumulating data (using START
//! command), two CTimer based events emulate sensors sending data to IOS.
//! When IOS has some data for controller, it implements a state machine,
//! synchronizing with the controller.
//!
//! The IOS interrupts the controller to indicate data availability. The controller then
//! reads the available data (as indicated by FIFOCTR) by READing using IOS FIFO
//! (at address 0x7F). The IOS keeps accumulating any new data coming in the
//! background.
//!
//! Controller sends an acknowledgment to IOS once it has finished reading a block
//! of data initiated by IOS (partially or complete). IOS interrupts the controller
//! again if and when it has more data for the controller to read, and the cycle
//! repeats - till controller indicates that it is no longer interested in receiving
//! data by sending STOP command.
//!
//! Printing takes place over the ITM at 1MHz.
//!
//! Additional Information:
//! In order to run this example, a controller (e.g. a second EVB) must be set
//! up to run the controller example, ios_fifo_controller. The two EVBs can be connected
//! using fly leads between the two EVBs as follows.
//!
//! @verbatim
//! The pin jumpers should be connected as follows, as defined in bsp_pins.src:
//!
//! Apollo510 EVB
//!
//! SPI:
//!     CONTROLLER (ios_fifo_controller)        DEVICE (ios_fifo)
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
#include <string.h>
#include "ios_fifo_common.h"

#if (USE_SPI == 0)
#define     IOS_INTERFACE               (AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(I2C_ADDR << 1))
#define     IOS_IO                      AM_HAL_IOS_USE_I2C
#define     SENSOR0_DATA_SIZE           100
#else
#define     IOS_INTERFACE               AM_HAL_IOS_USE_SPI
#define     IOS_IO                      AM_HAL_IOS_USE_SPI
#define     SENSOR0_DATA_SIZE           200
#endif
#define     SENSOR1_DATA_SIZE           500

// Sensor Frequencies as factor of 12KHz
// This test silently 'drops' the sensor data if the FIFO can not accomodate it
// Hence the controller data validation test would still pass as long as all the data
// written to the FIFO made it to the controller intact
// This allows us to configure these values to unrealistically high values for
// testing purpose
#define     SENSOR0_FREQ   12 // 12 times a second
#define     SENSOR1_FREQ    7 // 7 times a second

#define     AM_HAL_IOS_INT_ERR  (AM_HAL_IOS_INT_FOVFL | AM_HAL_IOS_INT_FUNDFL | AM_HAL_IOS_INT_FRDERR)
#define     AM_HAL_IOS_XCMP_INT (AM_HAL_IOS_INT_XCMPWR | AM_HAL_IOS_INT_XCMPWF | AM_HAL_IOS_INT_XCMPRR | AM_HAL_IOS_INT_XCMPRF)
#define     AM_HAL_IOS_DMA_INT  (AM_HAL_IOS_INT_DCMP | AM_HAL_IOS_INT_DERR)

typedef enum
{
    AM_IOSTEST_DEVICE_STATE_NODATA   = 0,
    AM_IOSTEST_DEVICE_STATE_DATA     = 1,
} AM_IOSTEST_DEVICE_STATE_E;

volatile AM_IOSTEST_DEVICE_STATE_E g_iosState;
volatile uint32_t g_sendIdx = 0;
volatile bool g_bSensor0Data, g_bSensor1Data;
volatile bool g_bCntlrReady = false;
static void *g_pIOSHandle;

//*****************************************************************************
//
// Message buffers.
//
// data from the IOS interface, which is only 8 bits wide.
//
//*****************************************************************************
#define AM_TEST_REF_BUF_SIZE    512
uint8_t g_pui8TestBuf[AM_TEST_REF_BUF_SIZE];
AM_SHARED_RW uint8_t g_pui8SendBuf[AM_TEST_REF_BUF_SIZE] __attribute__((aligned(32)));

#define AM_IOS_TX_BUFSIZE_MAX   1023
AM_SHARED_RW uint8_t g_pui8TxFifoBuffer[AM_IOS_TX_BUFSIZE_MAX] __attribute__((aligned(32)));

//*****************************************************************************
//
// GPIO Configuration
//
//*****************************************************************************
const am_hal_gpio_pincfg_t g_AM_BSP_SLINT =
{
    .GP.cfg_b.uFuncSel            = SLINT_FUN_SEL,
    .GP.cfg_b.eDriveStrength      = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.eCEpol              = AM_HAL_GPIO_PIN_CEPOL_ACTIVEHIGH,
    .GP.cfg_b.eGPOutCfg           = AM_HAL_GPIO_PIN_OUTCFG_OPENDRAIN,
#ifdef AM_PART_APOLLO330P_510L
    .GP.cfg_b.ePullup             = AM_HAL_GPIO_PIN_PULLUP_50K,
#else
    .GP.cfg_b.ePullup             = AM_HAL_GPIO_PIN_PULLUP_24K,
#endif
};


//*****************************************************************************
//
// IOS device Configuration
//
//*****************************************************************************
static am_hal_ios_config_t g_sIOSConfig =
{
    // Configure the IOS in SPI/I2C mode.
    .ui32InterfaceSelect = IOS_INTERFACE,

    // Eliminate the "read-only" section, so an external controller can use the
    // entire "direct write" section.
    .ui32ROBase = 0x8,

    // Making the "FIFO" section as big as possible.
    .ui32FIFOBase = 0x8,
#if defined(AM_PART_APOLLO330P_510L)
    .ui32RAMBase = AM_HAL_IOSFD_FIFO_MAX_SIZE,
#else
#if (TEST_IOS_MODULE == 0)
    // We don't need any RAM space, so extend the FIFO all the way to the end
    // of the LRAM.
    .ui32RAMBase = AM_HAL_IOS_FIFO_MAX_SIZE,
#else
    .ui32RAMBase = AM_HAL_IOSFD_FIFO_MAX_SIZE,
#endif
#endif // AM_PART_APOLLO330P_510L

    .ui32FIFOThreshold = IOS_FIFO_THRESHOLD,
    // FIFO Wraparound Enable
    .ui8WrapEnabled = 1,

    .pui8SRAMBuffer = g_pui8TxFifoBuffer,
    .ui32SRAMBufferCap = AM_IOS_TX_BUFSIZE_MAX,
};

//*****************************************************************************
//
// Timer handling to emulate sensor data
//
//*****************************************************************************

// Emulate Sensor0 New Data
void timer0_handler(void)
{
    // Inform main loop of sensor 0 Data availability
    g_bSensor0Data = true;
    am_util_stdio_printf(".");
}

// Emulate Sensor1 New Data
void timer1_handler(void)
{
    // Inform main loop of sensor 1 Data availability
    g_bSensor1Data = true;
    am_util_stdio_printf("*");
}

// Timer0 Interrupt Service Routine (ISR)
void am_timer00_isr(void)
{
    //
    // Clear Timer0 Interrupt (write to clear).
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE0));
#if (ENABLE_DMA == 0)
    am_hal_timer_clear(0);
#else
    am_hal_timer_clear_stop(0);
#endif
    timer0_handler();
}

// Timer1 Interrupt Service Routine (ISR)
void am_timer02_isr(void)
{
    //
    // Clear Timer0 Interrupt (write to clear).
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(2, AM_HAL_TIMER_COMPARE0));
#if (ENABLE_DMA == 0)
    am_hal_timer_clear(2);
#else
    am_hal_timer_clear_stop(2);
#endif

    timer1_handler();
}

void stop_sensors(void)
{
    //
    // Stop timer 0
    //
    am_hal_timer_stop(0);
#if (ENABLE_DMA == 0)
    //
    // Stop timer 2
    //
    am_hal_timer_stop(2);
#endif
}

void start_sensors(void)
{
    stop_sensors(); // Just in case controller died without sending STOP last time
    //
    // Start timer 0
    //
    am_hal_timer_start(0);
#if (ENABLE_DMA == 0)
    // Initialize Data Buffer Index
    g_sendIdx = 0;
    //
    // Start timer 2
    //
    am_hal_timer_start(2);
#endif
    g_iosState = AM_IOSTEST_DEVICE_STATE_NODATA;
}

void init_sensors(void)
{
    //
    // Set up timer 0.
    //
    am_hal_timer_config_t       Timer0Config;
    am_hal_timer_default_config_set(&Timer0Config);
    Timer0Config.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4K;   // 96MHz/4K = 24KHz
    Timer0Config.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    Timer0Config.ui32Compare0 = 24000 / SENSOR0_FREQ ;          // Sensor 0 Freq

    am_hal_timer_config(0, &Timer0Config);
    am_hal_timer_clear_stop(0);

    //
    // Set up timer 2.
    //
    am_hal_timer_config_t       Timer2Config;
    am_hal_timer_default_config_set(&Timer2Config);
    Timer2Config.eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV4K;   // 96MHz/4K = 24KHz
    Timer2Config.eFunction = AM_HAL_TIMER_FN_UPCOUNT;
    Timer2Config.ui32Compare0 = 24000 / SENSOR1_FREQ ;          // Sensor 1 Freq

    am_hal_timer_config(2, &Timer2Config);
    am_hal_timer_clear_stop(2);

    //
    // Clear the timer Interrupt
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(2, AM_HAL_TIMER_COMPARE0));

    //
    // Enable the timer Interrupt.
    //
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(0, AM_HAL_TIMER_COMPARE0));
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(2, AM_HAL_TIMER_COMPARE0));

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_SetPriority(TIMER0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_SetPriority(TIMER2_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(TIMER2_IRQn);
}

//*****************************************************************************
//
// Configure the SPI device.
//
//*****************************************************************************
static void ios_set_up(void)
{
    //
    // Configure the IOS interface and LRAM structure.
    //
    am_bsp_ios_pins_enable(TEST_IOS_MODULE, IOS_IO);
    am_hal_ios_initialize(TEST_IOS_MODULE, &g_pIOSHandle);
    am_hal_ios_power_ctrl(g_pIOSHandle, AM_HAL_SYSCTRL_WAKE, false);
    am_hal_ios_configure(g_pIOSHandle, &g_sIOSConfig);

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_interrupt_clear(g_pIOSHandle, AM_HAL_IOS_INT_ALL);
#if (ENABLE_DMA == 1)
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_DMA_INT | AM_HAL_IOS_INT_XCMPRF | AM_HAL_IOS_INT_XCMPWR);
#else
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_INT_ERR | AM_HAL_IOS_INT_FSIZE);
    am_hal_ios_interrupt_enable(g_pIOSHandle, AM_HAL_IOS_XCMP_INT);
#endif

    //
    // Set the bit in the NVIC to accept access interrupts from the IO device.
    //
    NVIC_SetPriority(IOS_IRQ, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(IOS_IRQ);
    // Set up the IOSINT interrupt pin
#if SLINT_GPIO
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);
#else
    am_hal_gpio_pinconfig(HANDSHAKE_PIN, g_AM_BSP_SLINT);
#endif
}

// Inform controller of new data available to read
void inform_controller(void)
{
    uint32_t ui32Arg = AM_IOSTEST_IOSTOIOM_DATAAVAIL_INTMASK;
    // Update FIFOCTR for controller to read
    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_FIFO_UPDATE_CTR, NULL);
    // Interrupt the controller
    am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_CNTLR_INTSET, &ui32Arg);
#if SLINT_GPIO
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_SET);
#endif // #if SLINT_GPIO
}

#if (ENABLE_DMA == 1)
static uint32_t ios_dma_send(uint32_t size)
{
    uint32_t ui32SentSize = 0;
    am_hal_ios_transfer_t Transaction;
    // Send data using FIFO only
    Transaction.eDirection = AM_HAL_IOS_WRITE;
    Transaction.pui32TxBuffer = (uint32_t *)g_pui8SendBuf;
    Transaction.ui32NumBytes[AM_HAL_IOS_WRITE] = size;
    Transaction.pui32BytesTransferred[AM_HAL_IOS_WRITE] = &ui32SentSize;
    Transaction.ui8Priority = 0;
    Transaction.pfnCallback[AM_HAL_IOS_WRITE] = NULL;
    am_hal_ios_dma_transfer(g_pIOSHandle, &Transaction);

    return 0;
}
#endif

//*****************************************************************************
//
// IOS Main ISR.
//
//*****************************************************************************
void ios_isr(void)
{
    uint32_t ui32Status;
    uint8_t  *pui8Packet;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    am_hal_ios_interrupt_status_get(g_pIOSHandle, true, &ui32Status);

    am_hal_ios_interrupt_clear(g_pIOSHandle, ui32Status);
#if SLINT_GPIO
    am_hal_gpio_state_write(HANDSHAKE_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);   // clear SLINT anyway
#endif

#if (ENABLE_DMA == 1)
    if (ui32Status & AM_HAL_IOS_INT_XCMPRF)
    {
        uint32_t ui32DMADir = 0;
        am_hal_ios_control(g_pIOSHandle, AM_HAL_IOS_REQ_GET_DMA_DIR, &ui32DMADir);
        if ( ui32DMADir == AM_HAL_IOS_WRITE )
        {
            g_iosState = AM_IOSTEST_DEVICE_STATE_NODATA;
            am_hal_ios_interrupt_service(g_pIOSHandle, ui32Status);
        }
    }
#endif

    if (ui32Status & AM_HAL_IOS_INT_FUNDFL)
    {
        am_util_stdio_printf("Hitting underflow for the requested IOS FIFO transfer\n");
        // We should never hit this case unless the threshold has beeen set
        // incorrect, or we are unable to handle the data rate
        // ERROR!
        //am_hal_debug_assert_msg(0,
        //    "Hitting underflow for the requested IOS FIFO transfer.");
    }

    if (ui32Status & AM_HAL_IOS_INT_ERR)
    {
        // We should never hit this case
        // ERROR!
        //am_hal_debug_assert_msg(0,
        //    "Hitting ERROR case.");
    }

    if (ui32Status & AM_HAL_IOS_INT_FSIZE)
    {
        //
        // Service the I2C device FIFO if necessary.
        //
        am_hal_ios_interrupt_service(g_pIOSHandle, ui32Status);
    }

    if (ui32Status & AM_HAL_IOS_INT_XCMPWR)
    {
        //
        // Set up a pointer for writing 32-bit aligned packets through the IOS
        // interface.
        //
        pui8Packet = (uint8_t *) lram_array;
        switch(pui8Packet[0])
        {
            case AM_IOSTEST_CMD_START_DATA:
                // Controller wants to start data exchange
                // Start the Sensor Emulation
                start_sensors();
                break;

            case AM_IOSTEST_CMD_STOP_DATA:
                // Controller no longer interested in data from us
                // Stop the Sensor emulation
                stop_sensors();
                // Initialize Data Buffer Index
                g_sendIdx = 0;
                g_iosState = AM_IOSTEST_DEVICE_STATE_NODATA;
                am_util_stdio_printf("#");
                break;

            case AM_IOSTEST_CMD_ACK_DATA:
#if (ENABLE_DMA == 1)
                g_bCntlrReady = true;
#else
                // Controller done reading the last block signalled
                // Check if any more data available
                uint32_t ui32UsedSpace = 0;
                am_hal_ios_fifo_space_used(g_pIOSHandle, &ui32UsedSpace);
                if (ui32UsedSpace)
                {
                    g_iosState = AM_IOSTEST_DEVICE_STATE_DATA;
                    inform_controller();
                }
                else
                {
                    g_iosState = AM_IOSTEST_DEVICE_STATE_NODATA;
                }
#endif
                break;

            default:
                break;
        }
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int main(void)
{
    int i;
    uint32_t ui32Size = 0;

    common_setup();

    am_util_stdio_printf("IOS *FIFO* [DEVICE] Example\n");

    // Initialize Test Data
    for (i = 0; i < AM_TEST_REF_BUF_SIZE; i++)
    {
        g_pui8TestBuf[i] = (i & 0xFF) ^ XOR_BYTE;
    }
    memset(g_pui8SendBuf, 0, sizeof(g_pui8SendBuf));

    init_sensors();
    //
    // Enable the IOS. Choose the correct protocol based on USE_SPI
    //
    ios_set_up();

    //
    // Enable interrupts so we can receive messages from the boot controller.
    //
    am_hal_interrupt_master_enable();

    //
    // Loop forever.
    //
    while(1)
    {
        uint32_t chunk1;
        if (g_bSensor0Data || g_bSensor1Data)
        {
            if (g_bSensor0Data)
            {
                ui32Size = SENSOR0_DATA_SIZE;
                chunk1 = AM_TEST_REF_BUF_SIZE - g_sendIdx;
                if (chunk1 > ui32Size)
                {
                    memcpy(g_pui8SendBuf, &g_pui8TestBuf[g_sendIdx], ui32Size);
                }
                else
                {
                    memcpy(g_pui8SendBuf, &g_pui8TestBuf[g_sendIdx], chunk1);
                    memcpy(&g_pui8SendBuf[chunk1], g_pui8TestBuf, ui32Size - chunk1);
                }
#if (ENABLE_DMA == 1)
                lram_array[4] = ui32Size & 0xFF;
                lram_array[5] = (ui32Size & 0xFF00) >> 8;
                inform_controller();
#else
                uint32_t numWritten = 0;
                am_hal_ios_fifo_write(g_pIOSHandle, g_pui8SendBuf, ui32Size, &numWritten);
                g_sendIdx += numWritten;
                g_sendIdx %= AM_TEST_REF_BUF_SIZE;
#endif
                g_bSensor0Data = false;
            }
            if (g_bSensor1Data)
            {
                ui32Size = SENSOR1_DATA_SIZE;
                chunk1 = AM_TEST_REF_BUF_SIZE - g_sendIdx;
                if (chunk1 > ui32Size)
                {
                    memcpy(g_pui8SendBuf, &g_pui8TestBuf[g_sendIdx], ui32Size);
                }
                else
                {
                    memcpy(g_pui8SendBuf, &g_pui8TestBuf[g_sendIdx], chunk1);
                    memcpy(&g_pui8SendBuf[chunk1], g_pui8TestBuf, ui32Size - chunk1);
                }
#if (ENABLE_DMA == 1)
                lram_array[4] = ui32Size & 0xFF;
                lram_array[5] = (ui32Size & 0xFF00) >> 8;
                inform_controller();
#else
                uint32_t numWritten = 0;
                am_hal_ios_fifo_write(g_pIOSHandle, g_pui8SendBuf, ui32Size, &numWritten);
                g_sendIdx += numWritten;
                g_sendIdx %= AM_TEST_REF_BUF_SIZE;
#endif
                g_bSensor1Data = false;
            }
#if (ENABLE_DMA == 0)
            // If we were Idle - need to inform controller if there is new data
            uint32_t ui32UsedSpace = 0;
            if (g_iosState == AM_IOSTEST_DEVICE_STATE_NODATA)
            {
                am_hal_ios_fifo_space_used(g_pIOSHandle, &ui32UsedSpace);
                if (ui32UsedSpace)
                {
                    g_iosState = AM_IOSTEST_DEVICE_STATE_DATA;
                    inform_controller();
                }
            }
#endif
        }
#if (ENABLE_DMA == 1)
        else if ( g_bCntlrReady )
        {
            am_hal_cachectrl_range_t sRange;
            sRange.ui32StartAddr = (uint32_t)g_pui8SendBuf;
            sRange.ui32Size = AM_TEST_REF_BUF_SIZE;
            am_hal_cachectrl_dcache_clean(&sRange);
            g_bCntlrReady = false;
            ios_dma_send(ui32Size);
            g_sendIdx += ui32Size;
            g_sendIdx %= AM_TEST_REF_BUF_SIZE;
        }
#endif
        else
        {
            //
            // Go to Sleep.
            //
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        }
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

