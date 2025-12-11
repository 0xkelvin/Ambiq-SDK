// ******************************************************************************
//!
//! @file uart_ios_bridge.c
//
//! @brief UART-to-IOS bridge: Converts UART transfer commands from a PC host
//!        to SPI transactions for Apollo5 SBL updates or MRAM recovery.
//
//! This program is runs for an intermediate host board that bridges the UART
//! to the targets IOS interface to work with the uart_wired_update.py and
//! uart_recovery_host.py scripts running on a PC, to perform SBL wired updates
//! and MRAM Recovery using the IOS wired interface.
//
//! Printing/logging is performed over SWO at 1MHz.
//
//! -----------------------------------------------------------------------------
//! PIN fly lead connections assumed:
//!    HOST Bridge(APOLLO510_EVB)[IOM]    TARGET (Apollo510_EVB)(IOS)
//!    ------------------                 ---------------------------------------
//!    GPIO[2]   GPIO Interrupt  <---     GPIO[0]  GPIO interrupt (SLINT)
//!    GPIO[18]  Boot OVERRIDE / --->     GPIO[18] Override pin/Initiate MRAM Recovery GPIO
//!              MRAM_RCVY_CTRL(INFO0)
//!    GPIO[1]   Reset           --->     Reset pin (or n/c)
//!    GND       Ground          <->      GND
//
//! Apollo5 SPI connections:
//!    IOM 0                              IOS 0
//!    GPIO[5]   IOM0 SPI CLK    --->     GPIO[11] IOS SPI SCK
//!    GPIO[7]   IOM0 SPI MISO   <---     GPIO[83] IOS SPI MISO
//!    GPIO[6]   IOM0 SPI MOSI   --->     GPIO[52] IOS SPI MOSI
//!    GPIO[10]  IOM0 SPI nCE    --->     GPIO[13] IOS SPI nCE
//
//! Reset/Override connections from the host are optional, but automate the
//! process. If not used, put the target device in the desired mode
//! before starting this bridge program.
//
//! General usage:
//!   1. Update pin #defines as needed.
//!   2. Connect all pins between host and target.
//!   3. Load and run this firmware on the host.
//!   4. Press reset. The subordinate is put in update mode.
//!   5. Start the UART script on the PC.
//!   6. The bridge converts UART commands to IOS (SPI/I2C) signals.
//
//! The program supports 3 operation:
//!  - SBL Wired Update: Transfers SBL update packets from the host to the
//!    subordinate device, reset the target device and puts it into
//!    boot-override. (Default Mode, on a simple reset) Use with
//!    uart_wired_update.py
//
//!  - Bridge Mode for MRAM Recovery: Does not reset the target, passively
//!    bridges UART <--> IOS. (When Reset w/SW1 pressed).Used with
//!    mram_recovery_host.py.
//
//!  - Initiate MRAM recovery via GPIO: When in BRIDGE mode, pressing SW1,
//!    will reset the target and initiate MRAM recovery via the "Initiate
//!    MRAM Recovery GPIO" pin.
//
// ******************************************************************************

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

#include "uart_ios_bridge.h"

_Static_assert((UART3_IRQn - UART0_IRQn) == 3, "UART IRQs are not sequential");

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
uint8_t g_pui8UARTTXBuffer[MAX_UART_PACKET_SIZE];
am_uart_buffer(12 * 1024) g_psWriteData;
am_uart_buffer(12 * 1024) g_psReadData;

volatile uint32_t   g_ui32UARTRxIndex           = 0;
volatile bool       g_bRxTimeoutFlag            = false;
volatile bool       g_bIosInt                     = false;
volatile bool       g_mramInProgEdgeSeen        = false;
static void         *g_IOMHandle;
static void         *g_pvUART;
volatile bool       g_bTimerIntOccured          = false;

//*****************************************************************************
//
// Forward declarations
//
//*****************************************************************************
static void iom_subordinate_write(uint32_t offset, uint32_t *pBuf, uint32_t size);
void start_boot_mode(bool bReset);

//*****************************************************************************
//
// Configuration structure for the IO manager.
//
//*****************************************************************************
static am_hal_gpio_pincfg_t g_AM_BSP_GPIO_BOOT_HANDSHAKE =
{
    .GP.cfg_b.uFuncSel       = AM_HAL_PIN_2_GPIO,
    .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.eGPOutCfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
};

static am_hal_gpio_pincfg_t g_AM_BSP_GPIO_MR_IN_PROG =
{
    .GP.cfg_b.uFuncSel       = AM_HAL_PIN_10_GPIO,
    .GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P1X,
    .GP.cfg_b.eGPOutCfg      = AM_HAL_GPIO_PIN_OUTCFG_DISABLE,
    .GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE,
    .GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI,
};

static am_hal_iom_config_t g_sIOMSpiConfig =
{
    .eInterfaceMode = AM_HAL_IOM_SPI_MODE,
    .ui32ClockFreq = AM_HAL_IOM_8MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_1MHZ,
//    .ui32ClockFreq = AM_HAL_IOM_2MHZ,
    .eSpiMode = AM_HAL_IOM_SPI_MODE_0,    // Default
};

//*****************************************************************************
//
// Stub for printf suppression
//
//*****************************************************************************
int no_print(char*pFmtStr, ...)
{
    return 0;
}

//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
#if UART_HOST == 0
void am_uart_isr(void)
#elif UART_HOST == 1
void am_uart1_isr(void)
#elif UART_HOST == 2
void am_uart2_isr(void)
#elif UART_HOST == 3
void am_uart3_isr(void)
#endif
{
    //
    // Service the FIFOs as necessary, and clear the interrupts.
    //
    uint32_t ui32IntStatus;

    am_hal_uart_interrupt_status_get(g_pvUART, &ui32IntStatus, true);
    am_hal_uart_interrupt_clear(g_pvUART, ui32IntStatus);
    am_hal_uart_interrupt_service(g_pvUART, ui32IntStatus);

    //
    // If there's an RX interrupt, handle it in a way that preserves the
    // timeout interrupt on gaps between packets.
    //
    if ( ui32IntStatus & (AM_HAL_UART_INT_RX_TMOUT | AM_HAL_UART_INT_RX) )
    {
        uint32_t ui32BytesRead;

        am_hal_uart_transfer_t sRead =
        {
            .eType = AM_HAL_UART_BLOCKING_READ,
            .pui8Data = (uint8_t *) &(g_psWriteData.bytes[g_ui32UARTRxIndex]),
            .ui32NumBytes = 23,
            .ui32TimeoutMs = 5,
            .pui32BytesTransferred = &ui32BytesRead,
        };

        am_hal_uart_transfer( g_pvUART, &sRead );

        g_ui32UARTRxIndex += ui32BytesRead;

        //
        // If there is a TMOUT interrupt, assume we have a compete packet, and
        // send it over SPI.
        //
        if ( ui32IntStatus & (AM_HAL_UART_INT_RX_TMOUT) )
        {
            g_bRxTimeoutFlag = true;
        }
    }
}


//*****************************************************************************
//
// GPIO ISR - Call HAL to service the IRQ and call the callback func
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

void
am_gpio1_203f_isr(void)
{
    am_hal_gpio_mask_t IntStatus;
    uint32_t    ui32IntStatus;

    am_hal_gpio_interrupt_status_get(AM_HAL_GPIO_INT_CHANNEL_1,
                                     false,
                                     &IntStatus);
    am_hal_gpio_interrupt_irq_status_get(GPIO1_203F_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO1_203F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_service(GPIO1_203F_IRQn, ui32IntStatus);
}

void
am_gpio1_405f_isr(void)
{
    am_hal_gpio_mask_t IntStatus;
    uint32_t    ui32IntStatus;

    am_hal_gpio_interrupt_status_get(AM_HAL_GPIO_INT_CHANNEL_1,
                                     false,
                                     &IntStatus);
    am_hal_gpio_interrupt_irq_status_get(GPIO1_405F_IRQn, false, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO1_405F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_service(GPIO1_405F_IRQn, ui32IntStatus);
}

//*****************************************************************************
//
// Callback: SLINT edge from DUT (Host-int)
//
//*****************************************************************************
static void hostint_handler(void *pArg)
{
    g_bIosInt = true;
    g_ui32UARTRxIndex = 0;
    //
    // Toggle the RTS line on the UART
    //
    if ( g_mramInProgEdgeSeen )
    {
        am_hal_gpio_output_set(AM_BSP_GPIO_COM_UART_RTS);
        am_util_delay_ms(20);
        am_hal_gpio_output_clear(AM_BSP_GPIO_COM_UART_RTS);
        g_mramInProgEdgeSeen = false;
    }
}

//*****************************************************************************
//
// Callback: MRAM-RECOVERY progress pin (both edges)
//
//*****************************************************************************
static void mram_rcv_progress_handler(void *pArg)
{
    g_mramInProgEdgeSeen = true;
}

//*****************************************************************************
//
// Callback: Initiate MRAM recovery
//
//*****************************************************************************
static void init_mram_recovery(void *pArg)
{
    //Initiate MRAM recovery
    //
    // Drive the override pin low to force the PERIPHERAL into boot mode.
    //
    am_hal_interrupt_master_disable();

    //
    // Drive the MRAM Recovery pin low to initiate MRAM recovery
    //
    uint32_t ui32GpioPinNum = INITIATE_MRAM_RCV_PIN;
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_1,
                                  AM_HAL_GPIO_INT_CTRL_INDV_DISABLE,
                                  (void *)&ui32GpioPinNum);
    start_boot_mode(true);
    g_ui32UARTRxIndex = 0;
    am_util_stdio_printf("GPIO MRAM Recovery initiated.\n");
    am_hal_interrupt_master_enable();
}


//*****************************************************************************
//
// Initialize the IOM.
//
//*****************************************************************************
static void iom_set_up(uint32_t iomModule)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    //
    // Initialize the IOM.
    //
    ui32Status = am_hal_iom_initialize(iomModule, &g_IOMHandle);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("IOM Init failed.\n");
    }

    ui32Status = am_hal_iom_power_ctrl(g_IOMHandle, AM_HAL_SYSCTRL_WAKE, false);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Failed to power IOM.\n");
    }

    ui32Status = am_hal_iom_configure(g_IOMHandle, &g_sIOMSpiConfig);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Failed to configure IOM.\n");
    }

    am_bsp_iom_pins_enable(iomModule, AM_HAL_IOM_SPI_MODE);
    am_hal_iom_enable(g_IOMHandle);
    am_hal_gpio_pinconfig(BOOTLOADER_HANDSHAKE_PIN, g_AM_BSP_GPIO_BOOT_HANDSHAKE);

    //
    // Set up the host IO interrupt
    //
    uint32_t IntNum = BOOTLOADER_HANDSHAKE_PIN;
    am_hal_gpio_pinconfig(IntNum, am_hal_gpio_pincfg_input);

    uint32_t ui32IntStatus = (1 << IntNum);
    am_hal_gpio_interrupt_irq_clear(GPIO0_001F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum, hostint_handler, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
    NVIC_EnableIRQ(GPIO0_001F_IRQn);
    NVIC_SetPriority(GPIO0_001F_IRQn, 2);
}

//*****************************************************************************
//
// Initialize the UART.
//
//*****************************************************************************
static void uart_set_up(uint32_t UARTNum)
{
    uint32_t ui32Status = AM_HAL_STATUS_SUCCESS;
    am_hal_uart_config_t sUartConfig =
    {
        //
        // Standard UART settings: 115200-8-N-1
        //
        .ui32BaudRate    = 115200,
        .eDataBits       = AM_HAL_UART_DATA_BITS_8,
        .eParity         = AM_HAL_UART_PARITY_NONE,
        .eStopBits       = AM_HAL_UART_ONE_STOP_BIT,
        .eFlowControl    = AM_HAL_UART_FLOW_CTRL_NONE,

        //
        // Set TX and RX FIFOs to interrupt at three-quarters full.
        //
        .eTXFifoLevel     = AM_HAL_UART_FIFO_LEVEL_24,
        .eRXFifoLevel     = AM_HAL_UART_FIFO_LEVEL_24,

    };

    //
    // Initialize the UART module.
    //
    ui32Status = am_hal_uart_initialize(UARTNum, &g_pvUART);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("UART Init failed.\n");
    }

    ui32Status = am_hal_uart_power_control(g_pvUART, AM_HAL_SYSCTRL_WAKE, false);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Failed to power UART.\n");
    }

    ui32Status = am_hal_uart_configure(g_pvUART, &sUartConfig);
    if ( ui32Status != AM_HAL_STATUS_SUCCESS )
    {
        am_util_stdio_printf("Failed to configure UART.\n");
    }

    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_TX, g_AM_BSP_GPIO_COM_UART_TX);
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RX, g_AM_BSP_GPIO_COM_UART_RX);
#if UART_HOST == 0
    am_hal_gpio_pinconfig(AM_BSP_GPIO_COM_UART_RTS, am_hal_gpio_pincfg_output);
    am_hal_gpio_state_write(AM_BSP_GPIO_COM_UART_RTS, AM_HAL_GPIO_OUTPUT_CLEAR);
#endif
    // Flush the UART TX FIFO.
    am_hal_uart_tx_flush(g_pvUART);

    //
    // Make sure to enable the interrupts for RX, since the HAL doesn't already
    // know we intend to use them.
    //
    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + UARTNum));
    NVIC_SetPriority((IRQn_Type)(UART0_IRQn + UARTNum), 2);
    am_hal_uart_interrupt_enable(g_pvUART, (AM_HAL_UART_INT_RX | AM_HAL_UART_INT_RX_TMOUT));
}

//*****************************************************************************
//
// Reset the subordinate device and force it into boot mode.
//
//*****************************************************************************
void start_boot_mode(bool bReset)
{
    if ( !bReset )
    {
        //
        // Drive RESET high and configure the pin.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_RESET_PIN, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_pinconfig(DRIVE_SLAVE_RESET_PIN, am_hal_gpio_pincfg_output);

        //
        // Drive the override pin high and configure the pin.
        //
        am_hal_gpio_state_write(TARGET_OVERRIDE_MRAM_RCV_PIN, AM_HAL_GPIO_OUTPUT_SET);
        am_hal_gpio_pinconfig(TARGET_OVERRIDE_MRAM_RCV_PIN, am_hal_gpio_pincfg_output);

        //
        // Configure Button1(EVB) to initialize MRAM Recovery on DUT
        //
        am_hal_gpio_pincfg_t sButton1PinCfg = AM_HAL_GPIO_PINCFG_INPUT;
        sButton1PinCfg.GP.cfg_b.ePullup = AM_HAL_GPIO_PIN_PULLUP_100K;
        am_hal_gpio_pinconfig(INITIATE_MRAM_RCV_PIN, sButton1PinCfg);
        am_hal_gpio_pinconfig(BOOTLOADER_HANDSHAKE_PIN, am_hal_gpio_pincfg_input);
    }
    else
    {   //
    // Drive the override pin low to force the PERIPHERAL into boot mode.
        //
        am_hal_gpio_state_write(TARGET_OVERRIDE_MRAM_RCV_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

        //
        // Short delay.
        //
        am_util_delay_us(20);

        //
        // Drive RESET low.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_RESET_PIN, AM_HAL_GPIO_OUTPUT_CLEAR);

        //
        // Short delay.
        //
        am_util_delay_us(20);

        //
        // Release RESET.
        //
        am_hal_gpio_state_write(DRIVE_SLAVE_RESET_PIN, AM_HAL_GPIO_OUTPUT_SET);

        //
        // Short delay.
        //
        am_util_delay_us(20);
    }
}
//*****************************************************************************
//
// Read a packet from the IOS.
//
//*****************************************************************************
void iom_subordinate_read(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui64Instr       = offset;
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;

    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}

//*****************************************************************************
//
// Write a packet to the IOS.
//
//*****************************************************************************
void iom_subordinate_write(uint32_t offset, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui32InstrLen    = 1;
    Transaction.ui64Instr       = offset;
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;
    Transaction.uPeerInfo.ui32SpiChipSelect = AM_BSP_IOM0_CS_CHNL;

    am_hal_iom_blocking_transfer(g_IOMHandle, &Transaction);
}

//*****************************************************************************
//
// Send a "HELLO" packet.
//
//*****************************************************************************
void send_hello()
{
    struct
    {
        am_secboot_ios_pkthdr_t   hdr;
        am_secboot_wired_msghdr_t msg;
    } pkt;

    pkt.hdr.bStart = 1;
    pkt.hdr.bEnd = 1;
    pkt.hdr.length = 12;
    pkt.msg.msgType = AM_SECBOOT_WIRED_MSGTYPE_HELLO;
    pkt.msg.length = sizeof(am_secboot_wired_msghdr_t);

    //
    // Compute CRC
    //
    PRT_INFO("send_hello: sending bytes: %d.\n", pkt.msg.length );
    am_hal_crc32((uint32_t)&pkt.msg.msgType, pkt.msg.length - sizeof(uint32_t), &pkt.msg.crc32);
    iom_subordinate_write(IOSOFFSET_WRITE_CMD, (uint32_t*)&pkt, sizeof(pkt));
}

//*****************************************************************************
//
// Function handles the actual transmission UART<->IOM
//
//*****************************************************************************
void start_bridge_transaction(uint32_t ui32MaxPktSize)
{

    bool     bIOShdr;
    uint32_t ui32DotCnt = 0;
    volatile uint32_t ui32CurrCnt = 0;
    uint32_t ui32XferSize;
    uint32_t ui32ByteCnt = 0;
    //g_ui32UARTRxIndex = 0;

    am_util_stdio_printf("Starting UART<--->SPI Bridge Operation.\n");

    //
    // Loop until SLINT timeout occurs.
    //
    for ( ; ; )
    {
        if ( g_bIosInt == true )
        {
            g_bIosInt = false;

            //
            // Read the Data Size from the subordinate device.
            //
            iom_subordinate_read(IOSOFFSET_READ_FIFOCTR, &ui32XferSize, 2);
            ui32XferSize &= 0xFF;
            ui32XferSize = (ui32XferSize > ui32MaxPktSize) ? ui32MaxPktSize : ui32XferSize;

            if ( ui32XferSize >= 0xFF )  // invalid size when MRAM recovery ends and SLINT goes high permenantly
            {
                ui32XferSize = 0;
            }

            if ( ui32XferSize > 0 )
            {
                //
                // Read the Data from the IOS.
                //
                iom_subordinate_read(IOSOFFSET_READ_FIFO, (uint32_t*)&g_psReadData, ui32XferSize);

                //
                // Write the Data to the UART.
                //
                uint32_t uartTXcnt;
                am_hal_uart_transfer_t sWrite = AM_HAL_UART_BLOCKING_WRITE_DEFAULTS;
                sWrite.pui8Data = g_psReadData.bytes,
                sWrite.ui32NumBytes = ui32XferSize,
                sWrite.pui32BytesTransferred = &uartTXcnt,

                PRT_INFO("\nUART%d: %3d bytes.\n", UART_HOST, ui32XferSize);
                bIOShdr = false;
                am_hal_uart_transfer(g_pvUART, &sWrite);
                am_util_delay_ms(1);
            }
            g_bIosInt = false;
        }
        else if ( g_bRxTimeoutFlag )
        {

            // bridge a small gap in the UART stream - up to 5 ms
            g_bRxTimeoutFlag = false;
            ui32CurrCnt = g_ui32UARTRxIndex;
            NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + UART_HOST));

            // if we got more UART data then wait for the next timeout
            if ( ui32CurrCnt != g_ui32UARTRxIndex )
            {
                break;
            }

            //
            // UART Buffer can contain more than one packet.
            // This outer loop parses
            //
            PRT_DATA("UART Data Size    : %d\n", g_ui32UARTRxIndex);

            uint32_t ui32UARTBufOffset = 0;
            uint32_t ui32PktLength = 0;
            while ( (ui32UARTBufOffset < g_ui32UARTRxIndex) && (g_ui32UARTRxIndex > 0) )
            {
                am_secboot_wired_msghdr_t *pHdr = (am_secboot_wired_msghdr_t *)&g_psWriteData.bytes[ui32UARTBufOffset];
                ui32PktLength = pHdr->length;

                PRT_DATA("Packet Length     : %d\n", ui32PktLength);

                if ( ui32PktLength != g_ui32UARTRxIndex )
                {
                    break;
                }

                if ( 0 == ui32PktLength || ui32PktLength > MAX_UART_PACKET_SIZE )
                {
                    PRT_DATA("Invalid Packet Length: %d\n", ui32PktLength);
                    g_ui32UARTRxIndex = 0;
                    g_bRxTimeoutFlag = false;
                    NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + UART_HOST));
                    break;
                }
#if (TARGET_DEVICE == AM_PART_APOLLO510)
                for ( uint32_t index = 0; index < ui32PktLength; index += sizeof(g_IosPktData.data) )
                {
                    PRT_DATA("index             : %d\n", index);

                    g_IosPktData.header.bStart = 0;
                    g_IosPktData.header.bEnd = 0;
                    g_IosPktData.header.length = 0;

                    //
                    // If this is the first packet, then set the Start flag.
                    //
                    if ( 0 == index )
                    {
                        g_IosPktData.header.bStart = 1;
                    }

                    //
                    // If this this the last packet, then set the End flag.
                    //
                    if ( (index + sizeof(g_IosPktData.data)) >= ui32PktLength )
                    {
                        g_IosPktData.header.bEnd = 1;
                    }

                    //
                    // Build and Send the next packet.
                    //
                    g_IosPktData.header.length = ((ui32PktLength - index) < sizeof(g_IosPktData.data)) ? (ui32PktLength - index) : sizeof(g_IosPktData.data);
                    memcpy(&g_IosPktData.data[0], &g_psWriteData.bytes[index + ui32UARTBufOffset], g_IosPktData.header.length);
                    g_IosPktData.header.length += sizeof(am_secboot_ios_pkthdr_t);
                    g_bIosInt = false;

                    PRT_DATA("IOS Length        : %d\n", g_IosPktData.header.length);
                    PRT_DATA("IOS Start Bit     : %d\n", g_IosPktData.header.bStart);
                    PRT_DATA("IOS End Bit       : %d\n", g_IosPktData.header.bEnd);

                    ui32ByteCnt += g_IosPktData.header.length;
                    if ( bIOShdr )
                    {
                        PRT_DATA("Pckt Length, %d\n", g_IosPktData.header.length );
                    }
                    else
                    {
                        PRT_DATA("IOM  : sent bytes: %d (count=%d)", g_IosPktData.header.length, ui32ByteCnt );
                        bIOShdr = true;
                    }

                    if ( ui32DotCnt >= 20 )
                    {
                        PRT_INFO("\n");
                        ui32DotCnt = 0;
                    }
                    PRT_INFO("*");
                    ui32DotCnt++;

                    iom_subordinate_write(IOSOFFSET_WRITE_CMD, (uint32_t*)&g_IosPktData, g_IosPktData.header.length);

                    //
                    // Wait for the GPIO Interrupt before sending the next packet.
                    //
                    for ( uint32_t timeout = 0; timeout < 20000; timeout++ )
                    {
                        if ( g_bIosInt )
                        {
                            PRT_DATA("Received Handshake for next packet\n");
                            break;
                        }
                        else
                        {
                            am_util_delay_us(1);
                        }
                    }

                    if ( !g_bIosInt )
                    {
                        PRT_DATA("Timed out waiting for Handshake signal\n");
                    }

                }
                ui32UARTBufOffset += ui32PktLength;
#elif (TARGET_DEVICE == AM_PART_APOLLO510L)
                //
                // Apollo510L does not use IOS packetization. Just send the data on UART.
                //
                iom_subordinate_write(IOSOFFSET_WRITE_CMD, (uint32_t*)&g_psWriteData, g_ui32UARTRxIndex);
                //
                // Wait for the GPIO Interrupt before sending the next packet.
                //
                for ( uint32_t timeout = 0; timeout < 10000; timeout++ )
                {
                    if ( g_bIosInt )
                    {
                        g_bIosInt--;
                        PRT_DATA("Received Handshake for next packet\n");
                        am_util_delay_ms(300);
                        break;
                    }
                    else
                    {
                        am_util_delay_us(1);
                    }
                }

                if ( !g_bIosInt )
                {
                    PRT_DATA("Timed out waiting for Handshake signal\n");
                }

                if ( ui32DotCnt >= 20 )
                {
                    PRT_INFO("\n");
                    ui32DotCnt = 0;
                }
                PRT_INFO("*\n");
                ui32DotCnt++;
#endif
            }
            if ( ui32PktLength == g_ui32UARTRxIndex )
            {
                g_ui32UARTRxIndex = 0;
            }

            g_bRxTimeoutFlag = false;
            NVIC_EnableIRQ((IRQn_Type)(UART0_IRQn + UART_HOST));
        }
    }
}

//*****************************************************************************
//
// Decides one of the three modes
//
//*****************************************************************************
uart_ios_bridge_mode_e decide_mode(void)
{
    //
    // Check if BUTTON 1 is pressed which will indicate that we will
    // have to initiate GPIO MRAM recovery on the DUT
    //
    uint32_t ui32ButtonHeld = 1;
    for ( int i = 0; i < 10; i++ )
    {
        if ( am_hal_gpio_input_read(INITIATE_MRAM_RCV_PIN) )
        {
            ui32ButtonHeld = 0;
            break;
        }
        am_util_delay_ms(1);
    }


    if ( ui32ButtonHeld )
    {
        am_util_stdio_printf("Button 1 Pressed\n");
        //
        // Wait till the button is released
        //
        while(!am_hal_gpio_input_read(INITIATE_MRAM_RCV_PIN));
        return BOOTROM_UPDATE;
    }

    //
    // No MRAM recovery flows have been discovered
    // Continue to SBL Wired Update(Default)
    //
    return SBL_WIRED_UPDATE;
}

//*****************************************************************************
//
// MODE 1 : Initialize GPIO initiated MRAM recovery
//
//*****************************************************************************
void bootrom_update_transaction_begin(uint32_t ui32MaxPktSize)
{
    //
    // Run BootROM update
    //
    am_util_stdio_printf("Running MODE 1 : BootROM Update\n");

    am_hal_interrupt_master_disable();

    am_hal_gpio_pinconfig(GPIO_IN_PROGRESS_PIN, g_AM_BSP_GPIO_MR_IN_PROG);

    //
    // Enable Interrupt on the GPIO in Progress Pin
    //
    uint32_t ui32IntNum = GPIO_IN_PROGRESS_PIN % 32;
    uint32_t ui32GpioPinNum = GPIO_IN_PROGRESS_PIN;
    uint32_t ui32IntStatus = (1 << (ui32IntNum));
    am_hal_gpio_interrupt_irq_clear(GPIO1_203F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_1, GPIO_IN_PROGRESS_PIN, mram_rcv_progress_handler, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_1,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&ui32GpioPinNum);
    ui32IntNum = INITIATE_MRAM_RCV_PIN % 32;
    ui32GpioPinNum = INITIATE_MRAM_RCV_PIN;
    ui32IntStatus = 0;
    ui32IntStatus = (1 << (ui32IntNum));
    am_hal_gpio_interrupt_irq_clear(GPIO1_405F_IRQn, ui32IntStatus);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_1, INITIATE_MRAM_RCV_PIN, init_mram_recovery, NULL);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_1,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&ui32GpioPinNum);

    NVIC_EnableIRQ(GPIO1_405F_IRQn);
    NVIC_SetPriority((IRQn_Type)(GPIO1_405F_IRQn), 2);
    NVIC_EnableIRQ(GPIO1_001F_IRQn);
    NVIC_SetPriority((IRQn_Type)(GPIO1_405F_IRQn), 2);

    am_hal_interrupt_master_enable();

    //
    // Start the bridge
    //
    start_bridge_transaction(ui32MaxPktSize);
}

//*****************************************************************************
//
// Handle the SBL Wired Update
//
//*****************************************************************************
void sbl_wired_update_transaction_begin(uint32_t ui32MaxPktSize)
{
    uint32_t ui32XferSize, ui32HdrSz;
    //
    // Run SBL update
    //
    am_util_stdio_printf("Running MODE 2 : SBL Update\n");

    //
    // Force the subordinate into boot mode.
    //
    start_boot_mode(true);

    //
    // The following section does not run for Apollo510L due to the use of DMA based transctions
    //
#ifndef AM_PART_APOLLO510L
    //
    // Wait for initial handshake signal to know that IOS interface is alive
    //
    while( !g_bIosInt );
    g_bIosInt = false;

    //
    // Short delay.
    //
    am_util_delay_ms(1);

    //
    // Send the "HELLO" message to connect to the interface.
    //
    send_hello();

    while( !g_bIosInt );
    g_bIosInt = false;

    //
    // Read the "STATUS" response from the subordinate device and
    // also determine the size of the transfer.
    //
    ui32HdrSz = sizeof(am_secboot_wired_msghdr_t);      // Size is 8 bytes
    iom_subordinate_read(IOSOFFSET_READ_FIFO, (uint32_t*)&g_psReadData, ui32HdrSz);
    ui32XferSize = ((am_secboot_wired_msghdr_t*)&g_psReadData)->length;

    //
    // Now that we know the size, get the rest of the data
    //
    while( !g_bIosInt );
    g_bIosInt = false;

    iom_subordinate_read(IOSOFFSET_READ_FIFO,
                   (uint32_t*)&g_psReadData.bytes[ui32HdrSz],
                   ui32XferSize - ui32HdrSz);
#endif
    //
    // Start the bridge
    //
    start_bridge_transaction(ui32MaxPktSize);
}

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32MaxSize = MAX_SPI_SIZE;
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
    am_util_stdio_printf("\nApollo5 UART to IOS Host Bridge\n");
    am_util_stdio_printf("SPI clock = %d.%d MHz\n",
                             g_sIOMSpiConfig.ui32ClockFreq / 1000000,
                             g_sIOMSpiConfig.ui32ClockFreq % 1000000);
    am_util_stdio_printf("Press Button 1 along with Reset for MRAM Recovery Operation\n");
    am_util_stdio_printf("(Configure the follwing pins on the DUT)\n");

    am_util_stdio_printf("\n* PIN connections :\n");
    am_util_stdio_printf("  BRIDGE BOARD (APOLLO510_EVB)(IOM)        TARGET (APOLLO510_EVB)(IOS)\n");
    am_util_stdio_printf("  ----------------------                   --------------------------------------\n");
    am_util_stdio_printf("  GPIO[2]  GPIO Interrupt       <---        GPIO interrupt (SLINT)\n");
    am_util_stdio_printf("  GPIO[18] Boot OVERRIDE(POL:0) --->        Boot Override pin (or n/c)\n");
    am_util_stdio_printf("  GPIO[32] MR IN PROGRESS(POL:1)<---        MRAM Recovery In-Progress pin\n");
    am_util_stdio_printf("  GPIO[17] Reset                --->        Reset pin (or n/c)\n");
    am_util_stdio_printf("  GND      Ground               <->         GND\n");

    am_util_stdio_printf("\n  Apollo510 IOM connections:\n");
    am_util_stdio_printf("    IOM0 (Bridge Board)                 IOS0 (TARGET)\n");
    am_util_stdio_printf("    GPIO[%d] IOM SPI CLK      --->      GPIO[11] IOS SPI SCK\n",  g_IOMPins[0][0]);
    am_util_stdio_printf("    GPIO[%d] IOM SPI MISO     <---      GPIO[83] IOS SPI MISO\n", g_IOMPins[0][2]);
    am_util_stdio_printf("    GPIO[%d] IOM SPI MOSI     --->      GPIO[52] IOS SPI MOSI\n", g_IOMPins[0][1]);
    am_util_stdio_printf("    GPIO[%d] IOM SPI nCE     --->      GPIO[13] IOS SPI nCE\n",  g_IOMPins[0][3]);

    am_util_stdio_printf("\n");

    //
    // Set and configure the reset/bootmode pins high, but don't reset subordinate.
    //
    start_boot_mode(false);

    //
    // Decide one of the three following modes
    //      1. MODE1 : Initialize MRAM Recovery using the OVERRIDE PIN connected to MRAM GPIO CTRL PIN
    //      2. MODE2(Default) : SBL WIRED UDPATE
    //
    uart_ios_bridge_mode_e eiosBridgeMode = decide_mode();

    //
    // Start the IOM interface.
    //
    iom_set_up(IOM_MODULE);

    //
    // Start up the UART interface.
    //
    uart_set_up(UART_HOST);

    am_hal_interrupt_master_enable();

    //
    // Run the selected mode
    //
    switch( eiosBridgeMode )
    {
        case BOOTROM_UPDATE:
            bootrom_update_transaction_begin(ui32MaxSize);
            break;
        case SBL_WIRED_UPDATE:
            sbl_wired_update_transaction_begin(ui32MaxSize);
            break;
        default:
            break;
    }

    am_util_stdio_printf("Bridge transaction completed.\n");

    while(1);

}

