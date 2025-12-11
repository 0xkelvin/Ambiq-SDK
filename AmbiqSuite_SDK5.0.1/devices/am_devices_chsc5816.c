//*****************************************************************************
//
//! @file am_devices_chsc5816.c
//!
//! @brief Functions for controlling the CHSC5816 Capacitive Touch Sensor
//!
//! @addtogroup devices_chsc5816 CHSC5816 - I2C Touch Driver
//! @ingroup devices
//! @{
//!
//! Purpose: This module provides a hardware abstraction layer
//!          for the CHSC5816 capacitive touch sensor, enabling reliable touch
//!          detection, event processing, and power-efficient operation for embedded
//!          applications requiring touch input capabilities. The driver supports
//!          multiple touch channels, customizable configuration, and features like
//!          gesture recognition and state tracking for optimal user interface
//!          performance and system integration.
//!
//! @section devices_chsc5816_features Key Features
//!
//! 1. @b Multi-channel @b Touch: Supports multiple touch inputs.
//! 2. @b Gesture @b Recognition: Detects gestures such as swipe and tap.
//! 3. @b State @b Management: Tracks touch state and changes.
//! 4. @b Power @b Efficiency: Low-power operation and sleep modes.
//! 5. @b Customizable @b Configuration: Flexible sensor and interrupt settings.
//!
//! @section devices_chsc5816_functionality Functionality
//!
//! - Initialize CHSC5816 device
//! - Configure touch channels and sensitivity
//! - Track and update touch state
//! - Support for gesture and event management
//!
//! @section devices_chsc5816_usage Usage
//!
//! 1. Initialize the device with am_devices_chsc5816_init()
//! 2. Periodically call am_devices_chsc5816_poll() to process touch events
//! 3. Use am_devices_chsc5816_get_event() to retrieve touch or gesture events
//!
//! @section devices_chsc5816_configuration Configuration
//!
//! - Configure touch channels and sensitivity
//! - Set up interrupt and power management options
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

#include "am_mcu_apollo.h"
#include "am_devices_chsc5816.h"
#include "am_bsp.h"
#include "string.h"
#include "am_util_delay.h"
#include "am_util.h"
#include "am_devices_display_generic.h"

#define PRINT_DBG               am_util_stdio_printf
#define semi_touch_msdelay      am_util_delay_ms
#define semi_touch_iic_write    chsc5816_write
#define semi_i2c_read_bytes     chsc5816_read

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
struct rt_i2c_msg
{
    uint16_t addr;
    uint16_t flags;
    uint16_t len;
    uint8_t  *buf;
};

struct touch_message
{
    rt_uint16_t x;
    rt_uint16_t y;
    rt_uint8_t event;
};

am_hal_iom_config_t     g_sI2cChsc5816Cfg =
{
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_400KHZ,
    .ui32NBTxnBufLength   = 0,
    .pNBTxnBuf = NULL,
};


static uint32_t chsc5816_i2c_read(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size);
static uint32_t chsc5816_i2c_write(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size);
static uint32_t chsc5816_master_xfer(struct rt_i2c_msg *msgs, uint32_t num);

static uint32_t chsc5816_i2c_noblocking_read(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt);
static uint32_t chsc5816_master_noblocking_xfer(struct rt_i2c_msg *msgs, uint32_t num, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt);

void *g_chsc5816_IOMHandle;

static uint32_t DMATCBBuffer[1024];
static am_hal_iom_config_t g_sI2cNBConfig =
{
    // Set up IOM
    // Initialize the Device
    .eInterfaceMode       = AM_HAL_IOM_I2C_MODE,
    .ui32ClockFreq        = AM_HAL_IOM_400KHZ,
    .ui32NBTxnBufLength = sizeof(DMATCBBuffer) / 4,
    .pNBTxnBuf = DMATCBBuffer
};

//*****************************************************************************
//
//! @brief
//! @param value - the value to be converted
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
swap_endian(uint32_t value)
{
    uint32_t result = 0;
    for ( uint8_t i = 0; i < sizeof(uint32_t); i++ )
    {
        result = (result << 8) | ((value>>(i*8)) & 0xff);
    }

    return result;
}

//*****************************************************************************
//
//! @brief
//! @param addr
//! @param buffer
//! @param length
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_read(uint32_t addr, uint8_t *buffer, unsigned long length)
{
    int ret;
    static uint32_t convertedAddr;

    convertedAddr = swap_endian(addr);

    if (buffer == NULL)
    {
        return -1;
    }

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = AM_DEVICES_CHSC5816_DEVICE_ADDR,
            .flags  = AM_DEVICES_CHSC5816_I2C_WR,
            .len    = sizeof(addr),
            .buf    = (uint8_t *)&convertedAddr,
        },
        {
            .addr   = AM_DEVICES_CHSC5816_DEVICE_ADDR,
            .flags  = AM_DEVICES_CHSC5816_I2C_RD,
            .len    = length,
            .buf    = buffer,
        },
    };

    ret = chsc5816_master_xfer(msgs, 2);

    if ( ret == (msgs[0].len - 1 + msgs[1].len) )
    {
        return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
    }

    return AM_DEVICES_CHSC5816_STATUS_ERROR;
}

//*****************************************************************************
//
//! @brief
//! @param addr
//! @param buffer
//! @param length
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_noblocking_read(uint32_t addr, uint8_t *buffer, unsigned long length, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt)
{
    int ret;
    static uint32_t convertedAddr;

    convertedAddr = swap_endian(addr);

    if (buffer == NULL)
    {
        return -1;
    }

    struct rt_i2c_msg msgs[] =
    {
        {
            .addr   = AM_DEVICES_CHSC5816_DEVICE_ADDR,
            .flags  = AM_DEVICES_CHSC5816_I2C_WR,
            .len    = sizeof(addr),
            .buf    = (uint8_t *)&convertedAddr,
        },
        {
            .addr   = AM_DEVICES_CHSC5816_DEVICE_ADDR,
            .flags  = AM_DEVICES_CHSC5816_I2C_RD,
            .len    = length,
            .buf    = buffer,
        },
    };

    ret = chsc5816_master_noblocking_xfer(msgs, 2, pfnCallback, pCallbackCtxt);

    if ( ret == (msgs[0].len - 1 + msgs[1].len) )
    {
        return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
    }

    return AM_DEVICES_CHSC5816_STATUS_ERROR;
}


//*****************************************************************************
//
//! @brief
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_read(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui8Priority     = 1;
    Transaction.ui32InstrLen    = 0;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    Transaction.ui32Instr       = 0;
#else
    Transaction.ui64Instr       = 0;
#endif
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    Transaction.uPeerInfo.ui32I2CDevAddr = ui32BusAddress;
    if (am_hal_iom_blocking_transfer(g_chsc5816_IOMHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_noblocking_read(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui8Priority     = 1;
    Transaction.ui32InstrLen    = 0;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    Transaction.ui32Instr       = 0;
#else
    Transaction.ui64Instr       = 0;
#endif
    Transaction.eDirection      = AM_HAL_IOM_RX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32RxBuffer   = pBuf;
    Transaction.bContinue       = false;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    Transaction.uPeerInfo.ui32I2CDevAddr = ui32BusAddress;
    if (am_hal_iom_nonblocking_transfer(g_chsc5816_IOMHandle, &Transaction, pfnCallback, pCallbackCtxt) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//! @return
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_write(uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;

    Transaction.ui8Priority     = 1;
    Transaction.ui32InstrLen    = 0;
#if defined(AM_PART_APOLLO3) || defined(AM_PART_APOLLO3P)
    Transaction.ui32Instr       = 0;
#else
    Transaction.ui64Instr       = 0;
#endif
    Transaction.eDirection      = AM_HAL_IOM_TX;
    Transaction.ui32NumBytes    = size;
    Transaction.pui32TxBuffer   = pBuf;
    Transaction.bContinue       = true;
    Transaction.ui8RepeatCount  = 0;
    Transaction.ui32PauseCondition = 0;
    Transaction.ui32StatusSetClr = 0;

    Transaction.uPeerInfo.ui32I2CDevAddr = ui32BusAddress;
    if (am_hal_iom_blocking_transfer(g_chsc5816_IOMHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param msgs
//! @param num
//! @return
//
//*****************************************************************************
static uint32_t
chsc5816_master_xfer(struct rt_i2c_msg *msgs, uint32_t num)
{
    struct rt_i2c_msg *msg;
    int i;
    rt_uint32_t msg_len = 0;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags == AM_DEVICES_CHSC5816_I2C_RD)
        {
            chsc5816_i2c_read(msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += msg->len;
        }
        else if ( msg->flags == AM_DEVICES_CHSC5816_I2C_WR )
        {
            chsc5816_i2c_write(msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += (msg->len - 1);
        }
    }

    return msg_len;
}

//*****************************************************************************
//
//! @brief
//! @param msgs
//! @param num
//! @return
//
//*****************************************************************************
static uint32_t
chsc5816_master_noblocking_xfer(struct rt_i2c_msg *msgs, uint32_t num, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt)
{
    struct rt_i2c_msg *msg;
    int i;
    rt_uint32_t msg_len = 0;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags == AM_DEVICES_CHSC5816_I2C_RD)
        {
            chsc5816_i2c_noblocking_read(msg->addr, (uint32_t *)msg->buf, msg->len, pfnCallback, pCallbackCtxt);
            msg_len += msg->len;
        }
        else if ( msg->flags == AM_DEVICES_CHSC5816_I2C_WR )
        {
            chsc5816_i2c_write(msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += (msg->len - 1);
        }
    }

    return msg_len;
}


static void pfnTMA_READINFO_Callback(void *pCallbackCtxt, uint32_t status)
{
    am_devices_tc_chsc5816_info_t *info = ((chsc5816_hihg_level_info_t*)(pCallbackCtxt))->info;
    uint8_t *buf = ((chsc5816_hihg_level_info_t*)(pCallbackCtxt))->data;

    union rpt_multi_point_t* mul_ppt;
    mul_ppt = (union rpt_multi_point_t*)&buf[2];

    uint16_t touch_x0;
    uint16_t touch_y0;
    uint16_t touch_x1;
    uint16_t touch_y1;
    int32_t pointed = 0;

    if ( (buf[0] == 0xff) && (buf[1] <= 2) )
     {
         //
         // read positions
         //
         if (buf[1] == 1)
         {
             touch_x0 = (unsigned int)(mul_ppt->rp[0].x_h4 << 8) | mul_ppt->rp[0].x_l8;
             touch_y0 = (unsigned int)(mul_ppt->rp[0].y_h4 << 8) | mul_ppt->rp[0].y_l8;

             if (touch_x0 != 1)
             {
                 info->x0 = touch_x0;
                 info->y0 = touch_y0;
             }
             info->finger_number = 1;
         }
         else if (buf[1] == 2)
         {
             touch_x0 = (unsigned int)(mul_ppt->rp[0].x_h4 << 8) | mul_ppt->rp[0].x_l8;
             touch_y0 = (unsigned int)(mul_ppt->rp[0].y_h4 << 8) | mul_ppt->rp[0].y_l8;

             touch_x1 = (unsigned int)(mul_ppt->rp[1].x_h4 << 8) | mul_ppt->rp[1].x_l8;
             touch_y1 = (unsigned int)(mul_ppt->rp[1].y_h4 << 8) | mul_ppt->rp[1].y_l8;

             if (touch_x0 != 1)
             {
                 info->x0 = touch_x0;
                 info->y0 = touch_y0;
                 info->x1 = touch_x1;
                 info->y1 = touch_y1;
             }
             info->finger_number = 2;
         }
         else
         {
             //
             // gesture code
             // not impletement on the current firmware v04
             //
         }

         pointed = 1;
     }

     if ( pointed )
     {
         if ( mul_ppt->rp[0].event == 8 )
         {
             //TP_move
         }
         else if ( mul_ppt->rp[0].event == 4 )
         {
             //TP_UP
             info->touch_released = true;
         }
         else
         {
             //TP_Down
             info->touch_released = false;
         }
     }
     else
     {
         //GUI_TOUCH_StoreState(-1, -1);
     }
}

// ****************************************************************************
//
//  Get the actual touch x coordinate and y coordinate
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_nonblocking_get_point(am_devices_tc_chsc5816_info_t *touch_info)
{
    static chsc5816_hihg_level_info_t touch_message;
    static uint8_t buf_info[12];

    touch_message.info = touch_info;
    touch_message.data = buf_info;

    chsc5816_noblocking_read(AM_DEVICES_CHSC5816_READ_DATA_BLOCK, buf_info, 12, pfnTMA_READINFO_Callback, &touch_message);
    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Get the actual touch x coordinate and y coordinate
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_get_point(am_devices_tc_chsc5816_info_t *touch_info)
{
    uint8_t buf[256];
    uint16_t touch_x0, touch_y0;
    uint16_t touch_x1, touch_y1;
    int32_t pointed = 0;

    chsc5816_read(0x2000002C, buf, 12);

    union rpt_multi_point_t* mul_ppt;
    mul_ppt = (union rpt_multi_point_t*)&buf[2];

    if ( (buf[0] == 0xff) && (buf[1] <= 2) )
    {
        //
        // read positions
        //
        if (buf[1] == 1)
        {
            touch_x0 = (unsigned int)(mul_ppt->rp[0].x_h4 << 8) | mul_ppt->rp[0].x_l8;
            touch_y0 = (unsigned int)(mul_ppt->rp[0].y_h4 << 8) | mul_ppt->rp[0].y_l8;

            if (touch_x0 != 1)
            {
                touch_info->x0 = touch_x0;
                touch_info->y0 = touch_y0;
            }
            touch_info->finger_number = 1;
        }
        else if (buf[1] == 2)
        {
            touch_x0 = (unsigned int)(mul_ppt->rp[0].x_h4 << 8) | mul_ppt->rp[0].x_l8;
            touch_y0 = (unsigned int)(mul_ppt->rp[0].y_h4 << 8) | mul_ppt->rp[0].y_l8;

            touch_x1 = (unsigned int)(mul_ppt->rp[1].x_h4 << 8) | mul_ppt->rp[1].x_l8;
            touch_y1 = (unsigned int)(mul_ppt->rp[1].y_h4 << 8) | mul_ppt->rp[1].y_l8;

            if (touch_x0 != 1)
            {
                touch_info->x0 = touch_x0;
                touch_info->y0 = touch_y0;
                touch_info->x1 = touch_x1;
                touch_info->y1 = touch_y1;
            }
            touch_info->finger_number = 2;
        }
        else
        {
            //
            // gesture code
            // not impletement on the current firmware v04
            //
        }

        pointed = 1;
    }

    if ( pointed )
    {
        if ( mul_ppt->rp[0].event == 8 )
        {
            //TP_move
        }
        else if ( mul_ppt->rp[0].event == 4 )
        {
            //TP_UP
            touch_info->touch_released = true;
        }
        else
        {
            //TP_Down
            touch_info->touch_released = false;
        }
    }
    else
    {
        //GUI_TOUCH_StoreState(-1, -1);
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}
//
// Take over the interrupt handler for whichever IOM we're using.
//
#define fram_iom_isr                                                          \
    am_iom_isr1(AM_BSP_TP_IOM_MODULE)
#define am_iom_isr1(n)                                                        \
    am_iom_isr(n)
#define am_iom_isr(n)                                                         \
    am_iomaster ## n ## _isr

//*****************************************************************************
//
// IOM ISRs.
//
//*****************************************************************************
//
//! Take over default ISR. (Queue mode service)
//
#if defined(__IAR_SYSTEMS_ICC__)
__weak
#else
__attribute__ ((weak))
#endif
void fram_iom_isr(void)
{
    uint32_t ui32Status;

    if (!am_hal_iom_interrupt_status_get(g_chsc5816_IOMHandle, true, &ui32Status))
    {
        if ( ui32Status )
        {
            am_hal_iom_interrupt_clear(g_chsc5816_IOMHandle, ui32Status);
            am_hal_iom_interrupt_service(g_chsc5816_IOMHandle, ui32Status);
        }
    }
}

// ****************************************************************************
//
//  Initialize the CHSC5816 driver for IOM I2C and DMA
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_init(uint32_t ui32Module, am_hal_gpio_handler_t touch_handler, void *pArg)
{
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(ui32Module, AM_HAL_IOM_I2C_MODE);

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    // HAL Success return is 0
    //
    if (am_hal_iom_initialize(ui32Module, &g_chsc5816_IOMHandle) ||
        am_hal_iom_power_ctrl(g_chsc5816_IOMHandle, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(g_chsc5816_IOMHandle, &g_sI2cNBConfig) ||
        am_hal_iom_enable(g_chsc5816_IOMHandle))
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    #define FRAM_IOM_IRQn           ((IRQn_Type)(IOMSTR0_IRQn + ui32Module))
    NVIC_ClearPendingIRQ(FRAM_IOM_IRQn);
    NVIC_EnableIRQ(FRAM_IOM_IRQn);

    am_hal_gpio_pinconfig(AM_BSP_GPIO_TOUCH_RST, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(AM_BSP_GPIO_TOUCH_RST);
    am_util_delay_ms(10);
    am_hal_gpio_output_set(AM_BSP_GPIO_TOUCH_RST);

    uint32_t IntNum = AM_BSP_GPIO_TOUCH_INT;
    am_hal_gpio_mask_t IntMask = AM_HAL_GPIO_MASK_DECLARE_ZERO;
    IntMask.U.Msk[GPIO_NUM2IDX(IntNum)] = GPIO_NUM2MSK(IntNum);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, &IntMask);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum, touch_handler, pArg);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
    NVIC_SetPriority(g_sInterrupts[TP_GPIO_IDX], 0x4);
    NVIC_EnableIRQ(g_sInterrupts[TP_GPIO_IDX]);

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Deinitialize the CHSC5816 driver for IOM I2C and DMA
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_deinit(uint32_t ui32Module)
{
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    am_bsp_iom_pins_disable(ui32Module, AM_HAL_IOM_I2C_MODE);

    if (am_hal_iom_uninitialize(g_chsc5816_IOMHandle) ||
        am_hal_iom_power_ctrl(g_chsc5816_IOMHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false) ||
        am_hal_iom_disable(g_chsc5816_IOMHandle))
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

