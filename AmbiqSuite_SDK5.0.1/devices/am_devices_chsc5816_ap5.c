//*****************************************************************************
//
//! @file am_devices_chsc5816_ap5.c
//!
//! @brief Functions for controlling the CHSC581 touch sensor device (Apollo5).
//!
//! @addtogroup devices_chsc5816_ap5 CHSC5816 - I2C Touch Driver (Apollo510)
//! @ingroup devices
//! @{
//!
//! Purpose: This module provides a hardware abstraction layer
//!          for the CHSC581 capacitive touch sensor device on Apollo5 platforms.
//!          It enables initialization, configuration, event detection, and power
//!          management for reliable touch input in embedded applications.
//!          The driver supports multi-channel touch detection, customizable
//!          sensitivity, interrupt/event handling, and integration with system
//!          power modes for optimal user interface performance.
//!
//! @section devices_chsc581_ap5_features Key Features
//!
//! 1. @b Multi-channel @b Touch: Detects multiple touch points.
//! 2. @b Event @b Handling: Supports touch, release, and gesture events.
//! 3. @b Sensitivity @b Configuration: Adjustable touch thresholds.
//! 4. @b Power @b Efficiency: Low-power operation and sleep mode support.
//! 5. @b Interrupt @b Support: Fast event notification via hardware interrupts.
//!
//! @section devices_chsc581_ap5_functionality Functionality
//!
//! - Initialize CHSC581 device
//! - Configure touch channels and sensitivity
//! - Detect and process touch events
//! - Manage power and sleep modes
//! - Support for event callbacks and interrupt handling
//!
//! @section devices_chsc581_ap5_usage Usage
//!
//! 1. Initialize device with am_devices_chsc581_ap5_init()
//! 2. Configure channels and sensitivity as needed
//! 3. Use am_devices_chsc581_ap5_poll() or interrupt handler for event detection
//! 4. Process touch/release/gesture events via provided APIs
//!
//! @section devices_chsc581_ap5_configuration Configuration
//!
//! - Set up channel mapping and sensitivity thresholds
//! - Configure interrupt/event callbacks
//! - Integrate with system power management
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
#include "am_devices_chsc5816_ap5.h"
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

typedef struct
{
    struct rt_i2c_msg       readinfo;
    am_hal_iom_callback_t   pfnCallback;
    void                    *ptxtCallback;
    void                    *pHandle;
}nbcb_para_info_t;


static uint32_t chsc5816_i2c_read(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size);
static uint32_t chsc5816_i2c_write(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size);
static uint32_t chsc5816_master_xfer(void *pDevHandle, struct rt_i2c_msg *msgs, uint32_t num);

static uint32_t chsc5816_i2c_nonblocking_read(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt);
static uint32_t chsc5816_i2c_nonblocking_write(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt);

static am_devices_iom_gpio_chsc5816_t gAmChsc5816[AM_DEVICES_CHSC5816_MAX_DEVICE_NUM] = {0};
nbcb_para_info_t gReadCBInfo = {0};

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
//! @param pDevHandle
//! @param addr
//! @param buffer
//! @param length
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_read(void *pDevHandle, uint32_t addr, uint8_t *buffer, unsigned long length)
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

    ret = chsc5816_master_xfer(pDevHandle, msgs, 2);

    if ( ret == (msgs[0].len - 1 + msgs[1].len) )
    {
        return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
    }

    return AM_DEVICES_CHSC5816_STATUS_ERROR;
}

//*****************************************************************************
//
//! @brief CHSC5816 nonblocking read callback function
//! @param pCallbackCtxt
//! @param status
//!
//! @return uint32_t
//
//*****************************************************************************
static void pfnCHSC_READ_Callback(void *pCallbackCtxt, uint32_t status)
{
    nbcb_para_info_t *cb_param = (nbcb_para_info_t*)pCallbackCtxt;

    chsc5816_i2c_nonblocking_read(cb_param->pHandle, cb_param->readinfo.addr, (uint32_t *)cb_param->readinfo.buf, cb_param->readinfo.len, cb_param->pfnCallback, cb_param->ptxtCallback);
}

//*****************************************************************************
//
//! @brief
//! @param pDevHandle
//! @param addr
//! @param buffer
//! @param length
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_nonblocking_read(void* pDevHandle, uint32_t addr, uint8_t *buffer, unsigned long length, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt)
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

    gReadCBInfo.readinfo = msgs[1];
    gReadCBInfo.pfnCallback = pfnCallback;
    gReadCBInfo.ptxtCallback = pCallbackCtxt;
    gReadCBInfo.pHandle = pDevHandle;

    if ( chsc5816_i2c_nonblocking_write(pDevHandle, msgs[0].addr, (uint32_t *)msgs[0].buf, msgs[0].len, pfnCHSC_READ_Callback, &gReadCBInfo) != AM_HAL_STATUS_SUCCESS )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}


//*****************************************************************************
//
//! @brief
//! @param pDevHandle
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_read(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;
    am_devices_iom_gpio_chsc5816_t *pChscIom = (am_devices_iom_gpio_chsc5816_t *)pDevHandle;

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
    if (am_hal_iom_blocking_transfer(pChscIom->pIomHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pDevHandle
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//! @param pfnCallback
//! @param pCallbackCtxt
//!
//! @return uint32_t
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_nonblocking_read(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt)
{
    am_hal_iom_transfer_t       Transaction;
    am_devices_iom_gpio_chsc5816_t *pChscIom = (am_devices_iom_gpio_chsc5816_t *)pDevHandle;

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
    if (am_hal_iom_nonblocking_transfer(pChscIom->pIomHandle, &Transaction, pfnCallback, pCallbackCtxt) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pDevHandle
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//! @return
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_write(void *pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size)
{
    am_hal_iom_transfer_t       Transaction;
    am_devices_iom_gpio_chsc5816_t *pChscIom = (am_devices_iom_gpio_chsc5816_t *)pDevHandle;
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
    if (am_hal_iom_blocking_transfer(pChscIom->pIomHandle, &Transaction) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pDevHandle
//! @param ui32BusAddress
//! @param pBuf
//! @param size
//! @param pfnCallback
//! @param pCallbackCtxt
//! @return
//
//*****************************************************************************
static uint32_t
chsc5816_i2c_nonblocking_write(void* pDevHandle, uint32_t ui32BusAddress, uint32_t *pBuf, uint32_t size, am_hal_iom_callback_t pfnCallback, void *pCallbackCtxt)
{
    am_hal_iom_transfer_t       Transaction;
    am_devices_iom_gpio_chsc5816_t *pChscIom = (am_devices_iom_gpio_chsc5816_t *)pDevHandle;

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
    if (am_hal_iom_nonblocking_transfer(pChscIom->pIomHandle, &Transaction, pfnCallback, pCallbackCtxt) != AM_HAL_STATUS_SUCCESS)
    {
         return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
//! @brief
//! @param pDevHandle
//! @param msgs
//! @param num
//! @return
//
//*****************************************************************************
static uint32_t
chsc5816_master_xfer(void *pDevHandle, struct rt_i2c_msg *msgs, uint32_t num)
{
    struct rt_i2c_msg *msg;
    int i;
    rt_uint32_t msg_len = 0;

    for (i = 0; i < num; i++)
    {
        msg = &msgs[i];

        if (msg->flags == AM_DEVICES_CHSC5816_I2C_RD)
        {
            chsc5816_i2c_read(pDevHandle, msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += msg->len;
        }
        else if ( msg->flags == AM_DEVICES_CHSC5816_I2C_WR )
        {
            chsc5816_i2c_write(pDevHandle, msg->addr, (uint32_t *)msg->buf, msg->len);
            msg_len += (msg->len - 1);
        }
    }

    return msg_len;
}

//*****************************************************************************
//
//! @brief Parse the data read from device to point info.
//! @param
//! @return
//
//*****************************************************************************
void chsc5816_pointinfo_parse(uint8_t *data_buf, am_devices_tc_chsc5816_info_t *touch_info)
{
    union rpt_multi_point_t* mul_ppt;
    mul_ppt = (union rpt_multi_point_t*)&data_buf[2];

    uint16_t touch_x0;
    uint16_t touch_y0;
    uint16_t touch_x1;
    uint16_t touch_y1;
    int32_t pointed = 0;

    if ( (data_buf[0] == 0xff) && (data_buf[1] <= 2) )
     {
         //
         // read positions
         //
         if (data_buf[1] == 1)
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
         else if (data_buf[1] == 2)
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
}

//*****************************************************************************
//
//! @brief The callback function of nonblocking get point.
//! @param
//! @return
//
//*****************************************************************************
static void pfnCHSC_GETPOINT_Callback(void *pCallbackCtxt, uint32_t status)
{
    chsc5816_high_level_info_t *cbInfo = (chsc5816_high_level_info_t*)(pCallbackCtxt);
    am_device_chsc_callback_t pfnCallback = (am_device_chsc_callback_t)cbInfo->pfnCallback;

    chsc5816_pointinfo_parse(cbInfo->data, cbInfo->info);
    pfnCallback(cbInfo->pCallbackCtxt);
}

// ****************************************************************************
//
//  Get the actual touch x coordinate and y coordinate
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_nonblocking_get_point(void* pDevHandle, am_devices_tc_chsc5816_info_t *touch_info, am_device_chsc_callback_t pfnCallback, void *pCallbackCtxt)
{
    static chsc5816_high_level_info_t touch_message;
    static uint8_t buf_info[AM_DEVICES_CHSC5816_REPORT_DATA_LENGTH];

    touch_message.info = touch_info;
    touch_message.data = buf_info;
    touch_message.pfnCallback = (void *)pfnCallback;
    touch_message.pCallbackCtxt = pCallbackCtxt;

    chsc5816_nonblocking_read(pDevHandle, AM_DEVICES_CHSC5816_READ_DATA_BLOCK, buf_info, AM_DEVICES_CHSC5816_REPORT_DATA_LENGTH, pfnCHSC_GETPOINT_Callback, &touch_message);
    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Get the actual touch x coordinate and y coordinate
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_get_point(void *pDevHandle, am_devices_tc_chsc5816_info_t *touch_info)
{
    uint8_t buf[256];

    chsc5816_read(pDevHandle, 0x2000002C, buf, AM_DEVICES_CHSC5816_REPORT_DATA_LENGTH);

    chsc5816_pointinfo_parse(buf, touch_info);
    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Initialize the CHSC5816 driver for IOM I2C and DMA
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_init(uint32_t ui32Module, am_devices_iom_chsc5816_config_t *pDevConfig, void **ppDevHandle, void **ppIomHandle, uint32_t ui32IntPin, uint32_t ui32RstPin, am_hal_gpio_handler_t touch_handler, void *pArg)
{
    void *pIomHandle;
    am_hal_iom_config_t stIOMCfg = {0};
    if ( ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    uint32_t      ui32Index = 0;
    // Allocate a vacant device handle
    for ( ui32Index = 0; ui32Index < AM_DEVICES_CHSC5816_MAX_DEVICE_NUM; ui32Index++ )
    {
        if ( gAmChsc5816[ui32Index].bOccupied == false )
        {
            break;
        }
    }

    if ( ui32Index == AM_DEVICES_CHSC5816_MAX_DEVICE_NUM )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }
    if ( (ui32Module > AM_REG_IOM_NUM_MODULES)  || (pDevConfig == NULL) )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    stIOMCfg.eInterfaceMode = AM_HAL_IOM_I2C_MODE;
    stIOMCfg.ui32ClockFreq  = pDevConfig->ui32ClockFreq;
    stIOMCfg.pNBTxnBuf      = pDevConfig->pNBTxnBuf;
    stIOMCfg.ui32NBTxnBufLength = pDevConfig->ui32NBTxnBufLength;

    //
    // Configure the IOM pins.
    //
    am_bsp_iom_pins_enable(ui32Module, stIOMCfg.eInterfaceMode);

    //
    // Initialize the IOM instance.
    // Enable power to the IOM instance.
    // Configure the IOM for Serial operation during initialization.
    // Enable the IOM.
    //
    if (am_hal_iom_initialize(ui32Module, &pIomHandle) ||
        am_hal_iom_power_ctrl(pIomHandle, AM_HAL_SYSCTRL_WAKE, false) ||
        am_hal_iom_configure(pIomHandle, &stIOMCfg) ||
        am_hal_iom_enable(pIomHandle))
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }
    else
    {
        am_util_delay_us(150);
        gAmChsc5816[ui32Index].bOccupied = true;
        gAmChsc5816[ui32Index].ui32Module = ui32Module;
        *ppIomHandle = gAmChsc5816[ui32Index].pIomHandle = pIomHandle;
        *ppDevHandle = (void *)&gAmChsc5816[ui32Index];
    }

    #define FRAM_IOM_IRQn           ((IRQn_Type)(IOMSTR0_IRQn + ui32Module))
    NVIC_ClearPendingIRQ(FRAM_IOM_IRQn);
    NVIC_EnableIRQ(FRAM_IOM_IRQn);

    am_hal_gpio_pinconfig(ui32RstPin, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(ui32RstPin);
    am_util_delay_ms(10);
    am_hal_gpio_output_set(ui32RstPin);
    gAmChsc5816[ui32Index].ui32RstPinNum = ui32RstPin;

    uint32_t IntNum = ui32IntPin;
    am_hal_gpio_mask_t IntMask = AM_HAL_GPIO_MASK_DECLARE_ZERO;
    IntMask.U.Msk[GPIO_NUM2IDX(IntNum)] = GPIO_NUM2MSK(IntNum);
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, &IntMask);
    am_hal_gpio_interrupt_register(AM_HAL_GPIO_INT_CHANNEL_0, IntNum, touch_handler, pArg);
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0,
                                  AM_HAL_GPIO_INT_CTRL_INDV_ENABLE,
                                  (void *)&IntNum);
    NVIC_SetPriority(g_sInterrupts[TP_GPIO_IDX], 0x4);
    NVIC_EnableIRQ(g_sInterrupts[TP_GPIO_IDX]);

    gAmChsc5816[ui32Index].ui32IntPinNum = ui32IntPin;

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

// ****************************************************************************
//
//  Deinitialize the CHSC5816 driver for IOM I2C and DMA
//
// ****************************************************************************
uint32_t
am_devices_chsc5816_deinit(void *pDveHandle)
{
    am_devices_iom_gpio_chsc5816_t *pChscIom = (am_devices_iom_gpio_chsc5816_t *)pDveHandle;
    if ( pChscIom->ui32Module > AM_REG_IOM_NUM_MODULES )
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    am_bsp_iom_pins_disable(pChscIom->ui32Module, AM_HAL_IOM_I2C_MODE);

    if (am_hal_iom_uninitialize(pChscIom->pIomHandle) ||
        am_hal_iom_power_ctrl(pChscIom->pIomHandle, AM_HAL_SYSCTRL_DEEPSLEEP, false) ||
        am_hal_iom_disable(pChscIom->pIomHandle))
    {
        return AM_DEVICES_CHSC5816_STATUS_ERROR;
    }

    // Free this device handle
    pChscIom->bOccupied = false;

    return AM_DEVICES_CHSC5816_STATUS_SUCCESS;
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

