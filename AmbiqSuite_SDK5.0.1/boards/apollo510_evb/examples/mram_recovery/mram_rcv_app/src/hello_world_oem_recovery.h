//*****************************************************************************
//
// hello_world_oem_recovery.h
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

#ifndef OEM_RECOVERY_H
#define OEM_RECOVERY_H

#include <stdint.h>
#include <string.h>
#include "am_mcu_apollo.h"

#define am_print_uart_terminal(...) \
do { \
        am_util_stdio_textmode_set(true);   \
        am_util_stdio_printf_init(uart_print);  \
        am_util_stdio_printf(__VA_ARGS__);    \
        am_util_stdio_textmode_set(false);   \
        am_util_stdio_printf_init(am_hal_itm_print);    \
} while(0);

#define am_print_swo_terminal(...) \
do { \
        am_util_stdio_printf_init(am_hal_itm_print);  \
        am_util_stdio_printf(__VA_ARGS__);    \
} while(0);

#define am_print_swo_uart_terminal(...) \
do { \
        am_util_stdio_textmode_set(true);   \
        am_util_stdio_printf_init(uart_print);  \
        am_util_stdio_printf(__VA_ARGS__);    \
        am_util_stdio_textmode_set(false);      \
        am_util_stdio_printf_init(am_hal_itm_print);    \
        am_util_stdio_printf(__VA_ARGS__);    \
} while(0);

#define CHECK_ERRORS(x)                                                   \
if ((x) != AM_HAL_STATUS_SUCCESS)                                         \
{                                                                         \
    error_handler(x);                                                     \
}

extern const uint32_t* const __pPatchable;
extern const VECTOR_TABLE_Type __VECTOR_TABLE[256];

#define AM_APPLICATION_SIZE                     (16 * 1024)
#define INVALID_PIN                             0xFF
#define CRC_SIGN1_STORE_ADDR                    (0x7F0000)
#define CRC_SIGN2_STORE_ADDR                    (0x7F0004)
#define CRC_STORE_ADDR                          (0x7F0008)
#define AM_MRAM_RECOVERY_MASK                   (0xFFFF8000)
#define AM_OEM_MRAM_RECOVERY_MASK               (0x20000000)
#define AM_AMBIQ_MRAM_RECOVERY_MASK             (0x80000000)
#define CRC_PATCH_WORDS_NUM                     (3)

#define AM_INFO0_SEL(bOtp)    \
    ((bOtp) ? AM_HAL_INFO_INFOSPACE_OTP_INFO0 : AM_HAL_INFO_INFOSPACE_CURRENT_INFO0) \

#define AM_OTP_INFO1_MRAM_RCVY_CNT_O            0x200
#define AM_OTP_INFO0_MRAM_RCVY_CNT_O            0x29
#define MAX_UART_ACTIVITY_COUNT                 10
#define ARM_VTOR_ADDRESS                        (0xE000ED08)

//
// Define for date and time signature
//
// __DATE__ returns a string as "Jan 14 2012"
#define MONTH  (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) \
              : __DATE__ [2] == 'b' ? 2 \
              : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) \
              : __DATE__ [2] == 'y' ? 5 \
              : __DATE__ [2] == 'l' ? 7 \
              : __DATE__ [2] == 'g' ? 8 \
              : __DATE__ [2] == 'p' ? 9 \
              : __DATE__ [2] == 't' ? 10 \
              : __DATE__ [2] == 'v' ? 12 : 12)

#define YEAR ((__DATE__ [7] - '0') * 1000 + \
              (__DATE__ [8] - '0') * 100  + \
              (__DATE__ [9] - '0') * 10   + \
              (__DATE__ [10] - '0') )

#define DAY ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 + (__DATE__ [5] - '0'))

#define DATE_AS_INT ((YEAR << 16) + (MONTH << 8) + DAY )

// __time__ retuns "hh:mm:ss" - convert to int in seconds
#define TIME_AS_INT ( ((__TIME__ [0] - '0') * 10  + (__TIME__ [1] - '0')) + \
                     (((__TIME__ [3] - '0') * 10  + (__TIME__ [4] - '0')) * 60) + \
                     (((__TIME__ [6] - '0') * 10  + (__TIME__ [7] - '0')) * 3600) )

extern int32_t g_UARTActCount;
//**************************************
//
//! MRAM Recovery Status
//
//**************************************
typedef enum
{
    AM_MRAM_RECOVERY_STATUS_RCVYINPROGRESS      = (0x8000), /*! <RCVYINPROGRESS (Bit 15)*/
    AM_MRAM_RECOVERY_STATUS_RCVYUARTLOAD        = (0x10000), /*! <RCVYUARTLOAD (Bit 16)*/
    AM_MRAM_RECOVERY_STATUS_RCVYSPILOAD         = (0x20000), /*! <RCVYSPILOAD (Bit 17)*/
    AM_MRAM_RECOVERY_STATUS_RCVYINITIATED       = (0xC0000), /*! <RCVYINITIATED (Bit 18)*/
    AM_MRAM_RECOVERY_STATUS_SBLWIREDOTA         = (0x100000), /*! <RCVYUARTDONE (Bit 20)*/
    AM_MRAM_RECOVERY_STATUS_SBLWIREDLOAD        = (0x200000), /*! <RCVYUARTDONE (Bit 21)*/
    AM_MRAM_RECOVERY_STATUS_SBLNVOTA            = (0x400000), /*! <RCVYUARTDONE (Bit 22)*/
    AM_MRAM_RECOVERY_STATUS_SBLNVIMAGELOAD      = (0x800000), /*! <RCVYUARTDONE (Bit 23)*/
    AM_MRAM_RECOVERY_STATUS_OEMWIREDOTA         = (0x1000000), /*! <RCVYUARTDONE (Bit 24)*/
    AM_MRAM_RECOVERY_STATUS_OEMWIREDLOAD        = (0x2000000), /*! <RCVYUARTDONE (Bit 25)*/
    AM_MRAM_RECOVERY_STATUS_OEMNVOTA            = (0x4000000), /*! <RCVYUARTDONE (Bit 26)*/
    AM_MRAM_RECOVERY_STATUS_OEMNVIMAGELOAD      = (0x8000000), /*! <RCVYUARTDONE (Bit 27)*/
    AM_MRAM_RECOVERY_STATUS_OEMRCVYFAIL         = (0x10000000), /*! <RCVYUARTDONE (Bit 28)*/
    AM_MRAM_RECOVERY_STATUS_OEMRCVYSUCCESS      = (0x20000000), /*! <RCVYUARTDONE (Bit 29)*/
    AM_MRAM_RECOVERY_STATUS_SBLRCVYFAIL         = (0x40000000), /*! <RCVYUARTDONE (Bit 30)*/
    AM_MRAM_RECOVERY_STATUS_SBLRCVYSUCCESS      = (0x80000000), /*! <RCVYUARTDONE (Bit 31)*/
}am_mram_recovery_status_e;

//**************************************
//
//! MRAM Recovery Status Structure
//
//**************************************
typedef struct
{
    am_mram_recovery_status_e eStatus;    // Returns all MRAM Recovery Status
    bool bRecoveryInProgress;             // Returns if Recovery is in Progress
    bool bRecoveryUARTLoad;               // Returns if Recovery UART Load is in Progress
    bool bRecoverySPILoad;                // Returns if Recovery SPI Load is in Progress
    uint32_t bRecoveryInitiated;          // Returns if Recovery is Initiated
    bool bSBLWiredOTA;                    // Returns if SBL Wired OTA is in Progress
    bool bSBLWiredLoad;                    // Returns if SBL Wired Load is in Progress
    bool bSBLNVOTA;                       // Returns if SBL NV OTA is in Progress
    bool bSBLNVImageLoad;                 // Returns if SBL NV Image Load is in Progress
    bool bOEMWiredOTA;                    // Returns if OEM Wired OTA is in Progress
    bool bOEMWiredLoad;                    // Returns if OEM Wired Load is in Progress
    bool bOEMNVOTA;                       // Returns if OEM NV OTA is in Progress
    bool bOEMNVImageLoad;                 // Returns if OEM NV Image Load is in Progress
    bool bOEMRecoveryFail;                // Returns if OEM Recovery Failed
    bool bOEMRecoverySuccess;             // Returns if OEM Recovery is Successful
    bool bSBLRecoveryFail;                // Returns if SBL Recovery Failed
    bool bSBLRecoverySuccess;             // Returns if SBL Recovery is Successful
}am_mram_recovery_status_t;

//**************************************
//
//! MRAM Recovery Configuration Structure
//
//**************************************
typedef struct
{
    uint32_t    appRcvyEnable       : 1;    // application iniated recover enable (1 = Enabled)
    uint32_t    wiredRcvyEnable     : 1;    // wired recovery enable (using wired config setting) (1 = Enabled)
    uint32_t    nvModuleNum         : 2;    // nv interface module 0-3 (MSPI will always use DEV0 of the module)
    uint32_t    nvRcvyType          : 3;    // Designate MSPI or eMMC non volatile Recovery media type:
                                            // MRAM_RECOVER_PINCFG_MEDIA_NONE, MRAM_RECOVER_PINCFG_MEDIA_MSPI, MRAM_RECOVER_PINCFG_MEDIA_EMMC
    uint32_t    rebootOnAppRecFail  : 1;    // What to do if app initiated recovery fails
    uint32_t    pinNumGpioRcvy      : 8;    // pin number for GPIO Forced Recovery
    uint32_t    gpioRcvyPol         : 1;    // polarity of GPIO Forced recovery pin
    uint32_t    enWDT               : 1;    // WatchDog Timer enable
    uint32_t    eMMCpartition       : 2;    // eMMC partition. 0=user, 1=boot1, 2=boot2
    uint32_t    gpioRcvyInprog      : 8;    // MRAM recover in progress output pin (0xFF = None, disabled)
    uint32_t    mstrRcvyEnable      : 4;    // master MRAM recovery enable 0110b = Enabled, Others = disabled

    //
    // Device Offset to recovery images (metadata)
    //
    uint32_t    metadataAddr        : 32;   // Address of recovery metadata

    //
    // reset and Pwr-on delays
    //
    uint32_t    pwrDelay            : 12;   // power-on Delay after setting setting pin to specified ON polarity
    uint32_t    pwrDelayUnits       : 1;    // power-on reset delay units 1=mS, 0=uS
    uint32_t    pwrPol              : 1;    // power-on polarity: 0=set low to power on the device, 1=set high to power on the device
    uint32_t    rsvd3               : 2;    // unused
    uint32_t    resetDelay          : 12;   // reset delaym delay after reset, 0 = No Reset pin activity (set to inactive)
    uint32_t    resetDelayUnits     : 1;    // reset delay units 1=mS, 0=uS
    uint32_t    resetPol            : 1;    // reset polarity: 0=reset is active low, 1=reset is active high
    uint32_t    resetJEDEC          : 1;    // use JEDEC Standard GPIO-reset to reset device (MSPI)
    uint32_t    rsvd4               : 1;    // unused

    //
    // Pin Number reset, Pwr_ctl (0xFF = unused)
    //
    uint32_t    pinNumPwrCtl        : 8;    // pin number for power to the device
    uint32_t    pinNumReset         : 8;    // devices reset pin number
    uint32_t    pinNumCE            : 8;    // MSPI CE pin number
    uint32_t    rsvd5               : 8;    // unused

    //
    // Pin configs CE, SCLK, Data, DQS
    //
    uint32_t    pinCfgCE            : 32;   // Pin Config for MSPI CE pin or eMMC CMD pin
    uint32_t    pinCfgSCK           : 32;   // Pin Config for SCK PIN
    uint32_t    pinCfgData          : 32;   // Pin Config for data pins (Data 0-7)
    uint32_t    pinCfgDQS           : 32;   // Pin Config for DQS pin

    //
    // MSPI device config - maps directly to the config registers
    //
    uint32_t    nvConfig0           : 32;   // MSPI - directly maps to module's CFG reg, EMMMC's -- target clock speed
    uint32_t    nvConfig1           : 32;   // MSPI - directly maps to module's CFG1 reg, EMMC's -- UHS mode
    uint32_t    nvConfig2           : 32;   // MSPI - Directly mapps to module's DDR confg reg
    uint32_t    nvConfig3           : 32;   // MSPI - directly maps to modules SCRAMBLING reg

    //
    // MSPI/eMMC specific options
    //
    uint32_t    mspiReadCmd         : 8;    // MSPI read cmd - size specified in dev_cfg (use [8:0] if 8-bit)
    uint32_t    mspiPreCmdsCtrl     : 4;    // MSPI Pre-commands control
    uint32_t    mspiPreCmdsClk      : 4;    // Clock for the Pre-Commands
    uint32_t    mspiWidths          : 4;    // MSPI data cmd-data-widths loaded to CTRL1.PIOMIXED (defines 1-x-x modes)
    uint32_t    mspiClkSel          : 4;    // HFRC clock divisor (CLKCTL for MSPI)
    uint32_t    mspiD4Clk           : 1;    // MSPI clock on D4
    uint32_t    mspiPadSet1         : 1;    // MSPI PADOUTEN.PADSET1 option
    uint32_t    emmcVoltage         : 2;    // EMMC I/O voltage to be used
    uint32_t    emmcDataWidth       : 3;    // eMMC data width 2^^n: 0=1, 2=4, 3=8
    uint32_t    rsvd7               : 1;    // unused

    //
    // MSPI Pre-commands  (LSB -> MSB)
    //
    uint32_t   preCmd4              : 8;    // Pre command 4
    uint32_t   preCmd3              : 8;    // Pre command 3
    uint32_t   preCmd2              : 8;    // Pre command 2
    uint32_t   preCmd1              : 8;    // Pre command 1

    //
    // Number of retries for recovery and delay in between
    //
    uint32_t   maxRetryDelayMin     : 8;
    uint32_t   minRetryDelaySec     : 8;
    uint32_t   maxTries             : 16;
} am_mram_recovery_cfg_t;

//**************************************
//
//! External Global Variables
//
//**************************************
extern volatile uint32_t g_ui32CRCCnt;
extern volatile uint32_t g_ui32WDTCnt;
extern volatile uint32_t g_ui32CRCFlag;
extern volatile uint32_t g_ui32WDTFlag;
extern bool g_bCRCFail;
extern volatile uint32_t g_bUartRxDone;
extern volatile bool g_btriggerWdtRst;
extern void* phUART;
extern volatile uint32_t ui32LastError;

//**************************************
//
//! Function Prototypes
//
//**************************************
extern bool am_crc_check_app(bool bStoreCRC);
extern void uart_print(char *pcStr);
extern void am_print_mram_recovery_message(bool bReset);
extern void am_update_mram_rcvy_count(void);
extern uint32_t am_mram_rcvy_count_get(bool bAmbiq);
extern void UART_printf_init(void);
extern void am_watchdog_init(void);
extern void am_stimer_init(void);
extern void error_handler(uint32_t ui32ErrorStatus);
extern bool crc2_check(void);
extern void am_uart_activity(void);

#endif // OEM_RECOVERY_H
