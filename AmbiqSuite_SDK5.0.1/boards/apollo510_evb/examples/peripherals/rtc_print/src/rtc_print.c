//*****************************************************************************
//
//! @file rtc_print.c
//!
//! @brief Example using the internal RTC.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup rtc_print Real Time Clock Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates Real-Time Clock (RTC)
//! functionality for timekeeping and deep sleep wake-up operations. The
//! application showcases RTC configuration, time management, and periodic
//! wake-up from deep sleep modes. The example implements sophisticated
//! time tracking with date/time formatting, alarm functionality, and
//! power management for battery-powered applications requiring accurate
//! timekeeping during extended sleep periods.
//!
//! @section rtc_print_features Key Features
//!
//! 1. @b RTC @b Configuration: Demonstrates RTC setup and
//!    time management for accurate timekeeping operations
//!
//! 2. @b Deep @b Sleep @b Wake @b Up: Implements periodic wake-up from
//!    deep sleep modes for power-efficient timekeeping
//!
//! 3. @b Time @b Formatting: Provides date/time formatting
//!    with weekday and month name support for human-readable output
//!
//! 4. @b Alarm @b Functionality: Implements RTC alarm capabilities for
//!    scheduled wake-up and event triggering
//!
//! 5. @b Power @b Management: Demonstrates power-efficient operation with
//!    deep sleep modes and periodic wake-up for time updates
//!
//! @section rtc_print_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes RTC with proper configuration and time settings
//! - Implements periodic timer interrupt for deep sleep wake-up
//! - Provides date/time formatting and display
//! - Implements RTC alarm functionality for scheduled events
//! - Demonstrates power management with deep sleep modes
//! - Monitors sleep states and provides GPIO status output
//!
//! @section rtc_print_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor SWO output for current time and date information
//! 3. Observe periodic wake-up from deep sleep modes
//! 4. Verify RTC timekeeping accuracy and alarm functionality
//!
//! @section rtc_print_configuration Configuration
//!
//! - @b PNTNUMSECS: Print interval in seconds (default: 5 seconds)
//! - @b SWO_PRINTF: Enable SWO output for time display
//! - @b GPIO_SLEEPSTATE: Enable GPIO output for sleep state monitoring
//! - @b PNT_SLEEPSTATE: Enable sleep state printing
//!
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
#include "am_bsp.h"
#include "am_util.h"

//
// Configuration options
//
#define SWO_PRINTF          // Defined for SWO, undefined for UART output
#define PNTNUMSECS  5       // Print the current time every 5 seconds
#define PNT_SLEEPSTATE      // Print sleep state
#define GPIO_SLEEPSTATE     // Use GPIOs to output the sleep state


//
// Define GPIO pins for outputting sleep state
// Note: Select GPIOs that remain powered and wake-capable during deeper sleep.
//
#ifdef GPIO_SLEEPSTATE
#define GPIO_CORESLEEP          11
#define GPIO_COREDEEPSLEEP      12
#define GPIO_SYSDEEPSLEEP       13
#define GPIO_SYSDEEPERSLEEP     14
#endif // GPIO_SLEEPSTATE

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
uint8_t g_ui8IsrCount  = 0;
bool g_bAlarmOccurred  = false;
bool g_bReadyToPrint   = false;

//
// String arrays to index Days and Months with the values returned by the RTC.
//
char *g_pcWeekday[] =
{
    "Sun",
    "Mon",
    "Tue",
    "Wed",
    "Thu",
    "Fri",
    "Sat",
    "Invalid day (7)"
};

//
// The hardware treats the month as 1-12 indexed so the first entry in the month is an invalid one.
//
//
char *g_pcMonth[] =
{
    "Invalid month (0)",
    "Jan",
    "Feb",
    "Mar",
    "Apr",
    "May",
    "Jun",
    "Jul",
    "Aug",
    "Sep",
    "Oct",
    "Nov",
    "Dec",
    "Invalid month (13)"
};

//*****************************************************************************
//
// RTC Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_rtc_isr(void)
{
    am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);

    g_bAlarmOccurred = true;
    if ( ++g_ui8IsrCount == PNTNUMSECS )
    {
        g_bReadyToPrint = true;
        g_ui8IsrCount = 0;
    }

} // am_rtc_isr()

//*****************************************************************************
//
// Support function:
// toVal() converts a string to an ASCII value.
//
//*****************************************************************************
int
toVal(char *pcAsciiStr)
{
    int iRetVal = 0;
    iRetVal += pcAsciiStr[1] - '0';
    iRetVal += pcAsciiStr[0] == ' ' ? 0 : (pcAsciiStr[0] - '0') * 10;
    return iRetVal;
} // toVal()

//*****************************************************************************
//
// Support function:
// mthToIndex() converts a string indicating a month to an index value.
// The return value is a value 0-12, with 0-11 indicating the month given
// by the string, and 12 indicating that the string is not a month.
//
//*****************************************************************************
int
mthToIndex(char *pcMon)
{
    int idx;
    for (idx = 0; idx < 12; idx++)
    {
        if ( am_util_string_strnicmp(g_pcMonth[idx], pcMon, 3) == 0 )
        {
            return idx;
        }
    }
    return 12;
} // mthToIndex()

//*****************************************************************************
//
// Support function:
// Enable or disable printing, whether SWO or UART.
//
//*****************************************************************************
static void
rtcprint_cfg(bool bEnableForPrinting)
{
#ifdef SWO_PRINTF
    if ( bEnableForPrinting )
    {
        //
        // Enable debug printf messages using ITM on SWO pin
        //
        am_bsp_itm_printf_enable();
    }
    else
    {
        //
        // We are done printing. Disable debug printf messages on ITM.
        //
        am_bsp_itm_printf_disable();
    }
#else // !SWO_PRINTF
    if ( bEnableForPrinting )
    {
        //
        // Enable debug printf messages using UART
        //
        am_bsp_uart_printf_enable();
    }
    else
    {
        //
        // We are done printing. Disable debug printf messages on UART.
        //
        am_bsp_uart_printf_disable();
    }
#endif // SWO_PRINTF

} // rtcprint_cfg()

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    am_hal_rtc_time_t hal_time;
    uint16_t ui16year;
#if defined(PNT_SLEEPSTATE) || defined(GPIO_SLEEPSTATE)
    uint32_t ui32SleepState;
#endif

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#ifdef AM_PART_APOLLO330P_510L
    //
    // Power off the RSS so that deepersleep can be attained.
    //
    am_hal_pwrctrl_rss_pwroff();
#endif // AM_PART_APOLLO330P_510L

    //
    // Enable the XT for the RTC.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL, 0);

    //
    // Select XT for RTC clock source
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);

    //
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();

#ifdef GPIO_SLEEPSTATE
    am_hal_gpio_output_clear(GPIO_CORESLEEP);
    am_hal_gpio_output_clear(GPIO_COREDEEPSLEEP);
    am_hal_gpio_output_clear(GPIO_SYSDEEPSLEEP);
    am_hal_gpio_pinconfig(GPIO_CORESLEEP,      am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(GPIO_COREDEEPSLEEP,  am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(GPIO_SYSDEEPSLEEP,   am_hal_gpio_pincfg_output);
#ifdef AM_PART_APOLLO330P_510L
    am_hal_gpio_output_clear(GPIO_SYSDEEPERSLEEP);
    am_hal_gpio_pinconfig(GPIO_SYSDEEPERSLEEP, am_hal_gpio_pincfg_output);
#endif // AM_PART_APOLLO330P_510L
#endif // GPIO_SLEEPSTATE

    //
    // Set the RTC time for this example.
    // WARNING this will destroy any time epoch currently in the RTC.
    //
#if defined(__GNUC__)  ||  defined(__ARMCC_VERSION)  ||  defined(__IAR_SYSTEMS_ICC__)
    //
    // The RTC is initialized from the date and time strings that are
    // obtained from the compiler at compile time.
    //
    hal_time.ui32Hour = toVal(&__TIME__[0]);
    hal_time.ui32Minute = toVal(&__TIME__[3]);
    hal_time.ui32Second = toVal(&__TIME__[6]);
    hal_time.ui32Hundredths = 0;
    hal_time.ui32Weekday = am_util_time_computeDayofWeek(2000 + toVal(&__DATE__[9]), mthToIndex(&__DATE__[0]), toVal(&__DATE__[4]) );
    hal_time.ui32DayOfMonth = toVal(&__DATE__[4]);
    hal_time.ui32Month = mthToIndex(&__DATE__[0]);
    hal_time.ui32Year = toVal(&__DATE__[9]);
#else
    //
    // The RTC is initialized from 2/28/2000
    //
    hal_time.ui32Hour = 23;
    hal_time.ui32Minute = 59;
    hal_time.ui32Second = 55;
    hal_time.ui32Hundredths = 0;
    hal_time.ui32Weekday = 2;
    hal_time.ui32DayOfMonth = 28;
    hal_time.ui32Month = 2;
    hal_time.ui32Year = 0;
#endif

    //
    // Handle the century bit is for tracking 2000s - 2100s
    //
    // 2000s  -> ui32CenturyBit == 1
    // 2100s  -> ui32CenturyBit == 0
    //
    hal_time.ui32CenturyBit = RTC_CTRUP_CB_2000;

    am_hal_rtc_alarm_interval_set(AM_HAL_RTC_ALM_RPT_SEC);

    //
    // Check the validity of the input values
    //
    if ( am_hal_rtc_time_set(&hal_time) )
    {
        am_util_stdio_printf("Invalid Input Value");
        return 1;
    }

    //
    // Print initial date/time information
    //
    rtcprint_cfg(true);

    //
    // Print RTC time.
    //
    am_hal_rtc_time_get(&hal_time);

    //
    // Print banner and introduction
    //
    am_util_stdio_printf("rtc_print example!\n\n");
    am_util_stdio_printf("The initial RTC values reflect the compilation date/time.\n");
    am_util_stdio_printf("The alarm interval is set to wake every 1 second, while actual\n"
                         " printing of RTC data occurs every %d seconds.\n", PNTNUMSECS);
    am_util_stdio_printf("Please be aware that the time delta between the initial and the next\n"
                         " read is affected by the application's startup overhead.\n\n");

#ifdef PNT_SLEEPSTATE
#ifdef AM_PART_APOLLO330P_510L
    am_util_stdio_printf("[dcns]: d=SYSDEEPSLEEP, c=COREDEEPSLEEP, n=CORESLEEP, s=SYSDEEPERSLEEP\n\n[dcns] ");
#else // AM_PART_APOLLO510
    am_util_stdio_printf("[dcn]: d=SYSDEEPSLEEP, c=COREDEEPSLEEP, n=CORESLEEP\n\n[dcn] ");
#endif
#endif // PNT_SLEEPSTATE

    ui16year = ( hal_time.ui32CenturyBit == RTC_CTRUP_CB_2000 ) ? 2000 : 2100;

    am_util_stdio_printf("Initial RTC values: %s %s %02d, %04d  ",
                         g_pcWeekday[hal_time.ui32Weekday], g_pcMonth[hal_time.ui32Month],
                         hal_time.ui32DayOfMonth, ui16year + hal_time.ui32Year);
    am_util_stdio_printf("%02d:%02d:%02d.%02d, CenturyBit=%d\n",
                         hal_time.ui32Hour,    hal_time.ui32Minute,
                         hal_time.ui32Second,  hal_time.ui32Hundredths,
                         hal_time.ui32CenturyBit);

    rtcprint_cfg(false);

    //
    // Enable the RTC interrupt in the NVIC.
    //
    NVIC_SetPriority(RTC_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(RTC_IRQn);
    am_hal_rtc_interrupt_enable(AM_HAL_RTC_INT_ALM);
    am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
    am_hal_interrupt_master_enable();

    //
    // Loop forever, printing current info when awakened by the RTC.
    //
    while (1)
    {
        //
        // Go to sleep and wait for the RTC alarm.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEPMAX);

#if defined(PNT_SLEEPSTATE) || defined(GPIO_SLEEPSTATE)
    //
    // Save, then clear the sleep status bits
    //
    ui32SleepState = PWRCTRL->SYSPWRSTATUS;
    PWRCTRL->SYSPWRSTATUS = PWRCTRL_SYSPWRSTATUS_SLEEP_Msk;
#endif

    if ( g_bReadyToPrint )
    {
        g_bReadyToPrint = false;

        rtcprint_cfg(true);

#ifdef PNT_SLEEPSTATE
#ifdef AM_PART_APOLLO330P_510L
        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SLEEP_Msk )
        {
            am_util_stdio_printf(" [%d%d%d%d] ",
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_SYSDEEPSLEEP,   ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP,  ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_CORESLEEP,      ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_SYSDEEPERSLEEP, ui32SleepState));
        }
#else   // AM_PART_APOLLO510
        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SLEEP_Msk )
        {
            am_util_stdio_printf("[%d%d%d] ",
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_SYSDEEPSLEEP,  ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP, ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_CORESLEEP,     ui32SleepState));
        }
#endif
        else
        {
            am_util_stdio_printf("[No sleep detected] ");
        }
#endif // PNT_SLEEPSTATE

        //
        // Print RTC time.
        //
        am_hal_rtc_time_get(&hal_time);

        am_util_stdio_printf("Current RTC values: %s %s %02d, %04d  ",
                             g_pcWeekday[hal_time.ui32Weekday], g_pcMonth[hal_time.ui32Month],
                             hal_time.ui32DayOfMonth, ui16year + hal_time.ui32Year);
        am_util_stdio_printf("%02d:%02d:%02d.%02d, CenturyBit=%d\n",
                             hal_time.ui32Hour,    hal_time.ui32Minute,
                             hal_time.ui32Second,  hal_time.ui32Hundredths,
                             hal_time.ui32CenturyBit);

        if ( g_bAlarmOccurred == true )
        {
            g_bAlarmOccurred = false;
            am_util_debug_printf("***** ALARM - ALARM - ALARM ***** \n");
        }

        rtcprint_cfg(false);
    }

#ifdef GPIO_SLEEPSTATE
    if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SLEEP_Msk )
    {
        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_CORESLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_CORESLEEP,     AM_HAL_GPIO_OUTPUT_SET);
        }

        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_COREDEEPSLEEP, AM_HAL_GPIO_OUTPUT_SET);
        }

        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SYSDEEPSLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_SYSDEEPSLEEP,  AM_HAL_GPIO_OUTPUT_SET);
        }
#ifdef AM_PART_APOLLO330P_510L
        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SYSDEEPERSLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_SYSDEEPERSLEEP, AM_HAL_GPIO_OUTPUT_SET);
        }
#endif // AM_PART_APOLLO330P_510L

        //
        // Short delay before toggling the signals
        //
        am_hal_delay_us(5);

        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_CORESLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_CORESLEEP,     AM_HAL_GPIO_OUTPUT_CLEAR);
        }

        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_COREDEEPSLEEP, AM_HAL_GPIO_OUTPUT_CLEAR);
        }

        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SYSDEEPSLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_SYSDEEPSLEEP,  AM_HAL_GPIO_OUTPUT_CLEAR);
        }
#ifdef AM_PART_APOLLO330P_510L
        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_SYSDEEPERSLEEP_Msk )
        {
            am_hal_gpio_state_write(GPIO_SYSDEEPERSLEEP, AM_HAL_GPIO_OUTPUT_CLEAR);
        }
#endif // AM_PART_APOLLO330P_510L
    }

#endif // GPIO_SLEEPSTATE
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

