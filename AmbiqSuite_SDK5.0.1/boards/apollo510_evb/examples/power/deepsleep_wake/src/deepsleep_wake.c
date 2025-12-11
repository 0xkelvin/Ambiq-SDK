//*****************************************************************************
//
//! @file deepsleep_wake.c
//!
//! @brief Example that goes to deepsleep and wakes from either the RTC or GPIO.
//!
//! @addtogroup power_examples Power Examples
//!
//! @defgroup deepsleep_wake Deepsleep Wake Example
//! @ingroup power_examples
//! @{
//!
//! Purpose: This example demonstrates deep sleep and wake-up
//! functionality for power optimization. The application
//! showcases power management with RTC and GPIO wake-up sources,
//! configurable memory retention, and power optimization.
//!
//! @section deepsleep_wake_features Key Features
//!
//! 1. @b Deep @b Sleep @b Management: Implements deep sleep
//!    functionality with configurable memory retention options
//!
//! 2. @b Multiple @b Wake @b Sources: Supports RTC and GPIO wake-up sources
//!    for flexible power management scenarios
//!
//! 3. @b Power @b Optimization: Provides low power operation with
//!    LFRC clock source for minimal power consumption
//!
//! 4. @b LED @b Status @b Indication: Uses LED toggling for wake-up event
//!    indication and power management monitoring
//!
//! 5. @b Configurable @b Memory @b Retention: Supports different memory
//!    retention configurations for power optimization
//!
//! @section deepsleep_wake_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes deep sleep with configurable memory retention
//! - Implements RTC wake-up every second with LED indication
//! - Provides GPIO wake-up on button press or pin toggle
//! - Manages power optimization with LFRC clock source
//! - Supports power management features
//! - Implements sleep and wake-up functionality
//!
//! @section deepsleep_wake_usage Usage
//!
//! 1. Compile and download the application to target device
//! 2. Monitor UART output at 115,200 BAUD for status messages
//! 3. Observe LED toggling for RTC and GPIO wake-up events
//! 4. Test button press or GPIO toggle for wake-up functionality
//! 5. Measure power consumption during deep sleep operation
//!
//! @section deepsleep_wake_configuration Configuration
//!
//! - @b ALL_RETAIN: Memory retention configuration (0 for min, 1 for all)
//! - @b WAKEUP_GPIO_PIN: GPIO pin for wake-up indication
//! - @b INTERRUPT_GPIO: GPIO pin for wake-up source
//! - @b LFRC @b Clock: Low-frequency RC clock for minimal power consumption
//!
//! The RTC Interrupt causes LED1 to toggle every second if LEDs are present
//! on the board. Else it toggles a custom pin every second.
//!
//! This example uses LFRC as the RTC clock source for lowest power. For better
//! accuracy, the XTAL could be used at a slight power increase.
//!
//! The GPIO interrupt can also wakeup the device and causes LED1 to toggle
//! every time the button0 is pressed or the designated gpio is toggled from Lo2Hi.
//! If Leds aren't found on the board it toggles a custom pin every second
//!
//! The ALL_RETAIN == 0 case, we requires a custom system-config.yaml different
//! from the default system-config.yaml. The reason that is necessary is because
//! the TCM Memory Configuration for ALL_RETAIN == 0 is AM_HAL_PWRCTRL_ITCM32K_DTCM128K
//! and the start and end addresses of stack, heap and TCM falls within that addresses
//! range.
//!
//! The example begins by printing out a banner annoucement message through
//! the UART, which is then completely disabled for the remainder of execution.
//!
//! Text is output to the UART at 115,200 BAUD, 8 bit, no parity.
//! Please note that text end-of-line is a newline (LF) character only.
//! Therefore, the UART terminal must be set to simulate a CR/LF.
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
#include "am_bsp.h"
#include "am_util.h"
#ifdef AM_BSP_IS_SIP
#include "am_devices_em9305.h"
#endif

#define ALL_RETAIN  0 // 0 for min TCM retain (default), 1 for all retain

//
// If push button and LED0 is not present, then custom define the pins.
// A pullup is required for INTERRUPT_GPIO (which configures an input)
// if the board doesn't provide it.
// Alternatively, set GPIO_INTERRUPT_ENABLE to 0 for maximum power reduction.
//
#if AM_BSP_NUM_BUTTONS > 0 & AM_BSP_NUM_LEDS > 0
// An external pullup on the board is assumed for INTERRUPT_GPIO.
#define WAKEUP_GPIO_PIN             AM_BSP_GPIO_LED0
#define WAKEUP_RTC_PIN              AM_BSP_GPIO_LED1
#define INTERRUPT_GPIO              AM_BSP_GPIO_BUTTON0
#else  // custom define
#define GPIOINPUT_PULLUP_NEEDED         // Assume there is no external pullup
#define WAKEUP_GPIO_PIN             0
#define WAKEUP_RTC_PIN              1
#define INTERRUPT_GPIO              5
#endif

//*****************************************************************************
//
// GPIO interrupt IRQn and ISR macros
//
//*****************************************************************************
#if INTERRUPT_GPIO <= 31
#define GPIO_INT_IRQN       GPIO0_001F_IRQn
#define am_gpio_isr         am_gpio0_001f_isr
#elif INTERRUPT_GPIO <= 63
#define GPIO_INT_IRQN       GPIO0_203F_IRQn
#define am_gpio_isr         am_gpio0_203f_isr
#elif INTERRUPT_GPIO <= 95
#define GPIO_INT_IRQN       GPIO0_405F_IRQn
#define am_gpio_isr         am_gpio0_405f_isr
#elif INTERRUPT_GPIO <= 127
#define GPIO_INT_IRQN       GPIO0_607F_IRQn
#define am_gpio_isr         am_gpio0_607f_isr
#elif INTERRUPT_GPIO <= 159
#define GPIO_INT_IRQN       GPIO0_809F_IRQn
#define am_gpio_isr         am_gpio0_809f_isr
#else
#error Invalid INTERRUPT_GPIO specified.
#endif

#define XT              1
#define LFRC            2
#define RTC_CLK_SRC     LFRC

#define GPIO_INTERRUPT_ENABLE   1
#define RTC_INTERRUPT_ENABLE    1

//
// Macro define for print statement upon wakeup
// When defined, set PRINT_WAKE to the number of seconds between prints.
//
#define PRINT_WAKE  5

#if PRINT_WAKE
uint32_t g_ui32RtcCounter = 0;
#endif // PRINT_WAKE

//*****************************************************************************
//
// GPIO ISR. Important note: Depending on the pin interrupt enabled, this
// ISR macro might need to change accordingly. Refer to device manual on the pin
// number to ISR mapping
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint32_t ui32IntStatus;

    //
    // Delay for GPIO debounce.
    //
    am_util_delay_us(20000);

    //
    // Clear the GPIO Interrupt (write to clear).
    //
    AM_CRITICAL_BEGIN
    am_hal_gpio_interrupt_irq_status_get(GPIO_INT_IRQN, true, &ui32IntStatus);
    am_hal_gpio_interrupt_irq_clear(GPIO_INT_IRQN, ui32IntStatus);
    AM_CRITICAL_END

    //
    // toggle GPIO wakeup pin, default LED0
    //
    am_hal_gpio_output_toggle(WAKEUP_GPIO_PIN);

#if PRINT_WAKE
    am_bsp_uart_printf_enable();
    am_util_stdio_printf("Device awakened by GPIO %d interrupt\n", INTERRUPT_GPIO);
    am_bsp_uart_printf_disable();
#endif // PRINT_WAKE
}

//*****************************************************************************
//
// RTC ISR
//
//*****************************************************************************
void
am_rtc_isr(void)
{
    //
    // Clear the RTC alarm interrupt and toggle RTC wakeup in, default LED1
    //
    am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);
    am_hal_gpio_output_toggle(WAKEUP_RTC_PIN);

#if PRINT_WAKE
    uint32_t ui32SleepState;

    g_ui32RtcCounter++;

    if ( g_ui32RtcCounter == PRINT_WAKE )
    {
        am_bsp_uart_printf_enable();

        //
        // Get sleep status
        //
        ui32SleepState = PWRCTRL->SYSPWRSTATUS;

        //
        // Clear the status bits
        //
        PWRCTRL->SYSPWRSTATUS = PWRCTRL_SYSPWRSTATUS_SLEEP_Msk;

        if ( ui32SleepState & PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP_Msk )
        {
#ifdef AM_PART_APOLLO330P_510L
            am_util_stdio_printf("[%d%d%d%d] ",
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_SYSDEEPSLEEP,   ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP,  ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_CORESLEEP,      ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_SYSDEEPERSLEEP, ui32SleepState));
#else // AM_PART_APOLLO510
            am_util_stdio_printf("[%d%d%d] ",
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_SYSDEEPSLEEP,   ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_COREDEEPSLEEP,  ui32SleepState),
                _FLD2VAL(PWRCTRL_SYSPWRSTATUS_CORESLEEP,      ui32SleepState));
#endif
        }
        else
        {
            am_util_stdio_printf("[No sleep detected] ");
        }

        am_util_stdio_printf("Device awakened by RTC interrupt %d times since last print\n", PRINT_WAKE);
        g_ui32RtcCounter = 0;

        am_bsp_uart_printf_disable();
    }
#endif // PRINT_WAKE
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
//
// The hardware treats the month as 1-12 indexed so the first entry in the month is an invalid one.
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
// RTC initialization function. Configures the RTC to run from designated source
// and enables RTC alarm interrupt. Set RTC wakeup pin as outut
//
//*****************************************************************************
void
am_rtc_init(void)
{
#if RTC_INTERRUPT_ENABLE
    am_hal_rtc_time_t hal_time;

    //
    // Configure RTC wakeup pin as output
    //
    am_hal_gpio_pinconfig(WAKEUP_RTC_PIN, am_hal_gpio_pincfg_output); // Wakeup Flag RTC
    am_hal_gpio_output_set(WAKEUP_RTC_PIN);

#if RTC_CLK_SRC == LFRC
    //
    // Enable the LFRC for the RTC.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC, 0);

    //
    // Select LFRC for RTC clock source.
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, 0);
    MCUCTRL->XTALCTRL = 0;
#endif
#if RTC_CLK_SRC == XT
    //
    // Enable the XT for the RTC.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_XTAL, 0);

    //
    // Select XT for RTC clock source
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_XT);
//#else
//#error RTC_CLK_SRC not properly specified.
#endif

    //
    // Clear RTC counter for this example
    //
    am_hal_rtc_counter_clear();

    //
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();

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
    // Century Bit is for tracking 2000s - 2100s
    //
    // 2000s  -> ui32CenturyBit == 1
    // 2100s  -> ui32CenturyBit == 0
    //
    hal_time.ui32CenturyBit = RTC_CTRUP_CB_2000;


    //
    // Set the alarm repeat interval to be every second.
    //
    am_hal_rtc_alarm_interval_set(AM_HAL_RTC_ALM_RPT_SEC);

    //
    // Check the validity of the input values
    //
    if (am_hal_rtc_time_set(&hal_time))
    {
        am_bsp_uart_printf_enable();
        am_util_stdio_printf("Invalid Input Value");
        am_bsp_uart_printf_disable();
        while(1);  // trap the device
    }

    //
    // Clear the RTC alarm interrupt.
    //
    am_hal_rtc_interrupt_clear(AM_HAL_RTC_INT_ALM);

    //
    // Enable the RTC alarm interrupt to the NVIC
    //
    am_hal_rtc_interrupt_enable(AM_HAL_RTC_INT_ALM);

    NVIC_SetPriority(RTC_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);
#else
    //
    // Enable the LFRC for the RTC.
    //
    am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_RTC_SEL_LFRC, 0);

    //
    // Select LFRC for RTC clock source.
    //
    am_hal_rtc_osc_select(AM_HAL_RTC_OSC_LFRC);

    //
    // Disable XTAL
    //
    MCUCTRL->XTALCTRL = 0;

    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_XTAL_PWDN_DEEPSLEEP, 0);

    //
    // Disable the RTC.
    //
    am_hal_rtc_osc_disable();

#endif
} // am_rtc_init()

//*****************************************************************************
//
// GPIO initialization function, configures GPIO interrupt pin as input and
// enable corresponding interrupt. Set wakeup GPIO pin as output
//
//*****************************************************************************
void
am_gpio_init(void)
{
#if GPIO_INTERRUPT_ENABLE

    am_hal_gpio_mask_t GpioIntMask = AM_HAL_GPIO_MASK_DECLARE_ZERO;

    //
    // Set the Input Pin Configuarions for Pin
    //
    am_hal_gpio_pincfg_t sInPinCfg    = AM_HAL_GPIO_PINCFG_DEFAULT;
    sInPinCfg.GP.cfg_b.eGPInput       = AM_HAL_GPIO_PIN_INPUT_ENABLE;
    sInPinCfg.GP.cfg_b.eIntDir        = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
    sInPinCfg.GP.cfg_b.eDriveStrength = AM_HAL_GPIO_PIN_DRIVESTRENGTH_0P5X;
#ifdef GPIOINPUT_PULLUP_NEEDED
    sInPinCfg.GP.cfg_b.ePullup        = AM_HAL_GPIO_PIN_PULLUP_100K;
#endif

    //
    // Interrupt Pin to be Tested
    //
    uint32_t ui32PinTestIntrpt = INTERRUPT_GPIO;
    am_hal_gpio_pinconfig(ui32PinTestIntrpt, sInPinCfg);

    // Set wakeup pin as output
    am_hal_gpio_pinconfig(WAKEUP_GPIO_PIN, am_hal_gpio_pincfg_output); // Wakeup Flag GPIO
    am_hal_gpio_output_set(WAKEUP_GPIO_PIN);

    //
    // Enable GPIO Interrupt
    //
    am_hal_gpio_interrupt_control(AM_HAL_GPIO_INT_CHANNEL_0, AM_HAL_GPIO_INT_CTRL_INDV_ENABLE, &ui32PinTestIntrpt);

    //
    // Read the GPIO Interrupt Status
    //
    am_hal_gpio_interrupt_status_get(AM_HAL_GPIO_INT_CHANNEL_0, true, &GpioIntMask);

    //
    // Clear the GPIO Interrupts
    //
    am_hal_gpio_interrupt_clear(AM_HAL_GPIO_INT_CHANNEL_0, &GpioIntMask);

    //
    // Enable GPIO interrupts to the NVIC
    //
    NVIC_SetPriority(GPIO_INT_IRQN, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_ClearPendingIRQ(GPIO_INT_IRQN);
    NVIC_EnableIRQ(GPIO_INT_IRQN);
#endif
} // am_gpio_init()

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // NVM, SRAM, and TCM configuration, retain minimal TCM
    //
    am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
    {
         //
         // In order to demonstrate the lowest possible power,
         // this example enables the ROM automatic power down feature.
         // This should not be used in general for most applications.
         //
        .eROMMode       = AM_HAL_PWRCTRL_ROM_AUTO,
#if defined(AM_PART_APOLLO510)
#if ALL_RETAIN
        .eDTCMCfg       = AM_HAL_PWRCTRL_ITCM256K_DTCM512K,
        .eRetainDTCM    = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#else
        .eDTCMCfg       = AM_HAL_PWRCTRL_ITCM32K_DTCM128K,
        .eRetainDTCM    = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#endif
        .eNVMCfg        = AM_HAL_PWRCTRL_NVM0_ONLY,
#elif defined(AM_PART_APOLLO330P_510L)
#if ALL_RETAIN
        .eDTCMCfg       = AM_HAL_PWRCTRL_DTCM256K,
        .eRetainDTCM    = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#else
        .eDTCMCfg       = AM_HAL_PWRCTRL_DTCM128K,
        .eRetainDTCM    = AM_HAL_PWRCTRL_MEMRETCFG_TCMPWDSLP_RETAIN,
#endif
        .eNVMCfg        = AM_HAL_PWRCTRL_NVM,
#endif // AM_PART_APOLLO510
        .bKeepNVMOnInDeepSleep     = false
    };

am_hal_pwrctrl_sram_memcfg_t SRAMMemCfg =
    {
#if ALL_RETAIN
#ifdef AM_PART_APOLLO330P_510L
      .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_1P75M,
#else
      .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_3M,
#endif
#else
      .eSRAMCfg         = AM_HAL_PWRCTRL_SRAM_NONE,
#endif
      .eActiveWithMCU   = AM_HAL_PWRCTRL_SRAM_NONE,
      .eActiveWithGFX   = AM_HAL_PWRCTRL_SRAM_NONE,
      .eActiveWithDISP  = AM_HAL_PWRCTRL_SRAM_NONE,
#if ALL_RETAIN
      .eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_1P75M
#else
      .eSRAMRetain      = AM_HAL_PWRCTRL_SRAM_NONE
#endif
    };

    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    // If Apollo510B device, turn off EM9305 to save power
    //
#ifdef AM_BSP_IS_SIP
    am_devices_em9305_shutdown();
#endif

#ifdef AM_PART_APOLLO330P_510L
    //
    // Power off the RSS
    //
    am_hal_pwrctrl_rss_pwroff();
#endif

    //
    // Initialize the printf interface for UART output.
    //
    am_bsp_uart_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Deepsleep Wake Example\n\n");
#if RTC_INTERRUPT_ENABLE
    am_util_stdio_printf("RTC interrupt enabled, RTC wakes up device every second.\n");
#else
    am_util_stdio_printf("RTC interrupt disabled.\n");
#endif
#if GPIO_INTERRUPT_ENABLE
    am_util_stdio_printf("GPIO interrupt enabled,\n");
    am_util_stdio_printf(" to wake device, press button 0 or toggle designated GPIO %d.\n", INTERRUPT_GPIO);
#else
    am_util_stdio_printf("GPIO interrupt disabled.\n");
#endif

#if PRINT_WAKE
    am_util_stdio_printf("\n[dcns]: d=SYSDEEPSLEEP, c=COREDEEPSLEEP, n=CORESLEEP, s=SYSDEEPERSLEEP\n\n");
#else
    am_util_stdio_printf("\n");
#endif // PRINT_WAKE

    //
    // We are done printing.
    // Disable the UART
    //
    am_bsp_uart_printf_disable();


    //
    // Disable all peripherals including Crypto
    //
    am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL, 0);

    //
    // Disable Debug Subsystem
    //
    MCUCTRL->DBGCTRL = 0;

    am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);

    //
    // Power off voltage comparator
    //
    VCOMP -> PWDKEY = VCOMP_PWDKEY_PWDKEY_Key;

    //
    // Disable SRAM
    //
    am_hal_pwrctrl_sram_config(&SRAMMemCfg);

    //
    // RTC configuration
    //
    am_rtc_init();

    //
    // GPIO configuration
    //
    am_gpio_init();

#if PRINT_WAKE
    //
    // Initialize (clear) the sleep state status bits
    //
    PWRCTRL->SYSPWRSTATUS = PWRCTRL_SYSPWRSTATUS_SLEEP_Msk;
#endif // PRINT_WAKE

    //
    // Enable interrupts to the core.
    //
    am_hal_interrupt_master_enable();

    while (1)
    {
        //
        // Go to Deep Sleep until wakeup.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
