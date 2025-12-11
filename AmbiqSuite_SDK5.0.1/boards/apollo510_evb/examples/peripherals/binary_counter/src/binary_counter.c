//*****************************************************************************
//!
//! @file binary_counter.c
//!
//! @brief Example that displays the timer count on the LEDs.
//!
//! @addtogroup peripheral_examples Peripheral Examples
//
//! @defgroup binary_counter Binary Counter Example
//! @ingroup peripheral_examples
//! @{
//!
//! Purpose: This example demonstrates timer-based LED counting functionality
//! with power mode switching capabilities. The application showcases timer
//! interrupt handling, LED control, and dynamic power mode management. The
//! example implements a binary counter using LEDs and demonstrates power
//! mode transitions between HIGH and LOW performance states.
//!
//! @section binary_counter_features Key Features
//!
//! 1. @b Timer @b Configuration: Demonstrates timer setup and
//!    interrupt-based counting operations
//!
//! 2. @b LED @b Control: Implements binary counting display using
//!    onboard LEDs for visual feedback
//!
//! 3. @b Power @b Management: Showcases dynamic switching between
//!    performance modes for power optimization
//!
//! 4. @b Sleep @b Mode: Utilizes deep sleep for power efficiency
//!    between timer events
//!
//! @section binary_counter_functionality Functionality
//!
//! The application performs the following operations:
//! - Initializes system timer with 1-second period
//! - Configures LED array for binary display output
//! - Implements timer interrupt for counter increment
//! - Displays binary count on LED array
//! - Toggles between power modes at counter reset
//! - Utilizes deep sleep between timer events
//!
//! @section binary_counter_usage Usage
//!
//! 1. Load and run the application
//! 2. Observe LED binary counting pattern
//! 3. Monitor power mode transitions via SWO output
//! 4. Verify deep sleep operation between counts
//!
//! @section binary_counter_configuration Configuration
//!
//! - Timer configured for 1-second interrupt
//! - LED array setup based on board configuration
//! - Power modes defined per device capabilities
//! - SWO/ITM configured for 1MHz output
//!
//! Additional Information:
//! Printing takes place over the SWO/ITM at 1MHz.
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

//*****************************************************************************
//
// Macros
//
//*****************************************************************************
#if AM_BSP_NUM_LEDS > 0
#define MAX_COUNT       ((1 << AM_BSP_NUM_LEDS) - 1)
#else
#define NUMLEDS         3       // Default to the expected number of LEDs
#define MAX_COUNT       ((1 << NUMLEDS) - 1)
#endif

#define TIMER_NUM       0       // Timer number to be used in the example

//#define PRINTF_DISABLE       // Defined inhibits all SWO printing (LEDs only)
#ifdef PRINTF_DISABLE
#define BC_PRINTF(...)
#else
#define BC_PRINTF(...)  am_util_stdio_printf(__VA_ARGS__)
#endif

//
// Structure used to describe power modes.
//
typedef struct
{
    char name[32];
    am_hal_pwrctrl_mcu_mode_e eMode;
    uint32_t ui32CoreFreqMHz;
} PowerModeInfo_t;

//*****************************************************************************
//
// Globals
//
//*****************************************************************************
volatile uint32_t g_ui32TimerCount = 0;
volatile bool     g_bTimerEvent = false;

//
// A list containing information about all power modes.
//
PowerModeInfo_t g_sPowerModeList[] =
{
#if defined(AM_PART_APOLLO510)
    {"Low Power", AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER, AM_HAL_CLKGEN_FREQ_MAX_MHZ},
    {"TurboSPOT", AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE, AM_HAL_CLKGEN_FREQ_HP250_MHZ},
#elif defined(AM_PART_APOLLO330P_510L)
    {"Low Power", AM_HAL_PWRCTRL_MCU_MODE_LOW_POWER, AM_HAL_CLKGEN_FREQ_MAX_MHZ},
    {"High Performance 1", AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE1, AM_HAL_CLKGEN_FREQ_HP192_MHZ},
    {"High Performance 2", AM_HAL_PWRCTRL_MCU_MODE_HIGH_PERFORMANCE2, AM_HAL_CLKGEN_FREQ_HP250_MHZ},
#endif
};

//*****************************************************************************
//
// Function to initialize the timer to interrupt once every second.
//
//*****************************************************************************
void
timer_init(void)
{
    am_hal_timer_config_t   TimerConfig;

    //
    // Set up the desired TIMER.
    // The default config parameters include:
    //  eInputClock = AM_HAL_TIMER_CLOCK_HFRC_DIV16
    //  eFunction = AM_HAL_TIMER_FN_EDGE
    //  Compare0 and Compare1 maxed at 0xFFFFFFFF
    //
    am_hal_timer_default_config_set(&TimerConfig);

    //
    // Modify the default parameters.
    // Configure the timer to a 1s period via ui32Compare1.
    //
    TimerConfig.eFunction        = AM_HAL_TIMER_FN_UPCOUNT;
    TimerConfig.ui32Compare1     = 6000000;

    //
    // Configure the timer
    //
    if ( am_hal_timer_config(TIMER_NUM, &TimerConfig) != AM_HAL_STATUS_SUCCESS )
    {
        BC_PRINTF("Failed to configure TIMER%d.\n", TIMER_NUM);
    }

    //
    // Clear the timer and its interrupt
    //
    am_hal_timer_clear(TIMER_NUM);
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));

} // timer_init()

//*****************************************************************************
//
// Timer Interrupt Service Routine (ISR)
//
//*****************************************************************************
void
am_timer00_isr(void)
{
    //
    // Clear the timer Interrupt (write to clear).
    //
    am_hal_timer_interrupt_clear(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));
    am_hal_timer_clear(TIMER_NUM);

    //
    // Increment count and set limit based on the number of LEDs available.
    //
    g_ui32TimerCount = ( g_ui32TimerCount >= MAX_COUNT ) ? 0 : g_ui32TimerCount + 1;
    //
    // Set the flag for main loop
    //
    g_bTimerEvent = true;

} // am_timer00_isr()

//*****************************************************************************
//
// Cycle through the power modes specified in g_sPowerModeList.
//
//*****************************************************************************
void
power_mode_loop(void)
{
    am_hal_pwrctrl_mcu_mode_e eCurrentPowerMode;
    uint32_t ui32Ret, ui32Idx0, ui32Idx1;
    bool bFoundInList = false;

    //
    // Get the current power mode.
    //
    ui32Ret = am_hal_pwrctrl_mcu_mode_status(&eCurrentPowerMode);
    if ( ui32Ret != AM_HAL_STATUS_SUCCESS )
    {
        BC_PRINTF("\n\nERROR (%d) getting current power mode status\n", ui32Ret);
        return;
    }

    //
    // Find the next power mode in the g_sPowerModeList.
    //
    uint32_t len = sizeof(g_sPowerModeList) / sizeof(PowerModeInfo_t);
    for (uint32_t i = 0; i < len; i++)
    {
        if (eCurrentPowerMode == g_sPowerModeList[i].eMode)
        {
            bFoundInList = true;
            ui32Idx0 = i;
            ui32Idx1 = (i + 1) % len;
        }
    }

    //
    // Switch to that power mode if found.
    //
    if (bFoundInList)
    {
        ui32Ret = am_hal_pwrctrl_mcu_mode_select(g_sPowerModeList[ui32Idx1].eMode);
        if (ui32Ret == AM_HAL_STATUS_SUCCESS)
        {
            BC_PRINTF("\n\nSwitched to %s mode (%dMHZ)\n", g_sPowerModeList[ui32Idx1].name, g_sPowerModeList[ui32Idx1].ui32CoreFreqMHz);
        }
        else
        {
            BC_PRINTF("\n\nERROR (%d) while switching from %s mode to %s mode.\n", ui32Ret, g_sPowerModeList[ui32Idx0].name, g_sPowerModeList[ui32Idx1].name);
        }
    }
    else
    {
        BC_PRINTF("\n\nERROR, invalid current power mode = (%d).\n", (uint32_t)eCurrentPowerMode);
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    //
    // Configure the board for low power operation.
    //
    am_bsp_low_power_init();

    //
    //  Enable the I-Cache and D-Cache.
    //
    am_hal_cachectrl_icache_enable();
    am_hal_cachectrl_dcache_enable(true);

#if AM_BSP_NUM_LEDS > 0
    //
    // Initialize the LED array
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
#endif

    //
    // Initialize the count so that the counting begins with 0
    // after the first trip to the ISR.
    //
    g_ui32TimerCount = MAX_COUNT;

#ifdef PRINTF_DISABLE
    //
    // Initialize timer.
    //
    timer_init();
#else
    //
    // Initialize the printf interface for SWO/ITM output
    //
    am_bsp_debug_printf_enable();

    //
    // Clear the terminal and print the banner.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("Binary Counter Example for %s\n", AM_HAL_DEVICE_NAME);
    am_util_stdio_printf("---------------------------------------------------- \n");
    am_util_stdio_printf("The Timer clock source is HFRC_DIV16.\n");
    am_util_stdio_printf("The Timer wakes once per second to show a binary \n");
    am_util_stdio_printf(" value on the LEDs and a decimal value on SWO.\n");
    am_util_stdio_printf("---------------------------------------------------- \n");

    //
    // Initialize timer.
    //
    timer_init();

    //
    // Disable debug printf messages on SWO.
    //
    am_bsp_debug_printf_disable();
#endif // !PRINTF_DISABLE

    //
    // Enable the timer Interrupt.
    //
    am_hal_timer_interrupt_enable(AM_HAL_TIMER_MASK(TIMER_NUM, AM_HAL_TIMER_COMPARE1));

    //
    // Enable the timer interrupt in the NVIC.
    //
    NVIC_SetPriority(TIMER0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(TIMER0_IRQn);
    am_hal_interrupt_master_enable();

    //
    // Start the timer
    //
    am_hal_timer_start(TIMER_NUM);

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // Go to Deep Sleep.
        //
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

        if (g_bTimerEvent)
        {
            g_bTimerEvent = false;
#if AM_BSP_NUM_LEDS > 0
            //
            // Set the LEDs.
            //
            am_devices_led_array_out(am_bsp_psLEDs, AM_BSP_NUM_LEDS, g_ui32TimerCount);
#endif // AM_BSP_NUM_LEDS

#ifndef PRINTF_DISABLE
            //
            // Enable debug printf messages using SWO.
            //
            am_bsp_debug_printf_enable();
#endif // !PRINTF_DISABLE

            //
            // Output current count.
            //
            BC_PRINTF("%2d ", g_ui32TimerCount);

            if ( g_ui32TimerCount >= MAX_COUNT )
            {
                //
                // Take this opportunity to toggle TurboSPOT mode.
                //
                power_mode_loop();

#if defined(AM_PART_APOLLO510)
                //
                // Polling the timer AM_HAL_INTERNAL_TIMER_NUM_A status until it is disabled,
                // to guarantee all power state updating operations are completed.
                //
                if (APOLLO5_B1_GE_PCM2P1 || APOLLO5_B2_GE_PCM2P1 || APOLLO5_GT_B2)
                {
                    if (am_hal_delay_us_status_change(10000,
                                                    (uint32_t)&TIMERn(AM_HAL_INTERNAL_TIMER_NUM_A)->CTRL0,
                                                    TIMER_CTRL0_TMR0EN_Msk,
                                                    TIMER_CTRL0_TMR0EN_DIS)
                        != AM_HAL_STATUS_SUCCESS )
                    {
                        BC_PRINTF("\n\nERROR, power mode switch failed.\n");
                    }
                }
#endif
            }

#ifndef PRINTF_DISABLE
            //
            // Disable debug printf messages on SWO.
            //
            am_bsp_debug_printf_disable();
#endif // !PRINTF_DISABLE

        }
    } // while()

} // main()

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

