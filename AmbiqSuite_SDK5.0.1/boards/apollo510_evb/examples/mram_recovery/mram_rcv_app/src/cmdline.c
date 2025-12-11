//*****************************************************************************
//
// cmdline.c - Simple command line processor
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

#include <ctype.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "cmdline.h"
#include "am_mcu_apollo.h"
#include "am_util_stdio.h"
#include "am_util_delay.h"
#include "hello_world_oem_recovery.h"

#define am_uart_printf   am_print_uart_terminal
#define am_printf        am_print_swo_uart_terminal

void trigger_reset(void)
{
    //
    // Reset the device
    //
    am_printf("Resetting the device\n");
    RSTGEN->SWPOR = RSTGEN_SWPOR_SWPORKEY_KEYVALUE;
}

// global buffer for the command line to be stored and processed
char gCbuf[CBUF_SIZE];

//
// cmdLineHelp() -- command line help
//
void
cmdLineHelp(void)
{
    am_printf("\n**********************************************\n\n COMMAND \n");
    am_printf("\n**********************************************\n\n");
    am_printf( "Command Line Help: \n\r\n" );
    am_printf( "    S         - Display CRC & WDT Count.\n");
    am_printf( "    A         - Application Initiated MRAM Recovery.\n");
    am_printf( "    R         - Device Reset( POR ).\n");
    am_printf( "    W         - Watchdog Reset.\n");
    am_printf( "    C         - Corrupt Image( Invalidate CRC ).\n");
    am_printf( "    TC[x]     - Change CRC trigger time. x is in decimal milli seconds.\n");
    am_printf( "    TW[x]     - Change Watchdog trigger time. x is in decimal milli seconds.\n");
    am_printf( "    I         - Display INFO 0 Settings.\n");
    am_printf( "    F0        - Force Hardfault without MRAM Corruption.\n");
    am_printf( "    F1        - Force Hardfault with MRAM Corruption.\n");
    am_printf( "    M         - Print the MRAM Recovery.\n");
    am_printf( "\n\n\n");
}

//
// Ascii decimal number to int32 (neg numbers preceed wtih a '-')
// returns the pointer to the next (non-numeric location) in the strig
// and the number in the *data parameter
//
char *
dec2int32(char *cp, int32_t *data)
{
    uint32_t d = 0;
    bool neg = false;
    uint16_t x;

    *data = 0;
    if (isdigit((int)*cp) || (*cp == '-' && isdigit((int)*(cp + 1))) )
    {
        if (*cp == '-')
        {
            neg = true;
            cp++;
        }
        for (x = 0; x < 10 && isdigit((int)*cp); x++)
        {
            d = (d*10) + (*cp - '0');
            cp++;
        }
        if (neg)
        {
            d = -d;
        }
        *data = d;
    }
    return cp;
}

//
// hex2int32() -- convert Hex ascii string into 32 bit number - 8 digits max
// returns the pointer to the next (non-hex location) in the strig
// and the number in the *num parameter
//
char *
hex2int32(char *cp, uint32_t *num)
{
    int32_t x;
    char c;

    *num = 0;
    for (x = 0; x < 8; x++)
    {
        c = toupper(*cp);
        if (c >= '0' && c <= '9')
        {
            *num = (*num << 4) + (c - '0');
        }
        else if (c >= 'A' && c <= 'F')
        {
            *num = (*num << 4) + (c - 'A' + 10);
        }
        else
        {
            break;
        }
        cp++;
    }
    return cp;
}

//
// S cmds, Display Status
//
bool
status_cmd(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        am_wdt_crc_status_print();
        am_printf( "\n\n\r" );
        return false;
    }
    return cmd_error;
}

//
// I cmds, Display INFO0 Settings
//
bool
info_cmd(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        am_print_info0();
        am_printf( "\n\n\r" );
        return false;
    }
    return cmd_error;
}

//
// T cmds, Display Status
//
bool
change_time_cmd(char *cp)
{
    bool cmd_error = false;
    int32_t ui32Time;

    char ch;
    ch = *(cp + 1);

    switch(toupper(*cp))
    {
        case '\0':
            cmd_error = true;
            break;

        case 'W':
            if (ch == '[')
            {
                cp = dec2int32(cp + 2, &ui32Time);
                if (*cp == ']')
                {
                    am_change_wdt_trigger_time(ui32Time);
                }
                else
                {
                    cmd_error = true;
                }
                am_printf( "\n\n\r" );
            }
            else
            {
                cmd_error = true;
            }
            break;

        case 'C':
            if (ch == '[')
            {
                cp = dec2int32(cp + 2, &ui32Time);
                if (*cp == ']')
                {
                    am_change_crc_trigger_time(ui32Time);
                }
                else
                {
                    cmd_error = true;
                }
                am_printf( "\n\n\r" );
            }
            else
            {
                cmd_error = true;
            }
            break;
    }
    return cmd_error;
}

//
// A cmds
//
bool
application_rcvy_cmd(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        am_printf("Initiating Application MRAM Recovery\n")
        am_util_delay_ms(200);
        am_hal_mram_recovery_init_app_recovery(AM_HAL_MRAM_RECOVERY_KEY, true);
        am_printf( "\n\n\r" );
        return false;
    }
    return cmd_error;
}

//
// M cmds
//
bool
print_mram_rcv_msg_cmd(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        am_print_mram_recovery_message(false);
        am_printf( "\n\n\r" );
        return false;
    }
    return cmd_error;
}

//
// R cmds,
//
bool
reset_cmd(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        trigger_reset();
        am_printf( "\n\n\r" );
    }
    return cmd_error;
}

//
// W cmds
//
bool
watchdog_reset_cmd(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        am_trigger_wdt_reset();
        am_printf( "\n\n\r" );
        return false;
    }
    return cmd_error;
}

//
// C cmds
//
bool
crc_corrupt(char *cp)
{
    bool cmd_error = false;

    if (*cp == '\0')
    {
        am_corrupt_crc();
        am_printf( "\n\n\r" );
        return false;
    }
    return cmd_error;
}

//
// F cmds
//
bool
hardfault_cmd(char *cp)
{
    bool cmd_error = false;
    char ch;

    ch = toupper(*(cp + 1));

    switch(toupper(*cp))
    {
        case '\0':
            cmd_error = true;
            break;
        case '0':
            if ( ch == '\0' )
            {
                am_hardfault_no_corruption();
            }
            else
            {
                cmd_error = true;
            }
            break;
        case '1':
            if ( ch == '\0' )
            {
                am_hardfault_corruption();
            }
            else
            {
                cmd_error = true;
            }
            break;
        default:
            cmd_error = true;
    }
    return cmd_error;
}

//****************************************************************************************************
//
// cmdLine() -- the main command line processor, it collects the characters into a buffer, processing
//              backspaces and parses the first letter of the command, calling the appropiate sub
//              function to process each group of commands.
//
//  uses gCbuf[] to store and process the command line in.
//
//****************************************************************************************************
bool
cmdLine(uint32_t ui32Module)
{
    static char *cp = gCbuf;
    static bool new_cmd = true;
    bool cmd_done = false;
    bool cmd_error = false;
    char ch;

    if (g_UARTActCount)
    {
        int32_t i32Temp = 21;
        while (i32Temp != 0)
        {
            am_print_uart_terminal("\b \b");
            i32Temp--;
        }
        g_UARTActCount = 0;
    }

    if (new_cmd)
    {
        am_uart_printf("> ");
        new_cmd = false;
        cp = gCbuf;  // Reset the buffer pointer at the start of a new command
        memset(gCbuf, 0, sizeof(gCbuf));  // Clear the buffer
    }

    // Get received character or zero if nothing has been received yet
    if ((ch = UARTn(ui32Module)->DR) )
    {
        if (ch == '\b')  // Handle backspace
        {
            if (cp > &gCbuf[0])  // Prevent buffer underflow
            {
                am_uart_printf("\b \b");  // Remove the character from the terminal
                cp--;  // Move the pointer back
            }
        }
        else if (ch == '\r')  // End of input (ENTER)
        {
            *cp = '\0';  // Null-terminate the input
            am_uart_printf("\n\r");  // Output CR/LF

            if (cp != &gCbuf[0])  // Check if there's a command to process
            {
                switch (toupper(gCbuf[0]))  // Process command based on the first character
                {
                    case 'H': case '?':
                        if (gCbuf[1] == '\0')
                        {
                            cmdLineHelp();
                        }
                        else
                        {
                            cmd_error = true;
                        }
                        break;

                    case 'S':
                        cmd_error = status_cmd(&gCbuf[1]);
                        break;

                    case 'A':
                        cmd_error = application_rcvy_cmd(&gCbuf[1]);
                        break;

                    case 'R':
                        cmd_error = reset_cmd(&gCbuf[1]);
                        break;

                    case 'W':
                        cmd_error = watchdog_reset_cmd(&gCbuf[1]);
                        break;

                    case 'C':
                        cmd_error = crc_corrupt(&gCbuf[1]);
                        break;

                    case 'T':
                        cmd_error = change_time_cmd(&gCbuf[1]);
                        break;

                    case 'I':
                        cmd_error = info_cmd(&gCbuf[1]);
                        break;

                    case 'F':
                        cmd_error = hardfault_cmd(&gCbuf[1]);
                        break;

                    case 'M':
                        cmd_error = print_mram_rcv_msg_cmd(&gCbuf[1]);
                        break;

                    default:
                        cmd_error = true;
                }

                // Wait until all data has been transmitted
                while (!UARTn(ui32Module)->FR_b.TXFE);

                if (cmd_error)
                {
                    am_uart_printf("cmd error\n\n\r");
                }
            }

            // Reset for the next command
            cp = gCbuf;  // Reset the buffer pointer
            new_cmd = true;  // Set flag to indicate a new command can be entered
            cmd_done = true;
        }
        else if (cp < &gCbuf[CBUF_SIZE - 1])  // Add character to buffer if there's space
        {
            *cp++ = ch;
            am_uart_printf("%c", ch);  // Echo the character
        }
        else
        {
            // Buffer is full, wait for Enter to reset
            am_uart_printf("\nBuffer full. Press Enter to continue.\r");
        }
    }

    return cmd_done;
}

