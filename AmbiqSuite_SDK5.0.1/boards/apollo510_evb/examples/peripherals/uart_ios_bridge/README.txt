
Name:
=====
 uart_ios_bridge

Description:
============
 UART-to-IOS bridge: Converts UART transfer commands from a PC host

        to SPI transactions for Apollo5 SBL updates or MRAM recovery.
 This program is runs for an intermediate host board that bridges the UART
 to the targets IOS interface to work with the uart_wired_update.py and
 uart_recovery_host.py scripts running on a PC, to perform SBL wired updates
 and MRAM Recovery using the IOS wired interface.
 Printing/logging is performed over SWO at 1MHz.
 -----------------------------------------------------------------------------
 PIN fly lead connections assumed:
    HOST Bridge(APOLLO510_EVB)[IOM]    TARGET (Apollo510_EVB)(IOS)
    ------------------                 ---------------------------------------
    GPIO[2]   GPIO Interrupt  <---     GPIO[0]  GPIO interrupt (SLINT)
    GPIO[18]  Boot OVERRIDE / --->     GPIO[18] Override pin/Initiate MRAM Recovery GPIO
              MRAM_RCVY_CTRL(INFO0)
    GPIO[1]   Reset           --->     Reset pin (or n/c)
    GND       Ground          <->      GND
 Apollo5 SPI connections:
    IOM 0                              IOS 0
    GPIO[5]   IOM0 SPI CLK    --->     GPIO[11] IOS SPI SCK
    GPIO[7]   IOM0 SPI MISO   <---     GPIO[83] IOS SPI MISO
    GPIO[6]   IOM0 SPI MOSI   --->     GPIO[52] IOS SPI MOSI
    GPIO[10]  IOM0 SPI nCE    --->     GPIO[13] IOS SPI nCE
 Reset/Override connections from the host are optional, but automate the
 process. If not used, put the target device in the desired mode
 before starting this bridge program.
 General usage:
   1. Update pin #defines as needed.
   2. Connect all pins between host and target.
   3. Load and run this firmware on the host.
   4. Press reset. The subordinate is put in update mode.
   5. Start the UART script on the PC.
   6. The bridge converts UART commands to IOS (SPI/I2C) signals.
 The program supports 3 operation:
  - SBL Wired Update: Transfers SBL update packets from the host to the
    subordinate device, reset the target device and puts it into
    boot-override. (Default Mode, on a simple reset) Use with
    uart_wired_update.py
  - Bridge Mode for MRAM Recovery: Does not reset the target, passively
    bridges UART <--> IOS. (When Reset w/SW1 pressed).Used with
    mram_recovery_host.py.
  - Initiate MRAM recovery via GPIO: When in BRIDGE mode, pressing SW1,
    will reset the target and initiate MRAM recovery via the "Initiate
    MRAM Recovery GPIO" pin.


******************************************************************************


