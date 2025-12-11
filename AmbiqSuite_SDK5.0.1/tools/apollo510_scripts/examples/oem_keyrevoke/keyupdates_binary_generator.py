#!/usr/bin/env python3

# *****************************************************************************
#
#    keyupdates_binary_generator.py
#
#   Script to generate binary file from key_updates.ini configuration.

#   The script reads key settings from an key_updates.ini file and creates a binary file
#   with 4 32-bit words in little-endian format, where specific bits are set
#   based on the key configurations.
#
# *****************************************************************************

# *****************************************************************************
#
#    Copyright (c) 2025, Ambiq Micro, Inc.
#    All rights reserved.
#
#    Redistribution and use in source and binary forms, with or without
#    modification, are permitted provided that the following conditions are met:
#
#    1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
#    3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
#    Third party software included in this distribution is subject to the
#    additional license terms as defined in the /docs/licenses directory.
#
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#    ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#    LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#    SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#    INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#    CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#    ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#    POSSIBILITY OF SUCH DAMAGE.
#
#  This is part of revision release_sdk5p0p1-61912905f0 of the AmbiqSuite Development Package.
#
# *****************************************************************************

import configparser
import sys
import os

def parse_ini_file(ini_filename):
    """
    Parse the INI file and extract key settings.
    
    Args:
        ini_filename (str): Path to the INI file
        
    Returns:
        dict: Dictionary with key settings
    """
    config = configparser.ConfigParser()
    
    try:
        config.read(ini_filename)
    except Exception as e:
        print(f"Error reading INI file: {e}")
        sys.exit(1)
    
    if 'Settings' not in config:
        print("Error: 'Settings' section not found in INI file")
        sys.exit(1)
    
    settings = config['Settings']
    key_settings = {}
    
    # Parse all keybank settings
    for key, value in settings.items():
        key_settings[key] = int(value)
    
    return key_settings

def uint32_to_little_endian_bytes(value):
    """
    Convert a 32-bit unsigned integer to 4 bytes in little-endian format.
    
    Args:
        value (int): 32-bit unsigned integer
        
    Returns:
        bytes: 4 bytes in little-endian format
    """
    return bytes([
        value & 0xFF,           # Least significant byte first
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF    # Most significant byte last
    ])

def generate_binary(key_settings, output_filename):
    """
    Generate the binary file based on key settings.
    
    Args:
        key_settings (dict): Dictionary with key settings
        output_filename (str): Path to output binary file
    """
    # Initialize 4 words (32-bit each) to 0
    words = [0, 0, 0, 0]
    
    # Mapping of key names to bit positions in word 0
    key_to_bit = {
        'keybank0key0': 0,   # bit 0
        'keybank0key1': 1,   # bit 1
        'keybank0key2': 2,   # bit 2
        'keybank0key3': 3,   # bit 3
        'keybank1key0': 4,   # bit 4
        'keybank1key1': 5,   # bit 5
        'keybank1key2': 6,   # bit 6
        'keybank1key3': 7,   # bit 7
        'keybank2key0': 8,   # bit 8
        'keybank2key1': 9,   # bit 9
        'keybank2key2': 10,  # bit 10
        'keybank2key3': 11,  # bit 11
        'keybank3key0': 12,  # bit 12
        'keybank3key1': 13,  # bit 13
        'keybank3key2': 14,  # bit 14
        'keybank3key3': 15,  # bit 15
    }
    
    # Set bits in word 0 based on key settings
    for key_name, bit_position in key_to_bit.items():
        if key_name in key_settings and key_settings[key_name] == 1:
            words[0] |= (1 << bit_position)
            print(f"Setting bit {bit_position} for {key_name}")
    
    # Write binary file with 4 words in little-endian format
    try:
        with open(output_filename, 'wb') as f:
            for i, word in enumerate(words):
                # Convert word to little-endian bytes manually
                little_endian_bytes = uint32_to_little_endian_bytes(word)
                bytes_written = f.write(little_endian_bytes)
                hex_bytes = ' '.join(f'{b:02x}' for b in little_endian_bytes)
                print(f"Word {i}: 0x{word:08X} -> bytes: {hex_bytes} ({bytes_written} bytes written)")
        
        # Verify file was created correctly
        file_size = os.path.getsize(output_filename)
        print(f"\nBinary file '{output_filename}' created successfully")
        print(f"File size: {file_size} bytes (expected: 16 bytes)")
        
        # Read back and verify
        with open(output_filename, 'rb') as f:
            file_content = f.read()
            hex_content = ' '.join(f'{b:02x}' for b in file_content)
            print(f"File content (hex): {hex_content}")
        
        print(f"\nWord summary:")
        print(f"Word 0: 0x{words[0]:08X}")
        print(f"Word 1: 0x{words[1]:08X}")
        print(f"Word 2: 0x{words[2]:08X}")
        print(f"Word 3: 0x{words[3]:08X}")
        
        # Expected output check for your current INI (keybank0key0=1)
        if words[0] == 1:  # Only keybank0key0=1
            print(f"\n✓ Expected result for little-endian: 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00")
        
    except Exception as e:
        print(f"Error writing binary file: {e}")
        sys.exit(1)

def main():
    """Main function."""
    # Default filenames
    ini_filename = "key_updates.ini"
    output_filename = "key_updates.bin"
    
    # Check command line arguments
    if len(sys.argv) > 1:
        ini_filename = sys.argv[1]
    if len(sys.argv) > 2:
        output_filename = sys.argv[2]
    
    # Check if input file exists
    if not os.path.exists(ini_filename):
        print(f"Error: Input file '{ini_filename}' not found")
        sys.exit(1)
    
    print(f"Reading configuration from: {ini_filename}")
    print(f"Output binary file: {output_filename}")
    
    # Parse INI file
    key_settings = parse_ini_file(ini_filename)
    
    # Generate binary file
    generate_binary(key_settings, output_filename)

if __name__ == "__main__":
    main()