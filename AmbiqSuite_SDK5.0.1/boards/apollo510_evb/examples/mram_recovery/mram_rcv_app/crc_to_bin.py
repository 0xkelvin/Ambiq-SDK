#!/usr/bin/env python3
import argparse
import binascii
from pathlib import Path

HEADER_SIZE_WORDS = 16
HEADER_SIZE = (HEADER_SIZE_WORDS * 4)

config_header = '''\
#******************************************************************************
#
# Configuration file for create_cust_image_blob.py
#
# Run "create_cust_image_blob.py --help" for more information about the options
# below.
#
# All numerical values below may be expressed in either decimal or hexadecimal
# "0x" notation.
#
# To re-generate this file using all default values, run
# "create_cust_image_blob.py --create-config"
#
#******************************************************************************
'''

def main():
    args = parse_arguments()
    binary = bytearray()

    # Open the file, and read it into an array of integers.
    print("Reading input file", args.binfile, "\n")
    
    with open(args.binfile, "rb") as f_app:
        binary.extend(f_app.read())
    
    # Add padding if needed
    binary = pad_to_block_size(binary, 4, 0)
    appLength  = (len(binary))

    crc = binascii.crc32(binary[args.codeOffset:appLength]) & 0xFFFFFFFF
    print("The following patch data has been written to image @ offset:", hex(args.patchOffset))
    print("Word 0: Code Offset -", hex(args.codeOffset))
    fill_word(binary, args.patchOffset, args.codeOffset)
    print("Word 1: Code Size -", hex(appLength - args.codeOffset))
    fill_word(binary, args.patchOffset + 4, appLength - args.codeOffset)
    print("Word 2: CRC -", hex(crc), "\n")
    fill_word(binary, args.patchOffset + 8, crc)

    outputFile = args.output if args.output else args.binfile
    
    print("Writing to file", outputFile)
    with open(outputFile, mode = 'wb') as out:
        out.write(binary)

def parse_arguments():
    parser = argparse.ArgumentParser(description = 'Generate Combined Recovery Image Blob')

    parser.add_argument('-i', '--bin', dest='binfile', type=str,
                        help='Input binary executable file (binfile.bin)')

    parser.add_argument('--patchOffset', dest = 'patchOffset', type=auto_int, default=0x25C,
                        help = 'Patch Region offset')

    parser.add_argument('--codeOffset', dest = 'codeOffset', type=auto_int, default=0x400,
                        help = 'Code Start offset')

    parser.add_argument('-o', '--output', dest = 'output', default=None,
                        help = 'Output filename')

    args = parser.parse_args()

    return args


#******************************************************************************
#
# Pad the text to the block_size. bZeroPad determines how to handle text which
# is already multiple of block_size
#
#******************************************************************************
def pad_to_block_size(text, blockSize, bZeroPad):
    textLength = len(text)
    amountToPad = blockSize - (textLength % blockSize)
    if (amountToPad == blockSize):
        if (bZeroPad == 0):
            amountToPad = 0
    for i in range(0, amountToPad, 1):
        text += bytes(chr(amountToPad), 'ascii')
    return text

def words_to_bytes(W):
    for word in W:
        yield (word & 0x000000FF)
        yield (word & 0x0000FF00) >> 8
        yield (word & 0x00FF0000) >> 16
        yield (word & 0xFF000000) >> 24

#******************************************************************************
#
# Fill one word in bytearray
#
#******************************************************************************
def fill_word(barray, offset, w):
    barray[offset + 0]  = (w >>  0) & 0x000000ff
    barray[offset + 1]  = (w >>  8) & 0x000000ff
    barray[offset + 2]  = (w >> 16) & 0x000000ff
    barray[offset + 3]  = (w >> 24) & 0x000000ff

#******************************************************************************
#
# automatically figure out the integer format (base 10 or 16)
#
#******************************************************************************
def auto_int(x):
    return int(x, 0)


if __name__ == '__main__':
    main()
