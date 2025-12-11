#!/usr/bin/env python3

import argparse
import configparser
import pylink
from pathlib import Path
import time
import importlib.util
import sys
import subprocess

# Add scripts directory to system path to import helper scripts
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from am_defines import *
import create_info0

def main():

    nvLoaderDefaults = get_defaults()

    loaderUtil = LoaderUtility()

    loaderUtil.apply_defaults(**nvLoaderDefaults)

    # If regenInfo0 is enabled, recreate an info0 binary from the given info0 config
    if loaderUtil.regenInfo0 or loaderUtil.loadInfo0:
        info0Defaults = create_info0.initialize_defaults()

    if loaderUtil.regenInfo0:
        create_info0.parse_config(loaderUtil.info0Cfg, info0Defaults)
        create_info0.process(info0Defaults)

        if loaderUtil.info0Path is None:
            print("No info0 path was given, placing inf0 binary in scripts directory.")
            # Move info0 binary to a known location
            subprocess.call(f"mv {(info0Defaults['output'] + '.bin')} ../../", shell=True)
            info0Path = os.path.join("../../",  (info0Defaults['output'] + '.bin'))
        else:
            # Move info0 binary to info0Path location
            os.replace('info0.bin', loaderUtil.info0Path)
            info0Path = loaderUtil.info0Path

    # If loadInfo0 is enabled, load the proper info0 binary
    if nvLoaderDefaults['loadInfo0']:
        otp = nvLoaderDefaults['otp']
        if os.path.isfile(info0Path):
            loaderUtil.write_info0(info0Path, otp)
        else:
            raise FileNotFoundError(f"File '{info0Path}' not found!")

    loaderUtil.write_nv_device()

# TODO: MAKE SURE VALIDATION OF FIELDS START HERE (test recoveryType 5)
def get_defaults():
    parser = parse_arguments()
    args = parser.parse_args()
    args.parser = parser

    defaults = {
        'jlinkSerialNum': None,
        'rcvyStructAddress': 0x00000000,
        'ambiqRcvyImg': None,
        'ambiqRcvyImgAddress': 0x00000000,
        'ambiqRcvyImgOffset': 0x00000000,
        'oemRcvyImg': None,
        'oemRcvyImgAddress': 0x00000000,
        'oemRcvyImgOffset': 0x00000000,
        'loaderApp': None,
        'loaderAppAddress': 0x00000000,
        'readWrite': 0x0,
        'MRAM_RCVY_CTRL_NV_RCVY_TYPE': 0x00000000,
        'NV_METADATA_OFFSET_META_OFFSET': 0x00000000,
        'MRAM_RCVY_CTRL_NV_MODULE_NUM': 0x00000000,
        'MRAM_RCVY_CTRL_EMMC_PARTITION': 0x00000000,
        'NV_CONFIG0_CONFIG0': 0x0,
        'NV_CONFIG3_CONFIG3': 0x00000000,
        'NV_PIN_NUMS_CE_PIN': 0x0,
        'otp' : 0x0,
        'info0Path': None,
        'regenInfo0': 0x0,
        'loadInfo0': 0x0,
        'info0Cfg': None,
        'loglevel': 2,
        'loaderCfg': None,
    }

    am_set_print_level(defaults['loglevel'])

    # Read config fields and update the values within the dictionary
    if args.loaderCfg:
        parse_config(args.loaderCfg, defaults)

    # Giving highest precedence to cmd line args, update the values within the dictionary
    for key,value in vars(args).items():
        if value == None:
            continue
        defaults[key] = value

    # If info0Cfg was given as argument, ensure both loader cfg and info0 cfg have the same values for MRAM recovery related fields.
    if defaults['info0Cfg'] is not None:
        compare_configs(defaults['loaderCfg'], defaults['info0Cfg'])

    return defaults

def parse_arguments():
    parser = argparse.ArgumentParser(description='load MRAM recovery assets to MSPI/EMMC')

    parser.add_argument('--baseAddr',  dest='rcvyStructAddress', type=str,
                    help='Location of the recovery information structure.')

    parser.add_argument('--ambiqRcvy',  dest='ambiqRcvyImg', type=str,
                        help='Relative path to Ambiq recovery image.')

    parser.add_argument('--oemRcvy',  dest='oemRcvyImg', type=str,
                        help='Relative path to OEM recovery image.')

    parser.add_argument('--ldrApp',  dest='loaderApp', type=str,
                        help='Relative path to the device loader binary.')

    parser.add_argument('--rcvyType',  dest='MRAM_RCVY_CTRL_NV_RCVY_TYPE', type=auto_int, choices=[0x0, 0x1, 0x2],
                        help='MRAM recovery NV device type. 0 = NV_OFF, 1 = MSPI, 2 = EMMC (Default = 0x0)')

    parser.add_argument('--meta', dest='NV_METADATA_OFFSET_META_OFFSET', type=auto_int,
                        help='Offset to the meta-data in the selected NV device. (Default = 0x00000000)')

    parser.add_argument('--rcvyModNum', dest='MRAM_RCVY_CTRL_NV_MODULE_NUM', type=auto_int, choices=[0x0, 0x1, 0x2, 0x3],
                        help='Module number for the selected NV device. (Default = 0x00000000)')

    parser.add_argument('--emmcPart',  dest='MRAM_RCVY_CTRL_EMMC_PARTITION', type=auto_int, choices=[0x0, 0x1],
                        help='When configured for MRAM recovery from eMMC, this field specifies the partition where the recovery image is stored. 0 = User, 1 = BOOT1, 2 = BOOT2 (Default = 0x0)')

    parser.add_argument('--nv0', dest='NV_CONFIG0_CONFIG0', type=auto_int, choices=[0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9],
                        help='The pin number for the CE pin used to connect to external MSPI MRAM Recovery device. (Default = 0x00000000)')

    parser.add_argument('--nv3', dest='NV_CONFIG3_CONFIG3', type=auto_int,
                        help='MRAM NV Config 3 (Default = 0x00000000)')

    parser.add_argument('--nvCEPin', dest='NV_PIN_NUMS_CE_PIN', type=auto_int,
                        help='The pin number for the CE pin used to connect to external MSPI MRAM Recovery device. (Default = 0x00000000)')

    parser.add_argument('--otp', dest = 'otp', type=auto_int, choices=[0, 1],
                        help = 'info0 source. 0 = MRAM, 1 = OTP (Default = 0x0)')

    parser.add_argument('--sn', dest="jlinkSerialNum",
                        help='Serial number of the jlink debugging device. (Default = None)')

    parser.add_argument('--ldrCfg', dest='loaderCfg', type=str,
                        help='Relative path to recovery loader configuration file. (Default = None)')

    parser.add_argument('--info0Path', dest='info0Path', type=str,
                        help='Relative path to INFO0 Binary file. (Default = None)')

    parser.add_argument('--info0Cfg', dest='info0Cfg', type=str,
                        help='Relative path to INFO0 configuration file. (Default = None)')

    parser.add_argument('--regenInfo', dest='regenInfo', type=auto_int, choices=[0x0, 0x1],
                        help='When enabled, create a info0 binary using the settings from the supplied info0 cfg. (Default = 0x0)')

    parser.add_argument('--loadInfo', dest='loadInfo', type=auto_int, choices=[0x0, 0x1],
                        help='When enabled, Info0 binary will be loaded to the device. (Default = None)')

    parser.add_argument('--loglevel', dest='loglevel', type=auto_int, default=AM_PRINT_LEVEL_INFO,
                        choices = range(AM_PRINT_LEVEL_MIN, AM_PRINT_LEVEL_MAX+1),
                        help=helpPrintLevel)

    return parser

def compare_configs(file1, file2, section='DEFAULT'):

        config1 = configparser.ConfigParser()
        config1.optionxform = str
        config1.read(file1)

        config2 = configparser.ConfigParser()
        config2.optionxform = str
        config2.read(file2)

        for key,value in config1.items(section):
            if config2.has_option(section, key):
                if auto_int(config1.get(section, key)) != auto_int(config2.get(section, key)):
                    raise Exception(f'Error - the following key/value pair are different among loader config and info config:\n{key} - {value}')

def parse_config(configFile, defaults, section='DEFAULT'):
    if not os.path.exists(configFile):
        raise FileNotFoundError(f"File '{configFile}' not found.")

    config = configparser.ConfigParser()

    # Preserve capitalization in config file
    config.optionxform = str
    config.read(configFile)

    for key, value in config.items(section):
        try:
            defaults[key] = auto_int(value)
        except:
            defaults[key] = value

class LoaderUtility():
    def __init__(self):
        # self.jlink = self.connect_jlink()
        self.jlinkSerialNum = None
        self.jlink = None

        # register index 15 and 17 correspond to ARM's PC and MSP register respectivly
        self.pcRegIdx = 15
        self.mspRegIdx = 17

        # location of the recovery information structure (defined in loader src file)
        self.loaderBaseAddress = 0x00000000

        self.amRcvyImage = None
        self.amRcvyImageAddr = 0x00000000
        self.amRcvyImageOff = 0x00000000

        self.oemRcvyImage = None
        self.oemRcvyImageAddr = 0x00000000
        self.oemRcvyImageOff = 0x00000000

        self.loaderApp = None
        self.loaderAppAddr = 0x00000000

        self.readWrite = 0x0

        self.recoveryType = 0x0
        self.metaOffset = 0x00000000
        self.nvModuleNum = 0x0

        self.mspiChipSel = 0x0

        self.info0Path = None
        self.regenInfo0 = 0x0
        self.loadInfo0 = 0x0
        self.info0Cfg = None

        self.loaderCfg = None

    def connect_jlink(self):
        jlink = pylink.JLink()

        if self.jlink:
            return self.jlink

        # Open the connection to a specific jlink device if given an serial number.
        if self.jlinkSerialNum:
            jlink.open(self.jlinkSerialNum)
        else:
            jlink.open()

        jlink.set_tif(pylink.enums.JLinkInterfaces.SWD)

        jlink.connect('AP510NFA-CBR', verbose=True)

        if jlink.connected() is False:
            sys.exit(-1)

        return jlink

    def apply_defaults(self, **defaults):
        self.jlinkSerialNum = defaults['jlinkSerialNum']
        self.loaderBaseAddress = defaults['rcvyStructAddress']

        self.amRcvyImage = defaults['ambiqRcvyImg']
        self.amRcvyImageAddr = defaults['ambiqRcvyImgAddress']
        self.amRcvyImageOff = defaults['ambiqRcvyImgOffset']

        self.oemRcvyImage = defaults['oemRcvyImg']
        self.oemRcvyImageAddr = defaults['oemRcvyImgAddress']
        self.oemRcvyImageOff = defaults['oemRcvyImgOffset']

        self.loaderApp = defaults['loaderApp']
        self.loaderAppAddr = defaults['loaderAppAddress']

        self.readWrite = defaults['readWrite']

        self.recoveryType = defaults['MRAM_RCVY_CTRL_NV_RCVY_TYPE']
        self.metaOffset = defaults['NV_METADATA_OFFSET_META_OFFSET']
        self.nvModuleNum = defaults['MRAM_RCVY_CTRL_NV_MODULE_NUM']

        if self.recoveryType == 0x1:
            self.scrambling = defaults['NV_CONFIG3_CONFIG3']
            self.ce_pin = defaults['NV_PIN_NUMS_CE_PIN']
            self.transmissionMode = defaults['NV_CONFIG0_CONFIG0']

            # Hard fix, need to create a device table for different mappings from ce pin number -> chip select for diff boards
            if self.ce_pin == 0x24:
                self.mspiChipSel = 0
            elif self.ce_pin == 0x35:
                self.mspiChipSel = 0

        elif self.recoveryType == 0x2:
            self.partition = defaults['MRAM_RCVY_CTRL_EMMC_PARTITION']

        self.info0Path = defaults['info0Path']
        self.regenInfo0 = defaults['regenInfo0']
        self.loadInfo0 = defaults['loadInfo0']
        self.info0Cfg = defaults['info0Cfg']

        self.loaderCfg = defaults['loaderCfg']

    def write_info0(self, info0Binary, otp):
        self.jlink = self.connect_jlink()
        self.jlink.reset(ms=50, halt=True)
        self.jlink.restart()
        self.jlink.reset(ms=50, halt=True)
        self.jlink.restart()
        self.jlink.halt()

        if otp == True:
            # Retrieve confirmation to program Info0 OTP
            confirmation = input("Warning - Info0 OTP is enabled, are you sure you want to program info0 OTP? (y/n)\n")
            if confirmation.lower() == "y":
                pcVal = 0x0200ff59
                numWords = 0x00000040
            else:
                sys.exit(1)
        else:
            pcVal = 0x0200ff15
            numWords = 0x00000200

        self.jlink.flash_file(info0Binary, 0x20081000)
        self.jlink.register_write(self.mspRegIdx, 0x20080100)
        self.jlink.register_write(self.pcRegIdx, pcVal)

        self.jlink.memory_write32(0x20080000, [0x00000000, numWords, 0xD894E09E, 0xFFFFFFFF])
        self.jlink.restart()
        time.sleep(4)

    def dump_recovery_struct_data(self):
        # Check the status
        res = self.jlink.memory_read32(self.loaderBaseAddress, 15)

        print("\nSuccessfully loaded recovery data into NV.\n")

        offset = 0x0
        wordCnt = 0
        for val in res:
            print(f"WD{wordCnt} @ {hex(self.loaderBaseAddress + offset)} -> {hex(val)}")
            wordCnt += 1
            offset += 0x4

    def write_mspi_image(self, readWrite, amImageStartBlock, amImageAddr, amImageFile, oemImageStartBlock, oemRcvyImageAddr, oemRcvyImageFile, scrambling, mspiChipSel, mspiDeviceNum, metadataFlashAddr, transmissionMode, loaderAppAddr, loaderApp):
        self.jlink = self.connect_jlink()
        self.jlink.reset(ms=5000, halt=True)

        # Signature
        self.jlink.memory_write32(self.loaderBaseAddress, [0xABCD1234])

        # Read/Write mode - 0x0 = read/verify only, None-Zero = for read/write/verify
        self.jlink.memory_write32(self.loaderBaseAddress + 0x4, [readWrite])

        # Set the Ambiq recovery image blk offset into flash
        self.jlink.memory_write32(self.loaderBaseAddress + 0x8, [amImageStartBlock])

        # Set the Ambiq recovery image size
        self.jlink.memory_write32(self.loaderBaseAddress + 0xc, [os.path.getsize(amImageFile)])

        # Set the OEM recovery image blk offset into flash
        self.jlink.memory_write32(self.loaderBaseAddress + 0x10, [oemImageStartBlock])

        # Set the OEM recovery image size
        self.jlink.memory_write32(self.loaderBaseAddress + 0x14, [os.path.getsize(oemRcvyImageFile)])

        # Set the load address for the Ambiq recovery image
        self.jlink.memory_write32(self.loaderBaseAddress + 0x18, [amImageAddr])

        # Set the load address for the OEM recovery image
        self.jlink.memory_write32(self.loaderBaseAddress + 0x1C, [oemRcvyImageAddr])

        # Clear result registers with improbable values
        self.jlink.memory_write32(self.loaderBaseAddress + 0x20, [0xFFFFFFFF])
        self.jlink.memory_write32(self.loaderBaseAddress + 0x24, [0xFFFFFFFF])
        self.jlink.memory_write32(self.loaderBaseAddress + 0x28, [0xFFFFFFFF])

        # Set the recovery type device selection to MSPI (0x0 for MSPI)
        self.jlink.memory_write32(self.loaderBaseAddress + 0x2C, [0x0])

        # Set scrambling (0x0 for disable, 0x1 for enable)
        # Configure chip select (0x0 for CE0, 0x1 for CE1)
        # Set the MSPI Device Number (0x0 - 0x3)
        if (scrambling == True):
            scrambling_byte = 1
        else:
            scrambling_byte = 0
        scramblingData = (scrambling_byte << 16) | (mspiChipSel << 8) | mspiDeviceNum
        self.jlink.memory_write32(self.loaderBaseAddress + 0x30, [scramblingData])

        # Set the offset into flash for the metadata
        self.jlink.memory_write32(self.loaderBaseAddress + 0x34, [metadataFlashAddr])

        #Set the transmission mode for the MSPI device
        self.jlink.memory_write32(self.loaderBaseAddress + 0x38, [transmissionMode])

        self.jlink.flash_file(amImageFile, amImageAddr)
        self.jlink.flash_file(oemRcvyImageFile, oemRcvyImageAddr)
        self.jlink.flash_file(loaderApp, loaderAppAddr)

        # Set the appropriate PC per the binary of the utility
        with open(loaderApp, "rb") as f:
            data = f.read(8)

        if len(data) < 8:
            raise ValueError(f"{loaderApp} is either empty or too small (file < 2 words).")

        # Setup to execute the loader application
        firstWord = word_from_bytes(data, 0)
        self.jlink.register_write(self.mspRegIdx, firstWord)

        secondWord = word_from_bytes(data, 4)
        self.jlink.register_write(self.pcRegIdx, secondWord)

        # Execute the loader and wait for the completion breakpoint with timeout.
        self.jlink.restart()
        time.sleep(1)

        # Print the data written to the loader struct location
        self.dump_recovery_struct_data()

        metadata_write_errcount = self.jlink.memory_read32(self.loaderBaseAddress + 0x20, 1)[0]
        am_image_write_errcount = self.jlink.memory_read32(self.loaderBaseAddress + 0x24, 1)[0]
        oem_image_write_errcount = self.jlink.memory_read32(self.loaderBaseAddress + 0x28, 1)[0]
        self.jlink.close()

    def write_emmc_image(self, readWrite, amImageStartBlock, amImageAddr, amImageFile, oemImageStartBlock, oemRcvyImageAddr, oemRcvyImageFile, metadataStartBlock, nvModuleNum, emmcPartition, loaderAppAddr, loaderApp):
            ''' Writes and verifies an Ambiq and OEM recovery OTA blobs to an EMMC device
            Populates the eMMC metadata structure with the OTA blob sizes and offsets for each image.
            Relies on the loader application found in boards/common5/examples/mram_recovery/image_loader.

            :param amImageStartBlock (int): Offset block in the eMMC device for the Ambiq recovery image
            :param amImageAddr (int): Load address for the Ambiq recovery image
            :param amImageFile (str): Filename of the Ambiq recovery image.
            :param oemImageStartBlock (int): Offset block in the eMMC device for the OEM recovery image
            :param oemRcvyImageAddr (int): Load address for the OEM recovery image
            :param oemRcvyImageFile (str): Filename of the OEM recovery image.
            :param metadataStartBlock (int): Offset block in the eMMC device for
            the metadata structure. Must not intersect with the OTA payload
            :param nvModuleNum (int): eMMC SDIO controller (bus) number for the device.
            :param emmcPartition (int): eMMC device partition - 0x0=USER, 0x1=BOOT1, 0x2=BOOT2
            :param loaderAppAddr (int):
            :param loaderApp (str): Filename of the Ambiq recovery image.
            '''
            self.jlink = self.connect_jlink()
            self.jlink.reset(ms=5000, halt=True)

            if nvModuleNum > 1:
                raise Exception('eMMC bus must be 0-1. Was {}'.format(nvModuleNum), True)
            if emmcPartition > 2:
                raise Exception('eMMC partition must be 0-2, where 0 is USER, 1 is BOOT1, and 2 is BOOT2. Was {}'.format(emmcPartition), True)

            # Signature
            self.jlink.memory_write32(self.loaderBaseAddress + 0x0, [0xABCD1234])

            # Read/Write mode - 0x0 = read/verify only, None-Zero = for read/write/verify
            self.jlink.memory_write32(self.loaderBaseAddress + 0x4, [readWrite])

            # Set the Ambiq recovery image blk offset into flash
            self.jlink.memory_write32(self.loaderBaseAddress + 0x8, [amImageStartBlock])

            # Set the Ambiq recovery image size
            self.jlink.memory_write32(self.loaderBaseAddress + 0xC, [os.path.getsize(amImageFile)])

            # Set the OEM recovery image blk offset into flash
            self.jlink.memory_write32(self.loaderBaseAddress + 0x10, [oemImageStartBlock])

            # Set the OEM recovery image size
            self.jlink.memory_write32(self.loaderBaseAddress + 0x14, [os.path.getsize(oemRcvyImageFile)])

            # Set the load address for the Ambiq recovery image
            self.jlink.memory_write32(self.loaderBaseAddress + 0x18, [amImageAddr])

            # Set the load address for the OEM recovery image
            self.jlink.memory_write32(self.loaderBaseAddress + 0x1C, [oemRcvyImageAddr])

            # Clear result registers with improbable values
            self.jlink.memory_write32(self.loaderBaseAddress + 0x20, [0xFFFFFFFF])
            self.jlink.memory_write32(self.loaderBaseAddress + 0x24, [0xFFFFFFFF])
            self.jlink.memory_write32(self.loaderBaseAddress + 0x28, [0xFFFFFFFF])

            # Set the recovery type device selection to eMMC (0x1 for eMMC)
            self.jlink.memory_write32(self.loaderBaseAddress + 0x2C, [0x1])

            # Set the block number of the Metadata address in the Flash
            self.jlink.memory_write32(self.loaderBaseAddress + 0x30, [metadataStartBlock])

            # Set the eMMC partition to target
            self.jlink.memory_write32(self.loaderBaseAddress + 0x34, [emmcPartition])

            # Set the SDIO Device Number (0x0 = SDIO0, 0x1 = SDIO1)
            self.jlink.memory_write32(self.loaderBaseAddress + 0x38, [nvModuleNum])

            self.jlink.flash_file(amImageFile, amImageAddr)
            self.jlink.flash_file(oemRcvyImageFile, oemRcvyImageAddr)
            self.jlink.flash_file(loaderApp, loaderAppAddr)

            # Set the appropriate PC per the binary of the utility
            with open(loaderApp, "rb") as f:
                data = f.read(8)

            if len(data) < 8:
                raise ValueError(f"{loaderApp} is either empty or too small (file < 2 words).")

            # Setup to execute the loader application
            firstWord = word_from_bytes(data, 0)
            self.jlink.register_write(self.mspRegIdx, firstWord)

            secondWord = word_from_bytes(data, 4)
            self.jlink.register_write(self.pcRegIdx, secondWord)

            # Execute the loader and wait for the completion breakpoint with timeout
            self.jlink.restart()
            time.sleep(1)

            # Print the data written to the loader struct location
            self.dump_recovery_struct_data()

            self.jlink.close()

    def write_nv_device(self):
        # Program MSPI image
        if self.recoveryType == 0x1:
            self.write_mspi_image(self.readWrite, self.amRcvyImageOff, self.amRcvyImageAddr, self.amRcvyImage, self.oemRcvyImageOff, self.oemRcvyImageAddr, self.oemRcvyImage, self.scrambling, self.mspiChipSel, self.nvModuleNum, self.metaOffset, self.transmissionMode, self.loaderAppAddr, self.loaderApp)

        # Program EMMC image
        elif self.recoveryType == 0x2:
            self.write_emmc_image(self.readWrite, self.amRcvyImageOff, self.amRcvyImageAddr, self.amRcvyImage, self.oemRcvyImageOff, self.oemRcvyImageAddr, self.oemRcvyImage, self.metaOffset, self.nvModuleNum, self.partition, self.loaderAppAddr, self.loaderApp)
        else:
            raise Exception(f"Invalid selection: {self.recoveryType}, please enter one of the following valid options: 1 = MSPI, 2 = EMMC ")

if __name__ == '__main__':
    main()