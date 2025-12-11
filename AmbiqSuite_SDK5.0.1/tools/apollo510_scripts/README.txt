Brief info on various files in this directory:
==============================================
Python Dependencies:
===================
-- AP510_tools_requirements.txt => Contains all Python package dependencies required for the Apollo510 development tools
        -- Install dependencies with: pip install -r AP510_tools_requirements.txt
        -- Includes packages for firmware updates, MRAM recovery, configuration generation, and other development utilities
        -- Ensures consistent Python environment setup across different development machines

Scripts/configs:
================
- helper scripts: These are used by other scripts
        - am_defines.py, apollo510_info0.py, apollo510_otp_info0.py, apollo510_keys.py, key_table.py
- create_info0.py => Should be used to generate INFO0
- create_cust_image_blob.py => Used to generate various OTA or wired images for Ambiq SBL
        - It requires keys.ini for keys information, and an operation specific ini file to generate the required image
- uart_wired_update.py => Used to interact with Ambiq SBL as part of Wired update
- sample: This folder contains various sample ini files. The required files should be copied to the parent directory and edited to be used with create_cust_image_blob.py
        - keys.ini => Determines location of key assets
        - wired.ini => Sample ini file to download a blob to a fixed location in the device
        - wired_ota.ini => Sample ini file to download a preconstructed OTA image to a temp place in device, and trigger OTA on reset
        - firmware.ini => Generate OTA image corresponding to a firmware update (Can be secure or non-secure)
        - oem_chain.ini => Generate OEM Cert chain update
- sbl_updates: This folder contains the SBL OTA images to do on field updates of Ambiq SBL
        - sbl_ota.bin
        - These files should be copied to tools/apollo510_scripts, to be used with supplied JLink scripts for SBL OTA
- oem_tools_package: This folder contains various key, asset and certificate generation utilities for provisioning and runtime usage
- socid.txt -- example input file for socid_to_bin.py script
- socid_to_bin.py -- script to convert socid.txt to socid.bin
- keybank_keys_generate.py -- example script to generate keybank keys.
- arm_utils -- arm utility scripts

Jlink scripts:
==============
- jlink-prog-info0.txt => Sample script to program INFO0 (info0.bin generated using create_info0.py)
- jlink-prog-info0_otp.txt => Sample script to program INFO0 OTP (info0_otp.bin generated using create_info0.py)
- jlink-ota.txt => Sample script to program OTA binary using SBL ( ota.bin generated using create_cust_image_blob.py)
- jlink-ota-bootrom.txt => Sample script to program OTA binary using bootrom ( ota.bin generated using create_cust_image_blob.py)

SBL OTA Notes:
==============
Sbl_updates folder contains sbl ota binaries if any. sbl_ota.bin will be provided by Ambiq. This can be programmed into the device using jlink-ota-bootrom.txt

