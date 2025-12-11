Steps of generating AMOTA file:

1. Compile the ble_freertos_amota project to generate the new binary that you want to upgrade for the Apollo4.

2. Go to SDK/tools/apollo5_amota/scripts folder, modify the items of TOOL_CHAIN, APOLLO5_BOARD, APOLLO5_SCRIPT and APOLLO5_KEYS in Makefile for your own development environment.
The default settings are:
TOOL_CHAIN?=gcc
APOLLO5_BOARD = apollo510_eb
APOLLO5_SCRIPT = apollo5a_scripts
APOLLO5_KEYS = apollo5a_keys.py

3. Go to SDK/tools/apollo5_amota/scripts/sbl_ota folder, modify the item CURRENT_VER in Makefile for your own development environment.

4. Execute gcc make under the path SDK/tools/apollo5_amota/scripts, then you will get the final update_binary_apollo5_blue.bin with OTA header for the AMOTA.