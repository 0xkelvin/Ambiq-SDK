#*******************************************************************************
#
# Simple Makefile to prepare binaries for AMOTA for APOLLO5.
#
#*******************************************************************************

TOOL_CHAIN?=gcc

APOLLO5_BOARD = apollo510_blue_eb
APOLLO5_SCRIPT = apollo510_scripts
APOLLO5_KEYS = apollo510_keys.py
UPDATEBIN_APOLLO5_BLUE =../../../boards/$(APOLLO5_BOARD)/examples/ble/ble_freertos_amota/$(TOOL_CHAIN)/bin/ble_freertos_amota.bin
APPBIN_APOLLO5_BLUE =../../../boards/$(APOLLO5_BOARD)/examples/ble/ble_freertos_amota/$(TOOL_CHAIN)/bin/ble_freertos_amota.bin

all: $(APPBIN_APOLLO5_BLUE) $(UPDATEBIN_APOLLO5_BLUE)
	# APOLLO5 Cordio
	cp $(APPBIN_APOLLO5_BLUE) starter_binary_APOLLO5_blue.bin
	cp -r ../../$(APOLLO5_SCRIPT)/oem_tools_pkg* oem_tools_pkg
	cp -r ../../$(APOLLO5_SCRIPT)/arm_utils* arm_utils
	cp ../../$(APOLLO5_SCRIPT)/am_defines.py am_defines.py
	cp ../../$(APOLLO5_SCRIPT)/$(APOLLO5_KEYS) $(APOLLO5_KEYS)
	cp ../../$(APOLLO5_SCRIPT)/create_cust_image_blob.py create_cust_image_blob.py
	cp ../../$(APOLLO5_SCRIPT)/key_table.py key_table.py
	cp ../../$(APOLLO5_SCRIPT)/keys.ini keys.ini
	python create_cust_image_blob.py -c firmware.ini
	python ota_binary_converter.py --appbin temp_binary_APOLLO5_blue.bin -o update_binary_APOLLO5_blue
	@rm -rf temp_binary_APOLLO5_blue.bin
	@rm -rf key_table.py
	@rm -rf keys.ini
	@rm -rf am_defines.py
	@rm -rf create_cust_image_blob.py
	@rm -rf oem_tools_pkg
	@rm -rf arm_utils
	@rm -rf $(APOLLO5_KEYS)

$(APPBIN_APOLLO5_BLUE):
	$(MAKE) -C ../../../boards/$(APOLLO5_BOARD)/examples/ble/ble_freertos_amota/$(TOOL_CHAIN)/ $(MAKECMDGOALS)

clean:
	rm -rf *.bin
	rm -rf $(APPBIN_APOLLO5_BLUE)
	