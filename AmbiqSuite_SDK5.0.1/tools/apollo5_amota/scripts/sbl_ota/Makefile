#*******************************************************************************
#
# Simple Makefile to prepare binaries for SBL OTA for Apollo5.
#
#*******************************************************************************

CURRENT_VER = sbl_ap5b
ENCRPTED_SBL0 = ../../../$(APOLLO5_SCRIPT)/sbl_updates/$(CURRENT_VER)/encrypted_sbl0.bin
SBL_OTA = ../../../$(APOLLO5_SCRIPT)/sbl_updates/$(CURRENT_VER)/sbl_ota.bin

all: $(ENCRPTED_SBL0) $(SBL_OTA)
	# Apollo5 Cordio
	cp $(ENCRPTED_SBL0) encrypted_sbl0.bin
	cp $(SBL_OTA) sbl_ota.bin
	cat encrypted_sbl0.bin sbl_ota.bin > temp_sbl_apollo5.bin
	python3 ota_binary_converter.py --appbin temp_sbl_apollo5.bin -o update_sbl_apollo5
	@rm -rf temp_sbl_apollo5.bin encrypted_sbl0.bin sbl_ota.bin

clean:
	rm -rf *.bin
	