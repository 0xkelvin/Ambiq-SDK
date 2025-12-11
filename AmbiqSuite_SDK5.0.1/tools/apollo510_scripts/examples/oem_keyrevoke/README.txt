Oem_keyrevoke is an example  of scripts/config needed to build an ota to revoke oem keybank keys

Steps to build this OTA

1) Edit key_updates.ini and set the keybank keys that need to be revoked to 1.
2) Generate key_updates binary
	python3 keyupdates_binary_generator_icvkeys.py key_updates.ini
   This generates a binary called key_updates.bin which is used as input file in keyrevoke.ini
4) Create a keyrevoke OTA
	python3 ../../create_cust_image_blob.py -c keyrevoke.ini
   This creates keyrevoke OTA image called key_update_output.bin.
5) This can be programmed into the DUT using jlink-ota.txt script.


