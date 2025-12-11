INPUT = $(wildcard *.bin)
CERT = $(INPUT:.bin=.cert)
SOTA = $(INPUT:.bin=.sota)
WIRED = $(INPUT:.bin=.wsota)

CERT_UTIL_PATH = ../../oem_tools_pkg/cert_utils/cert_gen_utils

all: $(SOTA) $(CERT) $(WIRED)

%.cert: %.bin am_oem_cnt_cert.cfg images_table_template.tbl
	sed "s/INPUT_FILE/$</" images_table_template.tbl > images_table.tbl
	python3 $(CERT_UTIL_PATH)/am_cert_content_util.py ./am_oem_cnt_cert.cfg
	cp $(CERT_UTIL_PATH)/am_cert_content_util_output/content_cert.bin $@

%.sota: %.bin %.cert secure-firmware.ini keys.ini
	python3 ../../create_cust_image_blob.py -c secure-firmware.ini --bin $< --certificate $(<:.bin=.cert) -o $@

%.wsota: %.sota wired_ota.ini
	python3 ../../create_cust_image_blob.py -c wired_ota.ini --bin $< -o $@

clean:
	rm -rf $(WIRED) $(SOTA) $(CERT) images_table.tbl *.log
