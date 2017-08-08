all: dgrp_serial_ethernet

SOURCE_DIR  = src/dgrp-1.9

dgrp_serial_ethernet: $(SOURCE_DIR)/installed


$(SOURCE_DIR)/installed:
	cd $(SOURCE_DIR) && sudo ./configure
	cd $(SOURCE_DIR) && sudo make
	cd $(SOURCE_DIR) && sudo make install
	cd $(SOURCE_DIR) && sudo make postinstall
	sudo chmod a+w /tmp
	touch $(SOURCE_DIR)/installed

clean:
	rm -f $(SOURCE_DIR)/installed
	cd $(SOURCE_DIR) && sudo make distclean

wipe: clean
