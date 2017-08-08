SOURCE_DIR = src/xboxdrv-linux-0.5.0_asrl

all:
	echo "blah"
	cd $(SOURCE_DIR) && make xboxdrv	
	sudo modprobe uinput
	cd $(SOURCE_DIR) && sudo make install	

clean:
	cd $(SOURCE_DIR) && make clean

