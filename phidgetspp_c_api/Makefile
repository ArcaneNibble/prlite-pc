all:build


TARBALL     = build/libphidget_2.1.8.20110310.tar.gz
TARBALL_URL = http://www.phidgets.com/downloads/libraries/libphidget_2.1.8.20110310.tar.gz
MD5SUM_FILE = Phidgetlinux.md5sum
UNPACK_CMD  = tar xzf
SOURCE_DIR  = build/libphidget-2.1.8.20110310

include $(shell rospack find mk)/download_unpack_build.mk

build: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure
	make -C $(SOURCE_DIR)
	mkdir -p include 
	cp $(SOURCE_DIR)/phidget21.h include/
	mkdir -p lib
	cp $(SOURCE_DIR)/.libs/libphidget21.so lib/


clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build
