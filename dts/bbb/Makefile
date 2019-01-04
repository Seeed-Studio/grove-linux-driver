# Peter Yang
# Copyright (c) 2019 Seeed Studio
#
# GPL License
#

DTBOS = BB-I2C2-LIS331DLH.dtbo

DEST  := /lib/firmware
SHELL := /bin/bash


%.dtbo : %.dts
	dtc -@ -I dts -O dtb $< -o $@


all: ${DTBOS}

clean:
	rm -f ${DTBOS}

install:
	@for i in ${DTBOS}; do			\
		cp $$i ${DEST}/;		\
		echo install ${DEST}/$$i;	\
	done
	# echo 'uboot_overlay_addr7=/lib/firmware/BB-I2C2-LIS331DLH.dtbo' >> /boot/uEnv.txt

uninstall:
	@for i in ${DTBOS}; do			\
		rm ${DEST}/$$i;			\
		echo uninstall ${DEST}/$$i;	\
	done

.PHONY: all clean install uninstall
