#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#
uname_r := $(shell uname -r)
_class  := drivers/iio/humidity
_tgt    := bme280

obj-m += ${_tgt}.o

BUILD := /lib/modules/$(uname_r)/build
DEST  := /lib/modules/$(uname_r)/kernel

all:
	make -C ${BUILD} M=$(PWD) modules

clean:
	make -C ${BUILD} M=$(PWD) clean

install:
	cp ${_tgt}.ko ${DEST}/${_class}/
	depmod -a

uninstall:
	rm -f ${DEST}/${_class}/${_tgt}.ko
	depmod -a

.PHONY: all clean install uninstall

