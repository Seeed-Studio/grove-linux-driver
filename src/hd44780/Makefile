#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# GPL License
#
uname_r := $(shell uname -r)

hd44780-objs := hd44780-i2c.o hd44780-dev.o

obj-m += hd44780.o

BUILD := /lib/modules/$(uname_r)/build
DEST  := /lib/modules/$(uname_r)/kernel

all:
	make -C ${BUILD} M=$(PWD) modules

clean:
	make -C ${BUILD} M=$(PWD) clean

install:
	cp hd44780.ko ${DEST}/drivers/misc/
	depmod -a

uninstall:
	rm -f ${DEST}/drivers/misc/hd44780.ko
	depmod -a

.PHONY: all clean install uninstall

