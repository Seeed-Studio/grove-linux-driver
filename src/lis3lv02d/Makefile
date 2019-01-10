#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#
uname_r := $(shell uname -r)

lis331dlh-i2c-objs := lis3lv02d.o lis3lv02d_i2c.o

obj-m += lis331dlh-i2c.o

BUILD := /lib/modules/$(uname_r)/build
DEST  := /lib/modules/$(uname_r)/kernel

all:
	make -C ${BUILD} M=$(PWD) modules

clean:
	make -C ${BUILD} M=$(PWD) clean

install:
	cp lis331dlh-i2c.ko ${DEST}/drivers/misc/
	depmod -a

uninstall:
	rm -f ${DEST}/drivers/misc/lis331dlh-i2c.ko
	depmod -a

.PHONY: all clean install uninstall

