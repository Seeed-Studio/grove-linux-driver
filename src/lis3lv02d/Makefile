#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

_tgt     := lis331dlh-i2c

_uname_r := $(shell uname -r)
_BUILD   := /lib/modules/$(_uname_r)/build
_PWD     := $(shell pwd)

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language
ifneq ($(KERNELRELEASE),)
lis331dlh-i2c-objs := lis3lv02d.o lis3lv02d_i2c.o
obj-m += ${_tgt}.o

else
_class  := drivers/misc

DEST  := /lib/modules/$(_uname_r)/kernel

all:
	make -C ${_BUILD} M=$(_PWD) modules

clean:
	make -C ${_BUILD} M=$(_PWD) clean

install:
	cp ${_tgt}.ko ${DEST}/${_class}/
	depmod -a

uninstall:
	rm -f ${DEST}/${_class}/${_tgt}.ko
	depmod -a

.PHONY: all clean install uninstall
endif
