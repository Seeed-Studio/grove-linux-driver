#
# Peter Yang <turmary@126.com>
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

_dts_model := $(shell strings /proc/device-tree/model)
_platform  := $(shell echo $(_dts_model) | sed -nr 's/TI AM335x.*/bbb/gp')
# _platform  := $(patsubst TI AM335x%, bbb, $(_dts_model))
# $(warning platform = $(_platform))

_DIRS      := $(wildcard src/*)
_DIRS      += dts/$(_platform)
_DEPMOD    := @:
export _DEPMOD

all      : _wrapper_all
clean    : _wrapper_clean
install  : _wrapper_install
uninstall: _wrapper_uninstall

_tgt = $(patsubst _wrapper_%,%, $(@))

_wrapper_%:
	@for i in ${_DIRS}; do					\
		if [ -f $${i}/Makefile ]; then			\
			make -C $${i} $(_tgt) || exit $$?;	\
		fi;						\
	done;
	@if [ "x$(_tgt)" = "xinstall" -o "x$(_tgt)" = "xuninstall" ]; then	\
		depmod -a;							\
	fi

%:
	@echo Available targets:
	@echo "  help"
	@echo "  all"
	@echo "  clean"
	@echo "  install"
	@echo "  uninstall"
	@echo "install/uninstall must be run as root (use sudo)"

FORCE:

.PHONY: first all clean install uninstall FORCE

