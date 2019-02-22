#!/bin/bash
# Peter Yang
# Copyright (c) 2019 Seeed Studio
#
# MIT License
#

if [[ $EUID -ne 0 ]]; then
	echo "This script must be run as root (use sudo)" 1>&2
	exit 1
fi

function platform_get() {
	local dts_model platform

	dts_model=$(strings /proc/device-tree/model)

	case "$dts_model" in
	TI\ AM335x*)
		platform=bbb;;
	Raspberry\ Pi*)
		platform=rpi;;
	*)
		platform="unknown";;
	esac
	echo $platform
}

_apt_updated=false
function apt_install() {
	local pkg=$1 status _pkg

	status=$(dpkg -l $pkg | tail -1)
	_pkg=$(  echo "$status" | awk '{ printf "%s", $2; }')
	status=$(echo "$status" | awk '{ printf "%s", $1; }')
	# echo $status $_pkg $pkg

	if [ "X$status" == "Xii" -a "X$_pkg" == "X$pkg" ]; then
		echo "debian package $pkg already installed."
		return 1
	fi

	# install the debian package
	if [ "X$_apt_updated" == "Xfalse" ]; then
		apt-get -y update
		_apt_updated=true
	fi
	apt-get -y install $pkg
	return 0
}


platform=$(platform_get)
uname_r=$(uname -r)

case "$platform" in
bbb)
	apt_install linux-headers-$uname_r
	;;
rpi)
	# if headers be updated, kernel also update for adapting.
	apt_install raspberrypi-kernel-headers
	apt_install raspberrypi-kernel
	;;
*)
	echo "unsupport platform $platform, abort ..."
	exit 1
	;;
esac

echo
make
make install

echo "#######################################################"
echo "     Grove Linux Driver installation complete     !!!!!"
echo "#######################################################"
exit 0

