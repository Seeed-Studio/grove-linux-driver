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

uname_r=$(uname -r)

apt-get update
apt-get install linux-headers-$uname_r

