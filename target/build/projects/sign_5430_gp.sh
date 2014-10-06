#!/bin/sh

if [ -z $1 ]; then
	echo "Error missing target name"
else
	result="../../host/Targets/2nd-Downloaders/$1.2nd"
	# for now just assume Linux or Windows. You can always set OS type
	# by passing OS_TYPE to makefile as argument
	OS_TYPE=`uname`
	if [ $OS_TYPE = 'Linux' ]; then
		omaptool="omapfileconvert"
	else
		omaptool="omapfileconvert.exe"
	fi

	cd ../../../host/omapfileconvert

	if [ -e $result ]; then
		rm $result
	fi

	./$omaptool -i ../../_out_/bin/omap5/Debug/$1.out -o ../../_out_/bin/omap5/Debug/$1.raw
	./$omaptool -imgoffset 0 -i ../../_out_/bin/omap5/Debug/$1.out -o $result
fi
