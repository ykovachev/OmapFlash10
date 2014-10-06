#!/bin/sh

if [ -z $1 ]; then
	echo "Error missing target name"
else
	result="../../../host/Targets/2nd-Downloaders/$1.2nd"
	# for now just assume Linux or Windows. You can always set OS type
	# by passing OS_TYPE to makefile as argument
	OS_TYPE=`uname`
	if [ $OS_TYPE = 'Linux' ]; then
		omaptool="omapfileconvert"
	else
		omaptool="omapfileconvert.exe"
	fi
	cd ../../../host/omapfileconvert
#	./$omaptool -signgp -config 3630/config/gpimage.xml -booting Peripheral -i ../../_out_/bin/omap3/Debug/$1.out -o ../Targets/2nd-Downloaders/$1.2nd
	./$omaptool -i ../../_out_/bin/omap3/Debug/$1.out -o ../../_out_/bin/omap3/Debug/$1.raw
	./$omaptool -i ../../_out_/bin/omap3/Debug/$1.out -o ../Targets/2nd-Downloaders/$1.2nd
fi
