#!/bin/sh

if [ -z $1 ]; then
	echo "Error missing target name"
else
	result="../../host/Targets/2nd-Downloaders/$1"
	# for now just assume Linux or Windows. You can always set OS type
	# by passing OS_TYPE to makefile as argument
	OS_TYPE=`uname`
	if [ $OS_TYPE = 'Linux' ]; then
		omaptool="omapfileconvert"
		mshield="$HOME/mshield-dk"
	else
		omaptool="omapfileconvert.exe"
		mshield="c:/mshield-dk"
	fi

	cd ../../../host/omapfileconvert

	if [ -e $result.raw ]; then
		rm $result.raw
	fi

	if [ -e $result.signed.2nd ]; then
		rm $result.signed.2nd
	fi

#	./omapfileconvert -genraw -i ../../_out_/bin/omap3/Debug/$1.out -o ../../_out_/bin/omap3/Debug/$1.raw
	./$omaptool -i ../../_out_/bin/omap3/Debug/$1.out -o ../../_out_/bin/omap3/Debug/$1.raw
	cp ../../_out_/bin/omap3/Debug/$1.raw $result.raw
	if [ ! -e $result.raw ]; then
		echo $result.raw " file is not created."
		exit
	fi
	$mshield/generate_2nd 2ND omap3 v1 es1 sha1 $result.raw $result.signed.2nd
fi
