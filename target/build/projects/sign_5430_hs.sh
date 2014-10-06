#!/bin/sh

ifndef MSHIELD_HOME

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
	./$omaptool -i ../../_out_/bin/omap5/Debug/$1.out -o ../../_out_/bin/omap5/Debug/$1.raw
	if [ -e '../../_out_/bin/omap5/Debug/$1.raw' ]; then
		echo $1.raw " file is not created."
		exit
	fi

	if [ -e $result.* ]; then
		rm -f $result.*
	fi

	cp ../../_out_/bin/omap5/Debug/$1.raw $result.raw

	$mshield/generate_2nd ULO omap5 v1 es1 sha2 $result.raw "$result"_es1.s2.signed.2nd

fi
