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

#	./$omaptool -gench -omap4 -config 4430/config/chconfigomap4.xml
#	./$omaptool -signgp -config 4430/config/gpimage.xml -booting Peripheral -CHsectionName 4430/certificates/CHSECTION -i ../../_out_/bin/omap4/Debug/$1.out -o $result
#	./$omaptool -signgp -config 4430/config/gpimage.xml -booting Peripheral -i ../../_out_/bin/omap4/Debug/$1.out -o $result
	./$omaptool -i ../../_out_/bin/omap4/Debug/$1.out -o ../../_out_/bin/omap4/Debug/$1.raw
	./$omaptool -imgoffset 200 -i ../../_out_/bin/omap4/Debug/$1.out -o $result
fi
