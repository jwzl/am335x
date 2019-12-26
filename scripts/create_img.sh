#!/bin/bash

ABS_IMG=boot.img
PWD=`pwd`
EXE=`echo $0 | sed s=$PWD==`
EXEPATH="$PWD"/"$EXE"
MLO_PATH="$PWD"/../advloader/MLO
APP_PATH="$PWD"/../project/rtos/bin/debug/app


echo "generate fullsize Image"
## generate the image.
dd if=/dev/zero of=$ABS_IMG bs=512 count=2097152

LOOPDEVICE=$(sudo losetup -f)
losetup "$LOOPDEVICE" "$ABS_IMG"

sfdisk  $LOOPDEVICE 1>/dev/null << EOF
,1024M,0x0C,*
write
EOF

partprobe ${LOOPDEVICE}
sync

mkfs.vfat -I -F 32 -n "boot" ${LOOPDEVICE}p1  2>/dev/null

#mount to boot 
mkdir -p  boot
mount ${LOOPDEVICE}p1  boot/ 

rm -rf boot/*
echo "[Copy MLO....]"
if [ -f $MLO_PATH ] ; then
	cp -a $MLO_PATH boot/ 
else
	echo "No MLO found, please check this"
	exit 1;
fi

echo "[Copy app....]"
if [ -f $APP_PATH ] ; then
	cp -a $APP_PATH boot/ 
else
	echo "No app found, please check this"
	exit 1;
fi

umount boot/ 

rm -rf boot
losetup -d "$LOOPDEVICE"

echo; echo
echo -e "\t dd if=""$ABS_IMG"" of=/dev/sdXXX bs=512 -> to a >= 2GB SD/eMMC-Card"
echo; echo

exit 0

