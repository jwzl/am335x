#!/bin/bash

export LANG=C
# Determine the absolute path to the executable
# EXE will have the PWD removed so we can concatenate with the PWD safely
PWD=`pwd`
EXE=`echo $0 | sed s=$PWD==`
EXEPATH="$PWD"/"$EXE"
MLO_PATH="$PWD"/../advloader/MLO
APP_PATH="$PWD"/../project/rtos/bin/debug/app

if [ "$1" = "" -o "$1" = "-h" ]; then
cat << EOM

################################################################################

This script will create a bootable SD card from custom or pre-built binaries.

The script must be run with root permissions and from the bin directory of
the SDK

Example:
 $./create-sdcard.sh path/to/device/node

Formatting can be skipped if the SD card is already formatted and
partitioned properly.

################################################################################

EOM
	exit 0;
fi

DEVEX=${1##*/}
echo $DEVEX
AMIROOT=`whoami | awk {'print $1'}`
if [ "$AMIROOT" != "root" ] ; then

	echo "	**** Error *** must run script with root"
	echo ""
	exit
fi

# find the avaible SD cards
ROOTDRIVE=
PARTITION_TEST=

check_for_sdcards()
{
	# find the avaible SD cards
	ROOTDRIVE=`mount | grep 'on / ' | awk {'print $1'} |  cut -c6-8`
	if [ "$ROOTDRIVE" = "root" ]; then
		ROOTDRIVE=`readlink /dev/root | cut -c1-3`
	else
		ROOTDRIVE=`echo $ROOTDRIVE | cut -c1-3`
	fi
  
	PARTITION_TEST=`cat /proc/partitions | grep -v $ROOTDRIVE | grep '\<sd.\>\|\<mmcblk.\>' | awk '{print $4}'`

	if [ "$PARTITION_TEST" = "" ]; then
		echo -e "Please insert a SD card to continue\n"
		while [ "$PARTITION_TEST" = "" ]; do
			read -p "Type 'y' to re-detect the SD card or 'n' to exit the script: " REPLY
			if [ "$REPLY" = 'n' ]; then
				exit 1
			fi

			PARTITION_TEST=`cat /proc/partitions | grep -v $ROOTDRIVE | grep '\<sd.\>\|\<mmcblk.\>' | awk '{print $4}'`
		done
	fi
}

print_avaliable_device() 
{
	echo -e "\nAvailable Drives to write images to: \n"
	echo "#  major   minor    size   name "
	cat /proc/partitions | grep -v $ROOTDRIVE | grep '\<sd.\>\|\<mmcblk.\>' | grep -n ''
	echo " "
}


# Check for available mounts
check_for_sdcards

DEVICEDRIVENAME=`echo $PARTITION_TEST | grep $DEVEX | awk '{print $1}'` 
if [ "$DEVICEDRIVENAME" = "" ];then
	print_avaliable_device
	exit 1;
fi 

echo "$DEVICEDRIVENAME was selected"

DRIVE=/dev/$DEVICEDRIVENAME
NUM_OF_DRIVES=`df | grep -c $DEVICEDRIVENAME`

echo "All data on "$DRIVE" now will be destroyed! Continue? [y/n]"
read ans
if [ $ans != 'y' ]; then exit 1; fi

# This if statement will determine if we have a mounted sdX or mmcblkX device.
# If it is mmcblkX, then we need to set an extra char in the partition names, 'p',
# to account for /dev/mmcblkXpY labled partitions.
if [[ ${DEVICEDRIVENAME} =~ ^sd. ]]; then
	echo "$DRIVE is an sdx device"
	P=''
else
	echo "$DRIVE is an mmcblkx device"
	P='p'
fi

if [ "$NUM_OF_DRIVES" != "0" ]; then
	for ((c=1; c<="$NUM_OF_DRIVES"; c++ ))
	do
		unmounted=`df | grep '\<'$DEVICEDRIVENAME$P$c'\>' | awk '{print $1}'`
		if [ -n "$unmounted" ]
		then
			echo " unmounted ${DRIVE}$P$c"
			umount -f ${DRIVE}$P$c
		fi
	done
fi

echo "[Partitioning $DRIVE...]"
## Clear partition table
dd if=/dev/zero of=$DRIVE bs=1024 count=1024

## create a 512 Mbyte partition. 
sfdisk  $DRIVE 1>/dev/null << EOF
,512M,0x0C,*
write
EOF

partprobe ${DRIVE}
sync
	
mkfs.vfat -I -F 32 -n "boot" ${DRIVE}${P}1  2>/dev/null

#Add directories for images
PATH_TO_SDBOOT=boot
mkdir -p $PATH_TO_SDBOOT

if ! mount -t vfat ${DRIVE}${P}1 $PATH_TO_SDBOOT/ 1> /dev/null; then
    echo  "Cannot mount ${DRIVE}${P}1"
    exit 1
fi

rm -rf  $PATH_TO_SDBOOT/*

#copy boot files out of board support
echo "[Copy MLO....]"
if [ -f $MLO_PATH ] ; then
	cp -a $MLO_PATH $PATH_TO_SDBOOT/
else
	echo "No MLO found, please check this"
	exit 1;
fi

echo "[Copy app....]"
if [ -f $APP_PATH ] ; then
	cp -a $APP_PATH $PATH_TO_SDBOOT/
else
	echo "No app found, please check this"
	exit 1;
fi

echo ""
echo ""
echo "Syncing..."
sync
sync
sync

umount -f $PATH_TO_SDBOOT
rm -rf $PATH_TO_SDBOOT


echo " "
echo "[Done]"
echo " "
