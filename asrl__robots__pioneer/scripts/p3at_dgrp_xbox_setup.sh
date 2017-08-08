#!/bin/bash

# This script brings up the dgrp serial to ethernet converter
# and the Xbox360 gamepad
#
# Before running this script, check:
# 1. P3AT is on
# 2. ethernet connection to robot 
# 3. network configuration (ip address of the computer)
# 3. xbox360 gamepad receiver is connected, and gamepad is on
#
# Keith Leung 2012
#

if [ $# -eq 0 ]
then
	echo "usage: p3at_dgrp_xbox_setup [48, 49]"
	exit 1
fi
ip=("$@")
if [ ${ip[0]} = 48 ] || [ ${ip[0]} = 49 ]  
then
	echo "============================================"
	echo "Setting up P3AT DGRP (192.168.0.${ip[0]})..."	
else
	echo "Invalid ip address (192.168.0.${ip[0]})"	
	echo "Setup Aborted"	
	exit 2
fi

# Set up the pioneer
sudo $(rospack find digi_realport_serial_ethernet)/scripts/dgrp-script.sh start 2 192.168.0.${ip[0]}

sleep 3
sudo chmod a+rw /dev/tty200

echo "Setting up XBox360 gamepad..."
# Set up the gamepad
sudo rmmod xpad
sudo modprobe uinput
sudo modprobe joydev
sudo xboxdrv --silent &

sleep 3
echo "Setup Compelete"
echo "============================================"

exit 0