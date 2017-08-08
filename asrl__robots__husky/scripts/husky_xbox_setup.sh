#!/bin/bash

echo robots | sudo -S chmod a+rw /dev/ttyS0

# Set up the gamepad

echo robots | sudo -S rmmod xpad
echo robots | sudo -S sudo modprobe uinput
echo robots | sudo -S sudo modprobe joydev
echo robots | sudo -S sudo xboxdrv
