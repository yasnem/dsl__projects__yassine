#!/bin/bash

# Sets up the xbox gamepad.
sudo rmmod xpad
sudo modprobe uinput
sudo modprobe joydev
sudo xboxdrv -s
