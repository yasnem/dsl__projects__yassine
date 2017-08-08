#!/bin/bash
#

DEFAULT_TIMEOUT=20

if [ $# -eq 1 ] && [ $1 = "help" ] ; then
	echo "Usage: [./dgrp-script start dgrp_ID]"
	echo "       starts a virtual dgrp serial port"
	echo ""
	echo "       [./dgrp-script kill dgrp_ID]"
	echo "       removes a node"
	echo "       (perhaps try this if you have trouble setting up a dgrp port)"
	echo ""
	echo "       [./dgrp-script setup dgrp_ID]"
	echo "       changes the permission of the ports"
	exit
fi

if [ $# -eq 3 ] && [ $1 = "start" ]; then
	# This is equivalent to adding/configuring a digi node wit
	#	Realport ID: $2
	#	IP address: $3
	#	# of ports: 1
	/usr/bin/dgrp/config/dgrp_cfg_node init -v -v -e never $2 $3 1
	sleep 1
	# This starts the daemon on this node
	/usr/bin/dgrp/config/dgrp_cfg_node -v -v start $2 $3
elif [ $# -eq 3 ] && [ $1 = "kill" ]; then
	# this removes the node
	#	Realport ID: $2
	#	IP address: $3
	/usr/bin/dgrp/config/dgrp_cfg_node -v -v uninit $2 $3 1
elif [ $# -eq 2 ] && [ $1 = "setup" ]; then
	a=0
	success=0
	while [ $a -lt $DEFAULT_TIMEOUT ] && [ $success -eq 0 ];
	do
		sleep 1
		a=$(($a+1))
		if ! ls /dev/tty* | grep /dev/tty"$2"00 > /dev/null; then
			continue
		elif ! ls /dev/tty* | grep /dev/ttyACM0 > /dev/null; then
			continue
		elif ! ls /dev/tty* | grep /dev/ttyUSB0 > /dev/null; then
			continue
		else
			success=1
		fi
	done
	if [ $success -eq 0 ]; then
		echo "Try setting up the dgrp port again"
		exit
	fi

	# Used for laser and ptu
	chmod a+rw /dev/tty"$2"00 #/dev/ttyACM0 /dev/ttyUSB0
else
	./dgrp-script help
	exit
fi

echo "done"
