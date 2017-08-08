#
# This is the spec file for the Digi RealPort (dgrp) Linux Driver
#

Summary:	The Digi RealPort (dgrp) driver package for Linux 
Name:		dgrp
Version:	1.9
Release:	36
License:	GPL
Group:		System/Utilities
Source:		dgrp-1.9.tgz
# the URL should point to the documentation for the package
URL:		TBA
Distribution:	Digi RealPort (dgrp)
Vendor:		Digi International
Packager:	Scott H Kilau <scottk at digi dot com>
#Requires:
BuildRoot:	/var/tmp/%{name}-buildroot

%description
40002086_X6P	- driver source package
93000359_X6P	- release notes (/usr/doc/dgrp-1.9/release_notes.txt)
92000582_A	- software manual

History:
v0.1r1  - first release
v0.1r2  - add the port status to the gui
v0.1r3  - auto init on system startup
v0.1r4  - cleaned up port status window 
v0.1r5  - fixed a bug causing a tight loop when using PPP
        - modified send_break to set minimum break time to 250
v0.1r6  - major code cleanup
        - final changes to location of driver module
v1.0r0  - first official, external Digi release
v1.1r0  - Digi internal build, testing 2.4 compatibility
v1.1r1  - Digi internal build, adding multiple distribution support
v1.1r2  - Digi internal build, better handling for RedHat 7
v1.1r3  - Digi internal build, better RPM file handling
        - bug fix to the GUI tool to actually display the DCD signal
v1.1r4  - Digi internal build, new default port settings
        - making all port settings sticky, like built-in COM ports
        - making CLOCAL a default cflag, like built-in COM ports
        - fixing configuration script cleanup to remove device files
v1.1r5  - Digi internal build, dropping CLOCAL will now no longer
          cause a hangup if the actual DCD line is already low...
          only PHYSICAL line drops will cause hangups.
v1.1r6  - Digi internal build, driver should now install the startup
          scripts in /etc/init.d if /etc/rc.d/init.d doesn't exist.
        - The RPM installation won't fail if "chkconfig" doesn't exist,
          it will simply note to users that the installation scripts
          need to be installed by hand.
        - A GUI bug (with newer tcl interpreters) was fixed (display
          ports stopped working with a TCL error)
        - Module permissions on install are changed to remove the
          "execute" bits in order to avoid RPM stripping the driver
          binary (which causes various errors)
v1.1r7  - Public release of the 1.1 stream, including release note
          modifications to note which distributions the driver was
          tested against.
v1.2r0  - Digi internal build, initial support for PSIII and PSLite
          is now advertised.
v1.2r1  - Digi internal build, fixes OPOST (much of the processing
          processing was inadvertently being duplicated in the PortServer)
v1.2r2  - Official release, including support for Digi One, EtherLite,
          and PortServer products.
        - Fixes supplied for widespread memory corruption and network
          buffer overflows.
        - Added new informational "proc" file "/proc/dgrp/nodeinfo"
        - Added arbitrary integer baud rate support
v1.3r0  - Official release, including full support for RealPort
          on EtherLite, including the EtherLite 2, EtherLite 160,
          EtherLite 162, and the EtherLite 80.
        - Fixed a kernel panic caused by port accesses after the
          network daemon has died.
        - Fixed a select(2) problem in kernel 2.2.15 and later of
          2.2.x kernels which made programs like telnet or ssh appear
          to hang until a key was pressed.
        - Fixed a data loss problem as a result of improper handling of
          the data if exactly 4k were transmitted just after a port was
          opened.
        - Added /lib/modules/`uname -r`/kernel/drivers/char to the list of
          locations that the kit will test for existence when trying to
          find an appropriate location to install the driver module,
          in response to testing in native 2.4 kernel-based distributions.
v1.3r1  - Unofficial release... initial test candidate including
          the alt_fail_open global driver parameter, which conditionally
          allows the "open" routine to call close on a failed open...
          necessary in kernels with a patch which prevents driver closes
          from being called if an open fails.
        - The "standard" Digi utility to configure the driver,
          dgrp_cfg_node, will now setup alt_fail_open on behalf of the
          user if the file /usr/bin/dgrp/config/alt_fail_open exists.
v1.3r2  - Unofficial release...
        - Modified daemon to include new switch (-n) which prevents
          the drpd program from becoming a daemon
        - Modified driver to prevent port "hangup" in the even that the
          daemon dies
v1.3r3  - Unofficial release...
        - Added TCFLSH handling to "ditty-rp"... previously, the "flushin",
          "flushout", and "flush" options didn't work.
        - Fixed a bug in the modifications from 1.3r2 which prevent
          drpd from becoming a daemon
v1.3r4  - Unofficial release...
        - Added "dgelreset" in order to allow customers to directly
          reset EtherLite units regardless of FAS or RealPort firmware,
          and without having to specify a MAC address.
        - Modified "dgipserv" to be able to reset RealPort units also.
        - Specifically mention the EL-8, EL-16 and EL-32 as supported.
v1.3r5  - Unofficial release...
        - Modified "dgelreset" to accept a parameter which allows one
          to ONLY attempt one reset method.
        - Added MODULE_LICENSE("GPL") to the code, to comply with the
          trend to avoid "tainting" in newer 2.4 kernels.
v1.3r6  - Beta release...
        - Modified descriptions to note E1P as the part number, and
          1.3-6 as the version.
v1.4r0  - Official release of previous beta. No new changes.
v1.4r2  - Unofficial release...
        - Fixed off-by-one error.
v1.4r3  - Unofficial release
        - Added Transparent print support.
v1.5r0  - Official release.
        - Added support for Redhat 7.3, Suse 8.0, Mandrake 8.2,
          OpenLinux 3.1.1 (server and workstation)
        - Changed rpm build procedure to run "configure" script
          first, to ensure the user's environment is correct.
v1.6r2  - Official release.
	- Added support for beta 2.6.0 Linux Kernels.
	- Expanded "configure" script.


%prep


%setup

if [ "$RPM_BUILD_ROOT" = "/" ]
then
	echo This package requires a seperate BuildRoot to build properly.
	exit 1
fi


%configure \
	%{?DISTRO:DISTRO%DISTRO}	\
	%{?DISTR:DISTR%DISTR}		\
	%{?DIST:DIST%DIST}		\
	%{?mandir:--mandir%mandir}	\
	%{?NEW_TTY_LOCKING:NEW_TTY_LOCKING%NEW_TTY_LOCKING} \
	%{?UDEV_SUPPORT:UDEV_SUPPORT%UDEV_SUPPORT}	\
	%{?NEW_TTY_BUFFERING:NEW_TTY_BUFFERING%NEW_TTY_BUFFERING}


%build
make all


%clean
if [ "$RPM_BUILD_ROOT" != "/" ]
then
	rm -rf $RPM_BUILD_ROOT
else
	make clean
fi


%install
make install


%post
	if [ -d /usr/bin/dgrp/config -a -f /usr/bin/dgrp/config/postinstall ]
	then
		/usr/bin/dgrp/config/postinstall
	fi


%preun
	if [ -d /usr/bin/dgrp/config -a -f /usr/bin/dgrp/config/preuninstall ]
	then
		/usr/bin/dgrp/config/preuninstall
	fi


%postun
	# Force a depmod after we have been removed.
	if [ -e /sbin/depmod ]
	then
		/sbin/depmod -a
	fi


%files
# this should be a list of files contained in the package
#
# this file gets stored in the RPM/docs directory 
#
%doc README
%doc COPYING
%doc release_notes.txt

#
# directories potentially created and used by the kit
#
%dir /usr/bin/dgrp
%dir /usr/bin/dgrp/daemon
%dir /usr/bin/dgrp/config

#
# this is all the binaries:
#	ditty-rp, the daemon, and the driver module
#
/usr/bin/ditty-rp
/usr/bin/dinc.dgrp
/usr/bin/dpa.dgrp
/usr/bin/dgrp/daemon/drpd
/usr/bin/dgrp_udev

# Could be .o or .ko
/lib/modules/*/misc/dgrp.*

#/tmp/10-dgrp.rules
/tmp/dgrp/10-dgrp.rules

#
# these are all the "system" binaries:
#	dgipserv
#	dgelreset
#
/usr/sbin/dgipserv
/usr/sbin/dgelreset
# 
# these are the man pages
# Could be .gz or .bz
#
/usr/*/man/man1/ditty-rp.1*
/usr/*/man/man1/dgipserv.1*
/usr/*/man/man1/dgelreset.1*
/usr/*/man/man8/dgrp.8*
/usr/*/man/man8/drpd.8*
/usr/*/man/man8/dgrp_cfg_node.8*
/usr/*/man/man8/dgrp_gui.8*

#/usr/*/man/man1/*
#/usr/*/man/man8/*

#/usr/man/man1/*
#/usr/man/man8/*

# 
# these are the PortServer configuration utilities
#
/usr/bin/dgrp/config/dgrp_cfg_node
/usr/bin/dgrp/config/dgrp_mk_spec
/usr/bin/dgrp/config/dgrp_mk_specs
/usr/bin/dgrp/config/dgrp_gui
/usr/bin/dgrp/config/dgrp.gif
/usr/bin/dgrp/config/postinstall
/usr/bin/dgrp/config/preuninstall

#
# these are used for init on reboot
#
/usr/bin/dgrp/config/file_locations

#
# depending ono the system, the initialization scripts
# might be in /etc/rc.d/init.d or /etc/init.d.
# Package up our entire local /etc tree to capture
# them in either case.
#
/etc

