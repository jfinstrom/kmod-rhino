#
# Makefile	Rhino PCI card drivers Makefile
#
# Copyright	(c) 2009, Rhino Equipment Corp.
#
#		This program is free software; you can redistribute it and/or
#		modify it under the terms of the GNU General Public License
#		version 2, as published by the Free Software Foundation.
# ----------------------------------------------------------------------------
# Author:  Bryce Chidester <brycec@rhinoequipment.com>
#

# Default zaptel directory to be overridden by user
ifndef ZAPDIR
ifeq "$(shell if [ -d /usr/src/zaptel/kernel ]; then echo yes; fi )" "yes"
	ZAPDIR=/usr/src/zaptel/kernel
else
	ZAPDIR=/usr/src/zaptel
endif
endif

# Kernel version and location
ifndef KVER
	KVER=$(shell uname -r)
endif
ifndef KMOD
	KMOD=/lib/modules/$(KVER)
endif
ifndef KINSTDIR
	KINSTDIR=$(KMOD)/kernel
endif
ifndef KSRC
	KSRC=$(KMOD)/build
endif
ifndef KINCLUDES
	KINCLUDES:=$(KSRC)/include
endif

# ARCH, for compiler
ifndef ARCH
	ARCH=$(shell uname -m)
endif

# Define which modules are to be built.
ifndef LEGACY_MODULES
	LEGACY_MODULES=r4fxo
endif
ifndef MODULES
	MODULES=r1t1 rxt1 rcbfx
endif

ifndef INSTALL_PREFIX
	INSTALL_PREFIX:=
endif

ifndef SKIPWAIT
	SKIPWAIT:=
endif

#ifdef FORCEZAP148
#	CFLAGS+=-DZT_GT_1471
#endif

KBUILD_NOPEDANTIC=1

ZAPMIN=$(shell modprobe zaptel; cat /sys/module/zaptel/version|cut -d- -f1|cut -d. -f3-)
ZAPMAJ=$(shell modprobe zaptel; cat /sys/module/zaptel/version|cut -d- -f1|cut -d. -f1-2)

#CFLAGS+=-DZT_GT_1471

PWD=$(shell pwd)
#Figure out the kernel version and sublevel
BUILDVER:=$(shell uname -r|cut -d. -f1-2)
SUBLEVEL:=$(shell head -n3 $(KSRC)/Makefile|tail -n1|awk '{print $$3}')

# Setup CFLAGS and co. (copied from Zaptel Makefile)
# Hotplug
ifeq ($(BUILDVER),2.6)
	HOTPLUG_FIRMWARE:=$(shell if grep CONFIG_FW_LOADER $(KINCLUDES)/linux/autoconf.h | grep -q undef; then echo "no"; else echo "yes"; fi)
	ifeq ($(HOTPLUG_FIRMWARE),yes)
		CFLAGS+=-DHOTPLUG_FIRMWARE
	endif
endif
# Includes and compiler directives
CFLAGS+=-I$(ZAPDIR) -O4 -g -Wall
CFLAGS+=$(shell if uname -m | grep -q x86_64; then echo "-m64"; fi)

# Kernel make directives
KFLAGS:=-I$(KINCLUDES) -O6
KFLAGS+=-DMODULE -D__KERNEL__ -DEXPORT_SYMTAB -I$(KSRC)/drivers/net -Wall -I. -I$(ZAPDIR) 
KFLAGS+=-Wstrict-prototypes -fomit-frame-pointer -I$(KSRC)/drivers/net/wan -I$(KINCLUDES)/net
ifneq (,$(wildcard $(KINCLUDES)/linux/modversions.h))
	KFLAGS+=-DMODVERSIONS -include $(KINCLUDES)/linux/modversions.h
endif

#Need this, trust me
KFLAGS+=-DSTANDALONE_ZAPATA
CFLAGS+=-DSTANDALONE_ZAPATA

# Set this to override hotplug firmware loading and revert to classic header based
#HOTPLUG_FIRMWARE=no
ifeq ($(HOTPLUG_FIRMWARE),yes)
  CFLAGS+=-DHOTPLUG_FIRMWARE
endif

ifneq ($(wildcard .svn),)
	RHINOPKGVER:=SVN-$(shell build_tools/make_svn_branch_name)
else
ifneq ($(wildcard .version),)
	RHINOPKGVER=$(shell cat .version)
endif
endif

#### Main body beings here ####
# Kernel's passing through, KERNELRELEASE is set, so just feed it the objects for it to build
ifneq ($(KERNELRELEASE),)

obj-m:=$(LEGACY_MODULES:%=%.o)
obj-m+=$(MODULES:%=%/)


# Called by user, so do user-y stuff
else
## Default target - checks headers and sources, then builds
all:   _checkzap _checksrc all_kmod


# To build the modules, check headers and sources, then also check the kernel is compatible
all_kmod:  _checkzap _checksrc _check_kver version.h
	@cp -f $(ZAPDIR)/Module*.symvers $(PWD)/ 2> /dev/null|| true
	$(MAKE) -C $(KSRC) SUBDIRS=$(PWD) modules
	@if [ ! -e $(ZAPDIR)/Module*.symvers ]; then \
		echo ; \
		echo "NOTICE: the above messages about undefined symbols are completely normal." ; \
		echo " they are simply a side-effect of the Zaptel kernel build dependency and" ; \
		echo " is absolutely harmless. Again, this is Hmmm .. OKAY." ; \
		echo " There is absolutely nothing wrong and no reason to panic."; \
		echo ; \
	fi


# Install alias to install the kernel modules
install: install_kmod install_modconf

# Install the kernel modules
install_kmod: all_kmod
	$(MAKE) -C $(KSRC) SUBDIRS=$(PWD) INSTALL_MOD_PATH=$(INSTALL_PREFIX) INSTALL_MOD_DIR=misc modules_install
	@if [ ! -e $(ZAPDIR)/Module*.symvers ]; then \
		echo ; \
		echo "NOTICE: the above messages about undefined symbols are completely normal." ; \
		echo " they are simply a side-effect of the Zaptel kernel build dependency and" ; \
		echo " is absolutely harmless. Again, this is Hmmm .. OKAY." ; \
		echo " There is absolutely nothing wrong and no reason to panic."; \
		echo ; \
	fi


# Pretty much just deleting everything to get back to the start.
clean:
	$(MAKE) -C $(KSRC) SUBDIRS=$(PWD) clean
	rm -f *.cmd *.o *.ko
	rm -f .*.cmd .*.o
	rm -rf .tmp_versions Module.symvers version.h
	rm -f core

	
## 'Private' targets used throughout the Makefile                                    
# Check the Linux headers
_checksrc: 
	@if [ ! -e $(KSRC) ]; then \
		echo ; \
		echo "!!! Error: Unable to locate the Linux kernel source and headers at $(KSRC) !"; \
		echo "    If your Linux kernel source sources are not located in $(KSRC), then"; \
		echo "    prepend your make command with KSRC=\"/path/to/linux/sources\""; \
		echo "    example: KSRC=\"/usr/src/linux-2.6.21\" make"; \
		echo ; \
		exit 1; \
	fi 
	@if [ ! -e $(KSRC)/.config ]; then \
		echo ; \
		echo "!!! Error: Your Linux kernel source is not configured, missing $(KSRC)/.config !"; \
		echo "    This is usually due to incomplete kernel sources, or specifying the wrong"; \
		echo "    path to your kernel sources. Contact your vendor for assistance."; \
		echo ; \
		exit 1; \
	fi 
	@if [ ! -e $(KSRC)/include ]; then \
		echo ; \
		echo "!!! Error: Your Linux kernel headers are incomplete, missing $(KSRC)/include !"; \
		echo "    This is usually due to incomplete kernel sources, or specifying the wrong"; \
		echo "    path to your kernel sources. Contact your vendor for assistance."; \
		echo ; \
		exit 1; \
	fi

# Check the kernel version - Kernel version must be at least 2.6.9
_check_kver:
ifneq (1,$(shell [ "$(BUILDVER)" = "2.6" -a 0$(SUBLEVEL) -ge 9 ] && echo 1))
	@echo ; \
	echo "!!! Error: Your Linux kernel is incompatible! Rhino PCI cards require"; \
	echo "    Linux kernel 2.6.9 and later. Detected: $(BUILDVER)-$(SUBLEVEL)"; \
	echo "    Please upgrade to a supported kernel before continuing."; \
	echo ; \
	exit 1;
endif

# Check for the Zaptel source/headers
_checkzap:
	@if [ ! -e $(ZAPDIR)/zaptel.h ]; then \
		echo ; \
		echo "!!! Error: Unable to locate the Zaptel source and headers at $(ZAPDIR) !"; \
		echo "    If your Zaptel sources are not located in $(ZAPDIR), then"; \
		echo "    prepend your make command with ZAPDIR=\"/path/to/zaptel/sources\""; \
		echo "    example: ZAPDIR=\"/usr/src/zaptel-1.4.8\" make"; \
		echo ; \
		exit 1; \
	fi
	@if [ ! -e $(ZAPDIR)/Module*.symvers ]; then \
		echo ; \
		echo "NOTICE: the file $(ZAPDIR)/Module[s].symvers is missing. This is perfectly"; \
		echo " normal. Again, this is Hmmm .. OKAY. You will see a number of warnings at the"; \
		echo " end of this compile complaining that certain symbols are undefined."; \
		echo " These are just warnings and will not affect anything. Again, this is OKAY."; \
		echo " There is absolutely nothing wrong and no reason to panic."; \
		echo ; \
		if [ ! $(SKIPWAIT) ]; then \
			echo -en "Continuing in 10...\007"; sleep 1; \
			echo -en " 9...\007"; sleep 1; \
			echo -en " 8...\007"; sleep 1; \
			echo -en " 7...\007"; sleep 1; \
			echo -en " 6...\007"; sleep 1; \
			echo -en " 5...\007"; sleep 1; \
			echo -en " 4...\007"; sleep 1; \
			echo -en " 3...\007"; sleep 1; \
			echo -en " 2...\007"; sleep 1; \
			echo -en " 1...\007"; sleep 1; \
			echo -e "\007" ; \
		fi \
	fi
endif

version.h:
	@build_tools/make_version_h "${RHINOPKGVER}" > $@.tmp
	@if cmp -s $@.tmp $@ ; then :; else \
		mv $@.tmp $@ ; \
	fi
	@rm -f $@.tmp

#modprobe file
install_modconf:
	build_tools/genmodconf "$(BUILDVER)" "$(INSTALL_PREFIX)" "$(MODULES) $(MODULE_ALIASES)"
	@if [ -d /etc/modutils ]; then \
		/sbin/update-modules ; \
	fi
