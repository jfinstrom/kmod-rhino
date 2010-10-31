#
# Makefile	Rhino PCI card drivers Makefile
#
# Copyright	(c) 2010, Rhino Equipment Corp.
#
#		This program is free software; you can redistribute it and/or
#		modify it under the terms of the GNU General Public License
#		version 2, as published by the Free Software Foundation.
# ----------------------------------------------------------------------------
# Author:  Bryce Chidester <helpdesk@rhinoequipment.com>
#

## For Debugging
ifdef DEBUG
  $(warning DEBUG enabled)
endif
# And more debugging
ifeq "$(DEBUG)" "2"
  OLD_SHELL := $(SHELL)
  SHELL = $(warning [$@ ($^) ($?)])$(OLD_SHELL)
endif
## End Debugging statements

# Default DAHDI directory to be overridden by user
ifndef DAHDI_DIR
  ifeq "$(shell if [ -d /usr/src/dahdi-complete/linux ]; then echo yes; fi )" "yes"
    DAHDI_DIR:=/usr/src/dahdi-complete/linux
  else ifeq "$(shell if [ -d /usr/src/dahdi-linux ]; then echo yes; fi )" "yes"
    DAHDI_DIR:=/usr/src/dahdi-linux
  else ifeq "$(shell if [ -d /usr/include/dahdi ]; then echo yes; fi )" "yes"
    DAHDI_DIR:=/usr
  endif
  ifdef DEBUG
    $(info DAHDI_DIR found to be as $(DAHDI_DIR))
  endif
else
  ifdef DEBUG
    $(info Using $(DAHDI_DIR) for DAHDI sources/headers)
  endif
endif

# Kernel version and location
ifndef KVER
  KVER=$(shell uname -r)
  ifdef DEBUG
    $(info KVER=$(KVER))
  endif
endif
# Kernel modules directory
ifndef KMOD
  KMOD=/lib/modules/$(KVER)
  ifdef DEBUG
    $(info KMOD=$(KMOD))
  endif
endif
# Determine where our Kernel Source is
ifndef KSRC
  ifneq (,$(wildcard $(KMOD)/build))
    KSRC:=$(KMOD)/build
  else
    KSRC_SEARCH_PATH:=/usr/src/linux /usr/src/linux-$(KVER) /usr/src/kernels/$(KVER)
    KSRC:=$(shell for dir in $(KSRC_SEARCH_PATH); do if [ -d $$dir ]; then echo $$dir; break; fi; done)
  endif
  ifdef DEBUG
    $(info KSRC=$(KSRC))
  endif
endif

# ARCH, for compiler
ifndef ARCH
  ARCH:=$(shell uname -m | sed -e s/i.86/i386/)
  ifdef DEBUG
    $(info ARCH=$(ARCH))
  endif
endif

ifndef INSTALL_PREFIX
  INSTALL_PREFIX:=
endif

#Extract the package version
# Note- Since this depends on relative paths, we export
# this and don't re-evaluate it once in the clutches of Kbuild
ifndef RHINOPKGVER
  ifneq ($(wildcard .svn),)
    RHINOPKGVER:=SVN-$(shell build_tools/make_svn_branch_name)
  else
    $(warning [$@ ($^) ($?)] $(DEBUG) $(shell pwd))
    ifneq ($(wildcard .version),)
    RHINOPKGVER:=$(shell cat .version)
    else
    $(error Version not defined. Incomplete package?)
    endif
  endif
  ifdef DEBUG
    $(info Compiling Rhino DAHDI-compatible drivers, package version $(RHINOPKGVER))
  endif
  export RHINOPKGVER
endif

#Figure out the kernel version and sublevel
#BUILDVER:=$(shell uname -r|cut -d. -f1-2)
#SUBLEVEL:=$(shell head -n3 $(KSRC)/Makefile|tail -n1|awk '{print $$3}')
BUILDVER:=$(shell cat $(KSRC)/include/config/kernel.release | cut -d- -f1 | cut -d. -f1-2)
SUBLEVEL:=$(shell cat $(KSRC)/include/config/kernel.release | cut -d- -f1 | cut -d.   -f3)
ifdef DEBUG
  $(info Kernel BUILDVER=$(BUILDVER))
  $(info Kernel SUBLEVEL=$(SUBLEVEL))
endif

#Include the config telling us what to compile
KCONFIG:=$(KSRC)/.config
MYCONFIG:=$(PWD)/.config
ifneq (,$(wildcard $(KCONFIG)))
  include $(KCONFIG)
endif
ifneq (,$(wildcard $(MYCONFIG)))
  include $(MYCONFIG)
endif


# H'okay, so here we go
#If DAHDI_DIR, use that
#else if DAHDI_VER, use that
#else Probe

# User specified a DAHDI to compile against, use it
ifdef DAHDI_DIR
  $(info Probing $(DAHDI_DIR) for DAHDI version)
  DAHDI_VERSION_H:=$(shell find $(DAHDI_DIR) -name version.h)
  ifeq "$(DAHDI_VERSION_H)" ""
    $(error Could not find version.h in your sources. Perhaps your install is incomplete? Or perhaps you tried pointing DAHDI_DIR to he /usr/include headers?)
  else
    $(info Found version.h at $(DAHDI_VERSION_H))
  endif
  DAHDI_VERSION_STRING:=$(shell grep DAHDI_VERSION $(DAHDI_VERSION_H) | head -n1)
  ifeq "$(DEBUG)" "2"
    $(info Version line: $(DAHDI_VERSION_STRING))
  endif
  DAHDI_VER:=$(shell echo '$(DAHDI_VERSION_STRING)' | awk -F\" '{print $$2}')
  ifeq "$(DEBUG)" "2"
    $(info Version extracted: $(DAHDI_VER))
  endif
  DMAJOR:=$(shell echo $(DAHDI_VER) | awk -F . '{ print $$1 }')
  DMINOR:=$(shell echo $(DAHDI_VER) | awk -F . '{ print $$2 }')
  DREVISION:=$(shell echo $(DAHDI_VER) | awk -F . '{ print $$3 }')
  ifeq "$(DEBUG)" "2"
    $(info DAHDI version MAJOR=$(DMAJOR) MINOR=$(DMINOR) REVISION=$(DREVISION))
  endif
  SYS_DAHDI_VER:=$(shell expr $(DMAJOR) \* 65536 + $(DMINOR) \* 256 + $(DREVISION))

# User is overriding the version logic
else ifdef DAHDI_VER
  $(info Using user-defined version $(DAHDI_VER))
  DMAJOR:=$(shell echo $(DAHDI_VER) | awk -F . '{ print $$1 }')
  DMINOR:=$(shell echo $(DAHDI_VER) | awk -F . '{ print $$2 }')
  DREVISION:=$(shell echo $(DAHDI_VER) | awk -F . '{ print $$3 }')
  SYS_DAHDI_VER:=$(shell expr $(DMAJOR) \* 65536 + $(DMINOR) \* 256 + $(DREVISION))

# Do our friendly modinfo probe
else
  $(info Asking modprobe for the version.)
  SYS_DAHDI_VER:=$(shell build_tools/get_dahdi_ver)
  ifeq "$(SYS_DAHDI_VER)" "Module dahdi.ko is not installed"
    $(error $(SYS_DAHDI_VER))
  endif

#End version checks
endif

## Just a debugging message
$(info Compiling against DAHDI version $(SYS_DAHDI_VER))

#The make command used
KMAKE=$(MAKE) -C $(KSRC) ARCH=$(ARCH) \
		SUBDIRS=$(PWD)/drivers/rhino \
		RHINO_INCLUDE=$(PWD)/include \
		DAHDI_INCLUDE=$(DAHDI_DIR)/include \
		DAHDI_SRC=$(DAHDI_DIR)/drivers/dahdi \
		DAHDI_VER=$(SYS_DAHDI_VER) \
		KBUILD_EXTRA_SYMBOLS=$(DAHDI_DIR)/drivers/dahdi/Module.symvers

#Default target
all: modules

#Some simple checks
_checkDAHDI:
	@if [ ! -e $(DAHDI_DIR)/include/dahdi/kernel.h ]; then \
		echo ; \
		echo "!!! Error: Unable to locate the DAHDI source or headers at $(DAHDI_DIR) !"; \
		echo "    If your DAHDI sources are not located in $(DAHDI_DIR), then"; \
		echo "    prepend your make command with DAHDI_DIR=\"/path/to/DAHDI/sources\""; \
		echo "    example: DAHDI_DIR=\"/usr/src/dahdi-linux-2.4.0\" make"; \
		echo ; \
		exit 1; \
	fi
_checkKernel:
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
ifneq (1,$(shell [ "$(BUILDVER)" = "2.6" -a 0$(SUBLEVEL) -ge 9 ] && echo 1))
	@echo ; \
	echo "!!! Error: Your Linux kernel is incompatible! Rhino PCI cards require"; \
	echo "    Linux kernel 2.6.9 and later. Detected: $(BUILDVER)-$(SUBLEVEL)"; \
	echo "    Please upgrade to a supported kernel before continuing."; \
	echo ; \
	exit 1;
endif

_checks: _checkDAHDI _checkKernel

#version.h
include/rhino/version.h:
	@build_tools/make_version_h "${RHINOPKGVER}" > $@.tmp
	@if cmp -s $@.tmp $@ ; then :; else \
		mv $@.tmp $@ ; \
	fi
	@rm -f $@.tmp

modules: include/rhino/version.h _checks
	$(KMAKE) modules

clean:
	$(MAKE) -C $(KSRC) SUBDIRS=$(PWD) clean
	rm -rf include/rhino/version.h

install: all
	$(KMAKE) INSTALL_MOD_PATH=$(INSTALL_PREFIX) INSTALL_MOD_DIR=rhino modules_install
	[ `id -u` = 0 ] && /sbin/depmod -a $(KVER) || :

.PHONY: clean install modules _checkDAHDI _checkKernel

