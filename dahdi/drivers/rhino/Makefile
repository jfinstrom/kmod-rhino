## Won't work because the .config's aren't included here
#obj-$(CONFIG_RHINO_R1T1)		+= r1t1/
#obj-$(CONFIG_RHINO_RCBFX)		+= rcbfx/
#obj-$(CONFIG_RHINO_RXT1)		+= rxt1/

obj-m += r1t1/
obj-m += rcbfx/
obj-m += rxt1/

CFLAGS_MODULE += -I$(DAHDI_INCLUDE) -I$(src) -I$(RHINO_INCLUDE) -DDAHDI_VER=$(DAHDI_VER)

ifndef HOTPLUG_FIRMWARE
ifneq (,$(filter y m,$(CONFIG_FW_LOADER)))
HOTPLUG_FIRMWARE := yes
else
HOTPLUG_FIRMWARE := no
endif
export HOTPLUG_FIRMWARE
endif

# fix typo present in CentOS and RHEL 2.6.9 kernels
BAD_KERNELS_VERS := 22 34 34.0.1 34.0.2
BAD_KERNELS := $(foreach ver,$(BAD_KERNELS_VERS),2.6.9-$(ver).EL 2.6.9-$(ver).ELsmp)
ifneq (,$(filter $(KVERS),$(BAD_KERNELS)))
EXTRA_CFLAGS+=-Drw_lock_t=rwlock_t
endif
