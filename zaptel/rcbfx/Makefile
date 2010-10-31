ifneq ($(KBUILD_EXTMOD),)

include $(obj)/Kbuild

else

install: install-firmware

install-firmware:
ifeq ($(HOTPLUG_FIRMWARE),yes)
	@if [ -d $(INSTALL_PREFIX)/usr/lib/hotplug/firmware ]; then \
		install -m 644 rcbfx.fw $(INSTALL_PREFIX)/usr/lib/hotplug/firmware; \
		echo "Rhino firmware installed into $(INSTALL_PREFIX)/usr/lib/hotplug/firmware/"; \
	fi
	@if [ -d $(INSTALL_PREFIX)/lib/firmware ]; then \
		install -m 644 rcbfx.fw $(INSTALL_PREFIX)/lib/firmware; \
		echo "Rhino rcbfx firmware installed into $(INSTALL_PREFIX)/lib/firmware/"; \
	fi
else
	@echo "Rhino rcbfx firmware not installed, hotplugging firmware is not supported by your kernel."
endif

clean:
	@rm -f *.o
	@rm -f *.cmd *.o *.ko
	@rm -f .*.cmd .*.o
	@rm -rf .tmp_versions Module.symvers
	@rm -f core

uninstall-firwamre:
	@if [ -d $(INSTALL_PREFIX)/usr/lib/hotplug/firmware ]; then \
		rm -fv $(INSTALL_PREFIX)/usr/lib/firmware/rcbfx.fw; \
		echo "Rhino rcbfx firmware uninstalled from $(INSTALL_PREFIX)/usr/lib/hotplug/firmware/"; \
	fi
	@if [ -d $(INSTALL_PREFIX)/lib/firmware ]; then \
		rm -fv $(INSTALL_PREFIX)/lib/firmware/rcbfx.fw; \
		echo "Rhino rcbfx firmware uninstalled from $(INSTALL_PREFIX)/lib/hotplug/firmware/"; \
	fi

endif
