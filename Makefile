#
# Top level
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

obj-y := rdmavt/ ib_uverbs/ hfi1/ ib_mad/

else
#normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

install:
	$(MAKE) INSTALL_MOD_DIR=updates -C $(KDIR) M=$$PWD modules_install

endif

