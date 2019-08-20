#
# Top level
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

CFLAGS_MODULE += -DUSE_PI_LED_ENABLE=1 -DIFS_RH76
obj-y := rdmavt/ hfi1/ ib_qib/ ib_ipoib/

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
