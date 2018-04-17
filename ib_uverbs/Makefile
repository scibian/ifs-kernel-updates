#
# Ib_uverbs module
#
#
# Called from the kernel module build system.
#
ifneq ($(KERNELRELEASE),)
#kbuild part of makefile

obj-$(CONFIG_INFINIBAND_USER_ACCESS) += ib_uverbs.o

ib_uverbs-y := uverbs_main.o uverbs_cmd.o uverbs_marshall.o

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

