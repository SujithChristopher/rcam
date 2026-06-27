# Out-of-tree build of the OmniVision ov9282 driver (covers OV9281/OV9282)
# for the Radxa Dragon Q6A (kernel 6.18.x-qcom), since CONFIG_VIDEO_OV9282 is
# not enabled in the shipped kernel.

obj-m += ov9282.o

KVER  ?= $(shell uname -r)
KDIR  ?= /lib/modules/$(KVER)/build
PWD   := $(shell pwd)

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	# Install alongside the in-tree i2c sensor modules
	install -D -m 0644 ov9282.ko \
		/lib/modules/$(KVER)/updates/ov9282.ko
	depmod -a $(KVER)

uninstall:
	rm -f /lib/modules/$(KVER)/updates/ov9282.ko
	depmod -a $(KVER)
