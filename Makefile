obj-m += imx294_imx492.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(shell pwd)

clean:
	$(MAKE) -C $(KDIR) M=$(shell pwd) clean
