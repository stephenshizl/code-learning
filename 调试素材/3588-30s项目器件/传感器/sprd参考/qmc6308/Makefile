# validate lookup result
ifeq ($(KDIR),)
$(error No KDIR found for platform $(TARGET_PLATFORM))
endif

ifeq ($(BUILD),debug)
ccflags-y += -DDEBUG
endif


#
# Makefile for qmc6308 sensor driver.
#
obj-m += qmc6308.o

modules:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) $@

clean:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) M=$(CURDIR) $@

kernelrelease:
	$(MAKE) ARCH=$(ARCH) -C $(KDIR) $@
