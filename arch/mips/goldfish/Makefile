#
# Makefile for Goldfish virtual platform
#

EXTRA_CFLAGS += -Wno-error

obj-$(CONFIG_GOLDFISH)	+= goldfish-platform.o
obj-$(CONFIG_GOLDFISH)	+= goldfish-interrupt.o
obj-$(CONFIG_GOLDFISH)	+= goldfish-time.o
obj-$(CONFIG_GOLDFISH)	+= pm.o

obj-$(CONFIG_MIPS_GOLDFISH_SWITCH)	+= switch.o

ifneq ($(CONFIG_MIPS_APPENDED_DTB), y)
obj-$(CONFIG_MIPS_RANCHU)        += ranchu.dtb.o

$(obj)/%.dtb: $(obj)/%.dts
       $(call if_changed,dtc)
endif
