# Copyright (C) 2015 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#******************************************************************************
#
# AndroidKernel.mk - kernel makefile file of virtual device armemu
#
# Copyright (c) 2015 Roger Ye.  All rights reserved.
#
# This is part of the build for virtual device armemu.
# Android makefile to build kernel as a part of Android Build.
#
#******************************************************************************

ifeq ($(TARGET_PREBUILT_KERNEL),)

ARCH=$(TARGET_ARCH)

TARGET_OUT_INTERMEDIATES ?= $(PRODUCT_OUT)/obj
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
KERNEL_CONFIG := $(KERNEL_OUT)/.config
TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/bzImage
KERNEL_HEADERS_INSTALL := $(KERNEL_OUT)/usr
KERNEL_MODULES_INSTALL := system
KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules
KERNEL_IMG=$(KERNEL_OUT)/arch/$(ARCH)/boot/Image

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/piggy
else
TARGET_PREBUILT_KERNEL := $(TARGET_PREBUILT_INT_KERNEL)
# TARGET_PREBUILT_KERNEL := $(KERNEL_OUT)/arch/x86/boot/bzImage
endif

define mv-modules
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(KERNEL_MODULES_OUT)/; done;\
fi
endef

define clean-module-folder
mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

# bison is needed to build kernel and external modules from source
BISON := $(HOST_OUT_EXECUTABLES)/bison$(HOST_EXECUTABLE_SUFFIX)
# MOD_ENABLED := $(shell grep ^CONFIG_MODULES=y $(KERNEL_CONFIG_FILE))

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(TARGET_KERNEL_CONFIG)

$(KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_KERNEL)
	$(hide) gunzip -c $(KERNEL_OUT)/arch/$(ARCH)/boot/compressed/piggy.gzip > $(KERNEL_OUT)/piggy

$(TARGET_PREBUILT_INT_KERNEL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH) modules
	$(MAKE) -C kernel O=../$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) INSTALL_MOD_STRIP=1 ARCH=$(TARGET_ARCH) modules_install
#	$(mv-modules)
#	$(clean-module-folder)

$(KERNEL_HEADERS_INSTALL): $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH) headers_install

kerneltags: $(KERNEL_OUT) $(KERNEL_CONFIG)
	$(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH) tags

kernelconfig: $(KERNEL_OUT) $(KERNEL_CONFIG)
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH) menuconfig
	env KCONFIG_NOTIMESTAMP=true \
	     $(MAKE) -C kernel O=../$(KERNEL_OUT) ARCH=$(TARGET_ARCH) savedefconfig
	cp $(KERNEL_OUT)/defconfig kernel/arch/$ARCH/configs/$(TARGET_KERNEL_CONFIG)

#ifneq ($(MOD_ENABLED),)
KERNEL_MODULES_DEP := $(firstword $(wildcard $(TARGET_OUT)/lib/modules/*/modules.dep))
KERNEL_MODULES_DEP := $(if $(KERNEL_MODULES_DEP),$(KERNEL_MODULES_DEP),$(TARGET_OUT)/lib/modules)

$(TARGET_OUT_INTERMEDIATES)/%.kmodule: $(INSTALLED_KERNEL_TARGET)
	$(hide) cp -an $(EXTRA_KERNEL_MODULE_PATH_$*) $(TARGET_OUT_INTERMEDIATES)/$*.kmodule
	@echo Building additional kernel module $*
	$(mk_kernel) M=$(abspath $@) modules

$(KERNEL_MODULES_DEP): $(INSTALLED_KERNEL_TARGET) $(patsubst %,$(TARGET_OUT_INTERMEDIATES)/%.kmodule,$(TARGET_EXTRA_KERNEL_MODULES))
	$(hide) rm -rf $(TARGET_OUT)/lib/modules
	$(mk_kernel) INSTALL_MOD_PATH=$(abspath $(TARGET_OUT)) modules_install
	+ $(hide) for kmod in $(TARGET_EXTRA_KERNEL_MODULES) ; do \
	        echo Installing additional kernel module $${kmod} ; \
	        $(subst +,,$(subst $(hide),,$(mk_kernel))) INSTALL_MOD_PATH=$(abspath $(TARGET_OUT)) M=$(abspath $(TARGET_OUT_INTERMEDIATES))/$${kmod}.kmodule modules_install ; \
	done
	$(hide) rm -f $(TARGET_OUT)/lib/modules/*/{build,source}
#endif

$(BUILT_SYSTEMIMAGE): $(KERNEL_MODULES_DEP)

endif  # TARGET_PREBUILT_KERNEL
