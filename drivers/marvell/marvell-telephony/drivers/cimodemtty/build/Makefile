#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License version 2 as
#  published by the Free Software Foundation.

# (C) Copyright 2012 Marvell International Ltd.
# All Rights Reserved
#

ifndef MODEM_ROOT
# MODEM_ROOT = $(shell pwd)/..
MODEM_ROOT = $(srctree)/drivers/marvell/marvell-telephony/drivers/cimodemtty
endif

BUILDROOT = $(srctree)/drivers/marvell/marvell-telephony/drivers

TARGET = gs_modem

PPP_DIR := $(BUILDROOT)/ppp

$(TARGET)-objs := \
	../ppp/src/chap.o \
	../ppp/src/ipcp.o \
	../ppp/src/lcp.o \
	../ppp/src/pap.o \
	../ppp/src/ppp.o \
	../ppp/src/ppp_log.o \
	../ppp/src/ppp_platform.o \
	src/modem_if.o

EXTRA_CFLAGS += -I$(MODEM_ROOT)/../inc \
		-I$(BUILDROOT)/include \
		-I$(PPP_DIR)/inc \
		-I$(BUILDROOT)/cimodemtty/inc

EXTRA_CFLAGS+=\
	-DENV_LINUX\
	-DCI_STUB_CLIENT_INCLUDE\
	-DUPGRADE_R99\
	-DTAVOR_AUDIO\
	-D_MICCO_B0_ \
	-DDEBUG_BUILD\
	-DACI_KERNEL_DEBUG \
	-DATCMDSRV_DEBUG\
	-DutlDEBUG\
	-DDEBUG_BUILD\
	-DMSL_TRACE_ON \
	-DMSLDL_DIRECT_ACCESS

# KBUILD_EXTRA_SYMBOLS = $(BUILDROOT)/cidatastub/Module.symvers

# all: modules

# ifneq ($(KERNELRELEASE),)

obj-m   := $(TARGET).o
#CC =arm-linux-gcc

# else

# modules:
# 	$(MAKE) -C $(KERNELDIR) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(MODEM_ROOT)/build modules
# endif

# install:
# 	install -d $(INSTALLDIR)
# 	install -c ../src/$(TARGET).ko $(INSTALLDIR)

# dev:

# clean:
# 	@echo 'Going to cleanup.\n'
# 	rm -rf                      \
# 	$(MODEM_ROOT)/src/*.o       \
# 	$(MODEM_ROOT)/src/*~        \
# 	$(MODEM_ROOT)/src/core      \
# 	$(MODEM_ROOT)/src/.*.d      \
# 	$(MODEM_ROOT)/src/.*.cmd    \
# 	$(MODEM_ROOT)/src/*.ko      \
# 	$(MODEM_ROOT)/src/*.mod.c   \
# 	$(MODEM_ROOT)/build/*.ko      \
# 	$(MODEM_ROOT)/build/*.mod.c   \
# 	$(MODEM_ROOT)/build/Module.symvers \
# 	$(MODEM_ROOT)/build/.tmp_versions   \
# 	$(MODEM_ROOT)/src/*.bak \
# 	$(PPP_DIR)/src/*.o
