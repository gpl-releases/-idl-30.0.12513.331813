# This file is provided under a dual BSD/LGPLv2.1 license.  When using 
# or redistributing this file, you may do so under either license.
#
# LGPL LICENSE SUMMARY
#
# Copyright(c) 2008-2011. Intel Corporation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify 
# it under the terms of version 2.1 of the GNU Lesser General Public 
# License as published by the Free Software Foundation.
#
# This library is distributed in the hope that it will be useful, but 
# WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public 
# License along with this library; if not, write to the Free Software 
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 
# USA. The full GNU Lesser General Public License is included in this 
# distribution in the file called LICENSE.LGPL.
#
# Contact Information:
#     Intel Corporation
#     2200 Mission College Blvd.
#     Santa Clara, CA  97052
#
#
#  BSD LICENSE
#
#  Copyright(c) 2008-2012 Intel Corporation. All rights reserved.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#    * Neither the name of Intel Corporation nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

########################################################################
#
# Intel Corporation Proprietary Information
# Copyright (c) 2008-2012 Intel Corporation. All rights reserved.
#
# This listing is supplied under the terms of a license agreement
# with Intel Corporation and may not be used, copied, nor disclosed
# except in accordance with the terms of that agreement.
#
########################################################################
                                                                       
ifndef IDL_BASE
        export IDL_BASE=$(PWD)/../../../../
endif
include ../../../Makefile.inc


ifndef VER1
VER1=9
VER2=9
VER3=9
VER4=9
VER=<Unofficial Build>
VERSION_NUMBER=<Unofficial Build>
endif

CFLAGS=-DFORMAT_ELF -O2 -Wuninitialized -D_INTERNAL_PCI_ -W -Wimplicit -Wreturn-type -Wswitch -Wcomment -Wformat -Wchar-subscripts -Wcast-qual
CFLAGS += -DVER1=$(VER1) -DVER2=$(VER2) -DVER3=$(VER3) -DVER4=$(VER4) -DVER="\"$(VER)\"" -DCOMPONENTNAME="\"$(COMPONENTNAME)\""


STATIC_LIB_OBJ_PVT = idl_gpio.o
SHARE_LIB_OBJ_PVT = $(patsubst %.o, %.pic.o, $(STATIC_LIB_OBJ_PVT))        

COMP_LIB_NAME = libidl_gpio
SHARE_LIB_NAME=$(COMP_LIB_NAME).so
#STATIC_LIB_NAME=$(COMP_LIB_NAME).a

COMP_LIBS = idl_gpio

CFLAGS += -I$(IDL_BASE)/idl/include
CFLAGS += -I$(BUILD_DEST)/include/
CFLAGS += -I$(BUILD_DEST)/include/linux_user
CFLAGS += -I$(BUILD_DEST)/include/linux
#CFLAGS += -I$(BUILD_ROOT)/kernel-30.0.12513.331813/kernel_source/linux-2.6.39/include

CFLAGS += -I$(IDL_BASE)/idl/linux_driver/drv/gpio

LDFLAGS += -lpthread

.PHONY: all clean
$(STATIC_LIB_NAME):$(STATIC_LIB_OBJ_PVT)
	mkdir -p $(outdir)/lib
	$(AR) $(ARFLAGS) $(STATIC_LIB_NAME) $^  
	cp $@  $(outdir)/lib/libidl_gpio_legacy.a

$(SHARE_LIB_NAME):$(SHARE_LIB_OBJ_PVT)
	mkdir -p $(outdir)/lib
	$(CC) -shared  $^ -o $@ 
	cp $@  $(outdir)/lib/libidl_gpio_legacy.so

all: $(SHARE_LIB_NAME)


%.o: %.c
	$(CC) -c $(CFLAGS) -o $@ $<

%.pic.o: %.c
	$(CC) -c $(CFLAGS) -fPIC -fpic -o $@ $<

clean: 
	rm -f $(STATIC_LIB_OBJ_PVT) $(SHARE_LIB_OBJ_PVT)  $(STATIC_LIB_NAME) $(SHARE_LIB_NAME)
