#/*
#
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2007-2012 Intel Corporation. All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#  The full GNU General Public License is included in this distribution
#  in the file called LICENSE.GPL.
#
#  Contact Information:
#  intel.com
#  Intel Corporation
#  2200 Mission College Blvd.
#  Santa Clara, CA  95052
#  USA
#  (408) 765-8080
#
#
#  BSD LICENSE
#
#  Copyright(c) 2007-2012 Intel Corporation. All rights reserved.
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
#*/

#----------------------------------------------------------------------------
# Copyright (c) Intel Corporation 2007-2010
#
# INTEL MAKES NO WARRANTY OF ANY KIND REGARDING THE CODE.
# THIS CODE IS LICENSED ON AN "AS IS" BASIS AND INTEL WILL NOT PROVIDE ANY
# SUPPORT, ASSISTANCE, INSTALLATION, TRAINING OR OTHER SERVICES.
# INTEL DOES NOT PROVIDE ANY UPDATES, ENHANCEMENTS OR EXTENSIONS.
# INTEL SPECIFICALLY DISCLAIMS ANY WARRANTY OF MERCHANTABILITY,
# NONINFRINGEMENT, FITNESS FOR ANY PARTICULAR PURPOSE, AND ALL OTHER
# WARRANTIES, EXPRESS OR IMPLIED.
#
# Intel disclaims all liability, including liability for infringement of
# any proprietary or intellectual property rights, relating to use of the code.
# Except as may be provided in an express, written license from Intel, no
# license, express or implied, by estoppel or otherwise, to any intellectual
# property rights is granted herein.
#----------------------------------------------------------------------------
ifndef BUILD_ROOT
	export BUILD_ROOT=$(PWD)/../../
endif

ifndef BUILD_DIR
	export BUILD_DIR=i686-linux-elf
endif

ifndef BUILD_DEST
	export BUILD_DEST=$(BUILD_ROOT)/$(BUILD_DIR)
endif

ifndef EXE_DIR
        EXE_DIR = $(BUILD_DEST)/bin
endif

ifndef TARGETDEST
        export TARGETDEST = $(BUILD_ROOT)/target/
endif

ifndef FSROOT
        export FSROOT = $(TARGETDEST)/fsroot/
endif

ifndef KERNEL_VER
	export KERNEL_VER=linux-2.6.28
endif

ifndef KERNEL_BUILD_DIR
	export KERNEL_BUILD_DIR=$(BUILD_DEST)/kernel/$(KERNEL_VER)
endif


#-----
# Architecture Types
#-----
ifndef TARG_ARCH
    export 	TARG_ARCH=intel_gen3-linux
endif

ifndef HOST_CC
    export HOST_CC=gcc
endif
ifeq ($(TARG_ARCH),intel_gen3-linux)
    BUILD_DIR=i686-linux-elf
    HOST=i386-linux
    TARGET=i386-linux
    BUILD=i386-linux
    CROSS_COMPILE=$(EXE_DIR)/i686-cm-linux-
endif

ifeq ($(TARG_ARCH),i686-linux)
    BUILD_DIR=i686-linux-elf
    TARGET=i386-linux
    BUILD=i386-linux
    CROSS_COMPILE=
endif

ifeq ($(TARG_ARCH),arm-linux)
    BUILD_DIR=arm-linux-elf
    HOST=arm-linux
    TARGET=arm-linux
    BUILD=i386-linux
    CROSS_COMPILE=arm-linux-
endif

export HOST
export TARGET
export BUILD
export CROSS_COMPILE

ifdef TARGETCC
    export CC=$(TARGETCC)
    export LD=$(TARGETLD)
    export AR=$(TARGETAR)
    export AR_RC=$(TARGETAR) rc
    export STRIP=$(TARGETSTRIP)
    export CXX=$(TARGETCXX)
    export RANLIB=$(TARGETRANLIB)
    export STRINGS=$(TARGETSTRINGS)
    export M4=$(TARGETM4)
    export BISON=$(TARGETBISON)
    export YACC=$(TARGETYACC)
else
    export CC=$(CROSS_COMPILE)gcc
    export LD=$(CROSS_COMPILE)ld
    export AR=$(CROSS_COMPILE)ar
    export AR_RC=$(CROSS_COMPILE)ar rc
    export STRIP=$(CROSS_COMPILE)strip
    export CXX=$(CROSS_COMPILE)g++
    export RANLIB=$(CROSS_COMPILE)ranlib
    export STRINGS=$(CROSS_COMPILE)strings
    export M4=$(CROSS_COMPILE)m4
    export BISON=$(CROSS_COMPILE)bison
    export YACC=$(CROSS_COMPILE)yacc
endif

#--------
#version information
#--------
ifdef COMPONENTNAME
VERSION = $(COMPONENTNAME) $(VER)
else
export VERSION = "Developer"
export COMPONENTNAME = `basename $(CURDIR)`
endif
ifdef VER4
TIC = $VER4
else
TIC= "$$Revision$$"
endif


#-----
# debug/release flags
#-----
ifndef DEBUG
   typedir +=
   CXXFLAGS += -O3 -fomit-frame-pointer
   CFLAGS += -O3 -fomit-frame-pointer
else
   typedir += debug
   CXXFLAGS += -ggdb3
   CFLAGS+=-g3 -DDEBUG
   LDFLAGS+=-g3
endif


#-----
# versioning
#-----
ifndef VER1
VER1=9
VER2=9
VER3=9
VER4=9
VER=<Unofficial Build>
VERSION_NUMBER=<Unofficial Build>
endif

CFLAGS += -DVER1=$(VER1) -DVER2=$(VER2) -DVER3=$(VER3) -DVER4=$(VER4) -DVER="\"$(VER)\"" -DCOMPONENTNAME="\"$(COMPONENTNAME)\""


CFLAGS += -I$(IDL_BASE)/idl/include -I$(IDL_BASE)/$(TARGET)/include

CFLAGS+=-g3
LDFLAGS+=-g3
#LDFLAGS+=-Wl,--allow-shlib-undefined
#LDFLAGS+=-Wl,--unresolved-symbols=ignore-in-shared-libs
LDFLAGS+=-L$(IDL_BASE)/idl/lib -L$(IDL_BASE)/$(TARGET)/lib


#outdir :=  $(IDL_BASE)/$(TARGET)/$(typedir)
#objects_c = $(patsubst %.c,%.o,$(filter %.c,$(SRC)))
#objects_cpp = $(patsubst %.cpp,%.o,$(filter %.cpp,$(SRC)))
#objects = $(objects_c) $(objects_cpp)
#objects_path_c = $(patsubst %.c,$(outdir)/%.o,$(filter %.c,$(SRC)))
#objects_path_cpp = $(patsubst %.cpp,$(outdir)/%.o,$(filter %.cpp,$(SRC)))
#objects_path = $(objects_path_c) $(objects_path_cpp)
outdir = $(IDL_BASE)/idl
lib_path := $(outdir)/$(MODULE)

vpath %.o $(outdir)

my_release_dir = $(IDL_BASE)


#-----
# default rules
#-----
%.o : %.cpp
ifndef VERBOSE
	@echo "  CXX   " $<
	@$(CXX) -c $(CXXFLAGS) -o $@ $<
else
	$(CXX) -c $(CXXFLAGS) -o $@ $<
endif

%.o : %.c
	echo Compiling: $< , with CC=$(CC) and CFLAGS=$(CFLAGS)
ifndef VERBOSE
	@echo "  CC   " $<
	$(CC) -c $(CFLAGS) -o $@ $<
else
	$(CC) -c $(CFLAGS) -o $@ $<
endif


all:

$(outdir):
	@mkdir -p $(outdir)

$(my_release_dir):
	@mkdir -p $(my_release_dir)
	@mkdir -p $(my_release_dir)/etc
	@mkdir -p $(my_release_dir)/lib
	@mkdir -p $(my_release_dir)/bin
	@mkdir -p $(my_release_dir)/docs
	@mkdir -p $(my_release_dir)/include


print-env:
	-@echo "*********************************************************************"
	-@echo Here are some environment variables that are used in the build:
	-@echo -- PATH=$(PATH)
	-@echo -- Compiler=$(CC)
	-@echo -- CFLAGS=$(CFLAGS)
	-@echo -- LDFLAGS=$(LDFLAGS)
	-@echo "*********************************************************************"



