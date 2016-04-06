#/*
#
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2008-2012 Intel Corporation. All rights reserved.
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
#*/

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
        export IDL_BASE=$(PWD)/../../../
endif
include ../../Makefile.inc

PWD       := $(shell pwd)
#export KBUILD_EXTRA_SYMBOLS := $(PWD)/Import.symvers
# Below is no longer used and can be deleted.  04/06/2010
IMPORTS =       \
                os_acquire_interrupt \
                os_release_interrupt \
                _os_debug \
                _os_print \
                os_sleep \
                os_map_io_to_mem_nocache \
                os_pci_find_first_device \
                os_pci_free_device \
                os_pci_get_interrupt \
                os_pci_read_config_32 \
                os_event_destroy \
                os_event_reset \
                os_event_create \
                os_event_set \
                os_event_wait \
                os_unmap_io_from_mem \
                os_sema_destroy \
                os_sema_get \
                os_sema_init \
                os_sema_put \

SUBDIRS = gpio gpio_shim i2c spi smartcard


.PHONY: all
all:$(KERNEL_BUILD_DIR)
	@echo ">>>Building IDL linux_driver"
	@$(foreach SUBDIR, $(SUBDIRS),  ($(MAKE) -C $(SUBDIR)) && ) exit 0;

clean:
	@echo ">>>Clean IDL linux_driver"
	@$(foreach SUBDIR, $(SUBDIRS), ($(MAKE) -C $(SUBDIR) -f Makefile clean) && ) exit 0;
	@#rm -rf Import.symvers

# Below is no longer used and can be deleted.  04/06/2010
Import.symvers:
	@echo ">>>produce Import.symvers"
	@for i in $(IMPORTS) ; do \
                echo "0x00000000	$$i	XXX	EXPORT_SYMBOL" >> Import.symvers ;\
        done
