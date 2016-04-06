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

ifndef IDL_BASE
	export IDL_BASE=$(PWD)/../
endif
include Makefile.inc

IDL_SUB_DIRS= \
linux_driver \
linux_user

.PHONY: all
all:$(KERNEL_BUILD_DIR)
	@echo ">>>Building IDL Components"
	@echo "STRIP=$(STRIP)"
	@$(foreach SUBDIR, $(IDL_SUB_DIRS), ($(MAKE) -C $(SUBDIR) -f Makefile) && ) exit 0;
	
$(KERNEL_BUILD_DIR):
#	tar -xzf $(IDL_BASE)/linux-2.6.16.16.tar.gz  -C $(IDL_BASE)

.PHONY: install
install: install_target install_dev 
	@echo "Install dev and target files"


.PHONY: install_dev 
install_dev:
	echo "**************$(PWD)***************"
	mkdir -p $(my_release_dir)/lib
	mkdir -p $(my_release_dir)/lib/modules
	mkdir -p $(my_release_dir)/include
	cp -f include/idl.h           $(my_release_dir)/include/
	cp -f include/idl_gpio.h      $(my_release_dir)/include/
	cp -f include/idl_i2c.h       $(my_release_dir)/include/
	cp -f include/idl_spi.h       $(my_release_dir)/include/
	cp -f include/idl_smartcard.h $(my_release_dir)/include/
	cp -f lib/libidlerror.a       $(my_release_dir)/lib
	cp -f lib/libidl_*.a          $(my_release_dir)/lib
	cp -f lib/libidl_*.so         $(my_release_dir)/lib
	cp -f linux_driver/drv/i2c/idl_i2c.ko $(my_release_dir)/lib/modules/idl_i2c.ko
	cp -f linux_driver/drv/gpio/idl_gpio_legacy.ko $(my_release_dir)/lib/modules/idl_gpio_legacy.ko
	cp -f linux_driver/drv/gpio_shim/idl_gpio_shim.ko $(my_release_dir)/lib/modules/idl_gpio_shim.ko
	cp -f linux_driver/drv/spi/idl_spi.ko $(my_release_dir)/lib/modules/idl_spi.ko
	cp -f linux_driver/drv/smartcard/idl_smartcard.ko $(my_release_dir)/lib/modules/idl_smartcard.ko
	if [ ! -d $(BUILD_DEST) ]; then mkdir -p $(BUILD_DEST); fi || exit 1
	if [ ! -d $(BUILD_DEST)/lib ]; then mkdir -p $(BUILD_DEST)/lib; fi || exit 1
	if [ ! -d $(BUILD_DEST)/lib/modules ]; then mkdir -p $(BUILD_DEST)/lib/modules; fi || exit 1
	if [ ! -d $(BUILD_DEST)/include ]; then mkdir -p $(BUILD_DEST)/include; fi || exit 1
	cp -f include/idl.h           $(BUILD_DEST)/include/
	cp -f include/idl_gpio.h      $(BUILD_DEST)/include/
	cp -f include/idl_i2c.h       $(BUILD_DEST)/include/
	cp -f include/idl_spi.h       $(BUILD_DEST)/include/
	cp -f include/idl_smartcard.h $(BUILD_DEST)/include/
	cp -f lib/libidlerror.a       $(BUILD_DEST)/lib
	cp -f lib/libidl_*.a          $(BUILD_DEST)/lib
	cp -f lib/libidl_*.so         $(BUILD_DEST)/lib
	cp -f linux_driver/drv/i2c/idl_i2c.ko $(BUILD_DEST)/lib/modules/idl_i2c.ko
	cp -f linux_driver/drv/gpio/idl_gpio_legacy.ko $(BUILD_DEST)/lib/modules/idl_gpio_legacy.ko
	cp -f linux_driver/drv/gpio_shim/idl_gpio_shim.ko $(BUILD_DEST)/lib/modules/idl_gpio_shim.ko
	cp -f linux_driver/drv/spi/idl_spi.ko $(BUILD_DEST)/lib/modules/idl_spi.ko
	cp -f linux_driver/drv/smartcard/idl_smartcard.ko $(BUILD_DEST)/lib/modules/idl_smartcard.ko
	if [ -e $(IDL_BASE)/init_idl ]; then mkdir -p $(BUILD_DEST)/etc/init.d/; cp -f $(IDL_BASE)/init_idl $(BUILD_DEST)/etc/init.d/idl; fi	
	if [ -e $(IDL_BASE)/README_IDL.txt ]; then cp -f $(IDL_BASE)/README_IDL.txt $(BUILD_DEST); fi
	
.PHONY: install_target
install_target:
	mkdir -p $(TARGETDEST)
	mkdir -p $(FSROOT)
	mkdir -p $(FSROOT)/lib
	mkdir -p $(FSROOT)/lib/modules
	cp -f lib/libidl*.so    $(FSROOT)/lib/
	cp -f linux_driver/drv/i2c/idl_i2c.ko $(FSROOT)/lib/modules/idl_i2c.ko
	cp -f linux_driver/drv/gpio/idl_gpio_legacy.ko $(FSROOT)/lib/modules/idl_gpio_legacy.ko
	cp -f linux_driver/drv/gpio_shim/idl_gpio_shim.ko $(FSROOT)/lib/modules/idl_gpio_shim.ko
	cp -f linux_driver/drv/spi/idl_spi.ko $(FSROOT)/lib/modules/idl_spi.ko
	cp -f linux_driver/drv/smartcard/idl_smartcard.ko $(FSROOT)/lib/modules/idl_smartcard.ko
	if [ -e $(IDL_BASE)/init_idl ]; then mkdir -p $(FSROOT)/etc/init.d/; cp -f $(IDL_BASE)/init_idl $(FSROOT)/etc/init.d/idl; fi	

debug: override DEBUG=1 CPPFLAGS += -ggdb3 CFLAGS += -ggdb3 typedir = debug
debug:
	@echo ">>>Creating Debug Building Block Libraries"
	@$(foreach SUBDIR, $(PSL_LIBRARY_DIRS), ($(MAKE) -C $(SUBDIR) -f Makefile -e DEBUG=1 ) && ) exit 0;
	#@echo ">>>Creating Debug Plugin" 
	#$(foreach SUBDIR, $(GST_PLUGIN_DIRS), 	$(MAKE) -C $(SUBDIR) -f Makefile -e DEBUG=1;)

test:
	@echo ">>>Creating Tests for Building Block Libraries"
	@$(foreach SUBDIR, $(PSL_LIBRARY_DIRS), ($(MAKE) -C $(SUBDIR) -f Makefile test ) &&) exit 0;
	
	@echo ">>>Creating Tests for Plugin"
	@$(foreach SUBDIR, $(GST_PLUGIN_DIRS), ( $(MAKE) -C $(SUBDIR) -f Makefile test ) && ) exit 0;
	@echo ">>>Creating Tests for Sample Applications"
	@$(foreach SUBDIR, $(GST_SAMPLE_DIRS), ($(MAKE) -C $(SUBDIR) -f Makefile test ) && ) exit 0;

clean:
	@echo ">>>Clean IDL Components"
	@$(foreach SUBDIR, $(IDL_SUB_DIRS), ($(MAKE) -C $(SUBDIR) -f Makefile clean ) && ) exit 0;
	make uninstall
doc:
	mkdir -p $(my_release_dir)/doc
	cp -rf ../doc/* $(my_release_dir)/doc/
	
uninstall:
	#rm -f $(BUILD_DEST)/include/idl.h
	#rm -f $(BUILD_DEST)/include/idl_gpio.h
	#rm -f $(BUILD_DEST)/include/idl_i2c.h 
	#rm -f $(BUILD_DEST)/include/idl_spi.h 
	#rm -f $(BUILD_DEST)/include/idl_smartcard.h
	rm -f $(BUILD_DEST)/lib/libidlerror.a 
	rm -f $(BUILD_DEST)/lib/libidl_*.a  
	rm -f $(BUILD_DEST)/lib/libidl_*.so  
	rm -f $(BUILD_DEST)/lib/modules/idl_i2c.ko
	rm -f $(BUILD_DEST)/lib/modules/idl_gpio_legacy.ko
	rm -f $(BUILD_DEST)/lib/modules/idl_gpio_shim.ko
	rm -f $(BUILD_DEST)/lib/modules/idl_spi.ko
	rm -f $(BUILD_DEST)/lib/modules/idl_smartcard.ko
	rm -f $(BUILD_DEST)/init_idl	
	rm -f $(BUILD_DEST)/README_IDL.txt
	#rm -f $(my_release_dir)/include/idl.h
	#rm -f $(my_release_dir)/include/idl_gpio.h
	#rm -f $(my_release_dir)/include/idl_i2c.h 
	#rm -f $(my_release_dir)/include/idl_spi.h 
	#rm -f $(my_release_dir)/include/idl_smartcard.h
	rm -f $(my_release_dir)/lib/libidlerror.a 
	rm -f $(my_release_dir)/lib/libidl_*.a  
	rm -f $(my_release_dir)/lib/libidl_*.so  
	rm -f $(my_release_dir)/lib/modules/idl_i2c.ko
	rm -f $(my_release_dir)/lib/modules/idl_gpio_shim.ko
	rm -f $(my_release_dir)/lib/modules/idl_gpio_legacy.ko
	rm -f $(my_release_dir)/lib/modules/idl_spi.ko
	rm -f $(my_release_dir)/lib/modules/idl_smartcard.ko
	rm -f $(my_release_dir)/doc/*
	rm -rf ./lib/

