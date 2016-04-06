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

ifndef BUILD_ROOT
	export BUILD_ROOT=$(PWD)/../
endif


<default>:
	@echo "below targets are supported:"
	@echo "	all: build idl"
	@echo "	install: install idl"
	@echo "	clean: clean idl"

.PHONY: all
all: bld doc test

.PHONY: bld
bld:
	@echo ">>>Building idl"
	cd idl; make; 
	cd $(PWD)

.PHONY: install
install: all
	cd idl; make install; cd $(PWD)

.PHONY: install_target
install_target: all
	cd idl; make install_target; cd $(PWD)

.PHONY: install_dev
install_dev: all
	cd idl; make install_dev; cd $(PWD)

.PHONY: clean
clean: doc-clean
	cd idl; make clean;
	cd $(PWD)
	rm -rf include
	rm -rf lib

.PHONY: debug
debug: dbld doc test

.PHONY: dbld
dbld:
	@echo ">>>IDL debug build"
	cd idl; make -e DEBUG=1;
	cd $(PWD)

.PHONY: test
test:
	@echo ">>>Do nothing"

.PHONY: doc
doc:
	@echo ">>>Generate doc to output directory"
	make -C docs

doc-clean:
	make clean -C docs
