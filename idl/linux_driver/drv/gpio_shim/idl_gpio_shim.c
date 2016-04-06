/*
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

/*----------------------------------------------------------------------
 * File Name:       idl_gpio_shim.c
 *----------------------------------------------------------------------
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/pci.h>
#include "idl.h"
#include "idl_gpio.h"
#include "pal.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    #include <asm/semaphore.h>
#else
    #include <linux/semaphore.h>
#endif
/* These are predefined macros that specify different parameters
 * for our driver */ 
MODULE_AUTHOR("Intel Corporation, (C) 2005 - 2011 - All Rights Reserved");
MODULE_DESCRIPTION("IDL GPIO Shim Device Driver");
MODULE_SUPPORTED_DEVICE("Intel Media Processors");

/* Notifies the kernel that our driver is not GPL. */
MODULE_LICENSE("Dual BSD/GPL");

/* These are the symbols we want exported to other modules */
EXPORT_SYMBOL(idl_gpio_init);
EXPORT_SYMBOL(idl_gpio_release);
EXPORT_SYMBOL(idl_gpio_line_config);
EXPORT_SYMBOL(idl_gpio_set_alt_function);
EXPORT_SYMBOL(idl_gpio_set_line);
EXPORT_SYMBOL(idl_gpio_get_line);

/* Unique name for GPIO driver */
#define GPIO_DEV_NAME "gpio_shim"

#ifndef MOD_NAME
#define MOD_NAME "idl_gpio_shim.ko"
#endif
//#define DEBUG 1
char *version_string = "#@#" MOD_NAME " " VER;

/* high level init function (called by insmod when driver is loaded) */
int gpio_init(void);

/* high level remove function (called by rmmod when driver is unloaded) */
void gpio_cleanup(void);

/* tell the kernel the name of our entry points (these macros are defined in linux/module.h */
module_init(gpio_init);
module_exit(gpio_cleanup);


/* High level gpio initialization - called at driver load time */
int gpio_init()
{
	return 0;
}

/* called when the system unloads our driver */
void gpio_cleanup()
{
	return;
}


idl_result_t idl_gpio_init()
{
	return IDL_SUCCESS;
}

void idl_gpio_release()
{
	return;
}

idl_result_t idl_gpio_line_config(uint32_t gpio_num, uint32_t gpio_config)
{
	int ret = 0;

	if (gpio_config & IDL_GPIO_OUTPUT) {
		ret = gpio_request_one(gpio_num, GPIOF_OUT_INIT_LOW, "IDL_GPIO_Shim");
	} else {
		ret = gpio_request_one(gpio_num, GPIOF_IN, "IDL_GPIO_Shim");
	}
	gpio_free(gpio_num);
	if (ret) 
		return IDL_DEVICE_BUSY;
	else
		return IDL_SUCCESS;
}

idl_result_t idl_gpio_set_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	int ret = 0;
	
	gpio_request(gpio_num, "IDL_GPIO_Shim");
	ret = gpio_set_multi_function(gpio_num, fn_num);
	gpio_free(gpio_num);
	if (ret) 
		return IDL_FAILURE;
	else
		return IDL_SUCCESS;
}

idl_result_t idl_gpio_set_line(uint32_t gpio_num, uint32_t val)
{
	int ret = 0;

	ret = gpio_request(gpio_num, "IDL_GPIO_Shim");
	if (ret)
		return IDL_FAILURE;
	gpio_set_value(gpio_num, val);
	gpio_free(gpio_num);
	return IDL_SUCCESS;
}

idl_result_t idl_gpio_get_line(uint32_t gpio_num, uint32_t *val)
{
	int ret = 0;

	if (val == NULL)
		return IDL_INVALID_PARAM;
	ret = gpio_request(gpio_num, "IDL_GPIO_Shim");
	*val = gpio_get_value(gpio_num);
	if (ret)
		return IDL_FAILURE;
	gpio_free(gpio_num);
	return IDL_SUCCESS;
}
