/*
#
# This file is provided under a dual BSD/LGPLv2.1 license.  When using
# or redistributing this file, you may do so under either license.
#
# LGPL LICENSE SUMMARY
#
# Copyright(c) 2007-2012. Intel Corporation. All rights reserved.
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

/*------------------------------------------------------------------------------
 * File Name: idl_gpio_shim.c
 *------------------------------------------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <linux/gpio.h>

#include <idl.h>
#include <idl_gpio.h>


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
	
	ret = gpio_request(gpio_num, "IDL_GPIO_Shim");
	if (ret)
		return IDL_DEVICE_BUSY;
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

idl_result_t idl_gpio_disable_interrupt(uint32_t gpio_num)
{
	return IDL_SUCCESS;
}

idl_result_t idl_gpio_clear_interrupt(uint32_t gpio_num)
{
	return IDL_SUCCESS;
}
#ifdef __cplusplus
}
#endif

