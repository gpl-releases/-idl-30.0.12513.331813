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
 * File Name:       idl_gpio_drv.h
 *----------------------------------------------------------------------
 *
 */
 
#ifndef _IDL_GPIO_DRV_H
#define _IDL_GPIO_DRV_H

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/ioctl.h>

/* WARNING: by including osal_interrupt.h in our IDL files, we are creating an 
 * external dependency that could break our device drivers. If OSAL ever includes
 * a user space header file, it will break our driver builds....
 */
#include "idl_gpio.h" 


/* Ioctl routines */
#define GPIO_IOCTL_MAGIC		0xD0 /* see /usr/src/linux/Documentation/ */
					     /* ioctl-number.txt to understand what this means */

#define GPIO_IOCTL_LINE_CONFIG		_IO (GPIO_IOCTL_MAGIC, 1)
#define GPIO_IOCTL_SET_ALT_FUNCTION	_IO (GPIO_IOCTL_MAGIC, 2)
#define GPIO_IOCTL_INTERRUPT_CONFIG	_IO (GPIO_IOCTL_MAGIC, 3)
#define GPIO_IOCTL_INTERRUPT_STATUS	_IO (GPIO_IOCTL_MAGIC, 4)
#define GPIO_IOCTL_CLEAR_INTERRUPT	_IO (GPIO_IOCTL_MAGIC, 5)
#define GPIO_IOCTL_SET_LINE		_IO (GPIO_IOCTL_MAGIC, 6)
#define GPIO_IOCTL_GET_LINE		_IO (GPIO_IOCTL_MAGIC, 7)
#define GPIO_IOCTL_CLEAR_TS _IO(GPIO_IOCTL_MAGIC, 8)
#define GPIO_IOCTL_GET_TS	_IO (GPIO_IOCTL_MAGIC, 9)
#define GPIO_IOCTL_WAIT_FOR_IRQ		_IO (GPIO_IOCTL_MAGIC, 10)
#define GPIO_IOCTL_ACK_IRQ		_IO (GPIO_IOCTL_MAGIC, 11)
#define GPIO_IOCTL_RELEASE_IRQ		_IO (GPIO_IOCTL_MAGIC, 12)
#define GPIO_IOCTL_REGISTER_IRQ		_IO (GPIO_IOCTL_MAGIC, 13)
#define GPIO_IOCTL_RELEASE_IRQ_HANDLER	_IO (GPIO_IOCTL_MAGIC, 14)
#define GPIO_IOCTL_SET_SMI _IO(GPIO_IOCTL_MAGIC, 15)
#define GPIO_IOCTL_SET_GPE _IO(GPIO_IOCTL_MAGIC, 16)
#define GPIO_IOCTL_SET_TRIGGER_NEGATIVE _IO(GPIO_IOCTL_MAGIC, 17)
#define GPIO_IOCTL_SET_TRIGGER_POSITIVE _IO(GPIO_IOCTL_MAGIC, 18)
#define GPIO_IOCTL_ENABLE_INTERRUPT _IO(GPIO_IOCTL_MAGIC, 19)
#define GPIO_IOCTL_DISABLE_INTERRUPT _IO(GPIO_IOCTL_MAGIC, 20)
#define GPIO_IOCTL_SET_INTERRUPT_ROUTER	_IO (GPIO_IOCTL_MAGIC, 21)
#define GPIO_IOCTL_GET_INTERRUPT_ROUTER	_IO (GPIO_IOCTL_MAGIC, 22)

/* I defined one structure for all ioctl calls. The intent is to ignore fields 
   that are not used for a particular ioctl. This structure will be passed into
   the ioctl by reference. */
typedef struct gpio_ioctl_args
{
	unsigned long			gpio_num;
	uint32_t			data;	/* used for both input and output */
	idl_gpio_interrupt_type_t	interrupt_type;
	idl_gpio_interrupt_router_t router;
	uint32_t	trigger_negative_setting;
	uint32_t	trigger_positive_setting;
} gpio_ioctl_args;

#ifdef __cplusplus
}
#endif

#endif

