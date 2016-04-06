/*
#
# This file is provided under a dual BSD/LGPLv2.1 license.  When using 
# or redistributing this file, you may do so under either license.
#
# LGPL LICENSE SUMMARY
#
# Copyright(c) 2007-2011. Intel Corporation. All rights reserved.
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
/*----------------------------------------------------------------------
 * File Name:       idl_gpio.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 */

#include "idl.h"
#include "idl_gpio.h"
#include "osal_memmap.h"
#include "osal_io.h"

#include "idl_gpio_drv.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
//the IRQ is unuseful in user mode,define it just to fix "undefined reference" in _gpio.c
uint32_t gpio_irq = 0;

static int m_fd = -1;

#define  MAX_INTERRUPT_GPIO_PINS   256

static void *irq_handler(void *arg); // arg - used for user data

typedef struct
{
	int exit;
	os_interrupt_handler_t *handler;
        uint32_t  gpio_num;
	void *data;
} thread_data;

static pthread_t thread_handle[MAX_INTERRUPT_GPIO_PINS];
static bool thread_created[MAX_INTERRUPT_GPIO_PINS]; 
static thread_data td[MAX_INTERRUPT_GPIO_PINS];
static void *irq_handler(void *arg); // arg - used for user data
 
idl_result_t
idl_gpio_init()
{

	if (m_fd > 0)
	{
		return IDL_ALREADY_INITIALIZED;
	}
	m_fd = open("/dev/gpio", O_RDWR);
 
        if (m_fd < 0)
        {
                OS_DEBUG("idl_gpio_init: Unable to attach to driver\n");
                return IDL_NOT_INITIALIZED;
        }

	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_line_config(uint32_t gpio_num, uint32_t gpio_config)
{
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_line_config: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}

	// configure the gpio based on user request
	gpio_ioctl_args a;
        a.gpio_num = gpio_num;
        a.data = gpio_config;
 
        // configure the gpio based on user request
        if (ioctl(m_fd, GPIO_IOCTL_LINE_CONFIG, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}

	return IDL_SUCCESS;
}

idl_result_t 
idl_gpio_set_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	// parameter checking
	// initialized?
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_set_alt_function: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.data = fn_num;

	// configure the gpio based on user request
        if (ioctl(m_fd, GPIO_IOCTL_SET_ALT_FUNCTION, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_interrupt_config(uint32_t gpio_num, idl_gpio_interrupt_type_t interrupt_type)
{
	// parameter checking
	// initialized?
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_interrupt_config: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.interrupt_type = interrupt_type;

	// configure the gpio based on user request
        if (ioctl(m_fd, GPIO_IOCTL_INTERRUPT_CONFIG, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}

	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_interrupt_status(uint32_t gpio_num, uint32_t *interrupt_status)
{
	// parameter checking
	// initialized?
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_interrupt_status: Not initialized\n");
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

	// configure the gpio based on user request
        if (ioctl(m_fd, GPIO_IOCTL_INTERRUPT_STATUS, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	*interrupt_status = a.data;

	return IDL_SUCCESS;
}

idl_result_t idl_gpio_clear_interrupt(uint32_t gpio_num)
{
	// parameter checking
	// initialized?
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_clear_interrupt: Not initialized\n");
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

    if (ioctl(m_fd, GPIO_IOCTL_CLEAR_INTERRUPT, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}

	return IDL_SUCCESS;
}

idl_result_t idl_gpio_register_interrupt_handler(uint32_t gpio_num, os_interrupt_handler_t *handler, void *data)
{
	int irq, devkey;
	pthread_attr_t attr;

	/* parameter checking
	   initialized? */
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_register_interrupt_handler: Not initialized\n");
		return IDL_NOT_INITIALIZED;
	}

	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	td[gpio_num].exit = 0;
    td[gpio_num].gpio_num = gpio_num; 
	td[gpio_num].handler = handler;
	td[gpio_num].data = data;

    if (ioctl(m_fd, GPIO_IOCTL_REGISTER_IRQ, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}

	pthread_attr_init(&attr);
	if (pthread_create(&thread_handle[gpio_num], &attr, &irq_handler, &td[gpio_num]) != 0)
	{
		OS_DEBUG("idl_gpio_register_interrupt_handler: Could not create thread.\n");
		thread_created[gpio_num] = 0;
		pthread_attr_destroy(&attr);
		return IDL_FAILURE;
	}
	else
	{
		thread_created[gpio_num] = 1;
	} 

	pthread_attr_destroy(&attr);
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_line(uint32_t gpio_num, uint32_t val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_line_set: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.data = (val & 0x1);

        if (ioctl(m_fd, GPIO_IOCTL_SET_LINE, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}

	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_get_line(uint32_t gpio_num, uint32_t *val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_line_set: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

        if (ioctl(m_fd, GPIO_IOCTL_GET_LINE, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	*val = a.data;
	return IDL_SUCCESS;
}

idl_result_t 
idl_gpio_release_interrupt_handler(uint32_t gpio_num)
{
	gpio_ioctl_args a;
	// parameter checking
	// initialized?
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_release_interrupt_handler: Not initialized\n");
		return IDL_NOT_INITIALIZED;
	}

	if(0 == thread_created[gpio_num])
	{
		OS_DEBUG("idl_gpio_release_interrupt_handler: no interrupt handler to release\n");
		return IDL_FIRST_ERROR;
	}

	td[gpio_num].exit = 1;
	a.gpio_num = gpio_num;
	
    if (ioctl(m_fd, GPIO_IOCTL_RELEASE_IRQ_HANDLER, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	pthread_cancel(thread_handle[gpio_num]);
	pthread_join(thread_handle[gpio_num], NULL);
	thread_created[gpio_num] = 0;
	thread_handle[gpio_num] = 0;
	return IDL_SUCCESS;
}

void
idl_gpio_release()
{
	if (m_fd >= 0)
	{
		close(m_fd);
		m_fd = -1;
	}
}
// arg - used for user data 
static void *irq_handler(void *arg)
{
	thread_data *td = (thread_data *)arg;
	gpio_ioctl_args a;
   int result = (int)(td->gpio_num);   
   if(result != -1)
      a.gpio_num = td->gpio_num;

	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	while (1)
	{
		// this blocks until an interrupt is detected
		ioctl(m_fd, GPIO_IOCTL_WAIT_FOR_IRQ, &a);
		
		// we'll need to check to see if we are exiting
		// the thread - we don't want to trigger a 
		// false interrupt. 
		if (td->exit)
		{
			break;	// exit while loop
		}
		// ok, an interrupt is detected...
		// call user-specified callback
		td->handler(td->data);
	
		// interrupt handling complete	
		ioctl(m_fd, GPIO_IOCTL_ACK_IRQ, &a); 
	}
	return arg;
}

idl_result_t
idl_gpio_clear_ts(uint32_t gpio_num)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_clear_ts: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

        if (ioctl(m_fd, GPIO_IOCTL_CLEAR_TS, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_get_ts(uint32_t gpio_num, uint32_t *val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_get_line: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

        if (ioctl(m_fd, GPIO_IOCTL_GET_TS, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	*val = a.data;
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_smi(uint32_t gpio_num, uint32_t val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_set_smi: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.data = val&0x1;
        if (ioctl(m_fd, GPIO_IOCTL_SET_SMI, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_gpe(uint32_t gpio_num, uint32_t val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_set_gpe: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.data = val&0x1;
	
        if (ioctl(m_fd, GPIO_IOCTL_SET_GPE, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_trigger_negative(uint32_t gpio_num, uint32_t val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_set_trigger_negetive: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.data = val&0x1;
        if (ioctl(m_fd, GPIO_IOCTL_SET_TRIGGER_NEGATIVE, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_trigger_positive(uint32_t gpio_num, uint32_t val)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_set_trigger_positive: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;
	a.data = val&0x1;
        if (ioctl(m_fd, GPIO_IOCTL_SET_TRIGGER_POSITIVE, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_enable_interrupt(uint32_t gpio_num)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_enable_interrupt: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

        if (ioctl(m_fd, GPIO_IOCTL_ENABLE_INTERRUPT, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}

	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_disable_interrupt(uint32_t gpio_num)
{
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_disable_interrupt: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	gpio_ioctl_args a;
	a.gpio_num = gpio_num;

    if (ioctl(m_fd, GPIO_IOCTL_DISABLE_INTERRUPT, &a) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_interrupt_router(uint32_t gpio_num, idl_gpio_interrupt_router_t router)
{
	gpio_ioctl_args arg;
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_disable_interrupt: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	arg.gpio_num = gpio_num;
	arg.router = router;
    if (ioctl(m_fd, GPIO_IOCTL_SET_INTERRUPT_ROUTER, &arg) < 0)
		return IDL_INVALID_PARAM;

	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_get_interrupt_router(uint32_t gpio_num, idl_gpio_interrupt_router_t *router)
{
	gpio_ioctl_args arg;
	// parameter checking
	if (m_fd < 0)
	{
		OS_DEBUG("idl_gpio_disable_interrupt: Not initialized\n", gpio_num);
		return IDL_NOT_INITIALIZED;
	}
	arg.gpio_num = gpio_num;
    if (ioctl(m_fd, GPIO_IOCTL_GET_INTERRUPT_ROUTER, &arg) < 0)
	{
		return IDL_INVALID_PARAM;
	}
	else
	{
		*router = (idl_gpio_interrupt_router_t )arg.data;
		return IDL_SUCCESS;
	}	
}

