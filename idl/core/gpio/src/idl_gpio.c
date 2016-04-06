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
 * File Name:       idl_gpio.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 *
 */

#include "idl.h"
#include "idl_gpio.h"
#include "idl_gpio_core.h"
#include "osal_lock.h"
#include "osal.h"
#include "_gpio.h" //gen3 and gen4
#include "_gpio_gen4_5.h"  //gen4.5
#include "_gpio_gen5.h" //gen5
#include "pal.h"
#include <linux/slab.h>

#include "idl_gpio_core.h"

static struct idl_gpio_host *gpio_host;
static os_interrupt_t *m_interrupt_handler;



/* gpio device suspend */
int idl_gpio_suspend(void)
{
	/*gpio suspend */
	if (gpio_host->suspend) 
		return gpio_host->suspend(gpio_host->vaddr);
	return 0;
}

/* gpio device resume */
int idl_gpio_resume(void)
{
    /*gpio resume */
    if (gpio_host->resume) 
		return gpio_host->resume(gpio_host->vaddr);
    return 0;
}


idl_result_t
idl_gpio_init(void)
{
	return IDL_SUCCESS;
}

void idl_gpio_release(void)
{
}

int
idl_gpio_core_init(void)
{
	int ret = 0;
	
	if (gpio_host->init(gpio_host)) {
		ret = -1;
	} else {
		m_interrupt_handler = (os_interrupt_t *)kzalloc( \
					gpio_host->get_number_of_gpio_interrupts() * sizeof(os_interrupt_t), GFP_KERNEL);
		if (m_interrupt_handler == NULL) {
			ret =  -ENOMEM;
		}
	}
	gpio_host->create_event();
	return ret;
}

int 
idl_gpio_core_release(void)
{
	gpio_host->destroy_event();
	kfree(m_interrupt_handler);
	m_interrupt_handler = NULL;
	gpio_host->exit(gpio_host);
	return 0;
}

idl_result_t
idl_valid_gpio_num(uint32_t gpio_num)
{
	if (gpio_host->is_valid(gpio_num))
		return IDL_SUCCESS;
	else
		return IDL_INVALID_PARAM;
}

idl_result_t
idl_gpio_supports_interrupts(uint32_t gpio_num)
{
	if (gpio_host->supports_interrupt(gpio_num))
		return IDL_SUCCESS;
	else
		return IDL_INVALID_PARAM;
}

idl_result_t
idl_gpio_line_config(uint32_t gpio_num, uint32_t gpio_config)
{
	if (gpio_host->is_valid(gpio_num))
	{
		gpio_host->set_line_config(gpio_host->vaddr, gpio_num, gpio_config);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t 
idl_gpio_set_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	if (gpio_host->supports_alt_function(gpio_num, fn_num))
	{
		gpio_host->set_alt_function(gpio_host->vaddr, gpio_num, fn_num);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}


idl_result_t
idl_gpio_interrupt_config(uint32_t gpio_num, idl_gpio_interrupt_type_t interrupt_type)
{
	if (!gpio_host->supports_interrupt(gpio_num))
		return IDL_INVALID_PARAM;
	
	switch (interrupt_type)
	{
		case IDL_GPIO_ACTIVE_HIGH_LEVEL:
		case IDL_GPIO_ACTIVE_LOW_LEVEL:
		case IDL_GPIO_RISING_UP_EDGE:
		case IDL_GPIO_FALLING_DOWN_EDGE:			
			break;
		default:
			return IDL_INVALID_PARAM;
	}

	gpio_host->set_interrupt_config(gpio_host->vaddr, gpio_num, interrupt_type);

	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_interrupt_status(uint32_t gpio_num, uint32_t *interrupt_status)
{
	if (gpio_host->supports_interrupt(gpio_num))
	{
		gpio_host->get_interrupt_status(gpio_host->vaddr, gpio_num, interrupt_status);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t 
idl_gpio_clear_interrupt(uint32_t gpio_num)
{
	if (gpio_host->supports_interrupt(gpio_num))
	{
		gpio_host->clear_interrupt(gpio_host->vaddr, gpio_num);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t idl_gpio_disable_interrupt(uint32_t gpio_num)
{
	if (gpio_host->supports_interrupt(gpio_num))
	{
		gpio_host->disable_interrupt(gpio_host->vaddr, gpio_num);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t 
idl_gpio_enable_interrupt(uint32_t gpio_num)
{
	if (gpio_host->supports_interrupt(gpio_num))
	{
		gpio_host->enable_interrupt(gpio_host->vaddr, gpio_num);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}


idl_result_t 
idl_gpio_register_interrupt_handler(uint32_t gpio_num, os_interrupt_handler_t *handler, void *data)
{
	int irq = 0;
	int devkey = 0;

	gpio_host->get_interrupt_info(gpio_host->vaddr, gpio_num, &irq, &devkey);
	m_interrupt_handler[gpio_num] = gpio_host->request_irq(irq, devkey, handler, data);

	if (m_interrupt_handler[gpio_num] == NULL)
	{
		return IDL_NOT_INITIALIZED;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_set_line(uint32_t gpio_num, uint32_t value)
{
	if (gpio_host->is_valid(gpio_num))
	{
		gpio_host->set_line(gpio_host->vaddr, gpio_num, (value & 1));
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t
idl_gpio_get_line(uint32_t gpio_num, uint32_t *value)
{

	if (gpio_host->is_valid(gpio_num))
	{
		gpio_host->get_line(gpio_host->vaddr, gpio_num, value);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t 
idl_gpio_release_interrupt_handler(uint32_t gpio_num)
{
	if (m_interrupt_handler[gpio_num] != NULL)
	{
		os_release_interrupt(m_interrupt_handler[gpio_num]);
		m_interrupt_handler[gpio_num] = NULL;
	}

	return IDL_SUCCESS;
}


idl_result_t idl_gpio_clear_ts(uint32_t gpio_num)
{
	if (gpio_host->is_in_NB(gpio_num))
	{
		gpio_host->clear_ts(gpio_num);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t idl_gpio_get_ts(uint32_t gpio_num, uint32_t *setting)
{
	if (gpio_host->is_in_NB(gpio_num))
	{
		gpio_host->get_ts(gpio_num, setting);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}	
}

idl_result_t idl_gpio_set_smi(uint32_t gpio_num, uint32_t setting)
{
	if (gpio_host->is_in_NB(gpio_num))
	{
		gpio_host->set_smi(gpio_num, setting);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
}

idl_result_t idl_gpio_set_gpe(uint32_t gpio_num, uint32_t setting)
{
	if (gpio_host->is_in_NB(gpio_num))
	{
		gpio_host->set_gpe(gpio_num, setting);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}	
}

idl_result_t idl_gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting)
{
	if (gpio_host->is_in_NB(gpio_num))
	{
		gpio_host->set_trigger_negative(gpio_num, setting);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}	
}

idl_result_t idl_gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting)
{
	if (gpio_host->is_in_NB(gpio_num))
	{
		gpio_host->set_trigger_positive(gpio_num, setting);
		return IDL_SUCCESS;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}	
}

void idl_gpio_event_create(void)
{
	gpio_host->create_event();
}

void idl_gpio_event_destroy(void)
{
	gpio_host->destroy_event();
}

int idl_gpio_wait_for_irq(unsigned int gpio_num)
{
	return gpio_host->wait_irq(gpio_num);
}

void idl_gpio_irq_handler(int gpio_num)
{
	gpio_host->handle_irq(gpio_num);
}


void idl_gpio_ack_irq(uint32_t gpio_num)
{
	gpio_host->ack_irq(gpio_num);
}

void idl_gpio_set_events(uint32_t gpio_num)
{
	gpio_host->set_events(gpio_num);
}

void idl_gpio_reset_events(uint32_t gpio_num)
{
	gpio_host->reset_events(gpio_num);
}

idl_result_t
idl_gpio_set_interrupt_router(uint32_t gpio_num, idl_gpio_interrupt_router_t router)
{
	if (gpio_host->supports_interrupt(gpio_num))
	{
		if (!gpio_host->set_interrupt_router || gpio_host->set_interrupt_router(gpio_host->vaddr, gpio_num, router))
			return IDL_INVALID_PARAM;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

idl_result_t
idl_gpio_get_interrupt_router(uint32_t gpio_num, idl_gpio_interrupt_router_t *router)
{
	if (gpio_host->supports_interrupt(gpio_num))
	{
		if (!gpio_host->get_interrupt_router || gpio_host->get_interrupt_router(gpio_host->vaddr, gpio_num, router))
			return IDL_INVALID_PARAM;
	}
	else
	{
		return IDL_INVALID_PARAM;
	}
	return IDL_SUCCESS;
}

int idl_gpio_register(struct idl_gpio_host *host)
{
	gpio_host = host;
	return 0;
}	
