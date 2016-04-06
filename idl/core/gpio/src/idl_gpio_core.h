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
#ifndef _IDL_GPIO_CORE_H
#define _IDL_GPIO_CORE_H

#include <linux/types.h>
#include "idl_gpio.h"
#include "osal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef DEBUG
#define gpio_dbg(fmt, args...) do \
{ \
      printk(KERN_INFO fmt, ##args); \
} while(0)
#else
#define gpio_dbg(fmt, arg...)
#endif


struct idl_gpio_host {
	int  (*init)(struct idl_gpio_host *host);
	int  (*exit)(struct idl_gpio_host *host);
	bool (*is_valid)(uint32_t gpio_num);
	bool (*is_in_NB)(uint32_t gpio_num);
	bool (*supports_interrupt)(uint32_t gpio_num);
	bool (*supports_alt_function)(uint32_t gpio_num, uint32_t fn_num);

	void (*set_line_config)(uint32_t *m_idl_gpio_base, uint32_t gpio_num,uint32_t gpio_config);
	void (*set_alt_function)(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num);
	void (*set_interrupt_config)(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_type_t interrupt_type);
	void (*set_trigger_positive)(uint32_t gpio_num, uint32_t setting);
	void (*set_trigger_negative)(uint32_t gpio_num, uint32_t setting);
	void (*set_gpe)(uint32_t gpio_num, uint32_t setting);
	void (*set_smi)(uint32_t gpio_num, uint32_t setting);
	void (*set_line)(uint32_t *m_idl_gpio_base, uint32_t gpio_num,uint32_t val);
	int  (*set_interrupt_router)(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_router_t router);


	uint32_t (*get_base_addr)(void);
	uint32_t (*get_get_size)(void);
	uint32_t (*get_number_of_gpios)(void);
	uint32_t (*get_number_of_gpio_interrupts)(void);
	void (*get_interrupt_status)(uint32_t *m_idl_gpio_base, uint32_t gpio_num,uint32_t *interrupt_status);
	void (*get_interrupt_info)(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey);
	void (*get_line)(uint32_t *m_idl_gpio_base, uint32_t gpio_num,uint32_t *val);
	void (*get_ts)(uint32_t gpio_num, uint32_t *setting);
	int  (*get_interrupt_router)(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_router_t *router);

	void (*clear_interrupt)(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
	void (*clear_ts)(uint32_t gpio_num);

	void (*disable_interrupt)(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
	void (*enable_interrupt)(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
	
	void (*create_event)(void);
	void (*destroy_event)(void);
	
	void (*set_events)(uint32_t gpio_num);
	void (*reset_events)(uint32_t gpio_num);

	int  (*wait_irq)(uint32_t gpio_num);
	void (*handle_irq)(int32_t gpio_num);
	void (*ack_irq)(uint32_t gpio_num);
	

	os_interrupt_t (*request_irq)(int irq, int devkey, os_interrupt_handler_t *handler, void *data);
	void (*free_irq)(uint32_t gpio_num);

	int (*suspend)(void *data);
	int (*resume)(void *data);


	void *vaddr;
	phys_addr_t iomem_addr;
	uint32_t iomem_size;
};

/*suspend/resume*/
int idl_gpio_suspend(void);
int idl_gpio_resume(void);

void idl_gpio_ack_irq(uint32_t gpio_num);
void idl_gpio_set_events(uint32_t gpio_num);
void idl_gpio_reset_events(uint32_t gpio_num);

int idl_gpio_core_init(void);
int idl_gpio_core_release(void);
int idl_gpio_register(struct idl_gpio_host *host);

/* This function checks to see if the gpio number specified is a valid gpio */
idl_result_t idl_valid_gpio_num(uint32_t gpio_num);

/* This function checks to see if the gpio number specified is a valid interrupt */
idl_result_t idl_gpio_supports_interrupts(uint32_t gpio_num);

void idl_gpio_event_create(void);
void idl_gpio_event_destroy(void);
int idl_gpio_wait_for_irq(unsigned int gpio_num);
void idl_gpio_irq_handler(int gpio);


static inline unsigned int gpio_port_read32 (unsigned short int port)
{
  return inl(port);
}

static inline void gpio_port_write32 (unsigned int value, unsigned short int port)
{
	outl(value, port);
}

static inline unsigned int gpio_mmio_read32(void * addr)
{
	return OS_READ32(addr);
}	

static inline void gpio_mmio_write32(unsigned int value, void *addr)
{
	OS_WRITE32(value, addr);
}	
#ifdef __cplusplus
}
#endif

#endif
/*@}*/


