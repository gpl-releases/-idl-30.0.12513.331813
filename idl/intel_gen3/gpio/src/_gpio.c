/*
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

/*----------------------------------------------------------------------
 * File Name:       _gpio.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 */

#include <linux/types.h>
#include "osal.h"
#include "_gpio.h"
//#include <linux/wait.h>	//For ERESTARTSYS
//here is io space operations, should put appropriate

//#define GEN3_DEBUG 1

/* Events used to signal the start and completion of interrupt 
 * handling.\n */
os_event_t gen3_gpio_interrupt_start[GEN3_GPIO_NUM_INT];
os_event_t gen3_gpio_interrupt_done[GEN3_GPIO_NUM_INT];
  
int gen3_irq_pending[GEN3_GPIO_NUM_INT] = {0}; 
uint32_t gen3_irq_params[GEN3_GPIO_NUM_INT] = {0};
int gen3_irq_release = 0;

#define GEN3_GPIO_PCI_BUS_NUM 1
#define GEN3_GPIO_PCI_DEVICE_NUM 11
#define GEN3_GPIO_PCI_FUNC_NUM 1


struct idl_gpio_host ce3100_gpio_host =
{
	.init = _gpio_init,
	.exit = _gpio_exit,

	.is_valid = _valid_gpio_num,
	.is_in_NB = _valid_gpio_num_NB,
	.supports_interrupt = _gpio_supports_interrupts,
	.supports_alt_function = _gpio_supports_alt_function,
	
	.set_line_config = _gpio_line_config,
	.set_alt_function = _gpio_set_alt_function,
	.set_interrupt_config = _gpio_interrupt_config,
	.set_trigger_positive = _gpio_set_trigger_positive,
	.set_trigger_negative = _gpio_set_trigger_negative,
	.set_gpe = _gpio_set_gpe,
	.set_smi = _gpio_set_smi,
	.set_line = _gpio_set_line,

	.get_number_of_gpios = _gpio_get_number_of_gpios,
	.get_number_of_gpio_interrupts = _gpio_get_number_of_gpio_interrupts,
	.get_interrupt_status = _gpio_interrupt_status,
	.get_interrupt_info = _gpio_get_interrupt_info,
	.get_line = _gpio_get_line,
	.get_ts = _gpio_get_ts,
	
	.clear_interrupt = _gpio_clear_interrupt,
	.clear_ts = _gpio_clear_ts,
	
	.enable_interrupt = _gpio_enable_interrupts,
	.disable_interrupt = _gpio_disable_interrupts,

	.wait_irq = _gpio_wait_for_irq,
	.ack_irq = _gpio_ack_irq,
	.handle_irq = _gpio_irq_handler,
	
	.create_event = _gpio_event_create,
	.destroy_event = _gpio_event_destroy,
	.set_events = _gpio_set_events,
	.reset_events = _gpio_reset_events,

	.request_irq = _gpio_os_acquire_interrupt,
};

static __inline unsigned int
IDL_REG_READ_IO (unsigned short int port)
{
  unsigned int _v;
  __asm__ __volatile__ ("inl %w1,%0\noutb %%al,$0x80":"=a" (_v):"Nd" (port));
  return _v;
}
static __inline void
IDL_REG_WRITE_IO (unsigned short int port, unsigned int value)
{
  __asm__ __volatile__ ("outl %0,%w1\noutb %%al,$0x80": :"a" (value),
                        "Nd" (port));
}
static os_sema_t m_gpio_sema;
static os_sema_t m_gpio_semb;
static os_sema_t m_gpio_semc;
static os_sema_t m_gpio_semd;

extern uint32_t gpio_irq;

int
_gpio_init( struct idl_gpio_host *host)
{
	int ret = 0;

	os_sema_init(&m_gpio_sema, 1);
	os_sema_init(&m_gpio_semb, 1);
	os_sema_init(&m_gpio_semc, 1);
	os_sema_init(&m_gpio_semd, 1);
	host->iomem_addr = (phys_addr_t) _gpio_get_base_addr();
	host->iomem_size= _gpio_get_size();
	host->vaddr = OS_MAP_IO_TO_MEM_NOCACHE(host->iomem_addr, host->iomem_size);
	if (NULL == host->vaddr) {
		ret = -ENOMEM;
	}
	return ret;
}

int
_gpio_exit(struct idl_gpio_host *host)
{
	OS_UNMAP_IO_FROM_MEM(host->vaddr, host->iomem_size);
	os_sema_destroy(&m_gpio_sema);
	os_sema_destroy(&m_gpio_semb);
	os_sema_destroy(&m_gpio_semc);
	os_sema_destroy(&m_gpio_semd);
	return 0;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen3.
*/
bool
_valid_gpio_num(uint32_t gpio_num)
{
	if (gpio_num > (uint32_t)GEN3_GPIO_NUM_MAX) 
	{
		return false;
	}
	return true;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen3 NB.
*/
bool
_valid_gpio_num_NB(uint32_t gpio_num)
{
	if (gpio_num > (uint32_t)GEN3_GPIO_NUM_MAX || gpio_num < GEN3_GPIO_GROUP_ONE) 
	{
		return false;
	}
	return true;
}

#if 0 //no polarity support in intel_gen3
/**
* This function checks to see if the specified gpio number supports polarity inversion.
*/
bool
_gpio_supports_polarity(uint32_t gpio_num)
{
	// On intel_gen3, GPIOs 8 - 103 support polarity (groups 1-3)
	if ((gpio_num < (uint32_t)GEN3_GPIO_GROUP_ONE) || 
		(gpio_num > (uint32_t)GEN3_GPIO_NUM_MAX))
	{
		return false;
	}
	return true;
}
#endif

/**
* This function checks to see if the specified gpio number supports interrupts. On intel_gen3
* only gpios 0 - 11 can be configured for interrupt support. 
*/
bool
_gpio_supports_interrupts(uint32_t gpio_num)
{
	// On intel_gen3, GPIOs 0 - 11 support interrupts (group 0)
	if (gpio_num >= (uint32_t)GEN3_GPIO_GROUP_ONE)
	{
		return false;
	}
	return true;
}

/** 
* This function checks to see whether or not the gpio supports alternate 
* functions.
*/
bool 
_gpio_supports_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	// On intel_gen3, GPIOs 0-11 and  15-21 support alternate functions 

	if (((gpio_num >= (uint32_t)GEN3_GPIO_GROUP_ONE)&& (gpio_num < (uint32_t)GEN3_GPIO_GROUP_TWO)) || 
		((gpio_num >= (uint32_t)GEN3_GPIO_GROUP_THREE)&&(gpio_num <= (uint32_t)GEN3_GPIO_NUM_MAX)))
	{
		return false;
	}
	else if (fn_num != 0 && fn_num != 1)
	{
		return false;
	}
	return true;
}

/**
* This function configures a gpio line on intel_gen3.
*/
void
_gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					  uint32_t gpio_config)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t addrio =(uint32_t)GEN3_GPIO_IO_BASE;
	uint32_t val, bit_num = 0;

	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
	if (gpio_num < GEN3_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_MUX_CNTL));
		switch(gpio_num){
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
				bit_num = 0;
				break;
			case 6:
			case 7:
				bit_num = 1;
				break;
			case 8:
			case 9:
			case 10:
			case 11:
				bit_num = 2;
				break;
		}

		// set gpio to act as gpio, not alt function
		IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_MUX_CNTL), (val & ~(1 << bit_num)));

		// set direction
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_GPOER));
		bit_num = gpio_num - GEN3_GPIO_GROUP_ZERO;
		if (gpio_config & IDL_GPIO_OUTPUT)
		{
			// set bit
			IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPOER), (val | (1 << bit_num)));
		}
		else
		{
			// clear bit
			IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPOER), (val & ~(1 << bit_num)));
		}
		os_sema_put(&m_gpio_sema);
	}
	else if (gpio_num < GEN3_GPIO_GROUP_TWO)
	{
		os_sema_get(&m_gpio_semb);
		
		//enable this port to gpio mode
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGEN);

		val |= 1<< (gpio_num-GEN3_GPIO_GROUP_ONE);
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGEN, val);

		// set direction
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGIO);
 
		bit_num = gpio_num-GEN3_GPIO_GROUP_ONE;
		if (gpio_config & IDL_GPIO_OUTPUT)
		{
			// clear bit
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGIO, (val & ~(1 << bit_num)));
		}
		else
		{
			// set bit
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGIO, (val | (1 << bit_num)));
		}
		os_sema_put(&m_gpio_semb);
	}
	else if (gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		os_sema_get(&m_gpio_semc);
		
		//enable this port to gpio mode
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO2_CGEN);
		val |= 1<< (gpio_num-GEN3_GPIO_GROUP_ONE);//use group one here
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO2_CGEN, val);
		
		//set to gpio mode
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO2_MUX_CNTL));
		switch(gpio_num-GEN3_GPIO_GROUP_TWO+3){
			case 3:
			case 4:
				bit_num = 3;
				break;
			case 5:
			case 6:
			case 7:
				bit_num = 4;
				break;
			case 8:
				bit_num = 5;
				break;
			case 9:
				bit_num = 6;
				break;
		}				

		IDL_REG32_WRITE((void*)(addr + GEN3_GPIO2_MUX_CNTL), (val & ~(1 << bit_num)));

		// set direction
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO2_CGIO);
		bit_num = gpio_num-GEN3_GPIO_GROUP_ONE;//use group one here

		if (gpio_config & IDL_GPIO_OUTPUT)
		{
			// clear bit
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO2_CGIO, (val & ~(1 << bit_num)));
		}
		else
		{
			// set bit
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO2_CGIO, (val | (1 << bit_num)));
		}
		
		os_sema_put(&m_gpio_semc);
	}
	else	// group 3
	{
		os_sema_get(&m_gpio_semd);
		
		//enable this port to gpio mode
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGEN);
		val |= 1<< (gpio_num-GEN3_GPIO_GROUP_THREE);
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGEN, val);

		// set direction
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGIO);
		bit_num = gpio_num-GEN3_GPIO_GROUP_THREE;
		if (gpio_config & IDL_GPIO_OUTPUT)
		{
			// clear bit
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGIO, (val & ~(1 << bit_num)));
		}
		else
		{
			// set bit
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGIO, (val | (1 << bit_num)));
		}
		os_sema_put(&m_gpio_semd);
	}
}

/**
* This function will set the pin associated with the gpio_num to it's alternate 
* function. 
*/
void 
_gpio_set_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num)
{

	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t val, bit_num =0;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num < GEN3_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_MUX_CNTL));
		switch(gpio_num){
			case 0:
			case 1:
			case 2:
			case 3:
			case 4:
			case 5:
				bit_num = 0;
				break;
			case 6:
			case 7:
				bit_num = 1;
				break;
			case 8:
			case 9:
			case 10:
			case 11:
				bit_num = 2;
				break;
		}

		if (1 == fn_num)
		    IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_MUX_CNTL), (val | (1 << bit_num)));
		else
		    IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_MUX_CNTL), (val & ~(1 << bit_num)));

		os_sema_put(&m_gpio_sema);
	}
	else if (((gpio_num >= (uint32_t)GEN3_GPIO_GROUP_TWO)&& (gpio_num < (uint32_t)GEN3_GPIO_GROUP_THREE)))
	{
		os_sema_get(&m_gpio_semc);
		//clear this port's gpio mode
		val = IDL_REG_READ_IO(addrio + GEN3_GPIO2_CGEN);
		val &= ~(1<< (gpio_num-GEN3_GPIO_GROUP_ONE));//use group one here
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO2_CGEN, val);
		
		//set to alt mode
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO2_MUX_CNTL));
		switch(gpio_num-GEN3_GPIO_GROUP_TWO+3){
			case 3:
			case 4:
				bit_num = 3;
				break;
			case 5:
			case 6:
			case 7:
				bit_num = 4;
				break;
			case 8:
				bit_num = 5;
				break;
			case 9:
				bit_num = 6;
				break;
		}				

		if (1 == fn_num)
		    IDL_REG32_WRITE((void*)(addr + GEN3_GPIO2_MUX_CNTL), (val | (1 << bit_num)));
		else
		    IDL_REG32_WRITE((void*)(addr + GEN3_GPIO2_MUX_CNTL), (val & ~(1 << bit_num)));

		os_sema_put(&m_gpio_semc);
	}
}

/** 
* This function configures the type of interrupt that the GPIO line detects.
*/
void
_gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						   idl_gpio_interrupt_type_t interrupt_type)
{
	uintptr_t addr;
	uint32_t val, clr_mask, set_mask;
	
	os_sema_get(&m_gpio_sema);
	
	addr = (uintptr_t)m_idl_gpio_base;
	if(gpio_num <= 7){
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_GPIT1R0));
		// clear the 4 bits associated with the interrupt being set
		clr_mask = ~(0xF << 4*gpio_num);
		set_mask = (interrupt_type << 4*gpio_num);

		val &= clr_mask;
		val |= set_mask;

		IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPIT1R0), val);
	}
	else{
		val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_GPIT1R1));
		gpio_num -= 8;
		// clear the 4 bits associated with the interrupt being set
		clr_mask = ~(0xF << 4*gpio_num);
		set_mask = (interrupt_type << 4*gpio_num);

		val &= clr_mask;
		val |= set_mask;

		IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPIT1R1), val);
	}
	os_sema_put(&m_gpio_sema);
}

/** 
* This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
*/
void
_gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
						   uint32_t *interrupt_status)
{
	uint32_t val;
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_GPSTR));
	val &= (1 << gpio_num);
	*interrupt_status = val >> gpio_num;
}

/** 
* This function clears the interrupt bit for the gpio number specified.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void
_gpio_clear_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uintptr_t addr;
	os_sema_get(&m_gpio_sema);

	// This register is a write-one to clear register
	addr = (uintptr_t)m_idl_gpio_base;
	IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPSTR), (1 << gpio_num));

//	val = IDL_REG32_READ(addr + GEN3_GPIO0_GPSTR);
	os_sema_put(&m_gpio_sema);
}

/** 
* This function sets the line state for a GPIO line. 
*/
void
_gpio_set_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t val)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t curr_reg_val;
	uint32_t addrio = GEN3_GPIO_IO_BASE;
	
	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
	if (gpio_num < GEN3_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
		curr_reg_val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_GPOUTR));
		
		if (val == 0)
		{
			IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPOUTR), 
			(curr_reg_val & (~(1 << gpio_num))));
		}
		else
		{
			IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_GPOUTR), 
			  (curr_reg_val | (1 << gpio_num)));
		}
		os_sema_put(&m_gpio_sema);
	}
	else if (gpio_num < GEN3_GPIO_GROUP_TWO)
	{
		os_sema_get(&m_gpio_semb);
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGLV);
		if (val == 0)
		{
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGLV, (curr_reg_val & (~(1 << (gpio_num-GEN3_GPIO_GROUP_ONE)))));
		}
		else
		{
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGLV, (curr_reg_val | (1 << (gpio_num-GEN3_GPIO_GROUP_ONE))));
		}
		os_sema_put(&m_gpio_semb);
	}
	else if (gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		os_sema_get(&m_gpio_semc);
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO2_CGLV);
		if (val == 0)
		{
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO2_CGLV, (curr_reg_val & (~(1 << (gpio_num-GEN3_GPIO_GROUP_ONE)))));
		}
		else
		{
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO2_CGLV, (curr_reg_val | (1 << (gpio_num-GEN3_GPIO_GROUP_ONE))));
		}
		os_sema_put(&m_gpio_semc);
	}
	else	// group 3
	{
		os_sema_get(&m_gpio_semd);
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGLV);
		if (val == 0)
		{
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGLV, (curr_reg_val & (~(1 << (gpio_num-GEN3_GPIO_GROUP_THREE)))));
		}
		else
		{
			IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGLV, (curr_reg_val | (1 << (gpio_num-GEN3_GPIO_GROUP_THREE))));
		}
		os_sema_put(&m_gpio_semd);
	}
}

/** 
* Reads the line state for the GPIO.
*/
void
_gpio_get_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t *val)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
	if (gpio_num < GEN3_GPIO_GROUP_ONE)
	{
		curr_reg_val = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_GPINR));
		*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
	}
	else if (gpio_num < GEN3_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGLV);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		*val = ((curr_reg_val & (1 << bit_num)) >> bit_num);
	}
	else if (gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO2_CGLV);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		*val = ((curr_reg_val & (1 << bit_num)) >> bit_num);
	}
	else	// group 3
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGLV);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		*val = ((curr_reg_val & (1 << bit_num)) >> bit_num);
	}
}

void _gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num >= GEN3_GPIO_GROUP_ONE && gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGTPE);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGTPE, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGTPE, (curr_reg_val |(1 << bit_num)));
	}
	else if (gpio_num >= GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGTPE);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGTPE, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGTPE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num >= GEN3_GPIO_GROUP_ONE && gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGTNE);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGTNE, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGTNE, (curr_reg_val |(1 << bit_num)));
	}
	else if (gpio_num >= GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGTNE);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGTNE, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGTNE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gpio_set_gpe(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num >= GEN3_GPIO_GROUP_ONE && gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGGPE);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGGPE, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGGPE, (curr_reg_val |(1 << bit_num)));
	}
	else if (gpio_num >= GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGGPE);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGGPE, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGGPE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gpio_set_smi(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num >= GEN3_GPIO_GROUP_ONE && gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGSMI);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGSMI, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGSMI, (curr_reg_val |(1 << bit_num)));
	}
	else if (gpio_num >= GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGSMI);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		if(setting == 0)
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGSMI, (curr_reg_val & ~(1 << bit_num)));
		else
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGSMI, (curr_reg_val |(1 << bit_num)));
	}
}

void _gpio_get_ts(uint32_t gpio_num, uint32_t *setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num >= GEN3_GPIO_GROUP_ONE && gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGTS);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		*setting = (curr_reg_val & (1<<bit_num))>>bit_num;
	}
	else if (gpio_num >= GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGTS);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		*setting = (curr_reg_val & (1<<bit_num))>>bit_num;
	}
}

void _gpio_clear_ts(uint32_t gpio_num)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN3_GPIO_IO_BASE;

	if (gpio_num >= GEN3_GPIO_GROUP_ONE && gpio_num < GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO1_CGTS);
		bit_num = gpio_num - GEN3_GPIO_GROUP_ONE;
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO1_CGTS, (curr_reg_val |(1 << bit_num)));
	}
	else if (gpio_num >= GEN3_GPIO_GROUP_THREE)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN3_GPIO3_CGTS);
		bit_num = gpio_num - GEN3_GPIO_GROUP_THREE;
		IDL_REG_WRITE_IO(addrio + GEN3_GPIO3_CGTS, (curr_reg_val |(1 << bit_num)));
	}
}

uint32_t _gpio_get_base_addr()
{
#ifdef HARDCODE_BAR
	return GEN3_GPIO_MEM_BASE;
#else		
	uint32_t phyad = 0;
	os_pci_dev_t  pci_dev = NULL;

	//get the device  GPIO
	if(os_pci_device_from_address(&pci_dev, GEN3_GPIO_PCI_BUS_NUM, GEN3_GPIO_PCI_DEVICE_NUM, GEN3_GPIO_PCI_FUNC_NUM) != OSAL_SUCCESS)
	{
		OS_INFO("Unable to access the PCI DEVICE GPIO\n");
		return false;
	}
	//read the GPIO_0 base address from bar 0 
	os_pci_read_config_32(pci_dev, 0x10, &phyad);	
	OS_PCI_FREE_DEVICE(pci_dev);
    return phyad;
#endif

}

uint32_t _gpio_get_size()
{
	return GEN3_GPIO_MEM_SIZE;
}

/*os_interrupt_t _gpio_register_interrupt_handler(os_interrupt_handler_t *handler, 
												 void *data)
{
	return os_acquire_interrupt(GEN3_GPIO_IRQ, GPIO_0, "IDL GPIO",
								handler, data);
}
*/
uint32_t _gpio_get_number_of_gpios()
{
	return GEN3_NUM_GPIO_PINS;
}

uint32_t _gpio_get_number_of_gpio_interrupts()
{
	return GEN3_GPIO_NUM_INT;
}

void _gpio_disable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uint32_t temp;
	uint32_t val = 0;
	uintptr_t irq_base;
        uintptr_t addr = (uintptr_t)m_idl_gpio_base;

	os_sema_get(&m_gpio_sema);

	temp = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_INT));
	IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_INT), temp&(~(1<<gpio_num)));

	os_sema_put(&m_gpio_sema);

	if(0==(temp&0xfff)){
	irq_base = (uintptr_t)OS_MAP_IO_TO_MEM_NOCACHE(GEN3_IRQ_BASE, GEN3_IRQ_SIZE);

	os_sema_get(&m_gpio_sema);

	val = IDL_REG32_READ((void*)(irq_base + GEN3_IRQ_MASK_OFFSET));
	
	/* clear all gpio interrupt bits */
	val |= (1 << GEN3_GPIO_IRQ_MASK_OFFSET);
	
	IDL_REG32_WRITE((void*)(irq_base + GEN3_IRQ_MASK_OFFSET), val);
	os_sema_put(&m_gpio_sema);

	OS_UNMAP_IO_FROM_MEM((void *)irq_base, GEN3_IRQ_SIZE);
	}
}

void _gpio_enable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uint32_t temp;
	uint32_t val = 0;
	uintptr_t irq_base;
        uintptr_t addr = (uintptr_t)m_idl_gpio_base;

	os_sema_get(&m_gpio_sema);
	temp = IDL_REG32_READ((void*)(addr + GEN3_GPIO0_INT));
	IDL_REG32_WRITE((void*)(addr + GEN3_GPIO0_INT), temp|(1<<gpio_num));
	os_sema_put(&m_gpio_sema);
	
	irq_base = (uintptr_t)OS_MAP_IO_TO_MEM_NOCACHE(GEN3_IRQ_BASE, GEN3_IRQ_SIZE);

	os_sema_get(&m_gpio_sema);
	val = IDL_REG32_READ((void*)(irq_base + GEN3_IRQ_MASK_OFFSET));
	
	/* enable all gpio interrupt bits */
	val &= ~(1 << GEN3_GPIO_IRQ_MASK_OFFSET);
	
	IDL_REG32_WRITE((void*)(irq_base + GEN3_IRQ_MASK_OFFSET), val);
	os_sema_put(&m_gpio_sema);

	OS_UNMAP_IO_FROM_MEM((void *)irq_base, GEN3_IRQ_SIZE);
}

void _gpio_get_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey)
{
	*irq = gpio_irq;
	*devkey = 0;
}

void _gpio_event_create(void)
{
	int i;
	for (i = 0; i < GEN3_GPIO_NUM_INT; i++) {
//		init_waitqueue_head(&listening_q[i]);	
		os_event_create(&gen3_gpio_interrupt_start[i], 0);
		os_event_create(&gen3_gpio_interrupt_done[i], 0);
	}
}

void _gpio_event_destroy(void)
{
	int i;
	for (i = 0; i < GEN3_GPIO_NUM_INT; i++){
		os_event_destroy(&gen3_gpio_interrupt_start[i]);
		os_event_destroy(&gen3_gpio_interrupt_done[i]);
	} 
}

int _gpio_wait_for_irq(unsigned int gpio_num)
{
	int status = 0;
	if (os_event_wait(&gen3_gpio_interrupt_start[gpio_num], -1) != OSAL_SUCCESS) {
		//status = -ERESTARTSYS;
		status = -512;
	}
	else {
#ifdef GEN3_DEBUG
		printk(KERN_INFO "Interrupt received..\n");
#endif
		os_event_reset(&gen3_gpio_interrupt_start[gpio_num]);
	}
	return status;
}

void _gpio_irq_handler(int gpio_num)
{
	gen3_irq_pending[gpio_num] = 1;
	os_event_set(&gen3_gpio_interrupt_start[gpio_num]);
	if (os_event_wait(&gen3_gpio_interrupt_done[gpio_num], -1) == OSAL_SUCCESS) {
		os_event_reset(&gen3_gpio_interrupt_done[gpio_num]);
	}
}

void _gpio_ack_irq(uint32_t gpio_num)
{
	gen3_irq_pending[gpio_num] = 0;
	os_event_set(&gen3_gpio_interrupt_done[gpio_num]);
}

void _gpio_set_events(uint32_t gpio_num)
{
	os_event_set(&gen3_gpio_interrupt_start[gpio_num]);
	os_event_set(&gen3_gpio_interrupt_done[gpio_num]);
}

void _gpio_reset_events(uint32_t gpio_num)
{
	os_event_set(&gen3_gpio_interrupt_start[gpio_num]);
	os_event_set(&gen3_gpio_interrupt_done[gpio_num]);
}

void _gpio_irq_release(int val)
{
	gen3_irq_release = val;
}

void _gpio_irq_set_irq_params(unsigned long gpio_num)
{
		gen3_irq_params[gpio_num] = gpio_num;
}

os_interrupt_t _gpio_os_acquire_interrupt(int irq, int devkey, os_interrupt_handler_t *handler, void *data)
{
	return os_acquire_interrupt(irq, 
			devkey, "IDL GPIO", handler, data);
}

uint32_t * _gpio_irq_get_irq_params(unsigned long gpio_num)
{
	return &gen3_irq_params[gpio_num];
}
