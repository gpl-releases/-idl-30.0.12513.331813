/*
#
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2009-2012 Intel Corporation. All rights reserved.
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
#  Copyright(c) 2009-2012 Intel Corporation. All rights reserved.
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
 * File Name:       _gpio_gen4.5.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 */
#include <linux/types.h>
#include "osal.h"
#include "_gpio_gen4_5.h"

//#include <linux/wait.h>	//For ERESTARTSYS
//here is io space operations, should put appropriate

//#define GEN4D5_DEBUG 1

/* Events used to signal the start and completion of interrupt 
 * handling.\n */
os_event_t gen4d5_gpio_interrupt_start[GEN4D5_GPIO_NUM_INT];
os_event_t gen4d5_gpio_interrupt_done[GEN4D5_GPIO_NUM_INT];
  
int gen4d5_irq_pending[GEN4D5_GPIO_NUM_INT] = {0};
uint32_t gen4d5_irq_params[GEN4D5_GPIO_NUM_INT] = {0};
int gen4d5_irq_release = 0;

#define GEN4D5_GPIO_PCI_BUS_NUM 1
#define GEN4D5_GPIO_PCI_DEVICE_NUM 11
#define GEN4D5_GPIO_PCI_FUNC_NUM 1
struct idl_gpio_host ce4200_gpio_host = {
	.init = _gen4d5_gpio_init,
	.exit = _gen4d5_gpio_exit,
	
	.is_valid = _gen4d5_valid_gpio_num,
	.is_in_NB = _gen4d5_valid_gpio_num_NB,
	.supports_interrupt = _gen4d5_gpio_supports_interrupts,
	.supports_alt_function = _gen4d5_gpio_supports_alt_function,
	
	.set_line_config = _gen4d5_gpio_line_config,
	.set_alt_function = _gen4d5_gpio_set_alt_function,
	.set_interrupt_config = _gen4d5_gpio_interrupt_config,
	.set_trigger_positive = _gen4d5_gpio_set_trigger_positive,
	.set_trigger_negative = _gen4d5_gpio_set_trigger_negative,
	.set_gpe = _gen4d5_gpio_set_gpe,
	.set_smi = _gen4d5_gpio_set_smi,
	.set_line = _gen4d5_gpio_set_line,
				 
	.get_number_of_gpios = _gen4d5_gpio_get_number_of_gpios,
	.get_number_of_gpio_interrupts = _gen4d5_gpio_get_number_of_gpio_interrupts,
	.get_interrupt_status = _gen4d5_gpio_interrupt_status,
	.get_interrupt_info = _gen4d5_gpio_get_interrupt_info,
	.get_line = _gen4d5_gpio_get_line,
	.get_ts = _gen4d5_gpio_get_ts,

	.clear_interrupt = _gen4d5_gpio_clear_interrupt,
	.clear_ts = _gen4d5_gpio_clear_ts,
	
	.enable_interrupt = _gen4d5_gpio_enable_interrupts,
	.disable_interrupt = _gen4d5_gpio_disable_interrupts,

	.wait_irq = _gen4d5_gpio_wait_for_irq,
	.handle_irq = _gen4d5_gpio_irq_handler,
	.ack_irq = _gen4d5_gpio_ack_irq,

	.create_event = _gen4d5_gpio_event_create,
	.destroy_event = _gen4d5_gpio_event_destroy,
	.set_events = _gen4d5_gpio_set_events,
	.reset_events = _gen4d5_gpio_reset_events,
	
	.request_irq = _gen4d5_gpio_os_acquire_interrupt,	

	.suspend = _gen4d5_gpio_suspend,
	.resume = _gen4d5_gpio_resume,
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
_gen4d5_gpio_init(struct idl_gpio_host *host)
{
	int ret = 0;

	os_sema_init(&m_gpio_sema, 1);
	os_sema_init(&m_gpio_semb, 1);
	os_sema_init(&m_gpio_semc, 1);
	os_sema_init(&m_gpio_semd, 1);
	host->iomem_addr = (phys_addr_t) _gen4d5_gpio_get_base_addr();
	host->iomem_size= _gen4d5_gpio_get_size();
	host->vaddr = OS_MAP_IO_TO_MEM_NOCACHE(host->iomem_addr, host->iomem_size);
	if (NULL == host->vaddr) {
		ret = -ENOMEM;
	}
	return ret;
}

int
_gen4d5_gpio_exit(struct idl_gpio_host *host)
{
	OS_UNMAP_IO_FROM_MEM(host->vaddr, host->iomem_size);
	os_sema_destroy(&m_gpio_sema);
	os_sema_destroy(&m_gpio_semb);
	os_sema_destroy(&m_gpio_semc);
	os_sema_destroy(&m_gpio_semd);
	return 0;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen4.5.
*/
bool
_gen4d5_valid_gpio_num(uint32_t gpio_num)
{
	if (gpio_num > (uint32_t)GEN4D5_GPIO_NUM_MAX) 
	{
		return false;
	}
	return true;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen4.5 NB.
*/
bool
_gen4d5_valid_gpio_num_NB(uint32_t gpio_num)
{
	if (gpio_num > (uint32_t)GEN4D5_GPIO_NUM_MAX || gpio_num < GEN4D5_GPIO_GROUP_ONE) 
	{
		return false;
	}
	return true;
}


/**
* This function checks to see if the specified gpio number supports interrupts. On intel_gen4.5
* only gpios 0 - 11 can be configured for interrupt support. 
*/
bool
_gen4d5_gpio_supports_interrupts(uint32_t gpio_num)
{
	// On intel_gen4.5, GPIOAUX [4:0] support adjustable interrupts (group 0)
	//GPIOAUX [77:5] do not support adjustable intterrupts
	if (gpio_num >= (uint32_t)GEN4D5_GPIO_NUM_INT)
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
_gen4d5_gpio_supports_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	// On intel_gen4.5, GPIOAUX[0] supports alternate functions 

	/*
	0 --- GBE_LINK /GPIO_AUX[21]  -> GPIO[0]
	1 --- Smart Card 0 / TSD_ICAM -> GPIO[1]
	2 --- Smart Card 1 / NAND_CE_N, NAND_RE_N -> GPIO[2]
	3 --- UART0_DSRB_I2S0_IN_BCK_GPIO_24 -> GPIO[3]
	4 --- UART0_DTRB_I2S0_IN_MCLK_GPIO_25 -> GPIO[4]
	5 --- UART0_DCDB_I2S1_IN_BCK_GPIO_26 -> GPIO[5]
	6 --- UART0_RIB_SPDIF_IN_GPIO_27 -> GPIO[6]
	7 --- UART0_RTSB_I2S1_IN_SDATA_3_GPIO_28 -> GPIO[7]
	8 --- UART0_CTSB_I2S1_IN_SDATA_2_GPIO_29 -> GPIO[8]
	9 --- UART1_RXD and UART1_TXD enable / UART1_RXD and UART1_TXD disable -> GPIO[9]
	*/
	
	if (9 < gpio_num)
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
* This function configures a gpio line on intel_gen4.5.
*/
void
_gen4d5_gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					  uint32_t gpio_config)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t addrio =(uint32_t)GEN4D5_GPIO_IO_BASE;
	uint32_t val, bit_num = 0;

	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
#ifdef GEN4D5_DEBUG
	printk("LINE:%d, gpio_num=0x%x, addr=0x%x\n", __LINE__, gpio_num, addr);
#endif
	if (gpio_num < GEN4D5_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
		if(0 == gpio_num) {
			bit_num = 4;
			val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_MUX_CNTL));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			// set gpio to act as gpio, not alt function
			IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_MUX_CNTL), (val & ~(1 << bit_num)));
		}
		
		if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
			bit_num = gpio_num - 0;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOER_0));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOER_0), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOER_0), (val & ~(1 << bit_num)));
			}
		}
		else if(gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
			bit_num = gpio_num - GEN4D5_GPIO0_GROUP_0;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOER_1));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOER_1), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOER_1), (val & ~(1 << bit_num)));
			}
		}
		else { //if (gpio_num >= GEN4D5_GPIO0_GROUP_1 && gpio_num < GEN4D5_GPIO0_GROUP_2) 
			bit_num = gpio_num - GEN4D5_GPIO0_GROUP_1;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOER_2));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOER_2), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOER_2), (val & ~(1 << bit_num)));
			}
		}
		os_sema_put(&m_gpio_sema);
	}
	else if (gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		os_sema_get(&m_gpio_semb);
		
		//enable this port to gpio mode
		val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGEN);
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
		val |= 1<< (gpio_num-GEN4D5_GPIO_GROUP_ONE);
		IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGEN, val);

		// set direction
		val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGIO);
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
		bit_num = gpio_num-GEN4D5_GPIO_GROUP_ONE;
		if (gpio_config & IDL_GPIO_OUTPUT)
		{
			// clear bit
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGIO, (val & ~(1 << bit_num)));
		}
		else
		{
			// set bit
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGIO, (val | (1 << bit_num)));
		}
		os_sema_put(&m_gpio_semb);
	}
}

/**
* This function will set the pin associated with the gpio_num to it's alternate 
* function. 
*/
void 
_gen4d5_gpio_set_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num)
{

	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t val, bit_num = 0;

	if (gpio_num < 10)
	{
		os_sema_get(&m_gpio_sema);
		val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_MUX_CNTL));
		//printk("before set val =0x%x\n", val);
		switch(gpio_num) {
			case 0:
				bit_num = 4;
				break;
			case 1:
				bit_num = 3;
				break;
			case 2:
				bit_num = 2;
				break;
			case 3:
			case 4:
			case 5:
			case 6:
			case 7:
			case 8:	
				bit_num = 0;
				break;
			case 9:
				bit_num = 1;
				break;
			default:
				bit_num = 0;
				break;
		}

		if (1 == fn_num)
		    IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_MUX_CNTL), (val | (1 << bit_num)));
		else
		    IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_MUX_CNTL), (val & ~(1 << bit_num)));
		val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_MUX_CNTL));
		//printk("after set val =0x%x\n", val);

		os_sema_put(&m_gpio_sema);
	}

}

/** 
* This function configures the type of interrupt that the GPIO line detects.
*/
void
_gen4d5_gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						   idl_gpio_interrupt_type_t interrupt_type)
{
	uintptr_t addr;
	uint32_t val, clr_mask, set_mask;
	
	os_sema_get(&m_gpio_sema);
	
	addr = (uintptr_t)m_idl_gpio_base;
	if(gpio_num < GEN4D5_GPIO_ADJ_NUM_INT){
		val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPIT1R0));
		// clear the 4 bits associated with the interrupt being set
		clr_mask = ~(0xF << 4*gpio_num);
		set_mask = (interrupt_type << 4*gpio_num);

		val &= clr_mask;
		val |= set_mask;

		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPIT1R0), val);
	}
	else {
		printk("GPIO[77:5] is not adjustable\n");
	}
	os_sema_put(&m_gpio_sema);
}

/** 
* This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
*/
void
_gen4d5_gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
						   uint32_t *interrupt_status)
{
	uint32_t val, bit_num;
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPSTR_0));
		val &= (1 << bit_num);
		*interrupt_status = val >> bit_num;
	}
	else if(gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_0;
		val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPSTR_1));
		val &= (1 << bit_num);
		*interrupt_status = val >> bit_num;
	}
	else {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_1;
		val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPSTR_2));
		val &= (1 << bit_num);
		*interrupt_status = val >> bit_num;
	}
}

/** 
* This function clears the interrupt bit for the gpio number specified.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void
_gen4d5_gpio_clear_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uintptr_t addr, bit_num;
	os_sema_get(&m_gpio_sema);

	// This register is a write-one to clear register
	addr = (uintptr_t)m_idl_gpio_base;
	if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPSTR_0), (1 << bit_num));
	}
	else if(gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_0;
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPSTR_1), (1 << bit_num));
	}
	else {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_1;
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPSTR_2), (1 << bit_num));
	}
	os_sema_put(&m_gpio_sema);
}

/** 
* This function sets the line state for a GPIO line. 
*/
void
_gen4d5_gpio_set_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t val)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t curr_reg_val;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;
	
	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
#ifdef GEN4D5_DEBUG
	printk("LINE:%d, gpio_num=0x%x, addr=0x%x\n", __LINE__, gpio_num, addr);
#endif
	if (gpio_num < GEN4D5_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
		
		if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
			gpio_num -= 0;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOUTR_0));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOUTR_0), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOUTR_0), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN4D5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOUTR_0));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif
		}
		else if (gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
			gpio_num -= GEN4D5_GPIO0_GROUP_0;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOUTR_1));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOUTR_1), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOUTR_1), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN4D5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOUTR_1));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
		}
		else { //if (gpio_num >= GEN4D5_GPIO0_GROUP_1 && gpio_num < GEN4D5_GPIO0_GROUP_2) 
			gpio_num -= GEN4D5_GPIO0_GROUP_1;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOUTR_2));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOUTR_2), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_GPOUTR_2), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN4D5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPOUTR_2));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
		}
		os_sema_put(&m_gpio_sema);
	}
	else if (gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		os_sema_get(&m_gpio_semb);
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGLV);
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
		if (val == 0) {
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGLV, (curr_reg_val & (~(1 << (gpio_num-GEN4D5_GPIO_GROUP_ONE)))));
		}
		else {
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGLV, (curr_reg_val | (1 << (gpio_num-GEN4D5_GPIO_GROUP_ONE))));
		}
#ifdef GEN4D5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO1_CGLV));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif				
		os_sema_put(&m_gpio_semb);
	}
}

/** 
* Reads the line state for the GPIO.
*/
void
_gen4d5_gpio_get_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t *val)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
	if (gpio_num < GEN4D5_GPIO_GROUP_ONE)
	{
		if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPINR_0));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}
		else if(gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
			gpio_num -= GEN4D5_GPIO0_GROUP_0;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPINR_1));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}
		else { // if (gpio_num >= GEN4D5_GPIO0_GROUP_1 && gpio_num < GEN4D5_GPIO0_GROUP_2) 
			gpio_num -= GEN4D5_GPIO0_GROUP_1;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_GPINR_2));
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}
	}
	else if(gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGLV);
#ifdef GEN4D5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif	
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		*val = ((curr_reg_val & (1 << bit_num)) >> bit_num);
	}
}

void _gen4d5_gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	if (gpio_num >= GEN4D5_GPIO_GROUP_ONE && gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGTPE);
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGTPE, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGTPE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen4d5_gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	if (gpio_num >= GEN4D5_GPIO_GROUP_ONE && gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGTNE);
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGTNE, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGTNE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen4d5_gpio_set_gpe(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	if (gpio_num >= GEN4D5_GPIO_GROUP_ONE && gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGGPE);
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGGPE, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGGPE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen4d5_gpio_set_smi(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	if (gpio_num >= GEN4D5_GPIO_GROUP_ONE && gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGSMI);
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGSMI, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGSMI, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen4d5_gpio_get_ts(uint32_t gpio_num, uint32_t *setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	if (gpio_num >= GEN4D5_GPIO_GROUP_ONE && gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGTS);
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		*setting = (curr_reg_val & (1<<bit_num))>>bit_num;
	}
}

void _gen4d5_gpio_clear_ts(uint32_t gpio_num)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN4D5_GPIO_IO_BASE;

	if (gpio_num >= GEN4D5_GPIO_GROUP_ONE && gpio_num < GEN4D5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN4D5_GPIO1_CGTS);
		bit_num = gpio_num - GEN4D5_GPIO_GROUP_ONE;
		IDL_REG_WRITE_IO(addrio + GEN4D5_GPIO1_CGTS, (curr_reg_val |(1 << bit_num)));
	}
}

uint32_t _gen4d5_gpio_get_base_addr()
{
#ifdef HARDCODE_BAR
	return GEN4D5_GPIO_MEM_BASE;
#else
	uint32_t phyad = 0;
	os_pci_dev_t  pci_dev = NULL;

	//get the device  GPIO
	if(os_pci_device_from_address(&pci_dev, GEN4D5_GPIO_PCI_BUS_NUM, GEN4D5_GPIO_PCI_DEVICE_NUM, GEN4D5_GPIO_PCI_FUNC_NUM) != OSAL_SUCCESS)
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

uint32_t _gen4d5_gpio_get_size()
{
	return GEN4D5_GPIO_MEM_SIZE;
}

/*os_interrupt_t _gpio_register_interrupt_handler(os_interrupt_handler_t *handler, 
												 void *data)
{
	return os_acquire_interrupt(GEN4D5_GPIO_IRQ, GPIO_0, "IDL GPIO",
								handler, data);
}
*/
uint32_t _gen4d5_gpio_get_number_of_gpios()
{
	return GEN4D5_NUM_GPIO_PINS;
}

uint32_t _gen4d5_gpio_get_number_of_gpio_interrupts()
{
	return GEN4D5_GPIO_NUM_INT;
}

void _gen4d5_gpio_disable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uint32_t temp, bit_num;
	uint32_t val = 0;
	uintptr_t irq_base;
    uintptr_t addr = (uintptr_t)m_idl_gpio_base;

	os_sema_get(&m_gpio_sema);
	if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		temp = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_INTEN_0));
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_INTEN_0), temp&(~(1<<bit_num)));
	}
	else if(gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_0;
		temp = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_INTEN_1));
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_INTEN_1), temp&(~(1<<bit_num)));
	}
	else {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_1;
		temp = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_INTEN_2));
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_INTEN_2), temp&(~(1<<bit_num)));
	}
	os_sema_put(&m_gpio_sema);

	if( 0 == (temp & 0xfff)) {
		irq_base = (uintptr_t)OS_MAP_IO_TO_MEM_NOCACHE(GEN4D5_IRQ_BASE, GEN4D5_IRQ_SIZE);

		os_sema_get(&m_gpio_sema);

		val = IDL_REG32_READ((void*)(irq_base + GEN4D5_IRQ_MASK_OFFSET));
		
		/* clear all gpio interrupt bits */
		val |= (1 << GEN4D5_GPIO_IRQ_MASK_OFFSET);
		
		IDL_REG32_WRITE((void*)(irq_base + GEN4D5_IRQ_MASK_OFFSET), val);
		os_sema_put(&m_gpio_sema);

		OS_UNMAP_IO_FROM_MEM((void *)irq_base, GEN4D5_IRQ_SIZE);
	}
}

void _gen4d5_gpio_enable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uint32_t temp, bit_num;
	uint32_t val = 0;
	uintptr_t irq_base;
    uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	os_sema_get(&m_gpio_sema);
	
	if(gpio_num < GEN4D5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		temp = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_INTEN_0));
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_INTEN_0), temp|(1<<bit_num));
	}
	else if(gpio_num >= GEN4D5_GPIO0_GROUP_0 && gpio_num < GEN4D5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_0;
		temp = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_INTEN_1));
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_INTEN_1), temp|(1<<bit_num));
	}
	else {
		bit_num = gpio_num - GEN4D5_GPIO0_GROUP_1;
		temp = IDL_REG32_READ((void*)(addr + GEN4D5_GPIO0_INTEN_2));
		IDL_REG32_WRITE((void*)(addr + GEN4D5_GPIO0_INTEN_2), temp|(1<<bit_num));
	}
	os_sema_put(&m_gpio_sema);
	
	irq_base = (uintptr_t)OS_MAP_IO_TO_MEM_NOCACHE(GEN4D5_IRQ_BASE, GEN4D5_IRQ_SIZE);

	os_sema_get(&m_gpio_sema);
	val = IDL_REG32_READ((void*)(irq_base + GEN4D5_IRQ_MASK_OFFSET));
	
	/* enable all gpio interrupt bits */
	val &= ~(1 << GEN4D5_GPIO_IRQ_MASK_OFFSET);
	
	IDL_REG32_WRITE((void*)(irq_base + GEN4D5_IRQ_MASK_OFFSET), val);
	os_sema_put(&m_gpio_sema);

	OS_UNMAP_IO_FROM_MEM((void *)irq_base, GEN4D5_IRQ_SIZE);
}

void _gen4d5_gpio_get_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey)
{
	*irq = gpio_irq;
	*devkey = 0;
}

void _gen4d5_gpio_event_create()
{
	int i;
	for (i = 0; i < GEN4D5_GPIO_NUM_INT; i++) {
		os_event_create(&gen4d5_gpio_interrupt_start[i], 0);
		os_event_create(&gen4d5_gpio_interrupt_done[i], 0);
	}
}

void _gen4d5_gpio_event_destroy()
{
	int i;
	for (i = 0; i < GEN4D5_GPIO_NUM_INT; i++){
		os_event_destroy(&gen4d5_gpio_interrupt_start[i]);
		os_event_destroy(&gen4d5_gpio_interrupt_done[i]);
	} 
}

int _gen4d5_gpio_wait_for_irq(unsigned int gpio_num)
{
	int status = 0;
	if (os_event_wait(&gen4d5_gpio_interrupt_start[gpio_num], -1) != OSAL_SUCCESS) {
		//status = -ERESTARTSYS;
		status = -512;
	}
	else {
#ifdef GEN4D5_DEBUG
		printk(KERN_INFO "Interrupt received..\n");
#endif
		os_event_reset(&gen4d5_gpio_interrupt_start[gpio_num]);
	}
	return status;
}

void _gen4d5_gpio_irq_handler(int gpio_num)
{
	gen4d5_irq_pending[gpio_num] = 1;
	os_event_set(&gen4d5_gpio_interrupt_start[gpio_num]);
	if (os_event_wait(&gen4d5_gpio_interrupt_done[gpio_num], -1) == OSAL_SUCCESS) {
		os_event_reset(&gen4d5_gpio_interrupt_done[gpio_num]);
	}
}

void _gen4d5_gpio_ack_irq(uint32_t gpio_num)
{
	gen4d5_irq_pending[gpio_num] = 0;
	os_event_set(&gen4d5_gpio_interrupt_done[gpio_num]);
}

void _gen4d5_gpio_set_events(uint32_t gpio_num)
{
	os_event_set(&gen4d5_gpio_interrupt_start[gpio_num]);
	os_event_set(&gen4d5_gpio_interrupt_done[gpio_num]);
}

void _gen4d5_gpio_reset_events(uint32_t gpio_num)
{
	os_event_reset(&gen4d5_gpio_interrupt_start[gpio_num]);
	os_event_reset(&gen4d5_gpio_interrupt_done[gpio_num]);
}

void _gen4d5_gpio_irq_release(val)
{
	gen4d5_irq_release = val;
}

void _gen4d5_gpio_irq_set_irq_params(unsigned long gpio_num)
{
	gen4d5_irq_params[gpio_num] = gpio_num;
}

os_interrupt_t _gen4d5_gpio_os_acquire_interrupt(int irq, int devkey, os_interrupt_handler_t *handler, void *data)
{
	return os_acquire_interrupt(irq, 
			devkey, "IDL GPIO", handler, data);
}

uint32_t * _gen4d5_gpio_irq_get_irq_params(unsigned long gpio_num)
{
	return &gen4d5_irq_params[gpio_num];
}

struct gpio_group {
	uint32_t input;
	uint32_t output;
	uint32_t output_enable;
	uint32_t int_status;
	uint32_t int_enable;
	uint32_t int_mode;
};

struct _gpio { 
	uint32_t cgen;
	uint32_t cgio;
	uint32_t cglv;
	uint32_t cgtpe;
	uint32_t cgtne;
	uint32_t cggpe;
	uint32_t cgsmi;
	uint32_t cgts;

struct gpio_group group[3];
uint32_t gpio_mux_cntl;
};

static struct _gpio  _gpio;

/*CE4200 gpio suspend routine */
int _gen4d5_gpio_suspend(void *data)
{
	char *virt_io_mem = (char *)data;
  	int i;
	
    /* keep status of Core Well GPIO */
	_gpio.cgen = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGEN);
	_gpio.cgio = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGIO);
	_gpio.cglv = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGLV);
	_gpio.cgtpe = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGTPE);
	_gpio.cgtne = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGTNE);
	_gpio.cggpe = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGGPE);
	_gpio.cgsmi = IDL_REG_READ_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGSMI);

    /* keep status of general purpose GPIO */
	_gpio.group[0].int_mode = IDL_REG32_READ(virt_io_mem + 0x14);
	_gpio.gpio_mux_cntl = IDL_REG32_READ(virt_io_mem + 0x18);
	for (i=0; i < 3; i++) {
		_gpio.group[i].output = IDL_REG32_READ(virt_io_mem + 0x0);
		_gpio.group[i].output_enable = IDL_REG32_READ(virt_io_mem + 0x4);
		_gpio.group[i].int_enable = IDL_REG32_READ(virt_io_mem + 0x10);
		virt_io_mem += 0x20;
	}
	return 0;
} 

/* CE4200 gpio resume routine */
int _gen4d5_gpio_resume(void *data)
{
	char *virt_io_mem = (char *)data;
  	int i;

    /* restore general purpose GPIO */
	IDL_REG32_WRITE(virt_io_mem + 0x14, _gpio.group[0].int_mode);
	IDL_REG32_WRITE(virt_io_mem + 0x18, _gpio.gpio_mux_cntl);
	for (i=0; i < 3; i++) {
		IDL_REG32_WRITE(virt_io_mem + 0x0, _gpio.group[i].output);
		IDL_REG32_WRITE(virt_io_mem + 0x4, _gpio.group[i].output_enable);
		IDL_REG32_WRITE(virt_io_mem + 0x10, _gpio.group[i].int_enable);
		virt_io_mem += 0x20;

	}
    /* restore Core Well GPIO */
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGEN, _gpio.cgen & 0x3FF);
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGIO, _gpio.cgio & 0x3FF);
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGLV, _gpio.cglv & 0x3FF);
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGTPE, _gpio.cgtpe & 0x3FF);
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGTNE, _gpio.cgtne & 0x3FF);
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGGPE, _gpio.cggpe & 0x3FF);
	IDL_REG_WRITE_IO(GEN4D5_GPIO_IO_BASE + GEN4D5_GPIO1_CGSMI, _gpio.cgsmi & 0x3FF);
	return 0;
}
