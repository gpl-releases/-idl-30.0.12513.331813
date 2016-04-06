/*
#
#  This file is provided under a dual BSD/GPLv2 license.  When using or
#  redistributing this file, you may do so under either license.
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2011-2012 Intel Corporation. All rights reserved.
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
#  Copyright(c) 2011-2012 Intel Corporation. All rights reserved.
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
 * File Name:       _gpio_gen5.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 *
 */
#include <linux/types.h>
#include "osal.h"
#include "_gpio_gen5.h"
//#include <linux/wait.h>	//For ERESTARTSYS
//here is io space operations, should put appropriate

//#define GEN5_DEBUG 1

/* Events used to signal the start and completion of interrupt 
 * handling.\n */
os_event_t gen5_gpio_interrupt_start[GEN5_GPIO_NUM_INT];
os_event_t gen5_gpio_interrupt_done[GEN5_GPIO_NUM_INT];
  
int gen5_irq_pending[GEN5_GPIO_NUM_INT] = {0};
uint32_t gen5_irq_params[GEN5_GPIO_NUM_INT] = {0};
int gen5_irq_release = 0;

#define GEN5_GPIO_PCI_BUS_NUM 1
#define GEN5_GPIO_PCI_DEVICE_NUM 11
#define GEN5_GPIO_PCI_FUNC_NUM 1

uint32_t GEN5_MUX_GPIO_ARRAY[]={
35, 36, 37, 38, 77, 78, 79, 80, //trigger input
95, //GBE_LINK
33, 34, 99, 100, 101, 102, //Smart card 0
58, 59, 60, 61, 62, 63, //Smart card 1
64, // UART1_TXD
66 //UART2_TXD
};

struct idl_gpio_host ce5300_gpio_host = {
	.init = _gen5_gpio_init,
	.exit = _gen5_gpio_exit,
	
	.is_valid = _gen5_valid_gpio_num,
	.is_in_NB = _gen5_valid_gpio_num_NB,
	.supports_interrupt = _gen5_gpio_supports_interrupts,
	.supports_alt_function = _gen5_gpio_supports_alt_function,
	
	.set_line_config = _gen5_gpio_line_config,
	.set_alt_function = _gen5_gpio_set_alt_function,
	.set_interrupt_config = _gen5_gpio_interrupt_config,
	.set_trigger_positive = _gen5_gpio_set_trigger_positive,
	.set_trigger_negative = _gen5_gpio_set_trigger_negative,
	.set_gpe = _gen5_gpio_set_gpe,
	.set_smi = _gen5_gpio_set_smi,
	.set_line = _gen5_gpio_set_line,

	.get_number_of_gpios = _gen5_gpio_get_number_of_gpios,
	.get_number_of_gpio_interrupts = _gen5_gpio_get_number_of_gpio_interrupts,
	.get_interrupt_status = _gen5_gpio_interrupt_status,
	.get_interrupt_info = _gen5_gpio_get_interrupt_info,
	.get_line = _gen5_gpio_get_line,
	.get_ts = _gen5_gpio_get_ts,

	.clear_interrupt = _gen5_gpio_clear_interrupt,
	.clear_ts = _gen5_gpio_clear_ts,

	.enable_interrupt = _gen5_gpio_enable_interrupts,
	.disable_interrupt = _gen5_gpio_disable_interrupts,

	.wait_irq = _gen5_gpio_wait_for_irq,
	.handle_irq = _gen5_gpio_irq_handler,
	.ack_irq = _gen5_gpio_ack_irq,
	
	.create_event = _gen5_gpio_event_create,
	.destroy_event = _gen5_gpio_event_destroy,
	
	.set_events = _gen5_gpio_set_events,
	.reset_events = _gen5_gpio_reset_events,

	.request_irq = _gen5_gpio_os_acquire_interrupt,
	.suspend = _gen5_gpio_suspend,
	.resume = _gen5_gpio_resume,				 
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
_gen5_gpio_init(struct idl_gpio_host *host)
{
	int ret = 0;

	os_sema_init(&m_gpio_sema, 1);
	os_sema_init(&m_gpio_semb, 1);
	os_sema_init(&m_gpio_semc, 1);
	os_sema_init(&m_gpio_semd, 1);
	host->iomem_addr = (phys_addr_t) _gen5_gpio_get_base_addr();
	host->iomem_size= _gen5_gpio_get_size();
	host->vaddr = OS_MAP_IO_TO_MEM_NOCACHE(host->iomem_addr, host->iomem_size);
	if (NULL == host->vaddr) {
		ret = -ENOMEM;
	}
	return ret;
}

int
_gen5_gpio_exit(struct idl_gpio_host *host)
{
	OS_UNMAP_IO_FROM_MEM(host->vaddr, host->iomem_size);
	os_sema_destroy(&m_gpio_sema);
	os_sema_destroy(&m_gpio_semb);
	os_sema_destroy(&m_gpio_semc);
	os_sema_destroy(&m_gpio_semd);
	return 0;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen5.
*/
bool
_gen5_valid_gpio_num(uint32_t gpio_num)
{
	if (gpio_num > (uint32_t)GEN5_GPIO_NUM_MAX) 
	{
		return false;
	}
	return true;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen5 NB.
*/
bool
_gen5_valid_gpio_num_NB(uint32_t gpio_num)
{
	if (gpio_num > (uint32_t)GEN5_GPIO_NUM_MAX || gpio_num < GEN5_GPIO_GROUP_ONE) 
	{
		return false;
	}
	return true;
}


/**
* This function checks to see if the specified gpio number supports interrupts. On intel_gen5
* only gpios 0 - 127 can be configured for interrupt support. 
*/
bool
_gen5_gpio_supports_interrupts(uint32_t gpio_num)
{

	if (gpio_num >= (uint32_t)GEN5_GPIO_NUM_INT)
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
_gen5_gpio_supports_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	// On intel_gen5 

	/*
	0 --- UART1_TXD_GPIO_64 enable [1] /disable [0]
	1 --- UART2_TXD_GPIO_66 enable [1] /disable [0]
	2 --- Smart Card 1 [1] / GPIO [0]
	3 --- Smart Card 0 [1] / GPIO [0]
	4 --- GBE_LINK [1] / GPIO95 [0]
	5-7 --- Reserved
	8 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	9 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	10 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	11 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	12 --- GPIO 77 supplies the trigger input [1] / GPIO 38 supplies the trigger input [0]
	13 --- GPIO 78 supplies the trigger input [1] / GPIO 37 supplies the trigger input [0]
	14 --- GPIO 79 supplies the trigger input [1] / GPIO 36 supplies the trigger input [0]
	15 --- GPIO 80 supplies the trigger input [1] / GPIO 35 supplies the trigger input [0]
	
	*/
	uint32_t i = 0, max_mux_num=0;

	if (fn_num != 0 && fn_num != 1)
	{
		return false;
	}

	max_mux_num = sizeof(GEN5_MUX_GPIO_ARRAY) / sizeof(uint32_t);

#ifdef GEN5_DEBUG
	printk("LINE:%d, gpio_num=0x%x, max_mux_num=0x%x\n", __LINE__, gpio_num, max_mux_num);
#endif

	for (i = 0; i < max_mux_num; i++)
	{
		if(gpio_num == GEN5_MUX_GPIO_ARRAY[i])
			return true;
	}

	printk("gpio %d can't be configured as muxed by gpio MUX register\n", gpio_num);
	return false;
}

/**
* This function configures a gpio line on intel_gen5.
*/
void
_gen5_gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					  uint32_t gpio_config)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t addrio =(uint32_t)GEN5_GPIO_IO_BASE;
	uint32_t val, bit_num = 0;

	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
#ifdef GEN5_DEBUG
	printk("LINE:%d, gpio_num=0x%x, addr=0x%x, gpio_config=0x%x\n", __LINE__, gpio_num, (uint32_t)addr, gpio_config);
#endif
	if (gpio_num < GEN5_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
//	GPIO pins act as GPIO functions by default. So set mux before line config is not need.
/*
		if(0 == gpio_num) {
			bit_num = 4;
			val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_MUX_CNTL));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			// set gpio to act as gpio, not alt function
			IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val & ~(1 << bit_num)));
		}
*/		
		if(gpio_num < GEN5_GPIO0_GROUP_0) {
			bit_num = gpio_num - 0;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOER_0));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_0), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_0), (val & ~(1 << bit_num)));
			}
			
			val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOER_0));
#ifdef GEN5_DEBUG
			printk("LINE:%d, After set -- val=0x%x\n", __LINE__, val);
#endif			
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
			bit_num = gpio_num - GEN5_GPIO0_GROUP_0;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOER_1));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_1), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_1), (val & ~(1 << bit_num)));
			}
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
			bit_num = gpio_num - GEN5_GPIO0_GROUP_1;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOER_2));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_2), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_2), (val & ~(1 << bit_num)));
			}
		}		
		else {
			bit_num = gpio_num - GEN5_GPIO0_GROUP_2;
			//set direction
			val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOER_3));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
			if (gpio_config & IDL_GPIO_OUTPUT) {
				// set bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_3), (val | (1 << bit_num)));
			}
			else {
				// clear bit
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOER_3), (val & ~(1 << bit_num)));
			}
		}
		os_sema_put(&m_gpio_sema);
	}
	else if (gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		os_sema_get(&m_gpio_semb);
		
		//enable this port to gpio mode
		val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGEN);
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
		val |= 1<< (gpio_num-GEN5_GPIO_GROUP_ONE);
		IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGEN, val);

		// set direction
		val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGIO);
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, val);
#endif
		bit_num = gpio_num-GEN5_GPIO_GROUP_ONE;
		if (gpio_config & IDL_GPIO_OUTPUT)
		{
			// clear bit
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGIO, (val & ~(1 << bit_num)));
		}
		else
		{
			// set bit
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGIO, (val | (1 << bit_num)));
		}

		val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGIO);
#ifdef GEN5_DEBUG
			printk("LINE:%d, After set -- val=0x%x\n", __LINE__, val);
#endif		
		os_sema_put(&m_gpio_semb);
	}
}

/**
* This function will set the pin associated with the gpio_num to it's alternate 
* function. 
*/
void 
_gen5_gpio_set_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num)
{

	/*
	0 --- UART1_TXD_GPIO_64 enable [1] /disable [0]
	1 --- UART2_TXD_GPIO_66 enable [1] /disable [0]
	2 --- Smart Card 1 [1] / GPIO [0]
	3 --- Smart Card 0 [1] / GPIO [0]
	4 --- GBE_LINK [1] / GPIO95 [0]
	5-7 --- Reserved
	8 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	9 --- Select Internal video sync 0 as trigger input [1] / Select a GPIO as trigger input
	10 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	11 --- Select Internal video sync 1 as trigger input [1] / Select a GPIO as trigger input
	12 --- GPIO 77 supplies the trigger input [1] / GPIO 38 supplies the trigger input [0]
	13 --- GPIO 78 supplies the trigger input [1] / GPIO 37 supplies the trigger input [0]
	14 --- GPIO 79 supplies the trigger input [1] / GPIO 36 supplies the trigger input [0]
	15 --- GPIO 80 supplies the trigger input [1] / GPIO 35 supplies the trigger input [0]
	*/
	
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t val, bit_num = 0xFF;

	if (true == _gen5_gpio_supports_alt_function(gpio_num, 0))
	{
		os_sema_get(&m_gpio_sema);
		val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_MUX_CNTL));
#ifdef GEN5_DEBUG
		printk("gpio_num=%d, before set val =0x%x\n", gpio_num, val);
#endif
		switch(gpio_num) {
			case 64:
				//UART1_TXD
				bit_num = 0;
				break;
			case 66:
				//UART2_TXD
				bit_num = 1;
				break;				
			case 58:
			case 59:
			case 60:
			case 61:
			case 62:
			case 63:
				//Smart card 1				
				bit_num = 2;
				break;
			case 33:
			case 34:
			case 99:
			case 100:
			case 101:
			case 102:
				//Smart card 0
				bit_num = 3;
				break;
			case 95:
				//GBE_LINK
				bit_num = 4;
				break;
			case 38:
				//Set GPIO38 alternative fun to 1. 
				bit_num = 8;
				if(1 == fn_num) { //mux 38, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) & ~(1 << (bit_num+4)))); // Set 0 in bit 8 and bit 12
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 8
				} 
				break;
			case 77:
				//Set GPIO77 alternative fun to 1
				bit_num = 8;
				if(1 == fn_num) {//mux 77, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) | (1 << (bit_num+4)))); // Set 0 in bit 8 and set 1 in bit 12
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 8
				} 
				break;
			case 37:
				//Set GPIO37 alternative fun to 1
				bit_num = 9;
				if(1 == fn_num) { // mux 37, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << (bit_num))) & ~(1 << (bit_num+4)))); // Set 0 in bit 9 and bit 13.
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 9
				} 
				break;
			case 78:
				//Set GPIO78 alternative fun to 1
				bit_num = 9;
				if(1 == fn_num) {// mux 78, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) | (1 << (bit_num+4)))); // Set 0 in bit 9 and set 1 in bit 13.
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 9
				} 
				break;
			case 36:
				//Set GPIO36 alternative fun to 1
				bit_num = 10;
				if(1 == fn_num) {// mux 36, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) & ~(1 << (bit_num+4)))); // Set 0 in bit 10 and bit 14
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 10
				} 
				break;
			case 79:
				//Set GPIO79 alternative fun to 1
				bit_num = 10;
				if(1 == fn_num) {// // mux 79, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) | (1 << (bit_num+4)))); // Set 0 in bit 10 and set 1 in bit 14
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 10
				} 
				break;			
			case 35:
				//Set GPIO35 alternative fun to 1
				bit_num = 11;
				if(1 == fn_num) {// mux 35, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) & ~(1 << (bit_num+4)))); // Set 0 in bit 11 and bit 15
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 11
				} 
				break;
			case 80:
				//Set GPIO80 alternative fun to 1
				bit_num = 11;
				if(1 == fn_num) { // mux 80, 1
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), ((val & ~(1 << bit_num)) | (1 << (bit_num+4)))); // Set 0 in bit 11 and set 1 bit 15
				}
				else {
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num))); //Set 1 in bit 11
				} 
				break;
			default:
				bit_num = 0xFF;
				break;
		}
		
		if(bit_num < 5) {
			if (1 == fn_num)
			    IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val | (1 << bit_num)));
			else
			    IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_MUX_CNTL), (val & ~(1 << bit_num)));
		}
		
#ifdef GEN5_DEBUG
		val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_MUX_CNTL));
		printk("gpio_num=%d, after set val =0x%x\n", gpio_num, val);
#endif

		os_sema_put(&m_gpio_sema);
	}

}

/** 
* This function configures the type of interrupt that the GPIO line detects. Level or edge detection
*/
void
_gen5_gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						   idl_gpio_interrupt_type_t interrupt_type)
{
	uintptr_t addr = 0;
	uint32_t val1=0, val2 = 0, bit_num = 0;
	
	os_sema_get(&m_gpio_sema);
	
	addr = (uintptr_t)m_idl_gpio_base;

	if (gpio_num < GEN5_GPIO_GROUP_ONE)
	{
		if(gpio_num < GEN5_GPIO0_GROUP_0) {
			bit_num = gpio_num - 0;
			val1 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODLE0));
			val2 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODRF0));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val1=0x%x\n", __LINE__, gpio_num, val1);
			printk("LINE:%d, gpio_num=0x%x, val2=0x%x\n", __LINE__, gpio_num, val2);
#endif
			switch (interrupt_type)
			{
				case IDL_GPIO_ACTIVE_HIGH_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE0), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF0), (val2 & ~(1 << bit_num)));
					break;
				case IDL_GPIO_ACTIVE_LOW_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE0), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF0), (val2 | (1 << bit_num)));
					break;
				case IDL_GPIO_RISING_UP_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE0), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF0), (val2 & ~(1 << bit_num)));					
					break;
				case IDL_GPIO_FALLING_DOWN_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE0), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF0), (val2 | (1 << bit_num)));					
					break;
				default:
					printk("LINE:%d, interrupt_type=0x%x is invalid\n", __LINE__, interrupt_type);
					return;
			}
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
			bit_num = gpio_num - GEN5_GPIO0_GROUP_0;
			val1 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODLE1));
			val2 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODRF1));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val1=0x%x\n", __LINE__, gpio_num, val1);
			printk("LINE:%d, gpio_num=0x%x, val2=0x%x\n", __LINE__, gpio_num, val2);
#endif
			switch (interrupt_type)
			{
				case IDL_GPIO_ACTIVE_HIGH_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE1), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF1), (val2 & ~(1 << bit_num)));
					break;
				case IDL_GPIO_ACTIVE_LOW_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE1), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF1), (val2 | (1 << bit_num)));
					break;
				case IDL_GPIO_RISING_UP_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE1), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF1), (val2 & ~(1 << bit_num)));					
					break;
				case IDL_GPIO_FALLING_DOWN_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE1), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF1), (val2 | (1 << bit_num)));					
					break;
				default:
					printk("LINE:%d, interrupt_type=0x%x is invalid\n", __LINE__, interrupt_type);
					return;
			}
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
			bit_num = gpio_num - GEN5_GPIO0_GROUP_1;
			val1 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODLE2));
			val2 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODRF2));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val1=0x%x\n", __LINE__, gpio_num, val1);
			printk("LINE:%d, gpio_num=0x%x, val2=0x%x\n", __LINE__, gpio_num, val2);
#endif
			switch (interrupt_type)
			{
				case IDL_GPIO_ACTIVE_HIGH_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE2), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF2), (val2 & ~(1 << bit_num)));
					break;
				case IDL_GPIO_ACTIVE_LOW_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE2), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF2), (val2 | (1 << bit_num)));
					break;
				case IDL_GPIO_RISING_UP_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE2), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF2), (val2 & ~(1 << bit_num)));					
					break;
				case IDL_GPIO_FALLING_DOWN_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE2), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF2), (val2 | (1 << bit_num)));					
					break;
				default:
					printk("LINE:%d, interrupt_type=0x%x is invalid\n", __LINE__, interrupt_type);
					return;
			}
		}		
		else {
			bit_num = gpio_num - GEN5_GPIO0_GROUP_2;
			val1 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODLE3));
			val2 = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPITMODRF3));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val1=0x%x\n", __LINE__, gpio_num, val1);
			printk("LINE:%d, gpio_num=0x%x, val2=0x%x\n", __LINE__, gpio_num, val2);
#endif
			switch (interrupt_type)
			{
				case IDL_GPIO_ACTIVE_HIGH_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE3), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF3), (val2 & ~(1 << bit_num)));
					break;
				case IDL_GPIO_ACTIVE_LOW_LEVEL:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE3), (val1 & ~(1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF3), (val2 | (1 << bit_num)));
					break;
				case IDL_GPIO_RISING_UP_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE3), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF3), (val2 & ~(1 << bit_num)));					
					break;
				case IDL_GPIO_FALLING_DOWN_EDGE:
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODLE3), (val1 | (1 << bit_num)));
					IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPITMODRF3), (val2 | (1 << bit_num)));					
					break;
				default:
					printk("LINE:%d, interrupt_type=0x%x is invalid\n", __LINE__, interrupt_type);
					return;
			}//end of switch
		}// end of else

	}

	os_sema_put(&m_gpio_sema);
}

/** 
* This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
*/
void
_gen5_gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
						   uint32_t *interrupt_status)
{
	uint32_t val=0, bit_num=0;
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	if(gpio_num < GEN5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPSTR_0));
		val &= (1 << bit_num);
		*interrupt_status = val >> bit_num;
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_0;
		val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPSTR_1));
		val &= (1 << bit_num);
		*interrupt_status = val >> bit_num;
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_1;
		val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPSTR_2));
		val &= (1 << bit_num);
		*interrupt_status = val >> bit_num;
	}
	else {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_2;
		val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPSTR_3));
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
_gen5_gpio_clear_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uintptr_t addr, bit_num=0;
	os_sema_get(&m_gpio_sema);

	// This register is a write-one to clear register
	addr = (uintptr_t)m_idl_gpio_base;
	if(gpio_num < GEN5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPSTR_0), (1 << bit_num));
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_0;
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPSTR_1), (1 << bit_num));
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_1;
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPSTR_2), (1 << bit_num));
	}
	else {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_2;
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPSTR_3), (1 << bit_num));
	}	
	os_sema_put(&m_gpio_sema);
}

/** 
* This function sets the line state for a GPIO line. 
*/
void
_gen5_gpio_set_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t val)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t curr_reg_val;
	uint32_t addrio = GEN5_GPIO_IO_BASE;
	
	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
#ifdef GEN5_DEBUG
	printk("LINE:%d, gpio_num=0x%x, addr=0x%x\n", __LINE__, gpio_num, (uint32_t)addr);
#endif
	if (gpio_num < GEN5_GPIO_GROUP_ONE)
	{
		os_sema_get(&m_gpio_sema);
		
		if(gpio_num < GEN5_GPIO0_GROUP_0) {
			gpio_num -= 0;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_0));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_0), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_0), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_0));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif
		}
		else if (gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
			gpio_num -= GEN5_GPIO0_GROUP_0;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_1));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_1), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_1), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_1));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) { 
			gpio_num -= GEN5_GPIO0_GROUP_1;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_2));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_2), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_2), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_2));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
		}
		else { 
			gpio_num -= GEN5_GPIO0_GROUP_2;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_3));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
			if (val == 0) {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_3), 
				(curr_reg_val & (~(1 << gpio_num))));
			}
			else {
				IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_GPOUTR_3), 
				  (curr_reg_val | (1 << gpio_num)));
			}
#ifdef GEN5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPOUTR_3));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
		}		
		os_sema_put(&m_gpio_sema);
	}
	else if (gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		os_sema_get(&m_gpio_semb);
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGLV);
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif					
		if (val == 0) {
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGLV, (curr_reg_val & (~(1 << (gpio_num-GEN5_GPIO_GROUP_ONE)))));
		}
		else {
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGLV, (curr_reg_val | (1 << (gpio_num-GEN5_GPIO_GROUP_ONE))));
		}
#ifdef GEN5_DEBUG
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO1_CGLV));
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif				
		os_sema_put(&m_gpio_semb);
	}
}

/** 
* Reads the line state for the GPIO.
*/
void
_gen5_gpio_get_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t *val)
{
	uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	// figure out what group the gpio number lives in
	// note range checking is done elsewhere, so here
	// we can assume that gpio_num is valid
	if (gpio_num < GEN5_GPIO_GROUP_ONE)
	{
		if(gpio_num < GEN5_GPIO0_GROUP_0) {
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPINR_0));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
			gpio_num -= GEN5_GPIO0_GROUP_0;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPINR_1));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}
		else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
			gpio_num -= GEN5_GPIO0_GROUP_1;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPINR_2));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}
		else { 
			gpio_num -= GEN5_GPIO0_GROUP_2;
			curr_reg_val = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_GPINR_3));
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif			
			*val = ((curr_reg_val & (1 << gpio_num)) >> gpio_num);
		}		
	}
	else if(gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGLV);
#ifdef GEN5_DEBUG
			printk("LINE:%d, gpio_num=0x%x, val=0x%x\n", __LINE__, gpio_num, curr_reg_val);
#endif	
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		*val = ((curr_reg_val & (1 << bit_num)) >> bit_num);
	}
}

void _gen5_gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	if (gpio_num >= GEN5_GPIO_GROUP_ONE && gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGTPE);
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGTPE, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGTPE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen5_gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	if (gpio_num >= GEN5_GPIO_GROUP_ONE && gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGTNE);
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGTNE, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGTNE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen5_gpio_set_gpe(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	if (gpio_num >= GEN5_GPIO_GROUP_ONE && gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGGPE);
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGGPE, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGGPE, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen5_gpio_set_smi(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	if (gpio_num >= GEN5_GPIO_GROUP_ONE && gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGSMI);
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		if(setting == 0)
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGSMI, (curr_reg_val & ~(1 << bit_num)));
		else
			IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGSMI, (curr_reg_val |(1 << bit_num)));
	}
}

void _gen5_gpio_get_ts(uint32_t gpio_num, uint32_t *setting)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	if (gpio_num >= GEN5_GPIO_GROUP_ONE && gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGTS);
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		*setting = (curr_reg_val & (1<<bit_num))>>bit_num;
	}
}

void _gen5_gpio_clear_ts(uint32_t gpio_num)
{
	uint32_t curr_reg_val, bit_num;
	uint32_t addrio = GEN5_GPIO_IO_BASE;

	if (gpio_num >= GEN5_GPIO_GROUP_ONE && gpio_num < GEN5_GPIO_GROUP_TWO)
	{
		curr_reg_val = IDL_REG_READ_IO(addrio + GEN5_GPIO1_CGTS);
		bit_num = gpio_num - GEN5_GPIO_GROUP_ONE;
		IDL_REG_WRITE_IO(addrio + GEN5_GPIO1_CGTS, (curr_reg_val |(1 << bit_num)));
	}
}

uint32_t _gen5_gpio_get_base_addr()
{
#ifdef HARDCODE_BAR
	return GEN5_GPIO_MEM_BASE;
#else
	uint32_t phyad = 0;
	os_pci_dev_t  pci_dev = NULL;

	//get the device  GPIO
	if(os_pci_device_from_address(&pci_dev, GEN5_GPIO_PCI_BUS_NUM, GEN5_GPIO_PCI_DEVICE_NUM, GEN5_GPIO_PCI_FUNC_NUM) != OSAL_SUCCESS)
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

uint32_t _gen5_gpio_get_size()
{
	return GEN5_GPIO_MEM_SIZE;
}

/*os_interrupt_t _gpio_register_interrupt_handler(os_interrupt_handler_t *handler, 
												 void *data)
{
	return os_acquire_interrupt(GEN5_GPIO_IRQ, GPIO_0, "IDL GPIO",
								handler, data);
}
*/
uint32_t _gen5_gpio_get_number_of_gpios()
{
	return GEN5_NUM_GPIO_PINS;
}

uint32_t _gen5_gpio_get_number_of_gpio_interrupts()
{
	return GEN5_GPIO_NUM_INT;
}

void _gen5_gpio_disable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uint32_t temp=0, bit_num=0;
	uint32_t val = 0;
	uintptr_t irq_base;
    uintptr_t addr = (uintptr_t)m_idl_gpio_base;

	os_sema_get(&m_gpio_sema);
	if(gpio_num < GEN5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_0));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_0), temp&(~(1<<bit_num)));
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_0;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_1));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_1), temp&(~(1<<bit_num)));
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_1;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_2));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_2), temp&(~(1<<bit_num)));
	}
	else {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_2;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_3));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_3), temp&(~(1<<bit_num)));
	}	
	os_sema_put(&m_gpio_sema);

	if( 0 == (temp & 0xfff)) {
		irq_base = (uintptr_t)OS_MAP_IO_TO_MEM_NOCACHE(GEN5_IRQ_BASE, GEN5_IRQ_SIZE);

		os_sema_get(&m_gpio_sema);

		val = IDL_REG32_READ((void*)(irq_base + GEN5_IRQ_MASK_OFFSET));
		
		/* clear all gpio interrupt bits */
		val |= (1 << GEN5_GPIO_IRQ_MASK_OFFSET);
		
		IDL_REG32_WRITE((void*)(irq_base + GEN5_IRQ_MASK_OFFSET), val);
		os_sema_put(&m_gpio_sema);

		OS_UNMAP_IO_FROM_MEM((void *)irq_base, GEN5_IRQ_SIZE);
	}
}

void _gen5_gpio_enable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	uint32_t temp=0, bit_num=0;
	uint32_t val = 0;
	uintptr_t irq_base;
    uintptr_t addr = (uintptr_t)m_idl_gpio_base;
	os_sema_get(&m_gpio_sema);
	
	if(gpio_num < GEN5_GPIO0_GROUP_0) {
		bit_num = gpio_num - 0;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_0));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_0), temp|(1<<bit_num));
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_0 && gpio_num < GEN5_GPIO0_GROUP_1) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_0;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_1));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_1), temp|(1<<bit_num));
	}
	else if(gpio_num >= GEN5_GPIO0_GROUP_1 && gpio_num < GEN5_GPIO0_GROUP_2) {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_1;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_2));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_2), temp|(1<<bit_num));
	}
	else {
		bit_num = gpio_num - GEN5_GPIO0_GROUP_2;
		temp = IDL_REG32_READ((void*)(addr + GEN5_GPIO0_INTEN_3));
		IDL_REG32_WRITE((void*)(addr + GEN5_GPIO0_INTEN_3), temp|(1<<bit_num));
	}	
	os_sema_put(&m_gpio_sema);
	
	irq_base = (uintptr_t)OS_MAP_IO_TO_MEM_NOCACHE(GEN5_IRQ_BASE, GEN5_IRQ_SIZE);

	os_sema_get(&m_gpio_sema);
	val = IDL_REG32_READ((void*)(irq_base + GEN5_IRQ_MASK_OFFSET));
	
	/* enable all gpio interrupt bits */
	val &= ~(1 << GEN5_GPIO_IRQ_MASK_OFFSET);
	
	IDL_REG32_WRITE((void*)(irq_base + GEN5_IRQ_MASK_OFFSET), val);
	os_sema_put(&m_gpio_sema);

	OS_UNMAP_IO_FROM_MEM((void *)irq_base, GEN5_IRQ_SIZE);
}

void _gen5_gpio_get_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey)
{
	*irq = gpio_irq;
	*devkey = 0;
}

void _gen5_gpio_event_create()
{
	int i;
	for (i = 0; i < GEN5_GPIO_NUM_INT; i++) {
		os_event_create(&gen5_gpio_interrupt_start[i], 0);
		os_event_create(&gen5_gpio_interrupt_done[i], 0);
	}
}

void _gen5_gpio_event_destroy()
{
	int i;
	for (i = 0; i < GEN5_GPIO_NUM_INT; i++){
		os_event_destroy(&gen5_gpio_interrupt_start[i]);
		os_event_destroy(&gen5_gpio_interrupt_done[i]);
	} 
}

int _gen5_gpio_wait_for_irq(unsigned int gpio_num)
{
	int status = 0;
	if (os_event_wait(&gen5_gpio_interrupt_start[gpio_num], -1) != OSAL_SUCCESS) {
		//status = -ERESTARTSYS;
		status = -512;
	}
	else {
#ifdef GEN5_DEBUG
		printk("Interrupt received..\n");
#endif
		os_event_reset(&gen5_gpio_interrupt_start[gpio_num]);
	}
	return status;
}

void _gen5_gpio_irq_handler(int gpio_num)
{
	gen5_irq_pending[gpio_num] = 1;
	os_event_set(&gen5_gpio_interrupt_start[gpio_num]);
	if (os_event_wait(&gen5_gpio_interrupt_done[gpio_num], -1) == OSAL_SUCCESS) {
		os_event_reset(&gen5_gpio_interrupt_done[gpio_num]);
	}
}

void _gen5_gpio_ack_irq(uint32_t gpio_num)
{
	gen5_irq_pending[gpio_num] = 0;
	os_event_set(&gen5_gpio_interrupt_done[gpio_num]);
}

void _gen5_gpio_set_events(uint32_t gpio_num)
{
	os_event_set(&gen5_gpio_interrupt_start[gpio_num]);
	os_event_set(&gen5_gpio_interrupt_done[gpio_num]);
}

void _gen5_gpio_reset_events(uint32_t gpio_num)
{
	os_event_reset(&gen5_gpio_interrupt_start[gpio_num]);
	os_event_reset(&gen5_gpio_interrupt_done[gpio_num]);
}

void _gen5_gpio_irq_release(val)
{
	gen5_irq_release = val;
}

void _gen5_gpio_irq_set_irq_params(unsigned long gpio_num)
{
	gen5_irq_params[gpio_num] = gpio_num;
}

os_interrupt_t _gen5_gpio_os_acquire_interrupt(int irq, int devkey, os_interrupt_handler_t *handler, void *data)
{
	return os_acquire_interrupt(irq, 
			devkey, "IDL GPIO", handler, data);
}

uint32_t * _gen5_gpio_irq_get_irq_params(unsigned long gpio_num)
{
	return &gen5_irq_params[gpio_num];
}

struct gpio_group {
	uint32_t input;
	uint32_t output;
	uint32_t output_enable;
	uint32_t int_status;
	uint32_t int_enable;
	uint32_t int_mode;
    uint32_t edge_mode;
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

	struct gpio_group group[4];
	uint32_t gpio_mux_cntl;
};

static struct _gpio  _gpio;

/*CE5300 gpio suspend routine */
int _gen5_gpio_suspend(void *data)
{
	char *virt_io_mem = (char *)data;
  	int i;
	
    /* Keep status of Core Well GPIO*/
	_gpio.cgen = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGEN);
	_gpio.cgio = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGIO);
	_gpio.cglv = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGLV);
	_gpio.cgtpe = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGTPE);
	_gpio.cgtne = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGTNE);
	_gpio.cggpe = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGGPE);
	_gpio.cgsmi = IDL_REG_READ_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGSMI);

    /* Keep status of general purpose GPIO*/
	_gpio.gpio_mux_cntl = IDL_REG32_READ(virt_io_mem + 0x1c);
	for (i=0; i < 4; i++) {
		_gpio.group[i].output = IDL_REG32_READ(virt_io_mem + 0x0);
		_gpio.group[i].output_enable = IDL_REG32_READ(virt_io_mem + 0x4);
		_gpio.group[i].int_enable = IDL_REG32_READ(virt_io_mem + 0x10);
		_gpio.group[i].int_mode = IDL_REG32_READ(virt_io_mem + 0x14);
		_gpio.group[i].edge_mode = IDL_REG32_READ(virt_io_mem + 0x18);
		virt_io_mem += 0x20;
	}
	return 0;
} 

/* CE5300 gpio resume routine */
int _gen5_gpio_resume(void *data)
{
	char *virt_io_mem = (char *)data;
  	int i;

    /* Restore status of general purpose GPIO*/
	IDL_REG32_WRITE(virt_io_mem + 0x1c, _gpio.gpio_mux_cntl);
	for (i=0; i < 4; i++) {
		IDL_REG32_WRITE(virt_io_mem + 0x0, _gpio.group[i].output);
		IDL_REG32_WRITE(virt_io_mem + 0x4, _gpio.group[i].output_enable);
		IDL_REG32_WRITE(virt_io_mem + 0x10, _gpio.group[i].int_enable);
		IDL_REG32_WRITE(virt_io_mem + 0x14, _gpio.group[i].int_mode);
		IDL_REG32_WRITE(virt_io_mem + 0x18, _gpio.group[i].edge_mode);
		virt_io_mem += 0x20;

	}

    /* Restore status of Core Well GPIO*/
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGEN, _gpio.cgen & 0x3FF);
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGIO, _gpio.cgio & 0x3FF);
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGLV, _gpio.cglv & 0x3FF);
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGTPE, _gpio.cgtpe & 0x3FF);
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGTNE, _gpio.cgtne & 0x3FF);
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGGPE, _gpio.cggpe & 0x3FF);
	IDL_REG_WRITE_IO(GEN5_GPIO_IO_BASE + GEN5_GPIO1_CGSMI, _gpio.cgsmi & 0x3FF);
	return 0;
}
