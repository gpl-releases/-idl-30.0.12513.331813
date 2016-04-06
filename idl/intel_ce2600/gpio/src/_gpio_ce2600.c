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
 * File Name:       _gpio_ce2600.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 *
 */

#include <linux/types.h>
#include <linux/version.h>

#ifdef CONFIG_HW_MUTEXES
#include <linux/hw_mutex.h>
#endif

#include "osal.h"
#include "pal.h"

#include "idl_gpio_core.h"
#include "_gpio_ce2600.h"




/*
 *Events used to signal the start and completion of interrupt 
 * handling.\n
 */
os_event_t ce2600_gpio_interrupt_start[CE2600_GPIO_NUM_INT];
os_event_t ce2600_gpio_interrupt_done[CE2600_GPIO_NUM_INT];
  
int ce2600_gpio_irq_pending[CE2600_GPIO_NUM_INT] = {0};

int ce2600_gpio_irq_release = 0;

#define CE2600_GPIO_PCI_BUS_NUM 1
#define CE2600_GPIO_PCI_DEVICE_NUM 11
#define CE2600_GPIO_PCI_FUNC_NUM 1

static os_mutex_t gpio_lock;
extern uint32_t gpio_irq;

struct idl_gpio_host ce2600_gpio_host = {
	 .init = _ce2600_gpio_init,
	 .exit = _ce2600_gpio_exit,
	 .is_valid = _ce2600_is_valid_gpio,
	 .is_in_NB = _ce2600_is_valid_gpio_in_NB,
	 .supports_interrupt = _ce2600_supports_gpio_interrupts,
	 .supports_alt_function = _ce2600_supports_gpio_alt_function,
	 
	 .set_line_config = _ce2600_set_gpio_line_config,
	 .set_alt_function = _ce2600_set_gpio_alt_function,
	 .set_interrupt_config = _ce2600_set_gpio_interrupt_config,
	 .set_trigger_positive = _ce2600_set_gpio_trigger_positive,
	 .set_trigger_negative = _ce2600_set_gpio_trigger_negative,
	 .set_gpe = _ce2600_set_gpio_gpe,
	 .set_smi = _ce2600_set_gpio_smi,
	 .set_line = _ce2600_set_gpio_line,
	 .set_interrupt_router = _ce2600_set_gpio_interrupt_router,

	 .get_number_of_gpios = _ce2600_get_gpio_number_of_gpios,
	 .get_number_of_gpio_interrupts = _ce2600_get_gpio_number_of_gpio_interrupts,
	 .get_interrupt_status = _ce2600_get_gpio_interrupt_status,
	 .get_interrupt_info = _ce2600_get_gpio_interrupt_info,
	 .get_line = _ce2600_get_gpio_line,
	 .get_ts = _ce2600_get_gpio_ts,
	 .get_interrupt_router = _ce2600_get_gpio_interrupt_router,

	 .clear_interrupt = _ce2600_clear_gpio_interrupt,
	 .clear_ts = _ce2600_clear_gpio_ts,
	 
	 .enable_interrupt = _ce2600_enable_gpio_interrupts,
	 .disable_interrupt = _ce2600_disable_gpio_interrupts,
	 .wait_irq = _ce2600_wait_gpio_irq,
	 .handle_irq = _ce2600_handle_gpio_irq,
	 .ack_irq = _ce2600_ack_gpio_irq,
		
	 .create_event = _ce2600_create_gpio_event,
	 .destroy_event = _ce2600_destroy_gpio_event,

	 .set_events = _ce2600_set_gpio_events,
	 .reset_events = _ce2600_reset_gpio_events,

	 .request_irq = _ce2600_request_gpio_irq,
	.suspend = _ce2600_gpio_suspend,
	.resume = _ce2600_gpio_resume,				 

};

int
_ce2600_gpio_init(struct idl_gpio_host *host)
{
	int ret = 0;

	os_mutex_init(&gpio_lock);
	host->iomem_addr = (phys_addr_t) _ce2600_get_gpio_base_addr();
	host->iomem_size= _ce2600_get_gpio_size();
	host->vaddr = OS_MAP_IO_TO_MEM_NOCACHE(host->iomem_addr, host->iomem_size);
	if (NULL == host->vaddr) {
		ret = -ENOMEM;
	}
	return ret;
}

int
_ce2600_gpio_exit(struct idl_gpio_host *host)
{
	OS_UNMAP_IO_FROM_MEM(host->vaddr, host->iomem_size);
	os_mutex_destroy(&gpio_lock);
	return 0;
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen5.
*/
bool
_ce2600_is_valid_gpio(uint32_t gpio_num)
{
	if (gpio_num >= (uint32_t)CE2600_GPIO_PINS) 
	{
		return false;
	}
	return true;	
}

/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen5 NB.
*/
bool
_ce2600_is_valid_gpio_in_NB(uint32_t gpio_num)
{
	if ((gpio_num < (uint32_t)CE2600_GPIO_PINS) && (gpio_num >= CE2600_CORE_WELL_GPIO_GROUP)) 
	{
		return true;
	}
	return false;
}


/**
* This function checks to see if the specified gpio number supports interrupts. On intel_gen5
* only gpios 0 - 127 can be configured for interrupt support. 
*/
bool
_ce2600_supports_gpio_interrupts(uint32_t gpio_num)
{
	if (gpio_num >= (uint32_t)CE2600_CORE_WELL_GPIO_GROUP)
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
_ce2600_supports_gpio_alt_function(uint32_t gpio_num, uint32_t fn_num)
{
	bool ret = false;

/*
 *   bit_num-----function [0] / [1]
 *	0 --- -disable / enable UART1_TXD_GPIO_50
 *	3 ---- GPIO fucntions /Smart Card0  GPIO 116, 117,118, 38, 39, 40, 41
 *	4 ---- gbe link assigned to 1 / GBE use LOS (loss of signal) pin as gbe link GPIO 42, 43
 *    7--- - disable / enableUART0_TXD_GPIO_48
 *	12 --- GPIO 55  / GPIO 56 supplies the PWM0 trigger
 *	13 --- GPIO 54  / GPIO 57 supplies the PWM1 trigger
 *	14 --- GPIO 53  / GPIO 58 supplies the PWM2 trigger 
 *	15 --- GPIO 52  / GPIO 93 supplies the PWM3 trigger 
 *    16---- disable  /eanble UART0 RTS GPIO 51
 */	
	switch (gpio_num)
	{
		case 50:
				
		case 116:
		case 117:
		case 118:			
		case 38:
		case 39:
		case 40:
		case 41:

		case 42:
		case 43:

		case 48:
			
		case 55:
		case 54:
		case 53:
		case 52:

		case 56:
		case 57:
		case 58:
		case 93:

		case 51:
			ret = true;
			break;
		default:
			break;
	}
	return ret;
}

/**
* This function configures a gpio line on CE2600
*/
void
_ce2600_set_gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					  uint32_t gpio_config)
{

	char *addr_oe = (char *)m_idl_gpio_base;
	char *addr_polarity = (char *)m_idl_gpio_base;
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;
	
		
/*
 * figure out what group the gpio number lives in
 * note range checking is done elsewhere, so here
 * we can assume that gpio_num is valid
 */
 	
	if (gpio_num < CE2600_CORE_WELL_GPIO_GROUP)
	{
		if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
			addr_oe += CE2600_PUB_GPIO_GPOER0;
			addr_polarity += CE2600_PUB_GPIO_POLARITY0;
		}
		else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
			addr_oe += CE2600_PUB_GPIO_GPOER1;
			addr_polarity += CE2600_PUB_GPIO_POLARITY1;		
		}
		else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
			addr_oe += CE2600_PUB_GPIO_GPOER2;
			addr_polarity += CE2600_PUB_GPIO_POLARITY2;

		}
		else if (gpio_num < CE2600_CORE_WELL_GPIO_GROUP)
		{ 
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
			addr_oe += CE2600_PUB_GPIO_GPOER3;
			addr_polarity += CE2600_PUB_GPIO_POLARITY3;

		}
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_lock(HW_MUTEX_GPIO);
#endif
		os_mutex_lock(&gpio_lock);
		curr_reg_val = gpio_mmio_read32(addr_oe);
		if (IDL_GPIO_OUTPUT & gpio_config)
			new_reg_val = curr_reg_val | (1 << bit_num);
		else
			new_reg_val = curr_reg_val & ~(1 << bit_num);

		gpio_mmio_write32(new_reg_val, addr_oe);

		curr_reg_val = gpio_mmio_read32(addr_polarity);

		if (IDL_GPIO_INVERT_POLARITY & gpio_config)
			new_reg_val = curr_reg_val | (1 << bit_num);
		else
			new_reg_val = curr_reg_val & ~(1 << bit_num);


		gpio_mmio_write32(new_reg_val, addr_polarity);
		os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	}	
	else 
	{
		bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;

#ifdef CONFIG_HW_MUTEXES
		hw_mutex_lock(HW_MUTEX_GPIO);
#endif
		os_mutex_lock(&gpio_lock);
		curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGEN);
		new_reg_val |= (1 << bit_num);
		gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGEN);

		curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGIO);
		if (IDL_GPIO_OUTPUT & gpio_config)
			new_reg_val = curr_reg_val & ~(1 << bit_num);
		else
			new_reg_val = curr_reg_val | (1 << bit_num);
	
		gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGIO);
		os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	}
}

/**
 * This function will set the pin associated with the gpio_num to it's alternate 
 * function. 
 * bit_num-----function [0] / [1]
 *  0 --- -disable / enable UART1_TXD_GPIO_50
 *  3 ---- GPIO fucntions /Smart Card0  GPIO 116, 117,118, 38, 39, 40, 41
 *  4 ---- gbe link assigned to 1 / GBE use LOS (loss of signal) pin as gbe link GPIO 42, 43
 *  7--- - disable / enableUART0_TXD_GPIO_48
 *  16---- disable  /eanble UART0 RTS GPIO 51
 */
void 
_ce2600_set_gpio_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num)
{
 	char *addr_mux = (char *)m_idl_gpio_base;
	uint32_t curr_reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t bit_num = 0;
	uint32_t revert = 0;

	addr_mux += CE2600_PUB_GPIO_MUX_CNTL;

	switch (gpio_num)
	{
		case 50:
			bit_num = 0;
			break;
		case 116:
		case 117:
		case 118:			
		case 38:
		case 39:
		case 40:
		case 41:
			bit_num = 3;
			break;
		case 42:
		case 43:
			bit_num = 4;
			break;
		case 48:
			bit_num = 7;
			break;
		case 55:
			bit_num = 12;
			revert = 1;
			break;
		case 54:
			bit_num = 13;
			revert = 1;
			break;
		case 53:
			bit_num = 14;
			revert = 1;
			break;
		case 52:
			bit_num = 15;
			revert = 1;
			break;
		case 56:
			bit_num = 12;
			break;
		case 57:
			bit_num = 13;
			break;
		case 58:
			bit_num = 14;
			break;
		case 93:
			bit_num = 15;
			break;
		case 51:
			bit_num = 16;
			break;
		default:
			return;
	}
	if (revert)
		fn_num = !fn_num;
	
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	os_mutex_lock(&gpio_lock);
	curr_reg_value = gpio_mmio_read32(addr_mux);
	if (0 == fn_num)
		new_reg_value = curr_reg_value & ~(1 << bit_num);
	else
		new_reg_value = curr_reg_value | (1 << bit_num);
	gpio_mmio_write32(new_reg_value, addr_mux);
	os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
}

/** 
* This function configures the type of interrupt that the GPIO line detects. Level or edge detection
*/
void
_ce2600_set_gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						   idl_gpio_interrupt_type_t interrupt_type)
{
	char *addr_le = (char *)m_idl_gpio_base;
	char *addr_rf = (char *)m_idl_gpio_base;
	uint32_t curr_reg_value_le = 0;
	uint32_t curr_reg_value_rf = 0;
	uint32_t new_reg_value_le = 0;
	uint32_t new_reg_value_rf = 0;	
	uint32_t bit_num = 0;
	
	if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
		addr_le += CE2600_PUB_GPIO_GPITMODLE0;
		addr_rf += CE2600_PUB_GPIO_GPITMODRF0;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
		addr_le += CE2600_PUB_GPIO_GPITMODLE1;
		addr_rf += CE2600_PUB_GPIO_GPITMODRF1;

	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
		addr_le += CE2600_PUB_GPIO_GPITMODLE2;
		addr_rf += CE2600_PUB_GPIO_GPITMODRF2;
	}
	else
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
		addr_le += CE2600_PUB_GPIO_GPITMODLE3;
		addr_rf += CE2600_PUB_GPIO_GPITMODRF3;
	}	
	
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	os_mutex_lock(&gpio_lock);
	curr_reg_value_le = gpio_mmio_read32(addr_le);
	curr_reg_value_rf = gpio_mmio_read32(addr_rf);	

	switch (interrupt_type)
	{
		case IDL_GPIO_ACTIVE_HIGH_LEVEL:
			new_reg_value_le = curr_reg_value_le & ~(1 << bit_num);
			new_reg_value_rf = curr_reg_value_rf & ~(1 << bit_num);
			break;
		case IDL_GPIO_ACTIVE_LOW_LEVEL:
			new_reg_value_le = curr_reg_value_le & ~(1 << bit_num);
			new_reg_value_rf = curr_reg_value_rf | (1 << bit_num);			
			break;
		case IDL_GPIO_RISING_UP_EDGE:
			new_reg_value_le = curr_reg_value_le | (1 << bit_num);
			new_reg_value_rf = curr_reg_value_rf & ~(1 << bit_num);							
			break;
		case IDL_GPIO_FALLING_DOWN_EDGE:
			new_reg_value_le = curr_reg_value_le | (1 << bit_num);
			new_reg_value_rf = curr_reg_value_rf | (1 << bit_num);		
			break;
		default:
				gpio_dbg("interrupt_type=%#x is invalid\n", interrupt_type);
				os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
				hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
			return;
		}	

	gpio_mmio_write32(new_reg_value_le, addr_le);
	gpio_mmio_write32(new_reg_value_rf,addr_rf);
	os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
		hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
}

/** 
* This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
*/
void
_ce2600_get_gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
						   uint32_t *interrupt_status)
{
	uint32_t val = 0;
	uint32_t bit_num = 0;
	char *addr = (char *)m_idl_gpio_base;

	if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
		addr += CE2600_PUB_GPIO_GPSTR0;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
		addr += CE2600_PUB_GPIO_GPSTR1;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
		addr += CE2600_PUB_GPIO_GPSTR2;
	}
	else
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
		addr += CE2600_PUB_GPIO_GPSTR3;
	}	
	
	val = gpio_mmio_read32(addr);
	*interrupt_status = (val >> bit_num) & 1;
}

/** 
* This function clears the interrupt bit for the gpio number specified.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void
_ce2600_clear_gpio_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	char *addr = (char *)m_idl_gpio_base;
	uint32_t bit_num = 0;
	
/* This register is a write-one to clear register */

	if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
		addr += CE2600_PUB_GPIO_GPSTR0;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
		addr += CE2600_PUB_GPIO_GPSTR1;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
		addr += CE2600_PUB_GPIO_GPSTR2;
	}
	else
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
		addr += CE2600_PUB_GPIO_GPSTR3;
	}	
	gpio_mmio_write32(1 << bit_num, addr);
}

/** 
* This function sets the line state for a GPIO line. 
*/
void
_ce2600_set_gpio_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t val)
{
	char *addr_set = (char *)m_idl_gpio_base;
	char *addr_clear = (char *)m_idl_gpio_base;
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

/*
 * figure out what group the gpio number lives in
 * note range checking is done elsewhere, so here
 * we can assume that gpio_num is valid
 */
 	if (gpio_num < CE2600_CORE_WELL_GPIO_GROUP)
 	{
		if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
			addr_set += CE2600_PUB_GPIO_SET0;
			addr_clear += CE2600_PUB_GPIO_CLEAR0;
		}
		else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
			addr_set += CE2600_PUB_GPIO_SET1;
			addr_clear += CE2600_PUB_GPIO_CLEAR1;			
		}
		else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
			addr_set += CE2600_PUB_GPIO_SET2;
			addr_clear += CE2600_PUB_GPIO_CLEAR2;
		}
		else if (gpio_num < CE2600_CORE_WELL_GPIO_GROUP)
		{ 
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
			addr_set += CE2600_PUB_GPIO_SET3;
			addr_clear += CE2600_PUB_GPIO_CLEAR3;
		}
		
		if (0 == val)
			gpio_mmio_write32(1 << bit_num, addr_clear);
		else
			gpio_mmio_write32(1 << bit_num, addr_set);
 	}	
	else 
	{
		bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;
		os_mutex_lock(&gpio_lock);
		curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGLV);
		if (0 == val)
			new_reg_val = curr_reg_val & ~(1 << bit_num);
		else
			new_reg_val = curr_reg_val | (1 << bit_num);
		gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGLV);
		os_mutex_unlock(&gpio_lock);
	}
}

/** 
* Reads the line state for the GPIO.
*/
void
_ce2600_get_gpio_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
				   uint32_t *val)
{
	char *addr_input = (char *)m_idl_gpio_base;
	char *addr_polarity = (char *)m_idl_gpio_base;
	uint32_t curr_reg_val = 0;
	uint32_t invert = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

/*
 * figure out what group the gpio number lives in
 * note range checking is done elsewhere, so here
 * we can assume that gpio_num is valid
 */
 	if (gpio_num < CE2600_CORE_WELL_GPIO_GROUP)
 	{
		if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
			addr_input += CE2600_PUB_GPIO_GPINR0;
			addr_polarity += CE2600_PUB_GPIO_POLARITY0;
		}
		else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
			addr_input += CE2600_PUB_GPIO_GPINR1;
			addr_polarity += CE2600_PUB_GPIO_POLARITY1;
		}
		else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
		{
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
			addr_input += CE2600_PUB_GPIO_GPINR2;
			addr_polarity += CE2600_PUB_GPIO_POLARITY2;
		}
		else if (gpio_num < CE2600_CORE_WELL_GPIO_GROUP)
		{ 
			bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
			addr_input += CE2600_PUB_GPIO_GPINR3;
			addr_polarity += CE2600_PUB_GPIO_POLARITY3;
 		}

		invert = gpio_mmio_read32(addr_polarity) & (1 << bit_num);
		curr_reg_val = gpio_mmio_read32(addr_input);
 	}	
	else 
	{
		bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;
		curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGLV);
	}

	gpio_dbg("get gpio: gpio_num=%#x, value=%#x\n", gpio_num, curr_reg_val);
	
	if (0 != invert)
		*val = !((curr_reg_val >> bit_num) & 1);
	else
		*val = (curr_reg_val >> bit_num) & 1;
}

void _ce2600_set_gpio_trigger_positive(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

	bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;

	os_mutex_lock(&gpio_lock);
	curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGTPE);
	if (0 == setting)
		new_reg_val = curr_reg_val & ~(1 << bit_num);
	else
		new_reg_val = curr_reg_val | (1 << bit_num);
	gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGTPE);
	os_mutex_unlock(&gpio_lock);
}

void _ce2600_set_gpio_trigger_negative(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

	bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;

	os_mutex_lock(&gpio_lock);
	curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGTNE);
	if (0 == setting)
		new_reg_val = curr_reg_val & ~(1 << bit_num);
	else
		new_reg_val = curr_reg_val | (1 << bit_num);

	gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGTNE);
	os_mutex_unlock(&gpio_lock);
}

void _ce2600_set_gpio_gpe(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

	bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;
	
	os_mutex_lock(&gpio_lock);
	curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGGPE);
	if (0 == setting)
		new_reg_val = curr_reg_val & ~(1 << bit_num);
	else
		new_reg_val = curr_reg_val | (1 << bit_num);
	gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGGPE);
	os_mutex_unlock(&gpio_lock);
}

void _ce2600_set_gpio_smi(uint32_t gpio_num, uint32_t setting)
{
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

	bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;
	
	os_mutex_lock(&gpio_lock);
	curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGSMI);
	if (0 == setting)
		new_reg_val = curr_reg_val & ~(1 << bit_num);
	else
		new_reg_val = curr_reg_val | (1 << bit_num);
	gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGSMI);
	os_mutex_unlock(&gpio_lock);
}

void _ce2600_get_gpio_ts(uint32_t gpio_num, uint32_t *setting)
{
	uint32_t curr_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

	curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGTS);
	bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;
	*setting = (curr_reg_val >> bit_num) & 1;
	
}

void _ce2600_clear_gpio_ts(uint32_t gpio_num)
{
	uint32_t curr_reg_val = 0;
	uint32_t new_reg_val = 0;
	uint32_t bit_num = 0;
	uint32_t io_base = CE2600_LEGACY_GPIO_IO_BASE;

	bit_num = gpio_num - CE2600_CORE_WELL_GPIO_GROUP;	
	os_mutex_lock(&gpio_lock);
	curr_reg_val = gpio_port_read32(io_base + CE2600_CORE_WELL_GPIO_CGTS);
	new_reg_val = curr_reg_val | (1 << bit_num);
	gpio_port_write32(new_reg_val, io_base + CE2600_CORE_WELL_GPIO_CGTS);
	os_mutex_unlock(&gpio_lock);
}

uint32_t _ce2600_get_gpio_base_addr()
{
#ifdef HARDCODE_BAR
	return CE2600_GPIO_MEM_BASE;
#else
	uint32_t phyad = 0;
	os_pci_dev_t  pci_dev = NULL;

	//get the device  GPIO
	if(os_pci_device_from_address(&pci_dev, CE2600_GPIO_PCI_BUS_NUM, CE2600_GPIO_PCI_DEVICE_NUM, CE2600_GPIO_PCI_FUNC_NUM) != OSAL_SUCCESS)
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

uint32_t _ce2600_get_gpio_size()
{
	return CE2600_GPIO_MEM_SIZE;
}


uint32_t _ce2600_get_gpio_number_of_gpios()
{
	return CE2600_GPIO_PINS;
}

uint32_t _ce2600_get_gpio_number_of_gpio_interrupts()
{
	return CE2600_GPIO_NUM_INT;
}

void _ce2600_disable_gpio_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	char *addr_inten = (char *)m_idl_gpio_base;
	uint32_t curr_reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t bit_num = 0;
	
	if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
		addr_inten += CE2600_PUB_GPIO_INTEN0;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
		addr_inten += CE2600_PUB_GPIO_INTEN1;

	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
		addr_inten += CE2600_PUB_GPIO_INTEN2;
	}
	else
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
		addr_inten += CE2600_PUB_GPIO_INTEN3;
	}

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	os_mutex_lock(&gpio_lock);
	curr_reg_value = gpio_mmio_read32(addr_inten);
	new_reg_value = curr_reg_value & ~(1 << bit_num);
	gpio_mmio_write32(new_reg_value, addr_inten);
	os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
}

void _ce2600_enable_gpio_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num)
{
	char *addr_inten = (char *)m_idl_gpio_base;
	uint32_t curr_reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t bit_num = 0;
	
	if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP1)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP0;
		addr_inten += CE2600_PUB_GPIO_INTEN0;
	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP2)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP1;
		addr_inten += CE2600_PUB_GPIO_INTEN1;

	}
	else if (gpio_num < CE2600_PUB_GPIO_SUB_GROUP3)
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP2;
		addr_inten += CE2600_PUB_GPIO_INTEN2;
	}
	else
	{
		bit_num = gpio_num - CE2600_PUB_GPIO_SUB_GROUP3;
		addr_inten += CE2600_PUB_GPIO_INTEN3;
	}

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	os_mutex_lock(&gpio_lock);
	curr_reg_value = gpio_mmio_read32(addr_inten);
	new_reg_value = curr_reg_value | (1 << bit_num);
	gpio_mmio_write32(new_reg_value, addr_inten);
	os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif

}

void _ce2600_get_gpio_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey)
{
	idl_gpio_interrupt_router_t router;
	int ret = 0;

	ret = _ce2600_get_gpio_interrupt_router(m_idl_gpio_base, gpio_num, &router);
	if (ret)
	{
		*irq = gpio_irq;
	}
	else
	{
		switch (router)
		{
			case IDL_GPIO_LEGACY_GROUP:
				*irq = gpio_irq;
				break;
			case IDL_GPIO_GROUP_A:
				*irq = CE2600_GROUPA_IRQ;
				break;
			case IDL_GPIO_GROUP_B:
				*irq = CE2600_GROUPA_IRQ;
				break;
			case IDL_GPIO_GROUP_RESERVE:
			default:
				*irq = gpio_irq;
				break;
		}
	}	
	*devkey = 0;
}

void _ce2600_create_gpio_event(void)
{
	int i;
	for (i = 0; i < CE2600_GPIO_NUM_INT; i++)
	{
		os_event_create(&ce2600_gpio_interrupt_start[i], 0);
		os_event_create(&ce2600_gpio_interrupt_done[i], 0);
	}
}

void _ce2600_destroy_gpio_event(void)
{
	int i;
	for (i = 0; i < CE2600_GPIO_NUM_INT; i++)
	{
		os_event_destroy(&ce2600_gpio_interrupt_start[i]);
		os_event_destroy(&ce2600_gpio_interrupt_done[i]);
	} 
}

int _ce2600_wait_gpio_irq(unsigned int gpio_num)
{
	int status = 0;
	
	if (os_event_wait(&ce2600_gpio_interrupt_start[gpio_num], -1) != OSAL_SUCCESS)
	{
		status = -512;
	}
	else {
		gpio_dbg("Interrupt received..\n");
		os_event_reset(&ce2600_gpio_interrupt_start[gpio_num]);
	}
	return status;
}

void _ce2600_handle_gpio_irq(int gpio_num)
{
	ce2600_gpio_irq_pending[gpio_num] = 1;
	os_event_set(&ce2600_gpio_interrupt_start[gpio_num]);
	if (os_event_wait(&ce2600_gpio_interrupt_done[gpio_num], -1) == OSAL_SUCCESS)
	{
		os_event_reset(&ce2600_gpio_interrupt_done[gpio_num]);
	}
}

void _ce2600_ack_gpio_irq(uint32_t gpio_num)
{
	ce2600_gpio_irq_pending[gpio_num] = 0;
	os_event_set(&ce2600_gpio_interrupt_done[gpio_num]);
}

void _ce2600_set_gpio_events(uint32_t gpio_num)
{
	os_event_set(&ce2600_gpio_interrupt_start[gpio_num]);
	os_event_set(&ce2600_gpio_interrupt_done[gpio_num]);
}

void _ce2600_reset_gpio_events(uint32_t gpio_num)
{
	os_event_reset(&ce2600_gpio_interrupt_start[gpio_num]);
	os_event_reset(&ce2600_gpio_interrupt_done[gpio_num]);
}


os_interrupt_t _ce2600_request_gpio_irq(int irq, int devkey, os_interrupt_handler_t *handler, void *data)
{
	return os_acquire_interrupt(irq, devkey, "IDL CE2600 GPIO", handler, data);
}

int _ce2600_set_gpio_interrupt_router(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_router_t router)
{
	char *addr_int_router = (char *)m_idl_gpio_base;
	uint32_t curr_reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t bit_num = 0;
	int ret = 0;
	
	if ((gpio_num < CE2600_PUB_GPIO_SUPPORT_ROUTER_MIN) || (gpio_num > CE2600_PUB_GPIO_SUPPORT_ROUTER_MAX))
		return -EINVAL;
	bit_num = (gpio_num - CE2600_PUB_GPIO_SUPPORT_ROUTER_MIN) << 1;
	addr_int_router += CE2600_PUB_GPIO_INT_ROUTER;
	
	switch (router)
	{
		case IDL_GPIO_LEGACY_GROUP:
			new_reg_value = 0 << bit_num;
			break;
		case IDL_GPIO_GROUP_A:
			new_reg_value = 1 << bit_num;
			break;
		case IDL_GPIO_GROUP_B:
			new_reg_value = 2 << bit_num;
			break;
		case IDL_GPIO_GROUP_RESERVE:
		default:
			ret = -EINVAL;
			break;
	}
	if (ret) return ret;
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	os_mutex_lock(&gpio_lock);
	curr_reg_value = gpio_mmio_read32(addr_int_router);
	curr_reg_value &= ~(3 << bit_num);
	new_reg_value |= curr_reg_value;
	gpio_mmio_write32(new_reg_value, addr_int_router);
	os_mutex_unlock(&gpio_lock);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return ret;
}

int _ce2600_get_gpio_interrupt_router(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_router_t *router)
{
	char *addr_int_router = (char *)m_idl_gpio_base;
	uint32_t curr_reg_value = 0;
	uint32_t bit_num = 0;
	int ret = 0;

	if ((gpio_num < CE2600_PUB_GPIO_SUPPORT_ROUTER_MIN) || (gpio_num > CE2600_PUB_GPIO_SUPPORT_ROUTER_MAX))
		return -EINVAL;

	if (NULL == router)
		return -EINVAL;

	bit_num = (gpio_num - CE2600_PUB_GPIO_SUPPORT_ROUTER_MIN) << 1;
	addr_int_router += CE2600_PUB_GPIO_INT_ROUTER;
	
	curr_reg_value = gpio_mmio_read32(addr_int_router);
	curr_reg_value >>= bit_num;
	curr_reg_value &= 0x3;
	switch (curr_reg_value)
	{
		case 0:
			*router = IDL_GPIO_LEGACY_GROUP;
			break;
		case 1:
			*router = IDL_GPIO_GROUP_A;
			break;
		case 2:
			*router = IDL_GPIO_GROUP_B;
			break;
		default:
			*router = IDL_GPIO_GROUP_RESERVE;
			break;
	}
	return ret;
}

struct gpio_group {
	uint32_t input;
	uint32_t output;
	uint32_t output_enable;
	uint32_t int_status;
	uint32_t int_enable;
	uint32_t mode_le;
	uint32_t mode_rf;
	uint32_t polarity;
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
	uint32_t mux_cntl;
	uint32_t int_router;
};

static struct _gpio  _gpio;

/*CE2600 gpio suspend routine */
int _ce2600_gpio_suspend(void *vaddr)
{
	char *io_mem = (char *)vaddr;
  	int i;
	
    /* Keep status of Core Well GPIO*/
	_gpio.cgen = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGEN);
	_gpio.cgio = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGIO);
	_gpio.cglv = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGLV);
	_gpio.cgtpe = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGTPE);
	_gpio.cgtne = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGTNE);
	_gpio.cggpe = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGGPE);
	_gpio.cgsmi = gpio_port_read32(CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGSMI);

 
	/* Keep status of general purpose GPIO*/
	_gpio.mux_cntl = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_MUX_CNTL);
	_gpio.group[0].polarity = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_POLARITY0);
	_gpio.group[1].polarity = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_POLARITY1);
	_gpio.group[2].polarity = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_POLARITY2);
	_gpio.group[3].polarity = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_POLARITY3);
	_gpio.int_router = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_INT_ROUTER);
	for (i=0; i < 4; i++) {
		_gpio.group[i].output = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_GPOUTR0);
		_gpio.group[i].output_enable = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_GPOER0);
		_gpio.group[i].int_enable = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_GPINR0);
		_gpio.group[i].mode_le = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_GPITMODLE0);
		_gpio.group[i].mode_rf = gpio_mmio_read32(io_mem + CE2600_PUB_GPIO_GPITMODRF0);
		io_mem += 0x20;
	}
	return 0;
} 

/* CE2600 gpio resume routine */
int _ce2600_gpio_resume(void *vaddr)
{
	char *io_mem = (char *)vaddr;
  	int i;

    /* Restore status of general purpose GPIO*/
	gpio_mmio_write32(_gpio.mux_cntl, io_mem + CE2600_PUB_GPIO_MUX_CNTL);
	gpio_mmio_write32(_gpio.group[0].polarity, io_mem + CE2600_PUB_GPIO_POLARITY0);
	gpio_mmio_write32(_gpio.group[1].polarity, io_mem + CE2600_PUB_GPIO_POLARITY1);
	gpio_mmio_write32(_gpio.group[2].polarity, io_mem + CE2600_PUB_GPIO_POLARITY2);
	gpio_mmio_write32(_gpio.group[3].polarity, io_mem + CE2600_PUB_GPIO_POLARITY3);
	gpio_mmio_write32(_gpio.int_router, io_mem + CE2600_PUB_GPIO_INT_ROUTER);
	for (i=0; i < 4; i++) {
		gpio_mmio_write32(_gpio.group[i].output, io_mem + CE2600_PUB_GPIO_GPOUTR0);
		gpio_mmio_write32(_gpio.group[i].output_enable, io_mem + CE2600_PUB_GPIO_GPOER0);
		gpio_mmio_write32(_gpio.group[i].int_enable, io_mem + CE2600_PUB_GPIO_INTEN0);
		gpio_mmio_write32(_gpio.group[i].mode_le, io_mem + CE2600_PUB_GPIO_GPITMODLE0);
		gpio_mmio_write32(_gpio.group[i].mode_rf, io_mem + CE2600_PUB_GPIO_GPITMODRF0);
		io_mem += 0x20;

	}

    /* Restore status of Core Well GPIO*/
	gpio_port_write32(_gpio.cgen & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGEN);
	gpio_port_write32(_gpio.cgio & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGIO);
	gpio_port_write32(_gpio.cglv & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGLV);
	gpio_port_write32(_gpio.cgtpe & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGTPE);
	gpio_port_write32(_gpio.cgtne & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGTNE);
	gpio_port_write32(_gpio.cggpe & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGGPE);
	gpio_port_write32(_gpio.cgsmi & 0x3FF, CE2600_LEGACY_GPIO_IO_BASE + CE2600_CORE_WELL_GPIO_CGSMI);
	return 0;
}

