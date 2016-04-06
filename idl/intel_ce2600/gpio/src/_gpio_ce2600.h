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
 * File Name:       _ce2600_gpio.h
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 *
 */

/*
 *  CE2600 specific GPIO routines.
 */

#ifndef _CE2600_GPIO_H
#define _CE2600_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "idl.h"
#include "idl_gpio.h"
#include "idl_gpio_core.h"

/* define HARDCODE_BAR, will use the hardcode base address, otherwise, will get base address from pal 
 *
 * #define HARDCODE_BAR
 *
 */



#define CE2600_GPIO_MEM_BASE		0xDFFE0400
#define CE2600_GPIO_MEM_SIZE		256

#define CE2600_LEGACY_GPIO_IO_BASE			0x1080


#define CE2600_IRQ_BASE				0xBFFFF000
#define CE2600_IRQ_SIZE				256
#define CE2600_IRQ_MASK_OFFSET		0x80
#define CE2600_GPIO_IRQ_MASK_OFFSET	15



#define CE2600_MIN_GPIO_NO		0
#define CE2600_MAX_GPIO_NO		(CE2600_SUM_GPIO_PINS - 1) 

/* starting point for the different GPIO groups on ce2600 */
/* PUB GPIO starts from GPIO[0], end at  GPIO[127] */
#define CE2600_PUB_GPIO_GROUP	    0
#define CE2600_PUB_GPIO_PINS		128

/* Core Well GPIO starts from GPIO[128], ends at GPIO[135] */
#define CE2600_CORE_WELL_GPIO_GROUP	128	
#define CE2600_CORE_WELL_GPIO_PINS  8

#define CE2600_GPIO_PINS		(CE2600_PUB_GPIO_PINS + CE2600_CORE_WELL_GPIO_PINS)


/* only PUB GPIO[127:0] support interrupt */
#define CE2600_GPIO_NUM_INT		(CE2600_PUB_GPIO_PINS)

#define CE2600_GROUPA_IRQ		(1 + 24)
#define CE2600_GROUPB_IRQ		(2 + 24)


/* PUB GPIO Group in PCI device, CE2600 supports GPIO[127:0] */
/* PUB GPIO[31:0]  */
#define CE2600_PUB_GPIO_SUB_GROUP0		0x0

#define CE2600_PUB_GPIO_GPOUTR0		0x00
#define CE2600_PUB_GPIO_GPOER0		0x04
#define CE2600_PUB_GPIO_GPINR0		0x08
#define CE2600_PUB_GPIO_GPSTR0		0x0C
#define CE2600_PUB_GPIO_INTEN0		0x10
#define CE2600_PUB_GPIO_GPITMODLE0	0x14
#define CE2600_PUB_GPIO_GPITMODRF0	0x18
#define CE2600_PUB_GPIO_MUX_CNTL	0x1C
#define CE2600_PUB_GPIO_CLEAR0		0x80
#define CE2600_PUB_GPIO_SET0		0x84
#define CE2600_PUB_GPIO_POLARITY0	0x88


/* PUB GPIO[63:32] */
#define CE2600_PUB_GPIO_SUB_GROUP1		0x20

#define CE2600_PUB_GPIO_GPOUTR1		0x20
#define CE2600_PUB_GPIO_GPOER1		0x24
#define CE2600_PUB_GPIO_GPINR1		0x28
#define CE2600_PUB_GPIO_GPSTR1		0x2C
#define CE2600_PUB_GPIO_INTEN1		0x30
#define CE2600_PUB_GPIO_GPITMODLE1	0x34
#define CE2600_PUB_GPIO_GPITMODRF1	0x38
#define CE2600_PUB_GPIO_CLEAR1		0x90
#define CE2600_PUB_GPIO_SET1		0x94
#define CE2600_PUB_GPIO_POLARITY1	0x98


/* PUB GPIO[93:64] */
#define CE2600_PUB_GPIO_SUB_GROUP2		0x40

#define CE2600_PUB_GPIO_GPOUTR2		0x40
#define CE2600_PUB_GPIO_GPOER2		0x44
#define CE2600_PUB_GPIO_GPINR2		0x48
#define CE2600_PUB_GPIO_GPSTR2		0x4C
#define CE2600_PUB_GPIO_INTEN2		0x50
#define CE2600_PUB_GPIO_GPITMODLE2	0x54
#define CE2600_PUB_GPIO_GPITMODRF2	0x58
#define CE2600_PUB_GPIO_CLEAR2		0xA0
#define CE2600_PUB_GPIO_SET2		0xA4
#define CE2600_PUB_GPIO_POLARITY2	0xA8


/* PUB GPIO[127:94] */
#define CE2600_PUB_GPIO_SUB_GROUP3		0x60

#define CE2600_PUB_GPIO_GPOUTR3		0x60
#define CE2600_PUB_GPIO_GPOER3		0x64
#define CE2600_PUB_GPIO_GPINR3		0x68
#define CE2600_PUB_GPIO_GPSTR3		0x6C
#define CE2600_PUB_GPIO_INTEN3		0x70
#define CE2600_PUB_GPIO_GPITMODLE3	0x74
#define CE2600_PUB_GPIO_GPITMODRF3	0x78
#define CE2600_PUB_GPIO_CLEAR3		0xB0
#define CE2600_PUB_GPIO_SET3		0xB4
#define CE2600_PUB_GPIO_POLARITY3	0xB8

#define CE2600_PUB_GPIO_INT_ROUTER  0xBC
#define CE2600_PUB_GPIO_SUPPORT_ROUTER_MIN   112
#define CE2600_PUB_GPIO_SUPPORT_ROUTER_MAX   127


/* Core Well GPIO Group in North brige */
/* Core Well GPIO[7:0] */
#define CE2600_CORE_WELL_GPIO_CGEN		0x00
#define CE2600_CORE_WELL_GPIO_CGIO		0x04
#define CE2600_CORE_WELL_GPIO_CGLV		0x08
#define CE2600_CORE_WELL_GPIO_CGTPE		0x0C
#define CE2600_CORE_WELL_GPIO_CGTNE		0x10
#define CE2600_CORE_WELL_GPIO_CGGPE		0x14
#define CE2600_CORE_WELL_GPIO_CGSMI		0x18
#define CE2600_CORE_WELL_GPIO_CGTS		0x1C

bool
_ce2600_is_valid_gpio_in_NB(uint32_t gpio_num);


/*
 * This function checks to see if the gpio number specified is a valid gpio on intel_ce2600_.
 * @param gpio_num - The number of the gpio to test.
 * @retval true if gpio num is valid for intel_ce2600_
 * @retval false if gpio num is invalid
 */
bool _ce2600_is_valid_gpio(uint32_t gpio_num);

/*
 * This function checks to see if the specified gpio number supports interrupts. On intel_ce2600_
 * only gpios 0 - 11 can be configured for interrupt support. 
 * @param gpio_num - The number of the GPIO to test.
 * @retval true if gpio num supports interrupts
 * @retval false if interrupts are not supported on the specified gpio line
 */
bool _ce2600_supports_gpio_interrupts(uint32_t gpio_num);

/* 
 * This function checks to see whether or not the gpio supports alternate  
 * functions.
 * @param gpio_num - The number of the GPIO to test.
 * @retval true if gpio num supports interrupts
 * @retval false if interrupts are not supported on the specified gpio line
 */
bool _ce2600_supports_gpio_alt_function(uint32_t gpio_num, uint32_t fn_num);

/*
 * This function configures a gpio line on intel_ce2600_.
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the gpio to configure.
 * @param gpio_config - The direction (input/output) of the gpio
 */
void _ce2600_set_gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						  uint32_t gpio_config);


/*
 * This function will set the pin associated with the gpio_num to it's alternate 
 * function. 
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the GPIO to configure.
 */
void _ce2600_set_gpio_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num);


/* 
 * This function configures the type of interrupt that the GPIO line detects.
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the GPIO to configure.
 * @param interrupt_type - The detection type of interrupt (level, edge, etc.). See idl_gpio_interrupt_type_t enumeration.
 */
void
_ce2600_set_gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						   idl_gpio_interrupt_type_t interrupt_type);

/* 
 * This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the GPIO to get interrupt status.
 * @param interrupt_status - Pointer to 32-bit variable to hold interrupt status value.
 */
void _ce2600_get_gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
							   uint32_t *interrupt_status);

/* 
 * This function clears the interrupt bit for the gpio number specified.
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the GPIO to configure.
 */
void _ce2600_clear_gpio_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num);

/* 
 * This function sets the line state for a GPIO line. 
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the GPIO to set
 * @param val - Value to write. Only the least significant bit is used, all other bits are ignored.
 */
void _ce2600_set_gpio_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					   uint32_t val);


/* 
 * Reads the line state for the GPIO.
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param gpio_num - The number of the GPIO to query
 * @param val - Pointer to return value to write. Will be 0 if clear, 1 if set.
 */
void _ce2600_get_gpio_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					   uint32_t *val);
/*
 * Returns the base address (physical) to the GPIOs.
 * @retval - base address (physical) of GPIO register set 
 */
uint32_t _ce2600_get_gpio_base_addr(void);

/*
 * Returns the size (in bytes) of the GPIO register space.
 * @retval - size (in bytes) of the GPIO register space
 */
uint32_t _ce2600_get_gpio_size(void);


/*
 * Registers the specified handler with the OAL interrupt infrastructure. 
 * The handler will be called when an interrupt occurs. It is up to the
 * handler to determine which GPIO the interrupt # occurs on.
 */
os_interrupt_t _gpio_register_interrupt_handler(os_interrupt_handler_t *handler, 
												 void *data);

/*
 * Returns the irq number and device key for GPIO device in intel_ce2600_
 * @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
 * @param irq - IRQ used for intel_ce2600_
 * @param devkey - Device Key used for ce2600_
 */
void _ce2600_get_gpio_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey);

/*
 * Returns the number of gpio lines for the architecture.
 * @retval - returns the number of gpios
 */
uint32_t _ce2600_get_gpio_number_of_gpios(void);

/*
 * Disable gpio interrupts by writing to IRQ4 configuration register 
 */
void _ce2600_disable_gpio_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
int _ce2600_gpio_init(struct idl_gpio_host *host);
int _ce2600_gpio_exit(struct idl_gpio_host *host);
uint32_t _ce2600_get_gpio_number_of_gpio_interrupts(void);
void _ce2600_clear_gpio_ts(uint32_t gpio_num);
void _ce2600_get_gpio_ts(uint32_t gpio_num, uint32_t *setting);
void _ce2600_set_gpio_smi(uint32_t gpio_num, uint32_t setting);
void _ce2600_set_gpio_gpe(uint32_t gpio_num, uint32_t setting);
void _ce2600_set_gpio_trigger_negative(uint32_t gpio_num, uint32_t setting);
void _ce2600_set_gpio_trigger_positive(uint32_t gpio_num, uint32_t setting);
int _ce2600_set_gpio_interrupt_router(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_router_t router);
int _ce2600_get_gpio_interrupt_router(uint32_t *m_idl_gpio_base, uint32_t gpio_num, idl_gpio_interrupt_router_t *router);

void _ce2600_ack_gpio_irq(uint32_t gpio_num);

void _ce2600_enable_gpio_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
void _ce2600_create_gpio_event(void);
void _ce2600_destroy_gpio_event(void);
int _ce2600_wait_gpio_irq(unsigned int gpio_num);
void _ce2600_handle_gpio_irq(int gpio_num);

void _ce2600_set_gpio_events(uint32_t gpio_num);
void _ce2600_reset_gpio_events(uint32_t gpio_num);

void _ce2600_release_gpio_irq(int val);
os_interrupt_t _ce2600_request_gpio_irq(int irq, int devkey, os_interrupt_handler_t *handler, void *data);

/* CE2600 suspend/resume fucntion definition */
int _ce2600_gpio_suspend(void *vaddr);
int _ce2600_gpio_resume(void *vaddr);
extern struct idl_gpio_host ce2600_gpio_host;

#ifdef __cplusplus
}
#endif

#endif /*end of _CE2600_GPIO_H */

