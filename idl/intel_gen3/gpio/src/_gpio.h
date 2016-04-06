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
 * File Name:       _gpio.h
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 */

/**
* intel_gen3 specific GPIO routines.
*/

#ifndef _GEN3_GPIO_H_
#define _GEN3_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "idl.h"
#include "idl_gpio.h"
#include "idl_gpio_core.h"

//define HARDCODE_BAR, will use the hardcode base address, otherwise, will get base address from pal
//#define HARDCODE_BAR

#define GEN3_NUM_GPIO_PINS		26

#define GEN3_GPIO_MEM_BASE		0xDFFE0400
#define GEN3_GPIO_IO_BASE			0x1080
#define GEN3_GPIO_MEM_SIZE			256

#define GEN3_IRQ_BASE				0xBFFFF000
#define GEN3_IRQ_SIZE				256
#define GEN3_IRQ_MASK_OFFSET		0x80
#define GEN3_GPIO_IRQ_MASK_OFFSET	15
#define GEN3_GPIO_IRQ			27//need to confirm


#define GEN3_GPIO_NUM_INT		12
#define GEN3_GPIO_NUM_MIN		0
#define GEN3_GPIO_NUM_MAX		GEN3_NUM_GPIO_PINS - 1 // 0 based count

// starting point for the different GPIO groups on intel_gen3
#define GEN3_GPIO_GROUP_ZERO	0
#define GEN3_GPIO_GROUP_ONE	12
#define GEN3_GPIO_GROUP_TWO	15
#define GEN3_GPIO_GROUP_THREE	22

/* GPIO Group 0 */
#define GEN3_GPIO0_GPOUTR		0x00
#define GEN3_GPIO0_GPOER		0x04
#define GEN3_GPIO0_GPINR		0x08
#define GEN3_GPIO0_GPSTR		0x0C
#define GEN3_GPIO0_GPIT1R0		0x10
#define GEN3_GPIO0_INT			0x14
#define GEN3_GPIO0_GPIT1R1		0x18
#define GEN3_GPIO0_MUX_CNTL	0x1C


/* GPIO Group 1*/
#define GEN3_GPIO1_CGEN		0x00
#define GEN3_GPIO1_CGIO		0x04
#define GEN3_GPIO1_CGLV		0x08
#define GEN3_GPIO1_CGTPE		0x0C
#define GEN3_GPIO1_CGTNE		0x10
#define GEN3_GPIO1_CGGPE		0x14
#define GEN3_GPIO1_CGSMI		0x18
#define GEN3_GPIO1_CGTS		0x1C

/* GPIO Group 2 */
#define GEN3_GPIO2_CGEN		0x00
#define GEN3_GPIO2_CGIO		0x04
#define GEN3_GPIO2_CGLV		0x08
#define GEN3_GPIO2_CGTPE		0x0C
#define GEN3_GPIO2_CGTNE		0x10
#define GEN3_GPIO2_CGGPE		0x14
#define GEN3_GPIO2_CGSMI		0x18
#define GEN3_GPIO2_CGTS		0x1C
#define GEN3_GPIO2_MUX_CNTL	0x1C


/* GPIO Group 3 */
#define GEN3_GPIO3_CGEN		0x20
#define GEN3_GPIO3_CGIO		0x24
#define GEN3_GPIO3_CGLV		0x28
#define GEN3_GPIO3_CGTPE		0x2C
#define GEN3_GPIO3_CGTNE		0x30
#define GEN3_GPIO3_CGGPE		0x34
#define GEN3_GPIO3_CGSMI		0x38
#define GEN3_GPIO3_CGTS		0x3C

bool
_valid_gpio_num_NB(uint32_t gpio_num);


/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen3.
* @param gpio_num - The number of the gpio to test.
* @retval true if gpio num is valid for intel_gen3
* @retval false if gpio num is invalid
*/
bool _valid_gpio_num(uint32_t gpio_num);

/**
* This function checks to see if the specified gpio number supports interrupts. On intel_gen3
* only gpios 0 - 11 can be configured for interrupt support. 
* @param gpio_num - The number of the GPIO to test.
* @retval true if gpio num supports interrupts
* @retval false if interrupts are not supported on the specified gpio line
*/
bool _gpio_supports_interrupts(uint32_t gpio_num);

/** 
* This function checks to see whether or not the gpio supports alternate 
* functions.
* @param gpio_num - The number of the GPIO to test.
* @retval true if gpio num supports interrupts
* @retval false if interrupts are not supported on the specified gpio line
*/
bool _gpio_supports_alt_function(uint32_t gpio_num, uint32_t fn_num);

/**
* This function configures a gpio line on intel_gen3.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the gpio to configure.
* @param gpio_config - The direction (input/output) of the gpio
*/
void _gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						  uint32_t gpio_config);


/**
* This function will set the pin associated with the gpio_num to it's alternate 
* function. 
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void _gpio_set_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num);


/** 
* This function configures the type of interrupt that the GPIO line detects.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
* @param interrupt_type - The time of interrupt (level, edge, etc.). See idl_gpio_interrupt_type_t enumeration.
*/
void _gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
							   idl_gpio_interrupt_type_t interrupt_type);


/** 
* This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to get interrupt status.
* @param interrupt_status - Pointer to 32-bit variable to hold interrupt status value.
*/
void _gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
							   uint32_t *interrupt_status);

/** 
* This function clears the interrupt bit for the gpio number specified.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void _gpio_clear_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num);

/** 
* This function sets the line state for a GPIO line. 
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to set
* @param val - Value to write. Only the least significant bit is used, all other bits are ignored.
*/
void _gpio_set_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					   uint32_t val);


/** 
* Reads the line state for the GPIO.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to query
* @param val - Pointer to return value to write. Will be 0 if clear, 1 if set.
*/
void _gpio_get_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					   uint32_t *val);
/**
* Returns the base address (physical) to the GPIOs.
* @retval - base address (physical) of GPIO register set 
*/
uint32_t _gpio_get_base_addr(void);

/**
* Returns the size (in bytes) of the GPIO register space.
* @retval - size (in bytes) of the GPIO register space
*/
uint32_t _gpio_get_size(void);

// DELETE
/**
* Registers the specified handler with the OAL interrupt infrastructure. 
* The handler will be called when an interrupt occurs. It is up to the
* handler to determine which GPIO the interrupt # occurs on.
*/
/*os_interrupt_t _gpio_register_interrupt_handler(os_interrupt_handler_t *handler, 
												 void *data);
*/

/**
* Returns the irq number and device key for GPIO device in intel_gen3
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param irq - IRQ used for intel_gen3
* @param devkey - Device Key used for intel_gen3
*/
void _gpio_get_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey);

/**
* Returns the number of gpio lines for the architecture.
* @retval - returns the number of gpios
*/
uint32_t _gpio_get_number_of_gpios(void);

/**
* Disable gpio interrupts by writing to IRQ4 configuration register 
*/
void _gpio_disable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
int  _gpio_init(struct idl_gpio_host *host);
int  _gpio_exit(struct idl_gpio_host *host);
uint32_t _gpio_get_number_of_gpio_interrupts(void);
void _gpio_clear_ts(uint32_t gpio_num);
void _gpio_get_ts(uint32_t gpio_num, uint32_t *setting);
void _gpio_set_smi(uint32_t gpio_num, uint32_t setting);
void _gpio_set_gpe(uint32_t gpio_num, uint32_t setting);
void _gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting);
void _gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting);
void _gpio_enable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
void _gpio_event_create(void);
void _gpio_event_destroy(void);
int _gpio_wait_for_irq(unsigned int gpio_num);
void _gpio_irq_handler(int gpio_num);
void _gpio_ack_irq(uint32_t gpio_num);
void _gpio_set_events(uint32_t gpio_num);
void _gpio_reset_events(uint32_t gpio_num);
void _gpio_irq_release(int val);
void _gpio_irq_set_irq_params(unsigned long gpio_num);
os_interrupt_t _gpio_os_acquire_interrupt(int irq, int devkey, os_interrupt_handler_t *handler, void *data);
uint32_t * _gpio_irq_get_irq_params(unsigned long gpio_num);
extern struct idl_gpio_host ce3100_gpio_host;
#ifdef __cplusplus
}
#endif

#endif

