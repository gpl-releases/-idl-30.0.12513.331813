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
* intel_GEN4D5 specific GPIO routines.
*/

#ifndef _GEN4D5_GPIO_H_
#define _GEN4D5_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "idl.h"
#include "idl_gpio.h"
#include "idl_gpio_core.h"

//define HARDCODE_BAR, will use the hardcode base address, otherwise, will get base address from pal
//#define HARDCODE_BAR

/********************************************
* For Intel Gen4.5
*********************************************/
#define GEN4D5_NUM_GPIO_PINS		86

#define GEN4D5_GPIO_MEM_BASE		0xDFFE0400
#define GEN4D5_GPIO_IO_BASE			0x1080
#define GEN4D5_GPIO_MEM_SIZE			256

#define GEN4D5_IRQ_BASE				0xBFFFF000
#define GEN4D5_IRQ_SIZE				256
#define GEN4D5_IRQ_MASK_OFFSET		0x80
#define GEN4D5_GPIO_IRQ_MASK_OFFSET	15
//#define GEN4D5_GPIO_IRQ			27//need to confirm, not use right now

//GPIOAUX[4:0] support adjustable interrupt
//GPIOAUX[77:5] do not support adjustable interrupt
#define GEN4D5_GPIO_ADJ_NUM_INT	5
#define GEN4D5_GPIO_NUM_INT		78

#define GEN4D5_GPIO_NUM_MIN		0
#define GEN4D5_GPIO_NUM_MAX		GEN4D5_NUM_GPIO_PINS - 1 // 0 based count

// starting point for the different GPIO groups on intel_gen4.5
#define GEN4D5_GPIO_GROUP_ZERO	0	
#define GEN4D5_GPIO_GROUP_ONE		78	//GPIO[77:0] = GPIOAUX[98:21]
#define GEN4D5_GPIO_GROUP_TWO		86	//GPIO[85:78] = GPIOS[7:0]

/* GPIOAUX Group 0 == GPIOs[77:0]*/
//GPIOAUX[31:0]
#define GEN4D5_GPIO0_GPOUTR_0		0x00
#define GEN4D5_GPIO0_GPOER_0		0x04
#define GEN4D5_GPIO0_GPINR_0		0x08
#define GEN4D5_GPIO0_GPSTR_0		0x0C
#define GEN4D5_GPIO0_INTEN_0		0x10
#define GEN4D5_GPIO0_GPIT1R0		0x14	//Only GPIOs[4:0] support Interrupt
#define GEN4D5_GPIO0_MUX_CNTL		0x18
#define GEN4D5_GPIO0_GROUP_0		0x20
//GPIOAUX[63:32]
#define GEN4D5_GPIO0_GPOUTR_1		0x20
#define GEN4D5_GPIO0_GPOER_1		0x24
#define GEN4D5_GPIO0_GPINR_1		0x28
#define GEN4D5_GPIO0_GPSTR_1		0x2C
#define GEN4D5_GPIO0_INTEN_1		0x30
#define GEN4D5_GPIO0_GROUP_1		0x40
//GPIOAUX[77:64]
#define GEN4D5_GPIO0_GPOUTR_2		0x40
#define GEN4D5_GPIO0_GPOER_2		0x44
#define GEN4D5_GPIO0_GPINR_2		0x48
#define GEN4D5_GPIO0_GPSTR_2		0x4C
#define GEN4D5_GPIO0_INTEN_2		0x50
#define GEN4D5_GPIO0_GROUP_2		0x4E

/* GPIO Group 1*/
//GPIOS[7:0]
#define GEN4D5_GPIO1_CGEN		0x00
#define GEN4D5_GPIO1_CGIO		0x04
#define GEN4D5_GPIO1_CGLV		0x08
#define GEN4D5_GPIO1_CGTPE		0x0C
#define GEN4D5_GPIO1_CGTNE		0x10
#define GEN4D5_GPIO1_CGGPE		0x14
#define GEN4D5_GPIO1_CGSMI		0x18
#define GEN4D5_GPIO1_CGTS		0x1C

bool
_gen4d5_valid_gpio_num_NB(uint32_t gpio_num);


/**
* This function checks to see if the gpio number specified is a valid gpio on intel_gen4.5.
* @param gpio_num - The number of the gpio to test.
* @retval true if gpio num is valid for intel_gen4.5
* @retval false if gpio num is invalid
*/
bool _gen4d5_valid_gpio_num(uint32_t gpio_num);

/**
* This function checks to see if the specified gpio number supports interrupts. On intel_gen4.5
* only gpios 0 - 11 can be configured for interrupt support. 
* @param gpio_num - The number of the GPIO to test.
* @retval true if gpio num supports interrupts
* @retval false if interrupts are not supported on the specified gpio line
*/
bool _gen4d5_gpio_supports_interrupts(uint32_t gpio_num);

/** 
* This function checks to see whether or not the gpio supports alternate 
* functions.
* @param gpio_num - The number of the GPIO to test.
* @retval true if gpio num supports interrupts
* @retval false if interrupts are not supported on the specified gpio line
*/
bool _gen4d5_gpio_supports_alt_function(uint32_t gpio_num, uint32_t fn_num);

/**
* This function configures a gpio line on intel_gen4.5.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the gpio to configure.
* @param gpio_config - The direction (input/output) of the gpio
*/
void _gen4d5_gpio_line_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
						  uint32_t gpio_config);


/**
* This function will set the pin associated with the gpio_num to it's alternate 
* function. 
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void _gen4d5_gpio_set_alt_function(uint32_t *m_idl_gpio_base, uint32_t gpio_num, uint32_t fn_num);


/** 
* This function configures the type of interrupt that the GPIO line detects.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
* @param interrupt_type - The time of interrupt (level, edge, etc.). See idl_gpio_interrupt_type_t enumeration.
*/
void _gen4d5_gpio_interrupt_config(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
							   idl_gpio_interrupt_type_t interrupt_type);


/** 
* This function reads the interrupt status register to see if the specified GPIO is signalling an interrupt.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to get interrupt status.
* @param interrupt_status - Pointer to 32-bit variable to hold interrupt status value.
*/
void _gen4d5_gpio_interrupt_status(uint32_t *m_idl_gpio_base, uint32_t gpio_num,
							   uint32_t *interrupt_status);

/** 
* This function clears the interrupt bit for the gpio number specified.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to configure.
*/
void _gen4d5_gpio_clear_interrupt(uint32_t *m_idl_gpio_base, uint32_t gpio_num);

/** 
* This function sets the line state for a GPIO line. 
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to set
* @param val - Value to write. Only the least significant bit is used, all other bits are ignored.
*/
void _gen4d5_gpio_set_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					   uint32_t val);


/** 
* Reads the line state for the GPIO.
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param gpio_num - The number of the GPIO to query
* @param val - Pointer to return value to write. Will be 0 if clear, 1 if set.
*/
void _gen4d5_gpio_get_line(uint32_t *m_idl_gpio_base, uint32_t gpio_num, 
					   uint32_t *val);
/**
* Returns the base address (physical) to the GPIOs.
* @retval - base address (physical) of GPIO register set 
*/
uint32_t _gen4d5_gpio_get_base_addr(void);

/**
* Returns the size (in bytes) of the GPIO register space.
* @retval - size (in bytes) of the GPIO register space
*/
uint32_t _gen4d5_gpio_get_size(void);

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
* Returns the irq number and device key for GPIO device in intel_gen4.5
* @param m_idl_gpio_base - Pointer to virtual memory address that represents the gpio register set
* @param irq - IRQ used for intel_gen4.5
* @param devkey - Device Key used for intel_gen4.5
*/
void _gen4d5_gpio_get_interrupt_info(uint32_t *m_idl_gpio_base, uint32_t gpio_num, int *irq, int *devkey);

/**
* Returns the number of gpio lines for the architecture.
* @retval - returns the number of gpios
*/
uint32_t _gen4d5_gpio_get_number_of_gpios(void);

/**
* Disable gpio interrupts by writing to IRQ4 configuration register 
*/
void _gen4d5_gpio_disable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
int _gen4d5_gpio_init(struct idl_gpio_host *host);
int _gen4d5_gpio_exit(struct idl_gpio_host *host);
uint32_t _gen4d5_gpio_get_number_of_gpio_interrupts(void);
void _gen4d5_gpio_clear_ts(uint32_t gpio_num);
void _gen4d5_gpio_get_ts(uint32_t gpio_num, uint32_t *setting);
void _gen4d5_gpio_set_smi(uint32_t gpio_num, uint32_t setting);
void _gen4d5_gpio_set_gpe(uint32_t gpio_num, uint32_t setting);
void _gen4d5_gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting);
void _gen4d5_gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting);
void _gen4d5_gpio_enable_interrupts(uint32_t *m_idl_gpio_base, uint32_t gpio_num);
void _gen4d5_gpio_event_create(void);
void _gen4d5_gpio_event_destroy(void);
int _gen4d5_gpio_wait_for_irq(unsigned int gpio_num);
void _gen4d5_gpio_irq_handler(int gpio_num);
void _gen4d5_gpio_ack_irq(uint32_t gpio_num);
void _gen4d5_gpio_set_events(uint32_t gpio_num);
void _gen4d5_gpio_reset_events(uint32_t gpio_num);
void _gen4d5_gpio_irq_release(int val);
void _gen4d5_gpio_irq_set_irq_params(unsigned long gpio_num);
os_interrupt_t _gen4d5_gpio_os_acquire_interrupt(int irq, int devkey, os_interrupt_handler_t *handler, void *data);
uint32_t * _gen4d5_gpio_irq_get_irq_params(unsigned long gpio_num);

int _gen4d5_gpio_suspend(void *data);
int _gen4d5_gpio_resume(void *data);
extern struct idl_gpio_host ce4200_gpio_host;
#ifdef __cplusplus
}
#endif

#endif

