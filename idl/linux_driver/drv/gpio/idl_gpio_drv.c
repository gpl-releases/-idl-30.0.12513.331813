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
 * File Name:       idl_gpio_drv.c
 *----------------------------------------------------------------------
 */

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/pci.h>

#include "osal.h"
#include "idl_gpio_core.h"
#include "pal.h"
#include <osal_event.h>
#include <osal_lock.h>

#include "idl_gpio_core.h"

#include "_gpio.h"
#include "_gpio_gen4_5.h"
#include "_gpio_gen5.h"
#include "_gpio_ce2600.h"
#include "idl_gpio_drv.h"


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    #include <asm/semaphore.h>
#else
    #include <linux/semaphore.h>
#endif
/* These are predefined macros that specify different parameters
 * for our driver */ 
MODULE_AUTHOR("Intel Corporation, (C) 2005 - 2011 - All Rights Reserved");
MODULE_DESCRIPTION("IDL GPIO Device Driver");
MODULE_SUPPORTED_DEVICE("Intel Media Processors");

/* Notifies the kernel that our driver is not GPL. */
MODULE_LICENSE("Dual BSD/GPL");

/* These are the symbols we want exported to other modules */
EXPORT_SYMBOL(idl_gpio_init);
EXPORT_SYMBOL(idl_gpio_release);
EXPORT_SYMBOL(idl_gpio_line_config);
EXPORT_SYMBOL(idl_gpio_set_alt_function);
EXPORT_SYMBOL(idl_gpio_interrupt_config);
EXPORT_SYMBOL(idl_gpio_interrupt_status);
EXPORT_SYMBOL(idl_gpio_clear_interrupt);
EXPORT_SYMBOL(idl_gpio_set_line);
EXPORT_SYMBOL(idl_gpio_get_line);
EXPORT_SYMBOL(idl_gpio_release_interrupt_handler);
EXPORT_SYMBOL(idl_gpio_set_trigger_negative);
EXPORT_SYMBOL(idl_gpio_set_trigger_positive);
EXPORT_SYMBOL(idl_gpio_get_ts);
EXPORT_SYMBOL(idl_gpio_clear_ts);
EXPORT_SYMBOL(idl_gpio_set_smi);
EXPORT_SYMBOL(idl_gpio_set_gpe);
EXPORT_SYMBOL(idl_gpio_disable_interrupt);
EXPORT_SYMBOL(idl_gpio_enable_interrupt);
EXPORT_SYMBOL(idl_gpio_register_interrupt_handler);
EXPORT_SYMBOL(idl_gpio_set_interrupt_router);
EXPORT_SYMBOL(idl_gpio_get_interrupt_router);
/* Unique name for GPIO driver */
#define GPIO_DEV_NAME "gpio"

#ifndef MOD_NAME
#define MOD_NAME "idl_gpio.ko"
#endif
//#define DEBUG 1
char *version_string = "#@#" MOD_NAME " " VER;

/* high level init function (called by insmod when driver is loaded) */
int gpio_init(void);

static struct pci_driver gpio_pci_driver;
/* high level remove function (called by rmmod when driver is unloaded) */
void gpio_cleanup(void);

/* tell the kernel the name of our entry points (these macros are defined in linux/module.h */
module_init(gpio_init);
module_exit(gpio_cleanup);

/* called when an application tries to open a connection to our driver */
int gpio_open(struct inode *i, struct file *f);

/* called when an application disconnects from the driver */
int gpio_close(struct inode *i, struct file *f);

/* called when the application requests that our driver perform a task or 
   service on the application's request. */ 
long  gpio_unlocked_ioctl(struct file *f, unsigned int cmd, unsigned long arg);
/* This is the major number assigned dynamically assigned by Linux */
/* This number is required by the mknod command. */
static int m_gpio_major_number;

/* structure that maps our functions to the OS */
struct file_operations m_gpio_ops = { 	
					.owner = THIS_MODULE,
					.unlocked_ioctl = gpio_unlocked_ioctl, 
					.open = gpio_open, 
					.release = gpio_close,
				 };


uint32_t gpio_irq = 0;
static int debug = 0;
module_param(debug, int, S_IRUGO);

void gpio_irq_handler(void *data);


/* High level gpio initialization - called at driver load time */
int gpio_init()
{
	int retval = 0;

	retval = pci_register_driver(&gpio_pci_driver);
	if (retval) {
		printk(KERN_ERR "GPIO controller pci driver register failure\n");
		return retval;
	}
	/* Register the device with the operating system */	
	if ((m_gpio_major_number = register_chrdev(0, GPIO_DEV_NAME, &m_gpio_ops)) < 0)	{
		/* Error occured with device registration */
		printk(KERN_ERR "%s:%4i: IDL GPIO device registration failed - code %d\n", __FILE__, __LINE__, m_gpio_major_number);
		return m_gpio_major_number;
	}
	gpio_dbg("%s:%4i: IDL GPIO device num returned = %d\n", __FILE__, __LINE__, m_gpio_major_number);
	
	return 0;
}

/* called when the system unloads our driver */
void gpio_cleanup()
{
	unregister_chrdev(m_gpio_major_number, GPIO_DEV_NAME);
  	pci_unregister_driver(&gpio_pci_driver);
	gpio_dbg("%s:%4i: %s device (%d) removed from system\n", __FILE__, __LINE__, GPIO_DEV_NAME, m_gpio_major_number);
}

/* IDL GPIO Open - lets the kernel know that an app is accessing the device */
int gpio_open(struct inode *i, struct file *f)
{
	gpio_dbg("%s:%4i: %s (pid %d) opened idl_gpio_drv.\n",	__FILE__, __LINE__, current->comm, current->pid);
	return 0;
}

/* IDL GPIO close - lets the kernel know that an app is finished accessing the device */
int gpio_close(struct inode *i, struct file *f)
{
	gpio_dbg("%s:%4i: %s (pid %d) closed idl_gpio_drv.\n",	__FILE__, __LINE__, current->comm, current->pid);
	return 0;
}
   
/* idl gpio ioctl - handles user requests to access the gpio device */
long gpio_unlocked_ioctl(struct file *f, u_int cmd, u_long arg)
{
	int status = 0;
	idl_result_t idl_status = IDL_SUCCESS; 
	gpio_ioctl_args gpio_args;
	/* Make sure that we haven't erroneously received the ioctl call */
	if (_IOC_TYPE(cmd) != GPIO_IOCTL_MAGIC)
	{
		printk(KERN_ERR "%s:%4i: ioctl command %x does not belong to this driver\n", 
				__FILE__, __LINE__, cmd);
		return -ENOTTY;
	}
	gpio_dbg(KERN_INFO "%s:%4i: Driver received ioctl command: %x\n", __FILE__, __LINE__, cmd);
	/* make sure we have a valid pointer to our parameters */
	if (!arg)
		return -EINVAL;
	/* read the parameters from user */
	status = copy_from_user(&gpio_args, (void *)arg, sizeof(gpio_ioctl_args));
	
	if (status)	{
		printk(KERN_ERR "%s:%4i: could not read ioctl arguments \n", __FILE__, __LINE__);
		return status;
	}
	else
	{
		gpio_dbg("%s:%4i: ioctl arguments (gpio_args) are:\n", __FILE__, __LINE__);
		gpio_dbg("%s:%4i: -----gpio_num = %lu\n", __FILE__, __LINE__, gpio_args.gpio_num);
		gpio_dbg("%s:%4i: -----data = %lux\n", __FILE__, __LINE__, (unsigned long)gpio_args.data);
		gpio_dbg("%s:%4i: -----interrupt type = %d\n", __FILE__, __LINE__, gpio_args.interrupt_type);
		gpio_dbg("%s:%4i: -----filter setting = %d\n", __FILE__, __LINE__, gpio_args.trigger_negative_setting);
		gpio_dbg("%s:%4i: -----edge setting = %d\n", __FILE__, __LINE__, gpio_args.trigger_positive_setting);
	}
	/* All of our ioctls require a gpio # */
	/* make sure its valid... */
	if (IDL_SUCCESS != idl_valid_gpio_num(gpio_args.gpio_num)) {
		return -EINVAL;
	}
	/* process user request */
	switch (cmd)
	{
		case GPIO_IOCTL_LINE_CONFIG:
			idl_status = idl_gpio_line_config(gpio_args.gpio_num, gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_ALT_FUNCTION:
			idl_status = idl_gpio_set_alt_function(gpio_args.gpio_num, gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_INTERRUPT_CONFIG:
			idl_status = idl_gpio_interrupt_config(gpio_args.gpio_num, gpio_args.interrupt_type);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_INTERRUPT_STATUS:
			idl_status = idl_gpio_interrupt_status(gpio_args.gpio_num, &gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
				break;
			}
			status = copy_to_user((void *)arg, &gpio_args, 
						sizeof(gpio_ioctl_args));
			break;
		case GPIO_IOCTL_CLEAR_INTERRUPT:
			idl_status = idl_gpio_clear_interrupt(gpio_args.gpio_num);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_LINE:
			idl_status = idl_gpio_set_line(gpio_args.gpio_num, gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_GET_LINE:
			idl_status = idl_gpio_get_line(gpio_args.gpio_num, &gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
				break;
			}
			status = copy_to_user((void *)arg, &gpio_args, 
						sizeof(gpio_ioctl_args));
			break;
		case GPIO_IOCTL_CLEAR_TS:
			idl_status = idl_gpio_clear_ts(gpio_args.gpio_num);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_GET_TS:
			idl_status = idl_gpio_get_ts(gpio_args.gpio_num, &gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_REGISTER_IRQ:
			if(IDL_SUCCESS != idl_gpio_supports_interrupts(gpio_args.gpio_num)) {
				return -EINVAL;
			}

			idl_gpio_reset_events(gpio_args.gpio_num);
			idl_gpio_register_interrupt_handler(gpio_args.gpio_num,&gpio_irq_handler,(void *)gpio_args.gpio_num);		

/*
			idl_gpio_register_interrupt_handler(gpio_args.gpio_num,&gpio_irq_handler,NULL);		
			os_acquire_interrupt(gpio_irq, devkey, "IDL GPIO", &gpio_irq_handler, &irq_params[gpio_args.gpio_num]);
*/
			gpio_dbg("%s:%4i: interrupt %lu registered\n", __FILE__, __LINE__, gpio_args.gpio_num);
			break;
		case GPIO_IOCTL_WAIT_FOR_IRQ:
			if(IDL_SUCCESS != idl_gpio_supports_interrupts(gpio_args.gpio_num)) {
				return -EINVAL;
			}
			/* irq_release is a mechanism to allow libray to tell
			 * us not to listen for interrupts any more 
			 * the only other way for us to wake up this wait_queue
			 * is for a interrutp to come in...
			 */
			gpio_dbg("Waiting for IRQ...\n");
			status = idl_gpio_wait_for_irq(gpio_args.gpio_num);
			break;
		case GPIO_IOCTL_ACK_IRQ:
			if(IDL_SUCCESS != idl_gpio_supports_interrupts(gpio_args.gpio_num)) {
				return -EINVAL;
			}
			gpio_dbg("Interrupt processing complete...\n");
			idl_gpio_ack_irq(gpio_args.gpio_num);
			break;
		case GPIO_IOCTL_RELEASE_IRQ:
			gpio_dbg("Releasing IRQ...\n");
			break;
		case GPIO_IOCTL_RELEASE_IRQ_HANDLER:
			gpio_dbg("Releasing IRQ Handler...\n");
			idl_gpio_set_events(gpio_args.gpio_num);
			idl_status = idl_gpio_release_interrupt_handler(gpio_args.gpio_num);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_SMI:
			idl_status = idl_gpio_set_smi(gpio_args.gpio_num, gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_GPE:
			idl_status = idl_gpio_set_gpe(gpio_args.gpio_num, gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_TRIGGER_NEGATIVE:
			idl_status = idl_gpio_set_trigger_negative(gpio_args.gpio_num, gpio_args.trigger_negative_setting);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_TRIGGER_POSITIVE:
			idl_status = idl_gpio_set_trigger_positive(gpio_args.gpio_num, gpio_args.trigger_positive_setting);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_ENABLE_INTERRUPT:
			idl_status = idl_gpio_enable_interrupt(gpio_args.gpio_num);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_DISABLE_INTERRUPT:
			idl_status = idl_gpio_disable_interrupt(gpio_args.gpio_num);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_SET_INTERRUPT_ROUTER:
			idl_status = idl_gpio_set_interrupt_router(gpio_args.gpio_num, gpio_args.router);
			if (idl_status != IDL_SUCCESS)
			{
				status = -EINVAL;
			}
			break;
		case GPIO_IOCTL_GET_INTERRUPT_ROUTER:
			idl_status = idl_gpio_get_interrupt_router(gpio_args.gpio_num, (idl_gpio_interrupt_router_t *)&gpio_args.data);
			if (idl_status != IDL_SUCCESS)
			{
				return status = -EINVAL;
			}
			status = copy_to_user((void *)arg, &gpio_args, 
						sizeof(gpio_ioctl_args));
			break;
		default:
			/* so that we are POSIX compliant, return
			 * ENOTTY as the ioctl command is invalid */
			status = -ENOTTY;
	}
	return status;
}

/* Interrupt handler for GPIO  - using os_acquire interrupt */
void gpio_irq_handler(void *data)
{
	
	int gpio = (int )data;
	gpio_dbg( "IRQ Handler called (gpio # %d)\n", gpio);
	gpio_dbg("Starting interrupt handler\n");
	idl_gpio_irq_handler(gpio);

//	wake_up_interruptible(&listening_q[gpio]);
	
	return;
}

static const struct pci_device_id gpio_id_tables[] __devinitdata = {
{ PCI_DEVICE(0x8086, 0x2e67), },
{0 },
};
MODULE_DEVICE_TABLE(pci, gpio_id_tables);


static int __devinit gpio_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
  	pal_soc_info_t pal_info;
	pal_soc_name_t soc;
	int ret = 0;
	
	if (PAL_SUCCESS != pal_get_soc_info(&pal_info)) {
		printk(KERN_ERR "Can't get SOC_INFO!\n");
		return -1;
	}

	soc = pal_info.name;
	switch (soc)
	{
		case SOC_NAME_CE3100:
			printk("CE3100 GPIO controller found currently!\n");
			idl_gpio_register(&ce3100_gpio_host);
			break;
		case SOC_NAME_CE4100:
			printk("CE4100 GPIO controller found currently!\n");
			idl_gpio_register(&ce3100_gpio_host);
			break;
		case SOC_NAME_CE4200:
			printk("CE4200 GPIO controller found currently!\n");
			idl_gpio_register(&ce4200_gpio_host);
			break;
		case SOC_NAME_CE5300:
			printk("CE5300 GPIO controller found currently!\n");
			idl_gpio_register(&ce5300_gpio_host);
			 break;
		case SOC_NAME_CE2600:
		    printk("CE2600 GPIO controller found currently!\n");
			idl_gpio_register(&ce2600_gpio_host);
			break;
		default:
		    printk(KERN_ERR "Unknown GPIO controller found currently!\n");
		  	ret = -1;
			break;
	}	
	
	if (ret) return ret;
	
	pci_enable_device(pdev);

	if (idl_gpio_core_init()) {
		pci_disable_device(pdev);
		return -1;
	}
	pci_intx(pdev, 1);
	gpio_irq = pdev->irq;
	gpio_dbg("GPIO irq line is %d\n", gpio_irq);
	pci_dev_get(pdev);
    return ret;
}
static void __devexit gpio_pci_remove(struct pci_dev *pdev)
{

  pci_disable_device(pdev);
  pci_intx(pdev, 0);
  pci_dev_put(pdev);
  idl_gpio_core_release();
}

#ifdef CONFIG_PM  
/* gpio pci device suspend */
static int gpio_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
  	int retval = 0;
	unsigned int id;

	intelce_get_soc_info(&id, NULL);
	if (CE2600_SOC_DEVICE_ID == id) 
		return 0;
    /*gpio suspend */
    retval = idl_gpio_suspend();
	if (retval)
	   	 return retval;
	/*pci device save*/
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));
	return 0;
}

/* gpio pci device resume */
static int gpio_pci_resume(struct pci_dev *pdev)
{
    int retval;
	unsigned int id;

	intelce_get_soc_info(&id, NULL);
	if (CE2600_SOC_DEVICE_ID == id) 
		return 0;
	/*pci device restore*/
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	retval = pci_enable_device(pdev);
	if (retval)
	   	 return retval;
	/*gpio resume */
	return idl_gpio_resume();
}
#endif
static struct pci_driver gpio_pci_driver = {
	.name = "gpio_pci_driver",
	.id_table = gpio_id_tables,
	.probe = gpio_pci_probe,
	.remove = __devexit_p(gpio_pci_remove),
#ifdef CONFIG_PM
	.suspend = gpio_pci_suspend,
	.resume = gpio_pci_resume,
#endif
};
