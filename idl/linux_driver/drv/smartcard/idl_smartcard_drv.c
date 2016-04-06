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

/*------------------------------------------------------------------------------
 * File Name: idl_smartcard_drv.c
 *------------------------------------------------------------------------------
 *
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/wait.h>
#include <linux/pci.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>

#include <osal_event.h>
#include <osal_sema.h>

#include <_smartcard.h>
#include <idl_smartcard.h>
#include <idl_smartcard_core.h>

#include <pal.h>
  
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    #include <asm/semaphore.h>
#else
    #include <linux/semaphore.h>
#endif

#ifdef DEBUG
#define scard_dbg(fmt, args...) do \
  { \
	      printk(KERN_INFO fmt, ##args); \
  } while(0)
#else
#define scard_dbg(fmt, arg...)
#endif

/* Driver identification */
 MODULE_AUTHOR("Intel Corporation, (C) 2006 - 2012 - All Rights Reserved");
 MODULE_DESCRIPTION("IDL SMARTCARD Device Driver for Linux 2.6");
 MODULE_SUPPORTED_DEVICE("Intel Media Processors");
 MODULE_LICENSE("Dual BSD/GPL"); /* Inform kernel that driver is not GPL. */

/* Unique name for driver */
static char devname[] = {"scard"};

#ifndef MOD_NAME
#define MOD_NAME "idl_smartcard.ko"
#endif

char *version_string = "#@#" MOD_NAME " " VER;


/* This function is the first function called to load the driver after an insmod */
/* command */

int
scard_init(void);

/* This function is the first function called to unload the driver after an rmmod */
/* command */

void
scard_cleanup(void);

/* This function is called when an application tries to open a connection to the */
/* driver */

int
scard_open(struct inode *p_inode, struct file *p_file);

/* This function is called when an application disconnects from the driver */

int
scard_close(struct inode *p_inode, struct file *p_file);


/* This function is called when an application requests that the driver performs a */
/* task or service on the application's behalf */

long
scard_unlocked_ioctl(struct file *p_file, u_int command, u_long p_args);
/* Specify driver entry points for kernel */
/* Macros are defined in linux/module.h */
module_init(scard_init);
module_exit(scard_cleanup);

static void smcard_handler(unsigned long ingnored);
static void irq_handler(void *data);
static struct timer_list smcard_event_timer[SC_MAX_SLOTS_SUPPORTED] = { \
	TIMER_INITIALIZER(smcard_handler, 0, 0), 
	TIMER_INITIALIZER(smcard_handler, 0, 1)  
};
static int TIME_INTERVAL_SM = 50;

static sc_slot_info_t g_slot_info[SC_MAX_SLOTS_SUPPORTED];

static bool device_open_count[SC_MAX_SLOTS_SUPPORTED]={false, false};

/* This is the major number assigned dynamically assigned by Linux */
/* This number is required by the mknod command */
static int m_scard_major_number;

static struct pci_driver scard_pci_driver;
/* Structure to map driver functions to kernel */
struct file_operations m_scard_ops = {
        .owner   = THIS_MODULE,
        .unlocked_ioctl = scard_unlocked_ioctl,
        .open    = scard_open, 
        .release = scard_close,
};

uint32_t scard_irq;
uint32_t ce4200_d0stepping_flag = 0;


/*------------------------------------------------------------------------------
 * Support structure for interrupt callback mechanism
 *------------------------------------------------------------------------------
 */
static os_event_t scard_interrupt_event[SC_MAX_SLOTS_SUPPORTED];
/*------------------------------------------------------------------------------
 * Functions to support interrupt callback mechanism
 *------------------------------------------------------------------------------
 */

 /*------------------------------------------------------------------------------
 * Functions to support interrupt callback mechanism
 *------------------------------------------------------------------------------
 */
static uint32_t event_status[SC_MAX_SLOTS_SUPPORTED];
static int  has_event_handler[SC_MAX_SLOTS_SUPPORTED];

/* Interrupt handler for SmartCard  - using os_acquire interrupt */
void irq_handler(void *data)
{
	 uint32_t irq_status=0;
     int slot = (int)data;	
	
	sc_infc_get_event_status((uint8_t)slot, &irq_status);
	if (irq_status == 0)
		return;

  	event_status[slot] = irq_status;
	scard_infc_irq_handler(slot);	
   	//del_timer(&smcard_event_timer);
	if (has_event_handler[slot] && (irq_status && SC_EVENT_CARD_DET))
  	 	mod_timer(&smcard_event_timer[slot], jiffies + TIME_INTERVAL_SM);

}
/* handler for timer, check whether a smartcard event happenned or not*/
void smcard_handler(unsigned long data)
{
   int slot = (int)data;
   sc_slot_info_t t_slot_info;
   // check whether the status is changed.   yes to continue, no to abort.
   sc_infc_get_slot_info(slot, &t_slot_info);
   if (t_slot_info.status == g_slot_info[slot].status)
      return;
   g_slot_info[slot].status = t_slot_info.status;
   // end check
   if (has_event_handler[slot])
		os_event_set(&scard_interrupt_event[slot]);
}       

/*------------------------------------------------------------------------------
 * scard_init
 *------------------------------------------------------------------------------
 */

int
scard_init(void)
{
    int  slot;
	int retval = 0;

	retval = pci_register_driver(&scard_pci_driver);
	if (retval) {
		printk(KERN_ERR "Smartcard pci register failure\n");
		return retval;
	}

    /* Register the device with the operating system */	
    if ((m_scard_major_number = register_chrdev(0, devname, &m_scard_ops)) < 0) {
        	printk(KERN_ERR "%s:%4i: scard_init failed\n", __FILE__, __LINE__);
            return m_scard_major_number;
	}		
    for (slot = 0; slot < SC_MAX_SLOTS_SUPPORTED; slot++) {
        if (sc_infc_init(slot) < 0) {
        	 printk(KERN_ERR "%s:%4i: IDL SmartCard initialization failed.\n",  __FILE__, __LINE__);
             return -1; /* initialization failed */
        }
        /* initialize event handlers prior to enabling interrupts */       

        os_event_create(&scard_interrupt_event[slot], 0);
    }
    return 0;
}
        
/*------------------------------------------------------------------------------
 * scard_cleanup
 *------------------------------------------------------------------------------
 */

void
scard_cleanup(void)
{
    int  slot;

    /* un-register the device from operating system */
    unregister_chrdev(m_scard_major_number, devname);
	
	for (slot = 0; slot < SC_MAX_SLOTS_SUPPORTED; slot++) {
    	/* release smartcard */
        sc_infc_deinit(slot);

        /* destroy event handlers */
        os_event_destroy(&scard_interrupt_event[slot]);

   }
   pci_unregister_driver(&scard_pci_driver);
}
        
/*------------------------------------------------------------------------------
 * scard_open
 *------------------------------------------------------------------------------
 */

int
scard_open(struct inode *p_inode, struct file *p_file)
{
	uint8_t slot = iminor(p_inode);

	if (slot >= SC_MAX_SLOTS_SUPPORTED)
		return -EINVAL;
	if(device_open_count[slot]){
			printk("The smartcard decice has already been opened!\n");
			return -1;
        } else {
        	set_scard_alt_fun_pin(slot, 1);
			device_open_count[slot]=true;
			has_event_handler[slot] = false;
    	    sc_infc_register_event_handler(slot, irq_handler,(void *)slot);
   			mod_timer(&smcard_event_timer[slot], jiffies + TIME_INTERVAL_SM);
        }

	return 0;
}
        
/*------------------------------------------------------------------------------
 * scard_close
 *------------------------------------------------------------------------------
 */
        
int
scard_close(struct inode *p_inode, struct file *p_file)
{
	uint8_t slot = iminor(p_inode);

	if (slot >= SC_MAX_SLOTS_SUPPORTED)
		return -EINVAL;
	if(!device_open_count[slot]){
		printk("The smartcard decice has already been closed!\n");
		return -1;
	}
    sc_infc_release_event_handler(slot);
	set_scard_alt_fun_pin(slot, 0);
	has_event_handler[slot] = false;
	device_open_count[slot]=false;
	return 0;
}
        


/*------------------------------------------------------------------------------
 * scard_unlocked_ioctl
 *------------------------------------------------------------------------------
 */
long
scard_unlocked_ioctl(struct file *p_file, u_int command, u_long args)
{
        uint8_t         slot;
        int             status = 0;        
		sc_ioctl_args_t sc_args;
		sc_ioctl_args_t *p_args = &sc_args;
		sc_slot_info_t slot_info;
		sc_section_t section;
		uint8_t *pdata;
		uint32_t len;

        /* Check to see if the ioctl command is meant for this device */
        if (_IOC_TYPE(command) != SC_IOCTL_MAGIC) {
                printk(KERN_ERR "%s:%4i:  scard_ioctl failed\n", __FILE__, __LINE__);
                return -ENOTTY;
        }
        
        /* Check for valid pointer to the parameter list */
        if (args == 0) {
                printk(KERN_ERR "%s:%4i:  scard_ioctl failed\n", __FILE__, __LINE__);
                return -EINVAL;
        }
   		if (copy_from_user((void *)p_args, (const void __user *) args, sizeof(sc_ioctl_args_t))) {
         	printk(KERN_ERR "%s:%4i:  scard_ioctl failed\n", __FILE__, __LINE__);
         	return -EFAULT;
   		}
        
        /* Execute ioctl request */
        switch (command) {
                case SC_IOCTL_GET_SLOT_INFO:
                        if (sc_infc_get_slot_info(
                                    (uint8_t)p_args->params[0],
									&slot_info) < 0) {
                                status = -EINVAL;
                        } else {
							if (copy_to_user((void *)p_args->params[1], &slot_info, sizeof(slot_info))) {
								status =  -EFAULT;
							}	
						}
                        break;

                case SC_IOCTL_GET_TX_LENGTH:
                        if (sc_infc_get_tx_length(
                                    (uint8_t)p_args->params[0],
                                    (uint32_t *) &p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;

                case SC_IOCTL_GET_RX_LENGTH:
                        if (sc_infc_get_rx_length(
                                    (uint8_t)p_args->params[0],
                                    (uint32_t *)&p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;

                case SC_IOCTL_RESET_RXFIFO:
                        if (sc_infc_reset_rx_fifo(
                                    (uint8_t )p_args->params[0]) < 0) {
                                status = -EINVAL;
                        }
                        break;

                case SC_IOCTL_RESET_TXFIFO:
                        if (sc_infc_reset_tx_fifo(
                                    (uint8_t )p_args->params[0]) < 0) {
                                status = -EINVAL;
                        }
                        break;

                case SC_IOCTL_START_SECTION:
						if (copy_from_user(&section, (void *)p_args->params[1], sizeof(section))) {
							status = -EFAULT;
							break;
						}
                        if (sc_infc_start_section(
                                    (uint8_t)p_args->params[0],
                                    &section) < 0) {
                                status = -EINVAL;
                        } else {
							if (copy_to_user((void *)p_args->params[1], &section, sizeof(section))) {
								status = -EFAULT;
							}
						}
                        break;

                case SC_IOCTL_REGISTER_EVENT_HANDLER:
                        slot = (uint8_t)p_args->params[0];
                        has_event_handler[slot] = true;
                        break;

                case SC_IOCTL_RELEASE_EVENT_HANDLER:
                        slot = (uint8_t)p_args->params[0];
                        has_event_handler[slot] = false;
                        break;

                case SC_IOCTL_WAIT_FOR_EVENT:
                        slot = (uint8_t)p_args->params[0];
            			if ( slot >= SC_MAX_SLOTS_SUPPORTED) {
							status = -EINVAL;
							break;
						}
                        if ( (os_event_wait(&scard_interrupt_event[slot], -1) != OSAL_SUCCESS) ) {
                                status = -ERESTARTSYS;
                        }
                        else {
                                os_event_reset(&scard_interrupt_event[slot]);
                        }

                        break;

                case SC_IOCTL_RELEASE_EVENT:
                        slot = (uint8_t)p_args->params[0];
            			if ( slot >= SC_MAX_SLOTS_SUPPORTED) {
							status = -EINVAL;
							break;
						}
                        os_event_set(&scard_interrupt_event[slot]);

                        break;

                case SC_IOCTL_NOTIFY_EVENT_DONE:
                case SC_IOCTL_CLEAR_EVENT:
                        event_status[(uint8_t)p_args->params[0]] &= ~(uint32_t)p_args->params[1];
                        break;

                case SC_IOCTL_GET_EVENT_STATUS:
                        event_status[(uint8_t)p_args->params[0]] = (event_status[(uint8_t)p_args->params[0]] & SC_EVENT_CARD_DET);
                        p_args->params[1] = (uint32_t)event_status[(uint8_t)p_args->params[0]];
                        break;

                case SC_IOCTL_READ:
                        len = p_args->params[2];
                        pdata = kmalloc(len, GFP_KERNEL);
                        if (!pdata) {
                              status = -ENOMEM;
                              break;
                        }   
                        status = sc_infc_read(
                                (uint8_t)p_args->params[0],
                                pdata,
                                len,
                                (uint32_t *)&p_args->params[3]);
						if (p_args->params[3] > 0) {
							if (copy_to_user((void *)p_args->params[1], pdata, p_args->params[3]))
								status = -EFAULT;
						}
						kfree(pdata);
                        break;

                case SC_IOCTL_WRITE:
						len = p_args->params[2];
      					pdata = kmalloc(len, GFP_KERNEL);
  					    if (!pdata) {
     						status = -ENOMEM;
					    	break;
    					}   
						if (copy_from_user(pdata, (void *)p_args->params[1], len)) {
							kfree(pdata);
							status = -EFAULT;
							break;
						}
                        status = sc_infc_write(
                                (uint8_t)p_args->params[0],
                                pdata,
                                len,
                                (uint32_t *)&p_args->params[3]);
						kfree(pdata);
                        break;

                case SC_IOCTL_DEACTIVATE:
                        if (sc_infc_card_deactivate(
                                    (uint8_t)p_args->params[0]) < 0) {
                                status = -EINVAL;
                        }
                        break;

                case SC_IOCTL_POWER_ON:
                        if (sc_infc_set_power_on(
                                    (uint8_t)p_args->params[0],
                                    (sc_vcc_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
               case SC_IOCTL_CARD_REMOVAL: 
                        if (sc_infc_card_removal(
                                    (uint8_t)p_args->params[0]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_WARM_RESET: 
                        if (sc_infc_warm_reset(
                                    (uint8_t)p_args->params[0]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_IO_MODE: 
                        if (sc_infc_set_iomode(
                                    (uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_CWTR: 
                        if (sc_infc_set_cwt(
                                    (uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_BWTR: 
                        if (sc_infc_set_bwt(
                                    (uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
				case SC_IOCTL_SET_DDS: 
						if (sc_infc_set_dds(
									(uint8_t)p_args->params[0],
									(uint32_t)p_args->params[1]) < 0) {
								status = -EINVAL;
						}
						break;
                case SC_IOCTL_SET_TOR: 
                        if (sc_infc_set_tor(
                                    (uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_CLOCK: 
                        if (sc_infc_set_clk_divisor(
                                    (uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_BGTR: 
                        if (sc_infc_set_bgtr(
                                    (uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_EGTR: 
                        if (sc_infc_set_egtr(
                                	(uint8_t)p_args->params[0],
                                	(uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                        break;
                case SC_IOCTL_SET_ETU: 
                        if (sc_infc_set_etu(
                                (uint8_t)p_args->params[0],
                                (uint8_t)p_args->params[1],
                                (uint32_t)p_args->params[2],
                                (uint32_t)p_args->params[3]) < 0) {
                                status = -EINVAL;
                        }
                        break;
				
			
                case SC_IOCTL_ADJUST_RESET_DELAY:
                        if (sc_infc_adjust_reset_delay(
                                    (uint8_t)p_args->params[0],
                                    (uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }
                  break;
                        
               case SC_IOCTL_ADJUST_RESET_TIME:
                  if (sc_infc_adjust_reset_time(
                                    (uint8_t)p_args->params[0],
                                    (uint32_t)p_args->params[1]) < 0) {
                     status = -EINVAL;
                  }
                  break;

               case SC_IOCTL_FORCE_DELAY_BTN_LINES:
                  if (sc_infc_force_delay_between_lines(
                                    (uint8_t)p_args->params[0],
                                    (uint32_t)p_args->params[1]) < 0) {
                     status = -EINVAL;
                  }
                  break;
               
               case SC_IOCTL_SET_DATA_PIN:
                  if (sc_infc_set_data_pin(
                                    (uint8_t)p_args->params[0],
                                    (uint8_t)p_args->params[1]) < 0) {
                     status = -EINVAL;
                  }
                  break;

               case SC_IOCTL_SET_CCR_USER_BITS:
                  if (sc_infc_set_ccr_user_bits(
                                    (uint8_t) p_args->params[0],
                                    (uint8_t) p_args->params[1]) < 0) {
                     status = -EINVAL;
                  }
                  break;
#if 0
                case IOCTL_SMARTCARD_GET_FEATURE:
                        ((idl_smartcard_ioctl_args_t *)p_args)->result = idl_sc_get_feature(
                                (idl_smartcard_feature_t *)ioctl_args.params[0]);
                        
                        if ( ((idl_smartcard_ioctl_args_t *)p_args)->result != IDL_SUCCESS ) {
                                status = -EINVAL;
                        }
                        break;

                case IOCTL_SMARTCARD_RESET_CONFIG:
                        ((idl_smartcard_ioctl_args_t *)p_args)->result = idl_sc_reset_config(
                                *((uint32_t *) (((idl_smartcard_ioctl_args_t *)p_args)->params[0])));
                        
                        if ( ((idl_smartcard_ioctl_args_t *)p_args)->result != IDL_SUCCESS ) {
                                status = -EINVAL;
                        }

                        break;
					
                case SC_IOCTL_SET_RX_TRIGGER_LEVEL:
                        if (sc_infc_set_rx_trigger_level(
                                    *((uint8_t *) (((sc_ioctl_args_t *)p_args)->params[0])),
                                    *((uint16_t *) (((sc_ioctl_args_t *)p_args)->params[1]))) < 0) {
                                status = -EINVAL;
                        }

                        break;

                case SC_IOCTL_SET_TX_TRIGGER_LEVEL:
                        if (sc_infc_set_tx_trigger_level(
                                    *((uint8_t *) (((sc_ioctl_args_t *)p_args)->params[0])),
                                    *((uint16_t *) (((sc_ioctl_args_t *)p_args)->params[1]))) < 0) {
                                status = -EINVAL;
                        }

                        break;
#endif
		case SC_IOCTL_SET_PROTOCOL:
                        if (sc_infc_set_protocol(
                                    (uint8_t )p_args->params[0],
                                    (uint32_t)p_args->params[1]) < 0) {
                                status = -EINVAL;
                        }

                        break;

                default:
                        /* POSIX compliance return returning -ENOTTY if invalid */
                        status = -ENOTTY;
        }

	if (((status == 0) || (command == SC_IOCTL_READ)) && copy_to_user((void *) args, p_args, sizeof(sc_ioctl_args_t))) {
		status = -EFAULT;
	}
    return status;
}

static const struct pci_device_id scard_id_tables[] __devinitdata = {
{ PCI_DEVICE(0x8086, 0x2e69), },
{0 },
};
MODULE_DEVICE_TABLE(pci, scard_id_tables);


static int __devinit scard_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
  	pal_soc_info_t pal_info;
	pal_soc_name_t soc;
	int retval = 0;
	if (PAL_SUCCESS != pal_get_soc_info(&pal_info)) {
		printk(KERN_ERR "Can't get SOC_INFO!\n");
		return -1;
	}
	soc = pal_info.name;
	switch (soc) {
	 		case SOC_NAME_CE3100:
			     scard_infc_set_ahb_clk(0, 133333);
			     scard_infc_set_ahb_clk(1, 133333);
				 printk("CE3100 Smartcard Controller found currently!\n");
				 break;
			case SOC_NAME_CE4100:
			     scard_infc_set_ahb_clk(0, 133333);
			     scard_infc_set_ahb_clk(1, 133333);
			     printk("CE4100 Smartcard Controller found currently!\n");
				 break;
			case SOC_NAME_CE4200:
			     if(pal_info.stepping == SOC_STEPPING_D0){
				pal_board_info_t board_info;
				ce4200_d0stepping_flag = 1;
				if (PAL_SUCCESS == pal_get_board_info(&board_info)) {
					switch(board_info.ddr_speed){
						case DDR_SPEED_1333MHZ:
							scard_infc_set_ahb_clk(0, 1333333);
							scard_infc_set_ahb_clk(1, 1333333);
							break;
						case DDR_SPEED_1066MHZ:
							scard_infc_set_ahb_clk(0, 1066666);
							scard_infc_set_ahb_clk(1, 1066666);
							break;
						case DDR_SPEED_800MHZ:
							scard_infc_set_ahb_clk(0, 800000);
							scard_infc_set_ahb_clk(1, 800000);
							break;
						default:
							printk("Unsupported DDR Speed!");
							retval = -1;
							break;
					}
				}
			     }
			     else {
				scard_infc_set_ahb_clk(0, 133333);
				scard_infc_set_ahb_clk(1, 133333);
			     }
						
			     printk("CE4200 Smartcard Controller found currently!\n");
			     break;
			case SOC_NAME_CE5300:
			     scard_infc_set_ahb_clk(0, 133333);
			     scard_infc_set_ahb_clk(1, 133333);
			     printk("CE5300 Smartcard Controller found currently!\n");
				 break;
			case SOC_NAME_CE2600:
			     scard_infc_set_ahb_clk(0, 125000);
			     scard_infc_set_ahb_clk(1, 125000);
				 printk("CE2600 Smartcard Controller found currently!\n");
				 break;
			default:
				 printk(KERN_ERR "The platform is not recognized!\n");
		  		 retval = -1;
				 break;
	}	
	if (retval) return retval;
	
	pci_enable_device(pdev);
	pci_intx(pdev, 1);
	scard_irq = pdev->irq;
	scard_dbg("Smartcard irq line is %d\n", scard_irq);
	pci_dev_get(pdev);
    return retval;
}

static void __devexit scard_pci_remove(struct pci_dev *pdev)
{

	pci_disable_device(pdev);
	pci_intx(pdev, 0);
	pci_dev_put(pdev);
	scard_irq = 0;
}
#ifdef CONFIG_PM  
/* scard pci device suspend */
static int scard_pci_suspend(struct pci_dev *pdev, pm_message_t state)
{
    int  slot = 0;
  	int retval = 0;
    /*smartcard suspend */
    for (slot=0; slot < SC_MAX_SLOTS_SUPPORTED; slot++) {
		retval = sc_infc_suspend(slot);
		if (retval) return retval;
	}	
	/*pci device save*/
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, pci_choose_state(pdev, state));
	return retval;
}

/* scard pci device resume */
static int scard_pci_resume(struct pci_dev *pdev)
{
    int  slot = 0;
    int retval = 0;
	/*pci device restore*/
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	retval = pci_enable_device(pdev);
	if (retval)
	   	 return retval;
    /*smartcard resume */
    for (slot=0; slot < SC_MAX_SLOTS_SUPPORTED; slot++) {
		retval = sc_infc_resume(slot);
		if (retval) return retval;
	}	
    return retval;
}
#endif
static struct pci_driver scard_pci_driver = {
	.name = "scard_pci_driver",
	.id_table = scard_id_tables,
	.probe = scard_pci_probe,
	.remove = __devexit_p(scard_pci_remove),
#ifdef CONFIG_PM
	.suspend = scard_pci_suspend,
	.resume = scard_pci_resume,
#endif
};

