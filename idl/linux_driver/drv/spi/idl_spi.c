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
 * File Name:       idl_spi_shim.c
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
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>

#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include "idl_spi.h"

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
    #include <asm/semaphore.h>
#else
    #include <linux/semaphore.h>
#endif

#define REMOVE_PRINTS 1
#define SPI_MASTER_BUS_NUM 0
#define SPI_BUF_SIZE   4096

/* Number of SPI ports supported in this implementation. */
#define NUM_SPI_PORTS   1

#define SPI_DATA_SIZE_MIN            4
#define SPI_DATA_SIZE_MAX            32 

#define SPI_MAX_BIT_RATE             5000000
#define SPI_MIN_BIT_RATE             7200
#define SPI_STATUS_ALT_FRM_MAX       3
#define SPI_FIFO_DEPTH               32 

//#define SPI_SHIM_DEBUG
struct spidev_data {
        dev_t                   devt;
        spinlock_t              spi_lock;
        struct spi_device       *spi;
        struct list_head        device_entry;

        /* buffer is NULL unless this device is open (users > 0) */
        struct mutex            buf_lock;
        unsigned                users;
        u8                      *buffer;
};


struct spi_port_info {
	uint32_t chip_select;
	bool  config_flag;
};

struct spi_read_fifo {
       uint32_t data[SPI_FIFO_DEPTH];
       uint32_t rindex;
       uint32_t windex;
       uint32_t count;
       spinlock_t mutex;
       int (*is_full)(struct spi_read_fifo *fifo);
       int (*is_empty)(struct spi_read_fifo *fifo);
       int (*write)(struct spi_read_fifo *fifo, uint32_t data);
       int (*read)(struct spi_read_fifo *fifo, uint32_t *data);
       uint32_t (*add)(uint32_t i);
       int (*init)(struct spi_read_fifo *fifo);
};

/* These are predefined macros that specify different parameters
 * for our driver */ 
MODULE_AUTHOR("Intel Corporation, (C) 2011 - All Rights Reserved");
MODULE_DESCRIPTION("Wrapper to map Intel 'IDL' SPI API to Linux SPI API");
MODULE_SUPPORTED_DEVICE("Intel Media Processors");

/* Notifies the kernel that our driver is not GPL. */
MODULE_LICENSE("Dual BSD/GPL");

/* Unique name for SPI driver */
#define SPI_DEV_NAME "spi_shim"

#ifndef MOD_NAME
#define MOD_NAME "idl_spi_shim.ko"
#endif

extern struct list_head device_list;
extern struct mutex device_list_lock;
extern ssize_t spidev_sync_write(struct spidev_data *spidev, size_t len);
extern ssize_t spidev_sync_read(struct spidev_data *spidev, size_t len);
extern ssize_t spidev_sync(struct spidev_data *spidev, struct spi_message *message);

static struct spi_port_info spi_port_chip_select[NUM_SPI_PORTS];
static struct spi_read_fifo read_fifo;


char *version_string = "#@#" MOD_NAME " " VER;

/* These are the symbols we want exported to other modules */
EXPORT_SYMBOL(idl_spi_init);
EXPORT_SYMBOL(idl_spi_release);
EXPORT_SYMBOL(idl_spi_set_config);
EXPORT_SYMBOL(idl_spi_get_status);
EXPORT_SYMBOL(idl_spi_read);
EXPORT_SYMBOL(idl_spi_write);

static inline uint32_t fifo_add(uint32_t i)
{
  return ((i + 1) == SPI_FIFO_DEPTH? 0:(i + 1));
}

static int is_full(struct spi_read_fifo * fifo)
{
  if (fifo->count == SPI_FIFO_DEPTH ) {
    return 1;
  }
  return 0;
}

static int is_empty(struct spi_read_fifo * fifo)
{
  if (fifo->count == 0 ) {
    return 1;
  }
  return 0;
}

static int fifo_write(struct spi_read_fifo * fifo, uint32_t data)
{
  spin_lock(&fifo->mutex);
  if (!fifo->is_full(fifo)) {
     fifo->data[fifo->windex] = data;
     fifo->windex = fifo->add(fifo->windex);
     fifo->count++;
     spin_unlock(&fifo->mutex);
     return 0;
  }
  spin_unlock(&fifo->mutex);
  return 1;

}
static int fifo_read(struct spi_read_fifo * fifo, uint32_t *data)
{
  uint32_t  pos;
  spin_lock(&fifo->mutex);
  if (!fifo->is_empty(fifo)) {
     pos = fifo->rindex;
     fifo->rindex = fifo->add(fifo->rindex);
     fifo->count--;
     *data = fifo->data[pos];
     spin_unlock(&fifo->mutex);
     return 0;
  }
  spin_unlock(&fifo->mutex);
  return 1;
}

static int fifo_init(struct spi_read_fifo * fifo)
{

  memset(&fifo->data, 0, sizeof(uint32_t)*SPI_FIFO_DEPTH);
  fifo->count = 0;
  fifo->windex = 0;
  fifo->rindex = 0;
  spin_lock_init(&fifo->mutex);
  return 0;
}



/* High level spi initialization - called at driver load time 
* This function will pnly be called by insmod
*/
int spi_init()
{
  memset(&spi_port_chip_select,0,sizeof(spi_port_chip_select));
  read_fifo.read = fifo_read;
  read_fifo.write = fifo_write;
  read_fifo.init = fifo_init;
  read_fifo.add = fifo_add;
  read_fifo.is_full = is_full;
  read_fifo.is_empty = is_empty;
  read_fifo.init(&read_fifo);

  return 0;

}

/* called when the system unloads our driver 
*	Called only by rmmod
*/
void spi_cleanup()
{
  return;
}

/* tell the kernel the name of our entry points (these macros are defined in linux/module.h */
module_init(spi_init);
module_exit(spi_cleanup);

bool spi_is_valid_port_num( uint32_t port_num )
{
	return ( port_num < NUM_SPI_PORTS );
}

bool spi_is_configed( uint32_t port_num )
{
	return ( spi_port_chip_select[port_num].config_flag != 0 );
}

/* Set the bit rate (i.e. clock rate) */
bool spi_set_bit_rate( struct spi_device * spi, uint32_t bit_rate )
{
	/* 
	 * Forumula for caluclating the SCR value from the bit rate:
	 *    SCR = ((Frequency / BitRate) - 1)
	 */

	bool return_value = false;

	if ( (bit_rate >= SPI_MIN_BIT_RATE) && (bit_rate <= SPI_MAX_BIT_RATE) ) {
                   spi->max_speed_hz = bit_rate;	
		return_value = true;
	}

	return ( return_value );
}


/* Set the data size (each data element can be 4 through 16 bits wides). */
bool spi_set_data_size( struct spi_device * spi, uint32_t data_size )
{
 	bool return_value = false;

	// If the specified data size is supported, then set it.
	if ( (data_size >= SPI_DATA_SIZE_MIN) &&
		 (data_size <= SPI_DATA_SIZE_MAX) ) {
                  spi->bits_per_word = data_size;
		return_value = true;
	}

	return ( return_value );
}

bool spi_set_phase_type(  struct spi_device * spi, uint32_t phase_type )
{
	bool return_value = false;

	switch (phase_type) {
		case PHASE_ONE_HALF:
			spi->mode  &= ~(SPI_CPHA);
			return_value = true;
			break;
		case PHASE_HALF_ONE:
			spi->mode  |= SPI_CPHA;
			return_value = true;
			break;
	}

	return ( return_value );
}


bool spi_set_clock_polarity( struct spi_device * spi, uint32_t polarity )
{
	bool return_value = false;

	switch (polarity) {
		case INACTIVE_LOW:
		         spi->mode &= ~(SPI_CPOL);
			return_value = true;
			break;
		case INACTIVE_HIGH:
			spi->mode |= SPI_CPOL;
			return_value = true;
			break;
	}

	return ( return_value );
}


bool spi_set_chip_select( uint32_t port_num, uint32_t chip_select )
{
	bool return_value = false;

	if ( chip_select <= SPI_STATUS_ALT_FRM_MAX ) {
		spi_port_chip_select[port_num].chip_select = chip_select;
		return_value = true;
	}

	return ( return_value );
}


struct spidev_data * get_spidev_by_port(  uint32_t spi_port_num )
{
	struct spidev_data	*spidev = NULL;
	 if ( !spi_is_valid_port_num(spi_port_num) )
	     return NULL;
	mutex_lock(&device_list_lock);
		
	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->spi->master->bus_num == spi_port_num &&
			spidev->spi->chip_select == spi_port_chip_select[spi_port_num].chip_select) {
			break;
		}
	}
		
	mutex_unlock(&device_list_lock);
	return spidev;

}
idl_result_t idl_spi_init(uint32_t spi_port_num)
{	
       return IDL_SUCCESS;
}


idl_result_t idl_spi_release(uint32_t spi_port_num)
{
  if ( spi_port_num  >=  NUM_SPI_PORTS){
        printk("idl_spi_set_config: spi_port_num = %u, NUM_SPI_PORTS = %u\n", spi_port_num, NUM_SPI_PORTS);
        return IDL_INVALID_PARAM;
  }

       spi_port_chip_select[spi_port_num].config_flag = false;
 
       return IDL_SUCCESS;
}

idl_result_t idl_spi_set_config(uint32_t spi_port_num, spi_config_t *spi_config) 
{ 
        idl_result_t return_status = IDL_NOT_IMPLEMENTED; 
	struct spidev_data *spidev;
	struct spi_device * spi;
	uint32_t status;

        if ( !spi_is_valid_port_num(spi_port_num) ){
	     return IDL_INVALID_PARAM;
	}
		
	if ( !spi_set_chip_select( spi_port_num, spi_config->chip_select)  ) {
                return IDL_INVALID_PARAM;
	}
				
	spidev = get_spidev_by_port( spi_port_num );
		   
	if ( !spidev ) {
	  return IDL_INVALID_PARAM;
	}
	
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL) {
		return IDL_INVALID_PARAM;
	}
        else if ( !spi_set_bit_rate(spi, spi_config->clock_rate) ) {
                return_status = IDL_INVALID_PARAM;
        }
        else if ( !spi_set_data_size(spi, spi_config->data_size) ) {
                return_status = IDL_INVALID_PARAM;
        }
        else if ( !spi_set_clock_polarity(spi, spi_config->clock_polarity) ) {
                return_status = IDL_INVALID_PARAM;
        }
        else if ( !spi_set_phase_type( spi, spi_config->phase)  ) {
                return_status = IDL_INVALID_PARAM;
        }
        else {
	       spi_port_chip_select[spi_port_num].config_flag = true;
	       spi->master->setup(spi);
                return_status = IDL_SUCCESS;
        }      
        return ( return_status );
}

idl_result_t idl_spi_get_status(uint32_t spi_port_num, spi_status_t *spi_status)
{             
               return  IDL_SUCCESS;
}

int idl_spi_shim_message(struct spidev_data *spidev, struct spi_transfer *k_xfers)
{
	struct spi_message	msg;
        int  status = -EFAULT;

	if (k_xfers == NULL)
		return -ENOMEM;

	spi_message_init(&msg);
#ifdef SPI_SHIM_DEBUG
	printk("k_xfer.tx_buf=0x%x,rx_buf =%x, len=%x\n",*((uint32_t *)k_xfers->tx_buf),*((uint32_t *)k_xfers->rx_buf), k_xfers->len);
#endif
	spi_message_add_tail(k_xfers, &msg);


	status = spidev_sync(spidev, &msg);
         return status;
}

/*
  *  Support Full Duplex for SPI transaction
  */
idl_result_t idl_spi_write_and_read(uint32_t spi_port_num, uint32_t indata, uint32_t * outdata)
{
        struct spidev_data	*spidev;
        struct spi_transfer xfer;
        uint32_t txbuf = 0;
        uint32_t rxbuf = 0;
        uint32_t status;
        uint8_t bits;	

	if (!outdata)
		return IDL_INVALID_PARAM;
	
        if ( !spi_is_configed(spi_port_num) ) 
		return  IDL_NOT_INITIALIZED;
	
		 
        spidev = get_spidev_by_port( spi_port_num );
	if (!spidev) 
		return IDL_INVALID_PARAM;

	memset(&xfer, 0, sizeof(xfer));

        bits = spidev->spi->bits_per_word;
	txbuf = indata;
	xfer.tx_buf = (void*)&txbuf;
	xfer.rx_buf = (void*)&rxbuf;
	xfer.len = sizeof(uint32_t);
	xfer.bits_per_word = bits;

        if (bits <= 8) {
          xfer.len = sizeof(uint8_t);
        }
        else if (bits <= 16) {
          xfer.len = sizeof(uint16_t);
        }
        else if (bits <= SPI_DATA_SIZE_MAX ) {
          xfer.len = sizeof(uint32_t);
        }
        else {
            return IDL_FAILURE;
        }
#ifdef SPI_SHIM_DEBUG	
	printk("xfer.tx_buf=0x%x,rx_buf =%x, len=%x\n",xfer.tx_buf,xfer.rx_buf, xfer.len);
	printk("xfer.tx_buf=0x%x,rx_buf =%x, len=%x\n",*((uint32_t *)xfer.tx_buf),*((uint32_t *)xfer.rx_buf), xfer.len);
#endif
	status = idl_spi_shim_message(spidev, &xfer);
	*outdata = rxbuf;
#ifdef SPI_SHIM_DEBUG	
	printk("status=0x%x,indata =%x, outdata=%x\n",status,indata, *outdata);
#endif
	if (status > 0)
	   return IDL_SUCCESS;

	   return IDL_FAILURE;
	
}

 /*
  *  Support TX SPI transtaction 
  */
idl_result_t idl_spi_write(uint32_t spi_port_num, uint32_t data)
{
  uint32_t outdata;
  idl_result_t ret;
  ret = idl_spi_write_and_read(spi_port_num, data, &outdata);
  if ( ret != IDL_SUCCESS ) {
    return IDL_FAILURE;
  }
  ret = read_fifo.write(&read_fifo, outdata);
  if (ret) {
        return IDL_DEVICE_BUSY;
  }

  return IDL_SUCCESS;
}


/*
  *  Support RX SPI transtaction 
  */
idl_result_t idl_spi_read(uint32_t spi_port_num, uint32_t *data)
{
   
   idl_result_t ret;
   if (!data)
     return IDL_INVALID_PARAM;

   if ( !spi_is_configed(spi_port_num) )
     return  IDL_NOT_INITIALIZED;

   ret = read_fifo.read(&read_fifo, data);
   if (ret) {
         return IDL_DEVICE_BUSY;
   }
  return IDL_SUCCESS;
}


