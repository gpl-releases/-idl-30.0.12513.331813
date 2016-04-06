/*
#
# This file is provided under a dual BSD/LGPLv2.1 license.  When using 
# or redistributing this file, you may do so under either license.
#
# LGPL LICENSE SUMMARY
#
# Copyright(c) 2007-2011. Intel Corporation. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify 
# it under the terms of version 2.1 of the GNU Lesser General Public 
# License as published by the Free Software Foundation.
#
# This library is distributed in the hope that it will be useful, but 
# WITHOUT ANY WARRANTY; without even the implied warranty of 
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public 
# License along with this library; if not, write to the Free Software 
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 
# USA. The full GNU Lesser General Public License is included in this 
# distribution in the file called LICENSE.LGPL.
#
# Contact Information:
#     Intel Corporation
#     2200 Mission College Blvd.
#     Santa Clara, CA  97052
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
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 */


#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>

#include "osal_memmap.h"
#include "osal_io.h"
#include "osal_thread.h"
#include "idl.h"
#include "idl_spi.h"
#include "linux/spi/spidev.h"

/* Number of SPI ports supported in this implementation. */
#define NUM_SPI_PORTS              1
#define SPI_STATUS_ALT_FRM_MAX     3
#define SPI_FIFO_DEPTH             32
#define SPI_DATA_SIZE_MIN          4
#define SPI_DATA_SIZE_MAX          32
//#define SPI_SHIM_DEBUG

struct spi_port_info {
	uint32_t chip_select;
	bool  config_flag;
};

struct spi_read_fifo {
       uint32_t data[SPI_FIFO_DEPTH];
       uint32_t rindex;
       uint32_t windex;
       uint32_t count;
       pthread_mutex_t mutex;
       int (*is_full)(struct spi_read_fifo *fifo);
       int (*is_empty)(struct spi_read_fifo *fifo); 
       int (*write)(struct spi_read_fifo *fifo, uint32_t data);
       int (*read)(struct spi_read_fifo *fifo, uint32_t *data);
       uint32_t (*add)(uint32_t i);
       int (*init)(struct spi_read_fifo *fifo);
};

static struct spi_read_fifo read_fifo;
static struct spi_port_info spi_port_chip_select[NUM_SPI_PORTS];

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
  uint32_t pos;
  pthread_mutex_lock(&fifo->mutex);
  if (!fifo->is_full(fifo)) {
     fifo->data[fifo->windex] = data;
     fifo->windex = fifo->add(fifo->windex);
     fifo->count++;
     pthread_mutex_unlock(&fifo->mutex);
     return 0;
  }
  pthread_mutex_unlock(&fifo->mutex);
  return 1;
  
}

static int fifo_read(struct spi_read_fifo * fifo, uint32_t *data)
{
  uint32_t  pos;
  pthread_mutex_lock(&fifo->mutex);
  if (!fifo->is_empty(fifo)) {
     pos = fifo->rindex; 
     fifo->rindex = fifo->add(fifo->rindex);
     fifo->count--;
     *data = fifo->data[pos]; 
     pthread_mutex_unlock(&fifo->mutex);
     return 0; 
  }
  pthread_mutex_unlock(&fifo->mutex);
  return 1;
}

static int fifo_init(struct spi_read_fifo * fifo)
{
  
  memset(&fifo->data, 0, sizeof(uint32_t)*SPI_FIFO_DEPTH);
  fifo->count = 0;
  fifo->windex = 0;
  fifo->rindex = 0;
  pthread_mutex_init(&fifo->mutex,NULL);
  return 0;
}

static int spi_open_fd (uint32_t spi_port_num, uint32_t chip_select)
{
	char  devbuf[128];
	const char *pathname;
	int fd;
	sprintf(devbuf, "/dev/spidev%d.%d",spi_port_num,chip_select);
	pathname = devbuf;
	fd = open(pathname, O_RDWR);
	if (fd == -1){
		printf("open %s error\n", pathname);
	}
	return fd;
}


idl_result_t idl_spi_init( uint32_t spi_port_num ) 
{
        memset(&spi_port_chip_select,0,sizeof(spi_port_chip_select));
        read_fifo.read = fifo_read; 
        read_fifo.write = fifo_write; 
        read_fifo.init = fifo_init; 
        read_fifo.add = fifo_add; 
        read_fifo.is_full = is_full; 
        read_fifo.is_empty = is_empty; 
        read_fifo.init(&read_fifo);
	return IDL_SUCCESS;
}

idl_result_t idl_spi_set_config(uint32_t spi_port_num, spi_config_t *spi_config)
{
	
	int spi_fd =-1;
	static uint8_t mode = SPI_MODE_0;
	int ret = IDL_FAILURE;

	if(NULL == spi_config) {
		printf("idl_spi_set_config: The second parameter, spi_config_t *, is NULL!\n");
		ret = IDL_INVALID_PARAM;
		return ret;
	}

	if ( spi_port_num  >=  NUM_SPI_PORTS){
		printf("idl_spi_set_config: spi_port_num = %u, NUM_SPI_PORTS = %u\n", spi_port_num, NUM_SPI_PORTS);
		ret  = IDL_INVALID_PARAM;
		return ret;
	}

	if (spi_config->chip_select <= SPI_STATUS_ALT_FRM_MAX ) {
		spi_port_chip_select[spi_port_num].chip_select = spi_config->chip_select ;
		spi_port_chip_select[spi_port_num].config_flag = true;
	}
	spi_fd = spi_open_fd( spi_port_num, spi_port_chip_select[spi_port_num].chip_select);
	if ( spi_fd < 0)
	{
		printf("idl_spi_set_config: Unable to open device \n");
	       return IDL_NO_MEDIA;
	 }

	/*
	 * spi mode
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		printf("can't set spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_config->data_size);
	if (ret == -1)
		printf("can't set bits per word");
	/*
	 * max speed hz
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_config->clock_rate);
	if (ret == -1)
		printf("can't set max speed hz");
	
	
	close(spi_fd);

	if ( ret <0 )
		return IDL_FAILURE;
	return IDL_SUCCESS;
}


idl_result_t idl_spi_get_status(uint32_t spi_port_num, spi_status_t *spi_status)
{
      return IDL_SUCCESS;
}

/*
  *  Support Full Duplex for SPI transaction
  */
idl_result_t idl_spi_write_and_read(uint32_t spi_port_num, uint32_t indata, uint32_t * outdata)
{
  	int spi_fd =-1;
	int ret = 0;
	uint8_t bits;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)&indata,
		.rx_buf = (unsigned long)outdata,
		.len = sizeof(uint16_t),
	};

	if ( outdata == NULL ) {
		return IDL_INVALID_PARAM;
	}


	if ( spi_port_num  >=  NUM_SPI_PORTS ) {
		printf("idl_spi_set_config: spi_port_num = %u, NUM_SPI_PORTS = %u\n", spi_port_num, NUM_SPI_PORTS);
		return IDL_INVALID_PARAM;
	}

	if ( !spi_port_chip_select[spi_port_num].config_flag ) {
		return IDL_NOT_INITIALIZED;
	}
     
		
	spi_fd = spi_open_fd( spi_port_num, spi_port_chip_select[spi_port_num].chip_select);
	
	if ( spi_fd < 0) {
	    printf("idl_spi_set_config: Unable to open device \n");
	    return IDL_NO_MEDIA;
	 }

	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
#ifdef SPI_SHIM_DEBUG
	printf("in read and write, bits=%d\n",bits);
#endif
	if (ret < 0) {
	    printf("can't get bits per word");
            close (spi_fd);
            return IDL_FAILURE;
	}

	tr.bits_per_word = bits;

        if (bits <= 8) {
	  tr.len = sizeof(uint8_t);
	}
	else if (bits <= 16) {
	  tr.len = sizeof(uint16_t);
	}
	else if (bits <= SPI_DATA_SIZE_MAX ) {
	  tr.len = sizeof(uint32_t);
	}
	else {
            close (spi_fd);
	    return IDL_FAILURE;
	}
	
	*outdata = 0;

	ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
        close (spi_fd);
	if (ret < 1) {
		printf("can't send spi message");
		return IDL_FAILURE;
	}
	return IDL_SUCCESS;
}

idl_result_t idl_spi_write(uint32_t spi_port_num, uint32_t data)
{
	
        uint32_t outdata;
	idl_result_t ret;
	if ( spi_port_num  >=  NUM_SPI_PORTS){
		printf("idl_spi_set_config: spi_port_num = %u, NUM_SPI_PORTS = %u\n", spi_port_num, NUM_SPI_PORTS);
		return IDL_INVALID_PARAM;
	}

	if ( !spi_port_chip_select[spi_port_num].config_flag ) {
		return IDL_NOT_INITIALIZED;
	}

	ret =idl_spi_write_and_read(spi_port_num, data, &outdata);
	if ( ret != IDL_SUCCESS ) {
	  return IDL_FAILURE;
	}
	  
        ret = read_fifo.write(&read_fifo, outdata); 
        if (ret) {
	    return IDL_DEVICE_BUSY;
	 }

#ifdef SPI_SHIM_DEBUG
        {
	 int i;
	 for (i = 0; i < SPI_FIFO_DEPTH; i++)
	     printf("0x%x ", read_fifo.data[i]);
	     printf("\n");
	     printf("fifo index=%x\n",read_fifo.windex);
	}
#endif
	  return IDL_SUCCESS;
}


idl_result_t idl_spi_read(uint32_t spi_port_num, uint32_t *data)
{
	
        int ret;
	if ( data == NULL ) {
		return IDL_INVALID_PARAM;
	}

	if ( spi_port_num  >=  NUM_SPI_PORTS){
		printf("idl_spi_set_config: spi_port_num = %u, NUM_SPI_PORTS = %u\n", spi_port_num, NUM_SPI_PORTS);
		return IDL_INVALID_PARAM;
	}

	if ( !spi_port_chip_select[spi_port_num].config_flag ) {
		return IDL_NOT_INITIALIZED;
	}
        ret = read_fifo.read(&read_fifo, data); 
        if (ret){
	    return IDL_DEVICE_BUSY;
	 }

#ifdef SPI_SHIM_DEBUG	  
         printf(" index=%x, data=0x%x\n", read_fifo.rindex, *data);
	 for (ret = 0; ret < SPI_FIFO_DEPTH; ret++)
	     printf("0x%x ", read_fifo.data[ret]);
	     printf("\n");
#endif

         return IDL_SUCCESS;

}


idl_result_t idl_spi_release(uint32_t spi_port_num)
{
 
  if ( spi_port_num  >=  NUM_SPI_PORTS){
	printf("idl_spi_set_config: spi_port_num = %u, NUM_SPI_PORTS = %u\n", spi_port_num, NUM_SPI_PORTS);
	return IDL_INVALID_PARAM;
  }

 spi_port_chip_select[spi_port_num].config_flag = false; 

 return IDL_SUCCESS;
}

