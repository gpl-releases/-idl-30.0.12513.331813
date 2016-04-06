/*
#
# This file is provided under a dual BSD/LGPLv2.1 license.  When using
# or redistributing this file, you may do so under either license.
#
# LGPL LICENSE SUMMARY
#
# Copyright(c) 2007-2012. Intel Corporation. All rights reserved.
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

/*------------------------------------------------------------------------------
 * File Name: idl_i2c_shim.c
 *------------------------------------------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IDL_DRIVER
#include <stdint.h>
#include <stdbool.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>

#include <osal_event.h>
#include <osal_thread.h>

#include <idl.h>
#include <idl_i2c.h>

#define MAX_BUS_COUNT 4

#define I2C_RETRIES	0x0701	/* number of times a device address      */
				/* should be polled when not            */
                                /* acknowledging 			*/
#define I2C_TIMEOUT	0x0702	/* set timeout - call with int 		*/
#define I2C_SLAVE	0x0703	/* Change slave address			*/
				/* Attn.: Slave address is 7 or 10 bits */
#define MAX_I2C_MSG         3

#define I2C_RETRIES     0x0701
#define I2C_TIMEOUT     0x0702
#define I2C_RDWR        0x0707
#define I2C_SMBUS       0x0720
#define I2C_SET_MODE    0x0780


struct i2c_msg
 {
   unsigned short addr;
   unsigned short flags;
   #define I2C_M_TEN 0x0010
   #define I2C_M_RD 0x0001
   unsigned short len;
   unsigned char *buf;
 };
 
struct i2c_rdwr_ioctl_data {
   struct i2c_msg *msgs;   /* pointers to i2c_msgs */
   int nmsgs;              /* number of i2c_msgs   */
};

#define I2C_SUB_ADDR_ENABLE       1
#define I2C_WRITE_ENABLE      (1<<1)
#define I2C_READ_ENABLE       (1<<2)
#define I2C_M_WRITE 0

//#define I2C_SHIM_DEBUG      
static uint8_t i2c_bus_count = MAX_BUS_COUNT;
static idl_i2c_mode_t i2c_bus_mode[MAX_BUS_COUNT];


/*------------------------------------------------------------------------------
 * idl_i2c_get_bus_count
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_get_bus_count(uint8_t *p_bus_count)
{
   int i,fd;
   char devstr[20];
   i2c_bus_count = 0;
   for (i = 0; i < MAX_BUS_COUNT; i++) {
     sprintf(devstr,"/dev/i2c-%x",i);
     fd=open(devstr,O_RDWR);
     if (fd >= 0) {
       i2c_bus_count++;
       close(fd);
     }
   }
   *p_bus_count = i2c_bus_count;
   return IDL_SUCCESS;
}

/*------------------------------------------------------------------------------
 * idl_i2c_open
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_open()
{
   return IDL_SUCCESS;
}

/*------------------------------------------------------------------------------
 * idl_i2c_close
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_close()
{
   return IDL_SUCCESS;
}

/*------------------------------------------------------------------------------
 * idl_i2c_reset
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_reset(uint8_t bus_num)
{
  /* There is no API to reset i2c controller in user mode */
   return IDL_SUCCESS;
}

/*------------------------------------------------------------------------------
 * idl_i2c_set_mode
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_set_mode(uint8_t bus_num, idl_i2c_mode_t mode)
{
   i2c_bus_mode[bus_num] = mode;
   return IDL_SUCCESS;
}


/*------------------------------------------------------------------------------
 * idl_i2c_enable_polling
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_enable_polling(uint8_t bus_num, bool to_poll)
{
 /* idl_i2c_shim only support the default interrupt mode */
   return IDL_SUCCESS;
}
/*------------------------------------------------------------------------------
 * idl_i2c_write_read_ex
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_write_read_ex(uint8_t bus_num, uint16_t slave_addr,
   uint8_t *p_sub_addr, uint32_t sub_addr_byte_count,
   uint8_t *p_write_data_buffer, uint32_t write_byte_count,
   uint8_t *p_read_data_buffer, uint32_t read_byte_count)
{
  char devstr[20];
  int fd,ret,num=0;
  uint32_t opflags = 0;
  uint8_t *wbuf = NULL;
  struct i2c_rdwr_ioctl_data i2c_ioctl_data;
  struct i2c_msg msg[2] = {
                             { slave_addr, I2C_M_WRITE, 1, NULL},
                             { slave_addr, I2C_M_WRITE, 1, NULL},
                          };

  sprintf(devstr,"/dev/i2c-%x",bus_num);
  fd = open(devstr, O_RDWR);
  if (fd < 0){
    printf("open fail\n");
    return IDL_INVALID_PARAM;
  }

  ioctl(fd,I2C_SET_MODE,i2c_bus_mode[bus_num]);
  ioctl(fd,I2C_TIMEOUT,2);
  ioctl(fd,I2C_RETRIES,2);
#ifdef I2C_SHIM_DEBUG
  printf("%x,%x\n",(uint32_t)msg[0].buf,(uint32_t)msg[1].buf);
#endif
  if (sub_addr_byte_count > 0 && p_sub_addr != NULL){
      opflags |= I2C_SUB_ADDR_ENABLE;
      msg[0].len = sub_addr_byte_count;
      msg[0].flags = I2C_M_WRITE;
      msg[0].buf = p_sub_addr;
      num++;
#ifdef I2C_SHIM_DEBUG
      printf("sub: num=%d\n", num);
#endif

    }
  
  if (write_byte_count > 0 && p_write_data_buffer != NULL ){
     opflags |= I2C_WRITE_ENABLE;
     if (opflags & I2C_SUB_ADDR_ENABLE) {
          msg[0].len = sub_addr_byte_count + write_byte_count;
          msg[0].flags = I2C_M_WRITE;
	  wbuf = (uint8_t *)malloc(msg[0].len);
	  if (wbuf == NULL)
	  {
	    close(fd);
	    return IDL_NO_RESOURCES;
	  }
	  memcpy(wbuf,p_sub_addr, sub_addr_byte_count);
	  memcpy(wbuf+sub_addr_byte_count,p_write_data_buffer, write_byte_count);
#ifdef I2C_SHIM_DEBUG
          printf("mem: 0x%x,0x%x,0x%x\n",wbuf[0],wbuf[sub_addr_byte_count],wbuf[sub_addr_byte_count+1]);
	  printf("mem len =%d\n",msg[0].len);
#endif
          msg[0].buf = wbuf;

      } else {
          msg[0].flags = I2C_M_WRITE;
          msg[0].len = write_byte_count;
          msg[0].buf = p_write_data_buffer;
	  num++;
      }
#ifdef I2C_SHIM_DEBUG
      printf("msg[0].len=%x, flags=%x,buf=%x\n",msg[0].len,msg[0].flags,(uint32_t)msg[0].buf);
      printf("buf0=%x,buf1=%x, buf2=%x\n", msg[0].buf[0],msg[0].buf[1],msg[0].buf[2]);
      printf("write: num=%d\n", num);
#endif
    }

    /* read opeartion */
   if (read_byte_count > 0 && p_read_data_buffer != NULL) {
        opflags |= I2C_READ_ENABLE;
        if (opflags & (I2C_SUB_ADDR_ENABLE | I2C_WRITE_ENABLE)) {
             msg[1].len = read_byte_count;
             msg[1].flags = I2C_M_RD;
             msg[1].buf = p_read_data_buffer;
        } else {
            msg[0].len = read_byte_count;
            msg[0].flags = I2C_M_RD;
            msg[0].buf = p_read_data_buffer;
        }
       num++;

    }
#ifdef I2C_SHIM_DEBUG
   printf("last: num=%d\n", num);
#endif
   i2c_ioctl_data.msgs = msg;
   i2c_ioctl_data.nmsgs = num;
 
#ifdef I2C_SHIM_DEBUG
   printf("%x,%x\n",(uint32_t)msg[0].buf,(uint32_t)msg[1].buf);
#endif
   ret = ioctl(fd, I2C_RDWR, (uint32_t)&i2c_ioctl_data);
#ifdef I2C_SHIM_DEBUG
   printf("ret=0x%x\n",ret);
#endif
   close(fd);
   free(wbuf);
   if (ret < 0)
     return IDL_FAILURE;
#ifdef I2C_SHIM_DEBUG
   printf("write_read_ex end\n");
#endif

   return IDL_SUCCESS;

}
/*------------------------------------------------------------------------------
 * idl_i2c_read_sub_addr_ex
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_read_sub_addr_ex(uint8_t bus_num, uint16_t slave_addr,
   uint8_t *p_sub_addr, uint32_t sub_addr_byte_count,
   uint8_t *p_data_buffer, uint32_t byte_count)
{
   return idl_i2c_write_read_ex(bus_num, slave_addr, p_sub_addr,
      sub_addr_byte_count, NULL, 0, p_data_buffer, byte_count);
}

/*------------------------------------------------------------------------------
 * idl_i2c_write_sub_addr_ex
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_write_sub_addr_ex(uint8_t bus_num, uint16_t slave_addr,
   uint8_t *p_sub_addr, uint32_t sub_addr_byte_count, uint8_t *p_data_buffer,
   uint32_t byte_count)
{
   return idl_i2c_write_read_ex(bus_num, slave_addr, p_sub_addr,
      sub_addr_byte_count, p_data_buffer, byte_count, NULL, 0);
}

/*------------------------------------------------------------------------------
 * idl_i2c_read_sub_addr
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_read_sub_addr(uint8_t bus_num, uint16_t slave_addr, uint8_t sub_addr,
   uint8_t *p_data_buffer, uint32_t byte_count)
{
 return idl_i2c_write_read_ex(bus_num, slave_addr, &sub_addr, 1, NULL, 0,
      p_data_buffer, byte_count);
}

/*------------------------------------------------------------------------------
 * idl_i2c_write_sub_addr
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_write_sub_addr(uint8_t bus_num, uint16_t slave_addr, uint8_t sub_addr,
   uint8_t *p_data_buffer, uint32_t byte_count)
{

 return idl_i2c_write_read_ex(bus_num, slave_addr, &sub_addr, 1, p_data_buffer, byte_count,
      NULL, 0);
}


#ifdef __cplusplus
}
#endif
