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
 * File Name: idl_i2c_drv.c
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
#include <linux/i2c.h>

#include <idl_i2c.h>

/* Driver identification */
MODULE_AUTHOR("Intel Corporation, (C) 2011 - All Rights Reserved");
MODULE_DESCRIPTION("Wrapper to map Intel 'IDL' I2C API to Linux I2C API");
MODULE_SUPPORTED_DEVICE("Intel CE Media Processors");
MODULE_LICENSE("Dual BSD/GPL"); /* Inform kernel that driver is not GPL. */

#define MAX_BUS_COUNT   4
#define _I2C_SUB_ADDR_ENABLE       1
#define _I2C_WRITE_ENABLE      (1<<1)
#define _I2C_READ_ENABLE       (1<<2)
#define I2C_M_WRITE 0
//#define I2C_SHIM_DEBUG


/* Unique name for driver */
#define I2C_DEV_NAME    "idl_i2c_shim"

#ifndef MOD_NAME
#define MOD_NAME "idl_i2c_shim.ko"
#endif

static int i2c_bus_count = MAX_BUS_COUNT;
static idl_i2c_mode_t i2c_bus_mode[MAX_BUS_COUNT];

char *version_string = "#@#" MOD_NAME " " VER;
int i2c_init(void);

void i2c_cleanup(void);

/* Specify driver entry points for kernel */
/* Macros are defined in linux/module.h */
module_init(i2c_init);
module_exit(i2c_cleanup);

/*------------------------------------------------------------------------------
 * Export symbols for other modules
 *------------------------------------------------------------------------------
 */

EXPORT_SYMBOL(idl_i2c_open);
EXPORT_SYMBOL(idl_i2c_close);
EXPORT_SYMBOL(idl_i2c_reset);
EXPORT_SYMBOL(idl_i2c_get_bus_count);
EXPORT_SYMBOL(idl_i2c_set_mode);
EXPORT_SYMBOL(idl_i2c_enable_polling);
EXPORT_SYMBOL(idl_i2c_read_sub_addr_ex);
EXPORT_SYMBOL(idl_i2c_write_sub_addr_ex);
EXPORT_SYMBOL(idl_i2c_read_sub_addr);
EXPORT_SYMBOL(idl_i2c_write_sub_addr);

int
i2c_init(void) {
   return 0;
}

void
i2c_cleanup(void) {
}


idl_result_t
idl_i2c_open(void) {
   return IDL_SUCCESS;
}

idl_result_t
idl_i2c_close(void) {
   return IDL_SUCCESS;
}

idl_result_t
idl_i2c_reset(uint8_t bus_num) {
   return IDL_SUCCESS;
}

idl_result_t
idl_i2c_set_mode(uint8_t bus_num, idl_i2c_mode_t mode) {
   i2c_bus_mode[bus_num] = mode;
   return IDL_SUCCESS;
}

idl_result_t
idl_i2c_enable_polling(uint8_t bus_num, bool to_poll) {
   return IDL_SUCCESS;
}

/*------------------------------------------------------------------------------
 * idl_i2c_get_bus_count
 *------------------------------------------------------------------------------
 */

idl_result_t
idl_i2c_get_bus_count(uint8_t *p_bus_count)
{
   struct i2c_adapter * adapter;
   int i;
   i2c_bus_count = 0;

   for (i = 0; i < MAX_BUS_COUNT; i++) {
     adapter = i2c_get_adapter(i);
     if (adapter != NULL) {
         i2c_bus_count++;
         i2c_put_adapter(adapter);
         adapter = NULL;
     }
   }
   *p_bus_count = i2c_bus_count;
   return IDL_SUCCESS;
}

 /*------------------------------------------------------------------------------
  * idl_i2c_read_sub_addr
  *------------------------------------------------------------------------------
  */
 
idl_result_t
idl_i2c_write_read_ex(uint8_t bus_num, uint16_t slave_addr, 
    uint8_t *p_sub_addr, uint32_t sub_addr_byte_count,
    uint8_t *p_write_data_buffer, uint32_t write_byte_count,
    uint8_t *p_read_data_buffer, uint32_t read_byte_count)
{
  /*
   * To implement this function, we need two I2C msgs.
   */
	
  struct i2c_adapter * adapter = NULL;
  uint32_t opflags = 0;
  int num = 0;
  int status = 0;
  uint8_t *wbuf = NULL;
  struct i2c_msg msg[2] = {
                             { slave_addr, I2C_M_WRITE, 1, NULL},
                             { slave_addr, I2C_M_WRITE, 1, NULL},
			  };
  adapter = i2c_get_adapter(bus_num);
  if (adapter == NULL)
    return IDL_INVALID_PARAM;
  adapter->mode = i2c_bus_mode[bus_num];
  
#ifdef I2C_SHIM_DEBUG
  printk("bus =%x, slave addr=0x%x, sub addr=0x%x, write count=%d, ready count=%d\n",
           bus_num, slave_addr, (sub_addr_byte_count>0?p_sub_addr[0]:0), write_byte_count, read_byte_count);
#endif
  if (sub_addr_byte_count > 0 && p_sub_addr != NULL){
      opflags |= _I2C_SUB_ADDR_ENABLE;
      msg[0].len = sub_addr_byte_count;
      msg[0].flags = I2C_M_WRITE;
      msg[0].buf = p_sub_addr;
      num++;
#ifdef I2C_SHIM_DEBUG
      printk("sub: num=%d\n", num);
#endif
    }
   if (write_byte_count > 0 && p_write_data_buffer != NULL ){
     opflags |= _I2C_WRITE_ENABLE;
     if (opflags & _I2C_SUB_ADDR_ENABLE) {
          msg[0].len = sub_addr_byte_count + write_byte_count;
          msg[0].flags = I2C_M_WRITE;
	  wbuf = (uint8_t *)kmalloc(sub_addr_byte_count + write_byte_count, GFP_KERNEL);
	  if (wbuf == NULL) {
            i2c_put_adapter(adapter);
	    return IDL_NO_RESOURCES;
	  }
	  memcpy(wbuf,p_sub_addr, sub_addr_byte_count);
	  memcpy(wbuf+sub_addr_byte_count,p_write_data_buffer, write_byte_count);
#ifdef I2C_SHIM_DEBUG
	  printk("mem: 0x%x,0x%x,0x%x\n",wbuf[0],wbuf[sub_addr_byte_count],wbuf[sub_addr_byte_count+1]);
	  printk("mem len =%d\n",msg[0].len);
#endif
          msg[0].buf = wbuf;
      } else {
          msg[0].flags = I2C_M_WRITE;
          msg[0].len = write_byte_count;
          msg[0].buf = p_write_data_buffer;
          num++;
      }
#ifdef I2C_SHIM_DEBUG
      printk("write: num=%d\n", num);
#endif
    }

    /* read opeartion */
   if (read_byte_count > 0 && p_read_data_buffer != NULL) {
        opflags|= _I2C_READ_ENABLE;
        if (opflags & (_I2C_SUB_ADDR_ENABLE | _I2C_WRITE_ENABLE)) {
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
   printk("last: num=%d\n", num);
#endif
   status = i2c_transfer(adapter, msg, num);
   i2c_put_adapter(adapter);
   kfree(wbuf);
   if (status < 0)
     return IDL_FAILURE;

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


