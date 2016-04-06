/*

 This file is provided under a dual BSD/GPLv2 license.  When using or
 redistributing this file, you may do so under either license.

 GPL LICENSE SUMMARY

 Copyright(c) 2011-2012 Intel Corporation. All rights reserved.

 This program is free software; you can redistribute it and/or modify
 it under the terms of version 2 of the GNU General Public License as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 The full GNU General Public License is included in this distribution
 in the file called LICENSE.GPL.

 Contact Information:

 Intel Corporation
 2200 Mission College Blvd.
 Santa Clara, CA  97052

 BSD LICENSE

 Copyright(c) 2011-2012 Intel Corporation. All rights reserved.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
/**
@weakgroup presudo_code Pseudo Code for IDL

<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">Content</h2>	

		- <A class="el" HREF=#>5.	Pseudo Code for IDL</A>
			- <A class="el" HREF=#index_5_1>5.1	IDL I2C User Application Pseudo Code</A> 
			- <A class="el" HREF=#index_5_2>5.2	IDL GPIO User Application Pseudo Code</A> 
			- <A class="el" HREF=#index_5_3>5.3	IDL SPI User Application Pseudo Code</A> 
			- <A class="el" HREF=#index_5_4>5.4	IDL SmartCard User Application Pseudo Code</A> 
	
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:18pt">5.	Pseudo Code for IDL</h2>  
<p>
The pseudo code below will give some basic user code application pseudo code examples for IDL, if development is in kernel mode via kernel symbols exported from IDL drivers. 
</p>



\anchor index_5_1
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">5.1	IDL I2C User Application Pseudo Code</h2>	
<p>
The following pseudo code is for I2C:
</p>
\code
#include <stdio.h>
#include "idl.h"
#include "idl_i2c.h"
int main(int argc, char *const argv[])
{
  static uint32_t regsize=2;
  uint32_t byte_count;
  uint8_t regadd[2];
  uint8_t bus_cnt;
  uint8_t buf[16];
  static uint32_t regsize=2;
  unsigned int i,j,k;
  idl_result_t result;
uint32_t fRegData;
 
  fRegData = fopen("RegData.txt\0", "w+");
  if(fRegData == NULL) fRegData=stdout;

  result = idl_i2c_get_bus_count(&bus_cnt);
  if(IDL_SUCCESS != result) {
     fprintf(fRegData, "\r\n[FAIL]: file: %s on line %d\n", __FILE__, __LINE__);
     fprintf(fRegData, "        In function main()\n");
     fprintf(fRegData, "        idl_i2c_get_bus_count() did not return IDL_SUCCESS\n");
     fprintf(fRegData, "        but returned 0x%04X\n", result);
     return false; // an EXIT 
  }
  else 
  {
     fprintf(fRegData, "\r\n\r\nidl_i2c_get_bus_count() claims there are %d (0x%02X) i2c busses\n",
     bus_cnt, bus_cnt);
     result = idl_i2c_open();
     if(IDL_SUCCESS != result) {
       fprintf(fRegData, "\r\n\r\nidl_i2c_open didn't\n");
       return false;
     }

     result = idl_i2c_set_mode(bus, IDL_I2C_MODE_FAST);
     if(IDL_SUCCESS != result) {
       fprintf(fRegData, "\r\n\r\n idl_i2c_set_mode for bus %d didn't\n",bus);
       return flase;
     }
     else {
       fprintf(fRegData, "\r\n\r\nbus %d is set at 400kHz\n", bus);
     }

     byte_count=4;
     regadd[0] = 0x05;
     regadd[1] = 0x18;
     buf[0] = 0xff;
     buf[1] = 0x0f;
     buf[2] = 0xff;
     buf[3] = 0xff;
     result = idl_i2c_write_sub_addr_ex(bus,dev >> 1U,regadd, regsize, &buf[0],byte_count);
     if(IDL_SUCCESS != result) {
         fprintf(fRegData, "Fail bus=0x%02X dev=0x%04X  reg 0x%02x%02x not found\n",
         bus, dev, regadd[0], regadd[1]);
         return false;
     }
     else {
         fprintf(fRegData, "Wrote bus=0x%02X dev=0x%04X reg 0x%02x%02x\n",
         bus, dev, regadd[0], regadd[1]);
     }

     result = idl_i2c_close();
     if(IDL_SUCCESS != result) {
       fprintf(fRegData, "idl_i2c_close didn't\n");
       return(1);
     } //End IF the close failed
     else {
       fprintf(fRegData, "idl_i2c_close success\n\n");
     }
  } //End ELSE could get the bus count

   if(fRegData != stdout) fclose(fRegData);
   return true;
}
\endcode

\anchor index_5_2
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">5.2	IDL GPIO User Application Pseudo Code</h2>
<p>The following pseudo code is for GPIO:</p>
\code
#include <stdio.h>
#include "idl.h"
#include "idl_gpio.h"
#include "osal_thread.h"
int main(int argc, char *const argv[])
{
  idl_result_t status;
  uint32_t *gpio_base=NULL;
  if (IDL_NOT_INITIALIZED == idl_gpio_init(&gpio_base)) return flase;
  if(IDL_SUCCESS != idl_gpio_line_config(7, IDL_GPIO_OUTPUT)) {
    idl_gpio_release();
    return false;
  }
  //general one wave
 status = idl_gpio_set_line(7, 0);
  if(IDL_SUCCESS != status) return  false;
 os_sleep(1);
status = idl_gpio_set_line(7, 1);
  if(IDL_SUCCESS != status) return  false;
 os_sleep(1);
status = idl_gpio_set_line(7, 0);
  if(IDL_SUCCESS != status) return  false;
 os_sleep(1);

  idl_gpio_release();
  return true;
}
\endcode

\anchor index_5_3
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">5.3	IDL SPI User Application Pseudo Code</h2>
<p>The following pseudo code is for SPI:</p>
\code
#include <stdio.h>
#include ¡°idl.h¡±
#include ¡°idl_spi.h¡±
#include "osal_thread.h"
int main(int argc, char *const argv[])
{
  uint32_t status;
  spi_config_t spi_conf;
  uint32_t read_data, count=0;
uint32_t portnum = 0;
uint32_t cs = 0;
  os_sleep(1);
  if(  IDL_SUCCESS != idl_spi_init(portnum)){
    return  false;
  }
  if(cs>0)
    spi_conf.chip_select = 1;
  else
  spi_conf.chip_select = 0;
  spi_conf.clock_rate = 115200;
  spi_conf.data_size  = 16;
  spi_conf.frame_format = MOTOROLA_SPI;
  spi_conf.rx_threshold = 1;
  spi_conf.clock_polarity = INACTIVE_LOW;
  spi_conf.tx_threshold = 1;
  spi_conf.phase = PHASE_ONE_HALF;
  do{
    os_sleep(1);
    status = idl_spi_set_config(portnum, &spi_conf);
    count ++;
    if(count > 0xffff ){
      _OS_DEBUG("cannot config spi it=%d, 0x%X\n", count, status);
      return false;
    }    
  }while(IDL_SUCCESS != status);
  count=0;
  do{
    os_sleep(1);
    status = idl_spi_write(portnum, data);
    idl_spi_read(portnum, &read_data);
    count++;
    if(count > 0xffff ){
      _OS_DEBUG("cannot write to spi it=%d ret=0x%X\n", count,status);
      return  false;
    }
  }while(IDL_SUCCESS !=status);
  os_sleep(1);
  if(IDL_SUCCESS != idl_spi_release(portnum)){
    _OS_DEBUG("cannot release spi \n");
    return false;
  }
  return true;
}
\endcode

\anchor index_5_4
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">5.4	IDL SmartCard User Application Pseudo Code</h2>
<p>The following pseudo code is for SmartCard:<p>
\code
#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include "idl.h"
#include "idl_smartcard.h"

static void send_command(int fd);
static void read_data(int fd);
static void answer_to_reset(int fd, uint8_t protocol);
static uint8_t get_slot_number(int fd);

int m_fd[] = {-1, -1};
int m_protocol[] = {SC_T0, SC_T1};
int m_slot[] = {SC_SLOT1, SC_SLOT2};
uint8_t  buffer[100];
int main(void)
{
        uint32_t       slot;
        sc_slot_info_t info;
        sc_section_t   section;     
                    
        for (slot = SC_SLOT1; slot <= SC_SLOT2; slot++) {
        m_fd[slot] = sc_open(slot);
        if (m_fd[slot] == -1 )
        {
            printf("Can not open the slot %d\n", slot);
            return (0);
         }
         // check if the card is inserted or removed 
         sc_get_slot_info(m_fd[slot], &info);
         printf("Smartcard protocol is %d\n", info.protocol);
         if (info.status == SC_CARD_POWERED_UP) {
         // the card was inserted, powered up 
              printf("SmartCard insert slot %d  ....[PASS] \n", slot);
              answer_to_reset(m_fd[slot], m_protocol[slot]);
              send_command(m_fd[slot]);
         }
         else if (info.status == SC_CARD_INSERTED_NOT_POWERED_UP) {
               printf("SmartCard insert %d but no power ....[PASS] \n", slot);
         // the card was inserted but not powered up 
               answer_to_reset(m_fd[slot], m_protocol[slot]);
         //send command and read data back 
              send_command(m_fd[slot]);
          }
          else {
         // the card was removed 
              sc_card_removal(m_fd[slot]);
              printf("SmartCard on slot %d is removed ....[PASS] \n", slot);
         }
        sc_close(m_fd[slot]);
       }
       return(0);
}
static void send_command(int fd)
{
        int i = 0;
        uint8_t cmd[50];

        // start mutual authentication process 
        cmd[0] = 0x80;
        cmd[1] = 0x84;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x08;
        i = 0;
        while (i++ < 1) {
               int scwrite= sc_write(fd, cmd, 5);
                if (scwrite == -1)
                    printf("Can not write the smart card interface");
                else{
                // data is in the RX buffer, get it 
                sleep(1);
                printf("This is start mutual authentication process \n");
                read_data(fd);
                sleep(1);

                }
        }
       
        // select file 
        cmd[0] = 0x80;
        cmd[1] = 0xA4;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x02;
        cmd[5] = 0xff;
        cmd[6] = 0x04;

         i = 0;
        while (i++ < 1) {
                int sewrite = sc_write(fd, cmd, 7);
                if ( sewrite ==-1 )
                   printf("Can not write the smart card interface");
                else{
                sleep(1);
                //data is in the RX buffer, get it 
                read_data(fd);
                sleep(1);
                }
        }

        cmd[0] = 0x80;
        cmd[1] = 0xB2;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x06;
         i = 0;
        while (i++ < 1) {
                int sewrite = sc_write(fd, cmd, 5);
                if ( sewrite ==-1 )
                   printf("Can not write the smart card interface");
                else{
                sleep(1);
                // data is in the RX buffer, get it 
                printf("This is user management file\n");
                read_data(fd);
               }

              }              
      // select file user manager file 
        cmd[0] = 0x80;
        cmd[1] = 0xA4;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x02;
        cmd[5] = 0xf0;
        cmd[6] = 0x00;

         i = 0;
        while (i++ < 1) {
                int sewrite = sc_write(fd, cmd, 7);
                if ( sewrite ==-1 )
                   printf("Can not write the smart card interface");
                else{
                sleep(1);
                // data is in the RX buffer, get it 
                 printf("This is user file data response for writing \n");
                 read_data(fd);
                sleep(1);
                }
        }
        
        // write 20 character to user manager first record file 
        cmd[0] = 0x80;
        cmd[1] = 0xD2;
        cmd[2] = 0x00;
        cmd[3] = 0x00;
        cmd[4] = 0x20;
        cmd[5] = 0x00;
        cmd[6] = 0x00;        
        cmd[7] = 0x00;
        cmd[8] = 0x00;
        cmd[9] = 0x00;
        cmd[10] = 0x00;
        cmd[11] = 0x00;
        cmd[12] = 0x00;
        cmd[13] = 0x00;
        cmd[14] = 0x00;
        cmd[15] = 0x00;
        cmd[16] = 0x00;
        cmd[17] = 0x00;
        cmd[18] = 0x00;
        cmd[19] = 0x00;
        cmd[20] = 0x00;
        cmd[21] = 0x00;
        cmd[22] = 0x00;
        cmd[23] = 0x00;
        cmd[24] = 0x00;        
        cmd[25] = 0x00;
        cmd[26] = 0x00;
        cmd[27] = 0x00;
        cmd[28] = 0x00;
        cmd[29] = 0x00;
        cmd[30] = 0x00;
        cmd[31] = 0x00;
        cmd[32] = 0x00;
        cmd[33] = 0x00;
        cmd[34] = 0x00;
        cmd[35] = 0x00;
        cmd[36] = 0x00;
         i = 0;
        while (i++ < 1) {
                int sewrite = sc_write(fd, cmd, 37);
                if ( sewrite ==-1 )
                   printf("Can not write the smart card interface");
                else{
                sleep(1);
                // data is in the RX buffer, get it 
                 printf("This is user data write response \n");
                 read_data(fd);
                if (buffer [0] == 0xd2 & buffer [1] == 0x90 & buffer [2] == 0x00)
                    printf("Successfully write 20 character into user file record\n");
                else 
                    printf("Can not write 20 character into user file record\n");
                sleep(1);
                }
        }


         
} 

static void read_data(int fd)
{

        int      nbytes;
        int      i;

        nbytes = sc_read(fd, buffer, sizeof(buffer));
        printf("This is nbytes number %d \n", nbytes);
        if ( nbytes == -1 )
            printf("Can not read the smart card interface");
        else if ( nbytes >0)
        {
                for (i = 0; i < nbytes; i++) {
                        printf("This data read from RX buffer %02x\n", buffer[i]);
                }
                printf("\n");
        }
}


static void answer_to_reset(int fd, uint8_t protocol)
{
        sc_section_t section;     
        uint8_t      i;

        section.protocol = protocol;
        section.vcc      = SC_VCC_SEL_5;
		 section.reset_type = SC_RESET_TYPE_COLD;

        // start section and get ATR string 
        sc_start_section(fd, &section);

        printf("\n\n");
        if (section.atr_len > 0) {
               printf("This ATR string length is %d\n", section.atr_len);
                printf("ATR string from slot %d:\n", get_slot_number(fd));
                for (i = 0; i < section.atr_len; i++) {
                        printf("%02x ", section.atr[i]);
                }
                printf("\n\n");
        }

}

static uint8_t get_slot_number(int fd)
{
        if (fd == m_fd[0]) {
                return SC_SLOT1;
        }
        else {
                return SC_SLOT2;
        }
}
\endcode

*/
