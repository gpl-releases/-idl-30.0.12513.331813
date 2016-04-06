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
@weakgroup usage_model IDL API Usage Model
\anchor index_4	
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">4	IDL API Usage Model</h2>
<p>
Each IDL Component (I2C, SPI, GPIO, SmartCard) has a similar usage model, below is one overview picture to show the steps:
</p>
\image html api_usage_model.gif

<p>
The usage model should be the same whether in the user application or in kernel mode (via kernel symbols).
</p>

\anchor index_9_1
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">4.1	IDL I2C API Usage Model</h2>

<p>
After the IDL I2C driver is inserted into kernel, the user application or other kernel component should perform the following to operate the I2C devices on the Intel CE Media Processor:
</p>
- Initialize/Open device
- Config Device
- Operate On Device
- De-initialize/Close Device

\image html note.gif "Note"
<BR/>
1. The Linux kernel IDL I2C driver is not applicable for PLL and PCI express clock devices.
<BR/>
2. The function prototypes of the I2C IDL user lib functions and kernel symbols are the same, so the descriptions below work on both, user application and kernel mode.

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.1.1	Initialize/Open device</h3>	
<p>
Get the I2C bus number count by <b>idl_i2c_get_bus_count()</b>; a code example follows:
</p>

\code
result = idl_i2c_get_bus_count(&bus_cnt);
if(IDL_SUCCESS != result) {
		// do error recover
}
\endcode
After the function is returned, bus_cnt will contain the current system I2C bus number. This step is not necessary.
<BR/>
Open the I2C device with idl_i2c_open(); a code example follows:
\code
result = idl_i2c_open();
if(IDL_SUCCESS != result) {
		// do error recover
}
\endcode


<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.1.2	Config Device</h3>
<p>
Set the transfer mode with <b>idl_i2c_set_mode()</b>;standard mode or fast mode can be set, the mode parameter is defined as <b>idl_i2c_mode_t structure</b>; a code example follows:
</p>

\code    
result = idl_i2c_set_mode(bus, IDL_I2C_MODE_FAST); //set to fast mode
if(IDL_SUCCESS != result) {
...// do error recover
}
\endcode


<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.1.3	Operate On Device</h3>
<p>
Use <b>idl_i2c_read_sub_addr_ex</b> and <b>idl_i2c_write_sub_addr_ex</b> to do read/write operations. The example below shows one way to write 4 bytes (buf[4]) to I2C bus 1, slave address 0x42, register offset 0x0518:
</p>
\code
uint8_t bus = 1;
uint8_t dev = 0x42;
uint8_t regsize=2;
uint8_t byte_count=4;
uint8_t regadd[2] = {0x05, 0x18};
uint8_t buf[4];
buf[0] = 0xff;
buf[1] = 0x0f;
buf[2] = 0xff;
buf[3] = 0xff;
result = idl_i2c_write_sub_addr_ex(bus,dev >> 1U,regadd, regsize, &buf[0],byte_count);
if(IDL_SUCCESS != result) {
...// do error recover
}
\endcode

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.1.4	De-initialize/Close Device</h3>
To close the device, call <b>idl_i2c_close()</b>,below:
	
	
\code	
result = idl_i2c_close();
if(IDL_SUCCESS != result) {
...// do error recover
}

\endcode
\anchor index_4_2
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">4.2	IDL GPIO API Usage Model</h2>
<p>
After the IDL GPIO driver has been inserted into kernel, the user application can do the following things to operate the GPIO device on the Intel CE Media Processors.
</p>
\image html note.gif "Note"
<BR/>
The function prototypes of GPIO IDL user lib functions and kernel symbols are the same, so the descriptions below work on both, user application and kernel mode.
<BR/>
Each GPIO pin in the Intel? CE Media Processors is assigned a label number in the IDL GPIO Driver. Applications should follow this rule to be coincident with the Driver. These GPIOs are listed below:

<p>
The General Purpose I/O interface provides additional flexibility to system designers. 
</p>

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.2.1	Introduction to GPIOs of Intel CE Media Processors</h3>
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">4.2.1.1	GPIOs of Intel Media Processor CE 3100 and the Intel Atom  processor CE4100</h4>
<p>
There are actually three discrete sets of GPIOs:
<BR/>
GPIOSUS[3:0].  These GPIOs are in the Suspend well and are implemented in the Legacy.  These GPIOs are D31:F0. 
<BR/>
GPIO[9:0].  These GPIOs are in the core well and are implemented in the Legacy cluster.  These GPIOs are also D31:F0. 
<BR/>
GPIOAUX[11:0].  These GPIOs are also in the core well and are implemented in the PUB GPIO unit.  These GPIOs are described in this chapter.
<BR/>
GPIOSUS[3:0] and GPIO[2:0] are dedicated GPIO pins and are not multiplexed with any other function.  The remaining GPIO pins, GPIO[9:3] and GPIOAUX[11:0], are multiplexed with various functions on the Intel(R) Media Processor CE 3100 .  The pin mux is controlled by the GPIO_MUXCNTL register in this unit; even the Legacy GPIO mux controls are handled here.  All pins default to GPIO upon power up.
<BR/>
PUB GPIO is assigned Device 11, Function 1.  The GPIO control registers may be accessed using <b>GPIO_AUX_MBAR</b>. 
</p>

<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">4.2.1.2	GPIOs of Intel Atom processor CE4200</h4>
<p>
There are actually two discrete sets of GPIOs:
<BR/>
1)	Legacy GPIOS[7:0].  These GPIOs are in the core well and are implemented in the Legacy cluster.  These GPIOs are also D31:F0. 
<BR/>
2)	GPIOAUX[77:0].  These GPIOs are also in the core well and are implemented in the PUB GPIO unit.  These GPIOs are described in this chapter.
<BR/>
Legacy GPIOS[7:0] and GPIOAUX[77:1] are dedicated GPIO pins and are not multiplexed with any other function.  GPIOAUX[0] is multiplexed with other function.  The pin mux is controlled by the GPIO_MUXCNTL register in this unit.  All pins default to GPIO upon power up.
<BR/>
DST_ICAM function is multiplexed with smart card 0 pins. NAND_CE_N and NAND_RE_N are multiplexed with smart card 1 pins. They are controlled by the GPIO_MUXCNTL register too, but they are not actually GPIO pins.
<BR/>
PUB GPIO is assigned Device 11, Function 1.  The GPIO control registers may be accessed using <b>GPIO_AUX_MBAR</b>.
<BR/>
GPIOAUX[4:0] are interrupt configurable. They can be configured as active high level or active low level. GPIOAUX[5:77] is only active high level.

<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">4.2.1.3	GPIOs of the Intel CE Media Processor codenamed CE5300</h4>
<p>
There are actually two discrete sets of GPIOs:
<BR/>
1)	Legacy GPIOS[7:0].  These GPIOs are in the core well and are implemented in the Legacy cluster.  These GPIOs are also D31:F0. 
2)	GPIOAUX[127:0].  These GPIOs are also in the core well and are implemented in the PUB GPIO unit.  These GPIOs are described in this chapter.
<BR/>
Legacy GPIOS[7:0] is dedicated GPIO pins and are not multiplexed with any other functions. Some of GPIO pins in GPIOAUX[127: 0] are multiplexed with other functions. But only GPIO pins in below table can be controlled to mux or not by the GPIO_MUXCNTL register in this unit. Other pins in GPIOAUX[127:0] default to GPIO upon power up and can¡¯t be controlled to mux function by GPIO driver.
	<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" ><td width="15%">GPIO pins</td><td width="15%">Mux function</td><td width="70%">Mux function API(val=0, GPIO; val=1, other functions)</td></tr>
<tr><td>64</td><td>UART1</td><td>idl_gpio_set_alt_function(64,val)</td></tr>
<tr><td>66</td><td>UART2</td><td>idl_gpio_set_alt_function(66,val)</td></tr>
<tr><td>33, 34, 99, 100, 101, 102	</td><td>Smart card0	</td><td>idl_gpio_set_alt_function(gpio_num,val)<BR/>gpio_num can be any value in 33, 34, 99, 100, 101, 102</td></tr>
<tr><td>58, 59, 60, 61, 62, 63</td><td>Smart card1</td><td>idl_gpio_set_alt_function(gpio_num,val)<BR/>gpio_num can be any value in 58, 59, 60, 61, 62, 63</td></tr>
<tr><td>95</td><td>GBE_LINK</td><td>idl_gpio_set_alt_function(95,val)</td></tr>
<tr><td>35, 36, 37, 38, 77, 78, 79, 80</td><td>PWM</td><td>If want to set gpio_num supplies the trigger inputidl_gpio_set_alt_function(gpio_num,1)
<BR/><BR/>
If want to set video_sync_1 as trigger input
idl_gpio_set_alt_function(gpio_num,0)
gpio_num should be 35, 36, 79, 80.
<BR/><BR/>
If want to set video_sync_0 as trigger input
idl_gpio_set_alt_function(gpio_num,0)
gpio_num should be 37,38,77,78.
</td></tr>
</table>
	PUB GPIO is assigned Device 11, Function 1.  The GPIO control registers may be accessed using <b>GPIO_AUX_MBAR</b>.
</p>


<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.2.2	Initialize/Open Device</h3>
Open the GPIO device using idl_gpio_init(), example below:
if (IDL_NOT_INITIALIZED == idl_gpio_init(&gpio_base)) {
...// do error recover
}
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.2.3	Config Device</h3>
<p>
Set the GPIO line to input/output mode (below). Here, the config gpio pin 7 is set to output mode:
</p>
\code
if(IDL_SUCCESS != idl_gpio_line_config(7, IDL_GPIO_OUTPUT)) {
...// do error recover
}
\endcode
To use the GPIO alternate function, use <b>idl_gpio_set_alt_function()</b>. Here, gpio pin 7 is set to its alternate function. Since it only has one alternate function, the second parameter, fn_num, should be 0:
\code
idl_gpio_set_alt_function(7, 0);
\endcode


<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.2.4	Operate On Device</h3>
<p>
Set the GPIO output level using <b>idl_gpio_set_line()</b>. Here, gpio pin 7 is set to high level:
</p>
\code
idl_gpio_set_line(7, 1)
\endcode
Get the GPIO input level using idl_gpio_get_line(). Here, gpio pin 7 will get the input level:
\code
uint32_t level;     
idl_gpio_get_line(7, &level)
\endcode
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.2.5	De-initialize/Close Device</h3>
To close the device, call <b>idl_gpio_release()</b>:
\code
idl_gpio_release();
\endcode
\anchor index_4_3
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">4.3	IDL SPI API Usage Model</h2>
<p>
After the IDL SPI driver has been inserted into kernel, the user application can do the following things to operate the SPI device on the Intel CE Media Processor.
</p>
\image html note.gif "Note"
The function prototypes of SPI IDL user lib functions and kernel symbols are the same, so the description below works on both, user application and kernel mode.
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.3.1	Initialize/Open device</h3>
Open the SPI device using idl_spi_init(). The input parameter portnum specifies which SPI port to open. There is only one port in the Intel? CE Media Processors, so just put 0 here:
\code    
if(IDL_SUCCESS != idl_spi_init(portnum)){
...// do error recover
}
\endcode
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.3.2	Config Device</h3>
<p>
Set the SPI work mode using idl_spi_set_config().
- chip_select is used to select which device on the SPI bus should be operated.
- clock_rate sets the SPI bus clock rate.
- data_size sets the bit width of data transferred in SPI bus, it can be from 4 or 16
- fram_format sets the type of frame format. Currently, only MOTOROLA_SPI is supported.
- rx_threshold and tx_threshold sets the actual level in FIFO at which the SPI controller will notify 
</p>
that there is data in the SPI FIFO. The table below describes the relation between threshold value and the actual level:

<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
<td width="33%">Rx_threshold</td><td>Tx_threshold</td><td>Actual Level</td></tr>
<tr><td>1</td><td>1</td><td>1</td></tr>
<tr><td>2</td><td>2</td><td>2</td></tr>
<tr><td>3</td><td>3</td><td>3</td></tr>
<tr><td>0</td><td>0</td><td>4</td></tr>
</tr> 
</table>	
<p>
From the table, setting <b>rx_threshold</b> to one means actual level 1;
From the table, setting <b>tx_threshold</b> to one means actual level 1;
- clock_polarity sets the style of SPI clock, it can be "INACTIVE_LOW" or "INACTIVE_HIGH"
- phase sets the type of phase on the SPI bus, it can be "PHASE_ONE_HALF" or "PHASE_HALF_ONE"
</p>
<p>
\code
uint32_t status;
spi_config_t spi_conf;
spi_conf.chip_select = 0;
spi_conf.clock_rate = 115200;
spi_conf.data_size  = 16;
spi_conf.frame_format = MOTOROLA_SPI;
spi_conf.rx_threshold = 1;
spi_conf.clock_polarity = INACTIVE_LOW;
spi_conf.tx_threshold = 1;
spi_conf.phase = PHASE_ONE_HALF;
status = idl_spi_set_config(portnum, &spi_conf);
if(  IDL_SUCCESS != status){
...// do error recover
}
\endcode
</p>

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.3.3	Operate On Device</h3>
Perform read/write operations on the SPI bus using <b>idl_spi_write</b> and <b>idl_spi_read</b>. Below is an example:
\code
//write data to spi port	
   status = idl_spi_write(portnum, data);
//read data from spi port
   idl_spi_read(portnum, &read_data);
\endcode
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.3.4	De-initialize/Close Device</h3>
To close the device, call <b>idl_spi_release()</b>:
\code  
if(IDL_SUCCESS != idl_spi_release(portnum)){
...// do error recover
}
\endcode
\anchor index_4_4
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">4.4	IDL SmartCard API Usage Model</h2>
<p>
After the IDL SmartCard driver has been inserted into kernel, the user application can do the following things to operate the SmartCard device on the Intel? CE Media Processors:
- Initialize/Open device
- Config Device
-	Operate On Device
-	De-initialize/Close Device
</p>

There is no SmartCard kernel symbol exported out. The examples below will only focus on user applications.
4.4.1	Initialize/Open device
Open the SmartCard device using sc_open(). The input parameter slot specifies which SmartCard slot to open. There are two SmartCard slots in the Intel? CE Media Processor. This function will return the file descriptor(fd):
\code
uint32_t fd;
fd = sc_open(portnum);
if(  -1 != fd){
...// do error recover
}
\endcode

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.4.2	Config Device</h3>
<p>
Check SmartCard status using sc_get_slot_info(). Slot indicates which slot to check. Info is a structure to contain the return data.
sc_slot_info_t info;
sc_get_slot_info(slot, &info);
The returned status can be:
	- SC_CARD_POWERED_UP
	- SC_CARD_INSERTED_NOT_POWERED_UP
	- SC_CARD_NO_CARD_INSERTED
</p>	
<BR/>
<p>
If the card is inserted, whether it is powered up or not, do the start section to get the ATR string from card. If the card is not inserted, do card removal to complete smooth shut down at the interface.
</p>
<BR/>
Do start section on the SmartCard slot to get the ATR string from the card using <b>sc_start_section</b>.
<BR/>
fd is the file descriptor which is returned from the <b>sc_open()</b> function.
<b>section.protocol</b> sets the protocol for the card to use, it can be <b>SC_T0</b> or <b>SC_T1</b>.
		The <b>section.vcc</b> sets the voltage on the slot, it can be
			-SC_VCC_SEL_0		//no power
 			-SC_VCC_SEL_3_3	//3.3V
			-SC_VCC_SEL_5		//5V
		<b>section.reset_type</b> indicates what reset type to use. It can be SC_RESET_TYPE_WARM or SC_RESET_TYPE_COLD. We must use SC_RESET_TYPE_COLD in the first time to read out the ATR info.
\code
sc_section_t section;     
section.protocol = SC_T0;
section.vcc      = SC_VCC_SEL_5;
section.reset_type = SC_RESET_TYPE_COLD;
// start section and get ATR string 
sc_start_section(fd, &section); 
\endcode
If the function executed successfully, the <b>section.atr_len</b> will return the ATR string's length and the <b>section.atr</b> is the point to refer the data buffer, which contains the ATR string.
Do card removal using <b>sc_card_removal()</b>. The input parameter <b>fd</b> is returned from <b>sc_open()</b>.
\code
sc_card_removal(fd);  
\endcode

<b>Notice</b>: Application should config smartcard controller correctly according to smartcard ATR string. For T1 protocol smartcard sc_set_cwtr(),sc_set_bwtr(),sc_set_etu() should be called to config smartcard controller,and sc_set_bwtr() should be called after sc_set_cwtr(). For T0 protocol smartcard sc_set_cwtr(),sc_set_etu() function should be called. If smartcard controller is not correctly configured, read and write operation will fail.

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.4.3	Operate On Device</h3>
Perform SmartCard read/write operations using <b>sc_read()</b> or <b>sc_write()</b>. Example below:
\code
uint8_t cmd[5];
uint8_t result[5];
cmd[0] = 0x80;
cmd[1] = 0x84;
cmd[2] = 0x00;
cmd[3] = 0x00;
cmd[4] = 0x08;
int status= sc_write(fd, cmd, 5);
if (status == -1)
	...//error recovery
else{
    // data is in the RX buffer, get it 
    sleep(1);
    sc_read(fd, result, 5);
}
\endcode
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">4.4.4	De-initialize/Close Device</h3>
To close the device, call <b>sc_close()</b>. The input parameter fd is derived from <b>sc_open()</b>:
sc_close(fd);

*/
