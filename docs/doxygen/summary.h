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
@weakgroup IDL_summary IDL Summary
\anchor Definitions	
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">1.1 Definitions</h2>	
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" ><td width="50%">Term</td><td>Definition</td></tr>
<tr><td>API</td><td>Application Programming Interface</td></tr>
<tr><td>NB</td><td>Northbridge</td></tr>
<tr><td>IDL</td><td>Internal Driver Layer</td></tr>
</tr> 
</table>
\anchor index_1_2
<p>The processor IDL driver is used to drive I2C, SPI Master,GPIO and SmartCard hardware on the platform. The IDL API is built on top of the driver to provide access to the features supported by the hardware.< /p>
\image html note.gif "Note"
<BR />
In the following chapters, the acronym of SPI is referred to the short name of SPI Master if there is no special explanation. 
<p>\image html idl_api.gif "IDL"</p>
<BR />
\anchor idl_package_contents
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:18pt">2.	IDL Package Contents</h2>	
The following tables list the IDL package contents:
\anchor table_2_1
@section Table_2_1  Table 2-1 Drivers
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr  style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" ><td width="50%"><b>IDL drivers</b></td><td><b>names</b></td></tr>
<tr><td>GPIO kernel driver</td><td>idl_gpio_shim.ko,  idl_gpio_legacy.ko</td></tr>
<tr><td>I2C kernel driver</td><td>idl_i2c.ko</td></tr>
<tr><td>SPI kernel driver</td><td>idl_spi.ko</td></tr>
<tr><td>SmartCard kernel</td><td>driver	idl_smartcard.ko</td>
</tr> 
</table>
<br/>
\anchor table_2_2
@section Table_2_2 Table 2-2 User Libs
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr  style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" ><td width="50%">IDL User Libs</td><td>names</td></tr>
<tr><td>GPIO user lib</td><td>libidl_gpio_shim.so, idl_gpio_legacy.so</td></tr>
<tr><td>I2C user lib</td><td>libidl_i2c.a</td></tr>
<tr><td>SPI user lib</td><td>libidl_spi.a</td></tr>
<tr><td>SmartCard user lib</td><td>libidl_smartcard.a</td></tr>
<tr><td>IDL error user lib</td><td>libidlerror.a</td>
</tr> 
</table>
	<br/>

\anchor table_2_3
@section Table_2_3 Table 2-3 User Head Files
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" ><td width="50%">IDL User Head Files</td><td>names</td></tr>
<tr><td>GPIO user head file</td><td>idl_gpio.h</td></tr>
<tr><td>I2C user head file</td><td>idl_i2c.h</td></tr>
<tr><td>SPI user head file</td><td>idl_spi.h</td></tr>
<tr><td>SmartCard user head file</td><td>idl_smartcard.h</td></tr>
<tr><td>IDL global user head file</td><td>idl.h</td>
</tr> 
</table>

\anchor idl_api_overview	
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:18pt">3. IDL API Overview</h2>	
In this section, the basic data types, data structures, and IDL API taxonomy are described. The IDL API list is also included.
\anchor index_3_1	
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">3.1 Scalar Data Types</h2>	
The traditional <b>(unsigned) char, (unsigned) short, (unsigned) int</b> are defined by uint*_t types which are classified by bit width. For example, <b>uint_32</b> is used to represent <b>unsigned int</b>. The following table describes all data types defined by the IDL API.
\anchor table_3_1
@section Table_3_1  Table 3-1 Drivers
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr  style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" ><td width="50%"><b>C/C++ data type</b></td><td><b>IDL Data Types</b></td></tr>
<tr><td>unsigned char</td><td>Uint8_t</td></tr>
<tr><td>unsigned short</td><td>Uint16_t</td></tr>
<tr><td>unsigned int</td><td>Uint32_t</td></tr>
</tr> 
</table>
\anchor index_3_2
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">3.2	Compound Data Types</h2>	
Variables of these compound data types are used as inputs to and/or outputs from IDL APIs. In this section, all compound types are described.
\anchor index_3_2_1
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.1	Common Compound Data Types</h3>	
Below are common compound data types used for all IDL drivers(I2C, SPI, GPIO, SmartCard).
<p>Defined in include/idl.h</p>
\code
enum idl_result_t
\endcode
\anchor index_3_2_1_1
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.1.1	IDL I2C Special Compound Data Types</h3>	
\anchor index_3_2_2_1
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.2.1	I2C transfer mode parameter - idl_i2c_mode_t</h4>	

<p>Defined in include/idl_i2c.h</p>
\code
enum idl_i2c_mode_t
\endcode
\anchor index_3_2_2
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.2	IDL I2C Special Compound Data Types</h3>	
\anchor index_3_2_2_2
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.2.2	Async handle structure - idl_i2c_read_write_async_handle_t</h4>	
<p>Defined in include/idl_i2c.h</p>

\code
enum idl_i2c_read_write_async_handle_t
\endcode
\anchor index_3_2_3
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.3	IDL GPIO Special Compound Data Types</h3>	
\anchor index_3_2_3_1
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.3.1	GPIO Interrupt type - idl_gpio_interrupt_type_t</h4>
<p>This is used to configure the GPIO Interrupt type.</p>
<p>Defined in include/idl_gpio.h</p>	
\code
enum idl_gpio_interrupt_type_t
\endcode
\anchor index_3_2_3_2
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.3.2	GPIO gpio_num Definition in IDL</h4>
<p> \ref idl_gpio_num_definition "Please refer to chapter IDL GPIO Functions." </p>

\anchor index_3_2_3_3
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.3.3	GPIO pin mux control in  IDL</h4>
<p> \ref idl_gpio_pin_mux_control "Please refer to chapter IDL GPIO pin mux control." </p>

\anchor index_3_2_4
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.4	IDL SmartCard Special Compound Data Types</h3>	
\anchor index_3_2_4_1
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.4.1	SmartCard Section Information - sc_section_t</h4>
<p>This information is for the start section during the SmartCard reset process.</p>
<p>Defined in include/idl_smartcard.h.</p>
\code
enum sc_section_t
\endcode

\anchor index_3_2_4_2
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.4.2	SmartCard Slot Information - sc_slot_info_t</h4>
<p>This data structure is used to contain the SmartCard slot information.</p>
<p>Defined in include/idl_smartcard.h.</p>
\code
enum sc_slot_info_t
\endcode
<p>
The field <b>protocol</b> indicates card type selected for operation: T0 or T1.</p>
<p>	
The field <b>status</b> indicates the current card status, it can be:
<p>
\code
#define SC_CARD_NO_CARD_INSERTED        0
#define SC_CARD_INSERTED_NOT_POWERED_UP 1
#define SC_CARD_POWERED_UP              2
\endcode

\anchor index_3_2_4_3
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.4.3	SmartCard Slot Information - sc_event_t</h4>

\code
#define SC_EVENT_OVRN      0x0001    // Receive data overrun interrupt 
#define SC_EVENT_PERR      0x0002    // Parity error interrupt 
#define SC_EVENT_T0ERR     0x0004    // T=0 error interrupt 
#define SC_EVENT_FRAMERR   0x0008    // Framing error interrupt 
#define SC_EVENT_TIMEO     0x0010    // Receive timeout interrupt 
#define SC_EVENT_CWT	   0x0020    // Character waiting time interrupt 
#define SC_EVENT_BWT       0x0040    // Block waiting time interrupt 
#define SC_EVENT_RDR       0x0100    // Receive Data Ready interrupt 
#define SC_EVENT_TDR       0x0200    // Transmit data refill interrupt 
#define SC_EVENT_CARD_DET  0x0400    // SmartCard detection interrupt 
#define SC_EVENT_FIFO_TIME 0x2000    // FIFO_RX interrupt 
#define SC_EVENT_FIFO_RX   0x4000    // FIFO_TIME interrupt 
//Only SC_EVENT_CARD_DET is available
\endcode

\anchor index_3_2_5
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.5	IDL SPI Special Compound Data Types</h3>
\anchor index_3_2_5_1
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.5.1	SPI Configuration Parameters - spi_config_t</h4>
	
<p>This configuration is used to set up the SPI work mode.</p>
<p>Defined in include/idl_spi.h.</p>
\code
enum spi_config_t;
\endcode
\anchor index_3_2_5_2
<h4 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:10pt">3.2.5.2	SPI Status Structure - spi_status_t</h4>
<p>This is used to describe SPI status.</p>
<p>Defined in include/idl_spi.h.</p>
\code
enum spi_status_t;
\endcode

<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.2.5	IDL SPI Special Compound Data Types</h3>

\anchor index_3_4
<h2 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:15pt">3.4	IDL Exported Kernel Symbols</h2>	
<p>
The following section lists the IDL drivers exported kernel symbols for other kernel components (i.e. drivers) that may leverage on the IDL drivers. 
<BR/>
The below APIs which are marked as the Deprecated will be eliminated in a future release, so use of deprecated functions in user code should be eliminated as soon as possible.
</p>

\anchor index_3_4_1
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.4.1	IDL I2C Kernel Symbols</h3>
\code
idl_result_t idl_i2c_get_bus_count(uint8_t *p_bus_count)
idl_result_t idl_i2c_open()
idl_result_t idl_i2c_close()
idl_result_t idl_i2c_reset(uint8_t bus_num)
idl_result_t idl_i2c_set_mode(uint8_t bus_num, idl_i2c_mode_t mode)
idl_result_t idl_i2c_enable_polling(uint8_t bus_num, bool to_poll)
idl_result_t 
idl_i2c_read_sub_addr(uint8_t bus_num, uint16_t slave_addr, uint8_t sub_addr, uint8_t *p_data_buffer, uint32_t byte_count)
idl_result_t 
idl_i2c_read_sub_addr_ex(uint8_t bus_num, uint16_t slave_addr, uint8_t *p_sub_addr, uint32_t sub_addr_byte_count, uint8_t *p_data_buffer, uint32_t byte_count)
idl_result_t 
idl_i2c_write_sub_addr(uint8_t bus_num, uint16_t slave_addr, uint8_t sub_addr, uint8_t *p_data_buffer, uint32_t byte_count)
idl_result_t 
idl_i2c_write_sub_addr_ex(uint8_t bus_num, uint16_t slave_addr, uint8_t *p_sub_addr, 
uint32_t sub_addr_byte_count, uint8_t *p_data_buffer, uint32_t byte_count)
\endcode


\anchor index_3_4_2
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.4.2	IDL GPIO Kernel Symbols</h3>
\code
idl_result_t idl_gpio_init();
void idl_gpio_release(void);
idl_result_t idl_gpio_line_config(uint32_t gpio_num, uint32_t gpio_config);
idl_result_t idl_gpio_set_alt_function(uint32_t gpio_num, uint32_t fn_num);
idl_result_t idl_gpio_set_line(uint32_t gpio_num, uint32_t val);
idl_result_t idl_gpio_get_line(uint32_t gpio_num, uint32_t *val);
\endcode


\anchor index_3_4_3
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.4.3	IDL SPI Kernel Symbols</h3>
\code
idl_result_t idl_spi_init(uint32_t port_num);
idl_result_t idl_spi_release(uint32_t port_num);
idl_result_t idl_spi_set_config(uint32_t spi_port_num, spi_config_t *spi_config)
idl_result_t idl_spi_get_status(uint32_t spi_port_num, spi_status_t *spi_status)
idl_result_t idl_spi_set_event_callback(uint32_t spi_port_num, spi_callback_t callback_function, void *callback_data)
idl_result_t idl_spi_event_enable( uint32_t spi_port_num, spi_event_t spi_events )
idl_result_t idl_spi_event_disable( uint32_t spi_port_num, spi_event_t spi_events )
idl_result_t idl_spi_write(uint32_t spi_port_num, uint32_t data)
idl_result_t idl_spi_read(uint32_t spi_port_num, uint32_t *data)
\endcode

\anchor index_3_4_4
<h3 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:12pt">3.4.4	IDL SmartCard Kernel Symbols</h3>

<p>
There are no IDL SmartCard Driver exported symbols.
</p>
*/
