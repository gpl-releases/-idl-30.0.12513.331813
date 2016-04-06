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

@mainpage Intel(R) Integrated Device Library API

<h1 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:18pt">Preface</h1>	
<p>This document is applicable to the IntelCE Media Processors. In this document, all IntelCE SoCs are referred to as the IntelCE Media Processors. When functional differences occur, applicability to platform/silicon will be delineated.
This document describes the Internal Driver Layer (IDL) driver-programming interface for the IntelCE Media Processors and explains how to access the relative device controller register sets (e.g. I2C, SPI Master, GPIO, and SmartCard). It describes the IDL Application Programming Interface (API), and illustrates the use of the API with some samples.
The IDL API might vary between different releases; therefore, the user should refer to the Release Notes for the latest information. 
Throughout this document, IDL is used as synonym for Internal Driver Layer and IDL API is used as the application-programming interface for accessing features exported by the IDL driver.
</p>

<h1 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:18pt">Contents</H1>     	
     	- <A class="el" HREF=group___i_d_l__summary.html#Definitions>1.1 Definitions</A>
     	
     	- <A class="el" HREF=group___i_d_l__summary.html#index_1_2>1.2 IDL</A>
     	
      - <A class="el" HREF=group___i_d_l__summary.html#idl_package_contents >2. IDL Package Contents</A>
     		- <A HREF=group___i_d_l__summary.html#table_2_1>Table 2-1 Drivers</A>
     		- <A HREF=group___i_d_l__summary.html#table_2_2>Table 2-2 User Libs</A>
     		- <A HREF=group___i_d_l__summary.html#table_2_3>Table 2-3 User Head Files</A>
     		
     	- <A class="el" HREF=group___i_d_l__summary.html#idl_api_overview >3.	IDL API Overview</A>
     	- <A class="el" HREF=group___i_d_l__summary.html#index_3_1>3.1	Scalar Data Types</A> 
     		- <A HREF=group___i_d_l__summary.html#table_3_1>Table 3-1 Drivers</A>
     	- <A class="el" HREF=group___i_d_l__summary.html#index_3_2>3.2	Compound Data Types</A>	
     	
     	- <A class="el" HREF=group___i_d_l__summary.html#index_3_2_1>3.2.1	Common Compound Data Types</A>
					- <A HREF=group___i_d_l__summary.html#index_3_2_1_1>3.2.1.1	IDL I2C Special Compound Data Types</A>
					- <A HREF=group___i_d_l__summary.html#index_3_2_2_1>3.2.2.1	I2C transfer mode parameter idl_i2c_mode_t</A>
     	
     	- <A class="el" HREF=group___i_d_l__summary.html#index_3_2_2>3.2.2	IDL I2C Special Compound Data Types</A>
				- <A HREF=group___i_d_l__summary.html#index_3_2_2_2>3.2.2.2	Async handle structure idl_i2c_read_write_async_handle_t</A>
     	
     	- <A class="el" HREF=group___i_d_l__summary.html#index_3_2_3>3.2.3	IDL GPIO Special Compound Data Types</A>
 					- <A HREF=group___i_d_l__summary.html#index_3_2_3_1 >3.2.3.1	GPIO Interrupt type idl_gpio_interrupt_type_t</A>
 					- <A HREF=group___i_d_l__summary.html#index_3_2_3_2 >3.2.3.2	GPIO gpio_num Definition in IDL</A>
 					- <A HREF=group___i_d_l__summary.html#index_3_2_3_3 >3.2.3.3	GPIO pin mux control </A>
  		
  		- <A class="el" HREF=group___i_d_l__summary.html#index_3_2_4>3.2.4	IDL SmartCard Special Compound Data Types</A>
						- <A HREF=group___i_d_l__summary.html#index_3_2_4_1>3.2.4.1	SmartCard Section Information - sc_section_t</A>
 						- <A HREF=group___i_d_l__summary.html#index_3_2_4_2>3.2.4.2	SmartCard Slot Information - sc_slot_info_t</A>
 						- <A HREF=group___i_d_l__summary.html#index_3_2_4_3>3.2.4.3	SmartCard Slot Information - sc_event_t</A>
  		
  		- <A class="el" HREF=group___i_d_l__summary.html#index_3_2_5>3.2.5	IDL SPI Special Compound Data Types</A>
 						- <A HREF=group___i_d_l__summary.html#index_3_2_5_1>3.2.5.1	SPI Configuration Parameters - spi_config_t</A>
 						- <A HREF=group___i_d_l__summary.html#index_3_2_5_2>3.2.5.2	SPI Status Structure - spi_status_t</A>
						
			- <A class="el" HREF="#" >3.3	IDL API Functions</A> 
						- <A  HREF=group__idl__.html#gaed901d24591e888cd20c8a891f4c5dcf>3.3.1	General IDL Functions</A>
						- <A  HREF=group__idl__i2c.html>3.3.2	IDL I2C Functions</A>
						- <A  HREF=group__idl__gpio.html>3.3.3	IDL GPIO Functions</A>
						- <A  HREF=group__idl__smartcard.html>3.3.4	IDL SmartCard Functions</A>
						- <A  HREF=group__idl__spi.html>3.3.5	IDL SPI Functions</A>

			- <A class="el" HREF="group___i_d_l__summary.html#index_3_4" >3.4	IDL Exported Kernel Symbols</A> 
						- <A HREF="group___i_d_l__summary.html#index_3_4_1" >3.4.1	IDL I2C Kernel Symbols</A>
						- <A HREF="group___i_d_l__summary.html#index_3_4_2" >3.4.2	IDL GPIO Kernel Symbols	</A>				
						- <A HREF="group___i_d_l__summary.html#index_3_4_3" >3.4.3	IDL SPI Kernel Symbols</A>				
						- <A HREF="group___i_d_l__summary.html#index_3_4_4" >3.4.4	IDL SmartCard Kernel Symbols</A>

			- <A class="el" HREF="group___i_d_l__summary.html#index_3_4" >4.	IDL API Usage Model</A> 
						- <A HREF="group__usage__model.html#index_4_1" >4.1	IDL I2C API Usage Model</A>
						- <A HREF="group__usage__model.html#index_4_2" >4.2	IDL GPIO API Usage Model</A>				
						- <A HREF="group__usage__model.html#index_4_3" >4.3	IDL SPI API Usage Model</A>				
						- <A HREF="group__usage__model.html#index_4_4" >4.4	IDL SmartCard API Usage Model</A>
			
			- <A class="el" HREF="group__presudo__code.html" >5.	Pseudo Code for IDL</A> 	
						
				
						
						
<h1 style="color:#FFFFFF; background-color:#0860A8;font-weight:bold; font-size:18pt">Legal Statement</h1>	
<p><b>
INFORMATION IN THIS DOCUMENT IS PROVIDED IN CONNECTION WITH INTEL PRODUCTS. NO LICENSE, EXPRESS OR IMPLIED, BY ESTOPPEL OR OTHERWISE, TO ANY INTELLECTUAL PROPERTY RIGHTS IS GRANTED BY THIS DOCUMENT. EXCEPT AS PROVIDED IN INTEL'S TERMS AND CONDITIONS OF SALE FOR SUCH PRODUCTS, INTEL ASSUMES NO LIABILITY WHATSOEVER, AND INTEL DISCLAIMS ANY EXPRESS OR IMPLIED WARRANTY, RELATING TO SALE AND/OR USE OF INTEL PRODUCTS INCLUDING LIABILITY OR WARRANTIES RELATING TO FITNESS FOR A PARTICULAR PURPOSE, MERCHANTABILITY, OR INFRINGEMENT OF ANY PATENT, COPYRIGHT OR OTHER INTELLECTUAL PROPERTY RIGHT. 
</b></p>
<p>
Intel products are not intended for use in medical, life saving, life sustaining applications.
<BR/>
Intel may make changes to specifications and product descriptions at any time, without notice.
<BR/>
Designers must not rely on the absence or characteristics of any features or instructions marked "reserved" or "undefined." Intel reserves these for future definition and shall have no responsibility whatsoever for conflicts or incompatibilities arising from future changes to them.
<BR/>
This manual may contain design defects or errors known as errata, which may cause the product to deviate from published specifications. Current characterized errata are available on request.
<BR/>
This manual as well as the software, hardware and/or technology described in it, if it is furnished under a license then it may only be used or copied in accordance with the terms of such license. The information in this document is furnished for informational use only, is subject to change without notice, and should not be construed as a commitment by Intel Corporation. <b>Intel Corporation assumes no responsibility or liability for any errors or inaccuracies that may appear in this document or any software that may be provided in association with this document.</b>
<BR/>
Except as permitted by any such license that may be provided with any Intel product no part of this document may be reproduced, stored in a retrieval system, or transmitted in any form or by any means without the express written consent of Intel Corporation. 
<BR/>
Supply of this Implementation of Dolby technology does not convey a license nor imply a right under any patent, or any other industrial or intellectual property right of Dolby Laboratories, to use this Implementation in any finished end-user or ready-to-use final product.  It is hereby notified that a license for such use is required from Dolby Laboratories.
<BR/>
The Intel&reg; Media Processor CE 3100 and Intel&reg; Atom&tm; processor CE4100 includes graphics functionality based on the POWERVR SGX535 from Imagination Technologies.
<BR/>
BunnyPeople, Celeron, Celeron Inside, Centrino, Centrino logo, Chips, Core Inside, Dialogic, EtherExpress, ETOX, FlashFile, i386, i486, i960, iCOMP, InstantIP, Intel, Intel logo, Intel386, Intel486, Intel740, Intel&reg; Media Processor CE 3100, Intel&reg; Atom&tm; processor CE4100, Atom&tm; processor CE5300,Atom&tm; processor CE2600, IntelDX2, IntelDX4, IntelSX2, Intel Core, Intel Inside, Intel Inside logo, Intel. Leap ahead., Intel. Leap ahead. logo, Intel NetBurst, Intel NetMerge, Intel NetStructure, Intel SingleDriver, Intel SpeedStep, Intel StrataFlash, Intel Viiv, Intel XScale, IPLink, Itanium, Itanium Inside, MCS, MMX, MMX logo, Optimizer logo, OverDrive, Paragon, PDCharm, Pentium, Pentium II Xeon, Pentium III Xeon, Performance at Your Command, Pentium Inside, skoool, Sound Mark, The Computer Inside., The Journey Inside, VTune, Xeon, Xeon Inside and Xircom are trademarks or registered trademarks of Intel Corporation or its subsidiaries in the United States and other countries.
<BR/>
Contact your local Intel sales office or your distributor to obtain the latest specifications and before placing your product order.
<BR/>
Copies of documents which have an ordering number and are referenced in this document, or other Intel literature may be obtained by calling 1-800-548-4725 or by visiting Intel's website at http://www.intel.com.
<BR/>
*Other names and brands may be claimed as the property of others.
<BR/>
Copyright 2008 - 2012, Intel Corporation
</p>


*/


