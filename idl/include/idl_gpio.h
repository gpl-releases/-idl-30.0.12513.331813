/*
#
#  This file is provided under a BSD license.  When using or
#  redistributing this file, you may do so under this license.
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
 * File Name:       idl_gpio.h
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 * 
 */

/** 
* @weakgroup idl_gpio IDL GPIO Functions
* IDL Interface for General Purpose Inputs/Outputs (GPIOs). 
*
* @section idl_gpio_audience Intended Audience
* This library is intended to be used by system designers, device driver 
* developers, and application developers.
*
* @section idl_gpio_overview IDL GPIO Overview
* This library provides a set of functions to access GPIOs on CE4100, CE4200, CE5300 and CE2600 platforms. 
* During GPIO upstreaming stage, one new component named <b>gpio</b> is added into SDK. In order to reduce upstreaming's influence, a shim version of 
* the IDL I2C API has been added to the IDL library. It preserves some essential APIs but implements them with calls to the upstream driver.  This will
* allow customers to make a transition to the upstream driver. In the PR29, the legacy gpio is enabled by default, in the PR30, upstream gpio is enabled by default. Customers could modify the shell varible "USE_UPSTREAM_GPIO" to choose the specific mode in the transition stage. In PR31, the legacy gpio will be removed.
* <BR><BR>     
* The following legacy IDL GPIO functions are <b>deprecated</b> and are not provided in the shim:
* 
* - idl_gpio_interrupt_config()
* - idl_gpio_interrupt_status()
* - idl_gpio_clear_interrupt()
* - idl_gpio_register_interrupt_handler()
* - idl_gpio_release_interrupt_handler()
* - idl_gpio_enable_interrupt()
* - idl_gpio_disable_interrupt()
* 
* - idl_gpio_set_trigger_positive()
* - idl_gpio_set_trigger_negative()
* - idl_gpio_set_gpe()
* - idl_gpio_set_smi()
* - idl_gpio_get_ts()
* - idl_gpio_clear_ts()
*
* Calls to the highlighted functions on interrupt can be replaced with calls to two new functions as indicated below. Other APIs are no long required.<BR>
* User mode:
* - gpio_request_irq()
* - gpio_free_irq()
*
* Kernel mode:
* - gpio_request_irq()
* - gpio_free_irq()
* - gpio_to_irq()
* - request_irq()
* - free_irq()
*
* Calls to the highlighted functions on SMI and GPE are no longer supported. The specific configuration will be done in CEFDK.
*
* If you want to use upstream GPIO APIs, please refer to GPIO document.
*
* @section idl_gpio_num_definition GPIO num definition
* GPIO contains legacy core well GPIO, legacy resume well GPIO and PUB GPIO. In the GPIO driver, there is no
* parameter to distinguish the GPIO group, only a GPIO index (software GPIO index) to specify the pins. 
* Software GPIO index maybe be different from hardware GPIO index. There is also other GPIO group such as Punit GPIO which is not controlled by IDL GPIO
* driver. It is controlled by other software such as Punit firmware.
* - CE4100 platform:
* <table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
* <tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
* <td width="50%">software GPIO index</td> <td>group</td> <td>hardware GPIO name</td></tr>
* <tr>
*	<td>0-11</td> <td>PUB</td> <td>GPIOAUX[0-11]</td>
* </tr>
* <tr>
*	<td>12-21</td> <td>Core Well</td> <td>GPIO[0-9]</td>
* </tr>
* <tr>
*	<td>22-25</td> <td>Resume Well</td> <td>GPIOSUS[0-3]</td>
* </tr>
* </table>

* - CE4200 platform:
* <table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
* <tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
* <td width="50%">software GPIO index</td> <td>group</td> <td>hardware GPIO name</td></tr>
* <tr>
*	<td>0-77</td> <td>PUB</td> <td> GPIO[21-98] </td>
* </tr>
* <tr>
*	<td>78-85</td> <td>Core Well</td> <td>GPIO[0-7] </td>
* </tr>
* <tr>
*	<td>N/A</td> <td>Punit GPIO</td> <td> GPIO[8-20] </td>
* </tr>
* </table>

* - CE5300 platform:
* <table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
* <tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
* <td width="50%">software GPIO index</td> <td>group</td> <td>hardware GPIO name</td></tr>
* <tr>
*	<td>0-127</td> <td>PUB</td> <td>GPIO[0-127]</td>
* </tr>
* <tr>
*	<td>128-135</td> <td>Core Well</td> <td>GPIO[128-135]</td>
* </tr>
* </table>

* - CE2600 platform:
* <table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
* <tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
* <td width="50%">software GPIO index</td> <td>group</td> <td>hardware GPIO name</td></tr>
* <tr>
*	<td>0-127</td> <td>PUB</td> <td>GPIO[0-127]</td>
* </tr>
* <tr>
*	<td>128-135</td> <td>Core Well</td> <td>GPIO[128-135]</td>
* </tr>
* </table>

* @section idl_gpio_pin_mux_control GPIO pin mux control
SOC chip's external pin is limited and valuable. Generally GPIO module multiplexs pins with other fucntion moudules. The API "idl_gpio_set_alt_function" is provided to control pin mux between GPIO module and other function modules. The second parameter determines which module the pin is used as.
The specific GPIO and its multiplex module vary from GPIO index and related platforms.The following table describles the details.
 
- CE4100 platform:
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
<td width="50%">software GPIO index</td> <td>mux value "0"</td> <td>mux value "1"</td></tr>
<tr>
	<td>0,1,2,3,4 or 5</td> <td>GPIO pins(GPIO_AUX[0:5])</td> <td>UART0 flow control pins</td>
</tr>
<tr>
	<td>6 or 7</td> <td>GPIO pins(GPIO_AUX[6:7])</td> <td>UART1_RXD, UART1_TXD enabled</td>
</tr>
<tr>
	<td>8 or 11</td> <td>GPIO pins(GPIO_AUX[8:11])</td> <td>Smartcard 1 pins</td>
</tr>
<tr>
	<td>15 or 16</td> <td>GPIO pins(GPIO[3:4])</td> <td>Micro_TX0, Micro_Rx0 pins</td>
</tr>
<tr>
	<td>17 18 or 19</td> <td>GPIO pins(GPIO[5:7])</td> <td>Smartcard 0 pins</td>
</tr>
<tr>
	<td>20</td> <td>GPIO pin(GPIO[8])</td> <td>GPE link pin</td>
</tr>
<tr>
	<td>21</td> <td>GPIO pin(GPIO[9])</td> <td>thermal sensor pin</td>
</tr>
</table>

- CE4200 platform:
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
<td width="50%">software GPIO index</td> <td>mux value "0"</td> <td>mux value "1"</td></tr>
<tr>
	<td>0</td> <td>GPIO pin(GPIO_AUX[21])</td> <td>GBE link pin</td>
</tr>
<tr>
	<td>1</td> <td>TSD_ICAM fucntion pins</td> <td>Smartcard 0 pins</td>
</tr>
<tr>
	<td>2</td> <td>NAND_CE, NAND_RE_N pins</td> <td>Smartcard 1 pins</td>
</tr>
<tr>
	<td>3,4,5,6,7 or 8</td> <td>GPIO pins(GPIO[24:29]</td> <td>UART0 flow control pins</td>
</tr>
<tr>
	<td>9</td> <td>UART1_RXD UART1_TXD disabled</td> <td>UART1_RXD UART1_TXD enabled</td>
</tr>
</table>

- CE5300 platform:
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
<td width="50%">software GPIO index</td> <td>mux value "0"</td> <td>mux value "1"</td></tr>
<tr>
	<td>64</td> <td>GPIO pin(GPIO[64])</td> <td>UART1_TXD pin</td>
</tr>
<tr>
	<td>66</td> <td>GPIO pin(GPIO[66])</td> <td>UART2_TXD pin</td>
</tr>
<tr>
	<td>33,34,99,100,101 or 102</td> <td>GPIO pins(GPIO[33:34], GPIO[99:102])</td> <td>Smartcard 0 pins</td>
</tr>
<tr>
	<td>58,59,60,61,62 or 63</td> <td>GPIO pins(GPIO[58:63])</td> <td>Smartcard 1 pins</td>
</tr>
<tr>
	<td>95</td> <td>GPIO pin(GPIO[95])</td> <td>GBE link pin</td>
</tr>
<tr>
	<td>35</td> <td>GPIO pin(GPIO[35]) (use internal video_sync_1 as PWM3 trigger input)</td> <td>PWM3 trigger input pin (GPIO[35] as PWM3 trigger input)</td>
</tr>
<tr>
	<td>36</td> <td>GPIO pin(GPIO[36]) (use internal video_sync_1 as PWM2 trigger input)</td> <td>PWM2 trigger input pin (GPIO[36] as PWM2 trigger input)</td>
</tr>
<tr>
	<td>37</td> <td>GPIO pin(GPIO[37]) (use internal video_sync_0 as PWM1 trigger input)</td> <td>PWM1 trigger input pin (GPIO[37] as PWM1 trigger input)</td>
</tr>
<tr>
	<td>38</td> <td>GPIO pin(GPIO[38]) (use internal video_sync_0 as PWM0 trigger input)</td> <td>PWM0 trigger input pin (GPIO[38] as PWM0 trigger input)</td>
</tr>
<tr>
	<td>77</td> <td>GPIO pin(GPIO[77]) (use internal video_sync_0 as PWM0 trigger input)</td> <td>PWM0 trigger input pin (GPIO[77] as PWM0 trigger input)</td>
</tr>
<tr>
	<td>78</td> <td>GPIO pin(GPIO[78]) (use internal video_sync_0 as PWM1 trigger input)</td> <td>PWM1 trigger input pin (GPIO[78] as PWM1 trigger input)</td>
</tr>
<tr>
	<td>79</td> <td>GPIO pin(GPIO[79]) (use internal video_sync_1 as PWM2 trigger input)</td> <td>PWM2 trigger input pin (GPIO[79] as PWM2 trigger input)</td>
</tr>
<tr>
	<td>80</td> <td>GPIO pin(GPIO[80]) (use internal video_sync_1 as PWM3 trigger input)</td> <td>PWM3 trigger input pin (GPIO[80] as PWM3 trigger input)</td>
</tr>
</table>

- CE2600 platform:
<table width="70%" style="border: 1px solid #555" border="1" cellspacing="0" cellpadding="3">
<tr style="color:#FFFFFF; background-color:#0860A8;font-weight:bold;" >
<td width="50%">software GPIO index</td> <td>mux value "0"</td> <td>mux value "1"</td></tr>
<tr>
	<td>38,39,40,41,116,117 or 118</td> <td>GPIO pins(GPIO[38:41],GPIO[116:118])</td> <td>Smart Cart pins</td>
</tr>
<tr>
	<td>42 or 43</td> <td>GPIO pins(GPIO[42:43])</td> <td>GBE link pins</td>
</tr>
<tr>
	<td>48</td> <td>GPIO pin(GPIO[48])</td> <td>UART0 TXD</td>
</tr>
<tr>
	<td>50</td> <td>GPIO pin(GPIO[50])</td> <td>UART1 TXD</td>
</tr>
<tr>
	<td>51</td> <td>GPIO pin(GPIO[51])</td> <td>UART0 RTS</td>
</tr>
<tr>
	<td>55</td> <td>GPIO pin(GPIO[55])</td> <td>PWM0 trigger input pin (GPIO[55]) as PWM0 trigger input</td>
</tr>
<tr>
	<td>54</td> <td>GPIO pin(GPIO[54])</td> <td>PWM1 trigger input pin (GPIO[54]) as PWM1 trigger input</td>
</tr>
<tr>
	<td>56</td> <td>GPIO pin(GPIO[56])</td> <td>PWM0 trigger input pin (GPIO[56]) as PWM0 trigger input</td>
</tr>
<tr>
	<td>57</td> <td>GPIO pin(GPIO[57])</td> <td>PWM1 trigger input pin (GPIO[57]) as PWM1 trigger input</td>
</tr>
</table>

* @section idl_gpio_organization Organization
* The library is subdivided into three general categories:
* - Initialization/Cleanup
* - Configuration
* - Get/Set line state information
* .
* <B> Initialization/Cleanup </B> <br><br>
* These routines are responsible for allocating the resources needed to 
* access the GPIOs and to free up these resources when they are no longer
* needed. This API defines two functions in this category:
* -# idl_gpio_init() - Initialize system resources needed to access GPIO
* interface.
* -# idl_gpio_release() - Release the resourced allocated by idl_gpio_init().
* .
* <B> Configuration </B><br><br>
* These routines configure the different capabilities of the GPIOs.
* -# idl_gpio_line_config() - Configure the specified gpio to act as a data 
* line.
* line.
* -# idl_gpio_set_alt_function() - Set specified GPIO to its alternate 
* function.
*
* <B> Get/Set routines </B><br><br>
* These routines get/set the data values of the GPIO lines, or read the GPIO
* interrupt status register.
* -# idl_gpio_get_line() - Reads the GPIO data line to determine its state.
* -# idl_gpio_set_line() - Outputs a 0 or 1 to the particular GPIO line.
*
* @section idl_gpio_api_notes IDL GPIO API Notes
* <B> Legacy GPIO </B><br><br>
* Legacy GPIO contains legacy core well GPIO and legacy resume well GPIO. It supports triggering SMI/GPE event. The following specific APIs are
* deprecated. Its configuration should be done in CEFDK.
* 
* <B> PUB GPIO </B><br><br>
* PUB GPIO supports interrupt. The IDL GPIO shim layer no longer supports its related API, please refer to GPIO document(in new component GPIO) and
* Linux standard GPIO document for more details on new upstream APIs.
* 
* @section idl_gpio_api_usage IDL GPIO API Usage
* <B>
* Example 1:
* Configure GPIO 73 as an input with inverted polarity.
* </B>
* 
* <CODE>
* idl_result_t status;<br>
* uint32_t line_val;
* 
* if ((status = idl_gpio_init()) != IDL_SUCCESS)
* {
* <I>
* <B>
* handle errors here
* </B>
* </I>
* }
* <br>
* if ((status = idl_gpio_line_config(73, IDL_GPIO_INPUT | IDL_GPIO_INVERT_POLARITY)) != IDL_SUCCESS)
* {
* <I>
* <B>
* handle errors here
* </B>
* </I>
* }
* 
* <I>
* <B>
* ...<br>sometime later, read the GPIO value<br>
* </B>
* </I>
* if ((status = idl_gpio_get_line(73, &line_val)) != IDL_SUCCESS)
* {
* <I>
* <B>
* handle errors here
* </B>
* </I>
* }
* 
* <I>
* <B>
* ...
* </B>
* </I>
* 
* <I>
* <B>
* when finished, release the GPIO device
* </B>
* </I>
* <br>idl_gpio_release();
* 
* </CODE>
* 
* <B>
* Example 2:
* Configure GPIO 3 to be an rising edge interrupt line
* </B>
* 
* <CODE>
* idl_result_t status;<br>
* uint32_t interrupt_status;
* 
* if ((status = idl_gpio_init()) != IDL_SUCCESS)
* {
* <I>
* <B>
* handle errors here
* </B>
* </I>
* }
* <br>
* if ((status = idl_gpio_interrupt_config(3, IDL_GPIO_RISING_EDGE)) != IDL_SUCCESS)
* {
* <I>
* <B>
* handle errors here
* </B>
* </I>
* }
* <br>
* <I>
* <B>
* 
* ...<br>
* Sometime later, after an interrupt<br>
* Read the interrupt status. 
* </B>
* </I>
* <br>
* if ((status = idl_gpio_interrupt_status(3, &interrupt_status)) != IDL_SUCCESS)
* {
* <I>
* <B>
* handle errors here
* </B>
* </I>
* }
* 
* <br>
* <I>
* <B>
* ...
* </B>
* </I>
* <br>
* 
*  <I>
* <B>
* when finished, release the GPIO device
* </B>
* </I>
* <br>idl_gpio_release();
* 
* </CODE>
* 
*
*
*/


#ifndef _IDL_GPIO_H_
#define _IDL_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "idl.h"
#include "osal_type.h"
#include "osal_interrupt.h"
//! \file
/** @defgroup idl_gpio_defs IDL GPIO Definitions */
/** @ingroup idl_gpio_defs */

/*@{*/

/***
* These defines are used to specify the type of GPIO configuration.
*/

/**
* Configure the GPIO for input 
*/
#define IDL_GPIO_INPUT				0

/**
* Configure the GPIO for output
*/
#define IDL_GPIO_OUTPUT				(1 << 0)

/**
* Configure the GPIO to invert polarity
*/
#define IDL_GPIO_INVERT_POLARITY		(1 << 1)

/**
* Configure the GPIO to alternate function
*/
//#define IDL_GPIO_INVERT_ALT	(1 << 2)

/** 
* This enumeration defines the different GPIO interrupt types. <BR><BR>
* The field IDL_GPIO_ACTIVE_HIGH_LEVEL indicates an active high level interrupt type.<BR>  
* The field IDL_GPIO_ACTIVE_LOW_LEVEL indicates an active low level interrupt type.<BR> 
* The field IDL_GPIO_RISING_UP_EDGE indicates a rising edge interrupt type.<BR> 
* The field IDL_GPIO_FALLING_DOWN_EDGE indicates an falling down edge interrupt type.<BR> 
*
*/
typedef enum {
	IDL_GPIO_ACTIVE_HIGH_LEVEL		= 0x0, /**< - Active high level interrupt */
	IDL_GPIO_ACTIVE_LOW_LEVEL		= 0x1, /**< - Active low level interrupt */
	IDL_GPIO_RISING_UP_EDGE			= 0x2, /**< - Rising up edge interrupt  */
	IDL_GPIO_FALLING_DOWN_EDGE		= 0x3, /**< - Falling down edge interrupt */
} idl_gpio_interrupt_type_t;

/**
* This enumeration is used to enable/disable the trigger positive
*/
typedef enum {
	IDL_GPIO_DISABLE_TRIGGER_POSITIVE  = 0x0, /**< - Disable trigger positive */
	IDL_GPIO_ENABLE_TRIGGER_POSITIVE   = 0x1,  /**< - Enable trigger positive */
} idl_gpio_trigger_positive_t;

/**
* This enumeration is used to enable/disable the trigger negative
*/
typedef enum {
	IDL_GPIO_DISABLE_TRIGGER_NEGATIVE  = 0x0, /**< - Disable trigger negative */
	IDL_GPIO_ENABLE_TRIGGER_NEGATIVE   = 0x1,  /**< - Enable trigger negative */
} idl_gpio_trigger_negative_t;

/*@}*/

/** @weakgroup idl_gpio IDL GPIO Functions
*/
/** @ingroup idl_gpio*/
/*@{
*/

/***
**Soc chipset judgment
***/

/**
* This function initializes the GPIO interface. This is the first function that should
* be called to begin using the GPIOs.
* @retval IDL_SUCCESS if GPIOs are initialized
* @retval IDL_NOT_INITIALIZED if system is unable to map GPIO register space or if the system is out of memory.
* @retval IDL_ALREADY_INITIALIZED - if GPIOs are already initialized
*/
idl_result_t idl_gpio_init(void);

/**
* This function releases the GPIO interface. Call this function when you are through using GPIOs.
*/
void	idl_gpio_release(void);

/**
* This function configures a GPIO line. Depending on the capabilities of the particular
* GPIO, the direction may be specified. For multiplexed GPIOs, this automatically
* configures the signal to act as a GPIO.
* @param gpio_num - The number of the GPIO to configure.
* @param gpio_config - Configuration parameters for GPIOs. May be:
* - IDL_GPIO_INPUT
* - IDL_GPIO_OUTPUT
* 
* Examples: 
* -# To configure a GPIO for input, gpio_config should be: 
* IDL_GPIO_INPUT.
* -# To configure a GPIO for output, gpio_config should be: IDL_GPIO_OUTPUT
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number exceeds range, or if GPIO number selected does not support direction type.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_line_config(uint32_t gpio_num, uint32_t gpio_config);

/**
* This function will set the pin associated with the gpio_num to its alternate 
* function. 
* @param gpio_num - The number of the GPIO to configure.
* @param fn_num - The alternate function number. If a pin has only one alternate 
* function, pass in 0. 
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number does not have an alternate function, or if the alternate 
* function number is invalid.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_set_alt_function(uint32_t gpio_num, uint32_t fn_num);

/** 
* This function sets the line state for a GPIO line. 
* @param gpio_num - The number of the GPIO to set
* @param val - Value to write. Only the least significant bit is used, all other bits are ignored.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_set_line(uint32_t gpio_num, uint32_t val);


/** 
* Reads the line state for the GPIO.
* @param gpio_num - The number of the GPIO to query
* @param val - Pointer to return value to write. Will be 0 if clear, 1 if set.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_get_line(uint32_t gpio_num, uint32_t *val);

/** 
* This function configures the type of interrupt that the GPIO line detects.
* @param gpio_num - The number of the GPIO to configure.
* @param interrupt_type - Specifies the type of interrupt. The following types 
* are currently supported:
* 
* - Active High Level Interrupt
* - Active Low Level Interrupt
* - Rising Up Edge Interrupt
* - Falling Down Edge Interrupt
* 
* See idl_gpio_interrupt_type_t enumeration for more information.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number selected does not support interrupts or the specified interrupt type.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_interrupt_config(uint32_t gpio_num, idl_gpio_interrupt_type_t interrupt_type);

/** 
* This function returns the value of the interrupt status bit for the specified GPIO. If interrupts have 
* not been configured for the specified GPIO, the value read is not defined.
* @param gpio_num - The number of the GPIO to get interrupt status for.
* @param interrupt_status - Value of the interrupt status for the specified GPIO. Will be 1 if interrupt is pending on GPIO; 0 otherwise.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number selected does not support interrupts or the specified interrupt type.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_interrupt_status(uint32_t gpio_num, uint32_t *interrupt_status);

/** 
* This function clears the interrupt for the specified GPIO.
* @param gpio_num - The number of the GPIO to clear.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number selected does not support interrupts.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_clear_interrupt(uint32_t gpio_num);

/** 
* This function registers an interrupt handler to be used by the GPIO.
* @param gpio_num - The number of the GPIO to register the interrupt handler for
* @param handler - The function to be called when the interrupt is trigger
* @param data -  A void pointer to be passed to the interrupt handler when the
* interrupt is triggered.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number selected does not support interrupts.
* @retval IDL_NOT_INITIALIZED if the system was unable to register interrupts.
*/
idl_result_t idl_gpio_register_interrupt_handler(uint32_t gpio_num, os_interrupt_handler_t *handler, void *data);

/**
* This function releases the OSAL interrupt handler. It always returns IDL_SUCCESS as 
* the underlying OSAL function does not return an error code.
* @param gpio_num - The number of the GPIO to free.
* @retval IDL_SUCCESS always
*/
idl_result_t 
idl_gpio_release_interrupt_handler(uint32_t gpio_num);

/** 
* This function enables the interrupt mode for the GPIO. 
* @param gpio_num - The number of the GPIO to set
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_enable_interrupt(uint32_t gpio_num);

/** 
* This function disables the interrupt mode for the GPIO. 
* @param gpio_num - The number of the GPIO to set
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_disable_interrupt(uint32_t gpio_num);

/** 
* This function sets the GPIO line to trigger positive mode. 
* @param gpio_num - The number of the GPIO to set
* @param setting - Value to write. Only the least significant bit is used, all other bits are ignored.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_set_trigger_positive(uint32_t gpio_num, uint32_t setting);

/** 
* This function sets the GPIO line to trigger negative mode. 
* @param gpio_num - The number of the GPIO to set
* @param setting - Value to write. Only the least significant bit is used, all other bits are ignored.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_set_trigger_negative(uint32_t gpio_num, uint32_t setting);

/** 
* This function sets the GPE for the GPIO. 
* @param gpio_num - The number of the GPIO to set
* @param setting - Value to write. Only the least significant bit is used, all other bits are ignored.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_set_gpe(uint32_t gpio_num, uint32_t setting);

/** 
* This function sets the SMI for the GPIO. 
* @param gpio_num - The number of the GPIO to set
* @param setting - Value to write. Only the least significant bit is used, all other bits are ignored.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_set_smi(uint32_t gpio_num, uint32_t setting);

/** 
* This function gets the trigger status value from the GPIO. 
* @param gpio_num - The number of the GPIO to set
* @param setting - point to return the ts value.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_get_ts(uint32_t gpio_num, uint32_t *setting);

/** 
* This function clears the trigger status for the GPIO. 
* @param gpio_num - The number of the GPIO to set
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number is invalid. 
*/
idl_result_t idl_gpio_clear_ts(uint32_t gpio_num);

/*@}*/

/**
* This enumeration is used to define the destination interrupt is routed, which is used in CE2600 Platform.
*/
typedef enum idl_gpio_interrupt_router {
	IDL_GPIO_LEGACY_GROUP = 0x0,
	IDL_GPIO_GROUP_A,
	IDL_GPIO_GROUP_B,
	IDL_GPIO_GROUP_RESERVE,
} idl_gpio_interrupt_router_t;

/**
* This function will set the gpio's interrupt router. It is a new feature in CE2600 SOC.
* @param gpio_num - The number of the GPIO to configure.
* @param router - The gpio's interrupt is routed to dest rounter.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number does not have an alternate function, or if the alternate 
* function number is invalid.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_set_interrupt_router(uint32_t gpio_num, idl_gpio_interrupt_router_t router);

/**
* This function will set the gpio's interrupt router. It is a new feature in CE2600 SOC.
* @param gpio_num - The number of the GPIO to configure.
* @param router - The pointer to returned rounter.
* @retval IDL_SUCCESS if successful.
* @retval IDL_INVALID_PARAM if GPIO number does not have an alternate function, or if the alternate 
* function number is invalid.
* @retval IDL_NOT_INITIALIZED if GPIOs are not initialized. Call idl_gpio_init() to initialize the GPIOs.
*/
idl_result_t idl_gpio_get_interrupt_router(uint32_t gpio_num, idl_gpio_interrupt_router_t *router);

#ifdef __cplusplus
}
#endif

#endif

