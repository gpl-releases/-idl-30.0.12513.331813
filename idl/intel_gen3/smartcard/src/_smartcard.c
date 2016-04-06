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
 * File Name:       _smartcard.c
 * $Revision: 0.1 $
 *----------------------------------------------------------------------
 *
 */

#include "osal.h"
#include "_smartcard.h"
#include <linux/wait.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/circ_buf.h>


#define MAX_BUF_SIZE         (512)

struct scard_infc_port {
	_scard_infc_reg_t *pregs;
	_scard_infc_reg_t save_regs;
	uint32_t  ahb_clk; /* ahb clk  KHz */
	struct circ_buf rx;
	spinlock_t lock;
	#define SCARD_RX_DONE           1
	#define SCARD_RX_ERROR          2
	#define SCARD_CARD_REMOVED      3
	#define SCARD_RESET_DONE        4
	#define SCARD_TXING             5
	#define SCARD_WAITING_BWT	6
	#define SCARD_RXING		7
	wait_queue_head_t rx_wait;
	uint32_t need_recv_bytes;
	uint32_t reset_delay;
	uint32_t err;
	uint32_t status;
	uint32_t protocol;
	uint32_t cwtr;
	uint32_t cwt_jiffies;
	uint32_t bwtr;
	uint32_t bwt_jiffies;
	u64	 read_jiffies; // DJG: we have to ensure the oposite delay, since the hardware is not doing this
	struct timer_list bwt_timer;
	struct timer_list cwt_timer;
   uint32_t rst_card_assert_time;
// Some vendors require a dleay between clock/power and reset lines changes
// This flag tell if to do such dleay ot not.
   uint32_t need_delay_between_lines;
   uint32_t ccr_user_bits;
};

static struct scard_infc_port gscard_infc_port[2];
extern uint32_t ce4200_d0stepping_flag;
/**
 * Forward references
 */
static uint32_t _scard_infc_get_base_addr(uint8_t scard_id);


static uint32_t gscard_infc_addr[] = {0x0, 0x0};
//static uint32_t gscard_phys_base_addr[] = {_BASE_REG_PA_USIM_0, _BASE_REG_PA_USIM_1};

// the delay time (ms) during reset, default to 40 ms;
// With 1 MHz clock 40000 CLK cycles are 40 msec.
#define WAIT_FOR_ATR_AFTER_RESET_MSEC (40)
/*
 * RST_CARD_N assert time - standard say for at least 400 UCLK cycles. 
 * When Fcard is set to the lowest frequency allowed by the standard, 400 cycles take 0.4ms
 * some vendors require the abilty to change this timing.
 */
#define ASSERT_TIME_400_CLOCK_MSEC (1)

static  void cwt_timer_handler(unsigned long data);
static  void bwt_timer_handler(unsigned long data);
static int using_sw_cwtr(uint8_t slot);
static int using_sw_bwtr(uint8_t slot);

#define  DEFAULT_CWTR            (9600 - 12)
#define SCARD_PCI_BUS_NUM 1
#define SCARD_PCI_DEVICE_NUM 11
#define SCARD_PCI_FUNC_NUM 3
/*
 * This routine will return the physical base address of Smart card
 * interface 
 * @retval Base address if smartcard interface is supported other wise 0x0 is 
 * returned.
 */
static uint32_t _scard_infc_get_base_addr(uint8_t slot)
{
	uint32_t base_addr;
	uint32_t phyad;

	if (gscard_infc_addr[slot] != 0)
	{
		/* base_addr already mapped */
		base_addr = gscard_infc_addr[slot];
	}
	else
	{
#ifdef HARDCODE_BAR
		base_addr = (uint32_t) OS_MAP_IO_TO_MEM_NOCACHE(gscard_phys_base_addr[slot], 0xff);
#else
		os_pci_dev_t  pci_dev = NULL;

		//get the device  SCARD
		if(os_pci_device_from_address(&pci_dev, SCARD_PCI_BUS_NUM, SCARD_PCI_DEVICE_NUM, SCARD_PCI_FUNC_NUM) != OSAL_SUCCESS)
		{
			OS_INFO("Unable to access the PCI DEVICE SCARD\n");
			return false;
		}

		
		if(slot == 0){
			//read the SCARD_0 base address from bar 0 
			os_pci_read_config_32(pci_dev, 0x10, &phyad);	
		}else{
			//read the SCARD_1 base address from bar 1 
			os_pci_read_config_32(pci_dev, 0x14, &phyad);		
		}

		base_addr = (uint32_t)OS_MAP_IO_TO_MEM_NOCACHE(phyad,0xff);
		OS_PCI_FREE_DEVICE(pci_dev);
#endif
	}

	return (base_addr);
}

/*
 * This function initializes the smartcard interface. This is the first function that should
 * be called, it reset all the smartcard interface register to HW reset values.
 */
int _scard_infc_init(uint8_t slot)
{
	char *buf;
	
	gscard_infc_addr[slot]  = _scard_infc_get_base_addr(slot);
	gscard_infc_port[slot].pregs = (_scard_infc_reg_t *)(gscard_infc_addr[slot]);

	gscard_infc_port[slot].pregs->egtr = 0x00000005;
	gscard_infc_port[slot].pregs->bgtr = 0x00000005;
	/*set default parity trigger level to 3*/
	gscard_infc_port[slot].pregs->ecr |= 0x18;
	/*the delay time (ms) during reset, default to 40 ms;*/
	gscard_infc_port[slot].reset_delay = WAIT_FOR_ATR_AFTER_RESET_MSEC;
	gscard_infc_port[slot].need_delay_between_lines = 0;
	gscard_infc_port[slot].ccr_user_bits=0;
	gscard_infc_port[slot].rst_card_assert_time= ASSERT_TIME_400_CLOCK_MSEC;
	buf = kmalloc(MAX_BUF_SIZE, GFP_KERNEL);
	if (buf == NULL)
		return -1;
	gscard_infc_port[slot].rx.buf = buf;
	gscard_infc_port[slot].rx.head = gscard_infc_port[slot].rx.tail = 0;
	gscard_infc_port[slot].status = 0;
	_scard_infc_set_protocol(slot, SC_T0);
	_scard_infc_set_cwt(slot, DEFAULT_CWTR);
	spin_lock_init(&gscard_infc_port[slot].lock);
	init_timer(&gscard_infc_port[slot].bwt_timer);
	gscard_infc_port[slot].bwt_timer.function = bwt_timer_handler;
	gscard_infc_port[slot].bwt_timer.data = slot;
	init_timer(&gscard_infc_port[slot].cwt_timer);
	gscard_infc_port[slot].cwt_timer.function = cwt_timer_handler;
	gscard_infc_port[slot].cwt_timer.data = slot;
	gscard_infc_port[slot].read_jiffies = 0;
	init_waitqueue_head(&gscard_infc_port[slot].rx_wait);

	_scard_infc_set_tx_trigger_level(slot, 0);
	_scard_infc_set_rx_trigger_level(slot, 1);
	/*set dds register for CE4200 D0, make USIM_CLK frequency be 66.6MHZ as other silicon*/
        if(ce4200_d0stepping_flag){
		switch(gscard_infc_port[slot].ahb_clk){
                       case 1066666:
                               gscard_infc_port[slot].pregs->dds = 0x10000000;
                               break;
                       case 1333333: 
                               gscard_infc_port[slot].pregs->dds = 0xCCCCCB3;
                               break;
                       case 800000:
                               gscard_infc_port[slot].pregs->dds = 0x15555555;
                               break;
                       default:
                               break;
               }
        }

        /*set dds_enable bit in CLKR register for CE4200 D0*/
        if(ce4200_d0stepping_flag){
                gscard_infc_port[slot].pregs->clkr |= _SCARD_INFC_CLKR_DDS_ENABLE;
        }
	_scard_infc_set_etu(slot,7,372,13);

	return (0);
}

/*
 * This function frees any memory associated with the smartcard interface. This 
 * functions should be called last
 */
int _scard_infc_deinit(uint8_t slot)
{
	/* clear all pending interrupts */
	if (_scard_infc_clear_interrupt(slot, SC_EVENT_MASK) < 0) {
		return IDL_FAILURE;
	}
	if(ce4200_d0stepping_flag){
		gscard_infc_port[slot].pregs->clkr &= ~_SCARD_INFC_CLKR_DDS_ENABLE;
	}
	kfree(gscard_infc_port[slot].rx.buf);
	if (gscard_infc_port[slot].pregs != NULL)
	{
		OS_UNMAP_IO_FROM_MEM(gscard_infc_port[slot].pregs, sizeof(*gscard_infc_port[slot].pregs));
		gscard_infc_port[slot].pregs = NULL;
		gscard_infc_addr[slot]  = 0x0;
	}

	return (0);
}

/*
 * This routine will perform cold/warm reset to the card connected to 
 * smartcard interface and activate it
 */
int _scard_infc_reset(uint8_t slot, sc_reset_t type, sc_vcc_t vcc)
{
	int ret = 0;
	uint32_t dummy;

	// If a warm start is requested with no cold start or power on before - move to cold start.
	if ((gscard_infc_port[slot].pregs->ccr &0x6)==0){
		printk("\nCard voltage at 0V. Move to cold reset\n");
		type=SC_RESET_TYPE_COLD;
	}

	switch (type) {
		case SC_RESET_TYPE_COLD:
			_scard_infc_card_deactivate(slot);
			os_sleep(1);
			/* 
			 * turn on the VCC voltage by writing to the CCR.VCC bits.on first 
			 * activation, set the lowest voltage level.(set the CCR.VCC bits 
			 * to "10").
			 */
			ret = _scard_infc_set_card_vcc(slot, vcc);
			if (ret < 0) 
				return ret; 

			/* enable the i/o line to return to Vhigh by setting the CCR.TXD_FORCE bit to 0 */
			gscard_infc_port[slot].pregs->ccr &= 0xffffffef;

			/* activate the card clock by setting the stop bit, CLKR.STOP_UCLK to 0 */
			gscard_infc_port[slot].pregs->clkr &= 0xffffdfff;
			/*
			 * verify that CCR.RST_CARD_N was asserted for configured time
			*/
			os_sleep(gscard_infc_port[slot].rst_card_assert_time);
			/*
			 * deassert CCR.RST_CARD_N by setting it to 1. the card will Answer To Reset(ATR)
			 * within 400-40000 card clock cycles 
			 */
			gscard_infc_port[slot].pregs->ccr |= 0x00000001;
			_scard_infc_reset_tx_fifo(slot);
			_scard_infc_reset_rx_fifo(slot);
			gscard_infc_port[slot].bwtr = DEFAULT_CWTR;
			gscard_infc_port[slot].pregs->bwtr = DEFAULT_CWTR;
			gscard_infc_port[slot].cwtr = DEFAULT_CWTR;
			gscard_infc_port[slot].pregs->cwtr = DEFAULT_CWTR;
			del_timer(&gscard_infc_port[slot].bwt_timer);
			del_timer(&gscard_infc_port[slot].cwt_timer);
			gscard_infc_port[slot].status = SCARD_RESET_DONE;
			_scard_infc_enable_interrupt(slot, SC_EVENT_RX);
			gscard_infc_port[slot].read_jiffies = 0;
			os_sleep(gscard_infc_port[slot].reset_delay);
			break;
		case SC_RESET_TYPE_WARM:
			/* set CCR.RST_CARD_N to logic 0 */
			gscard_infc_port[slot].pregs->ccr &= 0xfffffffe;
			
			/* enable the i/o line to return to Vhigh by setting the CCR.TXD_FORCE bit to 0 */
			gscard_infc_port[slot].pregs->ccr &= 0xffffffef;

			/* activate the card clock by setting the stop bit, CLKR.STOP_UCLK to 0 */
			gscard_infc_port[slot].pregs->clkr &= 0xffffdfff;

			/*
			 * verify that CCR.RST_CARD_N was asserted for configured time
			*/
			os_sleep(gscard_infc_port[slot].rst_card_assert_time);

			/*
			 * deassert CCR.RST_CARD_N by setting it to 1. the card will Answer To Reset(ATR)
			 * within 400-40000 card clock cycles 
			 */
			gscard_infc_port[slot].pregs->ccr |= 0x00000001;
			/* Card Deactivation.Perform the following in sequence to deactivate the USIM card */

			_scard_infc_reset_tx_fifo(slot);
			_scard_infc_reset_rx_fifo(slot);
			del_timer(&gscard_infc_port[slot].bwt_timer);
			del_timer(&gscard_infc_port[slot].cwt_timer);
			gscard_infc_port[slot].status = SCARD_RESET_DONE;
			_scard_infc_enable_interrupt(slot, SC_EVENT_RX);
			gscard_infc_port[slot].read_jiffies = 0;
			os_sleep(gscard_infc_port[slot].reset_delay);
			break;
		default: 
			ret = (-1);
			break;
	}
	dummy = gscard_infc_port[slot].pregs->lsr;
	return (ret);
}

int _scard_infc_set_ahb_clk(uint8_t slot, uint32_t clk)
{
	gscard_infc_port[slot].ahb_clk = clk;
	return 0;
}	

static inline uint32_t etus_to_us(uint8_t slot, uint32_t n_etu)
{
	u64 usec;
	u64 tmp = gscard_infc_port[slot].pregs->dds;
	uint32_t freq;
	
	tmp *= gscard_infc_port[slot].ahb_clk; /*kHz ms*/
	if(ce4200_d0stepping_flag){
		tmp = tmp>>1;
		freq = div_u64(tmp, (1 << 31)); /* 2^32 kHz*/
	} else {
		freq = div_u64(tmp, (1 << 24)); /* 2^24 kHz*/
	}
	usec = n_etu * (gscard_infc_port[slot].pregs->dlr  & 0xFFFF) * (1+(gscard_infc_port[slot].pregs->flr  & 0xFF));
	usec *= 1000;

	usec += freq/2; /* if bigger than 0.5freq, it will increase 1 us*/
	
	usec = div_u64(usec,freq);
	//printk("%s: Waiting time for %u etu's is %u us (freq=%u,clks=%u)\n",__FUNCTION__,n_etu,(uint32_t)usec,freq,clks);
	return (uint32_t)usec;
}

static inline uint32_t etus_to_jiffies(uint8_t slot, unsigned n_etu)
{
	return usecs_to_jiffies(etus_to_us(slot,n_etu));
}
static void pause_before_write(uint8_t slot)
{
	if( gscard_infc_port[slot].read_jiffies){
		u64 now =  get_jiffies_64();
		/* should be 16, one more for safety */
		u64 write_ok = etus_to_jiffies(slot,16 + gscard_infc_port[slot].pregs->egtr) +  gscard_infc_port[slot].read_jiffies;
		if( write_ok >= now)
		{
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(write_ok - now + 1);
		}
	}
	gscard_infc_port[slot].read_jiffies = 0;
}

/*return actual bytes collected in SW FIFO*/
int _scard_sw_fifo_bytes(uint8_t slot)
{
	return CIRC_CNT(gscard_infc_port[slot].rx.head, gscard_infc_port[slot].rx.tail, MAX_BUF_SIZE);
}

static int scard_recv(uint8_t slot)
{
	struct circ_buf *rx;
	uint32_t fsr;
	uint32_t items;
	uint32_t ch;
	uint32_t count;
	int ret = 0;
	
	rx = &gscard_infc_port[slot].rx;
	fsr = gscard_infc_port[slot].pregs->fsr;
	items = ((fsr & FSR_RX_LENGTH_MASK) >> FSR_RX_LENGTH_OFFSET);
	count = CIRC_SPACE(rx->head, rx->tail, MAX_BUF_SIZE);
	
	for (;;) {
		if (items == 0)
			break;
		if (count <= 1){
			ret = -EOVRN;
			break;
		}
		ch = gscard_infc_port[slot].pregs->rbr;
		*(uint8_t *)(rx->buf + rx->head) = ch & 0xFF;
		rx->head = (rx->head + 1) & (MAX_BUF_SIZE - 1);
		items--;
		count --;
	}
	gscard_infc_port[slot].read_jiffies = get_jiffies_64();
	return ret;
}

static int using_sw_cwtr(uint8_t slot)
{
	return (gscard_infc_port[slot].cwtr > 0xFFFF);
}	

static int using_sw_bwtr(uint8_t slot)
{
	return (gscard_infc_port[slot].bwtr > 0xFFFF);
}	
static void cwt_timer_handler(unsigned long data)
{
	uint8_t slot = (uint8_t )data;
	unsigned long flags;

	spin_lock_irqsave(&gscard_infc_port[slot].lock, flags);
	gscard_infc_port[slot].status = SCARD_RX_DONE;
	spin_unlock_irqrestore(&gscard_infc_port[slot].lock, flags);
	wake_up_all(&gscard_infc_port[slot].rx_wait);
}	
static void bwt_timer_handler(unsigned long data)
{
	uint8_t slot = (uint8_t )data;
	unsigned long flags;

	spin_lock_irqsave(&gscard_infc_port[slot].lock, flags);
	gscard_infc_port[slot].status = SCARD_RX_ERROR;
	gscard_infc_port[slot].err = -ETIMEDOUT;
	spin_unlock_irqrestore(&gscard_infc_port[slot].lock, flags);
	wake_up_all(&gscard_infc_port[slot].rx_wait);

}
/*
 * This routine will handle irq handler
 * @param id - 0,1 ( smartcard id)
 * @no retval  is returned
 */
void _scard_infc_irq_handler(uint8_t slot)
{
	uint32_t status;

	status = gscard_infc_port[slot].pregs->iir & SC_EVENT_MASK;
	gscard_infc_port[slot].pregs->iir = status;
	if (status & SC_EVENT_RDR) {
		if (using_sw_cwtr(slot)) {
			mod_timer(&gscard_infc_port[slot].cwt_timer,jiffies + gscard_infc_port[slot].cwt_jiffies);
		}
		if (using_sw_bwtr(slot)) {
			del_timer(&gscard_infc_port[slot].bwt_timer);
		}
		
		/*
		 *scard_recv() function put HW FIFO data into SW FIFO
		 *wake up _scard_infc_read() if there is enough bytes we need in SW FIFO
		 */
		if(scard_recv(slot) != 0){
			gscard_infc_port[slot].err = -EOVRN;
			gscard_infc_port[slot].status = SCARD_RX_ERROR;
		} 
		if ( gscard_infc_port[slot].status == SCARD_WAITING_BWT ){
			gscard_infc_port[slot].status = SCARD_RXING;
		}
		if(_scard_sw_fifo_bytes(slot) >= gscard_infc_port[slot].need_recv_bytes){
			wake_up_all(&gscard_infc_port[slot].rx_wait);
		}
	}
	
	if (status & (SC_EVENT_FRAMERR | SC_EVENT_OVRN)) {
		gscard_infc_port[slot].status = SCARD_RX_ERROR;
		if (status & SC_EVENT_FRAMERR) {
			gscard_infc_port[slot].err = -EFRAME;
		}
		if (status & SC_EVENT_OVRN) {
			gscard_infc_port[slot].err = -EOVRN;
		}	
	}
	
	if (status & SC_EVENT_CARD_DET) {
		gscard_infc_port[slot].status = SCARD_CARD_REMOVED;
		gscard_infc_port[slot].err = -ENODEV;
	}
	
	if ((status & SC_EVENT_PERR)) {
		gscard_infc_port[slot].status = SCARD_RX_ERROR;
		gscard_infc_port[slot].err = -EPARITY;
	}
	
	if (status & SC_EVENT_BWT) {
		if (using_sw_bwtr(slot)) {
			mod_timer(&gscard_infc_port[slot].bwt_timer, jiffies + gscard_infc_port[slot].bwt_jiffies);
		} else {
			gscard_infc_port[slot].status = SCARD_RX_ERROR;
			gscard_infc_port[slot].err = -ETIMEDOUT;
		}
	}

	if ((gscard_infc_port[slot].status == SCARD_RX_ERROR) || 
		(gscard_infc_port[slot].status == SCARD_CARD_REMOVED)) {
			wake_up_all(&gscard_infc_port[slot].rx_wait);
			if (using_sw_cwtr(slot)) {
				del_timer(&gscard_infc_port[slot].cwt_timer);
			}
			if (using_sw_bwtr(slot)) {
				del_timer(&gscard_infc_port[slot].bwt_timer);
			}
			return;
	}
	
	if(gscard_infc_port[slot].status != SCARD_WAITING_BWT){
		if (!using_sw_cwtr(slot) && (status & SC_EVENT_CWT)) {
			gscard_infc_port[slot].status = SCARD_RX_DONE;
			wake_up_all(&gscard_infc_port[slot].rx_wait);
		}
	}

}

/*
 * This routine will deactivate the card
 * @param id - 0,1 ( smartcard id)
 * @retval IDL_SUCCESS if card is deactivated 
 * is returned
 */
int _scard_infc_card_deactivate(uint8_t slot)
{
	/* set CCR.RST_CARD_N to logic 0 */
	gscard_infc_port[slot].pregs->ccr &= 0xfffffffe;

	if (gscard_infc_port[slot].need_delay_between_lines!=0) {
		/* some vendors require 7.5 uSec before clock is stopped.*/                          
		udelay(8);
	}

	/*
	 * stop the clock on Vlow by setting CLKR.STOP_LEVEL bit to 0, and setting the
	 * CLKR.STOP_UCLK bit to 1
	 */
	gscard_infc_port[slot].pregs->clkr &= 0xffff7fff;
	gscard_infc_port[slot].pregs->clkr |= 0x00002000;
	if (gscard_infc_port[slot].need_delay_between_lines!=0) {
		/* Some vendors require that lines will not drooped togther */
		udelay(1);
	}
      
	/* force the i/o line to ground level by setting 1 to the CCR.TXT_FORCE bit */
	gscard_infc_port[slot].pregs->ccr |= 0x00000010;
	if (gscard_infc_port[slot].need_delay_between_lines!=0) {
		/* Some vendors require that lines will not drooped togther */
		udelay(1);
	}

	/*
	 * turn on the VCC voltage to ground level by setting CCR.VCC bits to "00" at least 
	 * after the i/o line is forced low. the i/o line can be checked in the Line Status
	 * Register(LSR).
	 */
	gscard_infc_port[slot].pregs->ccr &= 0xfffffff9;

	return (0);
}

/*
 * This routine will get the current number of bytes in receive buffer 
 */
int _scard_infc_get_rx_length(uint8_t slot)
{
	return CIRC_CNT(gscard_infc_port[slot].rx.head, gscard_infc_port[slot].rx.tail, MAX_BUF_SIZE);
}

/*
 * This routine will get the current number of bytes in transmit buffer 
 */
int _scard_infc_get_tx_length(uint8_t slot)
{
	return ( (gscard_infc_port[slot].pregs->fsr & FSR_TX_LENGTH_MASK) >> FSR_TX_LENGTH_OFFSET);
}

/*
 * This routine will reset and flush the TX FIFO entries
 */
int _scard_infc_reset_tx_fifo(uint8_t slot)
{

	gscard_infc_port[slot].pregs->fcr |= 0x20000000;
	return (0);
}

/*
 * This routine will reset and flush the RX FIFO entries
 */
int _scard_infc_reset_rx_fifo(uint8_t slot)
{
	
	gscard_infc_port[slot].pregs->fcr |= 0x10000000;
	gscard_infc_port[slot].rx.head = gscard_infc_port[slot].rx.tail=0;

	return (0);
}

/*
 * This routine will set the RX trigger level
 */
int _scard_infc_set_rx_trigger_level(uint8_t slot, uint16_t rx_tl)
{
	if (rx_tl > MAX_RX_FIFO_ENTRIES) {
		return (-1);
	}
	else if (slot >= sizeof(gscard_infc_addr)/sizeof(gscard_infc_addr[0])) {
		_OS_DEBUG("SLOT value is incorrect=%d", slot);
		return (-1);
	}
	else {
		gscard_infc_port[slot].pregs->fcr &= (~FCR_RX_TL_MASK);
		gscard_infc_port[slot].pregs->fcr |= rx_tl;
	}

	return (0);
}

/*
 * This routine will set the TX trigger level
 */
int _scard_infc_set_tx_trigger_level(uint8_t slot, uint16_t tx_tl)
{
	if (tx_tl > MAX_TX_FIFO_ENTRIES) {
		return (-1);
	}
	else {
		gscard_infc_port[slot].pregs->fcr &= (~FCR_TX_TL_MASK);
		gscard_infc_port[slot].pregs->fcr |= (tx_tl << FCR_TX_TL_OFFSET);
	}

	return (0);
}

/*
 * This routine will detect the card at smart card interface.
 */
int _scard_infc_get_slot_info(uint8_t slot, sc_slot_info_t *info)
{
	/* get protocol */
	if (gscard_infc_port[slot].pregs->lcr & 0x00000018) {
		info->protocol = SC_T1;
	}
	else {
		info->protocol = SC_T0;
	}

	/* get status */
	if (gscard_infc_port[slot].pregs->ccr & _SCARD_INFC_CCR_SCARD_INSERTED) {
		/* card is inserted, we now have to check if it is powered up */
		if ( ((gscard_infc_port[slot].pregs->ccr & 0x00000007) >> 1) > 0) {
			info->status = SC_CARD_POWERED_UP;
		}
		else {
			info->status = SC_CARD_INSERTED_NOT_POWERED_UP;
		}             
	}
	else {
		info->status = SC_CARD_NO_CARD_INSERTED;
	}

	return (0);
}

/*
 * This routine will set the protocols for the transmitter/receiver
 * transmitter and receiver are programmed with same protocol type
 */
int _scard_infc_set_protocol(uint8_t slot, uint8_t protocol)
{
	int ret = 0;


	switch (protocol) {
		case SC_T0:
			gscard_infc_port[slot].protocol = SC_T0;
			gscard_infc_port[slot].pregs->lcr &= 0xffffffe7;
			break;
		case SC_T1:
			gscard_infc_port[slot].protocol = SC_T1;
			gscard_infc_port[slot].pregs->lcr |= 0x00000018;
			break;
		default:
			ret = (-1);
			break;
	}

	return (ret);
}

/*
 * This routine will enable the interrupts for smart card interface
 */
int _scard_infc_enable_interrupt(uint8_t slot, uint32_t mask)
{
	int ier = 0;

	if ( mask & (~SC_EVENT_MASK) ) {	
		return (-1);
	}

	ier = gscard_infc_port[slot].pregs->ier;
	ier |= (mask & SC_EVENT_MASK); 
	gscard_infc_port[slot].pregs->ier = ier; 

	return (0);
}

/*
 * This routine will disable the interrupts for smart card interface
 */
int _scard_infc_disable_interrupt(uint8_t slot, uint32_t mask)
{
	int ier = 0;

	if ( mask & (~SC_EVENT_MASK) ) {	
		return (-1);
	}

	ier = gscard_infc_port[slot].pregs->ier;
	ier &= (~(mask & SC_EVENT_MASK)); 
	gscard_infc_port[slot].pregs->ier = ier; 

	return (0);
}

/*
 * This routine will clear the interrupt for smart card interface by clearing
 * the status bits
 */
int _scard_infc_clear_interrupt(uint8_t slot, uint32_t mask)
{
	if ( (mask & (~SC_EVENT_MASK)) ) {
		return (-EINVAL);
	}
	gscard_infc_port[slot].pregs->iir |= mask;

	return (0);
}

int _scard_get_interrupt_status(uint8_t slot, uint32_t *status)
{
	*status = (gscard_infc_port[slot].pregs->iir & SC_EVENT_MASK);

	return (0);
}

/*
 * This routine will read a byte from smartcard interface RX FIFO and activate it
 */
int _scard_infc_read(uint8_t slot, uint8_t *s, uint32_t nbytes, uint32_t *actual_bytes_read) 
{
	struct circ_buf *rx;
	uint32_t count = 0;
	int ret = 0;
	int i;
	gscard_infc_port[slot].need_recv_bytes = nbytes;
	rx = &gscard_infc_port[slot].rx;
	*actual_bytes_read = 0;

	/*when receive enough bytes or RX operation done or RX operation error, stop waiting*/
	wait_event(gscard_infc_port[slot].rx_wait, (gscard_infc_port[slot].status == SCARD_RX_DONE) || \
									(gscard_infc_port[slot].status == SCARD_RX_ERROR) || \
									(gscard_infc_port[slot].status == SCARD_CARD_REMOVED) || \
									_scard_sw_fifo_bytes(slot) >= nbytes);
	count = CIRC_CNT(rx->head, rx->tail, MAX_BUF_SIZE);
	if (count < nbytes)
		nbytes = count;
	for (i=0; i < nbytes; i++) {
		*s++ = *(uint8_t *)(rx->buf + rx->tail);
		rx->tail = (rx->tail + 1) & (MAX_BUF_SIZE - 1);
	}
	if ((gscard_infc_port[slot].status == SCARD_RX_ERROR)||(gscard_infc_port[slot].status == SCARD_CARD_REMOVED)) {
		ret = gscard_infc_port[slot].err;
	}
	if(gscard_infc_port[slot].status ==  SCARD_RX_ERROR && ret ==  -EPARITY){
		gscard_infc_port[slot].status = SCARD_RXING;
	}
	*actual_bytes_read = nbytes;

	return ret;
	
}

/*
 * This routine will read a byte from smartcard interface RX FIFO
 * and activate it
 */
int _scard_infc_write(uint8_t slot, uint8_t *s, uint32_t nbytes, uint32_t *actual_bytes_written) 
{
	uint32_t i = 0;
	uint32_t val;
	uint32_t ccr = 0;
	uint32_t lsr = 0;
	uint32_t fsr = 0;
	int ret = 0;

	if (nbytes > MAX_TX_FIFO_ENTRIES)
		nbytes = MAX_TX_FIFO_ENTRIES;

	pause_before_write(slot);
	while (i < nbytes) {
		/* 
		 * if it is the last byte to be writen, then mark bit TX_ENABLE in the bit 31
		 */
		if (i == nbytes - 1) { /* last byte */
			val = (_SCARD_INFC_THR_TX_ENABLE | s[i]);

		}
		else {
			val = s[i];
		}
		/* Write a character byte to the FIFO. */
		gscard_infc_port[slot].pregs->thr = val;
		i++;
	}

	*actual_bytes_written = i;
	del_timer(&gscard_infc_port[slot].bwt_timer);
	del_timer(&gscard_infc_port[slot].cwt_timer);
	gscard_infc_port[slot].status = SCARD_WAITING_BWT;

	do {
		uint32_t left;
		uint32_t cost;
		
		ccr = gscard_infc_port[slot].pregs->ccr;
		lsr = gscard_infc_port[slot].pregs->lsr;
		fsr = gscard_infc_port[slot].pregs->fsr;
		left = (fsr & FSR_TX_LENGTH_MASK) >> FSR_TX_LENGTH_OFFSET;
		cost = etus_to_us(slot, left*12);

		if(cost < (100*1000)/HZ){
			/* 1/10 tick, use delay */
			udelay(cost);
		} else {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(usecs_to_jiffies(cost));
		}

		if (!(ccr & _SCARD_INFC_CCR_SCARD_INSERTED)) {
			ret = -ENODEV;
			break;
		}
		if (lsr & _SCARD_INFC_LSR_T0_ERROR) {
			gscard_infc_port[slot].pregs->ecr |= (1 << 6);/* Clear T0 Error*/
			udelay(10);
			ret = -ET0;
			break;

		}  
		/*when tx is not working and tx fifo is empty,break out*/
		if (!(lsr & _SCARD_INFC_LSR_TX_WORKING) && !(fsr & FSR_TX_LENGTH_MASK) ) {
			break;
		}
	
	} while (1);
	_scard_infc_enable_interrupt(slot, SC_EVENT_RX);
	return ret;

}

/*
 * This routine is called after the card is removed from the interface
 * This routine will complete smooth shut down at interface
 * @param id - 0,1 ( smartcard id)
 * @retval IDL_SUCCESS if card is deactivated 
 * is returned
 */
int _scard_infc_card_removal(uint8_t slot)
{
#if 1
	/* clear the FIFOs */
	_scard_infc_reset_tx_fifo(slot);
	_scard_infc_reset_rx_fifo(slot);

	/* shut down the power supply */
	if (_scard_infc_set_card_vcc(slot, SC_VCC_SEL_0) < 0) {
		return (-1);
	}
#else                
	/* stop the clocks */
	_scard_infc_stop_sclk(slot);

	/* set the clock rate to default value */
	_scard_infc_set_clk_divisor(slot, IDL_SCARD_INFC_CLK_DIV_DEF);
#endif
	return (0);
}

/*
 * This routine configure the clock frequency for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param c_div - 0x01-0xFF 
 * @retval true if etu values are set correctly
 */
int _scard_infc_set_clk_divisor(uint8_t slot, uint32_t c_div)
{
	int i=0;
	if ( (c_div == 0)||(c_div > 0xff) ) {
		_OS_DEBUG("Clock divisior value is incorrect=0x%X[0x0001-0x00FF]",c_div);
		return (-1);
	}
	//Any write to the CLKR when CLKR.RQST = '1' will be ignored
	while((i++<10) && (gscard_infc_port[slot].pregs->clkr & 0x1000));

	if(10 <= i)
	{
		_OS_DEBUG("Request occurring, do not update CLKR\n");
		return (-1);
	} 
	gscard_infc_port[slot].pregs->clkr = (gscard_infc_port[slot].pregs->clkr & 0xffffff00) | (uint8_t) c_div;

	os_sleep(1);

	return (0);
}

int _scard_infc_stop_sclk(uint8_t slot)
{
	gscard_infc_port[slot].pregs->clkr |= 0x00002000;

	return (0);
}

int _scard_infc_start_sclk(uint8_t slot)
{
	gscard_infc_port[slot].pregs->clkr &= 0xffffdfff;

	return (0);
}

int _scard_infc_set_card_vcc(uint8_t slot, sc_vcc_t vcc)
{
	gscard_infc_port[slot].pregs->ccr &= 0xfffffff9;
   gscard_infc_port[slot].pregs->ccr |= ((vcc << 1)|(gscard_infc_port[slot].ccr_user_bits));

	if( ((gscard_infc_port[slot].pregs->ccr & (~0xfffffff9)) >> 1) != vcc) 
		return -1;

	return (0);
}

int _scard_infc_set_egtr(uint8_t slot, uint32_t egtm)
{
	if ( egtm > 0xff ) {
		_OS_DEBUG("EGTR value is incorrect=0x%X[0-0xFF]", egtm);
		return (-1);
	}    

	gscard_infc_port[slot].pregs->egtr = (uint8_t) egtm;

	return (0);
}

int _scard_infc_set_bgtr(uint8_t slot, uint32_t bgt)
{
	if ( bgt > 0xff ) {
		_OS_DEBUG("BGTR value is incorrect=0x%X[0-0xFF]", bgt);
		return (-1);
	}    

	gscard_infc_port[slot].pregs->bgtr = (uint8_t) bgt;

	return IDL_SUCCESS;
}

/*
 * This routine configure the GPIO for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @retval true if gpio are programmed corretly
 */
int _scard_infc_set_gpio(uint8_t scard_id)
{
	/* need to add code to configure GPIO if they are not configured by default */
	return (0);
}

/*
 * This routine configure the etu for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param c_div - 0x01-0xFF 
 * @param D - 0-0xFFFF
 * @param F - 0x05-0xFF
 * @retval true if etu values are set correctly
 */
int _scard_infc_set_etu(uint8_t slot, uint32_t c_div, uint32_t divisor, uint32_t factor)
{
	int ret = -1;

	/* configure factor latch reg */
	if (_scard_infc_set_flr(slot, factor) < 0) {
		return (ret);
	}

	if (_scard_infc_set_clk_divisor(slot, c_div) < 0) {
		return (ret);
	}

	/*
	   The last register write in the sequence needs to be the DLR (Divisor Latch Register) at offset 0x38.  
	   This event triggers a state machine in the USIM baud generator logic to use the values that 
	   are programmed in the baud configuration registers.
	   */

	/* configure divisor latch reg */
	if (_scard_infc_set_dlr(slot, divisor) < 0) {
		return (ret);
	}

	/*update software bwt and cwt timer parameter after setting etu*/	
	if(using_sw_cwtr(slot)){
        	gscard_infc_port[slot].cwt_jiffies = etus_to_jiffies(slot, gscard_infc_port[slot].cwtr) + 1;

	}

	if(using_sw_bwtr(slot)){
		gscard_infc_port[slot].bwt_jiffies = etus_to_jiffies(slot, gscard_infc_port[slot].bwtr) + 1;
	}
	return (0);    
}

/*
 * This routine configure the baud factor for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param F - 0x05-0xFF
 * @retval IDL_SUCCESS if baud factor values are set correctly
 */
int _scard_infc_set_flr(uint8_t slot, uint32_t factor)
{
	if ((factor < 5)||(factor > 0xff)) {
		_OS_DEBUG("Baud factor value is incorrect=0x%X[0x0005-0x00FF]", factor);
		return (-1);
	}
	gscard_infc_port[slot].pregs->flr = (uint8_t) factor ;

	return (0);
}
/*
 * This routine configure the baud divisor for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param F - 0-0xFFFF
 * @retval IDL_SUCCESS if baud divisor values are set correctly
 */
int _scard_infc_set_dlr(uint8_t slot, uint32_t divisor)
{
	if (divisor > 0xffff) {
		_OS_DEBUG("Baud divisor value is incorrect=0x%X[0-0xFFFF]", divisor);
		return (-1);
	}    

	gscard_infc_port[slot].pregs->dlr = (uint16_t) divisor;

	return (0);
}

/*
 * This routine configure the character waiting time for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param D - 0-0xFFFF
 * @retval IDL_SUCCESS if character waiting time values are set correctly
 */
int _scard_infc_set_cwt(uint8_t slot, uint32_t cwt)
{
	/*workaround to add extra 25 etus, later will root cause out the hw timer expire earlier issue*/
	cwt += 25;

	gscard_infc_port[slot].cwtr = cwt;

	if (gscard_infc_port[slot].protocol == SC_T0) {
		_scard_infc_set_bwt(slot, cwt);
	}
	if (using_sw_cwtr(slot)) {
		gscard_infc_port[slot].pregs->cwtr = 0xFFFF;
		gscard_infc_port[slot].cwt_jiffies = etus_to_jiffies(slot, cwt) + 1;
	} else {
		gscard_infc_port[slot].pregs->cwtr = cwt;
		
	}
	return (0);
}

/*
 * This routine configure the block waiting time for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param data - 0-0xFFFF
 * @retval IDL_SUCCESS if block waiting time values are set correctly
 */
int _scard_infc_set_bwt(uint8_t slot, uint32_t bwt)
{
	/*workaround to add extra 25 etus, later will root cause out the hw timer expire earlier issue*/
	bwt += 25;

	gscard_infc_port[slot].bwtr = bwt;
	
	if (using_sw_bwtr(slot)) {
		bwt -= 0xFFFF;
		gscard_infc_port[slot].pregs->bwtr = 0xFFFF;
		gscard_infc_port[slot].bwt_jiffies = etus_to_jiffies(slot, bwt) + 1;
	} else {
		gscard_infc_port[slot].pregs->bwtr = bwt;
	}

	return (0);
}

/*
 * This routine configure the DDS for smartcard interface
 * @param slot - 0,1 ( smartcard id)
 * @param dds_reg_value -Clock Rate configuration (magig number)
 * @retval IDL_SUCCESS 
 */
int _scard_infc_set_dds(uint8_t slot, uint32_t dds)
{
	/*dds should set less than 24 bits and value make USIM_CLK more than 1MHZ */
	if((dds >= 0x1000000) || (dds <0x1EBA5)){
		_OS_DEBUG("DDS value is incorrect=0x%X[0x1EBA5-0xFFFFFF]", dds);
		return (-1);
	}

	/* for CE4200 D0 silicon, round dds as other CE4200 silicon do */
	if(ce4200_d0stepping_flag){
		switch(gscard_infc_port[slot].ahb_clk){
			case 1333333:
				dds = (dds << 8)/10;
				break;
			case 1066666:
				dds = (dds << 5);
				break;
			case 800000:
				dds = (dds << 8)/6;
				break;
			default:
				break;	
		}
                gscard_infc_port[slot].pregs->clkr &= ~_SCARD_INFC_CLKR_DDS_ENABLE;
        }
	gscard_infc_port[slot].pregs->dds =  dds;
	if(ce4200_d0stepping_flag){
                gscard_infc_port[slot].pregs->clkr |= _SCARD_INFC_CLKR_DDS_ENABLE;
        }
	return (0);
}

/*
 * This routine configure the time out register for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param data - 0-0xFFFF
 * @retval IDL_SUCCESS if block waiting time values are set correctly
 */
int _scard_infc_set_tor(uint8_t slot, uint32_t tor)
{
	if (tor > 0xff) {
		_OS_DEBUG("TOR value is incorrect=0x%X[0-0xFF]", tor);
		return (-1);
	}    

	gscard_infc_port[slot].pregs->tor = (uint8_t) tor;

	return (0);
}

/*
 * This routine configure the io mode for smartcard interface
 * @param id - 0,1 ( smartcard id)
 * @param data - 0-0x1
 * @retval IDL_SUCCESS if block waiting time values are set correctly
 */
int _scard_infc_set_iomode(uint8_t slot, uint32_t iomode)
{
	if (iomode > 0x1) {
		_OS_DEBUG("BWT value is incorrect=0x%X[0-0x1]", iomode);
		return (-1);
	}    

	if (iomode == DIRECT_CONVENTION)
	{
		gscard_infc_port[slot].pregs->lcr &= ((uint16_t)(0xFFFC));
		gscard_infc_port[slot].pregs->lcr |= ((uint16_t)(1<<0x2));
	}
	else
	{
		gscard_infc_port[slot].pregs->lcr &= ((uint16_t)(~(1<<0x2)));
		gscard_infc_port[slot].pregs->lcr |= ((uint16_t)(0x0003));
	}

	return (0);
}

/*
 * This routine will clear the interrupt for smart card interface by clearing
 * the status bits
 */
int _scard_infc_adjust_reset_delay(uint8_t slot, uint32_t delay)
{
	if ( delay > 500) {
		return (-1);
	}

	gscard_infc_port[slot].reset_delay = delay; 

	return (0);
}

void _scard_dump_registers(uint8_t slot)
{
	OS_PRINT( "---------------BEGIN(SCARD_%d)-----------------\n", slot);
	OS_PRINT( "ier  = 0x%08x\n", gscard_infc_port[slot].pregs->ier);
	OS_PRINT( "iir  = 0x%08x\n", gscard_infc_port[slot].pregs->iir);
	OS_PRINT( "fcr  = 0x%08x\n", gscard_infc_port[slot].pregs->fcr);
	OS_PRINT( "fsr  = 0x%08x\n", gscard_infc_port[slot].pregs->fsr);
	OS_PRINT( "ecr  = 0x%08x\n", gscard_infc_port[slot].pregs->ecr);
	OS_PRINT( "lcr  = 0x%08x\n", gscard_infc_port[slot].pregs->lcr);
	OS_PRINT( "ccr  = 0x%08x\n", gscard_infc_port[slot].pregs->ccr);
	OS_PRINT( "lsr  = 0x%08x\n", gscard_infc_port[slot].pregs->lsr);
	OS_PRINT( "egtr = 0x%08x\n", gscard_infc_port[slot].pregs->egtr);
	OS_PRINT( "bgtr = 0x%08x\n", gscard_infc_port[slot].pregs->bgtr);
	OS_PRINT( "tor  = 0x%08x\n", gscard_infc_port[slot].pregs->tor);
	OS_PRINT( "clkr = 0x%08x\n", gscard_infc_port[slot].pregs->clkr);
	OS_PRINT( "dlr  = 0x%08x\n", gscard_infc_port[slot].pregs->dlr);
	OS_PRINT( "flr  = 0x%08x\n", gscard_infc_port[slot].pregs->flr);
	OS_PRINT( "cwtr = 0x%08x\n", gscard_infc_port[slot].pregs->cwtr);
	OS_PRINT( "bwtr = 0x%08x\n", gscard_infc_port[slot].pregs->bwtr);
	OS_PRINT( "dds  = 0x%08x\n", gscard_infc_port[slot].pregs->dds);
	OS_PRINT( "-----------------END(SCARD_%d)-----------------\n", slot);
}

/*suspend one smartcard register into scard_reg[slot] structure*/
int _scard_infc_suspend(uint8_t slot)
{
	gscard_infc_port[slot].save_regs.thr = gscard_infc_port[slot].pregs->thr;
	gscard_infc_port[slot].save_regs.ier = gscard_infc_port[slot].pregs->ier;
	gscard_infc_port[slot].save_regs.iir = gscard_infc_port[slot].pregs->iir;
	gscard_infc_port[slot].save_regs.fcr = gscard_infc_port[slot].pregs->fcr;
	gscard_infc_port[slot].save_regs.fsr = gscard_infc_port[slot].pregs->fsr;
	gscard_infc_port[slot].save_regs.ecr = gscard_infc_port[slot].pregs->ecr;
	gscard_infc_port[slot].save_regs.lcr = gscard_infc_port[slot].pregs->lcr;
	gscard_infc_port[slot].save_regs.ccr = gscard_infc_port[slot].pregs->ccr;
	gscard_infc_port[slot].save_regs.lsr = gscard_infc_port[slot].pregs->lsr;
	gscard_infc_port[slot].save_regs.egtr = gscard_infc_port[slot].pregs->egtr;
	gscard_infc_port[slot].save_regs.bgtr = gscard_infc_port[slot].pregs->bgtr;
	gscard_infc_port[slot].save_regs.tor = gscard_infc_port[slot].pregs->tor;
	gscard_infc_port[slot].save_regs.clkr = gscard_infc_port[slot].pregs->clkr;
	gscard_infc_port[slot].save_regs.dlr = gscard_infc_port[slot].pregs->dlr;
	gscard_infc_port[slot].save_regs.flr = gscard_infc_port[slot].pregs->flr;
	gscard_infc_port[slot].save_regs.cwtr = gscard_infc_port[slot].pregs->cwtr;
	gscard_infc_port[slot].save_regs.bwtr = gscard_infc_port[slot].pregs->bwtr;
	gscard_infc_port[slot].save_regs.dds = gscard_infc_port[slot].pregs->dds;
	return 0;
}

/*resume one smartcard register from scard_reg[slot] structure*/
int _scard_infc_resume(uint8_t slot)
{
	gscard_infc_port[slot].pregs->dds = gscard_infc_port[slot].save_regs.dds;
	gscard_infc_port[slot].pregs->ier = (gscard_infc_port[slot].save_regs.ier & 0x707F);
	gscard_infc_port[slot].pregs->fcr = (gscard_infc_port[slot].save_regs.fcr & ((1 << 31) | 0x1FF | (0x1FF << 16) ));
	gscard_infc_port[slot].pregs->ecr = (gscard_infc_port[slot].save_regs.ecr & (0x3 | (0x3 << 3) | (1 << 7)));
	gscard_infc_port[slot].pregs->lcr = (gscard_infc_port[slot].save_regs.lcr & 0x1f);
	//    gscard_infc_port[slot].pregs->ccr = gscard_infc_port[slot].save_regs.ccr;
	gscard_infc_port[slot].pregs->egtr = (gscard_infc_port[slot].save_regs.egtr & 0xFF);
	gscard_infc_port[slot].pregs->bgtr = (gscard_infc_port[slot].save_regs.bgtr & 0xFF);
	gscard_infc_port[slot].pregs->tor = (gscard_infc_port[slot].save_regs.tor & 0xFF);
	if(ce4200_d0stepping_flag){
		gscard_infc_port[slot].save_regs.clkr |= _SCARD_INFC_CLKR_DDS_ENABLE;
        }
	gscard_infc_port[slot].pregs->clkr = (gscard_infc_port[slot].save_regs.clkr & 0x61FF);
	gscard_infc_port[slot].pregs->flr = (gscard_infc_port[slot].save_regs.flr & 0xFF);
	gscard_infc_port[slot].pregs->cwtr = (gscard_infc_port[slot].save_regs.cwtr & 0xFFFF);
	gscard_infc_port[slot].pregs->bwtr = (gscard_infc_port[slot].save_regs.bwtr & 0xFFFF);
	gscard_infc_port[slot].pregs->dlr = (gscard_infc_port[slot].save_regs.dlr & 0xFFFF);
	_scard_infc_reset_tx_fifo(slot);
	_scard_infc_reset_rx_fifo(slot);
	return 0;
}


/*
 * This routine will set power on with defined VCC
 */
int _scard_infc_set_power_on(uint8_t slot, sc_vcc_t vcc)
{
	if ((vcc<SC_VCC_SEL_0) || (vcc>SC_VCC_SEL_5))
	{
		return -1;
	}
	/* de-activate the card clock by setting the stop bit, CLKR.STOP_UCLK to 1 */
	gscard_infc_port[slot].pregs->clkr |= 0x2000;

	/* set CCR.RST_CARD_N to logic 0 */
	gscard_infc_port[slot].pregs->ccr &= 0xfffffffe;

	return (_scard_infc_set_card_vcc(slot,vcc));
}
/*
 * This routine set the time of reset is high during cold/warm reset
 */
int _scard_infc_adjust_reset_time(uint8_t slot, uint32_t rst_assert_time_ms){
   gscard_infc_port[slot].rst_card_assert_time = rst_assert_time_ms; 
   return (0);
}

/*
 * This routine force delay between card line changing during reset/power up/power down
 */
int _scard_infc_force_delay_between_lines(uint8_t slot, uint32_t need_delay){
   gscard_infc_port[slot].need_delay_between_lines=need_delay;
   return (0);
}

/*
 * This routine will change the pin used for data
 */
int _scard_infc_set_data_pin(uint8_t slot, uint8_t pin_for_data){

   gscard_infc_port[slot].pregs->ccr = 
      ((gscard_infc_port[slot].pregs->ccr&~(_SCARD_INFC_CCR_METHOD_BITS))|
       ((pin_for_data&_SCARD_INFC_CCR_METHOD_MASK)<<_SCARD_INFC_CCR_METHOD_POS));
   return 0;
}

/*
 * This routine will set default value for CCR User bit
 */
int _scard_infc_set_ccr_user_bits(uint8_t slot, uint8_t ccr_user_bits){
   gscard_infc_port[slot].ccr_user_bits = 
      ccr_user_bits & _SCARD_INFC_CCR_USER_MODE ;
   return 0;
}


