/*
 * Rhino Equipment Corp.  Rhino R1T1 Card Driver
 *
 * Written by
 *          Lee Reeves <lee@rhinoequipment.com>
 *          Bob Conklin <bob@rhinoequipment.com>
 *          Bryce Chidester <bryce@rhinoequipment.com>
 *
 * ** Based on Digium's wct1xxp module **
 *
 * Copyright (C) 2005-2009, Rhino Equipment Corp.
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/moduleparam.h>
#include "r1t1.h"
#include "GpakCust.h"
#include "GpakApi.h"

static int chanmap_t1[] =
{ 1,2,3,
  5,6,7,
  9,10,11,
  13,14,15,
  17,18,19,
  21,22,23,
  25,26,27,
  29,30,31 };

static int chanmap_e1[] =
{ 1,2,3,
  4,5,6,7,
  8,9,10,11,
  12,13,14,15,
  16,17,18,19,
  20,21,22,23,
  24,25,26,27,
  28,29,30,31 };

#define CANARY 0xca1e
#define addr_t (__u32)(dma_addr_t)
#define DEBUG_MAIN      (1 << 0)
#define DEBUG_DSP       (1 << 7)

static int debug = 0;	//Start out with no debugging enabled
static int e1 = 0;	//Defines whether or not the card is set to e1 mode
static int no_ec = 0;
static int ec_disable = 0;	//Mask defining where the ec should be disabled
static int ec_sw = 0xffffffff;	//Mask defining where the ec should be enabled
static int nlp_type = 3;
static int nlp_threshold = 21;
static int nlp_max_supress = 0;

static struct r1t1 *cards[RH_MAX_CARDS];

#ifndef __GENERIC_IO_H

unsigned int fastcall ioread8(void __iomem *);
{
    return *(volatile __u8*) (__iomem);
}

unsigned int fastcall ioread16(void __iomem *);
{
    return *(volatile __u16*) (__iomem);
}

unsigned int fastcall ioread32(void __iomem *);
{
    return *(volatile __u32*) (__iomem);
}

void fastcall iowrite8(u8, void __iomem *);
{
    *(volatile __u8*) (__iomem) = u8;
    return 0;
}

void fastcall iowrite16(u16, void __iomem *);
{
    *(volatile __u16*) (__iomem) = u16;
    return 0;
}

void fastcall iowrite32(u32, void __iomem *);
{
    *(volatile __u32*) (__iomem) = u32;
    return 0;
}

#endif

static int r1t1_open(struct zt_chan *chan)
{
    struct r1t1 *rh = chan->pvt;
    if (rh->dead)
        return -ENODEV;
    rh->usecount++;
    try_module_get(THIS_MODULE);
    if (debug)
        printk("R1T1: use count %d\n", rh->usecount);
    return 0;
}

static int __r1t1_get_reg(struct r1t1 *rh, int reg)
{
    unsigned char res;
    res = ioread8(rh->ioaddr + (reg << 2));
    return res;
}

static int __r1t1_set_reg(struct r1t1 *rh, int reg, unsigned char val)
{
    iowrite8(val, rh->ioaddr + (reg << 2));
    // an extra read to prevent back-to-back burst writes
    ioread8(rh->ioaddr + (reg << 2));
    return 0;
}


static void __r1t1_stop_framer(struct r1t1 *rh)
{
    __r1t1_set_reg(rh, R1T1_CONTROL / 4, 0x00);
}


static int r1t1_framer_hard_reset(struct r1t1 *rh)
{
    int i;
    unsigned long flags;
    unsigned long endjiffies;

    spin_lock_irqsave(&rh->lock, flags);
//    printk("R1T1 hard reset Framer Init\n");
    if (debug) printk("R1T1: is e1 %i\n",rh->ise1);

    /* Soft reset */
    __r1t1_set_reg(rh, DS2155_MSTRREG, 0x01); // Sets *ALL* regs to default values
    __r1t1_set_reg(rh, DS2155_IOCR2, 0x03); // 2.048 Mhz Pll clock for all
    /* 2 TCLK from TCLK pin if alive, or RCLK  6 TCLK from RCLK  4 TCLK from MCLK (Master) */
    __r1t1_set_reg(rh, DS2155_CCR1, 0x06); // 0 TCLK pin = RCLK from Rhino Chip
//    __r1t1_set_reg(rh, DS2155_CCR2, 0x07); // Enable BPCLK 2.048 Mhz
//
    __r1t1_set_reg(rh, DS2155_CCR2, 0x03); // Enable BPCLK 8.019 Mhz
    __r1t1_set_reg(rh, DS2155_IBOC, 0x28); // Set as #1 on 4 wide bus


    if(rh->ise1)
    {
        __r1t1_set_reg(rh, DS2155_MSTRREG, 0x02); // Sets E1 Mode
        __r1t1_set_reg(rh, DS2155_E1TCR1,  0x10); // International Si
        __r1t1_set_reg(rh, DS2155_SIGCR,   0x80); // Sig reinsertion en
        __r1t1_set_reg(rh, DS2155_LIC1,    0x21); // TPD turn off Power Down default 120 LBO
        __r1t1_set_reg(rh, DS2155_LIC4,    0x0f); // 120 transmit term
        __r1t1_set_reg(rh, DS2155_LIC2,    0xd8); // LIRST Line Intf Reset (takes 40ms)
        __r1t1_set_reg(rh, DS2155_LIC2,    0x98); // Sets E1 mode JAMUX, Stops TA1
        __r1t1_set_reg(rh, DS2155_TAF,     0x1b); // Tx Align Frame Sa and Si Pattern
        __r1t1_set_reg(rh, DS2155_TNAF,    0x5f); // Tx Non Align Frame Sa and Si Pattern
        __r1t1_set_reg(rh, DS2155_TSR1,    0x50); // CH-0 clear
        for(i = 0x51; i <= 0x5f; i++) __r1t1_set_reg(rh,i,0x55); // TSR2 - TSR16 = 55
        for(i = 0x8c; i <= 0x8f; i++) __r1t1_set_reg(rh,i,0xff); // TCBR1 - TCBR4 = ff
    }
    else
    {
        __r1t1_set_reg(rh, DS2155_T1RCR1, 0x0C); // Sync Time & Criteria
        __r1t1_set_reg(rh, DS2155_T1TCR1, 0x10); // SW Sig Insertion
        __r1t1_set_reg(rh, DS2155_IBCC, 0x22); // Enable Loop Up and Down
        __r1t1_set_reg(rh, DS2155_RUPCD1, 0x80); // Set the Code Values
        __r1t1_set_reg(rh, DS2155_RUPCD2, 0x00);
        __r1t1_set_reg(rh, DS2155_RDNCD1, 0x80);
        __r1t1_set_reg(rh, DS2155_RDNCD2, 0x00);
        __r1t1_set_reg(rh, DS2155_LIC1, 0x01); // TPD(0) turn off Power Down default LBO
        __r1t1_set_reg(rh, DS2155_LIC4, 0x05); // 75 transmit term
        __r1t1_set_reg(rh, DS2155_LIC2, 0x58); // LIRST(6) Line Intf Reset (takes 40ms)
        __r1t1_set_reg(rh, DS2155_LIC2, 0x18); // Sets T1 mode JAMUX, Stops TA1

    }
    /* Wait 100ms to give plenty of time for reset */
    endjiffies = jiffies + 10;
    while(endjiffies < jiffies);

    __r1t1_set_reg(rh, DS2155_ESCR, 0x55); // Re-align elastic stores
    __r1t1_set_reg(rh, DS2155_ESCR, 0x11); // TX & RX elastic (TSYSCLK IN)

    spin_unlock_irqrestore(&rh->lock, flags);
    return 0;
}

static int r1t1_shutdown(struct zt_span *span)
{
    if(span) {
        struct r1t1 *rh = span->pvt;
        span->flags &= ~ZT_FLAG_RUNNING;

        if(rh) {
            __r1t1_stop_framer(rh);
            r1t1_framer_hard_reset(rh);
        }
    }

    return 0;
}


static void r1t1_release(struct r1t1 *rh)
{

    r1t1_shutdown(&rh->span);

    zt_unregister(&rh->span);

    /* Free resources */
    free_irq(rh->dev->irq, rh);
    pci_free_consistent(rh->dev, ZT_MAX_CHUNKSIZE * 2 * 2 * 32 + 8, (void *)rh->writechunk, rh->writedma);
    iounmap(rh->ioaddr);
    release_mem_region(rh->pciaddr, R1T1_SIZE);
    kfree(rh);
    printk("Released a Rhino r1t1\n");
}

static int r1t1_close(struct zt_chan *chan)
{
    struct r1t1 *rh = chan->pvt;
    rh->usecount--;
    module_put(THIS_MODULE);
    if (debug)
        printk("R1T1: use count %d\n", rh->usecount);
    /* If we're dead, release us now */
    if (!rh->usecount && rh->dead)
        r1t1_release(rh);
    return 0;
}

static void r1t1_enable_interrupts(struct r1t1 *rh)
{
}

static void r1t1_start_dma(struct r1t1 *rh)
{
    /* Reset Master and TDM */
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(1);
    __r1t1_set_reg(rh, R1T1_CONTROL / 4, 0x01);
    if (debug) printk("R1T1: Started DMA\n");
}

static void __r1t1_set_clear(struct r1t1 *rh)
{
    /* Setup registers */
    int x,z;
    unsigned char b;

    /* No such thing under E1 */
    if (rh->ise1) {
        printk("R1T1: Can't set clear mode on an E1!\n");
        return;
    }

    for (x=0;x<3;x++) {
        b = 0;
        for (z=0;z<8;z++) {
            // Enable software signaling unless the channel is "clear"
            if (!(rh->chans[x * 8 + z].sig & ZT_SIG_CLEAR))
                b |= (1 << z);
        }
        __r1t1_set_reg(rh, DS2155_SSIE + x, b);
    }
}

static void r1t1_t1_framer_start(struct r1t1 *rh)
{
    char *coding, *framing;
    int alreadyrunning = rh->span.flags & ZT_FLAG_RUNNING;
    unsigned long flags;

    spin_lock_irqsave(&rh->lock, flags);

    if (rh->span.lineconfig & ZT_CONFIG_ESF) {
        // CCR1(2) TFM = 1
        __r1t1_set_reg(rh, DS2155_T1CCR1,(__r1t1_get_reg(rh, DS2155_T1CCR1) & 0xfb) | 0x04);
        // RCR2(6) RFM = 1
        __r1t1_set_reg(rh, DS2155_T1RCR2,(__r1t1_get_reg(rh, DS2155_T1RCR2) & 0xbf) | 0x40);
        // Fs bit insertion TCR2 TSLC96 = 0
        __r1t1_set_reg(rh, DS2155_T1TCR2,(__r1t1_get_reg(rh, DS2155_T1TCR2) & 0xbf) & ~0x40);
        __r1t1_set_reg(rh, DS2155_TFDL, 0x00); // FDL data
        framing = "ESF";
    } else {
        // CCR1(2) TFM = 0
        __r1t1_set_reg(rh, DS2155_T1CCR1,(__r1t1_get_reg(rh, DS2155_T1CCR1) & 0xfb) & ~0x04);
        // RCR2(6) RFM = 0
        __r1t1_set_reg(rh, DS2155_T1RCR2,(__r1t1_get_reg(rh, DS2155_T1RCR2) & 0xbf) & ~0x40);
        // Fs bit insertion TCR2(6) TSLC96 = 1 and TCR1(2) TFDLS = 0 (*default*)
        __r1t1_set_reg(rh, DS2155_T1TCR2,(__r1t1_get_reg(rh, DS2155_T1TCR2) & 0xbf) | 0x40);
        __r1t1_set_reg(rh, DS2155_TFDL, 0x1c); // FDL data
        framing = "D4";
    }
    if (rh->span.lineconfig & ZT_CONFIG_B8ZS) {
        // TCR2(7) TB8ZS = 1
        __r1t1_set_reg(rh, DS2155_T1TCR2,(__r1t1_get_reg(rh, DS2155_T1TCR2) & 0x7f) | 0x80);
        // RCR2(5) RB8ZS = 1
        __r1t1_set_reg(rh, DS2155_T1RCR2,(__r1t1_get_reg(rh, DS2155_T1RCR2) & 0xdf) | 0x20);
        coding = "B8ZS";
    } else {
        // TCR2(7) TB8ZS = 0
        __r1t1_set_reg(rh, DS2155_T1TCR2,(__r1t1_get_reg(rh, DS2155_T1TCR2) & 0x7f) & ~0x80);
        // RCR2(5) RB8ZS = 0
        __r1t1_set_reg(rh, DS2155_T1RCR2,(__r1t1_get_reg(rh, DS2155_T1RCR2) & 0xdf) & ~0x20);
        coding = "AMI";

    }
    // Force re-sync
    __r1t1_set_reg(rh, DS2155_T1RCR1,(__r1t1_get_reg(rh, DS2155_T1RCR1) & 0xfe) | 0x01);
    __r1t1_set_reg(rh, DS2155_T1RCR1,(__r1t1_get_reg(rh, DS2155_T1RCR1) & 0xfe) & ~0x01);


    /* Set outgoing LBO */
    // DS2155_LIC1, (rh->span.txlevel << 5) | 0x01);
    __r1t1_set_reg(rh, DS2155_LIC1,(__r1t1_get_reg(rh, DS2155_LIC1) & 0x1f) | (rh->span.txlevel << 5));
    __r1t1_set_clear(rh);

    printk("R1T1: Using %s/%s coding/framing\n", coding, framing);
    if (!alreadyrunning) {
        rh->span.flags |= ZT_FLAG_RUNNING;
    }
    spin_unlock_irqrestore(&rh->lock, flags);
}

static void r1t1_e1_framer_start(struct r1t1 *rh)
{
    char *coding, *framing;
    int alreadyrunning = rh->span.flags & ZT_FLAG_RUNNING;
    unsigned long flags;
    char *crcing = "";

    spin_lock_irqsave(&rh->lock, flags);
    if (debug)
        printk("R1T1: E1 framer start configuration\n");

    if (rh->span.lineconfig & ZT_CONFIG_CCS) {
        // sigcr |= 0x06; //      SIGCR GRSRE(7) = 0 CCS
        __r1t1_set_reg(rh, DS2155_SIGCR ,(__r1t1_get_reg(rh, DS2155_SIGCR ) & 0x7f) & ~0x80);
        // sigcr |= 0x06; //      SIGCR TCCS(1) RCCS(2) = 1 CCS
        __r1t1_set_reg(rh, DS2155_SIGCR ,(__r1t1_get_reg(rh, DS2155_SIGCR ) & 0xfb) | 0x04);
//        __r1t1_set_reg(rh, DS2155_SIGCR ,(__r1t1_get_reg(rh, DS2155_SIGCR ) & 0xfd) | 0x02); /* no RCCS */
        // rcr1 |= 0x40;  //      E1RCR1 RSIGM(6) = 1 CCS
        __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xbf) | 0x40);
        // tcr1 &= 0xbf   //      E1TCR1 T16S(6) = 0 TS16 from SSIE and TSHCS
        __r1t1_set_reg(rh, DS2155_E1TCR1,(__r1t1_get_reg(rh, DS2155_E1TCR1) & 0xbf) & ~0x40);
        framing = "CCS"; /* Receive CCS */
    } else {
        // sigcr |= 0x06; //      SIGCR GRSRE(7) = 1 CAS
        __r1t1_set_reg(rh, DS2155_SIGCR ,(__r1t1_get_reg(rh, DS2155_SIGCR ) & 0x7f) | 0x80);
        // sigcr &= 0xf9; //      SIGCR TCCS(1) RCCS(2) = 0 CAS
        __r1t1_set_reg(rh, DS2155_SIGCR ,(__r1t1_get_reg(rh, DS2155_SIGCR ) & 0xfb) & ~0x04);
//        __r1t1_set_reg(rh, DS2155_SIGCR ,(__r1t1_get_reg(rh, DS2155_SIGCR ) & 0xfd) & ~0x02); /* no RCCS */
        // rcr1 &= 0xbf;  //      E1RCR1 RSIGM(6) = 0 CAS
        __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xbf) & ~0x40);
        // tcr1 |= 0x40;  //      E1TCR1 T16S(6) = 1 TS16 from TS1-TS16
        __r1t1_set_reg(rh, DS2155_E1TCR1,(__r1t1_get_reg(rh, DS2155_E1TCR1) & 0xbf) | 0x40);
        framing = "CAS";
    }
    if (rh->span.lineconfig & ZT_CONFIG_HDB3) {
        // rcr1 |= 0x20;  //      E1RCR1 RHDB3(5) = 1 HDB3
        __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xdf) | 0x20);
        // tcr1 |= 0x04;  //      E1TCR1 THDB3(2) = 1 HDB3
        __r1t1_set_reg(rh, DS2155_E1TCR1,(__r1t1_get_reg(rh, DS2155_E1TCR1) & 0xfb) | 0x04);
        coding = "HDB3";
    } else {
        // rcr1 &= 0xdf;  //      E1RCR1 RHDB3(5) = 0 HDB3
        __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xdf) & ~0x20);
        // tcr1 &= 0xfb;  //      E1TCR1 THDB3(2) = 0 HDB3
        __r1t1_set_reg(rh, DS2155_E1TCR1,(__r1t1_get_reg(rh, DS2155_E1TCR1) & 0xfb) & ~0x04);
        coding = "AMI";
    }
    if (rh->span.lineconfig & ZT_CONFIG_CRC4) {
        // rcr1 |= 0x08; //       E1RCR1 RCRC4(3) = 1
        __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xf7) | 0x08);
        // tcr1 |= 0x01; //       E1TCR1 TCRC4(0) = 1
        __r1t1_set_reg(rh, DS2155_E1TCR1,(__r1t1_get_reg(rh, DS2155_E1TCR1) & 0xfe) | 0x01);
        // tcr2 |= 0x04; //       E1TCR2 AEBE(2) = 1
        __r1t1_set_reg(rh, DS2155_E1TCR2,(__r1t1_get_reg(rh, DS2155_E1TCR2) & 0xfb) | 0x04);
        crcing = " with CRC4";
    } else  {
        // rcr1 &= 0xf7; //       E1RCR1 RCRC4(3) = 0
        __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xf7) & ~0x08);
        // tcr1 &= 0xfe; //       E1TCR1 TCRC4(0) = 0
        __r1t1_set_reg(rh, DS2155_E1TCR1,(__r1t1_get_reg(rh, DS2155_E1TCR1) & 0xfe) & ~0x01);
        // tcr2 &= 0xfb; //       E1TCR2 AEBE(2) = 0
        __r1t1_set_reg(rh, DS2155_E1TCR2,(__r1t1_get_reg(rh, DS2155_E1TCR2) & 0xfb) & ~0x04);
        crcing = "";
    }

    /* Set outgoing LBO */
//    __r1t1_set_reg(rh, DS2155_LIC1, ((rh->span.txlevel << 5) | 0x01));
    __r1t1_set_reg(rh, DS2155_LIC1,(__r1t1_get_reg(rh, DS2155_LIC1) & 0x1f) | (rh->span.txlevel << 5));

    // Force re-sync  E1RCR1 RESYNC(0) = 1 - 0
    __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xfe) | 0x01);
    __r1t1_set_reg(rh, DS2155_E1RCR1,(__r1t1_get_reg(rh, DS2155_E1RCR1) & 0xfe) & ~0x01);


    printk("R1T1: Using %s/%s coding/signaling%s 120 Ohms\n", coding, framing, crcing);
    if (!alreadyrunning) {
        rh->span.flags |= ZT_FLAG_RUNNING;
    }
    spin_unlock_irqrestore(&rh->lock, flags);
}

static int r1t1_framer_sanity_check(struct r1t1 *rh)
{
    if(e1 & (1<<rh->num)) {
        printk("R1T1: %x configurated E1 by module_param\n",rh->num);
        rh->ise1 = 1;   /* Note, not directly assigning to avoid infractions */
    }
    else {
        rh->ise1 = 0;
        printk("R1T1: %x configured as T1\n",rh->num);
    }
    return 0;
}
static int r1t1_rbsbits(struct zt_chan *chan, int bits)
{
    struct r1t1 *rh = chan->pvt;
    unsigned long flags;
    int b,o;
    unsigned char mask;

    /* Byte offset */
    spin_lock_irqsave(&rh->lock, flags);

    if (rh->ise1) {
        if (chan->chanpos % 2) {
            mask = (bits | (rh->chans[chan->chanpos - 1 + 1].txsig << 4));
        }
        else {
            mask = ((bits << 4) | rh->chans[chan->chanpos - 1 - 1].txsig);
        }
        __r1t1_set_reg(rh, 0x51 + (chan->chanpos - 1) / 2, mask);
        rh->chans[chan->chanpos - 1].txsig = bits;
        if (debug)
            printk("R1T1: Register %x, Addr %x, mask %x\n", __r1t1_get_reg(rh, 0x51 + (chan->chanpos - 1) / 2), mask, (0x51 + (chan->chanpos - 1) / 2));

    } else {
        b = (chan->chanpos - 1) / 2;
        o = (chan->chanpos - 1) % 2;

        mask = o ? 0x80 : 0x08;

        if (bits & ZT_ABIT) {
            /* Set A-bit */
            rh->txsig[b] |= mask;
        } else {
            /* Clear A-bit */
            rh->txsig[b] &= ~mask;
        }
        if (bits & ZT_BBIT) {
            /* Set B-bit */
            rh->txsig[b] |= (mask >> 1);
        } else {
            rh->txsig[b] &= ~(mask >> 1);
        }
        if (bits & ZT_CBIT) {
            /* Set C-bit */
            rh->txsig[b] |= (mask >> 2);
        } else {
            rh->txsig[b] &= ~(mask >> 2);
        }
        if (bits & ZT_DBIT) {
            /* Set D-bit */
            rh->txsig[b] |= (mask >> 3);
        } else {
            rh->txsig[b] &= ~(mask >> 3);
        }
        /* Output new values */
        __r1t1_set_reg(rh, 0x50 + b, rh->txsig[b]);
//        printk("reg %x value %x\n", 0x50 + b, __r1t1_get_reg(rh, 0x50 + b));

    }
    spin_unlock_irqrestore(&rh->lock, flags);
    return 0;
}

static int r1t1_ioctl(struct zt_chan *chan, unsigned int cmd, unsigned long data)
{
    switch(cmd) {
    default:
        return -ENOTTY;
    }
}

static int r1t1_startup(struct zt_span *span)
{
    struct r1t1 *rh = span->pvt;

    int i,alreadyrunning = span->flags & ZT_FLAG_RUNNING;

    /* initialize the start value for the entire chunk of last ec buffer */
    for(i = 0; i < span->channels; i++)
    {
        memset(rh->ec_chunk1[i],
            ZT_LIN2X(0,&span->chans[i]),ZT_CHUNKSIZE);
        memset(rh->ec_chunk2[i],
            ZT_LIN2X(0,&span->chans[i]),ZT_CHUNKSIZE);
    }

    /* Reset framer with proper parameters and start */
    if (rh->ise1) {
        r1t1_e1_framer_start(rh);
    } else  {
        r1t1_t1_framer_start(rh);
    }
    printk("R1T1: Calling startup (flags is %d)\n", span->flags);

    if (!alreadyrunning) {
        /* Only if we're not already going */
        r1t1_enable_interrupts(rh);
        r1t1_start_dma(rh);
        span->flags |= ZT_FLAG_RUNNING;
    }
    return 0;
}

static int r1t1_maint(struct zt_span *span, int cmd)
{
    struct r1t1 *rh = span->pvt;
    int res = 0;
    unsigned long flags;
    spin_lock_irqsave(&rh->lock, flags);
    if (rh->ise1) {
        switch(cmd) {
        case ZT_MAINT_NONE:
            __r1t1_set_reg(rh,DS2155_LBCR,0); /* no loop */
            break;
        case ZT_MAINT_LOCALLOOP:
            __r1t1_set_reg(rh,DS2155_LBCR,0x08); /* local loop */
            break;
        case ZT_MAINT_REMOTELOOP:
            __r1t1_set_reg(rh,DS2155_LBCR,0x04); /* remote loop */
            break;
        case ZT_MAINT_LOOPUP:
        case ZT_MAINT_LOOPDOWN:
        case ZT_MAINT_LOOPSTOP:
            res = -ENOSYS;
            break;
        default:
            printk("R1T1: Unknown maint command: %d\n", cmd);
            res = -EINVAL;
            break;
        }
    } else {
        switch(cmd) {
        case ZT_MAINT_NONE:
            __r1t1_set_reg(rh,DS2155_LBCR,0); /* no loop */
            break;
        case ZT_MAINT_LOCALLOOP:
            __r1t1_set_reg(rh,DS2155_LBCR,0x08); /* local loop */
            break;
        case ZT_MAINT_REMOTELOOP:
            __r1t1_set_reg(rh,DS2155_LBCR,0x04); /* remote loop */
            break;
        case ZT_MAINT_LOOPUP:
            __r1t1_set_reg(rh,DS2155_T1CCR1,(__r1t1_get_reg(rh, DS2155_T1CCR1) & 0xfe) | 1); /* send loopup code */
            __r1t1_set_reg(rh,DS2155_IBCC,0x22); /* send loopup code */
            __r1t1_set_reg(rh,DS2155_TCD1,0x80); /* send loopup code */
            __r1t1_set_reg(rh,DS2155_TCD2,0x00); /* send loopup code */
            break;
        case ZT_MAINT_LOOPDOWN:
            __r1t1_set_reg(rh,DS2155_T1CCR1,(__r1t1_get_reg(rh, DS2155_T1CCR1) & 0xfe) | 1); /* send loopdown code */
            __r1t1_set_reg(rh,DS2155_IBCC,0x62); /* send loopdown code */
            __r1t1_set_reg(rh,DS2155_TCD1,0x90); /* send loopdown code */
            __r1t1_set_reg(rh,DS2155_TCD1,0x00); /* send loopdown code */
            break;
        case ZT_MAINT_LOOPSTOP:
            __r1t1_set_reg(rh,DS2155_T1CCR1,(__r1t1_get_reg(rh, DS2155_T1CCR1) & 0xfe));    /* stop sending loopup code */
            break;
        default:
            printk("R1T1: Unknown maint command: %d\n", cmd);
            res = -EINVAL;
       }
    }
    spin_unlock_irqrestore(&rh->lock, flags);
    return res;
}

static int r1t1_chanconfig(struct zt_chan *chan, int sigtype)
{
    if(chan) {
        struct r1t1 *rh = chan->pvt;
        unsigned long flags;
        int alreadyrunning = chan->span->flags & ZT_FLAG_RUNNING;

        spin_lock_irqsave(&rh->lock, flags);

        if (alreadyrunning && !rh->ise1)
            __r1t1_set_clear(rh);

        spin_unlock_irqrestore(&rh->lock, flags);
    }
    return 0;
}

static int r1t1_spanconfig(struct zt_span *span, struct zt_lineconfig *lc)
{
    if(span) {
        struct r1t1 *rh = span->pvt;
        span->lineconfig = lc->lineconfig;
        span->txlevel = lc->lbo;
        span->rxlevel = 0;
        /* Do we want to SYNC on receive or not */
        rh->sync = lc->sync;
        /* If already running, apply changes immediately */
        if (span->flags & ZT_FLAG_RUNNING)
            return r1t1_startup(span);
    }
    return 0;
}

static int r1t1_software_init(struct r1t1 *rh)
{
    int x;

    sprintf(rh->span.name, "R1T1/%d", rh->num);
    sprintf(rh->span.desc, "%s Card %d", rh->variety, rh->num);
    rh->span.spanconfig = r1t1_spanconfig;
    rh->span.chanconfig = r1t1_chanconfig;
    rh->span.startup = r1t1_startup;
    rh->span.shutdown = r1t1_shutdown;
    rh->span.rbsbits = r1t1_rbsbits;
    rh->span.maint = r1t1_maint;
    rh->span.open = r1t1_open;
    rh->span.close = r1t1_close;
    if (rh->ise1)
        rh->span.channels = 31;
    else
        rh->span.channels = 24;
#ifdef ZT_GT_1471
    if (rh->ise1) {
        rh->span.spantype = "E1";
    }
    else {
        rh->span.spantype = "T1";
    }
#endif

    if (rh->ise1) {
		rh->span.linecompat = ZT_CONFIG_HDB3 | ZT_CONFIG_CCS | ZT_CONFIG_CRC4;
    }
    else {
		rh->span.linecompat = ZT_CONFIG_AMI | ZT_CONFIG_B8ZS | ZT_CONFIG_D4 | ZT_CONFIG_ESF;
    }

    rh->span.chans = rh->chans;
    rh->span.flags = ZT_FLAG_RBS;
    rh->span.ioctl = r1t1_ioctl;
    rh->span.pvt = rh;
    if (rh->ise1)
        rh->span.deflaw = ZT_LAW_ALAW;
    else
        rh->span.deflaw = ZT_LAW_MULAW;
    init_waitqueue_head(&rh->span.maintq);
    for (x=0;x<rh->span.channels;x++) {

        if (rh->ise1) {
            rh->chans[x].writechunk = (u_char *)(rh->writechunk +
                (chanmap_e1[x] * ZT_CHUNKSIZE));
            rh->chans[x].readchunk = (u_char *)(rh->readchunk +
                (chanmap_e1[x] * ZT_CHUNKSIZE));
        } else {
            rh->chans[x].writechunk = (u_char *)(rh->writechunk +
                (chanmap_t1[x] * ZT_CHUNKSIZE));
            rh->chans[x].readchunk = (u_char *)(rh->readchunk +
                (chanmap_t1[x] * ZT_CHUNKSIZE));
        }

        if (debug)
            printk("R1T1: Chan %d chanmap %d write %x read %x\n", x, chanmap_t1[x], addr_t rh->chans[x].writechunk, addr_t rh->chans[x].readchunk);


        sprintf(rh->chans[x].name, "R1T1/%d/%d", rh->num, x + 1);
        rh->chans[x].sigcap = ZT_SIG_EM | ZT_SIG_CLEAR | ZT_SIG_EM_E1 |
                      ZT_SIG_FXSLS | ZT_SIG_FXSGS |
                      ZT_SIG_FXSKS | ZT_SIG_FXOLS | ZT_SIG_DACS_RBS |
#ifdef ZT_SIG_FXONS
                      ZT_SIG_FXONS |
#endif
                      ZT_SIG_FXOGS | ZT_SIG_FXOKS | ZT_SIG_CAS | ZT_SIG_SF;
        rh->chans[x].pvt = rh;
        rh->chans[x].chanpos = x + 1;
    }
    if (zt_register(&rh->span, 0)) {
        printk("R1T1: Unable to register span with zaptel\n");
        return -1;
    }
    return 0;
}

static void r1t1_transmitprep(struct r1t1 *rh, int nextbuf)
{
//    volatile unsigned char *txbuf;
//    static int last_int;
//    int x,y;
//    int y;
//    int pos;
//    int samp_num;
//    unsigned char * memptr;

//    txbuf = rh->writechunk + nextbuf * 32 * ZT_CHUNKSIZE;

    zt_transmit(&rh->span);
/*
   if (rh->intcount & 0x0100)
        printk("Intcount %d Intcount %d Intcount %d Intcount %d Intcount %d Intcount %d\n", rh->intcount, rh->intcount, rh->intcount, rh->intcount, rh->intcount, rh->intcount);


    y=0;

    if (rh->usecount) {
//    for (y=0;y<ZT_CHUNKSIZE-1;y++) {
//        for (x=0;x<rh->span.channels;x++) {
//        if ((rh->chans[0].writechunk[y] == (rh->chans[0].writechunk[y+1] + 24)) || (rh->chans[0].writechunk[y] == (rh->chans[0].writechunk[y+1] - 24))) {
        if ((rh->chans[0].writechunk[y] != (rh->chans[0].writechunk[y+1] - 24))) {
//        if (rh->intcount & 0xff00) {
            printk("Intount %d Last Int %d\n", rh->intcount, last_int);
            last_int = rh->intcount;
            printk("Chan 0 chunk %d value %x %x %x\n", y, rh->chans[0].writechunk[y], rh->chans[0].writechunk[y+1], rh->chans[0].writechunk[y+1] - rh->chans[0].writechunk[y]);
            printk("Chan 1 chunk %d value %x %x %x\n", y, rh->chans[1].writechunk[y], rh->chans[1].writechunk[y+1], rh->chans[1].writechunk[y+1] - rh->chans[1].writechunk[y]);
            printk("Chan 2 chunk %d value %x %x %x\n", y, rh->chans[2].writechunk[y], rh->chans[2].writechunk[y+1], rh->chans[2].writechunk[y+1] - rh->chans[2].writechunk[y]);

        }
    }
*/
/*
    memptr = &rh->span.chans[0].writechunk[0];
    memptr -= 8;
    memptr[0] = 0x80;
    memptr[1] = 0x00;
    memptr[2] = 0x80;
    memptr[3] = 0x00;
    memptr[4] = 0x80;
    memptr[5] = 0x00;
    memptr[6] = 0x80;
    memptr[7] = 0x00;

    for (samp_num=0;samp_num<ZT_CHUNKSIZE;samp_num++) {
        rh->span.chans[0].writechunk[samp_num] = 0xA5;
    }
*/
/*
    for (y=0;y<ZT_CHUNKSIZE;y++) {
        for (x=0;x<rh->span.channels;x++) {
            pos = y * 32 + rh->chanmap[x]; // + rh->offset;
            txbuf[pos] = rh->chans[x].writechunk[y];
        }
    }
*/
}

static void r1t1_receiveprep(struct r1t1 *rh, int nextbuf)
{
//    int chan, chunk;

    if (!rh->dsp_up)
        zt_ec_span(&rh->span);
    zt_receive(&rh->span);
/*    
    for (chan=0; chan < rh->span.channels; chan++) {
	for (chunk=0; chunk < ZT_CHUNKSIZE; chunk++ ) {
//	    if (chan > 15) {
                if ((rh->intcount > 12800) && (rh->intcount < 13000))
                    printk("int %d chan = %x chunk = %x value = %x\n", rh->intcount, chan, chunk, rh->chans[chan].readchunk[chunk]);
//            } 
	}
    } 
*/
}

static void __r1t1_check_sigbits(struct r1t1 *rh, int x)
{
    int a,i,y,rxs;

    if (rh->ise1) {
        for (y=0;y<10;y+=2) {
            i = x * 10 + y;
            a = __r1t1_get_reg(rh, 0x61 + (i / 2));
            rxs = (a & 0xf);
            if (!(rh->chans[i].sig & ZT_SIG_CLEAR)) {
                if (rh->chans[i].rxsig != rxs)
                    zt_rbsbits(&rh->chans[i], rxs);
            }
            i++;
            rxs = (a >> 4) & 0xf;
            if (!(rh->chans[i].sig & ZT_SIG_CLEAR)) {
                if (rh->chans[i].rxsig != rxs)
                    zt_rbsbits(&rh->chans[i], rxs);
            }
        }
    } else {
        for (y=0;y<8;y+=2) {
            i = x * 8 + y;
            a = __r1t1_get_reg(rh, 0x60 + (i / 2));
            rxs = a & 0x0f;
            if (!(rh->chans[i].sig & ZT_SIG_CLEAR)) {
                if (rh->chans[i].rxsig != rxs)
                    zt_rbsbits(&rh->chans[i], rxs);
            }
            i++;
            rxs = (a >> 4) & 0x0f;
            if (!(rh->chans[i].sig & ZT_SIG_CLEAR)) {
                if (rh->chans[i].rxsig != rxs)
                    zt_rbsbits(&rh->chans[i], rxs);
            }
        }
    }
}

static void __r1t1_check_alarms(struct r1t1 *rh)
{
    unsigned char c,led_state;
    int alarms;
    int x,j;

    /* Get RIR2 */
    __r1t1_set_reg(rh,DS2155_SR2,0x00);
    c = __r1t1_get_reg(rh, DS2155_SR2);
    //                    RLOSC                   FRCLC
    rh->span.rxlevel = (((c >> 4) & 1) << 1) || ((c >> 5) & 1);

    /* Get status register s*/
    __r1t1_set_reg(rh,DS2155_SR3,0x60);
    c = __r1t1_get_reg(rh, DS2155_SR3);

    /* Assume no alarms */
    led_state = NORM_OP;
    alarms = 0;

    /* And consider only carrier alarms */
    rh->span.alarms &= (ZT_ALARM_RED | ZT_ALARM_BLUE | ZT_ALARM_NOTOPEN);

#if SUPPORT_E1
    if (rh->ise1) {
        /* XXX Implement me XXX */
    } else {
#endif
        /* Detect loopup code if we're not sending one */
        if ((!rh->span.mainttimer) && (c & 0x20)) {
            /* Loop-up code detected */
            led_state = 0xd0;
            if (debug)
                printk("R1T1: SR3 20 LOOP UP\n");
            if ((rh->loopupcnt++ > 80)  && (rh->span.maintstat != ZT_MAINT_REMOTELOOP)) {
                __r1t1_set_reg(rh, DS2155_LBCR, 0x04);  /* Remote Loop */
                rh->span.maintstat = ZT_MAINT_REMOTELOOP;
            }
        } else {
            rh->loopupcnt = 0;
        }
        /* Same for loopdown code */
        if ((!rh->span.mainttimer) && (c & 0x40)) {
            /* Loop-down code detected */
            if (debug)
                printk("R1T1: SR3 04 LOOP DOWN\n");
            led_state = NORM_OP;
            if ((rh->loopdowncnt++ > 80)  && (rh->span.maintstat == ZT_MAINT_REMOTELOOP)) {
                __r1t1_set_reg(rh, DS2155_LBCR, 0x0);   /* No remote Loop */
                rh->span.maintstat = ZT_MAINT_NONE;
            }
        } else
            rh->loopdowncnt = 0;
#if SUPPORT_E1
    }
#endif

    if (rh->span.lineconfig & ZT_CONFIG_NOTOPEN) {
        for (x=0,j=0;x < rh->span.channels;x++)
            if ((rh->chans[x].flags & ZT_FLAG_OPEN) ||
                (rh->chans[x].flags & ZT_FLAG_NETDEV))
                j++;
        if (!j)
            alarms |= ZT_ALARM_NOTOPEN;
    }

    /* Check actual alarm status */
    __r1t1_set_reg(rh,DS2155_SR2,0xff);
    c = __r1t1_get_reg(rh, DS2155_SR2);
    if (c & 0x4) {
        led_state = YEL_ALM;
        if (debug)
            printk("R1T1: SR2 04 BLUE ALARM\n");
        alarms |= ZT_ALARM_BLUE;
    }
    if (c & 0x2) {
        led_state = NO_CARR;
        if (debug)
            printk("R1T1: SR2 02 RED NO CARRIER\n");
        alarms |= ZT_ALARM_RED;
    }
    if (c & 0x1) {
        led_state = NO_SYNC;
        if (debug)
            printk("R1T1: SR2 01 RED NO SYNC\n");
        alarms |= ZT_ALARM_RED;
    }

    /* Keep track of recovering */
    if ((!alarms) && rh->span.alarms)
        rh->alarmtimer = ZT_ALARMSETTLE_TIME;

    /* If receiving alarms, go into Yellow alarm state */
    if (alarms && (!rh->span.alarms)) {
        if (rh->ise1)
            __r1t1_set_reg(rh, DS2155_TNAF, 0x7f);
        else
            __r1t1_set_reg(rh, DS2155_T1TCR1, 0x11);
    }

    if (rh->span.alarms != alarms) {
        if (!(alarms & (ZT_ALARM_RED | ZT_ALARM_BLUE | ZT_ALARM_LOOPBACK)) && rh->sync) {
            /* Use the recieve signalling */
            rh->span.syncsrc = rh->span.spanno;
        } else {
            rh->span.syncsrc = 0;
        }
    }
    if (rh->alarmtimer) {
        alarms |= ZT_ALARM_RECOVER;
        if (debug)
            printk("R1T1: alarmtimer %x alarm clearing\n",rh->alarmtimer);
        led_state = RECOVER;
    }
    if ((c & 0x8) && !(rh->ise1)) {
        alarms |= ZT_ALARM_YELLOW;
        if (debug)
            printk("R1T1: SR2 08 YELLOW ALARM\n");
        led_state = YEL_ALM;
    }

    rh->span.alarms = alarms;
    c = __r1t1_get_reg(rh, R1T1_STATE / 4);
    if (c != led_state) {
         if (debug)
            printk("R1T1: State was %x, Now setting to %x \n",c,led_state);
         __r1t1_set_reg(rh, R1T1_STATE / 4, led_state);
    }
    zt_alarm_notify(&rh->span);
}

static void __r1t1_do_counters(struct r1t1 *rh)
{
    if (rh->alarmtimer) {
        if (!--rh->alarmtimer) {
            rh->span.alarms &= ~(ZT_ALARM_RECOVER);
            /* Clear yellow alarm */
            if (rh->ise1)
                __r1t1_set_reg(rh, DS2155_TNAF, 0x5f);
            else
                __r1t1_set_reg(rh, DS2155_T1TCR1, 0x10);
            zt_alarm_notify(&rh->span);
        }
    }
}
#ifdef ZAP_IRQ_HANDLER
ZAP_IRQ_HANDLER(r1t1_interrupt)
#else
static irqreturn_t r1t1_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
    struct r1t1 *rh = dev_id;
    unsigned long flags;
    unsigned int x, nextbuf;

    nextbuf = ioread8(rh->ioaddr + 0x805) & 0x3;

    if (!(nextbuf & 2))
        return IRQ_NONE;

    nextbuf &= 1;

    rh->intcount++;

    --rh->clocktimeout;

    r1t1_receiveprep(rh, rh->nextbuf);
    r1t1_transmitprep(rh, rh->nextbuf);

    spin_lock_irqsave(&rh->lock, flags);

    /* Acknowledge the interrupt */
    __r1t1_set_reg(rh, R1T1_CONTROL / 4, 0x03);
    __r1t1_set_reg(rh, R1T1_CONTROL / 4, 0x01);

    /* Count down timers */
    __r1t1_do_counters(rh);

    /* Do some things that we don't have to do very often */
    x = rh->intcount & 15 /* 63 */;
    switch(x) {
    case 0:
    case 1:
    case 2:
        __r1t1_check_sigbits(rh, x);
        break;
    case 4:
        /* Check alarms 1/4 as frequently */
        if (!(rh->intcount & 0x30))
            __r1t1_check_alarms(rh);
        break;
    }

    rh->nextbuf = (rh->nextbuf + 1) & 0x01;

    spin_unlock_irqrestore(&rh->lock, flags);

    return IRQ_RETVAL(1);
}

#define USE_G168_DSP

#ifdef USE_G168_DSP

inline void __r1t1_card_pci_out(struct r1t1 *rh, const unsigned int addr, const unsigned int value )
{
	writel(value, &rh->membase[addr]);
}

inline unsigned int __r1t1_card_pci_in(struct r1t1 *rh, const unsigned int addr)
{
    unsigned int value;
	value = readl(&rh->membase[addr]);
    return value;
}

static void r1t1_card_reset_dsp(struct r1t1 *rh)
{
    unsigned long flags;
    unsigned int hpi_c;

    spin_lock_irqsave(&rh->lock, flags);
    hpi_c = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC);

    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, (hpi_c & ~DSP_RST) );
    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, (hpi_c & ~DSP_RST) );
//    msleep(100);
    mdelay(100); // !!!!

    rh->hpi_fast = 0;
    rh->hpi_xadd = 0;
    rh->dsp_sel = 0;

    __r1t1_card_pci_out(rh, R1T1_HCS_REG + TARG_REGS, 0);
    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, (hpi_c | DSP_RST) );

    spin_unlock_irqrestore(&rh->lock, flags);
}


int try_select_dsp(struct r1t1 *rh)
{
    int sel = __r1t1_card_pci_in(rh, R1T1_HCS_REG + TARG_REGS);

    if (sel == 0) {

        __r1t1_card_pci_out(rh, R1T1_HCS_REG + TARG_REGS, 1);

        rh->dsp_sel = 1;
        return (0);
    }
    return (1);
}

void r1t1_card_select_dsp(struct r1t1 *rh)
{
    int ridiculous = 0;

    while (try_select_dsp(rh)) {
        ridiculous++;
        if (ridiculous > 100000) {
	    printk("R1T1: Waited 10 seconds ... nothing happened. Quittig.\n");
            return;
        }
    }
    return;
}

void r1t1_card_unselect_dsp(struct r1t1 *rh)
{
    __r1t1_card_pci_out(rh, R1T1_HCS_REG + TARG_REGS, 0);
    return;
}

static unsigned short int r1t1_card_dsp_ping(struct r1t1 *rh)
{

    gpakPingDspStat_t ping_stat;
    unsigned short int dsp_ver;

    r1t1_card_select_dsp(rh);

    ping_stat = gpakPingDsp(rh, rh->num, &dsp_ver);

    if (debug & DEBUG_DSP) {
        if (ping_stat == PngSuccess)
            printk("R1T1: %d %d: G168 DSP Ping DSP Version %x\n",  rh->num+1, rh->dsp_sel, dsp_ver);
        else
            printk("R1T1: %d %d: G168 DSP Ping Error %d\n", rh->num+1, rh->dsp_sel, ping_stat);
    }

    r1t1_card_unselect_dsp(rh);

    if (ping_stat == PngSuccess)
        return dsp_ver;
    else
        return 0;
}

static int __devinit r1t1_span_download_dsp(struct r1t1 *rh)
{
    unsigned short int DspId;
    gpakDownloadStatus_t dl_res=0;

    r1t1_card_select_dsp(rh);
    DspId = rh->num;
    if ((dl_res = gpakDownloadDsp_5510(rh, DspId, app_file)))
        printk("R1T1: %d DSP %d: G168 DSP App Loader Failed %d\n", rh->num+1, 1, dl_res);
    else
        printk("R1T1: %d DSP %d: G168 DSP App Loader Sucess %d\n", rh->num+1, 1, dl_res);

    r1t1_card_dsp_set(rh, DSP_IFBLK_ADDRESS, 0);
    r1t1_card_dsp_set(rh, DSP_IFBLK_ADDRESS+1, 0);

    r1t1_card_unselect_dsp(rh);

    if (dl_res)
        return -1;
    else
        return 0;
}

static void __devinit r1t1_span_run_dsp(struct r1t1 *rh)
{
    r1t1_card_select_dsp(rh);
    r1t1_card_hpic_set(rh, R1T1_BL_GO);
    r1t1_card_unselect_dsp(rh);
    return;
}

static GpakPortConfig_t Gpak_32_chan_port_config = {

    SlotCfgNone,        // GpakSlotCfg_t         SlotsSelect1          port 1 Slot selection
    0x0000,             // unsigned short int    FirstBlockNum1        port 1 first group Block Number
    0x0000,             // unsigned short int    FirstSlotMask1        port 1 first group Slot Mask
    0x0000,             // unsigned short int    SecBlockNum1          port 1 second group Block Number
    0x0000,             // unsigned short int    SecSlotMask1          port 1 second group Slot Mask
    SerWordSize8,       // GpakSerWordSize_t     SerialWordSize1       port 1 serial word size
    cmpNone,            // GpakCompandModes      CompandingMode1       port 1 companding mode
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t TxFrameSyncPolarity1  port 1 Tx Frame Sync Polarity
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t RxFrameSyncPolarity1  port 1 Rx Frame Sync Polarity
    SerClockActHigh,    // GpakSerClockPol_t     TxClockPolarity1      port 1 Tx Clock Polarity
    SerClockActHigh,     // GpakSerClockPol_t     RxClockPolarity1      port 1 Rx Clock Polarity
    DataDelay1,         // GpakSerDataDelay_t    TxDataDelay1          port 1 Tx data delay
    DataDelay1,         // GpakSerDataDelay_t    RxDataDelay1          port 1 Rx data delay
    Disabled,           // GpakActivation        DxDelay1              port 1 DX Delay
    0x0000,             // unsigned short int    ThirdSlotMask1        port 1 3rd group Slot Mask
    0x0000,             // unsigned short int    FouthSlotMask1        port 1 4th group Slot Mask
    0x0000,             // unsigned short int    FifthSlotMask1        port 1 5th group Slot Mask
    0x0000,             // unsigned short int    SixthSlotMask1        port 1 6th group Slot Mask
    0x0000,             // unsigned short int    SevenSlotMask1        port 1 7th group Slot Mask
    0x0000,             // unsigned short int    EightSlotMask1        port 1 8th group Slot Mask

    SlotCfg8Groups,     // GpakSlotCfg_t         SlotsSelect2          port 2 Slot selection
         0,             // unsigned short int    FirstBlockNum2        port 2 first group Block Number
    0x1110,             // unsigned short int    FirstSlotMask2        port 2 first group Slot Mask
         1,             // unsigned short int    SecBlockNum2          port 2 second group Block Number
    0x1111,             // unsigned short int    SecSlotMask2          port 2 second group Slot Mask
    SerWordSize8,       // GpakSerWordSize_t     SerialWordSize2       port 2 serial word size
    cmpNone,            // GpakCompandModes      CompandingMode2       port 2 companding mode
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t TxFrameSyncPolarity2  port 2 Tx Frame Sync Polarity
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t RxFrameSyncPolarity2  port 2 Rx Frame Sync Polarity
    SerClockActHigh,    // GpakSerClockPol_t     TxClockPolarity2      port 2 Tx Clock Polarity
    SerClockActHigh,     // GpakSerClockPol_t     RxClockPolarity2      port 2 Rx Clock Polarity
    DataDelay1,         // GpakSerDataDelay_t    TxDataDelay2          port 2 Tx data delay
    DataDelay1,         // GpakSerDataDelay_t    RxDataDelay2          port 2 Rx data delay
    Disabled,            // GpakActivation        DxDelay2              port 2 DX Delay
    0x1111,             // unsigned short int    ThirdSlotMask2        port 2 3rd group Slot Mask
    0x1111,             // unsigned short int    FouthSlotMask2        port 2 4th group Slot Mask
    0x1111,             // unsigned short int    FifthSlotMask2        port 2 5th group Slot Mask
    0x1111,             // unsigned short int    SixthSlotMask2        port 2 6th group Slot Mask
    0x1111,             // unsigned short int    SevenSlotMask2        port 2 7th group Slot Mask
    0x1111,             // unsigned short int    EightSlotMask2        port 2 8th group Slot Mask

    SlotCfg8Groups,     // GpakSlotCfg_t         SlotsSelect3          port 3 Slot selection
         0,             // unsigned short int    FirstBlockNum3        port 3 first group Block Number
    0x1110,             // unsigned short int    FirstSlotMask3        port 3 first group Slot Mask
         1,             // unsigned short int    SecBlockNum3          port 3 second group Block Number
    0x1111,             // unsigned short int    SecSlotMask3          port 3 second group Slot Mask
    SerWordSize8,       // GpakSerWordSize_t     SerialWordSize3       port 3 serial word size
    cmpNone,            // GpakCompandModes      CompandingMode3       port 3 companding mode
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t TxFrameSyncPolarity3  port 3 Tx Frame Sync Polarity
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t RxFrameSyncPolarity3  port 3 Rx Frame Sync Polarity
    SerClockActHigh,    // GpakSerClockPol_t     TxClockPolarity3      port 3 Tx Clock Polarity
    SerClockActHigh,    // GpakSerClockPol_t     RxClockPolarity3      port 3 Rx Clock Polarity
    DataDelay1,         // GpakSerDataDelay_t    TxDataDelay3          port 3 Tx data delay
    DataDelay1,         // GpakSerDataDelay_t    RxDataDelay3          port 3 Rx data delay
    Disabled,            // GpakActivation        DxDelay3              port 3 DX Delay
    0x1111,             // unsigned short int    ThirdSlotMask3        port 3 3rd group Slot Mask
    0x1111,             // unsigned short int    FouthSlotMask3        port 3 4th group Slot Mask
    0x1111,             // unsigned short int    FifthSlotMask3        port 3 5th group Slot Mask
    0x1111,             // unsigned short int    SixthSlotMask3        port 3 6th group Slot Mask
    0x1111,             // unsigned short int    SevenSlotMask3        port 3 7th group Slot Mask
    0x1111              // unsigned short int    EightSlotMask3        port 3 8th group Slot Mask

};


static void r1t1_card_dsp_show_portconfig(GpakPortConfig_t PortConfig)
{
    if (debug & DEBUG_DSP) {
        printk("%x = %s\n",PortConfig.SlotsSelect1,"SlotsSelect1");
        printk("%x = %s\n",PortConfig.FirstBlockNum1,"FirstBlockNum1");
        printk("%x = %s\n",PortConfig.FirstSlotMask1,"FirstSlotMask1");
        printk("%x = %s\n",PortConfig.SecBlockNum1,"SecBlockNum1");
        printk("%x = %s\n",PortConfig.SecSlotMask1,"SecSlotMask1");
        printk("%x = %s\n",PortConfig.SerialWordSize1,"SerialWordSize1");
        printk("%x = %s\n",PortConfig.CompandingMode1,"CompandingMode1");
        printk("%x = %s\n",PortConfig.TxFrameSyncPolarity1,"TxFrameSyncPolarity1");
        printk("%x = %s\n",PortConfig.RxFrameSyncPolarity1,"RxFrameSyncPolarity1");
        printk("%x = %s\n",PortConfig.TxClockPolarity1,"TxClockPolarity1");
        printk("%x = %s\n",PortConfig.RxClockPolarity1,"RxClockPolarity1");
        printk("%x = %s\n",PortConfig.TxDataDelay1,"TxDataDelay1");
        printk("%x = %s\n",PortConfig.RxDataDelay1,"RxDataDelay1");
        printk("%x = %s\n",PortConfig.DxDelay1,"DxDelay1");

        printk("%x = %s\n",PortConfig.ThirdSlotMask1,"ThirdSlotMask1");
        printk("%x = %s\n",PortConfig.FouthSlotMask1,"FouthSlotMask1");
        printk("%x = %s\n",PortConfig.FifthSlotMask1,"FifthSlotMask1");
        printk("%x = %s\n",PortConfig.SixthSlotMask1,"SixthSlotMask1");
        printk("%x = %s\n",PortConfig.SevenSlotMask1,"SevenSlotMask1");
        printk("%x = %s\n",PortConfig.EightSlotMask1,"EightSlotMask1");

        printk("%x = %s\n",PortConfig.SlotsSelect2,"SlotsSelect2");
        printk("%x = %s\n",PortConfig.FirstBlockNum2,"FirstBlockNum2");
        printk("%x = %s\n",PortConfig.FirstSlotMask2,"FirstSlotMask2");
        printk("%x = %s\n",PortConfig.SecBlockNum2,"SecBlockNum2");
        printk("%x = %s\n",PortConfig.SecSlotMask2,"SecSlotMask2");
        printk("%x = %s\n",PortConfig.SerialWordSize2,"SerialWordSize2");
        printk("%x = %s\n",PortConfig.CompandingMode2,"CompandingMode2");
        printk("%x = %s\n",PortConfig.TxFrameSyncPolarity2,"TxFrameSyncPolarity2");
        printk("%x = %s\n",PortConfig.RxFrameSyncPolarity2,"RxFrameSyncPolarity2");
        printk("%x = %s\n",PortConfig.TxClockPolarity2,"TxClockPolarity2");
        printk("%x = %s\n",PortConfig.RxClockPolarity2,"RxClockPolarity2");
        printk("%x = %s\n",PortConfig.TxDataDelay2,"TxDataDelay2");
        printk("%x = %s\n",PortConfig.RxDataDelay2,"RxDataDelay2");
        printk("%x = %s\n",PortConfig.DxDelay2,"DxDelay2");

        printk("%x = %s\n",PortConfig.ThirdSlotMask2,"ThirdSlotMask2");
        printk("%x = %s\n",PortConfig.FouthSlotMask2,"FouthSlotMask2");
        printk("%x = %s\n",PortConfig.FifthSlotMask2,"FifthSlotMask2");
        printk("%x = %s\n",PortConfig.SixthSlotMask2,"SixthSlotMask2");
        printk("%x = %s\n",PortConfig.SevenSlotMask2,"SevenSlotMask2");
        printk("%x = %s\n",PortConfig.EightSlotMask2,"EightSlotMask2");

        printk("%x = %s\n",PortConfig.SlotsSelect3,"SlotsSelect3");
        printk("%x = %s\n",PortConfig.FirstBlockNum3,"FirstBlockNum3");
        printk("%x = %s\n",PortConfig.FirstSlotMask3,"FirstSlotMask3");
        printk("%x = %s\n",PortConfig.SecBlockNum3,"SecBlockNum3");
        printk("%x = %s\n",PortConfig.SecSlotMask3,"SecSlotMask3");
        printk("%x = %s\n",PortConfig.SerialWordSize3,"SerialWordSize3");
        printk("%x = %s\n",PortConfig.CompandingMode3,"CompandingMode3");
        printk("%x = %s\n",PortConfig.TxFrameSyncPolarity3,"TxFrameSyncPolarity3");
        printk("%x = %s\n",PortConfig.RxFrameSyncPolarity3,"RxFrameSyncPolarity3");
        printk("%x = %s\n",PortConfig.TxClockPolarity3,"TxClockPolarity3");
        printk("%x = %s\n",PortConfig.RxClockPolarity3,"RxClockPolarity3");
        printk("%x = %s\n",PortConfig.TxDataDelay3,"TxDataDelay3");
        printk("%x = %s\n",PortConfig.RxDataDelay3,"RxDataDelay3");
        printk("%x = %s\n",PortConfig.DxDelay3,"DxDelay3");

        printk("%x = %s\n",PortConfig.ThirdSlotMask3,"ThirdSlotMask3");
        printk("%x = %s\n",PortConfig.FouthSlotMask3,"FouthSlotMask3");
        printk("%x = %s\n",PortConfig.FifthSlotMask3,"FifthSlotMask3");
        printk("%x = %s\n",PortConfig.SixthSlotMask3,"SixthSlotMask3");
        printk("%x = %s\n",PortConfig.SevenSlotMask3,"SevenSlotMask3");
        printk("%x = %s\n",PortConfig.EightSlotMask3,"EightSlotMask3");

    }
    return;
}

static int __devinit r1t1_span_dsp_configureports(struct r1t1 *rh, GpakPortConfig_t PortConfig)
{
    gpakConfigPortStatus_t cp_res;
    GPAK_PortConfigStat_t cp_error;
    unsigned short int DspId;

    r1t1_card_dsp_show_portconfig(PortConfig);

    r1t1_card_select_dsp(rh);
    DspId = rh->num;

    if ((cp_res = gpakConfigurePorts(rh , DspId, &PortConfig, &cp_error)))
        printk("R1T1: %d DSP %d: G168 DSP Port Config failed res = %d error = %d\n", rh->num+1, 1,cp_res, cp_error);
    else if (debug & DEBUG_DSP) {
        printk("R1T1: %d DSP %d: G168 DSP Port Config success %d\n", rh->num+1, 1, cp_res);
    }

    r1t1_card_unselect_dsp(rh);

    if (cp_res)
        return -1;
    else
        return 0;
}

static GpakChannelConfig_t Gpak_chan_config = {

    SerialPort2,    // GpakSerialPort_t    PCM Input Serial Port A Id
    0,              // unsigned short int  PCM Input Time Slot
    SerialPort3,    // GpakSerialPort_t    PCM Output Serial Port A Id
    0,              // unsigned short int  PCM Output Time Slot
    SerialPort3,    // GpakSerialPort_t    PCM Input Serial Port B Id
    0,              // unsigned short int  PCM Input Time Slot
// Should be Null !!!
    SerialPortNull,    // GpakSerialPort_t    PCM Output Serial Port B Id
//    SerialPort2,    // GpakSerialPort_t    PCM Output Serial Port B Id
    0,              // unsigned short int  PCM Output Time Slot
    Null_tone,      // GpakToneTypes       ToneTypesA A side Tone Detect Types
    Null_tone,      // GpakToneTypes       ToneTypesB B side Tone Detect Types
    Disabled,       // GpakActivation      Echo Cancel A Enabled
    Disabled,       // GpakActivation      Echo Cancel B Enabled

    {   1024, // short int  Echo Can Num Taps (tail length) 64 = 512 32 = 256
        3,    // short int  Echo Can NLP Type
        1,    // short int  Echo Can Adapt Enable flag
        1,    // short int  Echo Can G165 Detect Enable flag
        4,    // short int  Echo Can Double Talk threshold
        21,   // short int  Echo Can NLP threshold
        17,   // short int  Dynamic NLP control, NLP limit when EC about to converged
        12,   // short int  Dynamic NLP control, NLP limit when EC not converged yet
        0,    // short int  suppression level for NLP_SUPP mode
        50,   // short int  Echo Can CNG Noise threshold
        40,   // short int  Echo Can Max Adapts per frame
        20,   // short int  Echo Can Cross Correlation limit
        3,    // short int  Echo Can Num FIR Segments
        64    // short int  Echo Can FIR Segment Length
    },

    {   1024, // short int  Echo Can Num Taps (tail length)
        3,    // short int  Echo Can NLP Type
        1,    // short int  Echo Can Adapt Enable flag
        1,    // short int  Echo Can G165 Detect Enable flag
        4,    // short int  Echo Can Double Talk threshold
        21,   // short int  Echo Can NLP threshold
        17,   // short int  Dynamic NLP control, NLP limit when EC about to converged
        12,   // short int  Dynamic NLP control, NLP limit when EC not converged yet
        0,    // short int  suppression level for NLP_SUPP mode
        50,   // short int  Echo Can CNG Noise threshold
        40,   // short int  Echo Can Max Adapts per frame
        20,   // short int  Echo Can Cross Correlation limit
        3,    // short int  Echo Can Num FIR Segments
        64    // short int  Echo Can FIR Segment Length
    },

    cmpNone,         // GpakCompandModes    software companding
//    rate10ms,         // GpakRate_t          FrameRate;          // Gpak Frame Rate
    rate2ms,         // GpakRate_t          FrameRate;          // Gpak Frame Rate
	Disabled,
	Disabled,
	Disabled,
	Disabled

};

static void r1t1_card_dsp_show_chanconfig(GpakChannelConfig_t ChanConfig)
{
    if (debug & DEBUG_DSP) {
        printk("%d = %s\n",ChanConfig.PcmInPortA,"PcmInPortA");
        printk("%d = %s\n",ChanConfig.PcmInSlotA,"PcmInSlotA");
        printk("%d = %s\n",ChanConfig.PcmOutPortA,"PcmOutPortA");
        printk("%d = %s\n",ChanConfig.PcmOutSlotA,"PcmOutSlotA");
        printk("%d = %s\n",ChanConfig.PcmInPortB,"PcmInPortB");
        printk("%d = %s\n",ChanConfig.PcmInSlotB,"PcmInSlotB");
        printk("%d = %s\n",ChanConfig.PcmOutPortB,"PcmOutPortB");
        printk("%d = %s\n",ChanConfig.PcmOutSlotB,"PcmOutSlotB");

        printk("%d = %s\n",ChanConfig.ToneTypesA,"ToneTypesA");
        printk("%d = %s\n",ChanConfig.ToneTypesB,"ToneTypesB");

        printk("%d = %s\n",ChanConfig.EcanEnableA,"EcanEnableA");
        printk("%d = %s\n",ChanConfig.EcanEnableB,"EcanEnableB");

        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanTapLength,"EcanParametersA.EcanTapLength");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanNlpType,"EcanParametersA.EcanNlpType");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanAdaptEnable,"EcanParametersA.EcanAdaptEnable");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanG165DetEnable,"EcanParametersA.EcanG165DetEnable");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanDblTalkThresh,"EcanParametersA.EcanDblTalkThresh");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanNlpThreshold,"EcanParametersA.EcanNlpThreshold");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanNlpConv,"EcanParametersA.EcanNlpConv");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanNlpUnConv,"EcanParametersA.EcanNlpUnConv");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanNlpMaxSuppress,"EcanParametersA.EcanNlpMaxSuppress");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanCngThreshold,"EcanParametersA.EcanCngThreshold");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanAdaptLimit,"EcanParametersA.EcanAdaptLimit");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanCrossCorrLimit,"EcanParametersA.EcanCrossCorrLimit");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanNumFirSegments,"EcanParametersA.EcanNumFirSegments");
        printk("%d = %s\n",ChanConfig.EcanParametersA.EcanFirSegmentLen,"EcanParametersA.EcanFirSegmentLen");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanTapLength,"EcanParametersB.EcanTapLength");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanNlpType,"EcanParametersB.EcanNlpType");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanAdaptEnable,"EcanParametersB.EcanAdaptEnable");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanG165DetEnable,"EcanParametersB.EcanG165DetEnable");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanDblTalkThresh,"EcanParametersB.EcanDblTalkThresh");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanNlpThreshold,"EcanParametersB.EcanNlpThreshold");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanNlpConv,"EcanParametersB.EcanNlpConv");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanNlpUnConv,"EcanParametersB.EcanNlpUnConv");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanNlpMaxSuppress,"EcanParametersB.EcanNlpMaxSuppress");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanCngThreshold,"EcanParametersB.EcanCngThreshold");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanAdaptLimit,"EcanParametersB.EcanAdaptLimit");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanCrossCorrLimit,"EcanParametersB.EcanCrossCorrLimit");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanNumFirSegments,"EcanParametersB.EcanNumFirSegments");
        printk("%d = %s\n",ChanConfig.EcanParametersB.EcanFirSegmentLen,"EcanParametersB.EcanFirSegmentLen");

        printk("%d = %s\n",ChanConfig.SoftwareCompand,"SoftwareCompand");

        printk("%d = %s\n",ChanConfig.FrameRate,"FrameRate");

        printk("%d = %s\n",ChanConfig.MuteToneA,"MuteToneA");
        printk("%d = %s\n",ChanConfig.MuteToneB,"MuteToneB");
        printk("%d = %s\n",ChanConfig.FaxCngDetA,"FaxCngDetA");
        printk("%d = %s\n",ChanConfig.FaxCngDetB,"FaxCngDetB");

    }
    return;
}

static int __devinit r1t1_span_dsp_configurechannel(struct r1t1 *rh, GpakChannelConfig_t ChanConfig,int chan_num)
{
    GPAK_ChannelConfigStat_t chan_config_err;
    gpakConfigChanStatus_t chan_conf_stat;
    unsigned short int DspId;

    r1t1_card_dsp_show_chanconfig(ChanConfig);

    r1t1_card_select_dsp(rh);
    DspId = rh->num;

    if ((chan_conf_stat = gpakConfigureChannel(rh, DspId, chan_num, tdmToTdm, &Gpak_chan_config, &chan_config_err)))
        printk("R1T1: %d DSP %d: Chan %d G168 DSP Chan Config failed error = %d  %d\n", rh->num+1, 1, chan_num, chan_config_err, chan_conf_stat);
    else if (debug & DEBUG_DSP) {
        printk("R1T1: %d DSP %d: G168 DSP Chan %d Config success %d\n", rh->num+1,  1, chan_num, chan_conf_stat);
    }

    r1t1_card_unselect_dsp(rh);

    if (chan_conf_stat)
        return -1;
    else
        return 0;
}

static void r1t1_card_dsp_framestats(struct r1t1 *rh)
{
    gpakReadFramingStatsStatus_t framing_status_status;
    unsigned short int ec1, ec2, ec3, dmaec, slips[6];
    unsigned short int DspId;

    r1t1_card_select_dsp(rh);
    DspId = rh->num;

    if (debug & DEBUG_DSP) {
        framing_status_status = gpakReadFramingStats(rh, DspId, &ec1, &ec2, &ec3, &dmaec, &slips[0]);
        if (framing_status_status == RfsSuccess) {
            printk("R1T1: %d DSP %d: G168 DSP Framing Status Sucess %d\n", rh->num+1, 1, framing_status_status);
            if (ec1+ec2+ec3+dmaec+slips[0]+slips[1]+slips[2]+slips[3]+slips[4]+slips[5]) {
                printk("R1T1: %d DSP %d: G168 DSP Framing Status p1 %2d p2 %2d p3 %2d stop %2d\n", rh->num+1, 1, ec1, ec2, ec3, dmaec);
                printk("R1T1: %d DSP %d: G168 DSP Framing Status slip0 %2d slip1 %2d slip2 %2d slip3 %2d slip4 %2d slip5 %2d\n", 1, 1,
                slips[0], slips[1], slips[2], slips[3], slips[4], slips[5]);
            }
            else
                printk("R1T1: %d DSP %d: G168 DSP Framing Status GOOD!!\n", rh->num+1, 1);
        }
        else {
            printk("R1T1: %d DSP %d: G168 DSP Framing Status Failed %d\n", rh->num+1, 1, framing_status_status);
            printk("R1T1: %d DSP %d: G168 DSP Framing Status %d %d %d %d %d\n", rh->num+1, 1, ec1, ec2, ec3, dmaec, slips[0]);
        }
    }

    r1t1_card_unselect_dsp(rh);
    return;
}

static void r1t1_card_dsp_reetframestats(struct r1t1 *rh)
{
    gpakResetFramingStatsStatus_t framing_reset_status;
    unsigned short int DspId;

    r1t1_card_select_dsp(rh);
    DspId = rh->num;

    if ((framing_reset_status = gpakResetFramingStats(rh, DspId)))
        printk("R1T1: %d DSP %d: G168 DSP Reset Framing Stats Failed %d\n", rh->num+1, 1, framing_reset_status);
    else
        printk("R1T1: %d DSP %d: G168 DSP Reset Framing Stats Sucess %d\n", rh->num+1, 1, framing_reset_status);

    r1t1_card_unselect_dsp(rh);
    return;
}

static void r1t1_card_dsp_cpustats(struct r1t1 *rh)
{
    gpakReadCpuUsageStat_t cpu_status_status;
    unsigned short int pPeakUsage, pPrev1SecPeakUsage;
    unsigned short int DspId;

    r1t1_card_select_dsp(rh);
    DspId = rh->num;

    cpu_status_status = gpakReadCpuUsage(rh, DspId, &pPeakUsage, &pPrev1SecPeakUsage);
    if (cpu_status_status)
        printk("R1T1: %d DSP %d: G168 DSP CPU Status Failed %d\n", rh->num+1, 1, cpu_status_status);
    else
        printk("R1T1: %d DSP %d: G168 DSP CPU Status peek %2d  1 S %2d\n", rh->num+1, 1, pPeakUsage, pPrev1SecPeakUsage);


    r1t1_card_unselect_dsp(rh);
    return;
}

static void r1t1_chan_ec_enable(struct r1t1 *rh, int chan_num)
{
    gpakAlgControlStat_t a_c_stat;
    GPAK_AlgControlStat_t a_c_err;
    unsigned short int DspId;
    unsigned int mask, slot_num;

    if (ec_disable & (1 << chan_num)) {
        if (debug & DEBUG_DSP)
            printk("r1t1 %d: Echo Can NOT enable DSP EC Chan %d\n",rh->num+1, chan_num);
        return;
    }

    DspId = rh->num;

    if (rh->ise1)
        slot_num = chanmap_e1[chan_num];
    else
        slot_num = chanmap_t1[chan_num];

//    printk("chan %x slot %x\n", chan_num, slot_num);

    if (debug & DEBUG_DSP)
        printk("R1T1: %d: Echo Can enable DSP %d EC Chan %d\n",rh->num+1, 1, chan_num);

    r1t1_card_select_dsp(rh);

//    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num-1, ResetEcanB, &a_c_err)))
//        printk("r1t1 %d: G168 DSP Reset Alg Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);


    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableEcanB, &a_c_err))) {
        if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableEcanB, &a_c_err))) {
            if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableEcanB, &a_c_err))) {
                printk("R1T1: %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);
            }
        }
    }
/*
    msleep(1);

    if (rh->ise1) {
        if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableALawSwCompanding, &a_c_err))) {
            if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableALawSwCompanding, &a_c_err))) {
                if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableALawSwCompanding, &a_c_err))) {
                    printk("R1T1: %d: G168 DSP Enable Compand Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);
                }
            }
        }
        printk("r1t1 %d chan %d A law (E1)\n",rh->num+1, chan_num);
    }else {
        if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableMuLawSwCompanding, &a_c_err))) {
            if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableMuLawSwCompanding, &a_c_err))) {
                if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, EnableMuLawSwCompanding, &a_c_err))) {
                    printk("R1T1: %d: G168 DSP Enable Compand Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);
                }
            }
        }
        printk("r1t1 %d chan %d U law (T1)\n",rh->num+1, chan_num);
    }
*/

//    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, ResetEcanB, &a_c_err)))
//        printk("R1T1: %d: G168 DSP Reset Alg Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);

//    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num-1, EnableEcanA, &a_c_err)))
//        printk("r1t1 %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);

    mask = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_ECA1);
    mask |= (1 << slot_num);
//    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECA1, mask);

    r1t1_card_unselect_dsp(rh);

//    printk("Mask %x\n",mask);

    return;
}

static void r1t1_chan_ec_disable(struct r1t1 *rh, int chan_num)
{
    gpakAlgControlStat_t a_c_stat;
    GPAK_AlgControlStat_t a_c_err;
    unsigned short int DspId;
    unsigned int mask, slot_num;

    DspId = rh->num;

    if (rh->ise1)
        slot_num = chanmap_e1[chan_num];
    else
        slot_num = chanmap_t1[chan_num];

//    printk("chan %x slot %x\n", chan_num, slot_num);

    if (debug & DEBUG_DSP)
        printk("R1T1: %d: Echo Can disable DSP %d EC Chan %d\n",rh->num+1, 1, chan_num);

    r1t1_card_select_dsp(rh);

//    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num-1, BypassEcanA, &a_c_err)))
//        printk("r1t1 %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);
    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, BypassEcanB, &a_c_err))) {
        if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, BypassEcanB, &a_c_err))) {
            if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, BypassEcanB, &a_c_err))) {
                printk("R1T1: %d: G168 DSP Disable Alg Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);
            }
        }
    }
/*
    msleep(1);

    if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, BypassSwCompanding, &a_c_err))) {
        if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, BypassSwCompanding, &a_c_err))) {
            if ((a_c_stat = gpakAlgControl(rh, DspId, chan_num, BypassSwCompanding, &a_c_err))) {
                printk("R1T1: %d: G168 DSP Disable Compand Control failed res = %d error = %d\n", rh->num+1, a_c_stat, a_c_err);
            }
        }
    } else
        printk("R1T1: %d: chan %d G168 DSP Disable Compand Control\n", rh->num+1, chan_num);

*/
    mask = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_ECA1);
    mask &= ~(1 << slot_num);
//    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECA1, mask);

    r1t1_card_unselect_dsp(rh);

//    printk("Mask %x\n",mask);

    return;
}


static int r1t1_zap_chan_echocan(struct zt_chan *zap_chan, int eclen)
{
    struct r1t1 *rh = zap_chan->pvt;
    int chan_num;

    chan_num = zap_chan->chanpos-1;

    if (debug & DEBUG_DSP) {
        printk("R1T1: %d Echo Can control Span %d Chan %d zt_chan %d\n",rh->num+1, 1, chan_num, zap_chan->channo);
        printk("DSP up %x\n",rh->dsp_up);
    }
    if (rh->dsp_up == 1) {
        if (eclen)
            rh->nextec |= (1 << chan_num);
        else
            rh->nextec &= ~(1 << chan_num);

        queue_work(rh->wq, &rh->work);
    }
    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void echocan_bh(void *data)
{
	struct r1t1 *rh = data;
#else
static void echocan_bh(struct work_struct *data)
{
	struct r1t1 *rh = container_of(data, struct r1t1, work);
#endif
    unsigned int todo, chan_num;

    todo = rh->nextec ^ rh->currec;
    if (debug & DEBUG_DSP) {
        printk("R1T1: %d Echo Can control bh change %x to %x\n",rh->num+1, todo, (rh->nextec & todo));
        printk("nextec %x currec %x\n",rh->nextec,rh->currec);
        printk("span.channels = %d\n",rh->span.channels);
    }
    for (chan_num=0; chan_num < rh->span.channels; chan_num++) {
        if (todo & (1 << chan_num)) {
            if (rh->nextec & (1 << chan_num)) {
                r1t1_chan_ec_enable(rh, chan_num);
                rh->currec |= (1 << chan_num);
                schedule();
            }
            else {
                r1t1_chan_ec_disable(rh, chan_num);
                rh->currec &= ~(1 << chan_num);
                schedule();
            }
        }
    }
}


static int __devinit r1t1_card_init_dsp(struct r1t1 *rh)
{
    int loops = 0;
    __u16 high, low;
    int chan_num, chan_count, slot_num;
    int ifb_z = 4;

    if (debug & DEBUG_DSP)
        printk("R1T1: Reset DSP\n");
    r1t1_card_reset_dsp(rh);
    if (debug & DEBUG_DSP)
        printk("R1T1: Un-Reset DSP\n");

    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECB1, 0x00000000); // use ec b
    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECA1, 0x00000000); // use ec a

    if (no_ec)
        return -1;

    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, (~EC_ON & __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC)) );

    if (r1t1_span_download_dsp(rh)) {
        return -1;
    }

    r1t1_span_run_dsp(rh);
    if (debug & DEBUG_DSP)
        printk("R1T1: %d DSP %d: GO!!\n", rh->num+1, 1);

    while (ifb_z != 0) {
        ifb_z = 0;
        msleep(1);
        r1t1_card_select_dsp(rh);
        high = r1t1_card_dsp_get(rh, DSP_IFBLK_ADDRESS);
        low = r1t1_card_dsp_get(rh, DSP_IFBLK_ADDRESS+1);
        if (debug & DEBUG_DSP)
            printk("R1T1: %d DSP %d: IfBlockPntr %x\n", rh->num+1, 1, ((high << 16) + low));
        if ((high == 0) && (low == 0))
            ifb_z++;
        r1t1_card_unselect_dsp(rh);
        schedule();
        if ((loops++) > 2) {
            printk("R1T1: DSP not responding...is EC installed?\n");
            return -1;
        }
    }

    rh->hpi_fast = 0;

    r1t1_card_dsp_ping(rh);

    if (r1t1_span_dsp_configureports(rh, Gpak_32_chan_port_config))
        return -1;

    Gpak_chan_config.EcanParametersA.EcanNlpType=nlp_type;
    Gpak_chan_config.EcanParametersB.EcanNlpType=nlp_type;

    Gpak_chan_config.EcanParametersA.EcanNlpThreshold = nlp_threshold;
    Gpak_chan_config.EcanParametersB.EcanNlpThreshold = nlp_threshold;

    Gpak_chan_config.EcanParametersA.EcanNlpMaxSuppress = nlp_max_supress;
    Gpak_chan_config.EcanParametersB.EcanNlpMaxSuppress = nlp_max_supress;

//    Gpak_chan_config.EcanEnableA = Enabled;
    Gpak_chan_config.EcanEnableB = Enabled;

/*
    if (rh->ise1)
        Gpak_chan_config.SoftwareCompand = cmpPCMA;
    else
        Gpak_chan_config.SoftwareCompand = cmpPCMU;
*/

    Gpak_chan_config.EcanEnableA = Disabled;
//    Gpak_chan_config.EcanEnableB = Disabled;
    Gpak_chan_config.SoftwareCompand = cmpNone;

    r1t1_card_dsp_ping(rh);
    chan_count = 0;

    for (chan_num=0; chan_num<rh->span.channels; chan_num++) {
        chan_count++;

        if (rh->ise1) {
            unsigned int hpi_c;
            hpi_c = __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC);
            __r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, (hpi_c | XLATE) );
            __r1t1_card_pci_out(rh, TARG_REGS + R1T1_XLATE_EN, 0xffff7fff );
            slot_num = chanmap_e1[chan_num];
            if (chan_num != 15)
                Gpak_chan_config.SoftwareCompand = cmpPCMU;
            else
                Gpak_chan_config.SoftwareCompand = cmpNone;
        }
        else {
            slot_num = chanmap_t1[chan_num];
            Gpak_chan_config.SoftwareCompand = cmpPCMU;
        }

        Gpak_chan_config.PcmInSlotA  = slot_num*4;
        Gpak_chan_config.PcmOutSlotA = slot_num*4;
        Gpak_chan_config.PcmInSlotB  = slot_num*4;
        Gpak_chan_config.PcmOutSlotB = slot_num*4;

        if (r1t1_span_dsp_configurechannel(rh, Gpak_chan_config, chan_num))
            return -1;

        r1t1_chan_ec_disable(rh, chan_num);

    }
    printk("R1T1: %d DSP %d: %d channels configured\n", rh->num+1, 1, chan_count);

    rh->dsp_up = 1 ;
    rh->span.echocan = r1t1_zap_chan_echocan;

    if (debug & DEBUG_DSP) {

        r1t1_card_dsp_cpustats(rh);
        r1t1_card_dsp_framestats(rh);
        r1t1_card_dsp_reetframestats(rh);

        msleep(400);

        r1t1_card_dsp_cpustats(rh);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_cpustats(rh);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
        msleep(400);
        r1t1_card_dsp_framestats(rh);
    }

    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_HPIC, (EC_ON | __r1t1_card_pci_in(rh, TARG_REGS + R1T1_HPIC)) );

//#if 1
// this is selecting the B EC (route around the A)
// SB zero !!!
//    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECB1, 0xffffffff);

//    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECA1, 0);
//#else
// this is selecting the A EC (route around the B)
    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECB1, 0x00000000); // use ec b

    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECA1, ec_sw); // use ec a
//    __r1t1_card_pci_out(rh, TARG_REGS + R1T1_ECA1, 0xffffffff); // use ec a
//#endif

    r1t1_card_dsp_ping(rh);

    rh->wq = create_singlethread_workqueue("r1t1_ec");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK(&rh->work, echocan_bh, rh);
#else
	INIT_WORK(&rh->work, echocan_bh);
#endif


    return(0);
}

#endif // USE_G168_DSP


static int r1t1_hardware_init(struct r1t1 *rh)
{
    /* Setup DMA Addresses */
    /* Start at writedma */
    iowrite32(rh->writedma, rh->ioaddr + R1T1_TXBUFSTART); /* Write start */
    ioread32(rh->ioaddr + R1T1_TXBUFSTART);
    iowrite32(rh->readdma, rh->ioaddr + R1T1_RXBUFSTART); /* Read start */
    ioread32(rh->ioaddr + R1T1_RXBUFSTART);
    iowrite16((ZT_CHUNKSIZE * 32 * 2) >> 2, rh->ioaddr + R1T1_BUFLEN);
    ioread16(rh->ioaddr + R1T1_BUFLEN);

    /* Sanity check also determines e1 or t1 */
    if (r1t1_framer_sanity_check(rh))
        return -1;
    if (rh->ise1) {
        rh->chanmap = chanmap_e1;
    }
    else
        rh->chanmap = chanmap_t1;

    rh->clocktimeout = 100;

    /* Reset the T1 and report */
    r1t1_framer_hard_reset(rh);

    return 0;

}

static int __devinit r1t1_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int res;
    struct r1t1 *rh;
    unsigned int *canary;
    int x;

    if (debug)
        printk("R1T1: init_one debug=%x e1=%d\n", debug, e1);

    if (pci_enable_device(pdev)) {
        printk("R1T1: No Rhino spotted\n");
        res = -EIO;
    } else {
        rh = kmalloc(sizeof(struct r1t1), GFP_KERNEL);
        if (rh) {
            memset(rh, 0x0, sizeof(struct r1t1));
            spin_lock_init(&rh->lock);
            rh->dev = pdev;
            rh->pciaddr = pci_resource_start(pdev, 0);

            /* Find position */
            for (x=0;x<RH_MAX_CARDS;x++) {
                if (!cards[x]) {
                    cards[x] = rh;
                    break;
                }
            }
            if (x >= RH_MAX_CARDS)
                return -1;
            rh->num = x;

            printk("R1T1: pciaddr = %x \n", (__u32)(dma_addr_t)rh->pciaddr);

            if(!request_mem_region(rh->pciaddr, R1T1_SIZE, "R1T1")) {
                printk("R1T1: Unable to claim R1T1 PCI space\n");
                return -ENOMEM;
            }

            printk("R1T1: claimed R1T1 PCI space\n");

            rh->ioaddr = ioremap_nocache((dma_addr_t)rh->pciaddr, R1T1_SIZE);
//            rh->offset = 28;
            rh->membase = rh->ioaddr;
            printk("R1T1: mapped R1T1 PCI space to %x\n", (__u32)(dma_addr_t)rh->ioaddr);

            rh->writechunk =
                /* 32 channels, Double-buffer, Read/Write */
                (unsigned char *)pci_alloc_consistent(pdev, ZT_MAX_CHUNKSIZE * 32 * 2 * 2 + 8, &rh->writedma);
            if (!rh->writechunk) {
                printk("R1T1: Unable to allocate DMA-able memory\n");
                return -ENOMEM;
            }

            /* Read is after the whole write piece (in bytes) */
            rh->readchunk = rh->writechunk + ZT_CHUNKSIZE * 32 * 2;

            /* Same thing...  */
            rh->readdma = rh->writedma + ZT_CHUNKSIZE * 32 * 2;

            /* Initialize Write/Buffers to all blank data */
            memset((void *)rh->writechunk,0x00,ZT_MAX_CHUNKSIZE * 2 * 2 * 32);
            /* Initialize canary */
            canary = (unsigned int *)(rh->readchunk + ZT_CHUNKSIZE * 64 - 4);
            *canary = (CANARY << 16) | (0xffff);

            rh->nextbuf = 0;

            /* Enable bus mastering */
            pci_set_master(pdev);

            /* Keep track of which device we are */
            pci_set_drvdata(pdev, rh);

            /* Disable interrupts on the board before enabling them in Linux */
            __r1t1_set_reg(rh, R1T1_CONTROL / 4, 0x00);

#ifdef ZAP_IRQ_SHARED
            if (request_irq(pdev->irq, r1t1_interrupt, ZAP_IRQ_SHARED, "r1t1", rh)) {
#else
            if (request_irq(pdev->irq, r1t1_interrupt, SA_SHIRQ, "r1t1", rh)) {
#endif
                printk("R1T1: Unable to request IRQ %d\n", pdev->irq);
                kfree(rh);
                return -EIO;
            }

            rh->version = ioread16(rh->ioaddr + R1T1_VERSION);

            printk("R1T1: Version %d\n",rh->version);

            if (rh->version < 36) {
                printk("R1T1: Found HW version less than 36, need to upgrade your hardware! \n");
//                release_region(rh->pciaddr, R1T1_SIZE);
                iounmap(rh->ioaddr);
                release_mem_region(rh->pciaddr, R1T1_SIZE);
                free_irq(pdev->irq, rh);
                kfree(rh);
                return -1;
            }

//            init_waitqueue_head(&rh->regq);

            /* Initialize hardware */
            r1t1_hardware_init(rh);

            /* We now know which version of card we have */
            if (rh->ise1)
                rh->variety = "Rhino R1T1 E1/PRA";
            else
                rh->variety = "Rhino R1T1 T1/PRI";

            /* Misc. software stuff */
            r1t1_software_init(rh);

        #ifdef USE_G168_DSP
            rh->dsp_type = DSP_5510;
            r1t1_card_init_dsp(rh);

        #endif // USE_G168_DSP



            printk("R1T1: Spotted a Rhino: %s version %d. Module Version " R1T1VER "\n", rh->variety, rh->version);
            res = 0;
        } else
        {
            printk("R1T1: No memory available\n");
            res = -ENOMEM;
        }
    }
    return res;
}

static void __devexit r1t1_remove_one(struct pci_dev *pdev)
{
    struct r1t1 *rh = pci_get_drvdata(pdev);
    if (rh) {
#ifdef USE_G168_DSP
        if (rh->dsp_up) {
            flush_workqueue(rh->wq);
            destroy_workqueue(rh->wq);
        }
#endif // USE_G168_DSP

        r1t1_shutdown(&rh->span);

        /* Release span, possibly delayed */
        if (!rh->usecount)
            r1t1_release(rh);
        else
            rh->dead = 1;
    }
}

static struct pci_device_id r1t1_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_RHINO, PCI_DEVICE_R1T1) },
    { 0 }
};

MODULE_DEVICE_TABLE(pci,r1t1_pci_tbl);

static struct pci_driver r1t1_driver = {
    name:   "r1t1",
    probe:  r1t1_init_one,
    remove: __devexit_p(r1t1_remove_one),
    suspend: NULL,
    resume: NULL,
    id_table: r1t1_pci_tbl,
};

static int __init r1t1_init(void)
{
    return pci_register_driver(&r1t1_driver);
}

static void __exit r1t1_cleanup(void)
{
    pci_unregister_driver(&r1t1_driver);
}


module_param(debug, int, 0600);
MODULE_PARM_DESC(debug, "1 for debugging messages");
module_param(e1, int, 0600);
MODULE_PARM_DESC(e1, "1 for E1 mode set from module_param");
module_param(ec_disable, int, 0600);
module_param(ec_sw, int, 0600);
module_param(no_ec, int, 0600);
module_param(nlp_type, int, 0600);
MODULE_PARM_DESC(nlp_type, "0 - off, 1 - mute, 2 - rand, 3 - hoth, 4 - supp");
module_param(nlp_threshold, int, 0600);
module_param(nlp_max_supress, int, 0600);

MODULE_DESCRIPTION("Rhino R1T1 T1-E1-J1 Driver " RHINOPKGVER );
MODULE_AUTHOR("Lee Reeves <lee@rhinoequipment.com>\n\tBob Conklin <bob@rhinoequipment.com>\n\tBryce Chidester <bryce@rhinoequipment.com>");
MODULE_VERSION( RHINOPKGVER );

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(r1t1_init);
module_exit(r1t1_cleanup);
