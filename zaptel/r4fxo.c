/*
 * Rhino R4FXO QUAD FXO Interface Driver
 *
 * Written by Bob Conklin <bob@rhinoequipment.com>
 *
 * ** Based on Digium's TDM400P TDM FXS/FXO **
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
 * * * * * * * * * * * * * NO WARRANTY * * * * * * * * * * * * * * * * *
 *
 * THE PROGRAM IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT
 * LIMITATION, ANY WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT,
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. Each Recipient is
 * solely responsible for determining the appropriateness of using and
 * distributing the Program and assumes all risks associated with its
 * exercise of rights under this Agreement, including but not limited to
 * the risks and costs of program errors, damage to or loss of data,
 * programs or equipment, and unavailability or interruption of operations.
 *
 * * * * * * * * * * * * DISCLAIMER OF LIABILITY * * * * * * * * * * * *
 *
 * NEITHER RECIPIENT NOR ANY CONTRIBUTORS SHALL HAVE ANY LIABILITY FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING WITHOUT LIMITATION LOST PROFITS), HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OR DISTRIBUTION OF THE PROGRAM OR THE EXERCISE OF ANY RIGHTS GRANTED
 * HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES
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
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>

//#include <asm/io.h>
#include <asm/types.h>
#include <asm/mman.h>
#include <asm/io.h>
#include <stdarg.h>
#include <asm/stat.h>
#include <asm/page.h>
#include "r4fxo.h"

static inline void r4fxo_transmitprep(struct r4fxo *rh, unsigned char ints)
{
    volatile unsigned char *writechunk;
    int sample, card, addr;

    if (ints & 0x01)
        /* Write is at interrupt address.  Start writing from normal offset */
        writechunk = rh->writechunk; // 0x00 to 0x3f
    else
        writechunk = rh->writechunk + ZT_CHUNKSIZE * NUM_CARDS;
    zt_transmit(&rh->span);


    addr = 0;
    for (sample=0;sample<ZT_CHUNKSIZE;sample++) {
        for (card=0;card<NUM_CARDS;card++) {
            if (rh->cardflag & (1 << card))

                writechunk[addr] = (rh->chans[card].writechunk[sample]);

            addr++;
        }
    }

#if 0
    for (x=0;x<ZT_CHUNKSIZE;x++) {
        /* Send a sample, as a 32-bit word */
        writechunk[x] = 0;
#ifdef __BIG_ENDIAN
        if (rh->cardflag & (1 << 3))
            writechunk[x] |= (rh->chans[3].writechunk[x]);
        if (rh->cardflag & (1 << 2))
            writechunk[x] |= (rh->chans[2].writechunk[x] << 8);
        if (rh->cardflag & (1 << 1))
            writechunk[x] |= (rh->chans[1].writechunk[x] << 16);
        if (rh->cardflag & (1 << 0))
            writechunk[x] |= (rh->chans[0].writechunk[x] << 24);
#else
        if (rh->cardflag & (1 << 3))
            writechunk[x] |= (rh->chans[3].writechunk[x] << 24);
        if (rh->cardflag & (1 << 2))
            writechunk[x] |= (rh->chans[2].writechunk[x] << 16);
        if (rh->cardflag & (1 << 1))
            writechunk[x] |= (rh->chans[1].writechunk[x] << 8);
        if (rh->cardflag & (1 << 0))
            writechunk[x] |= (rh->chans[0].writechunk[x]);
#endif
    }
#endif
}

#ifdef AUDIO_RINGCHECK
static inline void ring_check(struct r4fxo *rh, int card)
{
    int x;
    short sample;
    if (rh->modtype[card] != MOD_TYPE_FXO)
        return;
    rh->mod[card].fxo.pegtimer += ZT_CHUNKSIZE;
    for (x=0;x<ZT_CHUNKSIZE;x++) {
        /* Look for pegging to indicate ringing */
        sample = ZT_XLAW(rh->chans[card].readchunk[x], (&(rh->chans[card])));
        if ((sample > 10000) && (rh->mod[card].fxo.peg != 1)) {
            if (debug > 1) printk("High peg!\n");
            if ((rh->mod[card].fxo.pegtimer < PEGTIME) && (rh->mod[card].fxo.pegtimer > MINPEGTIME))
                rh->mod[card].fxo.pegcount++;
            rh->mod[card].fxo.pegtimer = 0;
            rh->mod[card].fxo.peg = 1;
        } else if ((sample < -10000) && (rh->mod[card].fxo.peg != -1)) {
            if (debug > 1) printk("Low peg!\n");
            if ((rh->mod[card].fxo.pegtimer < (PEGTIME >> 2)) && (rh->mod[card].fxo.pegtimer > (MINPEGTIME >> 2)))
                rh->mod[card].fxo.pegcount++;
            rh->mod[card].fxo.pegtimer = 0;
            rh->mod[card].fxo.peg = -1;
        }
    }
    if (rh->mod[card].fxo.pegtimer > PEGTIME) {
        /* Reset pegcount if our timer expires */
        rh->mod[card].fxo.pegcount = 0;
    }
    /* Decrement debouncer if appropriate */
    if (rh->mod[card].fxo.ringdebounce)
        rh->mod[card].fxo.ringdebounce--;
    if (!rh->mod[card].fxo.offhook && !rh->mod[card].fxo.ringdebounce) {
        if (!rh->mod[card].fxo.ring && (rh->mod[card].fxo.pegcount > PEGCOUNT)) {
            /* It's ringing */
            if (debug)
                printk("RING on %d/%d!\n", rh->span.spanno, card + 1);
            if (!rh->mod[card].fxo.offhook)
                zt_hooksig(&rh->chans[card], ZT_RXSIG_RING);
            rh->mod[card].fxo.ring = 1;
        }
        if (rh->mod[card].fxo.ring && !rh->mod[card].fxo.pegcount) {
            /* No more ring */
            if (debug)
                printk("NO RING on %d/%d!\n", rh->span.spanno, card + 1);
            zt_hooksig(&rh->chans[card], ZT_RXSIG_OFFHOOK);
            rh->mod[card].fxo.ring = 0;
        }
    }
}
#endif
static inline void r4fxo_receiveprep(struct r4fxo *rh, unsigned char ints)
{
    volatile unsigned char *readchunk;
    int sample, card, addr, x;

    if (ints & 0x01)
        readchunk = rh->readchunk + ZT_CHUNKSIZE * NUM_CARDS;
    else
        /* Read is at interrupt address.  Valid data is available at normal offset */
        readchunk = rh->readchunk;


    addr = 0;
    for (sample=0;sample<ZT_CHUNKSIZE;sample++) {
        for (card=0;card<NUM_CARDS;card++) {
            if (rh->cardflag & (1 << card))

                rh->chans[card].readchunk[sample] = readchunk[addr];

            addr++;
        }
    }


#if 0

    for (x=0;x<ZT_CHUNKSIZE;x++) {
#ifdef __BIG_ENDIAN
        if (rh->cardflag & (1 << 3))
            rh->chans[3].readchunk[x] = (readchunk[x]) & 0xff;
        if (rh->cardflag & (1 << 2))
            rh->chans[2].readchunk[x] = (readchunk[x] >> 8) & 0xff;
        if (rh->cardflag & (1 << 1))
            rh->chans[1].readchunk[x] = (readchunk[x] >> 16) & 0xff;
        if (rh->cardflag & (1 << 0))
            rh->chans[0].readchunk[x] = (readchunk[x] >> 24) & 0xff;
#else
        if (rh->cardflag & (1 << 3))
            rh->chans[3].readchunk[x] = (readchunk[x] >> 24) & 0xff;
        if (rh->cardflag & (1 << 2))
            rh->chans[2].readchunk[x] = (readchunk[x] >> 16) & 0xff;
        if (rh->cardflag & (1 << 1))
            rh->chans[1].readchunk[x] = (readchunk[x] >> 8) & 0xff;
        if (rh->cardflag & (1 << 0))
            rh->chans[0].readchunk[x] = (readchunk[x]) & 0xff;
#endif
    }

#endif

#ifdef AUDIO_RINGCHECK
    for (x=0;x<rh->cards;x++)
        ring_check(rh, x);
#endif
    /* XXX We're wasting 8 taps.  We should get closer :( */
    for (x = 0; x < NUM_CARDS; x++) {
        if (rh->cardflag & (1 << x))
            //MSG("chan %x echostate %x\n", x, rh->chans[x].echostate);
            zt_ec_chunk(&rh->chans[x], rh->chans[x].readchunk, rh->chans[x].writechunk);
    }
    zt_receive(&rh->span);
}

static inline void __r4fxo_setcard(struct r4fxo *rh, int card)
{
//    if (rh->curcard != card) {
        *(volatile __u8*) (rh->memaddr + RH_CSR) = (__u8) (1 << card);
        rh->curcard = card;
//    }
}

static void __r4fxo_setreg(struct r4fxo *rh, int card, unsigned char reg, unsigned char value)
{
    __u32 data = 0x00200000; // write bit, 0 CID
    __r4fxo_setcard(rh, card);
    data |= (((reg & 0x3f) << 8) | value);
    *(volatile __u32*) (rh->memaddr + SPI_WR) = data;
    *(volatile __u32*) (rh->memaddr + SPI_WR) = data;
}

static void r4fxo_setreg(struct r4fxo *rh, int card, unsigned char reg, unsigned char value)
{
    unsigned long flags;
    spin_lock_irqsave(&rh->lock, flags);
    __r4fxo_setreg(rh, card, reg, value);
    spin_unlock_irqrestore(&rh->lock, flags);
}

static unsigned char __r4fxo_getreg(struct r4fxo *rh, int card, unsigned char reg)
{
    unsigned char uschar;
    __u32 data = 0x00600000; // resd bit, zero CID
    __r4fxo_setcard(rh, card);
    data |= ((reg & 0x3f) << 8);
    *(volatile __u32*) (rh->memaddr + SPI_WR) = data;
    *(volatile __u32*) (rh->memaddr + SPI_WR) = data;
    uschar = (unsigned char) (*(volatile __u32*) (rh->memaddr + SPI_RD) & 0xff);
    uschar = (unsigned char) (*(volatile __u32*) (rh->memaddr + SPI_RD) & 0xff);
    return uschar;
}

static unsigned char r4fxo_getreg(struct r4fxo *rh, int card, unsigned char reg)
{
    unsigned long flags;
    unsigned char res;
    spin_lock_irqsave(&rh->lock, flags);
    res = __r4fxo_getreg(rh, card, reg);
    spin_unlock_irqrestore(&rh->lock, flags);
    return res;
}

static inline void r4fxo_voicedaa_check_hook(struct r4fxo *rh, int card)
{
#ifndef AUDIO_RINGCHECK
    unsigned char res;
#endif
    signed char b;

    res = rh->reg0shadow[card];
    res &= ~(RDTP | RDTN | RDT);
    if (rh->mod[card].fxo.offhook) {
        if (res != 0x9)
            r4fxo_setreg(rh, card, DAACON1, 0x9);
    } else {
        if (res != 0x8)
            r4fxo_setreg(rh, card, DAACON1, 0x8);
    }


#ifndef AUDIO_RINGCHECK
    if (!rh->mod[card].fxo.offhook) {

        res = rh->reg0shadow[card];
        if ((res & 0x60) && rh->mod[card].fxo.battery) { // Cannot stop the battery
            rh->mod[card].fxo.ringdebounce += (ZT_CHUNKSIZE * 16);
            if (rh->mod[card].fxo.ringdebounce >= ZT_CHUNKSIZE * 64) {
                if (!rh->mod[card].fxo.wasringing) {
                    rh->mod[card].fxo.wasringing = 1;
                    zt_hooksig(&rh->chans[card], ZT_RXSIG_RING);
                    if (debug)
                        printk("RING on %d/%d!\n", rh->span.spanno, card + 1);
                }
                rh->mod[card].fxo.ringdebounce = ZT_CHUNKSIZE * 64;
            }
        } else {
            rh->mod[card].fxo.ringdebounce -= ZT_CHUNKSIZE * 4;
            if (rh->mod[card].fxo.ringdebounce <= 0) {
                if (rh->mod[card].fxo.wasringing) {
                    rh->mod[card].fxo.wasringing = 0;
                    zt_hooksig(&rh->chans[card], ZT_RXSIG_OFFHOOK);
                    if (debug)
                        printk("NO RING on %d/%d!\n", rh->span.spanno, card + 1);
                }
                rh->mod[card].fxo.ringdebounce = 0;
            }

        }
    }
#endif

    b = rh->reg1shadow[card];

#if 0
    {
        static int count = 0;
        if (!(count++ % 100)) {
            printk("Card %d: Voltage: %d  Debounce %d\n", card + 1,
                   b, rh->mod[card].fxo.battdebounce);
        }
    }
#endif
    if (abs(b) < battthresh) {
        rh->mod[card].fxo.nobatttimer++;
#if 0
        if (rh->mod[card].fxo.battery)
            printk("Battery loss: %d (%d debounce) %d V\n", b, rh->mod[card].fxo.battdebounce, b);
#endif
        if (rh->mod[card].fxo.battery && !rh->mod[card].fxo.battdebounce) {
            if (debug)
                printk("NO BATTERY on %d/%d! %d V\n", rh->span.spanno, card + 1, b);
            rh->mod[card].fxo.battery =  0;
#ifdef  JAPAN
            if ((!rh->ohdebounce) && rh->offhook) {
                zt_hooksig(&rh->chans[card], ZT_RXSIG_ONHOOK);
                if (debug)
                    printk("Signalled On Hook\n");
#ifdef  ZERO_BATT_RING
                rh->onhook++;
#endif
            }
#else
            zt_hooksig(&rh->chans[card], ZT_RXSIG_ONHOOK);
#endif
            rh->mod[card].fxo.battdebounce = battdebounce;
        } else if (!rh->mod[card].fxo.battery)
            rh->mod[card].fxo.battdebounce = battdebounce;
    } else if (abs(b) > battthresh) {
        if (!rh->mod[card].fxo.battery && !rh->mod[card].fxo.battdebounce) {
            if (debug)
                printk("BATTERY on %d/%d (%s)! %d V\n", rh->span.spanno, card + 1,
                    (b < 0) ? "-" : "+", b);
#ifdef  ZERO_BATT_RING
            if (rh->onhook) {
                rh->onhook = 0;
                zt_hooksig(&rh->chans[card], ZT_RXSIG_OFFHOOK);
                if (debug)
                    printk("Signalled Off Hook\n");
            }
#else
            zt_hooksig(&rh->chans[card], ZT_RXSIG_OFFHOOK);
#endif
            rh->mod[card].fxo.battery = 1;
            rh->mod[card].fxo.nobatttimer = 0;
            rh->mod[card].fxo.battdebounce = battdebounce;
        } else if (rh->mod[card].fxo.battery)
            rh->mod[card].fxo.battdebounce = battdebounce;

        if (rh->mod[card].fxo.lastpol >= 0) {
            if (b < 0) {
            rh->mod[card].fxo.lastpol = -1;
            rh->mod[card].fxo.polaritydebounce = POLARITY_DEBOUNCE;
            }
        }
        if (rh->mod[card].fxo.lastpol <= 0) {
            if (b > 0) {
            rh->mod[card].fxo.lastpol = 1;
            rh->mod[card].fxo.polaritydebounce = POLARITY_DEBOUNCE;
            }
        }
    } else {
        /* It's something else... */
        rh->mod[card].fxo.battdebounce = battdebounce;
    }
    if (rh->mod[card].fxo.battdebounce)
        rh->mod[card].fxo.battdebounce--;
    if (rh->mod[card].fxo.polaritydebounce) {
            rh->mod[card].fxo.polaritydebounce--;
        if (rh->mod[card].fxo.polaritydebounce < 1) {
            if (rh->mod[card].fxo.lastpol != rh->mod[card].fxo.polarity) {
                if (debug)
                    printk("%lu Polarity reversed (%d -> %d) on %x %d V\n", jiffies,
                       rh->mod[card].fxo.polarity,
                       rh->mod[card].fxo.lastpol, card, b);
                if (rh->mod[card].fxo.polarity)
                    zt_qevent_lock(&rh->chans[card], ZT_EVENT_POLARITY);
                rh->mod[card].fxo.polarity = rh->mod[card].fxo.lastpol;
            }
        }
    }
}

#ifdef ZAP_IRQ_HANDLER
ZAP_IRQ_HANDLER(r4fxo_interrupt)
#else
static irqreturn_t r4fxo_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
    struct r4fxo *rh = dev_id;
    int x;
    int mode;
    unsigned char res, sgn, val;
//    signed int sum;

    __u8 ints;

    *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) |= (__u8) INT_ACK;
    *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) &= (__u8) ~INT_ACK;
    ints = *(volatile __u8*) (rh->memaddr + R4FXO_INTSTAT) & 0x01;
    rh->intcount++;

    x = rh->intcount & 0x3;
    mode = rh->intcount & 0xc;

    if (rh->cardflag & (1 << x)) {
        switch(mode) {
        case 0:
            // Rest
            break;
        case 4:
            // Read first shadow reg
            res = r4fxo_getreg(rh, x, LSSTAT); // check line side
            if ((res & ARZ) || !(res & FDT))
                break; // poopy 1
            else {
                res = r4fxo_getreg(rh, x, DAACON1); // check constant bits
                if ((res & ALWAYS1) || !(res & ONHM))
                    break; // poopy 2
                else
                    rh->reg0shadow[x] = res;  // accept a new value
            }
            break;
        case 8:
            // Read second shadow reg
            res = r4fxo_getreg(rh, x, LSSTAT); // check line side
            if ((res & ARZ) || !(res & FDT))
                break; // poopy 1
            else {
                res = r4fxo_getreg(rh, x, LINEVOLTS);
                val = res;
                sgn = res & 0x80;
                if (sgn != 0)
                    val = ~val + 1;
                if (val > 0x70)
                    break;
//                if (val < 3)
//                    res = 0;
                rh->reg1shadow[x] = res;
            }
            break;
        case 12:
            // Perform processing
            r4fxo_voicedaa_check_hook(rh, x);
            break;
        }
    }

    r4fxo_receiveprep(rh, ints);
    r4fxo_transmitprep(rh, ints);

    return IRQ_RETVAL(1);
}

static int r4fxo_voicedaa_insane(struct r4fxo *rh, int card)
{
    int res;
    res = r4fxo_getreg(rh, card, CONTROL2);
    if (res != (RXE | HBE)) {
    MSG("insane failed res= %x\n",res);
        return -2;
    }
    res = r4fxo_getreg(rh, card, LSIDSSREV);
    MSG("VoiceDAA System Side Rev: %02x\n", (res & SSREV));
    return 0;
}

static void wait_just_a_bit(int waitjiffies) // should be a standard linux call to protect from rollover
{
    long newjiffies;
    newjiffies = jiffies + waitjiffies;
    while(jiffies < newjiffies);
}

static int r4fxo_init_voicedaa(struct r4fxo *rh, int card, int fast, int manual, int sane)
{
    unsigned char reg16=0, reg26=0, reg30=0, reg31=0;
    long newjiffies;

    rh->modtype[card] = MOD_TYPE_FXO;
    /* Sanity check the ProSLIC */
    if (!sane && r4fxo_voicedaa_insane(rh, card)) {
        return -2;
    }
    /* Software reset */
    r4fxo_setreg(rh, card, CONTROL1, SRST);

    /* Wait just a bit */
    wait_just_a_bit(HZ/10);

    /* Enable PCM, ulaw */
    if (alawoverride)
        r4fxo_setreg(rh, card, COMMODE, PCME);
    else
        r4fxo_setreg(rh, card, COMMODE, (PCME | MULAW));

    /* Set On-hook speed, Ringer impedence, and ringer threshold */
    reg16 |= (fxo_modes[_opermode].ohs << 6);
    reg16 |= (fxo_modes[_opermode].rz << 1);
    reg16 |= (fxo_modes[_opermode].rt);
    r4fxo_setreg(rh, card, INTERNAT1, reg16);

    /* Set DC Termination:
       Tip/Ring voltage adjust, minimum operational current, current limitation */
    reg26 |= (fxo_modes[_opermode].dcv << 6);
    reg26 |= (fxo_modes[_opermode].mini << 4);
    reg26 |= (fxo_modes[_opermode].ilim << 1);
    r4fxo_setreg(rh, card, DCTERM, reg26);

    /* Set AC Impedence */
    reg30 = (fxo_modes[_opermode].acim);
    r4fxo_setreg(rh, card, ACTERM, reg30);

    /* Misc. DAA parameters */
    reg31 = FULL | FOH128 | FILT | LVFD;
    reg31 |= (fxo_modes[_opermode].ohs2 << 3);
    r4fxo_setreg(rh, card, DAACON5, reg31);

    /* Set Transmit/Receive timeslot */                          // card  0  1  2  3
    r4fxo_setreg(rh, card, PCMTXSLO, (((NUM_CARDS-1)-card) * 8)); // dec  24 16 08 00
    r4fxo_setreg(rh, card, PCMTXSHI, 0x00);                      // hex  18 10 08 00
    r4fxo_setreg(rh, card, PCMRXSLO, (((NUM_CARDS-1)-card) * 8));
    r4fxo_setreg(rh, card, PCMRXSHI, 0x00);

    /* Enable ISO-Cap */
    r4fxo_setreg(rh, card, DAACON2, PUP);
    r4fxo_setreg(rh, card, DAACON2, PUP);

    /* Wait 1000ms for ISO-cap to come up */
    newjiffies = jiffies;
    newjiffies += 2 * HZ;
//    newjiffies += 10 * HZ;
    while((jiffies < newjiffies) && (r4fxo_getreg(rh, card, LSIDSSREV) & LSID) != 0x30)
        wait_just_a_bit(HZ/10);

    if ((r4fxo_getreg(rh, card, LSIDSSREV) & LSID) != 0x30) {
        if ((r4fxo_getreg(rh, card, LSIDSSREV) & LSID) != 0x30) {
            if ((r4fxo_getreg(rh, card, LSIDSSREV) & LSID) != 0x30) {
                MSG("VoiceDAA did not bring up ISO link properly!\n");
                return -1;
            }
        }
    }

    if (debug)
        MSG("ISO-Cap is now up, line side: %02x rev %02x\n",r4fxo_getreg(rh, card, LSIDSSREV) >> 4,(r4fxo_getreg(rh, card, LSREV) >> 2) & 0xf);
    /* Enable on-hook line monitor */
    r4fxo_setreg(rh, card, DAACON1, ONHM);

    /* NZ -- crank the tx gain up by 7 dB */
    if (!strcmp(fxo_modes[_opermode].name, "NEWZEALAND")) {
        printk("Adjusting gain\n");
        r4fxo_setreg(rh, card, TXGAIN2, 0x7);
    }

    return 0;

}


static int r4fxo_ioctl(struct zt_chan *chan, unsigned int cmd, unsigned long data)
{
    struct r4fxo_stats stats;
    struct r4fxo_regs regs;
    struct r4fxo_regop regop;
    struct r4fxo_echo_coefs echoregs;
    struct r4fxo *rh = chan->pvt;
    int x;
    switch (cmd) {
    case ZT_ONHOOKTRANSFER:
        return -EINVAL;
        break;
    case ZT_SETPOLARITY:
        return -EINVAL;
        break;
    case R4FXO_GET_STATS:
        stats.tipvolt = (signed char)r4fxo_getreg(rh, chan->chanpos - 1, LINEVOLTS) * 1000;
        stats.ringvolt = (signed char)r4fxo_getreg(rh, chan->chanpos - 1, LINEVOLTS) * 1000;
        stats.batvolt = (signed char)r4fxo_getreg(rh, chan->chanpos - 1, LINEVOLTS) * 1000;
        if (copy_to_user((struct r4fxo_stats *)data, &stats, sizeof(stats)))
            return -EFAULT;
        break;
    case R4FXO_GET_REGS:
        memset(&regs, 0, sizeof(regs));
        for (x=0;x<NUM_FXO_REGS;x++)
            regs.direct[x] = r4fxo_getreg(rh, chan->chanpos - 1, x);
        if (copy_to_user((struct r4fxo_regs *)data, &regs, sizeof(regs)))
            return -EFAULT;
        break;
    case R4FXO_SET_REG:
        if (copy_from_user(&regop, (struct r4fxo_regop *)data, sizeof(regop)))
            return -EFAULT;
        if (regop.indirect)
            return -EINVAL;
        else {
            regop.val &= 0xff;
            printk("Setting direct %d to %04x on %d\n", regop.reg, regop.val, chan->chanpos);
            r4fxo_setreg(rh, chan->chanpos - 1, regop.reg, regop.val);
        }
        break;
    case R4FXO_SET_ECHOTUNE:
        printk("-- Setting echo registers: \n");
        if (copy_from_user(&echoregs, (struct r4fxo_echo_coefs*)data, sizeof(echoregs)))
            return -EFAULT;

            /* Set the ACIM register */
            r4fxo_setreg(rh, chan->chanpos - 1, ACTERM, echoregs.acim);

            /* Set the digital echo canceller registers */
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP1, echoregs.coef1);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP2, echoregs.coef2);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP3, echoregs.coef3);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP4, echoregs.coef4);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP5, echoregs.coef5);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP6, echoregs.coef6);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP7, echoregs.coef7);
            r4fxo_setreg(rh, chan->chanpos - 1, HYBTAP8, echoregs.coef8);

            printk("-- Set echo registers successfully\n");

            break;
        break;
    default:
        return -ENOTTY;
    }
    return 0;

}

static int r4fxo_open(struct zt_chan *chan)
{
    struct r4fxo *rh = chan->pvt;
    if (!(rh->cardflag & (1 << (chan->chanpos - 1))))
        return -ENODEV;
    if (rh->dead)
        return -ENODEV;
    rh->usecount++;
    MSG("Use Count %x\n",rh->usecount);
    try_module_get(THIS_MODULE);
    return 0;
}

static int r4fxo_watchdog(struct zt_span *span, int event)
{
    MSG("Restarting DMA\n");
    r4fxo_restart_dma(span->pvt);
    return 0;
}

static int r4fxo_close(struct zt_chan *chan)
{
    struct r4fxo *rh = chan->pvt;
    rh->usecount--;
    MSG("Use Count %x\n",rh->usecount);
    module_put(THIS_MODULE);
    /* If we're dead, release us now */
    if (!rh->usecount && rh->dead)
        r4fxo_release(rh);
    return 0;
}

static int r4fxo_hooksig(struct zt_chan *chan, zt_txsig_t txsig)
{
    struct r4fxo *rh = chan->pvt;
//    long int x;
        switch(txsig) {
        case ZT_TXSIG_START:
        case ZT_TXSIG_OFFHOOK:
            rh->mod[chan->chanpos - 1].fxo.offhook = 1;
            r4fxo_setreg(rh, chan->chanpos - 1, DAACON1,(r4fxo_getreg(rh, chan->chanpos - 1, DAACON1) | OFFHOOK));
            /*for (x=0;x<100000;x++)
                r4fxo_getreg(rh, chan->chanpos, 5);*/
            break;
        case ZT_TXSIG_ONHOOK:
            rh->mod[chan->chanpos - 1].fxo.offhook = 0;
            r4fxo_setreg(rh, chan->chanpos - 1, DAACON1,(r4fxo_getreg(rh, chan->chanpos - 1, DAACON1) & ~OFFHOOK));
            break;
        default:
            printk("r4fxo: Can't set tx state to %d\n", txsig);
        }
    return 0;
}

static int r4fxo_initialize(struct r4fxo *rh)
{
    int x;

    /* Zapata stuff */
    sprintf(rh->span.name, "R4FXO/%d", rh->pos);
    sprintf(rh->span.desc, "%s Board %d", rh->variety, rh->pos + 1);
    if (alawoverride) {
        printk("ALAW override parameter detected.  Device will be operating in ALAW\n");
        rh->span.deflaw = ZT_LAW_ALAW;
    } else
        rh->span.deflaw = ZT_LAW_MULAW;
    for (x = 0; x < NUM_CARDS; x++) {
        sprintf(rh->chans[x].name, "R4FXO/%d/%d", rh->pos, x);
        rh->chans[x].sigcap = ZT_SIG_FXOKS | ZT_SIG_FXOLS | ZT_SIG_FXOGS | ZT_SIG_SF | ZT_SIG_EM | ZT_SIG_CLEAR;
        rh->chans[x].sigcap |= ZT_SIG_FXSKS | ZT_SIG_FXSLS | ZT_SIG_SF | ZT_SIG_CLEAR;
        rh->chans[x].chanpos = x+1;
        rh->chans[x].pvt = rh;
    }
    rh->span.chans = rh->chans;
    rh->span.channels = NUM_CARDS;
    rh->span.hooksig = r4fxo_hooksig;
    rh->span.open = r4fxo_open;
    rh->span.close = r4fxo_close;
    rh->span.flags = ZT_FLAG_RBS;
    rh->span.ioctl = r4fxo_ioctl;
    rh->span.watchdog = r4fxo_watchdog;
    init_waitqueue_head(&rh->span.maintq);

    rh->span.pvt = rh;
    if (zt_register(&rh->span, 0)) {
        printk("Unable to register span with zaptel\n");
        return -1;
    }
    return 0;
}

static void r4fxo_post_initialize(struct r4fxo *rh)
{
    int x;
    /* Finalize signalling  */
    for (x = 0; x < NUM_CARDS; x++) {
        if (rh->cardflag & (1 << x)) {
            if (rh->modtype[x] == MOD_TYPE_FXO)
                rh->chans[x].sigcap = ZT_SIG_FXSKS | ZT_SIG_FXSLS | ZT_SIG_SF | ZT_SIG_CLEAR;
        }
    }
}

static int r4fxo_hardware_init(struct r4fxo *rh)
{
    /* Hardware stuff */
    __u32 x;
    int res;

    MSG("hardware version %d\n",*(volatile __u16*) (rh->memaddr + R4FXO_VERSION));
    if (*(volatile __u16*) (rh->memaddr + R4FXO_VERSION) < 9) {
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
        MSG("YOU ARE NOT USING THE CORRECT HARDWARE VERSION\n");
    }

    *(volatile __u8*) (rh->memaddr + RH_SPI) = (__u8) BIT_RST | 0xff;  // release reset

    res = __r4fxo_getreg(rh, 0, 0x02);
    if (res != 3) {
        for (x=0;x<90;x++) {
            res = __r4fxo_getreg(rh, 0, 0x02);
            if (res == 3) {
                MSG("SPI read worked in %x attempts\n", x);
                break;
            }
        if (x==89)
            MSG("Timed out reading chip 1 SPI\n");
        }
    } else
        MSG("SPI port previously up\n");

    /* Setup DMA Addresses */
    *(volatile __u32*) (rh->memaddr + R4FXO_TXBUFSTART) = (__u32) rh->writedma;
    *(volatile __u32*) (rh->memaddr + R4FXO_RXBUFSTART) = (__u32) rh->readdma;
    *(volatile __u16*) (rh->memaddr + R4FXO_BUFLEN) = ((__u16) (ZT_CHUNKSIZE * NUM_CARDS * 2) >> 2);
    // 256K * 1/4 * 1/8 = 8K
    for (x = 0; x < NUM_CARDS; x++) {
        int sane=0,ret=0;
#if 1
        if (!(ret = r4fxo_init_voicedaa(rh, x, 0, 0, sane))) {
            rh->cardflag |= (1 << x);
            printk("Module %d: Installed -- AUTO FXO (%s mode)\n",x, fxo_modes[_opermode].name);

        } else
            printk("Module %d: Not installed\n", x);
#endif
    }

    /* Return error if nothing initialized hmm-okay. */
    if (!rh->cardflag && !timingonly)
        return -1;
    return 0;
}

static void r4fxo_restart_dma(struct r4fxo *rh)
{
    *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) &= (__u8) ~BIT_DMAGO;
    *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) |= (__u8) BIT_DMAGO;
}

static void r4fxo_start_dma(struct r4fxo *rh)
{
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(1);
    *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) |= (__u8) BIT_DMAGO;
}


static void r4fxo_stop_dma(struct r4fxo *rh)
{

    *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) &= (__u8) ~BIT_DMAGO;
}


static void r4fxo_cleaner(struct r4fxo *rh, int flags)
{
    MSG("Clenaer rh = %x\n",(__u32)(addr_t) rh);
    if (rh) {
        MSG("rh exists\n");

        if (flags & STOP_DMA)   {
            MSG("STOP_DMA: Stopping DMA and INTS on card\n");
            MSG("rh->memaddr = %x\n", (__u32)(addr_t) rh->memaddr);
            MSG("Control Register = %x\n",*(volatile __u8*) (rh->memaddr + R4FXO_CONTROL));
            *(volatile __u8*) (rh->memaddr + R4FXO_CONTROL) &= (__u8) ~BIT_DMAGO;
            MSG("Control Register = %x\n",*(volatile __u8*) (rh->memaddr + R4FXO_CONTROL));
        }

        if (flags & FREE_DMA)   {
        MSG("FREE_DMA: release pci consitent memory\n");
        MSG("rh->writechunk = %x\n", (__u32)(addr_t) rh->writechunk);
            if (rh->writechunk) {
                MSG("Unmapping PCI consistent memory block\n");
                pci_free_consistent(rh->dev, ZT_MAX_CHUNKSIZE * 2 * 2 * 2 * 4, (void *)rh->writechunk, rh->writedma);
                rh->writechunk = NULL;
                MSG("rh->writechunk = %x\n", (__u32)(addr_t) rh->writechunk);
            }
        }

        if (flags & IOUNMAP) {
            MSG("IOUNMAP: release pci iomem\n");
            MSG("Freeregion = %x\n", rh->freeregion);
        }


        if (flags & ZUNREG) {
            MSG("Trying unregister zaptel\n");
            MSG("rh->usecount = %x\n",rh->usecount);
            if (!rh->usecount) {
                MSG("Unregistering with Zaptel\n");
                zt_unregister(&rh->span);
            } else  {
                MSG("Marking as dead\n");
                rh->dead = 1;
            }
        }
        if (flags & FREE_INT) {
            MSG("Releasing interrupt\n");
        }
        if (flags & RH_KFREE)   {
            MSG("Releasing mem for struct\n");
        }
        if (flags & PCI_FREE)   {
            MSG("Wiping Driver Data\n");
        }
    }
}


static int __devinit r4fxo_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int res, memlen;
    struct r4fxo *rh;
    struct r4fxo_desc *d = (struct r4fxo_desc *)ent->driver_data;
    int x;
    int y;
    static int initd_ifaces=0;

    MSG("__devinit r4fxo_init_one\n");
    MSG("debug = %d, loopcurrent = %d, reversepolarity = %d\n", debug, loopcurrent, reversepolarity);
    MSG("_opermode = %d, opermode = %s, timingonly = %d, lowpower = %d\n", _opermode, opermode, timingonly, lowpower);
    MSG("boostringer = %d, fastringer = %d, battdebounce = %d\n", boostringer, fastringer, battdebounce);
    MSG("battthresh = %d, alawoverride = %d, initd_ifaces = %d\n", battthresh, alawoverride, initd_ifaces);

    if(initd_ifaces){
        MSG("running memset ifaces\n");
        memset((void *)ifaces,0,(sizeof(struct r4fxo *))*RH_MAX_IFACES);
        initd_ifaces=1;
    }
    for (x=0;x<RH_MAX_IFACES;x++)
        if (!ifaces[x]) break;
    if (x >= RH_MAX_IFACES) {
        MSG("Too many interfaces\n"); // had some chickens EI
        return -EIO;
    }

    if (pci_enable_device(pdev)) {
        MSG("No Rhino spotted\n"); // had some cows EI
        res = -EIO;
    } else {
        rh = kmalloc(sizeof(struct r4fxo), GFP_KERNEL);
        if (rh) {
            int cardcount = 0;
            ifaces[x] = rh;
            memset(rh, 0, sizeof(struct r4fxo));
            spin_lock_init(&rh->lock);
            rh->curcard = -1;
            rh->variety = d->name;
            rh->baseaddr =  pci_resource_start(pdev, 0);
            memlen = pci_resource_len(pdev, 0);
            rh->memaddr = ioremap_nocache(rh->baseaddr, memlen);
            MSG("Rhino PCI BAR0 %x IOMem mapped at %x\n", (__u32)(addr_t) rh->baseaddr, (__u32)(addr_t) rh->memaddr);

            rh->freeregion = 1;
            rh->dev = pdev;
            rh->pos = x;
            for (y=0;y<NUM_CARDS;y++)
                rh->flags[y] = d->flags;

            //  chunksize = 8 smplea * 4 channels * 2 buffers * 2 read and write = 0x80 byte block
            rh->writechunk = (unsigned char *)pci_alloc_consistent(pdev, ZT_MAX_CHUNKSIZE * NUM_CARDS * 2 * 2, &rh->writedma);

            if (!rh->writechunk) {
                MSG("Unable to allocate DMA-able memory\n");
                r4fxo_cleaner(rh, RH_KFREE | PCI_FREE | IOUNMAP);
                return -ENOMEM;
            }
            // read starts at write plus 8 samples * 4 channels * 2 buffers later = 0x40 bytes
            rh->readchunk = rh->writechunk + (ZT_MAX_CHUNKSIZE * NUM_CARDS * 2);  // half the total
            rh->readdma = rh->writedma + (ZT_MAX_CHUNKSIZE * NUM_CARDS * 2);                      // in bytes

            MSG("PCI DMA common memory area from %x to %x\n", (__u32)(addr_t)rh->writedma, (__u32)(addr_t)rh->writedma + (ZT_MAX_CHUNKSIZE * 4 * 2 * 2) - 1);
            MSG("Kernel common memory area from %x to %x\n",  (__u32)(addr_t)rh->writechunk, (__u32)(addr_t)rh->writechunk + (__u32)(ZT_MAX_CHUNKSIZE * 4 * 2 * 2) - 1);
            MSG("PCI Master Wrting to %x mapped to Kernel Address %x\n", (__u32)(addr_t)rh->writedma, (__u32)(addr_t) rh->writechunk);
            MSG("PCI Master Reading from %x mapped to Kernel Address %x\n", (__u32)(addr_t)rh->readdma, (__u32)(addr_t) rh->readchunk);

            if (r4fxo_initialize(rh)) {
                MSG("Unable to intialize \n");
                r4fxo_cleaner(rh, RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | FREE_INT);
                return -EIO;  // had some goats EI
            }

            pci_set_master(pdev);
            pci_set_drvdata(pdev, rh);



#ifdef ZAP_IRQ_SHARED
            if (request_irq(pdev->irq, r4fxo_interrupt, ZAP_IRQ_SHARED, "r4fxo", rh)) {
#else
            if (request_irq(pdev->irq, r4fxo_interrupt, SA_SHIRQ, "r4fxo", rh)) {
#endif
                MSG("Unable to request IRQ %d\n", pdev->irq);
                r4fxo_cleaner(rh, RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | FREE_INT);
                return -EIO; // had some pigs EI
            }

            if (r4fxo_hardware_init(rh)) {
                MSG("Unable to intialize hardware\n");
                r4fxo_cleaner(rh, RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | FREE_INT); // STOP_DMA | ZUNREG
                return -EIO; // had some ducks EI
            }

            r4fxo_post_initialize(rh);
            r4fxo_start_dma(rh);

            for (x = 0; x < NUM_CARDS; x++) {
                if (rh->cardflag & (1 << x))
                    cardcount++;
            }
            MSG("Spotted a Rhino: %s (%d modules)\n", rh->variety, cardcount);

            res = 0;

        } else  //  if (!rh)
            res = -ENOMEM;
    }
    return res;
}

static void r4fxo_release(struct r4fxo *rh)
{
    zt_unregister(&rh->span);
    kfree(rh);
    MSG("Released a Rhino\n");
}

static void __devexit r4fxo_remove_one(struct pci_dev *pdev)
{
    struct r4fxo *rh = pci_get_drvdata(pdev);
    if (rh) {
        r4fxo_stop_dma(rh);
        pci_free_consistent(pdev, ZT_MAX_CHUNKSIZE * 2 * 2 * 2 * 4, (void *)rh->writechunk, rh->writedma);
        free_irq(pdev->irq, rh);
        if (!rh->usecount)
            r4fxo_release(rh);
        else
            rh->dead = 1;
    }
}

static struct pci_device_id r4fxo_pci_tbl[] = {
//    vendor                              subvendor               class   driver_data
//                      device                        subdevice      class_mask
    { PCI_VENDOR_RHINO, PCI_DEVICE_R4FXO, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &r4fxo },
//    { PCI_VENDOR_RHINO, PCI_DEVICE_R4FXO, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &r4fxo },
    { 0 }
};

MODULE_DEVICE_TABLE(pci, r4fxo_pci_tbl);

static struct pci_driver r4fxo_driver = {
    name:   "r4fxo",
    probe:  r4fxo_init_one,
    remove: __devexit_p(r4fxo_remove_one),
    suspend: NULL,
    resume: NULL,
    id_table: r4fxo_pci_tbl,
};

static int __init r4fxo_init(void)
{
    int res;
    int x;
    for (x=0;x<(sizeof(fxo_modes) / sizeof(fxo_modes[0])); x++) {
        if (!strcmp(fxo_modes[x].name, opermode))
            break;
    }
    if (x < sizeof(fxo_modes) / sizeof(fxo_modes[0])) {
        _opermode = x;
    } else {
        printk("Invalid/unknown operating mode '%s' specified.  Please choose one of:\n", opermode);
        for (x=0;x<sizeof(fxo_modes) / sizeof(fxo_modes[0]); x++)
            printk("  %s\n", fxo_modes[x].name);
        printk("Note this option is CASE SENSITIVE!\n");
        return -ENODEV;
    }

    res = zap_pci_module(&r4fxo_driver);
    if (res)
        return -ENODEV;
    return 0;
}

static void __exit r4fxo_cleanup(void)
{
    pci_unregister_driver(&r4fxo_driver);
}

module_param(debug, int, 0600);
module_param(loopcurrent, int, 0600);
module_param(reversepolarity, int, 0600);
module_param(_opermode, int, 0600);
module_param(opermode, charp, 0600);
module_param(timingonly, int, 0600);
module_param(lowpower, int, 0600);
module_param(boostringer, int, 0600);
module_param(fastringer, int, 0600);
module_param(battdebounce, int, 0600);
module_param(battthresh, int, 0600);
module_param(alawoverride, int, 0600);

MODULE_DESCRIPTION("Rhino Equipment Quad FXO (R4FXO) Driver v" RHINOPKGVER );
MODULE_AUTHOR("Bob Conklin <bob@rhinoequipment.com>");
MODULE_VERSION( RHINOPKGVER );
MODULE_LICENSE("GPL");

module_init(r4fxo_init);
module_exit(r4fxo_cleanup);
