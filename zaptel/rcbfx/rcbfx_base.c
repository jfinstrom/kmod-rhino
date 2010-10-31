/*
 * Rhino RCB FXO / FXS Interface Driver
 *
 * Written by Bob Conklin <bob@rhinoequipment.com>
 *
 * Based on Digium's TDM400P TDM FXS/FXO
 * and Zapata Telephony's Zaptel Telephony Interface
 *
 * Copyright (C) 2005-2009, Rhino Equipment Corp.
 * Copyright (C) 2001-2005, Digium, Inc.
 * Copyright (C) 2001 Jim Dixon / Zapata Telephony.
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
#define USE_G168_DSP

// <linux>/include
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/delay.h>

#include <asm/types.h>
#include <asm/mman.h>
#include <asm/io.h>
#include <asm/stat.h>
#include <asm/page.h>
//#ifdef HOTPLUG_FIRMWARE
#include <linux/firmware.h>
//#else
//#include "rcbfx.hex"
//#endif

#include "rcbfx.h"
#include "rcbfx_ioctl.h"

#ifdef USE_G168_DSP
#include "GpakCust.h"
#include "GpakApi.h"
#endif


struct rcb_card_desc{
    char *name;
    unsigned int type_idx;
    unsigned int num_chans;
    unsigned int num_slots;
    unsigned int hw_ver_min;
};

#define FLAG_2CHAN_SPI (1 << 0)



//unsigned char fx_digital_milliwatt[8] = { 0x1e, 0x0b, 0x0b, 0x1e, 0x9e, 0x8b, 0x8b, 0x9e };

static struct rcb_card_desc rcb8fxx   =  { "Rhino RCB8FXX"  , FLAG_2CHAN_SPI ,  8,  8, 8};
static struct rcb_card_desc rcb24fxs  =  { "Rhino RCB24FXS" , FLAG_2CHAN_SPI , 24, 32, 8};
static struct rcb_card_desc rcb24fxx  =  { "Rhino RCB24FXX" , FLAG_2CHAN_SPI , 24, 32, 8};
static struct rcb_card_desc rcb24fxo  =  { "Rhino RCB24FXO" , FLAG_2CHAN_SPI , 24, 32, 8};
static struct rcb_card_desc rcb4fxo   =  { "Rhino RCB4FXO"  , FLAG_2CHAN_SPI ,  4,  4, 7};

static struct rcb_card_t *ifaces[RH_MAX_IFACES];

static int show_pointers = 0;
static int debug = 0;
//static int debug = (DEBUG_MAIN | DEBUG_INTS | DEBUG_DSP);
//static int debug = (DEBUG_MAIN | DEBUG_INTS | DEBUG_DSP | DEBUG_SIG);
static int ec_fxo_alg = 0; // B side on
static int ec_fxs_alg = 0xffffff; // A side on
static int use_ec_fxo = 0; // B side from alg
static int use_ec_fxs = 0xffffff; // A side from alg
static int use_ec_zap = -1; // Enable zaptel echo can
static int use_ec = -1; // Switch in DSP's TDM bus
static int force_fw = 0;
static int no_ec = 0;
// Internal results of calculations
static int zt_ec_chanmap=0;
static int fxs_alg_chanmap=0;
static int fxo_alg_chanmap=0;
static int use_fxs_chanmap=0;
static int use_fxo_chanmap=0;
static int lvs[24];
static int reg_val[24];
static int battime = 100;
static int reg_addr = 0;
static int arr_argc = 24;
static int adid_map = 0;
//static int dsp_disable = 0;
//static int show_stats_z = 0;
static int nlp_type = 3;
static int nlp_threshold = 21;
static int nlp_max_supress = 0;

void *mmap(void *, size_t, int, int, int, off_t);
int open(const char *path, int oflags);

static const char *rcbfx_firmware = "rcbfx.fw";


static inline void rcb_card_transmit(struct rcb_card_t *rcb_card, unsigned char ints)
{
    zt_transmit(&rcb_card->zap_span);
}


static inline void rcb_card_receive(struct rcb_card_t *rcb_card, unsigned char ints)
{
    int chan;

    for (chan = 0; chan < rcb_card->num_chans; chan++) {
        if (rcb_card->chanflag & (1 << chan) & zt_ec_chanmap) {
            zt_ec_chunk(&rcb_card->zap_chans[chan], rcb_card->zap_chans[chan].readchunk, rcb_card->zap_chans[chan].writechunk);
        }
    }

    zt_receive(&rcb_card->zap_span);
}

static int rcb_zap_chan_rbsbits(struct zt_chan *zap_chan, int bits)
{

    int channum, high_nib, regnum;
    struct rcb_card_t *rcb_card = zap_chan->pvt;
    unsigned long flags;

    if(debug & DEBUG_SIG)
        printk("rcbfx %d: Zaptel RBS bits Setting bits to %d on channel %s\n", rcb_card->pos+1, bits, zap_chan->name);

    spin_lock_irqsave(&rcb_card->lock, flags);

    channum = zap_chan->chanpos - 1; // Zero base
    high_nib = 4 * (channum & 1);
    regnum = channum >> 1;

    if (rcb_card->rev_pol[channum]) {
        bits ^= 1;
	if(debug & DEBUG_SIG)
            printk("rcbfx %d: Zaptel switching polarity bits to %d on channel %s\n", rcb_card->pos+1, bits, zap_chan->name);
    }
    *(volatile __u8*) (rcb_card->memaddr + RCB_TXSIG0 + regnum) &= (__u8) (~(0xF << high_nib)) ;
    *(volatile __u8*) (rcb_card->memaddr + RCB_TXSIG0 + regnum) |= (__u8) (bits << high_nib) ;
    if(debug & DEBUG_SIG)
        printk("rcbfx %d: txsig0 = %x\n", rcb_card->pos+1, *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIG0));

    spin_unlock_irqrestore(&rcb_card->lock, flags);

    return 0;
}

static void rcb_card_check_sigbits(struct rcb_card_t *rcb_card, int status)
{
    int rxs, regnum;
    int bits, channum;
    int pol;

    if(debug & DEBUG_SIG)
        printk("rcbfx %d: Checking sigbits %x configged %x\n", rcb_card->pos+1, status, rcb_card->chans_configed);

    if (rcb_card->chans_configed) {
        for (regnum=0;regnum<12;regnum++) { // 12 bytes of sig data to check
            if (status & (1 << regnum)) {
                rxs = *(volatile __u8*) (rcb_card->memaddr + RCB_RXSIG0 + regnum);

                if (rcb_card->num_chans > (regnum*2)) {   // make sure the chan exists
                    bits = rxs & 0x0f;
                    channum = regnum*2;
                    pol = (bits & 0x01) ^ ((bits & 0x04) >> 2);
                    if (rcb_card->last_pol[channum] != pol) {
                        rcb_card->last_pol[channum] = pol;
                        if(debug & DEBUG_SIG)
                            printk("rcbfx %d: pol event channel: %x, bits: %x\n", rcb_card->pos+1, regnum*2, (rxs & 0xf));
                        
                        zt_qevent_lock(&rcb_card->zap_chans[channum], ZT_EVENT_POLARITY);

                    }
                    
                    if(debug & DEBUG_SIG)
                        printk("rcbfx %d: zt_rbsbits channel: %x, bits: %x\n", rcb_card->pos+1, regnum*2, (rxs & 0xf));
                    zt_rbsbits(&rcb_card->zap_chans[channum], (bits));
                }
                else
                    if(debug & DEBUG_SIG)
                        printk("rcbfx %d: invalid signaling data recieved channel: %d, bits: %x\n", rcb_card->pos+1, regnum*2, (rxs & 0xf));

                if (rcb_card->num_chans > (regnum*2+1)) {   // make sure the chan exists
                    bits = (rxs & 0xf0) >> 4;
                    channum = regnum*2+1;
                    pol = (bits & 0x01) ^ ((bits & 0x04) >> 2);
                    if (rcb_card->last_pol[channum] != pol) {
                        rcb_card->last_pol[channum] = pol;
                        if(debug & DEBUG_SIG)
                            printk("rcbfx %d: pol event channel: %x, bits: %x\n", rcb_card->pos+1, regnum*2+1, ((rxs & 0xf0) >> 4));
                        
                        zt_qevent_lock(&rcb_card->zap_chans[channum], ZT_EVENT_POLARITY);

                    }


                    if(debug & DEBUG_SIG)
                        printk("rcbfx %d: zt_rbsbits channel: %x, bits: %x\n", rcb_card->pos+1, regnum*2+1, ((rxs & 0xf0) >> 4));
                    zt_rbsbits(&rcb_card->zap_chans[channum], bits);
                }
                else
                    if(debug & DEBUG_SIG)
                        printk("rcbfx %d: invalid signaling data recieved channel: %d, bits: %x\n", rcb_card->pos+1, regnum*2+1, ((rxs & 0xf0) >> 4));
            }
        }
    }
    else
        if(debug & DEBUG_SIG)
            printk("rcbfx %d: Channels not configured for signaling yet !!\n", rcb_card->pos+1);

}

static unsigned short int rcb_card_dsp_ping(struct rcb_card_t *rcb_card);

#ifdef ZAP_IRQ_HANDLER
ZAP_IRQ_HANDLER(rcb_card_interrupt)
#else
static irqreturn_t rcb_card_interrupt(int irq, void *dev_id, struct pt_regs *regs)
#endif
{
    struct rcb_card_t *rcb_card = dev_id;
    int status, upd_state, regnum;
    __u8 ints, regval;

    status = *(volatile __u16*) (rcb_card->memaddr + RCB_RXSIGSTAT); // read flancter

    if (status) {
        if (debug & DEBUG_SIG)
            printk("RXCHANGE flags = %x\n",status);
        *(volatile __u16*) (rcb_card->memaddr + RCB_RXSIGSTAT) = status; // reset flancter
        if (debug & DEBUG_SIG)
            printk("RXCHANGE flags = %x\n",*(volatile __u16*) (rcb_card->memaddr + RCB_RXSIGSTAT));

        rcb_card_check_sigbits(rcb_card, status);
    }

    else if (*(volatile __u8*) (rcb_card->memaddr + RCB_INTSTAT) & 0x02) {
        *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) |= (__u8) INT_ACK; // int acknowledge
        *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) &= (__u8) ~INT_ACK;

        // read DMA address pointer MSB
        ints = *(volatile __u8*) (rcb_card->memaddr + RCB_INTSTAT) & 0x01;

        if ((rcb_card->intcount < 10) && (debug))
            printk("rcbfx %d: INT count %d PTR %x\n", rcb_card->pos+1, rcb_card->intcount, *(volatile __u32*) (rcb_card->memaddr + RCB_TDM_PTR));

        rcb_card->intcount++;
        rcb_card_receive(rcb_card, ints);
        rcb_card_transmit(rcb_card, ints);


        if ((rcb_card->intcount & RCB_LVSSAMP) == 0) {
            for(regnum=0;regnum<24;regnum++) {
                regval = *(volatile __u8*)(rcb_card->memaddr + RCB_LVSTAB + regnum);
                if (regval & 0x80) { // negative
                    regval &= 0x7f;
                    lvs[regnum] = -(0x80 - regval);
                } else
                    lvs[regnum] = regval;
            }
        }

        if (reg_addr != rcb_card->oldreg_addr) {
            if (debug)
                printk("New reg_addr = %x\n",reg_addr);
            rcb_card->oldreg_addr = reg_addr;
            *(volatile __u8*)(rcb_card->memaddr + RCB_REGADDR) = reg_addr;
            rcb_card->read_on_int = rcb_card->intcount + RCB_REGTIME;
        }

        if (rcb_card->intcount == rcb_card->read_on_int) {
            if (debug)
                printk("updating valuesfor reg %x\n",reg_addr);
            for(regnum=0;regnum<24;regnum++) {
                reg_val[regnum] = *(volatile __u8*)(rcb_card->memaddr + RCB_REGTAB + regnum);
            }
        }

    }

    else
        return IRQ_NONE;

    upd_state = (*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x08000));
    if ((!upd_state) && (rcb_card->param_upd_state)) {
        printk("parameters accepted and set\n");
        rcb_card->param_upd_state = 0;
    }
    if ((upd_state) && (!rcb_card->param_upd_state)) {
        printk("parameters loaded to card\n");
        rcb_card->param_upd_state = 1;
    }

    return IRQ_RETVAL(1);
}

static int rcb_zap_chan_ioctl(struct zt_chan *zap_chan, unsigned int cmd, unsigned long data)
{

    int regnum;
    struct rcb_card_params_t rcb_card_params;
    struct rcb_card_t *rcb_card = zap_chan->pvt;
    int num_chans = rcb_card->num_chans;
//    struct zt_hwgain hwgain;
    struct rcb_chan_echo_coefs coefs;
	int x;
    int channum;

    switch (cmd) {
    case ZT_ONHOOKTRANSFER:
        return -EINVAL;
        break;
    case ZT_SETPOLARITY:
		if (get_user(x, (int *)data))
			return -EFAULT;
		
        channum = zap_chan->chanpos - 1;
        if (rcb_card->modtype[channum] != MOD_TYPE_FXS)
			return -EINVAL;
        rcb_card->rev_pol[channum] = x;
        // Change it now !!
        regnum = channum >> 1;
        if (channum & 1) {// odd = high nibble
            *(volatile __u8*) (rcb_card->memaddr + RCB_TXSIG0 + regnum) ^= 0x10;
        }
        else {// even = low nibble
            *(volatile __u8*) (rcb_card->memaddr + RCB_TXSIG0 + regnum) ^= 0x01;
        }

        break;
    case RCB_CHAN_SET_CBPARAMS:
        if (debug)
            printk("rcbfx %d: Setting cbfx parameters: \n", rcb_card->pos+1);
        if (*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x09000)) {
            printk("rcbfx %d: Board not ready for params -- not setting\n", rcb_card->pos+1);
            return -EFAULT;
            break;
        }
        if (copy_from_user(&rcb_card_params, (struct rcb_card_params_t *)data, sizeof(rcb_card_params)))
            return -EFAULT;

        printk("rcbfx %d: Recieved cbfx parameters: \n", rcb_card->pos+1);

        for (regnum = 0; regnum < P_TBL_CNT; regnum++) {
            if (debug)
                printk("rcbfx %d: IO Address: %x, Regnum: %x, Data: %x \n", rcb_card->pos+1, PARAM_TBL + regnum, regnum, (__u8) (rcb_card_params.settings[regnum]));
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + regnum) = (__u8) (rcb_card_params.settings[regnum]) ;
        }
        // notify of parameter update
        *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) = (__u32)(0x08000);
        break;
    case RCB_CHAN_GET_BDINFO:
        if (debug)
            printk("rcbfx %d: ioctl RCB_CHAN_GET_BDINFO sending %d\n", rcb_card->pos+1, num_chans);
        if (copy_to_user((int *)data, &num_chans, sizeof(num_chans)))
            return -EFAULT;
        break;

    case RCB_CHAN_SET_ECHOTUNE:
        if (debug) {
            printk("rcbfx %d: ioctl RCB_CHAN_SET_ECHOTUNE sending\n", rcb_card->pos+1);
            printk("chan %x, ac %x, 1 %x, 2 %x, 3 %x, 4 %x, 5 %x, 6 %x, 7 %x, 8 %x,\n",
                zap_chan->chanpos,coefs.acim,coefs.coef1,coefs.coef2,coefs.coef3,
                coefs.coef4,coefs.coef5,coefs.coef6,coefs.coef7,coefs.coef8);
        }
        if (*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x05000)) {
            printk("rcbfx %d: Board not ready for params -- not setting %x\n", rcb_card->pos+1, *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT));
            return -EFAULT;
            break;
        }

        if (copy_from_user(&coefs, (struct rcb_chan_echo_coefs*)data, sizeof(coefs)))
            return -EFAULT;

        if (rcb_card->modtype[zap_chan->chanpos - 1] == MOD_TYPE_FXO) {
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG) = (__u8) (zap_chan->chanpos);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+1) = (__u8) (coefs.acim);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+2) = (__u8) (coefs.coef1);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+3) = (__u8) (coefs.coef2);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+4) = (__u8) (coefs.coef3);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+5) = (__u8) (coefs.coef4);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+6) = (__u8) (coefs.coef5);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+7) = (__u8) (coefs.coef6);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+8) = (__u8) (coefs.coef7);
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG+9) = (__u8) (coefs.coef8);
        } else {
            return -EINVAL;
        }

        // notify of parameter update
        *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) = (__u32)(0x04000);
        msleep(100);
        break;

#if 0
    case ZT_SET_HWGAIN:
        if (copy_from_user(&hwgain, (struct zt_hwgain*) data, sizeof(hwgain)))
            return -EFAULT;

//      wctdm_set_hwgain(wc, chan->chanpos-1, hwgain.newgain, hwgain.tx);

        if (debug)
            printk("Setting hwgain on channel %d to %d for %s direction\n",
                zap_chan->chanpos-1, hwgain.newgain, hwgain.tx ? "tx" : "rx");
        break;
#endif

    default:
        return -ENOTTY;
        break;
    }
    return 0;
}

static int rcb_card_update_fw(struct rcb_card_t *rcb_card, const struct firmware *firmware)
{
    long end_jiffies;
    unsigned int block_count = 0;
    unsigned int total_blocks, frac_block, char_count, zeros=0;
    unsigned char block_sum;


    total_blocks = (firmware->size - 2) >> 8; // 256 char blocks
    frac_block = (firmware->size - 2) & 0xff;

    printk("rcbfx %d: New Firmware is %xh bytes - loading into 800h to %xh\n", rcb_card->pos+1, (unsigned int)(firmware->size - 2), (unsigned int)(firmware->size - 2) + 0x800);
    printk("Firmware Size is %d blocks plus %d more bytes\n", total_blocks, frac_block);
    if ((firmware->size - 2) > 0x7600) {
        printk("rcbfx %d: Firmware file to large %x GT %x\n", rcb_card->pos+1, (unsigned int)(firmware->size - 2), 0x7600);
        return -2;
    }
    if ((firmware->size - 2) < 0x5000) {
        printk("rcbfx %d: Firmware file too small %x\n", rcb_card->pos+1, (unsigned int)(firmware->size - 2));
        return -2;
    }
    for (char_count = 2; char_count < firmware->size; char_count++) {
        if (firmware->data[char_count] == 0)
            zeros++;
        else
            break;
    }
    if (zeros > 20) {
        printk("rcbfx %d: %d too many leading zeros\n", rcb_card->pos+1, zeros);
        return -2;
    }
//    spin_lock_irqsave(&rcb_card->lock, flags);
    // reset and set boot code to bootloader
    *(volatile __u8*) (rcb_card->memaddr + FW_BOOT) = FW_BOOT_LOAD | FW_BOOT_RST;
    end_jiffies = jiffies + 50; while (end_jiffies > jiffies);
    *(volatile __u8*) (rcb_card->memaddr + FW_BOOT) = FW_BOOT_LOAD; // release reset
    end_jiffies = jiffies + 50; while (end_jiffies > jiffies); // wait for reset
    if (debug)
        printk("rcbfx %d: Starting to send firmware\n", rcb_card->pos+1);
    for (block_count = 1; block_count <= total_blocks; block_count++) {
        schedule();
        // send block
        block_sum = 0;
        for (char_count = 0; char_count < 0x100; char_count++) {
            *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + char_count) = (__u8)(firmware->data[char_count + 2 + ((block_count-1) << 8)]);
            block_sum = (unsigned char)(block_sum + firmware->data[char_count + 2 + ((block_count-1) << 8)]);
        }
        // update block counter
        *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = block_count;
        *(volatile __u8*) (rcb_card->memaddr + FW_SUM) = block_sum;

        end_jiffies = jiffies + 2000;
        // wait for block to take
        while ((*(volatile __u8*) (rcb_card->memaddr + FW_COMIN) != block_count) && (end_jiffies > jiffies));
        if (*(volatile __u8*) (rcb_card->memaddr + FW_COMIN) != block_count) {
            printk("rcbfx %d: Time out!!!! %d blocks transfered %d blocks taken\n", rcb_card->pos+1, block_count, *(volatile __u8*) (rcb_card->memaddr + FW_COMIN));
//            spin_unlock_irqrestore(&rcb_card->lock, flags);
            return -1;
        }
        if (debug)
            printk("rcbfx %d: Acked block %x of %x  ", rcb_card->pos+1, block_count, total_blocks);
    }
    // clear block for checksum purposes
    for (char_count = 0; char_count < 0x100; char_count++)
        *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + char_count) = 0;
    // send the partial block
    block_sum = 0;
    for (char_count = 0; char_count < frac_block; char_count++) {
        *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + char_count) = (__u8)(firmware->data[char_count + 2 + ((block_count-1) << 8)]);
        block_sum = (unsigned char)(block_sum + firmware->data[char_count + 2 + ((block_count-1) << 8)]);
    }
    // update block counter
    *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = block_count;
    *(volatile __u8*) (rcb_card->memaddr + FW_SUM) = block_sum;
    if (debug)
         printk("rcbfx %d: Sent partial block %x\n", rcb_card->pos+1, block_count);

    end_jiffies = jiffies + 2000;
    // wait for block to take
    while ((*(volatile __u8*) (rcb_card->memaddr + FW_COMIN) != block_count) && (end_jiffies > jiffies));
    if (*(volatile __u8*) (rcb_card->memaddr + FW_COMIN) != block_count) {
        printk("rcbfx %d: Time out!!!! %d blocks transfered %d blocks taken\n", rcb_card->pos+1, block_count, *(volatile __u8*) (rcb_card->memaddr + FW_COMIN));
//        spin_unlock_irqrestore(&rcb_card->lock, flags);
        return -1;
    }
    if (debug)
        printk("rcbfx %d: Acked partial block %x\n", rcb_card->pos+1, block_count);
    // say done
    *(volatile __u8*) (rcb_card->memaddr + FW_BOOT) = 0;
    *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = 0xff;
    // wait for it
    end_jiffies = jiffies + 100;
    while ((*(volatile __u8*) (rcb_card->memaddr + FW_COMIN) != 0xff) || (end_jiffies > jiffies));

    if (*(volatile __u8*)(rcb_card->memaddr + FW_COMIN) != 0xff) {
        printk("rcbfx %d: All blocks transfered no ACK\n", rcb_card->pos+1);
        return -1;
    }
    else
        printk("rcbfx %d: All blocks transfered and acked\n", rcb_card->pos+1);

    *(volatile __u8*) (rcb_card->memaddr + FW_DATA) = 0x00;
    *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = 0x00;

//    spin_unlock_irqrestore(&rcb_card->lock, flags);
    return 0;
}

static int rcb_zap_chan_open(struct zt_chan *zap_chan)
{
    struct rcb_card_t *rcb_card = zap_chan->pvt;
//    if (!(rcb_card->chanflag & (1 << (zap_chan->chanpos - 1))))
    if (rcb_card->num_chans < (zap_chan->chanpos - 1))   // make sure the chan exists
        return -ENODEV;
    if (rcb_card->dead)
        return -ENODEV;
    rcb_card->usecount++;
    if(debug)
        printk("rcbfx %d: Use Count %x\n", rcb_card->pos+1, rcb_card->usecount);
    try_module_get(THIS_MODULE);
    return 0;
}

static void rcb_card_restart_dma(struct rcb_card_t *rcb_card)
{
    if (debug)
        printk("rcbfx %d: Restarting DMA\n", rcb_card->pos+1);
    *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) &= (__u8) ~BIT_DMAGO;
    *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) |= (__u8) BIT_DMAGO;
    return;
}

static int rcb_zap_span_watchdog(struct zt_span *zap_span, int event)
{
    rcb_card_restart_dma(zap_span->pvt);
    return 0;
}

static void rcb_card_release(struct rcb_card_t *rcb_card)
{
    *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT) &= (__u8)~(0x00);
    if(debug)
        printk("rcbfx %d: Statout = %x\n", rcb_card->pos+1, *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT));
    zt_unregister(&rcb_card->zap_span);
    if (rcb_card->freeregion)
        release_region(rcb_card->baseaddr, rcb_card->memlen);
    printk("rcbfx %d: Released a Rhino\n", rcb_card->pos+1);
    kfree(rcb_card);
}

static int rcb_zap_chan_close(struct zt_chan *zap_chan)
{
    struct rcb_card_t *rcb_card = zap_chan->pvt;
    rcb_card->usecount--;
    if(debug)
        printk("rcbfx %d: Use Count %d\n", rcb_card->pos+1, rcb_card->usecount);
    module_put(THIS_MODULE);
    /* If we're dead, release us now */
    if (!rcb_card->usecount && rcb_card->dead)
        rcb_card_release(rcb_card);
    return 0;
}

static int rcb_card_initialize(struct rcb_card_t *rcb_card)
{
    int chan;
    /* Zapata stuff */
    sprintf(rcb_card->zap_span.name, "%s/%d",rcb_card->variety ,rcb_card->pos+1);
    sprintf(rcb_card->zap_span.desc, "%s/%d",rcb_card->variety ,rcb_card->pos+1);
//    if (alawoverride) {
//        printk("rcbfx %d: ALAW override parameter detected. Device will be operating in ALAW\n", rcb_card->pos+1);
//        rcb_card->zap_span.deflaw = ZT_LAW_ALAW;
//    } else
        rcb_card->zap_span.deflaw = ZT_LAW_MULAW;
    for (chan = 0; chan < rcb_card->num_chans; chan++) {

        rcb_card->zap_chans[chan].writechunk = (u_char *)(rcb_card->writechunk + (chan * ZT_CHUNKSIZE));
        rcb_card->zap_chans[chan].readchunk = (u_char *)(rcb_card->readchunk + (chan * ZT_CHUNKSIZE));
        if (show_pointers)
            printk("rcbfx %d: Chan %d writechunk %lx readchunk %lx\n",
            rcb_card->pos+1, chan ,(long unsigned int) rcb_card->zap_chans[chan].writechunk, (long unsigned int) rcb_card->zap_chans[chan].readchunk);

        if (rcb_card->chanflag & (1 << chan)) {
            if (rcb_card->modtype[chan] == MOD_TYPE_FXO) {
                sprintf(rcb_card->zap_chans[chan].name, "FXO/%d/%d", rcb_card->pos+1, chan);
                rcb_card->zap_chans[chan].sigcap = ZT_SIG_FXSKS | ZT_SIG_FXSLS | ZT_SIG_SF | ZT_SIG_CLEAR;

            }
            else {
                sprintf(rcb_card->zap_chans[chan].name, "FXS/%d/%d", rcb_card->pos+1, chan);
                rcb_card->zap_chans[chan].sigcap = ZT_SIG_FXOKS | ZT_SIG_FXOLS | ZT_SIG_FXOGS | ZT_SIG_SF | ZT_SIG_EM | ZT_SIG_CLEAR;
#ifdef ZT_SIG_FXONS            
                rcb_card->zap_chans[chan].sigcap |= ZT_SIG_FXONS;
#endif            
            }
        }
        else {
            sprintf(rcb_card->zap_chans[chan].name, "---/%d/%d", rcb_card->pos+1, chan);
            rcb_card->zap_chans[chan].sigcap = ZT_SIG_FXOKS | ZT_SIG_FXOLS | ZT_SIG_FXOGS | ZT_SIG_SF | ZT_SIG_EM | ZT_SIG_CLEAR;
            rcb_card->zap_chans[chan].sigcap |= ZT_SIG_FXSKS | ZT_SIG_FXSLS;
#ifdef ZT_SIG_FXONS            
            rcb_card->zap_chans[chan].sigcap |= ZT_SIG_FXONS;
#endif            
        }
        rcb_card->zap_chans[chan].chanpos = chan+1;
        rcb_card->zap_chans[chan].pvt = rcb_card;
    }

    rcb_card->zap_span.chans = rcb_card->zap_chans;
    rcb_card->zap_span.channels = rcb_card->num_chans;
    rcb_card->zap_span.rbsbits = rcb_zap_chan_rbsbits;
    rcb_card->zap_span.open = rcb_zap_chan_open;
    rcb_card->zap_span.close = rcb_zap_chan_close;
    rcb_card->zap_span.flags = ZT_FLAG_RBS;
    rcb_card->zap_span.ioctl = rcb_zap_chan_ioctl;
    rcb_card->zap_span.watchdog = rcb_zap_span_watchdog;
    init_waitqueue_head(&rcb_card->zap_span.maintq);
    rcb_card->zap_span.pvt = rcb_card;

    if (zt_register(&rcb_card->zap_span, 0)) {
        printk("rcbfx %d: Unable to register span with zaptel\n", rcb_card->pos+1);
        return -1;
    }

    rcb_card->chans_configed = 1;
    return 0;
}

static int rcb_card_hardware_init(struct rcb_card_t *rcb_card)
{
    /* Hardware stuff */
    __u16 bd_pres;
    __u32 is_fxo;
    long end_jiffies;
    int chan, time_out, res, tries=0;
    int fwv_register, fwv_file;
    int done=0;
    static const struct firmware *firmware_rcb;

    *(volatile __u8*) (rcb_card->memaddr + FW_BOOT) = 0x00 | DSP_RST; // reset DSP
    end_jiffies = jiffies + 30000; //
    time_out = 0;
    *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) = (__u32)(0x02000); // ping!!
    if (*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x02000)) {   // check sign of life and move on
        printk("rcbfx %d: Waiting for response from card ......... \n", rcb_card->pos+1);
        while ((*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x02000)) && (time_out == 0)) {
            if (jiffies > end_jiffies )
                time_out = 1;
        }
    }

    *(volatile __u8*) (rcb_card->memaddr + FW_BOOT) = 0x00; // release DSP reset

    fwv_register = *(volatile __u16*) (rcb_card->memaddr + FW_VER);
    printk("rcbfx %d: Firmware Version %x.%x\n", rcb_card->pos+1,(fwv_register & 0xff00)>> 8, (fwv_register & 0xff));

#if 1
    if ((request_firmware(&firmware_rcb, rcbfx_firmware, &rcb_card->dev->dev) != 0) || !firmware_rcb) {
        printk("rcbfx %d: firmware %s not available from userspace\n", rcb_card->pos+1, rcbfx_firmware);
    }
    else {
        fwv_file = ((firmware_rcb->data[0] << 8) | (firmware_rcb->data[1]));
        printk("rcbfx %d: Firmware File Version is %x.%x\n", rcb_card->pos+1,(fwv_file & 0xff00)>> 8, fwv_file & 0xff);
        if ((fwv_file > fwv_register) || (force_fw)) {
            if (force_fw) printk("rcbfx %d: Firmware Upgrade beeing forced\n", rcb_card->pos+1);
            printk(KERN_ALERT "rcbfx %d: Firmware Uprgrade In Progress -- Do Not Interrupt!!\n", rcb_card->pos+1);
            while (!done) {
                res = rcb_card_update_fw(rcb_card, firmware_rcb);
                tries++;
                if ((res==0) || (res==-2))
                    done = 1;
                else if ((res==-1) && (tries > 3))
                    done = 1;
            }
        }

        release_firmware(firmware_rcb);
        *(volatile __u8*) (rcb_card->memaddr + FW_DATA) = 0x00;
        *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = 0x00;
    }
#endif

    *(volatile __u8*)(rcb_card->memaddr + RCB_BATTTIME) = battime;

    printk("rcbfx %d: Hardware version %d\n", rcb_card->pos+1, *(volatile __u16*) (rcb_card->memaddr + RCB_VERSION));
    if (*(volatile __u16*) (rcb_card->memaddr + RCB_VERSION) < rcb_card->hw_ver_min) {
        if(debug)
            printk("rcbfx %d: YOU ARE NOT USING THE LATEST HARDWARE VERSION\n", rcb_card->pos+1);
    }
    if (done) { // updated
        end_jiffies = jiffies + 30000;
        time_out = 0;
        *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) = (__u32)(0x02000); // ping!!
        if (*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x02000)) {   // check sign of life and move on
            printk("Waiting for response from card ......... \n");
            while ((*(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) & (__u32)(0x02000)) && (time_out == 0)) {
                schedule();
                if (jiffies > end_jiffies )
                    time_out = 1;
            }
        }
    }

    if ((time_out == 0) && (debug))
        printk("rcbfx %d: Got response from card\n", rcb_card->pos+1);
    if (time_out == 1) {
        printk("rcbfx %d: Not responding !!!!\n", rcb_card->pos+1);
//        return -1;
    }

    bd_pres = *(volatile __u16*) (rcb_card->memaddr + RCB_CHANPRES);
    is_fxo  = *(volatile __u32*) (rcb_card->memaddr + RCB_CHANTYPE);
    if (debug) {
        printk("rcbfx %d: Channels Present: %x, Channel Type: %x\n", rcb_card->pos+1, bd_pres, is_fxo);
        printk("rcbfx %d: num_chans: %d\n", rcb_card->pos+1, rcb_card->num_chans);
    }

    rcb_card->fxs_chanmap=0;
    rcb_card->fxo_chanmap=0;

    for (chan = 0; chan < rcb_card->num_chans; chan++) {
        if ((1 << (chan >> 1)) & bd_pres) {
            rcb_card->chanflag |= (1 << chan);

            if ((1 << chan) & is_fxo) {
                rcb_card->modtype[chan] = MOD_TYPE_FXO;
                rcb_card->fxo_chanmap |= (1 << chan);
            }
            else {
                rcb_card->modtype[chan] = MOD_TYPE_FXS;
                rcb_card->fxs_chanmap |= (1 << chan);
            }
        }
    }

#ifdef USE_G168_DSP
//    printk("bd_pres %x - is_fxo %x\n",bd_pres,is_fxo);
//    printk("chanflag %x - fxo_chanmap %x - fxs_chanmap %x\n",rcb_card->chanflag,rcb_card->fxo_chanmap,rcb_card->fxs_chanmap);

    // Set the global variables here
    if ((ec_fxo_alg > 0) || (ec_fxo_alg == 0)) // forced value
        fxo_alg_chanmap = ec_fxo_alg;
    else
        fxo_alg_chanmap = rcb_card->fxo_chanmap; // On in B diraction

    if ((ec_fxs_alg > 0) || (ec_fxs_alg == 0)) // forced value
        fxs_alg_chanmap = ec_fxs_alg;
    else
        fxs_alg_chanmap = rcb_card->fxs_chanmap; // On in A diraction

    if ((use_ec_fxo > 0) || (use_ec_fxo == 0)) // forced value
        use_fxo_chanmap = use_ec_fxo;
    else
        use_fxo_chanmap = rcb_card->fxo_chanmap;  // Off for fxs

    if ((use_ec_fxs > 0) || (use_ec_fxs == 0)) // forced value
        use_fxs_chanmap = use_ec_fxs;
    else
        use_fxs_chanmap = rcb_card->fxs_chanmap;  // off for fxo

//    printk("fxo_alg_chanmap %x - fxs_alg_chanmap %x\n",fxo_alg_chanmap,fxs_alg_chanmap);
//    printk("use_fxo_chanmap %x - use_fxs_chanmap %x\n",use_fxo_chanmap,use_fxs_chanmap);
#endif

    /* Setup DMA Addresses */
    *(volatile __u32*) (rcb_card->memaddr + RCB_TXBUFSTART) = (__u32) rcb_card->writedma;
    *(volatile __u32*) (rcb_card->memaddr + RCB_RXBUFSTART) = (__u32) rcb_card->readdma;
    *(volatile __u16*) (rcb_card->memaddr + RCB_BUFLEN) = ((__u16) (ZT_CHUNKSIZE * rcb_card->num_chans * 2) >> 2);

    return 0;
}

static void rcb_card_start_dma(struct rcb_card_t *rcb_card)
{
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(1);
    *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) |= (__u8) BIT_DMAGO;
}


static void rcb_card_stop_dma(struct rcb_card_t *rcb_card)
{
    *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) &= (__u8) ~BIT_DMAGO;
    *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT) &= (__u8)~(0x00);
    if (debug)
        printk("rcbfx %d: Statout = %x\n", rcb_card->pos+1, *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT));
    *(volatile __u8*) (rcb_card->memaddr + FW_DATA) = 0x00;
    *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = 0x00;
}

static void rcb_card_cleaner(struct rcb_card_t *rcb_card, int flags)
{
    if (rcb_card) {

        if (flags & STOP_DMA)   {
            *(volatile __u8*) (rcb_card->memaddr + RCB_CONTROL) &= (__u8) ~BIT_DMAGO;
        }
        if (flags & FREE_DMA)   {
            if (rcb_card->writechunk) {
                pci_free_consistent(rcb_card->dev, ZT_MAX_CHUNKSIZE * rcb_card->num_chans * 2 * 2, (void *)rcb_card->writechunk, rcb_card->writedma);
                rcb_card->writechunk = NULL;
            }
        }
        if (flags & IOUNMAP) {
            if (rcb_card->freeregion) {
                rcb_card->freeregion = -1;
                release_region(rcb_card->baseaddr, rcb_card->memlen);
            }
            iounmap(rcb_card->memaddr);
        }
        if (flags & ZUNREG) {
            if (!rcb_card->usecount)
                zt_unregister(&rcb_card->zap_span);
            else
                rcb_card->dead = 1;
        }
        if (flags & FREE_INT)
            free_irq(rcb_card->dev->irq, rcb_card);
        if (flags & RH_KFREE)
            kfree(rcb_card);
        if (flags & PCI_FREE)
            pci_set_drvdata(rcb_card->dev, NULL);
    }
}

#ifdef USE_G168_DSP

static  GpakPortConfig_t Gpak_24_chan_port_config = {

    SlotCfg2Groups,     // GpakSlotCfg_t         SlotsSelect1          port 1 Slot selection
    0x0000,             // unsigned short int    FirstBlockNum1        port 1 first group Block Number
    0xffff,             // unsigned short int    FirstSlotMask1        port 1 first group Slot Mask
    0x0001,             // unsigned short int    SecBlockNum1          port 1 second group Block Number
    0x00ff,             // unsigned short int    SecSlotMask1          port 1 second group Slot Mask
    SerWordSize8,       // GpakSerWordSize_t     SerialWordSize1       port 1 serial word size
    cmpNone,            // GpakCompandModes      CompandingMode1       port 1 companding mode
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t TxFrameSyncPolarity1  port 1 Tx Frame Sync Polarity
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t RxFrameSyncPolarity1  port 1 Rx Frame Sync Polarity
    SerClockActHigh,    // GpakSerClockPol_t     TxClockPolarity1      port 1 Tx Clock Polarity
    SerClockActLow,     // GpakSerClockPol_t     RxClockPolarity1      port 1 Rx Clock Polarity
    DataDelay0,         // GpakSerDataDelay_t    TxDataDelay1          port 1 Tx data delay
    DataDelay0,         // GpakSerDataDelay_t    RxDataDelay1          port 1 Rx data delay
    Disabled,           // GpakActivation        DxDelay1              port 1 DX Delay
    0x0000,             // unsigned short int    ThirdSlotMask1        port 1 3rd group Slot Mask
    0x0000,             // unsigned short int    FouthSlotMask1        port 1 4th group Slot Mask
    0x0000,             // unsigned short int    FifthSlotMask1        port 1 5th group Slot Mask
    0x0000,             // unsigned short int    SixthSlotMask1        port 1 6th group Slot Mask
    0x0000,             // unsigned short int    SevenSlotMask1        port 1 7th group Slot Mask
    0x0000,             // unsigned short int    EightSlotMask1        port 1 8th group Slot Mask
    SlotCfg2Groups,     // GpakSlotCfg_t         SlotsSelect2          port 2 Slot selection
    0x0000,             // unsigned short int    FirstBlockNum2        port 2 first group Block Number
    0xffff,             // unsigned short int    FirstSlotMask2        port 2 first group Slot Mask
    0x0001,             // unsigned short int    SecBlockNum2          port 2 second group Block Number
    0x00ff,             // unsigned short int    SecSlotMask2          port 2 second group Slot Mask
    SerWordSize8,       // GpakSerWordSize_t     SerialWordSize2       port 2 serial word size
    cmpNone,            // GpakCompandModes      CompandingMode2       port 2 companding mode
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t TxFrameSyncPolarity2  port 2 Tx Frame Sync Polarity
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t RxFrameSyncPolarity2  port 2 Rx Frame Sync Polarity
    SerClockActHigh,    // GpakSerClockPol_t     TxClockPolarity2      port 2 Tx Clock Polarity
    SerClockActLow,     // GpakSerClockPol_t     RxClockPolarity2      port 2 Rx Clock Polarity
    DataDelay0,         // GpakSerDataDelay_t    TxDataDelay2          port 2 Tx data delay
    DataDelay0,         // GpakSerDataDelay_t    RxDataDelay2          port 2 Rx data delay
    Disabled,           // GpakActivation        DxDelay2              port 2 DX Delay
    0x0000,             // unsigned short int    ThirdSlotMask2        port 2 3rd group Slot Mask
    0x0000,             // unsigned short int    FouthSlotMask2        port 2 4th group Slot Mask
    0x0000,             // unsigned short int    FifthSlotMask2        port 2 5th group Slot Mask
    0x0000,             // unsigned short int    SixthSlotMask2        port 2 6th group Slot Mask
    0x0000,             // unsigned short int    SevenSlotMask2        port 2 7th group Slot Mask
    0x0000,             // unsigned short int    EightSlotMask2        port 2 8th group Slot Mask
    SlotCfgNone,        // GpakSlotCfg_t         SlotsSelect3          port 3 Slot selection
    0x0000,             // unsigned short int    FirstBlockNum3        port 3 first group Block Number
    0x0000,             // unsigned short int    FirstSlotMask3        port 3 first group Slot Mask
    0x0000,             // unsigned short int    SecBlockNum3          port 3 second group Block Number
    0x0000,             // unsigned short int    SecSlotMask3          port 3 second group Slot Mask
    SerWordSize8,       // GpakSerWordSize_t     SerialWordSize3       port 3 serial word size
    cmpNone,            // GpakCompandModes      CompandingMode3       port 3 companding mode
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t TxFrameSyncPolarity3  port 3 Tx Frame Sync Polarity
    FrameSyncActHigh,   // GpakSerFrameSyncPol_t RxFrameSyncPolarity3  port 3 Rx Frame Sync Polarity
    SerClockActHigh,    // GpakSerClockPol_t     TxClockPolarity3      port 3 Tx Clock Polarity
    SerClockActHigh,    // GpakSerClockPol_t     RxClockPolarity3      port 3 Rx Clock Polarity
    DataDelay0,         // GpakSerDataDelay_t    TxDataDelay3          port 3 Tx data delay
    DataDelay0,         // GpakSerDataDelay_t    RxDataDelay3          port 3 Rx data delay
    Disabled,           // GpakActivation        DxDelay3              port 3 DX Delay
    0x0000,             // unsigned short int    ThirdSlotMask3        port 3 3rd group Slot Mask
    0x0000,             // unsigned short int    FouthSlotMask3        port 3 4th group Slot Mask
    0x0000,             // unsigned short int    FifthSlotMask3        port 3 5th group Slot Mask
    0x0000,             // unsigned short int    SixthSlotMask3        port 3 6th group Slot Mask
    0x0000,             // unsigned short int    SevenSlotMask3        port 3 7th group Slot Mask
    0x0000              // unsigned short int    EightSlotMask3        port 3 8th group Slot Mask

};

static GpakChannelConfig_t Gpak_chan_config = {

    SerialPort1,    // GpakSerialPort_t    PCM Input Serial Port A Id
    0,              // unsigned short int  PCM Input Time Slot
    SerialPortNull, // GpakSerialPort_t    PCM Output Serial Port A Id
    0,              // unsigned short int  PCM Output Time Slot
    SerialPort2,    // GpakSerialPort_t    PCM Input Serial Port B Id
    0,              // unsigned short int  PCM Input Time Slot
    SerialPort1,    // GpakSerialPort_t    PCM Output Serial Port B Id
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
        18,   // short int  Dynamic NLP control, NLP limit when EC about to converged
        12,   // short int  Dynamic NLP control, NLP limit when EC not converged yet
        0,    // short int  suppression level for NLP_SUPP mode
        50,   // short int  Echo Can CNG Noise threshold
        50,   // short int  Echo Can Max Adapts per frame
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
        18,   // short int  Dynamic NLP control, NLP limit when EC about to converged
        12,   // short int  Dynamic NLP control, NLP limit when EC not converged yet
        0,    // short int  suppression level for NLP_SUPP mode
        50,   // short int  Echo Can CNG Noise threshold
        50,   // short int  Echo Can Max Adapts per frame
        20,   // short int  Echo Can Cross Correlation limit
        3,    // short int  Echo Can Num FIR Segments
        64    // short int  Echo Can FIR Segment Length
    },

    cmpNone,         // GpakCompandModes    software companding
    rate2ms,         // GpakRate_t          FrameRate;          // Gpak Frame Rate

};


static void rcb_card_dsp_show_portconfig(GpakPortConfig_t PortConfig)
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

static void rcb_card_dsp_show_chanconfig(GpakChannelConfig_t ChanConfig)
{
    if (debug & DEBUG_DSP) {
        printk("%x = %s\n",ChanConfig.PcmInPortA,"PcmInPortA");
        printk("%x = %s\n",ChanConfig.PcmInSlotA,"PcmInSlotA");
        printk("%x = %s\n",ChanConfig.PcmOutPortA,"PcmOutPortA");
        printk("%x = %s\n",ChanConfig.PcmOutSlotA,"PcmOutSlotA");
        printk("%x = %s\n",ChanConfig.PcmInPortB,"PcmInPortB");
        printk("%x = %s\n",ChanConfig.PcmInSlotB,"PcmInSlotB");
        printk("%x = %s\n",ChanConfig.PcmOutPortB,"PcmOutPortB");
        printk("%x = %s\n",ChanConfig.PcmOutSlotB,"PcmOutSlotB");

        printk("%x = %s\n",ChanConfig.ToneTypesA,"ToneTypesA");
        printk("%x = %s\n",ChanConfig.ToneTypesB,"ToneTypesB");

        printk("%x = %s\n",ChanConfig.EcanEnableA,"EcanEnableA");
        printk("%x = %s\n",ChanConfig.EcanEnableB,"EcanEnableB");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanTapLength,"EcanParametersA.EcanTapLength");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanNlpType,"EcanParametersA.EcanNlpType");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanAdaptEnable,"EcanParametersA.EcanAdaptEnable");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanG165DetEnable,"EcanParametersA.EcanG165DetEnable");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanDblTalkThresh,"EcanParametersA.EcanDblTalkThresh");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanNlpThreshold,"EcanParametersA.EcanNlpThreshold");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanNlpConv,"EcanParametersA.EcanNlpConv");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanNlpUnConv,"EcanParametersA.EcanNlpUnConv");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanNlpMaxSuppress,"EcanParametersA.EcanNlpMaxSuppress");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanCngThreshold,"EcanParametersA.EcanCngThreshold");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanAdaptLimit,"EcanParametersA.EcanAdaptLimit");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanCrossCorrLimit,"EcanParametersA.EcanCrossCorrLimit");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanNumFirSegments,"EcanParametersA.EcanNumFirSegments");
        printk("%x = %s\n",ChanConfig.EcanParametersA.EcanFirSegmentLen,"EcanParametersA.EcanFirSegmentLen");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanTapLength,"EcanParametersB.EcanTapLength");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanNlpType,"EcanParametersB.EcanNlpType");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanAdaptEnable,"EcanParametersB.EcanAdaptEnable");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanG165DetEnable,"EcanParametersB.EcanG165DetEnable");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanDblTalkThresh,"EcanParametersB.EcanDblTalkThresh");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanNlpThreshold,"EcanParametersB.EcanNlpThreshold");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanNlpConv,"EcanParametersB.EcanNlpConv");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanNlpUnConv,"EcanParametersB.EcanNlpUnConv");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanNlpMaxSuppress,"EcanParametersB.EcanNlpMaxSuppress");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanCngThreshold,"EcanParametersB.EcanCngThreshold");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanAdaptLimit,"EcanParametersB.EcanAdaptLimit");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanCrossCorrLimit,"EcanParametersB.EcanCrossCorrLimit");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanNumFirSegments,"EcanParametersB.EcanNumFirSegments");
        printk("%x = %s\n",ChanConfig.EcanParametersB.EcanFirSegmentLen,"EcanParametersB.EcanFirSegmentLen");
        printk("%x = %s\n",ChanConfig.SoftwareCompand,"SoftwareCompand");
        printk("%x = %s\n",ChanConfig.FrameRate,"FrameRate");

    }
    return;
}

static unsigned short int rcb_card_dsp_ping(struct rcb_card_t *rcb_card)
{

    gpakPingDspStat_t ping_stat;
    unsigned short int dsp_ver;

        ping_stat = gpakPingDsp(rcb_card, rcb_card->pos, &dsp_ver);

    if (debug & DEBUG_DSP) {
        if (ping_stat == PngSuccess)
            printk("rcbfx %d: G168 DSP Ping DSP Version %x\n",rcb_card->pos+1, dsp_ver);
        else
            printk("rcbfx %d: G168 DSP Ping Error %d\n",rcb_card->pos+1, ping_stat);
    }

    if (ping_stat == PngSuccess)
        return dsp_ver;
    else
        return 0;
}

#if 0
static int rcb_chan_echocan_switch(struct zt_chan *zap_chan, int eclen)
{

    struct rcb_card_t *rcb_card = zap_chan->pvt;
    int chan_num = zap_chan->chanpos - 1; // These are the ones that start with 1
//    __u32 use_ec_a;
//    __u32 use_ec_b;

    if (debug & DEBUG_DSP)
        printk("rcbfx %d: Echo can switch channel %d Ecan Length %d\n", rcb_card->pos+1, chan_num, eclen);
/*
    if (eclen > 0) {
        if (rcb_card->chanflag & (1 << chan_num) & use_fxo_chanmap) {
            use_ec_b = *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENB);
            use_ec_b |= (1 << chan_num);
            *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENB) = use_ec_b;
            if (debug & DEBUG_DSP)
                printk("rcbfx %d: Enable G168 DSP into card as FXO on chan %d %x\n",rcb_card->pos+1,chan_num,use_ec_b);
        }

        if (rcb_card->chanflag & (1 << chan_num) & use_fxs_chanmap) {
            use_ec_a = *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA);
            use_ec_a |= (1 << chan_num);
            *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA) = use_ec_a;
            if (debug & DEBUG_DSP)
                printk("rcbfx %d: Enable G168 DSP out of card as FXS on chan %d %x\n",rcb_card->pos+1,chan_num,use_ec_a);
        }
    }
*/
//    if (eclen == 0) {
//        *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENB) &= ~(__u32)(1 << chan_num);
//        *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA) &= ~(__u32)(1 << chan_num);
//        if (rcb_card->chanflag & (1 << chan_num))
//            printk("rcbfx %d: Disable G168 DSP on chan %d Ecan Length 0\n",rcb_card->pos+1,chan_num);
//    }

    rcb_chan_echocan_soft(zap_chan, eclen);

	return 0;
}

static int rcb_chan_echocan_soft(struct zt_chan *zap_chan, int eclen)
{

    struct rcb_card_t *rcb_card = zap_chan->pvt;
    int chan_num = zap_chan->chanpos - 1; // These are the ones that start with 1
    gpakAlgControlStat_t a_c_stat;
    GPAK_AlgControlStat_t a_c_err;

    if (debug & DEBUG_DSP)
        printk("rcbfx %d: Echo Can software len %d chan %d\n",rcb_card->pos+1 , eclen, chan_num);

    if (rcb_card->chanflag & (1 << chan_num)) {

        if (debug & DEBUG_DSP)
            printk("rcbfx %d: Echo can channel %d Ecan Length %d\n", rcb_card->pos+1, chan_num, eclen);

        if (eclen > 0) {
            if (debug & DEBUG_DSP)
                printk("rcbfx %d: Chan %d G168 DSP Enabled\n",rcb_card->pos+1, chan_num+1);
            if ((a_c_stat = gpakAlgControl(rcb_card, rcb_card->pos, chan_num, EnableEcanA, &a_c_err))) {
                if (debug & DEBUG_DSP)
                    printk("rcbfx %d: G168 DSP Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);
                return -1;
            }
        }
        else {
            if (debug & DEBUG_DSP)
                printk("rcbfx %d: Chan %d G168 DSP Disabled\n",rcb_card->pos+1, chan_num+1);
            if ((a_c_stat = gpakAlgControl(rcb_card, rcb_card->pos, chan_num, BypassEcanA, &a_c_err))) {
                if (debug & DEBUG_DSP)
                    printk("rcbfx %d: G168 DSP Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);
                return -1;
            }
        }
    }
	return 0;
}
#endif


static void rcbfx_chan_ec_enable(struct rcb_card_t *rcb_card, int chan_num)
{
    gpakAlgControlStat_t a_c_stat;
    GPAK_AlgControlStat_t a_c_err;
    unsigned short int DspId;
//    unsigned int mask, slot_num;

//    if (ec_disable & (1 << chan_num)) {
//        if (debug & DEBUG_DSP)
//            printk("rcbfx %d: Echo Can NOT enable DSP EC Chan %d\n",rcb_card->pos+1, chan_num);
//        return;
//    }

    DspId = rcb_card->pos;
//    printk("chan %x slot %x\n", chan_num, slot_num);

    if (debug & DEBUG_DSP)
        printk("rcbfx: %d: Echo Can enable DSP %d EC Chan %d\n",rcb_card->pos+1, 1, chan_num-1);

//    if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num-1, ResetEcanB, &a_c_err)))
//        printk("r1t1 %d: G168 DSP Reset Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);

    if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num, EnableEcanB, &a_c_err)))
        printk("rcbfx: %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);

//    if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num, ResetEcanB, &a_c_err)))
//        printk("rcbfx: %d: G168 DSP Reset Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);

//    if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num-1, EnableEcanA, &a_c_err)))
//        printk("r1t1 %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);

//    mask = __r1t1_card_pci_in(rcb_card, TARG_REGS + R1T1_ECA1);
//    mask |= (1 << slot_num);
//    __r1t1_card_pci_out(rcb_card, TARG_REGS + R1T1_ECA1, mask);

//    printk("Mask %x\n",mask);

    return;
}

static void rcbfx_chan_ec_disable(struct rcb_card_t *rcb_card, int chan_num)
{
    gpakAlgControlStat_t a_c_stat;
    GPAK_AlgControlStat_t a_c_err;
    unsigned short int DspId;
//    unsigned int mask, slot_num;

    DspId = rcb_card->pos;

//    printk("chan %x slot %x\n", chan_num, slot_num);

    if (debug & DEBUG_DSP)
        printk("rcbfx: %d: Echo Can disable DSP %d EC Chan %d\n",rcb_card->pos+1, 1, chan_num);

//    if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num-1, BypassEcanA, &a_c_err)))
//        printk("r1t1 %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);

    if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num, BypassEcanB, &a_c_err)))
        printk("rcbfx: %d: G168 DSP Enable Alg Control failed res = %d error = %d\n", rcb_card->pos+1, a_c_stat, a_c_err);

//    mask = __r1t1_card_pci_in(rcb_card, TARG_REGS + R1T1_ECA1);
//    mask &= ~(1 << slot_num);
//    __r1t1_card_pci_out(rcb_card, TARG_REGS + R1T1_ECA1, mask);

//    printk("Mask %x\n",mask);

    return;
}


static int rcbfx_zap_chan_echocan(struct zt_chan *zap_chan, int eclen)
{
    struct rcb_card_t *rcb_card = zap_chan->pvt;
    int chan_num;

    chan_num = zap_chan->chanpos-1;

    if (debug & DEBUG_DSP) {
        printk("rcbfx: %d Echo Can control Span %d Chan %d zt_chan %d\n",rcb_card->pos+1, 1, chan_num, zap_chan->channo);
        printk("DSP up %x\n",rcb_card->dsp_up);
    }
    if (rcb_card->dsp_up == 1) {
        if (eclen)
            rcb_card->nextec |= (1 << chan_num);
        else
            rcb_card->nextec &= ~(1 << chan_num);

        queue_work(rcb_card->wq, &rcb_card->work);
    }
    return 0;
}


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void echocan_bh(void *data)
{
	struct rcb_card_t *rcb_card = data;
#else
static void echocan_bh(struct work_struct *data)
{
	struct rcb_card_t *rcb_card = container_of(data, struct rcb_card_t, work);
#endif
    unsigned int todo, chan_num;

    todo = rcb_card->nextec ^ rcb_card->currec;
    if (debug & DEBUG_DSP) {
        printk("rcbfx %d Echo Can control bh change %x to %x\n",rcb_card->pos+1, todo, (rcb_card->nextec & todo));
        printk("nextec %x currec %x\n",rcb_card->nextec,rcb_card->currec);
    }

    for (chan_num = 0; chan_num < rcb_card->num_chans; chan_num++) {

        if (todo & (1 << chan_num)) {
            if (rcb_card->nextec & (1 << chan_num)) {
                rcbfx_chan_ec_enable(rcb_card, chan_num);
                rcb_card->currec |= (1 << chan_num);
//                schedule();
            }
            else {
                rcbfx_chan_ec_disable(rcb_card, chan_num);
                rcb_card->currec &= ~(1 << chan_num);
//                schedule();
            }
        }
    }
}


static int rcb_card_dsp_init(struct rcb_card_t *rcb_card)
{
    gpakDownloadStatus_t dl_res=0;
    gpakConfigPortStatus_t cp_res;
    GPAK_PortConfigStat_t cp_error;
    GPAK_ChannelConfigStat_t chan_config_err;
    int chan_num;
    gpakConfigChanStatus_t chan_conf_stat;
    int dsp_chans=0;
    int ec_mask_a=0;
    int ec_mask_b=0;
    int dsp_in_use=0;
    gpakReadFramingStatsStatus_t framing_status_status;
    unsigned short int ec1, ec2, ec3, dmaec, slips;

    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA) = (__u32) ec_mask_a;
    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENB) = (__u32) ec_mask_b;

    if (no_ec)
        return 0;

    rcb_card->hpi_fast = 0;
    *(volatile __u32*) (rcb_card->memaddr + RCB_HPIC) = (__u32) (0);
    rcb_card_wait_hpi(rcb_card, RCB_HRDY);
    rcb_card->hpi_xadd = 0;

    // quit here if no dsp channels on
    if (!(dsp_chans = rcb_card->chanflag)) {
        printk("rcbfx %d: G168 DSP Disabled\n",rcb_card->pos+1);
        return 0;
    }

    if (rcb_card->dsp_type == DSP_5507) {

        // Load the loader file
        if ((dl_res = gpakDownloadLoader(rcb_card , rcb_card->pos, loader_file))) {
            printk("rcbfx %d: G168 DSP Loader Loader Failed %d\n",rcb_card->pos+1, dl_res);
            return -1;
        }
        if (debug & DEBUG_DSP)
            printk("rcbfx %d: G168 DSP Loader Loader Sucess\n",rcb_card->pos+1);

        // execute the loader
        rcb_card_dsp_set(rcb_card, DSP_ENTRY_ADDR_LO, (0xFFFF & BL_DSP_BOOTLOADER_ENTRY));
        rcb_card_dsp_set(rcb_card, DSP_ENTRY_ADDR_HI, (0xFF00 | (0xFF & (BL_DSP_BOOTLOADER_ENTRY >> 16))));

        if (debug & DEBUG_DSP) {
            printk("HPIA 0x0061 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x0061) );
            printk("HPIA 0x0060 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x0060) );
            printk("HPIA 0x3800 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3800) );
            printk("HPIA 0x3801 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3801) );
            printk("HPIA 0x3802 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3802) );
            printk("HPIA 0x3803 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3803) );
            printk("HPIA 0x3804 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3804) );
            printk("HPIA 0x3805 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3805) );
        }

    }

    if (rcb_card->dsp_type == DSP_5507) {
        if ((dl_res = gpakDownloadDsp_5507(rcb_card, rcb_card->pos, app_file))) {
            printk("rcbfx %d: G168 DSP App Loader Failed %d\n",rcb_card->pos+1, dl_res);
            return -1;
        }
    }

    if (rcb_card->dsp_type == DSP_5510) {
        if ((dl_res = gpakDownloadDsp_5510(rcb_card, rcb_card->pos, app_file))) {
            printk("rcbfx %d: G168 DSP App Loader Failed %d\n",rcb_card->pos+1, dl_res);
            return -1;
        }
    }

    if (debug & DEBUG_DSP)
        printk("rcbfx %d: G168 DSP App Loader Sucess %d\n",rcb_card->pos+1, dl_res);

    if (rcb_card->dsp_type == DSP_5510) {
        *(volatile __u32*) (rcb_card->memaddr + RCB_HPIC) = (__u32) (RCB_BL_GO);
        rcb_card_wait_hpi(rcb_card, RCB_HRDY);
    }

    {
        long end_jiffies;
        end_jiffies = jiffies + 10;
        while (end_jiffies > jiffies);
    }

    rcb_card_dsp_ping(rcb_card);
    rcb_card_dsp_ping(rcb_card);
    rcb_card_dsp_ping(rcb_card);
    printk("rcbfx %d: G168 DSP Ping DSP Version %x\n",rcb_card->pos+1, rcb_card_dsp_ping(rcb_card));

    if (debug & DEBUG_DSP) {
        framing_status_status = gpakReadFramingStats(rcb_card, rcb_card->pos, &ec1, &ec2, &ec3, &dmaec, &slips);
        if (framing_status_status == RfsSuccess) {
            printk("rcbfx %d: G168 DSP Framing Status Sucess %d\n",rcb_card->pos+1, framing_status_status);
            printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",rcb_card->pos+1, ec1, ec2, ec3, dmaec, slips);
        }
        else {
            printk("rcbfx %d: G168 DSP Framing Status Failed %d\n",rcb_card->pos+1, framing_status_status);
            printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",rcb_card->pos+1, ec1, ec2, ec3, dmaec, slips);
        }
    }

    Gpak_24_chan_port_config.FirstSlotMask1 = (rcb_card->chanflag & 0xffff);
    Gpak_24_chan_port_config.SecSlotMask1 = ((rcb_card->chanflag >> 16) & 0xffff);

    Gpak_24_chan_port_config.FirstSlotMask2 = (rcb_card->chanflag & 0xffff);
    Gpak_24_chan_port_config.SecSlotMask2 = ((rcb_card->chanflag >> 16) & 0xffff);

    rcb_card_dsp_show_portconfig(Gpak_24_chan_port_config);


    if ((cp_res = gpakConfigurePorts(rcb_card , rcb_card->pos, &Gpak_24_chan_port_config, &cp_error))) {
        printk("rcbfx %d: G168 DSP Port Config failed res = %d error = %d\n", rcb_card->pos+1, cp_res, cp_error);
        return -1;
    }


    if (debug & DEBUG_DSP)
        printk("rcbfx %d: G168 DSP Port Config success %d\n",rcb_card->pos+1, cp_res);

    rcb_card_dsp_ping(rcb_card);

    if (debug & DEBUG_DSP) {
        framing_status_status = gpakReadFramingStats(rcb_card, rcb_card->pos, &ec1, &ec1, &ec3, &dmaec, &slips);
        if (framing_status_status == RfsSuccess) {
            printk("rcbfx %d: G168 DSP Framing Status Sucess %d\n",rcb_card->pos+1, framing_status_status);
            printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",rcb_card->pos+1, ec1, ec2, ec3, dmaec, slips);
        }
        else {
            printk("rcbfx %d: G168 DSP Framing Status Failed %d\n",rcb_card->pos+1, framing_status_status);
            printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",rcb_card->pos+1, ec1, ec2, ec3, dmaec, slips);
        }
    }

    Gpak_chan_config.EcanParametersA.EcanNlpType = nlp_type;
    Gpak_chan_config.EcanParametersB.EcanNlpType = nlp_type;

    Gpak_chan_config.EcanParametersA.EcanNlpThreshold = nlp_threshold;
    Gpak_chan_config.EcanParametersB.EcanNlpThreshold = nlp_threshold;

    Gpak_chan_config.EcanParametersA.EcanNlpMaxSuppress = nlp_max_supress;
    Gpak_chan_config.EcanParametersB.EcanNlpMaxSuppress = nlp_max_supress;

    for (chan_num = 0; chan_num < rcb_card->num_chans; chan_num++) {

        if (rcb_card->chanflag & (1 << chan_num)) {

            dsp_in_use++;
            Gpak_chan_config.EcanEnableA = Enabled;
            Gpak_chan_config.SoftwareCompand = cmpPCMU;

            Gpak_chan_config.PcmInSlotA = chan_num;
            Gpak_chan_config.PcmOutSlotA = chan_num;
            Gpak_chan_config.PcmInSlotB = chan_num;
            Gpak_chan_config.PcmOutSlotB = chan_num;

            rcb_card_dsp_show_chanconfig(Gpak_chan_config);

            if ((chan_conf_stat = gpakConfigureChannel(rcb_card, rcb_card->pos, chan_num, tdmToTdm, &Gpak_chan_config, &chan_config_err))) {
                printk("rcbfx %d: Chan %d G168 DSP Chan Config failed error = %d  %d\n",rcb_card->pos+1, chan_num+1, chan_config_err, chan_conf_stat);
                return -1;
            }

            else if (debug & DEBUG_DSP)
                printk("rcbfx %d: G168 DSP Chan %d Config success %d\n",rcb_card->pos+1, chan_num, chan_conf_stat);

            rcb_card_dsp_ping(rcb_card);

            rcbfx_chan_ec_disable(rcb_card, chan_num);

        }
    }

    // switch to dsp audio stream
    if ((use_ec == -1) || (use_ec == 1))
        *(volatile __u8*) (rcb_card->memaddr + EC_CNTL) |= (__u8) EC_ON;

    rcb_card->dsp_up = 1;

//    printk("rcbfx %d: Enable ECan Mask A %x ECan Mask B %x\n",rcb_card->pos+1, ec_mask_a, ec_mask_b);
//    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA) = (__u32) ec_mask_a;
//    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENB) = (__u32) ec_mask_b;

    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA) = (__u32) 0xffffffff;
//    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENA) = (__u32) 0;
    *(volatile __u32*) (rcb_card->memaddr + RCB_EC_ENB) = (__u32) 0;

    rcb_card->wq = create_singlethread_workqueue("rcbfx_ec");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK(&rcb_card->work, echocan_bh, rcb_card);
#else
	INIT_WORK(&rcb_card->work, echocan_bh);
#endif

    rcb_card->zap_span.echocan = rcbfx_zap_chan_echocan;

    printk("rcbfx %d: G168 DSP Active and Servicing %d Channels - %x\n",rcb_card->pos+1, dsp_in_use, dsp_chans);

    return dsp_chans;
}

#endif


static int __devinit rcb_card_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    int res, regnum;
    static struct rcb_card_t *rcb_card;
    struct rcb_card_desc *d = (struct rcb_card_desc *)ent->driver_data;
    int x;
    static int initd_ifaces=0;
    int channum;

    if(initd_ifaces){
        memset((void *)ifaces,0,(sizeof(struct rcb_card_t *))*RH_MAX_IFACES);
        initd_ifaces=1;
    }
    for (x=0;x<RH_MAX_IFACES;x++)
        if (!ifaces[x]) break;
    if (x >= RH_MAX_IFACES) {
        printk("Too many interfaces\n"); // had some chickens EI
        return -EIO;
    }

    if (pci_enable_device(pdev)) {
        printk("No Rhino spotted\n"); // had some cows EI
        res = -EIO;
    } else {
        rcb_card = kmalloc(sizeof(struct rcb_card_t), GFP_KERNEL);
        if (rcb_card) {
            int cardcount = 0;
            ifaces[x] = rcb_card;
            memset(rcb_card, 0, sizeof(struct rcb_card_t));
            spin_lock_init(&rcb_card->lock);
            rcb_card->curcard = -1;
            rcb_card->baseaddr =  pci_resource_start(pdev, 0);
            rcb_card->memlen = pci_resource_len(pdev, 0);
            rcb_card->chans_configed = 0;
            rcb_card->dev = pdev;
            rcb_card->pos = x;

            rcb_card->num_chans = d->num_chans;
            rcb_card->num_slots = d->num_slots;
            rcb_card->variety = d->name;
            rcb_card->hw_ver_min = d->hw_ver_min;
            if (rcb_card->num_chans > 8)
                rcb_card->dsp_type = DSP_5510;
            else
                rcb_card->dsp_type = DSP_5507;

            if (request_region(rcb_card->baseaddr, rcb_card->memlen, rcb_card->variety))
                rcb_card->freeregion = 1;

            rcb_card->memaddr = ioremap_nocache(rcb_card->baseaddr, rcb_card->memlen);
            printk("rcbfx %d: Rhino PCI BAR0 %lx IOMem mapped at %lx\n", rcb_card->pos+1, (long unsigned int)rcb_card->baseaddr, (long unsigned int) rcb_card->memaddr);

            //  chunksize * num channels * 2 swap buffers * 2 read and write
            rcb_card->writechunk = (unsigned char *)pci_alloc_consistent(pdev, ZT_MAX_CHUNKSIZE * rcb_card->num_slots * 2 * 2, &rcb_card->writedma);

            if (!rcb_card->writechunk) {
                printk("rcbfx %d: Unable to allocate DMA-able memory\n", rcb_card->pos+1);
                rcb_card_cleaner(rcb_card, IOUNMAP);
                return -ENOMEM;
            }

            // read starts at write plus 8 samples * 4 channels * 2 buffers later = 0x40 bytes
            rcb_card->readchunk = rcb_card->writechunk + (ZT_MAX_CHUNKSIZE * rcb_card->num_slots * 2);  // half the total
            rcb_card->readdma = rcb_card->writedma + (ZT_MAX_CHUNKSIZE * rcb_card->num_slots * 2);      // in bytes

            pci_set_master(pdev);

            pci_set_drvdata(pdev, rcb_card);
#ifdef ZAP_IRQ_SHARED
            if (request_irq(pdev->irq, rcb_card_interrupt, ZAP_IRQ_SHARED, rcb_card->variety, rcb_card)) {
#else
            if (request_irq(pdev->irq, rcb_card_interrupt, SA_SHIRQ, rcb_card->variety, rcb_card)) {
#endif
                printk("rcbfx %d: Unable to request IRQ %d\n", rcb_card->pos+1, pdev->irq);
                rcb_card_cleaner(rcb_card, RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | STOP_DMA | FREE_INT | ZUNREG);
                return -EIO; // had some pigs EI
            }

            if (rcb_card_hardware_init(rcb_card)) {  // sticks in here without up
                printk("rcbfx %d: Unable to intialize hardware\n", rcb_card->pos+1);
                rcb_card_cleaner(rcb_card, RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | STOP_DMA | FREE_INT | ZUNREG);
                return -EIO; // had some ducks EI
            }

            rcb_card->dsp_up = 0;

#ifdef USE_G168_DSP

                if (rcb_card_dsp_init(rcb_card) < 0)
                    printk("rcbfx %d: Unable to intialize G168 DSP\n", rcb_card->pos+1);
#endif

            if ((use_ec_zap > 0) || (use_ec_zap == 0)) // forced value
                zt_ec_chanmap = use_ec_zap;
            else if (rcb_card->dsp_up == 0) // Switch on if there is no DSP
                zt_ec_chanmap = -1;
            else
                zt_ec_chanmap = 0;


            if (use_ec == 1)
                *(volatile __u8*) (rcb_card->memaddr + EC_CNTL) |= (__u8) EC_ON;

            if (use_ec == 0)
                *(volatile __u8*) (rcb_card->memaddr + EC_CNTL) &= (__u8) ~EC_ON;

            if (rcb_card_initialize(rcb_card)) { // set up and register span and channels
                printk("rcbfx %d: Unable to intialize \n", rcb_card->pos+1);
                rcb_card_cleaner(rcb_card, RH_KFREE | IOUNMAP | FREE_DMA | STOP_DMA | FREE_INT);
                return -EIO;  // had some goats EI
            }

            printk("rcbfx %d: Starting DMA\n", rcb_card->pos+1);

            rcb_card_start_dma(rcb_card);

            for (regnum = 0; regnum < P_TBL_CNT; regnum++) {
                if (regnum < 24) {
                    channum = regnum*2;
                    if (adid_map & (1 << channum))
                        rcb_settings_default[regnum] = (rcb_settings_default[regnum] & 0xf0) | 0x07;
                    channum = regnum*2 + 1;
                    if (adid_map & (1 << channum))
                        rcb_settings_default[regnum] = (rcb_settings_default[regnum] & 0x0f) | 0x70;

                }
//                printk("register %x - address %x - data %x\n", regnum, PARAM_TBL + regnum, rcb_settings_default[regnum]);
                *(volatile __u8*) (rcb_card->memaddr + PARAM_TBL + regnum) = (__u8) (rcb_settings_default[regnum]) ;
            }

//            for (regnum = 0x800; regnum <= 0x9fc; regnum += 4) {
//                printk("%x = %x \n", regnum, *(volatile __u32*)(rcb_card->memaddr + regnum));
//	    }

            *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) = (__u32)(0x01000); // notify changes

            for (x = 0; x < rcb_card->num_chans; x++) {
                if (rcb_card->chanflag & (1 << x))
                    cardcount++;
            }
            // let board run signaling data now
            *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT) |= (__u8)(0x01); // notify changes
            if (debug)
                printk("rcbfx %d: Statout = %x\n", rcb_card->pos+1, *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT));
            *(volatile __u32*) (rcb_card->memaddr + RCB_TXSIGSTAT) = (__u32)(0x04000); // notify changes

//            rcb_card_check_sigbits(rcb_card, 0xfff);

            if ((rcb_card->dev->device == PCI_DEVICE_RCB4FXO) ||
                (rcb_card->dev->device == PCI_DEVICE_RCB24FXO) ||
                (rcb_card->dev->device == PCI_DEVICE_RCB24FXS))

                printk("rcbfx %d: Spotted a Rhino: %s (%d channels)\n", rcb_card->pos+1, rcb_card->variety, cardcount);
            else
                printk("rcbfx %d: Spotted a Rhino: %s (%d modules)\n", rcb_card->pos+1, rcb_card->variety, cardcount/2);

            res = 0;

        } else  //  if (!rcb_card)
            res = -ENOMEM;
    }
    return res;
}

static void __devexit rcb_card_remove_one(struct pci_dev *pdev)
{
    struct rcb_card_t *rcb_card = pci_get_drvdata(pdev);
    if (rcb_card) {
        if (rcb_card->dsp_up) {
            flush_workqueue(rcb_card->wq);
            destroy_workqueue(rcb_card->wq);
        }
        *(volatile __u8*) (rcb_card->memaddr + FW_BOOT) = 0x00 | DSP_RST; // reset DSP
        rcb_card_stop_dma(rcb_card);
        *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT) &= (__u8)~(0x00);
        if (debug)
            printk("rcbfx %d: Statout = %x\n", rcb_card->pos+1, *(volatile __u8*) (rcb_card->memaddr + RCB_STATOUT));
        *(volatile __u8*) (rcb_card->memaddr + FW_DATA) = 0x00;
        *(volatile __u8*) (rcb_card->memaddr + FW_COMOUT) = 0x00;
        pci_free_consistent(pdev, ZT_MAX_CHUNKSIZE * 2 * rcb_card->num_chans, (void *)rcb_card->writechunk, rcb_card->writedma);
        free_irq(pdev->irq, rcb_card);
        if (!rcb_card->usecount)
            rcb_card_release(rcb_card);
        else
            rcb_card->dead = 1;
    }
}

static struct pci_device_id rcb_card_pci_tbl[] = {
//    vendor                              subvendor               class   driver_data
//                      device                        subdevice      class_mask
    { PCI_VENDOR_RHINO, PCI_DEVICE_RCB4FXO,   PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &rcb4fxo },
    { PCI_VENDOR_RHINO, PCI_DEVICE_RCB8FXX,   PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &rcb8fxx },
    { PCI_VENDOR_RHINO, PCI_DEVICE_RCB24FXS,  PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &rcb24fxs },
    { PCI_VENDOR_RHINO, PCI_DEVICE_RCB24FXX,  PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &rcb24fxx },
    { PCI_VENDOR_RHINO, PCI_DEVICE_RCB24FXO,  PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &rcb24fxo },
    { 0 }
};

MODULE_DEVICE_TABLE(pci, rcb_card_pci_tbl);

static struct pci_driver rcb_driver = {
    name     : "rcbfx",
    probe    : rcb_card_init_one,
    remove   : __devexit_p(rcb_card_remove_one),
    suspend  : NULL,
    resume   : NULL,
    id_table : rcb_card_pci_tbl,
};

static int __init rcb_card_init(void)
{
    int res;
    res = zap_pci_module(&rcb_driver);
    if (res)
        return -ENODEV;
    return 0;
}

static void __exit rcb_card_cleanup(void)
{
    pci_unregister_driver(&rcb_driver);
}

module_param(force_fw, int, 0600);
MODULE_PARM_DESC(force_fw, "Reprogram firmware regardless of version");
module_param(debug, int, 0600);
MODULE_PARM_DESC(debug, "1 for debugging messages");
module_param(nlp_type, int, 0600);
MODULE_PARM_DESC(nlp_type, "0 - off, 1 - mute, 2 - rand, 3 - hoth, 4 - supp");

module_param(zt_ec_chanmap, int, 0600);
module_param(fxs_alg_chanmap, int, 0600);
module_param(fxo_alg_chanmap, int, 0600);
module_param(use_fxs_chanmap, int, 0600);
module_param(use_fxo_chanmap, int, 0600);
module_param_array(lvs, int , &arr_argc, 0600);
module_param_array(reg_val, int , &arr_argc, 0600);
module_param(battime, int, 0600);
module_param(reg_addr, int, 0600);
module_param(no_ec, int, 0600);
module_param(adid_map, int, 0600);
module_param(nlp_threshold, int, 0600);
module_param(nlp_max_supress, int, 0600);

MODULE_DESCRIPTION("Rhino Equipment Modular Analog Interface Driver " RHINOPKGVER );
MODULE_AUTHOR("Bob Conklin <bob@rhinoequipment.com>");
MODULE_VERSION( RHINOPKGVER );

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(rcb_card_init);
module_exit(rcb_card_cleanup);
