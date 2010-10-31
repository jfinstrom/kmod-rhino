/*
 * Copyright (c) 2005, Adaptive Digital Technologies, Inc.
 *
 * File Name: GpakCust.c
 *
 * Description:
 *   This file contains host system dependent functions to support generic
 *   G.PAK API functions. The file is integrated into the host processor
 *   connected to C55x G.PAK DSPs via a Host Port Interface.
 *
 *   Note: This file needs to be modified by the G.PAK system integrator.
 *
 * Version: 1.0
 *
 * Revision History:
 *   06/15/05 - Initial release.
 *
 */

#include "GpakCust.h"
#include "rxt1.h"
#include <linux/delay.h>

void __rxt1_card_wait_hpi(struct rxt1_card_t *rxt1_card, __u8 flags)
{
    __u8 temp;
    int loops;
    int bus_sel;
    static int debug_limit=0;

    bus_sel = (HPI_SEL | DSP_RST) & __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC);
    if (bus_sel != (HPI_SEL | DSP_RST))
        printk("r%dt1: Locking Error %x\n",rxt1_card->numspans, bus_sel);

    if (!(rxt1_card->hpi_fast)) {
        temp = (__u8)(__rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIRDX_STAT) >> 16);
        if (debug_limit < 100) {
            debug_limit++;
//            printk("Stat %x \n",temp); 
        }
        for (loops=0;loops<200;loops++) {
            temp = (__u8) (__rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIRDX_STAT) >> 16);

//            if (debug_limit < 100)
//                printk("Stat %x %x %x\n",temp, loops, bus_sel); 

            if ((temp & flags) == flags) {
                return;
            }

//            printk("HRDY %s BUSY_N %s NEW %s Loop %d temp %x\n", (temp & RXT1_HRDY) ? "ON" : "OFF",
//            (temp & RXT1_BUSY_N) ? "ON" : "OFF", (temp & RXT1_D_NEW) ? "ON" : "OFF", loops, temp);

        }
        
//    printk(". %x %x", temp, flags);
//    printk("rxt1: loop timed out waiting for HPI %x %x\n", temp, flags);

    }
    return;
}

void __rxt1_card_hpic_set(struct rxt1_card_t *rxt1_card, __u16 hpic_data)
{
    __rxt1_card_pci_out(rxt1_card, RXT1_DSP_HPIC + TARG_REGS, (__u32) (hpic_data), 0);
    __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
}

void rxt1_card_hpic_set(struct rxt1_card_t *rxt1_card, __u16 hpic_data)
{
    unsigned int hpi_lock;
    hpi_lock = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC);
    __rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, ((hpi_lock & ~0x1F) | HPI_SEL | DSP_RST) , 0 );
    __rxt1_card_hpic_set(rxt1_card, hpic_data);
    __rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, hpi_lock, 0 );
}

void rxt1_card_dsp_set(struct rxt1_card_t *rxt1_card, __u32 dsp_address, __u16 dsp_data)
{
    __u32 u_nib;
    unsigned int hpi_lock;
    unsigned long flags;
    int hcs;
    
    spin_lock_irqsave(&rxt1_card->reglock, flags);
    hcs = (1 << rxt1_card->dsp_sel);
    
    __rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) (hcs), 0);

    hpi_lock = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC);
    __rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, ((hpi_lock & ~0x1F) | HPI_SEL | DSP_RST) , 0 );

    if (rxt1_card->dsp_type == DSP_5510) {
        u_nib = ((dsp_address & 0xf0000) >> 16);
        if (!(rxt1_card->hpi_xadd[rxt1_card->dsp_sel] == u_nib)) {
//            printk("rxt1: DSP %x set hpi_xadd %x u_nib %x\n", rxt1_card->dsp_sel, rxt1_card->hpi_xadd[rxt1_card->dsp_sel], u_nib);
        
            rxt1_card->hpi_xadd[rxt1_card->dsp_sel] = u_nib;
            
            __rxt1_card_pci_out(rxt1_card, RXT1_DSP_HPIC + TARG_REGS, (__u32) (RXT1_XADD), 0);
            __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
            __rxt1_card_pci_out(rxt1_card, RXT1_HPIA + TARG_REGS, (__u32) (u_nib), 0);
            __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
            __rxt1_card_pci_out(rxt1_card, RXT1_DSP_HPIC + TARG_REGS, (__u32)(0), 0);
            __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);

        }
    }

    __rxt1_card_pci_out(rxt1_card, RXT1_HPIA + TARG_REGS, (__u32) (dsp_address), 0);
    __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);

    __rxt1_card_pci_out(rxt1_card, RXT1_HPID + TARG_REGS, (__u32) (dsp_data), 0);
    __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);

    __rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, hpi_lock, 0 );

    __rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) 0, 0);

    spin_unlock_irqrestore(&rxt1_card->reglock, flags);

    return;
}

__u16 rxt1_card_dsp_get(struct rxt1_card_t *rxt1_card, __u32 dsp_address)
{
    __u32 dsp_data; 
    __u32 u_nib;
    unsigned int hpi_lock;
    unsigned long flags;
    int hcs;
    
    spin_lock_irqsave(&rxt1_card->reglock, flags);
    hcs = (1 << rxt1_card->dsp_sel);
    __rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) (hcs), 0);

    hpi_lock = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC);
    __rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, ((hpi_lock & ~0x1F) | HPI_SEL | DSP_RST) , 0 );
    
    if (rxt1_card->dsp_type == DSP_5510) {
    
        u_nib = ((dsp_address & 0xf0000) >> 16);

        if (!(rxt1_card->hpi_xadd[rxt1_card->dsp_sel] == u_nib)) {
//            printk("rxt1: DSP %x get hpi_xadd %x u_nib %x\n", rxt1_card->dsp_sel, rxt1_card->hpi_xadd[rxt1_card->dsp_sel], u_nib);
            rxt1_card->hpi_xadd[rxt1_card->dsp_sel] = u_nib;
            
            __rxt1_card_pci_out(rxt1_card, RXT1_DSP_HPIC + TARG_REGS, (__u32) (RXT1_XADD), 0);
            __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
            __rxt1_card_pci_out(rxt1_card, RXT1_HPIA + TARG_REGS, (__u32) (u_nib), 0);
            __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
            __rxt1_card_pci_out(rxt1_card, RXT1_DSP_HPIC + TARG_REGS, (__u32)(0), 0);
            __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);

        }
    }

    __rxt1_card_pci_out(rxt1_card, RXT1_HPIA + TARG_REGS, (__u32) (dsp_address), 0);
    __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
    dsp_data = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPID);
    __rxt1_card_wait_hpi(rxt1_card, RXT1_HRDY);
    dsp_data = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIRDX);

    __rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, hpi_lock, 0 );

    __rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) 0, 0);

    spin_unlock_irqrestore(&rxt1_card->reglock, flags);

//    printk("rxt1: HPI reading %x from address %x\n", dsp_data, dsp_address);

    return (__u16) dsp_data;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadDspMemory - Read DSP memory.
 *
 * FUNCTION
 *  This function reads a contiguous block of words from DSP memory starting at
 *  the specified address.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakReadDspMemory(
    struct rxt1_card_t *rxt1_card, /* Card containing the DSP */
    unsigned short int DspId,    /* DSP Identifier (0 to MAX_DSP_CORES-1) */
    DSP_ADDRESS DspAddress,      /* DSP's memory address of first word */
    unsigned int NumWords,       /* number of contiguous words to read */
    DSP_WORD *pWordValues        /* pointer to array of word values variable */
    )
{

    unsigned int word_num;

    if (!rxt1_card) {
        printk("r%dt1: gpakReadDspMemory: No iface exists for DSP number %d\n", rxt1_card->numspans, DspId);
        return;
    }

//    spin_lock_irqsave(&rxt1_card->lock, gpak_flags);

    for (word_num=0;word_num<NumWords;word_num++) { // read NumWords from auto increment data register
        pWordValues[word_num] = rxt1_card_dsp_get(rxt1_card, DspAddress + word_num);
    }
//    spin_unlock_irqrestore(&rxt1_card->lock, gpak_flags);

    return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakWriteDspMemory - Write DSP memory.
 *
 * FUNCTION
 *  This function writes a contiguous block of words to DSP memory starting at
 *  the specified address.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakWriteDspMemory(
    struct rxt1_card_t *rxt1_card, /* Card containing the DSP */
    unsigned short int DspId,    /* DSP Identifier (0 to MAX_DSP_CORES-1) */
    DSP_ADDRESS DspAddress,      /* DSP's memory address of first word */
    unsigned int NumWords,       /* number of contiguous words to write */
    DSP_WORD *pWordValues        /* pointer to array of word values to write */
    )
{

    unsigned int word_num;
//    unsigned int rec_data;

    if (!rxt1_card) {
        printk("r%dt1: gpakWriteDspMemory: No iface exists for DSP number %d\n", rxt1_card->numspans, DspId);
        return;
    }

//    printk("rxt1 Sending %x words to DSP addresses %x to %x\n",NumWords, DspAddress, DspAddress + NumWords);
//    spin_lock_irqsave(&rxt1_card->lock, gpak_flags);

    for (word_num=0;word_num<NumWords;word_num++) { // read NumWords from auto increment data register
        rxt1_card_dsp_set(rxt1_card, DspAddress + word_num, pWordValues[word_num]);
//        printk("rxt1 writing DSP address %x with %x\n", DspAddress + word_num, pWordValues[word_num]);
    }
    
#if 0
    
    for (word_num=0;word_num<NumWords;word_num++) { // read NumWords from auto increment data register

        rec_data = rxt1_card_dsp_get(rxt1_card, DspAddress + word_num);

        if ((rec_data != pWordValues[word_num]) && ((DspAddress + word_num)!= 0x3801)) {
            printk("r%dt1: check write DSP address %x with %x should be %x\n", rxt1_card->numspans, DspAddress + word_num, rec_data, pWordValues[word_num]);
        }
    }
    
#endif
//    spin_unlock_irqrestore(&rxt1_card->lock, gpak_flags);

    return; 
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakHostDelay - Delay for a fixed time interval.
 *
 * FUNCTION
 *  This function delays for a fixed time interval before returning. The time
 *  interval is the Host Port Interface sampling period when polling a DSP for
 *  replies to command messages.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakHostDelay(void)
{
//    mdelay(1);
    msleep(5);
    return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakLockAccess - Lock access to the specified DSP.
 *
 * FUNCTION
 *  This function aquires exclusive access to the specified DSP.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakLockAccess(
    struct rxt1_card_t *rxt1_card, /* Card containing the DSP */
    unsigned short int DspId     /* DSP Identifier (0 to MAX_DSP_CORES-1) */
    )
{
//    spin_lock(&rxt1_card->lock);
    return;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakUnlockAccess - Unlock access to the specified DSP.
 *
 * FUNCTION
 *  This function releases exclusive access to the specified DSP.
 *
 * RETURNS
 *  nothing
 *
 */
void gpakUnlockAccess(
    struct rxt1_card_t *rxt1_card, /* Card containing the DSP */
    unsigned short int DspId     /* DSP Identifier (0 to MAX_DSP_CORES-1) */
    )
{
//    spin_unlock(&rxt1_card->lock);
    return;
}

extern const unsigned char _binary_GpakDsp_fw_start[];
extern const unsigned int _binary_GpakDsp_fw_size;

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 * gpakReadFile - Read a block of bytes from a G.PAK Download file.
 *
 * FUNCTION
 *  This function reads a contiguous block of bytes from a G.PAK Download file
 *  starting at the current file position.
 *
 * RETURNS
 *  The number of bytes read from the file.
 *   -1 indicates an error occurred.
 *    0 indicates all bytes have been read (end of file)
 *
 */


int gpakReadFile_5510(
    struct rxt1_card_t *rxt1_card, /* Card containing the DSP         */
    GPAK_FILE_ID FileId,           /* G.PAK Download File Identifier  */
    unsigned char *pBuffer,        /* pointer to buffer for storing bytes */
    unsigned int NumBytes          /* number of bytes to read         */
    )
{
    static int stats=0;
    int core_num;
    int byte_num=-1;
    int DspId;
    static unsigned int file_pos[MAX_DSP_CORES];
    static unsigned int file_size;
    if (!(stats)) {
        stats++;
        for(core_num=0;core_num<MAX_DSP_CORES;core_num++) {
            file_pos[core_num] = 0;
        }

        file_size = (unsigned int) &_binary_GpakDsp_fw_size;

        printk("r%dt1 %d: G168 DSP App file size = %d %x\n",rxt1_card->numspans, rxt1_card->num+1, (unsigned int) &_binary_GpakDsp_fw_size, (unsigned int) &_binary_GpakDsp_fw_size);
    }

    DspId = rxt1_card->dsp_sel+rxt1_card->num*4;

//    printk("Reading %d from %d to %d on DSP %d\n", NumBytes, file_pos[DspId], file_pos[DspId] + NumBytes, DspId);

    for (byte_num=0;byte_num<NumBytes;byte_num++) {
        if ((file_size - 1) >= file_pos[DspId]) {
            pBuffer[byte_num] = _binary_GpakDsp_fw_start[file_pos[DspId]++];
        }
    }

    return byte_num;
}



