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

#ifndef _R4FXO_H
#define _R4FXO_H

#include "version.h"

#include <linux/ioctl.h>
#ifdef STANDALONE_ZAPATA
#include "zaptel.h"
#else
#include <zaptel/zaptel.h>
#endif
#include <linux/moduleparam.h>

/*
 *  Define for audio vs. register based ring detection
 *
 */
/* #define AUDIO_RINGCHECK  */

#define SPI_SUPPORT

static int loopcurrent = 20;

static int reversepolarity = 0;

static struct fxo_mode {
    char *name;
    /* FXO */
    int ohs;   // ms hookspeed - loop current slew rate
    int ohs2;  // ls hookspeed
    int rz;    // ring impedance 0 = high
    int rt;    // ring thresh
    int ilim;  // 60 mA current limit enble
    int dcv;   // tip - ring dc voltage
    int mini;  // minimum loop current
    int acim;  // ac impedance
    int ring_osc;
    int ring_x;
} fxo_modes[] =
{ //   name          ohs ohs2 rz rt ilim  dcv  mini acim ring_osc ring_x
    { "FCC",         0,  0,   0, 1, 0,    0x3, 0,   0,   },   /* US, Canada */
    { "TBR21",       0,  0,   0, 0, 1,    0x3, 0,   0x2, 0x7e6c,  0x023a, },
 /* Austria, Belgium, Denmark, Finland, France, Germany,
    Greece, Iceland, Ireland, Italy, Luxembourg, Netherlands,
    Norway, Portugal, Spain, Sweden, Switzerland, and UK */
    { "ARGENTINA",   0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "AUSTRALIA",   1,  0,   0, 0, 0,    0,   0x3, 0x3, },
    { "AUSTRIA",     0,  1,   0, 0, 1,    0x3, 0,   0x3, },
    { "BAHRAIN",     0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "BELGIUM",     0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "BRAZIL",      0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "BULGARIA",    0,  0,   0, 0, 1,    0x3, 0x0, 0x3, },
    { "CANADA",      0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "CHILE",       0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "CHINA",       0,  0,   0, 0, 0,    0,   0x3, 0xf, },
    { "COLUMBIA",    0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "CROATIA",     0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "CYRPUS",      0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "CZECH",       0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "DENMARK",     0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "ECUADOR",     0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "EGYPT",       0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "ELSALVADOR",  0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "FINLAND",     0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "FRANCE",      0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "GERMANY",     0,  1,   0, 0, 1,    0x3, 0,   0x3, },
    { "GREECE",      0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "GUAM",        0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "HONGKONG",    0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "HUNGARY",     0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "ICELAND",     0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "INDIA",       0,  0,   0, 0, 0,    0x3, 0,   0x4, },
    { "INDONESIA",   0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "IRELAND",     0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "ISRAEL",      0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "ITALY",       0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "JAPAN",       0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "JORDAN",      0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "KAZAKHSTAN",  0,  0,   0, 0, 0,    0x3, 0,        },
    { "KUWAIT",      0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "LATVIA",      0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "LEBANON",     0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "LUXEMBOURG",  0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "MACAO",       0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "MALAYSIA",    0,  0,   0, 0, 0,    0,   0x3, 0,   },  /* Current loop >= 20ma */
    { "MALTA",       0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "MEXICO",      0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "MOROCCO",     0,  0,   0, 0, 1,    0x3, 0,   0x2, },
    { "NETHERLANDS", 0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "NEWZEALAND",  0,  0,   0, 0, 0,    0x3, 0,   0x4, },
    { "NIGERIA",     0,  0,   0, 0, 0x1,  0x3, 0,   0x2, },
    { "NORWAY",      0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "OMAN",        0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "PAKISTAN",    0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "PERU",        0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "PHILIPPINES", 0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "POLAND",      0,  0,   1, 1, 0,    0x3, 0,   0,   },
    { "PORTUGAL",    0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "ROMANIA",     0,  0,   0, 0, 0,    0x3, 0,0,      },
    { "RUSSIA",      0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "SAUDIARABIA", 0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "SINGAPORE",   0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "SLOVAKIA",    0,  0,   0, 0, 0,    0x3, 0,   0x3, },
    { "SLOVENIA",    0,  0,   0, 0, 0,    0x3, 0,   0x2, },
    { "SOUTHAFRICA", 1,  0,   1, 0, 0,    0x3, 0,   0x3, },
    { "SOUTHKOREA",  0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "SPAIN",       0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "SWEDEN",      0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "SWITZERLAND", 0,  1,   0, 0, 1,    0x3, 0,   0x2, },
    { "SYRIA",       0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "TAIWAN",      0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "THAILAND",    0,  0,   0, 0, 0,    0,   0x3, 0,   },
    { "UAE",         0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "UK",          0,  1,   0, 0, 1,    0x3, 0,   0x5, },
    { "USA",         0,  0,   0, 0, 0,    0x3, 0,   0,   },
    { "YEMEN",       0,  0,   0, 0, 0,    0x3, 0,   0,   },
};


#define NUM_FXO_REGS 60

#define RH_MAX_IFACES 128
#define PCI_VENDOR_RHINO 0xb0b
#define PCI_DEVICE_R4FXO 0x205

#define R4FXO_STATE      0x800
#define R4FXO_VERSION    0x802
#define R4FXO_CONTROL    0x804
#define R4FXO_INTSTAT    0x805
#define R4FXO_BUFLEN     0x806
#define R4FXO_RXBUFSTART 0x808
#define R4FXO_TXBUFSTART 0x80C
#define R4FXO_JTAG       0x810
#define RH_OLDSPI        0x814
#define RH_SPI           0x814
#define RH_CSR           0x815
#define SPI_WR           0x818
#define SPI_RD           0x81C
#define R4FXO_SIZE       0x820

#define RH_SYNC     0x0
#define RH_TEST     0x1
#define RH_CS       0x2
#define RH_VER      0x3

#define BIT_CS      (1 << 2)
#define BIT_SCLK    (1 << 3)
#define BIT_SDI     (1 << 4)
#define BIT_SDO     (1 << 5)
#define BIT_RST     (1 << 6)
#define BIT_DMAGO   (1 << 0)
#define INT_ACK     (1 << 1)

#define STOP_DMA    (1 << 0)
#define FREE_DMA    (1 << 1)
#define IOUNMAP     (1 << 2)
#define ZUNREG      (1 << 3)
#define FREE_INT    (1 << 4)
#define RH_KFREE    (1 << 5)
#define PCI_FREE    (1 << 6)
#define RST_SLAB    (1 << 7)

#define FLAG_EMPTY  0
#define FLAG_WRITE  1
#define FLAG_READ   2

/* the constants below control the 'debounce' periods enforced by the
   check_hook routines; these routines are called once every 4 interrupts
   (the interrupt cycles around the four modules), so the periods are
   specified in _4 millisecond_ increments
*/
#define RING_DEBOUNCE       4       /* Ringer Debounce (64 ms) */
#define DEFAULT_BATT_DEBOUNCE   4       /* Battery debounce (64 ms) */
#define POLARITY_DEBOUNCE   4       /* Polarity debounce (64 ms) */
#define DEFAULT_BATT_THRESH 3       /* Anything under this is "no battery" */

#define OHT_TIMER 6000    /* How long after RING to retain OHT */

#define FLAG_3215 (1 << 0)

#define NUM_CARDS 4

#define MAX_ALARMS 10

#define MOD_TYPE_FXS    0
#define MOD_TYPE_FXO    1

#define MINPEGTIME  10 * 8      /* 30 ms peak to peak gets us no more than 100 Hz */
#define PEGTIME     50 * 8      /* 50ms peak to peak gets us rings of 10 Hz or more */
#define PEGCOUNT    5       /* 5 cycles of pegging means RING */

#define NUM_CAL_REGS 12

// Voice DAA constants
#define CONTROL1  1
#define SRST    (1 << 7)

#define CONTROL2  2
#define RXE     (1 << 0)
#define HBE     (1 << 1)

#define DAACON1   5
#define OFFHOOK (1 << 0)
#define ALWAYS1 (1 << 1)
#define RDT     (1 << 2)
#define ONHM    (1 << 3)
#define RDTP    (1 << 5)
#define RDTN    (1 << 6)

#define DAACON2   6
#define PUP       0

#define LSIDSSREV 11
#define SSREV 0x0f
#define LSID  0xf0

#define LSSTAT    12
#define FDT     (1 << 6)
#define ARZ     (1 << 5)

#define LSREV     13

#define INTERNAT1 16

#define DCTERM    26

#define LINEVOLTS 0x1d

#define ACTERM    30

#define DAACON5   31
#define FULL    (1 << 7)
#define FOH128  (3 << 5)
#define FILT    (1 << 1)
#define LVFD    (1 << 0)

#define COMMODE   33

#define PCME    (1 << 5)
#define MULAW   (1 << 3)

#define PCMTXSLO  34
#define PCMTXSHI  35
#define PCMRXSLO  36
#define PCMRXSHI  37

#define TXGAIN2   38

#define HYBTAP1   45
#define HYBTAP2   46
#define HYBTAP3   47
#define HYBTAP4   48
#define HYBTAP5   49
#define HYBTAP6   50
#define HYBTAP7   51
#define HYBTAP8   52


#define NUM_REGS      109
#define NUM_INDIRECT_REGS 105

struct r4fxo_stats {
    int tipvolt;    /* TIP voltage (mV) */
    int ringvolt;   /* RING voltage (mV) */
    int batvolt;    /* VBAT voltage (mV) */
};

struct r4fxo_regs {
    unsigned char direct[NUM_REGS];
    unsigned short indirect[NUM_INDIRECT_REGS];
};

struct r4fxo_regop {
    int indirect;
    unsigned char reg;
    unsigned short val;
};

struct r4fxo_echo_coefs {
    unsigned char acim;
    unsigned char coef1;
    unsigned char coef2;
    unsigned char coef3;
    unsigned char coef4;
    unsigned char coef5;
    unsigned char coef6;
    unsigned char coef7;
    unsigned char coef8;
};

#define R4FXO_GET_STATS _IOR (ZT_CODE, 60, struct r4fxo_stats)
#define R4FXO_GET_REGS  _IOR (ZT_CODE, 61, struct r4fxo_regs)
#define R4FXO_SET_REG   _IOW (ZT_CODE, 62, struct r4fxo_regop)
#define R4FXO_SET_ECHOTUNE _IOW (ZT_CODE, 63, struct r4fxo_echo_coefs)
#define addr_t dma_addr_t
struct r4fxo {
    struct pci_dev *dev;
    char *variety;
    struct zt_span span;
    unsigned char ios;
    int usecount;
    unsigned int intcount;
    int dead;
    int pos;
    int flags[NUM_CARDS];
    int freeregion;
    int alt;
    int curcard;
    int cardflag;       /* Bit-map of present cards */
    spinlock_t lock;

    union {
        struct {
#ifdef AUDIO_RINGCHECK
            unsigned int pegtimer;
            int pegcount;
            int peg;
            int ring;
#else
            int wasringing;
#endif
            int ringdebounce;
            int offhook;
            int battdebounce;
            int nobatttimer;
            int battery;
                int lastpol;
                int polarity;
                int polaritydebounce;
        } fxo;
    } mod[NUM_CARDS];

    /* Receive hook state and debouncing */
    int modtype[NUM_CARDS];
    int comerr[NUM_CARDS];
    int lastcomerr[NUM_CARDS];
    unsigned char reg0shadow[NUM_CARDS];
    unsigned char reg1shadow[NUM_CARDS];
    addr_t baseaddr;
    void* memaddr;
    dma_addr_t  readdma;
    dma_addr_t  writedma;
    volatile unsigned char *writechunk;                   /* Double-word aligned write memory */
    volatile unsigned char *readchunk;                    /* Double-word aligned read memory */
    struct zt_chan chans[NUM_CARDS];
};

struct r4fxo_desc {
    char *name;
    int flags;
};

static struct r4fxo_desc r4fxo = { "Rhino R4FXO Rev. A", 0 };

static struct r4fxo *ifaces[RH_MAX_IFACES];

static void r4fxo_release(struct r4fxo *rh);

static int battdebounce = DEFAULT_BATT_DEBOUNCE;
static int battthresh = DEFAULT_BATT_THRESH;
static int debug = 0;
static int timingonly = 0;
static int lowpower = 0;
static int boostringer = 0;
static int fastringer = 0;
static int _opermode = 0;
static char *opermode = "FCC";
static int alawoverride = 0;

void *mmap(void *, size_t, int, int, int, off_t);
int open(const char *path, int oflags);
static void r4fxo_restart_dma(struct r4fxo *rh);

#define DEBUG

#ifdef DEBUG
#define MSG(string, args...) if (debug) printk("r4fxo: " string, ##args)
#else
#define MSG(string, args...)
#endif

#endif /* _R4FXO_H */


