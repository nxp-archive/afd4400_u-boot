/*
 * (C) Copyright 2013 Freescale Semiconductor, Inc.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <div64.h>
#include <asm/arch/d4400-regs.h>
#include <asm/arch/clock.h>

/* Enhanced periodic interrupt timers registers */
struct d4400_epit {
	unsigned int control;
	unsigned int status;
	unsigned int load;
	unsigned int compare;
	unsigned int counter;
};

static struct d4400_epit *cur_epit = (struct d4400_epit *)EPIT_BASE_ADDR;

/* EPIT timers bitfields */
#define EPITCR          0x00
#define EPITSR          0x04
#define EPITLR          0x08
#define EPITCMPR        0x0c
#define EPITCNR         0x10

#define EPITCR_EN                       (1 << 0)
#define EPITCR_ENMOD                    (1 << 1)
#define EPITCR_OCIEN                    (1 << 2)
#define EPITCR_RLD                      (1 << 3)
#define EPITCR_PRESC(x)                 (((x) & 0xfff) << 4)
#define EPITCR_SWR                      (1 << 16)
#define EPITCR_IOVW                     (1 << 17)
#define EPITCR_DBGEN                    (1 << 18)
#define EPITCR_WAITEN                   (1 << 19)
#define EPITCR_RES                      (1 << 20)
#define EPITCR_STOPEN                   (1 << 21)
#define EPITCR_OM_DISCON                (0 << 22)
#define EPITCR_OM_TOGGLE                (1 << 22)
#define EPITCR_OM_CLEAR                 (2 << 22)
#define EPITCR_OM_SET                   (3 << 22)
#define EPITCR_CLKSRC_OFF               (0 << 24)
#define EPITCR_CLKSRC_PERIPHERAL        (1 << 24)
#define EPITCR_CLKSRC_REF_HIGH          (2 << 24)
#define EPITCR_CLKSRC_REF_LOW           (3 << 24)

#define EPITSR_OCIF                     (1 << 0)


DECLARE_GLOBAL_DATA_PTR;

#define timestamp (gd->arch.tbl)
#define lastinc (gd->arch.lastinc)

static inline unsigned long long tick_to_time(unsigned long long tick)
{
	tick *= CONFIG_SYS_HZ;
	do_div(tick, d4400_get_sync_ckil());

	return tick;
}

static inline unsigned long long us_to_tick(unsigned long long usec)
{
	usec = usec * d4400_get_sync_ckil() + 999999;
	do_div(usec, 1000000);

	return usec;
}

int timer_init(void)
{
/*
* Initialise to a known state (all timers off, and timing reset)
*/
	__raw_writel(0x0, &cur_epit->control);

	__raw_writel(0xffffffff, &cur_epit->load);
	__raw_writel(EPITCR_EN | EPITCR_CLKSRC_REF_LOW | EPITCR_WAITEN,
		&cur_epit->control);

	lastinc = __raw_readl(&cur_epit->counter);
	timestamp = 0;

	return 0;
}

unsigned long long get_ticks(void)
{
	ulong now = __raw_readl(&cur_epit->counter); /* current tick value */

	if (now <= lastinc) {
		/*
		 * normal mode (non roll)
		 * move stamp downward with absolute diff ticks
		 */
		timestamp += (lastinc - now);
	} else {
		/* we have rollover of decrementer */
		timestamp += (0xFFFFFFFF - now) + lastinc;
	}
	lastinc = now;
	return timestamp;
}

ulong get_timer_masked(void)
{
	/*
	 * get_ticks() returns a long long (64 bit), it wraps in
	 * 2^64 / d4400_get_sync_ckil() ~ 2^64 / 2^15 = 2^49 ~ 5 * 10^14 (s) ~
	 * 5 * 10^9 days... and get_ticks() * CONFIG_SYS_HZ wraps in
	 * 5 * 10^6 days - long enough.
	 */
	return tick_to_time(get_ticks());
}

ulong get_timer(ulong base)
{
    return get_timer_masked() - base;
}

/* delay x useconds AND preserve advance timstamp value */
void __udelay(unsigned long usec)
{
	unsigned long long tmp;
	ulong tmo;

	tmo = us_to_tick(usec);
	tmp = get_ticks() + tmo;	/* get current timestamp */

	while (get_ticks() < tmp)	/* loop till event */
		 /*NOP*/;
}

/*
 * This function is derived from PowerPC code (timebase clock frequency).
 * On ARM it returns the number of timer ticks per second.
 */
ulong get_tbclk(void)
{
	return d4400_get_sync_ckil();
}
