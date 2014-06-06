/*
 * (C) Copyright 2013 Freescale Semiconductor, Inc.
 * Partha Hazra <b43678@freescale.com>
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
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/ccm_regs.h>

u32 get_reset_cause(void)
{
	u32 cause;
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;

	cause = readl(&src_regs->srsr);
	writel(cause, &src_regs->srsr);

	return cause;
}

#if defined(CONFIG_DISPLAY_CPUINFO)
int print_cpuinfo(void)
{
	u8 prev;
	u8 srev;

	prev = get_product_rev();
	srev = get_silicon_rev();

	printf("CPU:Freescale D4400 prev-0x%x, srev-0x%x at %d MHz\n",
	       prev, srev,
	       d4400_get_clock(MXC_ARM_CLK, 0) / 1000000);
	printf("Reset cause: 0x%08x\n",
	       get_reset_cause());
	printf("DDR Clock:  %dMHz\n",
	       d4400_get_clock(MXC_DDR_CLK, 0) / 1000000);
	printf("ARM Clock:  %dMHz\n",
	       d4400_get_clock(MXC_ARM_CLK, 0) / 1000000);
	printf("VSPA Clock: %dMHz\n",
	       d4400_get_clock(MXC_VSPA_CLK, 0) / 1000000);

	return 0;
}
#endif

int cpu_eth_init(bd_t *bis)
{
	int rc = -ENODEV;
	return rc;
}

// Address of Debug block for each VSPA
static u32 vspa_debug[] = {
	0x2001E800, // VSPA_DBG1_GDBEN
	0x2001F800, // VSPA_DBG2_GDBEN
	0x20020800, // VSPA_DBG3_GDBEN
	0x20021800, // VSPA_DBG4_GDBEN
	0x20026800, // VSPA_DBG5_GDBEN
	0x20027800, // VSPA_DBG6_GDBEN
	0x20028800, // VSPA_DBG7_GDBEN
	0x20022800, // VSPA_DBG8_GDBEN
	0x20023800, // VSPA_DBG9_GDBEN
	0x20024800, // VSPA_DBG10_GDBEN
	0x20025800  // VSPA_DBG11_GDBEN
};

void reset_cpu(ulong addr)
{
	int i;
	u32* dbg_ptr;

	// Halt each VSPA core to avoid a power spike on hard reset
	for (i = 0; i < (sizeof(vspa_debug)/sizeof(u32)); i++) {
		dbg_ptr = (u32*)vspa_debug[i];
		writel(0x1, &dbg_ptr[0]); // GDBEN : enable invasive debug mode
		writel(0x4, &dbg_ptr[1]); // RCR : force VSPA into halt mode
		udelay(5000);
	}
	// Use watchdog to force a reset
	__raw_writew(4, WDOG_BASE_ADDR);
}
