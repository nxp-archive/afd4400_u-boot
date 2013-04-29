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
#include <asm/arch/d4400-regs.h>
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

	printf("CPU:Freescale D4400 prev-0x%x, srev-0x%x"
		"at %d MHz\n",
		prev, srev,
		d4400_get_clock(D4400_ARM_CLK, 0) / 1000000);
	printf("Reset cause: 0x%08x\n", get_reset_cause());
	return 0;
}
#endif

int cpu_eth_init(bd_t *bis)
{
	int rc = -ENODEV;
	return rc;
}

void reset_cpu(ulong addr)
{
	__raw_writew(4, WDOG_BASE_ADDR);
}
