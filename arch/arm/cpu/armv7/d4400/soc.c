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
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/d4400-regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sys_proto.h>

u8 get_silicon_rev(void)
{
	struct iim_regs *iim = (struct iim_regs *)IIM_BASE_ADDR;
	int reg = readl(&iim->srev);

	return reg;
}

u8 get_product_rev(void)
{
	struct iim_regs *iim = (struct iim_regs *)IIM_BASE_ADDR;
	int reg = readl(&iim->prev);

	return reg;
}

void init_aips(void)
{
	struct aipstz_regs *aips1, *aips2, *aips3;

	aips1 = (struct aipstz_regs *)AIPS1_BASE_ADDR;
	aips2 = (struct aipstz_regs *)AIPS2_BASE_ADDR;
	aips3 = (struct aipstz_regs *)AIPS3_BASE_ADDR;

	/*
	 * Set all MPRx to be non-bufferable, trusted for R/W,
	 * not forced to user-mode.
	 */
	writel(0x77777777, &aips1->mpr1);
	writel(0x77777777, &aips1->mpr2);

	writel(0x77777777, &aips2->mpr1);
	writel(0x77777777, &aips2->mpr2);

	writel(0x77777777, &aips3->mpr1);
	writel(0x77777777, &aips3->mpr2);

	/*
	 * Set all PACRx and OPACRx to be non-bufferable, not require
	 * supervisor privilege level for access,allow for
	 * write access and untrusted master access.
	 */
	writel(0x00000000, &aips1->pacr1);
	writel(0x00000000, &aips1->pacr2);
	writel(0x00000000, &aips1->pacr3);
	writel(0x00000000, &aips1->pacr4);

	writel(0x00000000, &aips2->pacr1);
	writel(0x00000000, &aips2->pacr2);
	writel(0x00000000, &aips2->pacr3);
	writel(0x00000000, &aips2->pacr4);

	writel(0x00000000, &aips3->pacr1);
	writel(0x00000000, &aips3->pacr2);
	writel(0x00000000, &aips3->pacr3);
	writel(0x00000000, &aips3->pacr4);

	writel(0x00000000, &aips1->opacr1);
	writel(0x00000000, &aips1->opacr2);
	writel(0x00000000, &aips1->opacr3);
	writel(0x00000000, &aips1->opacr4);
	writel(0x00000000, &aips1->opacr5);

	writel(0x00000000, &aips2->opacr1);
	writel(0x00000000, &aips2->opacr2);
	writel(0x00000000, &aips2->opacr3);
	writel(0x00000000, &aips2->opacr4);
	writel(0x00000000, &aips2->opacr5);

	writel(0x00000000, &aips3->opacr1);
	writel(0x00000000, &aips3->opacr2);
	writel(0x00000000, &aips3->opacr3);
	writel(0x00000000, &aips3->opacr4);
	writel(0x00000000, &aips3->opacr5);
}

int arch_cpu_init(void)
{
	init_aips();

	return 0;
}

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif

#if defined(CONFIG_TSEC_ENET)
void d4400_get_mac_from_fuse(int dev_id, unsigned char *mac)
{
	struct iim_fuse_t *iim_f = (struct iim_fuse_t *)(IIM_BASE_ADDR +
			IIM_FUSE_BITS_OFFSET);

	mac[0] = (unsigned char)readl(&iim_f->macaddr0);
	mac[1] = (unsigned char)readl(&iim_f->macaddr1);
	mac[2] = (unsigned char)readl(&iim_f->macaddr2);
	mac[3] = (unsigned char)readl(&iim_f->macaddr3);
	mac[4] = (unsigned char)readl(&iim_f->macaddr4);
	mac[5] = (unsigned char)readl(&iim_f->macaddr5);
}
#endif
