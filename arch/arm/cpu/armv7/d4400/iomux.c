/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/d4400-regs.h>
#include <asm/arch/iomux.h>

static void *base = (void *)IOMUXC_BASE_ADDR;
static void *base_ddr = (void *)IOMUXC_DDR_BASE_ADDR;
/*
 * configures a single pad in the iomuxer
 */
int d4400_iomux_setup_pad(iomux_cfg_t pad)
{
	u32 mux_ctrl_ofs = (pad & MUX_CTRL_OFS_MASK) >> MUX_CTRL_OFS_SHIFT;
	u32 mux_mode = (pad & MUX_MODE_MASK) >> MUX_MODE_SHIFT;
	u32 pad_ctrl_ofs =
		(pad & MUX_PAD_CTRL_OFS_MASK) >> MUX_PAD_CTRL_OFS_SHIFT;
	u32 pad_ctrl = (pad & MUX_PAD_CTRL_MASK) >> MUX_PAD_CTRL_SHIFT;
	u32 ddr_pad = (pad & DDR_PAD_CTRL_MASK) >> DDR_PAD_CTRL_SHIFT;

	if (!ddr_pad) {
		if (mux_ctrl_ofs)
			__raw_writeb(mux_mode, base + mux_ctrl_ofs);

		if ((!(pad_ctrl & NO_PAD_CTRL)) && pad_ctrl_ofs)
			__raw_writew(pad_ctrl, base + pad_ctrl_ofs);
	} else {
		if (pad_ctrl_ofs)
			__raw_writew(pad_ctrl, base_ddr + pad_ctrl_ofs);
	}

	return 0;
}

int d4400_iomux_setup_multiple_pads(iomux_cfg_t *pad_list, unsigned count)
{
	iomux_cfg_t *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = d4400_iomux_setup_pad(*p);

		if (ret)
			return ret;
		p++;
	}

	return 0;
}
