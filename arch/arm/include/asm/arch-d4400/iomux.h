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

#ifndef __MACH_IOMUX_H__
#define __MACH_IOMUX_H__

/*
 *	Build IOMUX_PAD Structure
 *
 * This iomux scheme is based around pads, which are the physical balls
 * on the processor.
 *
 * - Each pad has a pad control register (IOMUXC_SW_PAD_CTRL_x) which controls
 *   things like driving strength and pullup/pulldown.
 * - Each pad can have but not necessarily does have an output routing register
 *   (IOMUXC_SW_MUX_CTL_PAD_x).
 *
 * The three register sets do not have a fixed offset to each other,
 * hence we order this table by pad control registers (which all pads
 * have) and put the optional i/o routing registers into additional
 * fields.
 *
 * The naming convention for the pad modes is SOC_PAD_<padname>__<padmode>
 * If <padname> or <padmode> refers to a GPIO, it is named GPIO_<unit>_<num>
 *
 * IOMUX/PAD Bit field definitions
 *
 * MUX_CTRL_OFS:	    0..11 (12)
 * PAD_CTRL_OFS:	   12..23 (12)
 * MUX_MODE:               24..30 (07)
 * PAD_CTRL:               31..45 (15)
 * DDR_PAD:		   46..46 (01)
 * reserved:               47..63 (17)
*/

typedef u64 iomux_cfg_t;

#define MUX_CTRL_OFS_SHIFT	0
#define MUX_CTRL_OFS_MASK	((iomux_cfg_t)0xfff << MUX_CTRL_OFS_SHIFT)

#define MUX_PAD_CTRL_OFS_SHIFT	12
#define MUX_PAD_CTRL_OFS_MASK	((iomux_cfg_t)0xfff << \
						MUX_PAD_CTRL_OFS_SHIFT)
#define MUX_MODE_SHIFT		24
#define MUX_MODE_MASK		((iomux_cfg_t)0x7f << MUX_MODE_SHIFT)

#define MUX_PAD_CTRL_SHIFT	31
#define MUX_PAD_CTRL_MASK	((iomux_cfg_t)0x7fff << MUX_PAD_CTRL_SHIFT)

#define MUX_PAD_CTRL(x)		((iomux_cfg_t)(x) << MUX_PAD_CTRL_SHIFT)

#define DDR_PAD_CTRL_SHIFT	46
#define DDR_PAD_CTRL_MASK	((iomux_cfg_t)1 << DDR_PAD_CTRL_SHIFT)

#define NO_PAD_CTRL             45

#define IOMUX_PAD(pad_ctrl_ofs, mux_ctrl_ofs, mux_mode, \
		 pad_ctrl, ddr_pad)					\
	(((iomux_cfg_t)(mux_ctrl_ofs) << MUX_CTRL_OFS_SHIFT)     |	\
	((iomux_cfg_t)(mux_mode)      << MUX_MODE_SHIFT)         |	\
	((iomux_cfg_t)(pad_ctrl_ofs)  << MUX_PAD_CTRL_OFS_SHIFT) |	\
	((iomux_cfg_t)(pad_ctrl)      << MUX_PAD_CTRL_SHIFT)     |	\
	((iomux_cfg_t)(ddr_pad)       << DDR_PAD_CTRL_SHIFT))


int d4400_iomux_setup_pad(iomux_cfg_t pad);
int d4400_iomux_setup_multiple_pads(iomux_cfg_t *pad_list, unsigned count);

#endif	/* __MACH_IOMUX_H__*/
