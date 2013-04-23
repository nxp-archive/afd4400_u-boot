/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __ASM_ARCH_D4400_REGS_H__
#define __ASM_ARCH_D4400_REGS_H__

#define ARCH_D4400

#define CONFIG_SYS_CACHELINE_SIZE	32

#define ROMCP_BASE1_ADDR		0x00000000
#define ROMCP_END1_ADDR			0x00003FFF
#define ROMCP_BASE2_ADDR		0x00404000
#define ROMCP_END2_ADDR			0x00433FFF

#define AIPS1_BASE_ADDR			0x01000000
#define AIPS1_END_ADDR			0x011FFFFF
#define AIPS2_BASE_ADDR			0x01200000
#define AIPS2_END_ADDR			0x03FFFFFF
#define AIPS3_BASE_ADDR			0x04000000
#define AIPS3_END_ADDR			0x05FFFFFF

#define OCRAM_BASE_ADDR			0x10000000
#define OCRAM_END_ADDR			0x1000FFFF
#define L2_CACHE_BASE_ADDR		0x10900000
#define L2_CACHE_END_ADDR		0x10903FFF
#define WEIM_BASE_ADDR			0x30000000
#define WEIM_END_ADDR			0x4FFFFFFF
#define OCRAM1_BASE_ADDR		0x60000000
#define OCRAM1_END_ADDR			0x6000FFFF
#define OCRAM2_BASE_ADDR		0x61000000
#define OCRAM2_END_ADDR			0x6100FFFF
#define OCRAM3_BASE_ADDR		0x70000000
#define OCRAM3_END_ADDR			0x7000FFFF
#define OCRAM4_BASE_ADDR		0x71000000
#define OCRAM4_END_ADDR			0x7100FFFF
#define MMDC_BASE_ADDR			0x90000000
#define MMDC_END_ADDR			0xEFFFFFFF
#define OCRAM5_BASE_ADDR		0xF0000000
#define OCRAM5_END_ADDR			0xF000FFFF
#define OCRAM6_BASE_ADDR		0xF1000000
#define OCRAM6_END_ADDR			0xF100FFFF

/* Defines for Blocks connected via AIPS */
#define ATZ1_BASE_ADDR		AIPS1_BASE_ADDR
#define ATZ2_BASE_ADDR		AIPS2_BASE_ADDR
#define ATZ3_BASE_ADDR		AIPS3_BASE_ADDR

#define AIPS1_ON_BASE_ADDR	(ATZ1_BASE_ADDR)
#define AIPS1_OFF_BASE_ADDR	(ATZ1_BASE_ADDR + 0x80000)

#define MMDC_CTRL_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x0000)
#define SERDES2_BASE_ADDR       (AIPS1_OFF_BASE_ADDR + 0x8000)
#define SRC_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0xC000)
#define CCM_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x14000)
#define WEIM_CTRL_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x18000)
#define ROMCP_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x1C000)
#define WDOG_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x20000)
#define ECSPI1_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x24000)
#define ECSPI2_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x28000)
#define ECSPI3_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x2C000)
#define ECSPI4_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x30000)
#define ECSPI5_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x34000)
#define ECSPI6_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x38000)
#define ECSPI7_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x3C000)
#define ECSPI8_BASE_ADDR	(AIPS1_OFF_BASE_ADDR + 0x40000)
#define I2C1_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x44000)
#define I2C2_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x48000)
#define I2C3_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x4C000)
#define I2C4_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x50000)
#define GPIO1_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x54000)
#define GPIO2_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x58000)
#define GPIO3_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x5C000)
#define GPIO4_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x60000)
#define GPIO5_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x64000)
#define EPIT_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x68000)
#define UART1_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x6C000)
#define UART2_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x70000)
#define UART3_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x74000)
#define UART4_BASE_ADDR		(AIPS1_OFF_BASE_ADDR + 0x78000)

#define AIPS2_ON_BASE_ADDR	(ATZ2_BASE_ADDR)
#define AIPS2_OFF_BASE_ADDR	(ATZ2_BASE_ADDR + 0x80000)

#define SCM_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x40000)
#define I2C5_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x5C000)
#define I2C6_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x60000)
#define I2C7_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x64000)
#define I2C8_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x68000)
#define I2C9_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x6C000)
#define I2C10_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x70000)
#define I2C11_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x74000)
#define IIM_BASE_ADDR		(AIPS2_OFF_BASE_ADDR + 0x78000)

#define AIPS3_ON_BASE_ADDR	(ATZ3_BASE_ADDR)
#define AIPS3_OFF_BASE_ADDR	(ATZ3_BASE_ADDR + 0x80000)

#define VETSEC0_GROUP0_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x0000)
#define VETSEC1_GROUP0_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x1000)
#define VETSEC0_MDIO_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x4000)
#define IOMUXC_DDR_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0xC000)
#define GPC_BASE_ADDR		(AIPS3_OFF_BASE_ADDR + 0x18000)
#define IOMUXC_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x1C000)
#define VETSEC0_GROUP1_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x20000)
#define VETSEC1_GROUP1_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x21000)
#define VETSEC1_MDIO_BASE_ADDR	(AIPS3_OFF_BASE_ADDR + 0x24000)

#define WEIM_CONFIGURATION_REG               (WEIM_CTRL_BASE_ADDR + 0x90)
#define WEIM_GENERAL_CONFIGURATION_REG_1     (WEIM_CTRL_BASE_ADDR + 0x00)
#define WEIM_GENERAL_CONFIGURATION_REG_2     (WEIM_CTRL_BASE_ADDR + 0x04)
#define WEIM_READ_CONFIGURATION_REG_1        (WEIM_CTRL_BASE_ADDR + 0x08)
#define WEIM_READ_CONFIGURATION_REG_2        (WEIM_CTRL_BASE_ADDR + 0x0C)
#define WEIM_WRITE_CONFIGURATION_REG_1       (WEIM_CTRL_BASE_ADDR + 0x10)
#define GCR_51_CONFIG_REG		     (SCM_BASE_ADDR + 0xE8)

#define IRAM_BASE_ADDR	OCRAM_BASE_ADDR
#define IRAM_SIZE	(OCRAM_END_ADDR - OCRAM_BASE_ADDR)
#define IIM_FUSE_BITS_OFFSET	0x800

#if !(defined(__KERNEL_STRICT_NAMES) || defined(__ASSEMBLY__))
#include <asm/types.h>

/* System Reset Controller (SRC) */
struct src {
	u32	sbmr;
	u32	srsr;
	u32	sscr;
	u32	scrcr;
	u32	srbr;
	u32	sgpr;
};

struct iim_fuse_t {
	u32 reserved0[2];
	u32 ext_eth_phy_addr;
	u32 reserved1[3];
	u32 srec;
	u32 reserved2[39];
	u32 macaddr5;
	u32 macaddr4;
	u32 macaddr3;
	u32 macaddr2;
	u32 macaddr1;
	u32 macaddr0;
};

struct iim_regs {
	u32	stat;
	u32	statm;
	u32	err;
	u32	emask;
	u32	fctl;
	u32	ua;
	u32	la;
	u32	sdat;
	u32	prev;
	u32	srev;
	u32	prg_p;
	u32	scs0;
	u32	scs1;
	u32	scs2;
	u32	scs3;
	struct	iim_fuse_t	iim_fuse;
};

struct aipstz_regs {
	u32	mpr1;
	u32	mpr2;
	u32	rsvd1[6];
	u32	pacr1;
	u32	pacr2;
	u32	pacr3;
	u32	pacr4;
	u32	rsvd2[4];
	u32	opacr1;
	u32	opacr2;
	u32	opacr3;
	u32	opacr4;
	u32	opacr5;
};

struct iomuxc_base_regs {
	u32	swmux_ctl;
	u32	gpio_swpad_ctl;
	u32	ddr_swpad_ctl;
};

#endif /* __ASSEMBLER__*/
#endif /* __ASM_ARCH_D4400_REGS_H__ */
