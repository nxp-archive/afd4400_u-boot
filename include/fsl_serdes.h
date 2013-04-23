/*
 *  fsl_serdes.h
 *
 *  Driver for the SerDes
 *
 *  This software may be used and distributed according to the
 *  terms of the GNU Public License, Version 2, incorporated
 *  herein by reference.
 *
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 */

#ifndef __FSL_SERDES_H
#define __FSL_SERDES_H

#define BOOT_ETH_MODE_SGMII 0

#define SRC_SBMR_ETH_MODE_MASK 0x0C00
#define SRC_SBMR_ETH_MODE_SHIFT 0x0A

struct serdes_regs {
	volatile unsigned int pll1_rstctl_offs; /* 0x0 */
	/* u-boot does not use these registers marking as reserved */
	volatile unsigned char reserved0[0x1c];
	volatile unsigned int pll2_rstctl_offs; /* 0x20 */
};

#endif /* __FSL_SERDES_H */
