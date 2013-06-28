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

#define SERDES_PLL1_RST_REQ 0x80000000
#define SERDES_PLL1_RST_DONE 0x40000000
#define SERDES_PLL1_RST_ERR 0x20000000

struct serdes_regs {
	unsigned int pll1_rstctl_offs; /* 0x0 */
	/* u-boot does not use these registers marking as reserved */
	unsigned char reserved0[0x1c];
	unsigned int pll2_rstctl_offs; /* 0x20 */
};

#endif /* __FSL_SERDES_H */
