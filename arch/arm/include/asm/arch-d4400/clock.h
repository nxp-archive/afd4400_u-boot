/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
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

#ifndef __ASM_ARCH_D4400_CLOCK_H
#define __ASM_ARCH_D4400_CLOCK_H

#include <common.h>

#ifdef PALLADIUM_SETUP
#define D4400_DEV_CLK		125000000
#else
#define D4400_DEV_CLK		122880000
#endif
#define D4400_SGMII_CLK		125000000
#define D4400_GPIO_DDR_CLK	2000000
#define D4400_CORE_BYPASS_CLK	2000000

#define D4400_CKIL_CLK		30000

/* CCM Ranges*/
#define D4400_CCM_CCDR1_PLL_SYS_DIV_MAX_RANGE 9
#define D4400_CCM_CCDR1_MMDC_DIV_MAX_RANGE    9
#define D4400_CCM_CCDR2_GEN_CLK_MAX_RANGE     7
#define PLL_TBGEN_DIV_BY_2                    2
#define D4400_ARM_PERCLK_DIV                  2
#define D4400_AXII_BY_2	                      2
#define D4400_SYNC_REF_DIV                    3
#define D4400_PLL_DDR_BY_2                    2
#define D4400_DIV_BY_2_BISR_CLK               2

enum mxc_clock {
	MXC_ARM_CLK = 0,
	MXC_PER_CLK,
	MXC_AHB_CLK,
	MXC_IPG_CLK,
	MXC_UART_CLK,
	MXC_UART1_CLK,
	MXC_UART2_CLK,
	MXC_UART3_CLK,
	MXC_UART4_CLK,
	MXC_AXI_CLK,
	MXC_DDR_CLK,
	MXC_VSPA_CLK,
	MXC_RAM_2X_CLK,
	MXC_SYS_BUS_CLK,
	MXC_GPC_CLK,
	MXC_REF_CLK,
	MXC_WEIM_CLK,
	MXC_ETSEC_AXI_CLK,
	MXC_ETSEC_RTC_CLK,
	MXC_SYNC_REF_CLK,
	MXC_I2C_CLK,
	MXC_I2C1_CLK,
	MXC_I2C2_CLK,
	MXC_I2C3_CLK,
	MXC_I2C4_CLK,
	MXC_I2C5_CLK,
	MXC_I2C6_CLK,
	MXC_I2C7_CLK,
	MXC_I2C8_CLK,
	MXC_I2C9_CLK,
	MXC_I2C10_CLK,
	MXC_I2C11_CLK,
	MXC_CSPI_CLK,
	MXC_ECSPI_CLK,
	MXC_ECSPI1_CLK,
	MXC_ECSPI2_CLK,
	MXC_ECSPI3_CLK,
	MXC_ECSPI4_CLK,
	MXC_ECSPI5_CLK,
	MXC_ECSPI6_CLK,
	MXC_ECSPI7_CLK,
	MXC_ECSPI8_CLK,
	MXC_TBGEN_DEV_CLK,
	MXC_TBGEN_CLK,
	MXC_JESD204RX_CLK,
	MXC_JESD204TX_CLK,
	MXC_JESD204TX_CPRI_CLK,
	MXC_JESD204RX_CPRI_CLK,
	MXC_VSPA_DP_CLK,
	MXC_CPRI1_AXI_CLK,
	MXC_CPRI2_AXI_CLK,
	MXC_CCM_AT_CLK,
	MXC_DEBUG_CLK,
	MXC_SYNC_CKIL_CLK,
	MXC_ASYNC_CKIL_CLK,
	MXC_BISR_CLK,
	MXC_MMDC_CLK,
	MXC_SGMII_PHY_REF_CLK,
};

unsigned int d4400_get_clock(enum mxc_clock clk, u32 port);
u32 d4400_get_uart_clk(u32 port);
u32 d4400_get_i2c_clk(u32 port);
u32 d4400_get_ecspi_clk(u32 port);
u32 d4400_get_sync_ckil(void);

unsigned int mxc_get_clock(enum mxc_clock clk);
unsigned int mxc_get_clock_bus(enum mxc_clock clk, int bus);
int enable_i2c_clk(unsigned char enable, unsigned i2c_num);

#endif /* __ASM_ARCH_D4400_CLOCK_H */
