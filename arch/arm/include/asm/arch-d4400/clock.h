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
#define D4400_REF_DIV                         2
#define D4400_PLL_DDR_BY_2                    2
#define D4400_DIV_BY_2_BISR_CLK               2

enum d4400_clock {
	D4400_ARM_CLK = 0,
	D4400_PER_CLK,
	D4400_AHB_CLK,
	D4400_IPG_CLK,
	D4400_UART1_CLK,
	D4400_UART2_CLK,
	D4400_UART3_CLK,
	D4400_UART4_CLK,
	D4400_AXI_CLK,
	D4400_DDR_CLK,
	D4400_VSPA_CLK,
	D4400_RAM_CLK,
	D4400_SYS_BUS_CLK,
	D4400_GPC_CLK,
	D4400_REF_CLK,
	D4400_WEIM_CLK,
	D4400_ETSEC_AXI_CLK,
	D4400_ETSEC_RTC_CLK,
	D4400_SYNC_REF_CLK,
	D4400_I2C1_CLK,
	D4400_I2C2_CLK,
	D4400_I2C3_CLK,
	D4400_I2C4_CLK,
	D4400_I2C5_CLK,
	D4400_I2C6_CLK,
	D4400_I2C7_CLK,
	D4400_I2C8_CLK,
	D4400_I2C9_CLK,
	D4400_I2C10_CLK,
	D4400_I2C11_CLK,
	D4400_ECSPI1_CLK,
	D4400_ECSPI2_CLK,
	D4400_ECSPI3_CLK,
	D4400_ECSPI4_CLK,
	D4400_ECSPI5_CLK,
	D4400_ECSPI6_CLK,
	D4400_ECSPI7_CLK,
	D4400_ECSPI8_CLK,
	D4400_TBGEN_DEV_CLK,
	D4400_TBGEN_CLK,
	D4400_JESD204RX_CLK,
	D4400_JESD204TX_CLK,
	D4400_JESD204TX_CPRI_CLK,
	D4400_JESD204RX_CPRI_CLK,
	D4400_VSPA_DP_CLK,
	D4400_CPRI1_AXI_CLK,
	D4400_CPRI2_AXI_CLK,
	D4400_CCM_AT_CLK,
	D4400_DEBUG_CLK,
	D4400_SYNC_CKIL_CLK,
	D4400_ASYNC_CKIL_CLK,
	D4400_BISR_CLK,
	D4400_MMDC_CLK
};

unsigned int d4400_get_clock(enum d4400_clock clk, u32 port);
u32 d4400_get_uart_clk(u32 port);
u32 d4400_get_i2c_clk(u32 port);
u32 d4400_get_ecspi_clk(u32 port);
u32 d4400_get_sync_ckil(void);
#endif /* __ASM_ARCH_D4400_CLOCK_H */
