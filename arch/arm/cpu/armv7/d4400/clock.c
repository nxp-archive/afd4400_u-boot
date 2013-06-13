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

#include <common.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/ccm_regs.h>
#include <asm/arch/clock.h>

enum pll_clocks {
	PLL_SYS,        /* System PLL */
	PLL_DDR,        /* DDR PLL*/
	PLL_TBGEN,      /* TBGEN PLL */
	PLL_TBGEN_HALF, /* TBGEN HALF PLL */
};

enum d4400_uarts {
	UART_1 = 1,
	UART_2,
	UART_3,
	UART_4,
};

enum d4400_ecspi {
	ECSPI_1 = 1,
	ECSPI_2,
	ECSPI_3,
	ECSPI_4,
	ECSPI_5,
	ECSPI_6,
	ECSPI_7,
	ECSPI_8,
};

enum d4400_i2c {
	I2C_1 = 1,
	I2C_2,
	I2C_3,
	I2C_4,
	I2C_5,
	I2C_6,
	I2C_7,
	I2C_8,
	I2C_9,
	I2C_10,
	I2C_11,
};

enum d4400_mmdc_sel {
	MMDC_DDR_REF_BYP_CLK,
	MMDC_PLL_SYS_CLK,
	MMDC_PLL_DDR_CLK,
	MMDC_PLL_DDR_BYP_2_CLK,
};

enum d4400_vspa_dp_sel {
	VSPA_REF_SYS_BYP_CLK,
	VSPA_PLL_SYS_CLK,
	VSPA_PLL_DDR_CLK,
	VSPA_PLL_DDR_BYP_2_CLK,
};
struct d4400_ccm_reg *d4400_ccm = (struct d4400_ccm_reg *)CCM_BASE_ADDR;
static u32 get_clk_source(u32 plltype)
{
    u32 src;
    u32 gcr0_bits_19_20;
    u32 gcr0_bits_21_22;
    switch(plltype)
    {
        case PLL_SYS:
            gcr0_bits_19_20 = ((*((volatile u32 *)(GCR_0_CONFIG_REG))) >> 19) & 0x3;
            if(gcr0_bits_19_20 == 0x00)
                src = D4400_DEV_CLK;
            else if(gcr0_bits_19_20 == 0x11)
                src = D4400_SGMII_CLK;
            else
                src = 0x00;
	    break;

        case PLL_DDR:
            gcr0_bits_21_22 = ((*((volatile u32 *)(GCR_0_CONFIG_REG))) >> 21) & 0x3;
            if(gcr0_bits_21_22 == 0x00)
                src = D4400_DEV_CLK;
            else if(gcr0_bits_21_22 == 0x11)
                src = D4400_SGMII_CLK;
            else
                src = 0x00;
            break;

        case PLL_TBGEN:
            src = D4400_DEV_CLK;
            break;

        case PLL_TBGEN_HALF:
            src = D4400_DEV_CLK;
            break;

        default:
            src = 0x00;
            break;
    }
  return src;
}

/* i2c_num can be from 0 - 10 */
int enable_i2c_clk(unsigned char enable, unsigned i2c_num)
{
#ifdef CONFIG_D4400_HW_DBG
	u32 reg;
	u32 mask;
#endif
	if (i2c_num > 10)
		return -EINVAL;

#if CONFIG_D4400_HW_DBG
	/* TODO - fixup to use LPCGR registers */
	mask = MXC_CCM_CCGR_CG_MASK
		<< (MXC_CCM_CCGR2_I2C1_SERIAL_OFFSET + (i2c_num << 1));
	reg = __raw_readl(&imx_ccm->CCGR2);
	if (enable)
		reg |= mask;
	else
		reg &= ~mask;
	__raw_writel(reg, &imx_ccm->CCGR2);
#endif
	return 0;
}

static u32 decode_pll(enum pll_clocks pll)
{
	u32 mult;
	u32 src;
	src= get_clk_source(pll);
	switch (pll) {
	case PLL_SYS:
		mult = __raw_readl(&d4400_ccm->spllgsr);
		mult &= D4400_CCM_SPLLGSR_CFG_MASK;
		mult >>= D4400_CCM_SPLLGSR_CFG_OFFSET;
		break;
	case PLL_DDR:
		mult = __raw_readl(&d4400_ccm->dpllgsr);
		mult &= D4400_CCM_DPLLGSR_CFG_MASK;
		mult >>= D4400_CCM_DPLLGSR_CFG_OFFSET;
		break;
	case PLL_TBGEN:
		mult = __raw_readl(&d4400_ccm->tpllgsr);
		mult &= D4400_CCM_TPLLGSR_CFG_MASK;
		mult >>= D4400_CCM_TPLLGSR_CFG_OFFSET;
		break;
	case PLL_TBGEN_HALF:
		mult = __raw_readl(&d4400_ccm->tpllgsr);
		mult &= D4400_CCM_TPLLGSR_CFG_MASK;
		mult >>= D4400_CCM_TPLLGSR_CFG_OFFSET;
		src /= PLL_TBGEN_DIV_BY_2;
		break;
	default:
		printf("Error: Decode Pll invalid argument- "
				"0x%08x passed\n", pll);
		return 0;
	}

	/* PLL is bypassed */
	if (mult == 0)
		mult = 1;

	return src * mult;
}

static u32 get_ref_clk(void)
{
	u32 sel;

	sel = __raw_readl(&d4400_ccm->ccsr);
	sel &= D4400_CCM_CCSR_SYS_REF_SEL_MASK;
	sel >>= D4400_CCM_CCSR_SYS_REF_SEL_OFFSET;

	if (sel)
		return D4400_SGMII_CLK;
	else
		return D4400_DEV_CLK;
}

static u32 get_pllsys_or_ref_clk_freq(void)
{
	u32 sel;
	s32 freq;

	sel = __raw_readl(&d4400_ccm->ccsr);
	sel &= D4400_CCM_CCSR_SCS_MASK;
	sel >>= D4400_CCM_CCSR_SCS_OFFSET;

	if (sel)
		freq = decode_pll(PLL_SYS);
	 else
		freq = get_ref_clk();

	return freq;
}

static u32 get_mcu_main_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_ARM_DIV_MASK;
	div >>= D4400_CCM_CCDR1_ARM_DIV_OFFSET;

	return get_pllsys_or_ref_clk_freq() / (div + 1);
}

static u32 get_arm_per_clk(void)
{
	return get_mcu_main_clk() / D4400_ARM_PERCLK_DIV;
}

static u32 get_vspa_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_VSPA_DIV_MASK;
	div >>= D4400_CCM_CCDR1_VSPA_DIV_OFFSET;

	return get_pllsys_or_ref_clk_freq() / (div + 1);
}

static u32 get_ram_2x_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_AXII_DIV_MASK;
	div >>= D4400_CCM_CCDR1_AXII_DIV_OFFSET;

	return get_vspa_clk() / (div + 1);
}

u32 get_sys_bus_clk(void)
{
	return get_ram_2x_clk() / D4400_AXII_BY_2;
}

static u32 get_ahb_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_AHB_DIV_MASK;
	div >>= D4400_CCM_CCDR1_AHB_DIV_OFFSET;

	return get_sys_bus_clk() / (div + 1);
}

static u32 get_ipg_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_IP_DIV_MASK;
	div >>= D4400_CCM_CCDR1_IP_DIV_OFFSET;

	return get_ahb_clk() / (div + 1);
}

static u32 get_gpc_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_GPC_DIV_MASK;
	div >>= D4400_CCM_CCDR1_GPC_DIV_OFFSET;

	return get_ipg_clk() / (div + 1);
}

static u32 get_sync_ref_clk(void)
{
	return get_ref_clk() / D4400_SYNC_REF_DIV;
}

static u32 get_uart_clk(u32 port)
{
	u32 div, sel;

	div = __raw_readl(&d4400_ccm->ccdr2);
	sel = __raw_readl(&d4400_ccm->cscsr);

	switch (port) {
	case UART_1:
		div &= D4400_CCM_CCDR2_UART1_DIV_MASK;
		div >>= D4400_CCM_CCDR2_UART1_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range Uart1 Div 0x%08x\n", div);
			return 0;
		}

		sel &= D4400_CCM_CSCSR_UART1_MASK;
		sel >>= D4400_CCM_CSCSR_UART1_SEL_OFFSET;
		break;
	case UART_2:
		div &= D4400_CCM_CCDR2_UART2_DIV_MASK;
		div >>= D4400_CCM_CCDR2_UART2_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range Uart2 Div 0x%08x\n", div);
			return 0;
		}

		sel &= D4400_CCM_CSCSR_UART2_MASK;
		sel >>= D4400_CCM_CSCSR_UART2_SEL_OFFSET;
		break;
	case UART_3:
		div &= D4400_CCM_CCDR2_UART3_DIV_MASK;
		div >>= D4400_CCM_CCDR2_UART3_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range Uart3 Div 0x%08x\n", div);
			return 0;
		}

		sel &= D4400_CCM_CSCSR_UART3_MASK;
		sel >>= D4400_CCM_CSCSR_UART3_SEL_OFFSET;
		break;
	case UART_4:
		div &= D4400_CCM_CCDR2_UART4_DIV_MASK;
		div >>= D4400_CCM_CCDR2_UART4_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range Uart4 Div 0x%08x\n", div);
			return 0;
		}

		sel &= D4400_CCM_CSCSR_UART4_MASK;
		sel >>= D4400_CCM_CSCSR_UART4_SEL_OFFSET;
		break;
	default:
		/* NOTREACHED */
		break;
	}

	if (sel)
		return get_sys_bus_clk() / (div + 1);
	else
		return get_ref_clk() / (div + 1);
}

static u32 get_axi_clk(void)
{
	return get_ram_2x_clk();
}

static u32 get_weim_clk(void)
{
	u32 div, freq, sel;

	sel = __raw_readl(&d4400_ccm->cscsr);
	sel &= D4400_CCM_CSCSR_WEIM_MASK;
	sel >>= D4400_CCM_CSCSR_WEIM_SEL_OFFSET;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_WEIM_DIV_MASK;
	div >>= D4400_CCM_CCDR1_WEIM_DIV_OFFSET;

	if (!div) {
		printf("Error: Out of range WEIM Div %d\n", div);
		return 0;
	}

	if (sel)
		freq = decode_pll(PLL_SYS);
	else
		freq = get_ref_clk();

	return freq / (div + 1);
}

static u32 get_mmdc_clk(void)
{
	u32 div, sel, sel1;
	u32 freq = 0;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_MMDC_DIV_MASK;
	div >>= D4400_CCM_CCDR1_MMDC_DIV_OFFSET;

	if (div > D4400_CCM_CCDR1_MMDC_DIV_MAX_RANGE) {
		printf("Error: out of range MMDC DIV 0x%08x\n", div);
		return 0;
	}
	sel = __raw_readl(&d4400_ccm->cscsr);
	sel &= D4400_CCM_CSCSR_MMDC_MASK;
	sel >>= D4400_CCM_CSCSR_MMDC_SEL_OFFSET;

	switch (sel) {
	case MMDC_DDR_REF_BYP_CLK:
		sel1 = __raw_readl(&d4400_ccm->ccsr);
		sel1 &= D4400_CCM_CCSR_DDR_BYP_MASK;
		sel1 >>= D4400_CCM_CCSR_DDR_BYP_OFFSET;

		if (sel1)
			return get_ref_clk() / (div + 1);
		else
			return D4400_GPIO_DDR_CLK / (div + 1);

	case MMDC_PLL_SYS_CLK:
		return decode_pll(PLL_SYS) / (div + 1);
	case MMDC_PLL_DDR_CLK:
		return decode_pll(PLL_DDR) / (div + 1);
	case MMDC_PLL_DDR_BYP_2_CLK:
		freq = decode_pll(PLL_DDR);
		freq >>= 1;
		return freq  / (div + 1);
	default:
		printf("Error:MMDC source selection invalid argument-"
			       " 0x%08x passed\n", sel);
		return 0;
	}
}

u32 d4400_get_uart_clk(u32 port)
{
	return get_uart_clk(port);
}

static u32 get_i2c_clk(u32 port)
{
	return get_sync_ref_clk();
}

u32 d4400_get_i2c_clk(u32 port)
{
	return get_i2c_clk(port);
}

static u32 get_ecspi_clk(u32 port)
{
	u32 div, sel;

	div = __raw_readl(&d4400_ccm->ccdr3);
	sel = __raw_readl(&d4400_ccm->cscsr);

	switch (port) {
	case ECSPI_1:
		sel &= D4400_CCM_CSCSR_ECSPI1_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI1_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI1_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI1_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI1 Div 0x%08x\n", div);
			return 0;
		}

		break;
	case ECSPI_2:
		sel &= D4400_CCM_CSCSR_ECSPI2_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI2_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI2_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI2_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI2 Div 0x%08x\n", div);
			return 0;
		}
		break;
	case ECSPI_3:
		sel &= D4400_CCM_CSCSR_ECSPI3_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI3_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI3_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI3_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI3 Div 0x%08x\n", div);
			return 0;
		}

		break;
	case ECSPI_4:
		sel &= D4400_CCM_CSCSR_ECSPI4_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI4_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI4_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI4_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI4 Div 0x%08x\n", div);
			return 0;
		}

		break;
	case ECSPI_5:
		sel &= D4400_CCM_CSCSR_ECSPI5_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI5_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI5_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI5_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI5 Div 0x%08x\n", div);
			return 0;
		}

		break;
	case ECSPI_6:
		sel &= D4400_CCM_CSCSR_ECSPI6_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI6_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI6_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI6_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI6 Div 0x%08x\n", div);
			return 0;
		}

		break;
	case ECSPI_7:
		sel &= D4400_CCM_CSCSR_ECSPI7_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI7_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI7_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI7_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI7 Div 0x%08x\n", div);
			return 0;
		}

		break;
	case ECSPI_8:
		sel &= D4400_CCM_CSCSR_ECSPI8_MASK;
		sel >>= D4400_CCM_CSCSR_ECSPI8_SEL_OFFSET;

		div &= D4400_CCM_CCDR3_ECSPI8_DIV_MASK;
		div >>= D4400_CCM_CCDR3_ECSPI8_DIV_OFFSET;

		if (!div) {
			printf("Error: Out of range ECSPI8 Div 0x%08x\n", div);
			return 0;
		}

		break;
	}

	if (sel)
		return get_sys_bus_clk() / (div + 1);
	else
		return get_ref_clk() / (div + 1);

}

u32 d4400_get_ecspi_clk(u32 port)
{
	return get_ecspi_clk(port);
}

static u32 get_gated_dev_clk(void)
{
	return D4400_DEV_CLK;
}

static u32 get_bisr_clk(void)
{
	return get_gated_dev_clk() / D4400_DIV_BY_2_BISR_CLK;
}

static u32 get_etsec_rtc_clk(void)
{
	u32 sel;

	sel = __raw_readl(&d4400_ccm->cscsr);
	sel &= D4400_CCM_CSCSR_ETSEC_RTC_MASK;
	sel >>= D4400_CCM_CSCSR_ETSEC_RTC_SEL_OFFSET;

	if (sel)
		return D4400_SGMII_CLK;
	else
		return get_gated_dev_clk();
}

static u32 get_sgmii_phy_ref_clk(void)
{
	u32 div;

	div = __raw_readl(&d4400_ccm->ccdr2);
	div &= D4400_CCM_CCDR2_SGMII_PHY_CLK_MASK;
	div >>= D4400_CCM_CCDR2_SGMII_PHY_CLK_OFFSET;

	return D4400_SGMII_CLK / (div + 1);
}

static u32 get_sync_ckil_clk(void)
{
	return D4400_CKIL_CLK;
}

u32 d4400_get_sync_ckil(void)
{
	return get_sync_ckil_clk();
}

static u32 get_async_ckil_clk(void)
{
	return get_sync_ckil_clk();
}

static u32 get_cpri1_axi_clk(void)
{
	return get_sys_bus_clk();
}

static u32 get_cpri2_axi_clk(void)
{
	return get_sys_bus_clk();
}

static u32 get_etsec_axi_clk(void)
{
	return get_sys_bus_clk();
}

#ifdef CONFIG_D4400_HW_DBG
static u32 get_ref_sys_byp_clk(void)
{
	u32 sel;

	sel = __raw_readl(&d4400_ccm->ccsr);
	sel &= D4400_CCM_CCSR_BYP_SYS_MASK;
	sel >>= D4400_CCM_CCSR_BYP_SYS_OFFSET;

	if (sel)
		return D4400_CORE_BYPASS_CLK;
	else
		return get_ref_clk();

}
#endif

static u32 get_vspa_dp_clk(void)
{
	u32 div, sel;

	div = __raw_readl(&d4400_ccm->ccdr1);
	div &= D4400_CCM_CCDR1_VSPA_DP_DIV_MASK;
	div >>= D4400_CCM_CCDR1_VSPA_DP_DIV_OFFSET;

	sel = __raw_readl(&d4400_ccm->cscsr);
	sel &= D4400_CCM_CSCSR_VSPA_DP_MASK;
	sel >>= D4400_CCM_CSCSR_VSPA_DP_SEL_OFFSET;

	switch (sel) {
	case VSPA_REF_SYS_BYP_CLK:
		return get_sys_bus_clk() / (div + 1);
	case VSPA_PLL_SYS_CLK:
		return decode_pll(PLL_SYS) / (div + 1);
	case VSPA_PLL_DDR_CLK:
		return decode_pll(PLL_DDR) / (div + 1);
	case VSPA_PLL_DDR_BYP_2_CLK:
		return decode_pll(PLL_DDR) /
					(D4400_PLL_DDR_BY_2 * (div + 1));
	}
	return 0;
}

static u32 get_jesd204tx_clk(void)
{
	return decode_pll(PLL_TBGEN);
}

static u32 get_jesd204tx_cpri_clk(void)
{
	return decode_pll(PLL_TBGEN);
}

static u32 get_jesd204rx_clk(void)
{
	return decode_pll(PLL_TBGEN_HALF);
}

static u32 get_jesd204rx_cpri_clk(void)
{
	return decode_pll(PLL_TBGEN_HALF);
}

static u32 get_tbgen_clk(void)
{
	return decode_pll(PLL_TBGEN);
}

static u32 get_tbgen_dev_clk(void)
{
	return get_gated_dev_clk();
}

static u32 get_debug_clk(void)
{
	return get_ipg_clk();
}

static u32 get_ccm_at_clk(void)
{
	u32 div, sel, clk;

	sel = __raw_readl(&d4400_ccm->cscsr);
	sel &= D4400_CCM_CSCSR_TRACE_MASK;
	sel >>= D4400_CCM_CSCSR_TRACE_SEL_OFFSET;

	switch (sel) {
	case 0: /* mmdc_clk */
		clk = get_mmdc_clk();
		break;
	case 1: /* gated_pll_sys_clk_tpiu */
		clk = decode_pll(PLL_SYS);
		break;
	case 2: /* tbgen_pll_clk */
		clk = get_tbgen_clk();
		break;
	default:
		printf("Error: out of range TRACE_SEL 0x%08x\n", sel);
		return 0;
	}

	div = __raw_readl(&d4400_ccm->ccdr2);
	div &= D4400_CCM_CCDR2_TRACE_DIV_MASK;
	div >>= D4400_CCM_CCDR2_TRACE_DIV_OFFSET;

	return clk / (div + 1);
}

unsigned int d4400_get_clock(enum mxc_clock clk, unsigned int port)
{
	switch (clk) {
	case MXC_ARM_CLK:
		return get_mcu_main_clk();
	case MXC_PER_CLK:
		return get_arm_per_clk();
	case MXC_AHB_CLK:
		return get_ahb_clk();
	case MXC_IPG_CLK:
		return get_ipg_clk();
	case MXC_UART_CLK:
		return get_uart_clk(port);
	case MXC_UART1_CLK:
		return get_uart_clk(UART_1);
	case MXC_UART2_CLK:
		return get_uart_clk(UART_2);
	case MXC_UART3_CLK:
		return get_uart_clk(UART_3);
	case MXC_UART4_CLK:
		return get_uart_clk(UART_4);
	case MXC_AXI_CLK:
		return get_axi_clk();
	case MXC_DDR_CLK:
		return get_mmdc_clk();
	case MXC_VSPA_CLK:
		return get_vspa_clk();
	case MXC_RAM_2X_CLK:
		return get_ram_2x_clk();
	case MXC_SYS_BUS_CLK:
		return get_sys_bus_clk();
	case MXC_GPC_CLK:
		return get_gpc_clk();
	case MXC_REF_CLK:
		return get_ref_clk();
	case MXC_WEIM_CLK:
		return get_weim_clk();
	case MXC_ETSEC_AXI_CLK:
		return get_etsec_axi_clk();
	case MXC_ETSEC_RTC_CLK:
		return get_etsec_rtc_clk();
	case MXC_SGMII_PHY_REF_CLK:
		return get_sgmii_phy_ref_clk();
	case MXC_SYNC_REF_CLK:
		return get_sync_ref_clk();
	case MXC_I2C_CLK:
		return get_i2c_clk(port);
	case MXC_I2C1_CLK:
		return get_i2c_clk(I2C_1);
	case MXC_I2C2_CLK:
		return get_i2c_clk(I2C_2);
	case MXC_I2C3_CLK:
		return get_i2c_clk(I2C_3);
	case MXC_I2C4_CLK:
		return get_i2c_clk(I2C_4);
	case MXC_I2C5_CLK:
		return get_i2c_clk(I2C_5);
	case MXC_I2C6_CLK:
		return get_i2c_clk(I2C_6);
	case MXC_I2C7_CLK:
		return get_i2c_clk(I2C_7);
	case MXC_I2C8_CLK:
		return get_i2c_clk(I2C_8);
	case MXC_I2C9_CLK:
		return get_i2c_clk(I2C_9);
	case MXC_I2C10_CLK:
		return get_i2c_clk(I2C_10);
	case MXC_I2C11_CLK:
		return get_i2c_clk(I2C_11);
	case MXC_CSPI_CLK:
	case MXC_ECSPI_CLK:
		return get_ecspi_clk(port);
	case MXC_ECSPI1_CLK:
		return get_ecspi_clk(ECSPI_1);
	case MXC_ECSPI2_CLK:
		return get_ecspi_clk(ECSPI_2);
	case MXC_ECSPI3_CLK:
		return get_ecspi_clk(ECSPI_3);
	case MXC_ECSPI4_CLK:
		return get_ecspi_clk(ECSPI_4);
	case MXC_ECSPI5_CLK:
		return get_ecspi_clk(ECSPI_5);
	case MXC_ECSPI6_CLK:
		return get_ecspi_clk(ECSPI_6);
	case MXC_ECSPI7_CLK:
		return get_ecspi_clk(ECSPI_7);
	case MXC_ECSPI8_CLK:
		return get_ecspi_clk(ECSPI_8);
	case MXC_MMDC_CLK:
		return get_mmdc_clk();
	case MXC_BISR_CLK:
		return get_bisr_clk();
	case MXC_ASYNC_CKIL_CLK:
		return get_async_ckil_clk();
	case MXC_SYNC_CKIL_CLK:
		return get_sync_ckil_clk();
	case MXC_DEBUG_CLK:
		return get_debug_clk();
	case MXC_CCM_AT_CLK:
		return get_ccm_at_clk();
	case MXC_CPRI2_AXI_CLK:
		return get_cpri2_axi_clk();
	case MXC_CPRI1_AXI_CLK:
		return get_cpri1_axi_clk();
	case MXC_VSPA_DP_CLK:
		return get_vspa_dp_clk();
	case MXC_JESD204RX_CLK:
		return get_jesd204rx_clk();
	case MXC_JESD204TX_CLK:
		return get_jesd204tx_clk();
	case MXC_JESD204TX_CPRI_CLK:
		return get_jesd204tx_cpri_clk();
	case MXC_JESD204RX_CPRI_CLK:
		return get_jesd204rx_cpri_clk();
	case MXC_TBGEN_CLK:
		return get_tbgen_clk();
	case MXC_TBGEN_DEV_CLK:
		return get_tbgen_dev_clk();
	default:
		printf("Error: Invalid clk argument passed %d\n", clk);
		return 0;
	}
}

unsigned int mxc_get_clock(enum mxc_clock clk)
{
	return d4400_get_clock(clk, 0);
}

unsigned int mxc_get_clock_bus(enum mxc_clock clk, int bus)
{
	return d4400_get_clock(clk, bus+1);
}

/*
 * Dump some core clocks.
 */
int do_d4400_showclocks(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	u32 freq;
	freq = decode_pll(PLL_SYS);
	printf("PLL_SYS        %8d MHz\n", (freq + 500000) / 1000000);
	freq = decode_pll(PLL_DDR);
	printf("PLL_DDR        %8d MHz\n", (freq + 500000) / 1000000);
	freq = decode_pll(PLL_TBGEN);
	printf("PLL_TBGEN      %8d MHz\n", (freq + 500000) / 1000000);
	freq = decode_pll(PLL_TBGEN_HALF);
	printf("PLL_TBGEN_HALF %8d MHz\n", (freq + 500000) / 1000000);

	printf("\n");
	printf("ARM            %8d kHz\n",
		(d4400_get_clock(MXC_ARM_CLK, 0) + 500) / 1000);
	printf("PER            %8d kHz\n",
		(d4400_get_clock(MXC_PER_CLK, 0) + 500) / 1000);
	printf("AHB            %8d kHz\n",
		(d4400_get_clock(MXC_AHB_CLK, 0) + 500) / 1000);
	printf("IPG            %8d kHz\n",
		(d4400_get_clock(MXC_IPG_CLK, 0) + 500) / 1000);
	printf("UART1          %8d kHz\n",
		(d4400_get_clock(MXC_UART1_CLK, 0) + 500) / 1000);
	printf("UART2          %8d kHz\n",
		(d4400_get_clock(MXC_UART2_CLK, 0) + 500) / 1000);
	printf("UART3          %8d kHz\n",
		(d4400_get_clock(MXC_UART3_CLK, 0) + 500) / 1000);
	printf("UART4          %8d kHz\n",
		(d4400_get_clock(MXC_UART4_CLK, 0) + 500) / 1000);
	printf("AXI            %8d kHz\n",
		(d4400_get_clock(MXC_AXI_CLK, 0) + 500) / 1000);
	printf("DDR            %8d kHz\n",
		(d4400_get_clock(MXC_DDR_CLK, 0) + 500) / 1000);
	printf("VSPA           %8d kHz\n",
		(d4400_get_clock(MXC_VSPA_CLK, 0) + 500) / 1000);
	printf("RAM 2X         %8d kHz\n",
		(d4400_get_clock(MXC_RAM_2X_CLK, 0) + 500) / 1000);
	printf("SYS            %8d kHz\n",
		(d4400_get_clock(MXC_SYS_BUS_CLK, 0) + 500) / 1000);
	printf("GPC            %8d kHz\n",
		(d4400_get_clock(MXC_GPC_CLK, 0) + 500) / 1000);
	printf("REF            %8d kHz\n",
		(d4400_get_clock(MXC_REF_CLK, 0) + 500) / 1000);
	printf("WEIM           %8d kHz\n",
		(d4400_get_clock(MXC_WEIM_CLK, 0) + 500) / 1000);
	printf("ETSEC AXI      %8d kHz\n",
		(d4400_get_clock(MXC_ETSEC_AXI_CLK, 0) + 500) / 1000);
	printf("ETSEC RTC      %8d kHz\n",
		(d4400_get_clock(MXC_ETSEC_RTC_CLK, 0) + 500) / 1000);
	printf("SYNC_REF       %8d kHz\n",
		(d4400_get_clock(MXC_SYNC_REF_CLK, 0) + 500) / 1000);
	printf("I2C1           %8d kHz\n",
		(d4400_get_clock(MXC_I2C1_CLK, 0) + 500) / 1000);
	printf("I2C2           %8d kHz\n",
		(d4400_get_clock(MXC_I2C2_CLK, 0) + 500) / 1000);
	printf("I2C3           %8d kHz\n",
		(d4400_get_clock(MXC_I2C3_CLK, 0) + 500) / 1000);
	printf("I2C4           %8d kHz\n",
		(d4400_get_clock(MXC_I2C4_CLK, 0) + 500) / 1000);
	printf("I2C5           %8d kHz\n",
		(d4400_get_clock(MXC_I2C5_CLK, 0) + 500) / 1000);
	printf("I2C6           %8d kHz\n",
		(d4400_get_clock(MXC_I2C6_CLK, 0) + 500) / 1000);
	printf("I2C7           %8d kHz\n",
		(d4400_get_clock(MXC_I2C7_CLK, 0) + 500) / 1000);
	printf("I2C8           %8d kHz\n",
		(d4400_get_clock(MXC_I2C8_CLK, 0) + 500) / 1000);
	printf("I2C9           %8d kHz\n",
		(d4400_get_clock(MXC_I2C9_CLK, 0) + 500) / 1000);
	printf("I2C10          %8d kHz\n",
		(d4400_get_clock(MXC_I2C10_CLK, 0) + 500) / 1000);
	printf("I2C11          %8d kHz\n",
		(d4400_get_clock(MXC_I2C11_CLK, 0) + 500) / 1000);
	printf("ECSPI1         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI1_CLK, 0) + 500) / 1000);
	printf("ECSPI2         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI2_CLK, 0) + 500) / 1000);
	printf("ECSPI3         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI3_CLK, 0) + 500) / 1000);
	printf("ECSPI4         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI4_CLK, 0) + 500) / 1000);
	printf("ECSPI5         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI5_CLK, 0) + 500) / 1000);
	printf("ECSPI6         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI6_CLK, 0) + 500) / 1000);
	printf("ECSPI7         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI7_CLK, 0) + 500) / 1000);
	printf("ECSPI8         %8d kHz\n",
		(d4400_get_clock(MXC_ECSPI8_CLK, 0) + 500) / 1000);
	printf("TBGEN DEV      %8d kHz\n",
		(d4400_get_clock(MXC_TBGEN_DEV_CLK, 0) + 500) / 1000);
	printf("TBGEN          %8d kHz\n",
		(d4400_get_clock(MXC_TBGEN_CLK, 0) + 500) / 1000);
	printf("JESD204RX      %8d kHz\n",
		(d4400_get_clock(MXC_JESD204RX_CLK, 0) + 500) / 1000);
	printf("JESD204TX      %8d kHz\n",
		(d4400_get_clock(MXC_JESD204TX_CLK, 0) + 500) / 1000);
	printf("JESD204TX CPRI %8d kHz\n",
		(d4400_get_clock(MXC_JESD204TX_CPRI_CLK, 0) + 500) / 1000);
	printf("JESD204RX CPRI %8d kHz\n",
		(d4400_get_clock(MXC_JESD204RX_CPRI_CLK, 0) + 500) / 1000);
	printf("VSPA DP        %8d kHz\n",
		(d4400_get_clock(MXC_VSPA_DP_CLK, 0) + 500) / 1000);
	printf("CPRI1 AXI      %8d kHz\n",
		(d4400_get_clock(MXC_CPRI1_AXI_CLK, 0) + 500) / 1000);
	printf("CPRI2 AXI      %8d kHz\n",
		(d4400_get_clock(MXC_CPRI2_AXI_CLK, 0) + 500) / 1000);
	printf("CCM AT         %8d kHz\n",
		(d4400_get_clock(MXC_CCM_AT_CLK, 0) + 500) / 1000);
	printf("DEBUG          %8d kHz\n",
		(d4400_get_clock(MXC_DEBUG_CLK, 0) + 500) / 1000);
	printf("SYNC CKIL      %8d kHz\n",
		(d4400_get_clock(MXC_SYNC_CKIL_CLK, 0) + 500) / 1000);
	printf("ASYNC CKIL     %8d kHz\n",
		(d4400_get_clock(MXC_ASYNC_CKIL_CLK, 0) + 500) / 1000);
	printf("BISR           %8d kHz\n",
		(d4400_get_clock(MXC_BISR_CLK, 0) + 500) / 1000);
	printf("MMDC           %8d kHz\n",
		(d4400_get_clock(MXC_MMDC_CLK, 0) + 500) / 1000);
	printf("SGMII PHY REF  %8d kHz\n",
		(d4400_get_clock(MXC_SGMII_PHY_REF_CLK, 0) + 500) / 1000);
	return 0;
}

/***************************************************/

U_BOOT_CMD(
	clocks, CONFIG_SYS_MAXARGS, 1, do_d4400_showclocks,
	"display clocks",
	""
);
