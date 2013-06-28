/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Partha Hazra <b44332@freescale.com>
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
#include <asm/arch/imx-regs.h>
#include <asm/arch/d4400_pins.h>
#include <asm/arch/ccm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/arch/iomux.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/clock.h>
#include <tsec.h>
#include <fsl_mdio.h>
#include <asm/arch/fsl_serdes.h>
#include <asm/arch/i2c.h>
#include <i2c.h>

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSL_1  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSL_1   | PAD_CTL_HYS)

#ifndef CONFIG_D4400_EVB_MDC_BUG_SW_FIX
#define MDIO_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSL_2   | PAD_CTL_HYS | PAD_CTL_ODE)
#else
#define MDIO_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSL_2   | PAD_CTL_HYS)
#endif

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSL_1 | PAD_CTL_HYS | PAD_CTL_ODE)

#if defined(CONFIG_CMD_WEIM_NOR) && defined(CONFIG_QIXIS)
enum FPGA_GVDD_VSEL {
	MANUAL_1_8_V_SEL,
	MANUAL_2_5_V_SEL,
	MANUAL_3_0_V_SEL,
	MANUAL_3_3_V_SEL,
};

#define QIXIS_PWR_CTL2_REG_OFFSET 0x21
#define QIXIS_GVDD_REG_MASK 0x3
#define QIXIS_GVDDA_REG_OFFSET 0x0
#define QIXIS_GVDDB_REG_OFFSET 0x2
#define QIXIS_GVDDC_REG_OFFSET 0x4
#endif
int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

#ifdef CONFIG_D4400_UART
iomux_cfg_t uart4_pads[] = {
	D4400_PAD_UART4_TXD_UART4_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	D4400_PAD_UART4_RXD_UART4_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	d4400_iomux_setup_multiple_pads(uart4_pads, ARRAY_SIZE(uart4_pads));
}
#endif

#ifdef CONFIG_CMD_WEIM_NOR
/*
 * CS1 pin is used as Address line A26.
 * It's used in collapsed mode.
 */
iomux_cfg_t nor_pads[] = {
	D4400_PAD_FLASH_DAT0_EIM_DATA0 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT1_EIM_DATA1 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT2_EIM_DATA2 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT3_EIM_DATA3 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT4_EIM_DATA4 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT5_EIM_DATA5 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT6_EIM_DATA6 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT7_EIM_DATA7 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT8_EIM_DATA8 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT9_EIM_DATA9 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT10_EIM_DATA10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT11_EIM_DATA11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT12_EIM_DATA12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT13_EIM_DATA13 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT14_EIM_DATA14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_DAT15_EIM_DATA15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A0_EIM_A0 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A1_EIM_A1 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A2_EIM_A2 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A3_EIM_A3 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A4_EIM_A4 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A5_EIM_A5 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A6_EIM_A6 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A7_EIM_A7 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A8_EIM_A8 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A9_EIM_A9 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A10_EIM_A10 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A11_EIM_A11 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A12_EIM_A12 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A13_EIM_A13 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A14_EIM_A14 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A15_EIM_A15 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A16_EIM_A16 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A17_EIM_A17 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A18_EIM_A18 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A19_EIM_A19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A20_EIM_A20 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A21_EIM_A21 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A22_EIM_A22 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A23_EIM_A23 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A24_EIM_A24 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_A25_EIM_A25 | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_CS0_B_EIM_CS0_B | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_CS1_B_EIM_CS1_B | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_OE_B_EIM_OE_B | MUX_PAD_CTRL(NO_PAD_CTRL),
	D4400_PAD_FLASH_WE_B_EIM_WE_B | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#ifdef PALLADIUM_SETUP
static void weim_norflash_cs_setup(void)
{
	/* Memory WDOG enable, External interrupt polarity high */
	writel(0x00000120, WEIM_CONFIGURATION_REG);
	/* CS0 enable, Non Mux, Data port D[0-15], CRE signal Active High */
	writel(0x00010081, WEIM_GENERAL_CONFIGURATION_REG_1);
	/* 2 Cycles of Address hold time  */
	writel(0x00000002, WEIM_GENERAL_CONFIGURATION_REG_2);
	writel(0x16020000, WEIM_READ_CONFIGURATION_REG_1);
	writel(0x00000000, WEIM_READ_CONFIGURATION_REG_2);
	writel(0x1c092480, WEIM_WRITE_CONFIGURATION_REG_1);
}
#else
/*
 * WEIM Bus connect Nor Flash in Async mode
 * 16 bit port resides on DATA[15:0]: Non Mux mode
 * Nor memory - [0-256]MB
 * Chip select - CS0
 * collapsed mode.
 */
static void weim_norflash_cs_setup(void)/* FIXME-Settings may change */
{
	/* Collapsed mode */
	writel(0x00800000, GCR_51_CONFIG_REG);

	/* Memory WDOG enable, External interrupt polarity high */
	/* CS0 enable, Non Mux, Data port D[0-15], CRE signal Active low */
	writel(0x10110001, WEIM_CS0_GENERAL_CONFIGURATION_REG_1);
	/* 2 Cycles of Address hold time  */
	writel(0x00000002, WEIM_CS0_GENERAL_CONFIGURATION_REG_2);
	/* Read wait 12 Cycles */
	writel(0x0c082000, WEIM_CS0_READ_CONFIGURATION_REG_1);
	/* Write wait  0 Cycles, BE assert- 1 Cycle, BE negate - 1 Cycle
	 * WE assert -1 Cycle, WE negate - 1 Cycle
	 */
	writel(0x0d009240, WEIM_CS0_WRITE_CONFIGURATION_REG_1);

	/* Memory WDOG enable, External interrupt polarity high */
	/* CS1 enable, Non Mux, Data port D[0-15], CRE signal Active low */
	writel(0x10110001, WEIM_CS1_GENERAL_CONFIGURATION_REG_1);
	/* 2 Cycles of Address hold time  */
	writel(0x00000002, WEIM_CS1_GENERAL_CONFIGURATION_REG_2);
	/* Read wait 12 Cycles */
	writel(0x0c082000, WEIM_CS1_READ_CONFIGURATION_REG_1);
	/* Write wait  0 Cycles, BE assert- 1 Cycle, BE negate - 1 Cycle
	 * WE assert -1 Cycle, WE negate - 1 Cycle
	 */
	writel(0x0d009240, WEIM_CS1_WRITE_CONFIGURATION_REG_1);
}
#endif

static void setup_weim_nor(void)
{
	d4400_iomux_setup_multiple_pads(nor_pads, ARRAY_SIZE(nor_pads));
	weim_norflash_cs_setup();
}
#endif

#ifdef CONFIG_QIXIS
iomux_cfg_t qixis_pads[] = {
	D4400_PAD_FLASH_CS2_B_EIM_CS2_B | MUX_PAD_CTRL(NO_PAD_CTRL)
};

static void setup_weim_qixis(void)
{
	d4400_iomux_setup_multiple_pads(qixis_pads, ARRAY_SIZE(qixis_pads));

	writel(0x000400C1, WEIM_CS2_GENERAL_CONFIGURATION_REG_1);
	writel(0x3F007000, WEIM_CS2_READ_CONFIGURATION_REG_1);
	writel(0x0f000000, WEIM_CS2_WRITE_CONFIGURATION_REG_1);
}
#endif

#ifdef CONFIG_TSEC_ENET
iomux_cfg_t enet_pads[] = {
	/* PIN settings for RGMII TSEC1 */
	D4400_PAD_GPIO_C24_TSEC1_RXD_1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C25_TSEC1_RXD_2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C26_TSEC1_RXD_3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C27_TSEC1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C28_TSEC1_GTX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C29_TSEC1_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C30_TSEC1_TXD_0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_C31_TSEC1_TXD_1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D00_TSEC1_TXD_2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D01_TSEC1_TXD_3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D02_TSEC1_TX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D03_TSEC1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D04_TSEC1_COL | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D05_TSEC1_CRS | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D06_TSEC1_RX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D07_TSEC1_RX_DV | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_GPIO_D08_TSEC1_RXD_0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

iomux_cfg_t mdio_pads[] = {
	D4400_PAD_TSEC_MDC_GPIO_D09 | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	D4400_PAD_TSEC_MDIO_GPIO_D10 | MUX_PAD_CTRL(MDIO_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	d4400_iomux_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
}

static void setup_iomux_mdio(void)
{
	d4400_iomux_setup_multiple_pads(mdio_pads, ARRAY_SIZE(mdio_pads));
}

static void setup_serdes_sgmii_mode(void)
{
	struct serdes_regs *serdes_regs =
		(struct serdes_regs *)(SERDES2_BASE_ADDR);
	int mask, pll1_rstctl_offs;
	int timeout = 1000000;

	/* Soft reset SERDES PLL1*/
	pll1_rstctl_offs = readl(&serdes_regs->pll1_rstctl_offs);
	pll1_rstctl_offs &= ~(SERDES_PLL1_RST_REQ);
	writel(pll1_rstctl_offs, &serdes_regs->pll1_rstctl_offs);

	pll1_rstctl_offs = readl(&serdes_regs->pll1_rstctl_offs);
	pll1_rstctl_offs |= SERDES_PLL1_RST_REQ;
	writel(pll1_rstctl_offs, &serdes_regs->pll1_rstctl_offs);

	/* RST_DONE should be 1, RST_ERR should be 0 */
	mask = (readl(&(serdes_regs->pll1_rstctl_offs)) &
			(SERDES_PLL1_RST_DONE | SERDES_PLL1_RST_ERR));
	while (mask != SERDES_PLL1_RST_DONE && timeout--)
		mask = (readl(&(serdes_regs->pll1_rstctl_offs)) &
				(SERDES_PLL1_RST_DONE | SERDES_PLL1_RST_ERR));

	if (0 == timeout)
		printf("SERDES reset failed\n");

}

static void setup_enet(void)
{

	struct d4400_ccm_reg *ccm_regs =
		(struct d4400_ccm_reg *) CCM_BASE_ADDR;

	setup_iomux_mdio();

	/* Configure clock divider for 25MHz to 125MHz conversion */
	unsigned int ccdr2 = ccm_regs->ccdr2;
	ccdr2 &= ~D4400_CCM_CCDR2_SGMII_PHY_CLK_MASK;
	ccdr2 |= (0x4 << D4400_CCM_CCDR2_SGMII_PHY_CLK_OFFSET);
	ccm_regs->ccdr2 = ccdr2;

	/* Checking BOOT MODE */
	/* If BOOT mode is SGMII setup SERDES */

//	if (BOOT_ETH_MODE_SGMII == d4400_get_eth0_mode())
	if (d4400_get_tsec_flags() & TSEC_SGMII) {
		setup_serdes_sgmii_mode();
	}
	/* If BOOT mode is RMII/RGMII setup IOMUX */
	else {
		setup_iomux_enet();
	}
}

int board_eth_init(bd_t *bis)
{
	struct fsl_pq_mdio_info mdio_info;
	struct tsec_info_struct tsec_info[2];

	int num = 0;

	int flags = d4400_get_tsec_flags();

#ifdef CONFIG_TSEC1
	SET_STD_TSEC_INFO(tsec_info[num], 1);
	tsec_info[num].flags = flags;
	if (flags & TSEC_SGMII)
		printf("SGMII: ");
	else if (flags & TSEC_GIGABIT) {
		tsec_info[num].phyaddr = TSEC1_PHY_ADDR_RGMII;
		printf("RGMII: ");
	}
	else if (flags & TSEC_REDUCED) {
		tsec_info[num].phyaddr = TSEC1_PHY_ADDR_RMII;
		printf("RMII: ");
	}
	else
		printf("MII: ");
	num++;
#endif

#ifdef CONFIG_TSEC2
	if (flags & TSEC_SGMII) {
		SET_STD_TSEC_INFO(tsec_info[num], 2);
		tsec_info[num].flags = flags;
		num++;
	}
#endif

	if (!num) {
		printf("No TSECs initialized\n");
		return 0;
	}

	mdio_info.regs = (struct tsec_mii_mng *)CONFIG_SYS_MDIO_BASE_ADDR;
	mdio_info.name = DEFAULT_MII_NAME;

	fsl_pq_mdio_init(bis, &mdio_info);
	if (num != tsec_eth_init(bis, tsec_info, num))
		printf("TSEC : Unable to register initialize TSEC\n");


	return 0;
}
#endif

#ifdef CONFIG_MXC_SPI

#define SPI_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_PUS_100K_UP | PAD_CTL_DSL_3)

iomux_cfg_t spi_pads[] = {
	/* RoC1 */
	D4400_PAD_SPI1_MOSI_eCSPI1_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI1_MISO_eCSPI1_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI1_CLK_eCSPI1_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI1_SS0_eCSPI1_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI1_SS1_eCSPI1_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOC0_eCSPI1_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	/* RoC2 */
	D4400_PAD_SPI3_MOSI_eCSPI3_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI3_MISO_eCSPI3_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI3_CLK_eCSPI3_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI3_SS0_eCSPI3_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI3_SS1_eCSPI3_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOC4_eCSPI3_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	/* PA1 */
	D4400_PAD_SPI5_MOSI_eCSPI5_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI5_MISO_eCSPI5_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI5_CLK_eCSPI5_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI5_SS0_eCSPI5_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI5_SS1_eCSPI5_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA28_eCSPI5_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA29_eCSPI5_SS3 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	/* PA2 */
	D4400_PAD_SPI6_MOSI_eCSPI6_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI6_MISO_eCSPI6_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI6_CLK_eCSPI6_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI6_SS0_eCSPI6_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI6_SS1_eCSPI6_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA26_eCSPI6_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA27_eCSPI6_SS3 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	/* PA3 */
	D4400_PAD_SPI7_MOSI_eCSPI7_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI7_MISO_eCSPI7_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI7_CLK_eCSPI7_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI7_SS0_eCSPI7_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI7_SS1_eCSPI7_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA3_eCSPI7_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA25_eCSPI7_SS3 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	/* PA4 */
	D4400_PAD_SPI8_MOSI_eCSPI8_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI8_MISO_eCSPI8_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI8_CLK_eCSPI8_CLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI8_SS0_eCSPI8_SS0 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_SPI8_SS1_eCSPI8_SS1 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA2_eCSPI8_SS2 | MUX_PAD_CTRL(SPI_PAD_CTRL),
	D4400_PAD_GPIOA24_eCSPI8_SS3 | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void setup_iomux_spi(void)
{
	d4400_iomux_setup_multiple_pads(spi_pads, ARRAY_SIZE(spi_pads));
}
#endif

#ifdef CONFIG_I2C_MXC

#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)

/* I2C1 */
struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C1_SCL_I2C1_SCL | PC,
		.gpio_mode = D4400_PAD_I2C1_SCL_GPIOE1 | PC,
		.gp = D4400_GPIO_NR(5, 1)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C1_SDA_I2C1_SDA | PC,
		.gpio_mode = D4400_PAD_I2C1_SDA_GPIOE0 | PC,
		.gp = D4400_GPIO_NR(5, 0)
	}
};

/* I2C2 */
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C2_SCL_I2C2_SCL | PC,
		.gpio_mode = D4400_PAD_I2C2_SCL_GPIOE3 | PC,
		.gp = D4400_GPIO_NR(5, 3)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C2_SDA_I2C2_SDA | PC,
		.gpio_mode = D4400_PAD_I2C2_SDA_GPIOE2 | PC,
		.gp = D4400_GPIO_NR(5, 2)
	}
};

/* I2C3 */
struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C3_SCL_I2C3_SCL | PC,
		.gpio_mode = D4400_PAD_I2C3_SCL_GPIOE5 | PC,
		.gp = D4400_GPIO_NR(5, 5)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C3_SDA_I2C3_SDA | PC,
		.gpio_mode = D4400_PAD_I2C3_SDA_GPIOE4 | PC,
		.gp = D4400_GPIO_NR(5, 4)
	}
};

/* I2C5 */
struct i2c_pads_info i2c_pad_info4 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C5_SCL_I2C5_SCL | PC,
		.gpio_mode = D4400_PAD_I2C5_SCL_GPIOD9 | PC,
		.gp = D4400_GPIO_NR(4, 9)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C5_SDA_I2C5_SDA | PC,
		.gpio_mode = D4400_PAD_I2C5_SDA_GPIOE31 | PC,
		.gp = D4400_GPIO_NR(5, 31)
	}
};

/* I2C6 */
struct i2c_pads_info i2c_pad_info5 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C6_SCL_I2C6_SCL | PC,
		.gpio_mode = D4400_PAD_I2C6_SCL_GPIOD11 | PC,
		.gp = D4400_GPIO_NR(4, 11)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C6_SDA_I2C6_SDA | PC,
		.gpio_mode = D4400_PAD_I2C6_SDA_GPIOD10 | PC,
		.gp = D4400_GPIO_NR(4, 10)
	}
};

/* I2C7 */
struct i2c_pads_info i2c_pad_info6 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C7_SCL_I2C7_SCL | PC,
		.gpio_mode = D4400_PAD_I2C7_SCL_GPIOD13 | PC,
		.gp = D4400_GPIO_NR(4, 13)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C7_SDA_I2C7_SDA | PC,
		.gpio_mode = D4400_PAD_I2C7_SDA_GPIOD12 | PC,
		.gp = D4400_GPIO_NR(4, 12)
	}
};

/* I2C8 */
struct i2c_pads_info i2c_pad_info7 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C8_SCL_I2C8_SCL | PC,
		.gpio_mode = D4400_PAD_I2C8_SCL_GPIOD15 | PC,
		.gp = D4400_GPIO_NR(4, 15)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C8_SDA_I2C8_SDA | PC,
		.gpio_mode = D4400_PAD_I2C8_SDA_GPIOD14 | PC,
		.gp = D4400_GPIO_NR(4, 14)
	}
};

/* I2C9 */
struct i2c_pads_info i2c_pad_info8 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C9_SCL_I2C9_SCL | PC,
		.gpio_mode = D4400_PAD_I2C9_SCL_GPIOD17 | PC,
		.gp = D4400_GPIO_NR(4, 17)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C9_SDA_I2C9_SDA | PC,
		.gpio_mode = D4400_PAD_I2C9_SDA_GPIOD16 | PC,
		.gp = D4400_GPIO_NR(4, 16)
	}
};

/* I2C10 */
struct i2c_pads_info i2c_pad_info9 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C10_SCL_I2C10_SCL | PC,
		.gpio_mode = D4400_PAD_I2C10_SCL_GPIOD19 | PC,
		.gp = D4400_GPIO_NR(4, 19)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C10_SDA_I2C10_SDA | PC,
		.gpio_mode = D4400_PAD_I2C10_SDA_GPIOD18 | PC,
		.gp = D4400_GPIO_NR(4, 18)
	}
};

/* I2C11 */
struct i2c_pads_info i2c_pad_info10 = {
	.scl = {
		.i2c_mode = D4400_PAD_I2C11_SCL_I2C11_SCL | PC,
		.gpio_mode = D4400_PAD_I2C11_SCL_GPIOD21 | PC,
		.gp = D4400_GPIO_NR(4, 21)
	},
	.sda = {
		.i2c_mode = D4400_PAD_I2C11_SDA_I2C11_SDA | PC,
		.gpio_mode = D4400_PAD_I2C11_SDA_GPIOD20 | PC,
		.gp = D4400_GPIO_NR(4, 20)
	}
};

struct d4400_i2c_bus {
	char *name;
	struct i2c_pads_info *info;
};

static struct d4400_i2c_bus d4400_i2c_busses[] = {
	{ "I2C1",  &i2c_pad_info0 },
	{ "I2C2",  &i2c_pad_info1 },
	{ "I2C3",  &i2c_pad_info2 },
	{ "I2C4",  NULL },
	{ "I2C5",  &i2c_pad_info4 },
	{ "I2C6",  &i2c_pad_info5 },
	{ "I2C7",  &i2c_pad_info6 },
	{ "I2C8",  &i2c_pad_info7 },
	{ "I2C9",  &i2c_pad_info8 },
	{ "I2C10",  &i2c_pad_info9 },
	{ "I2C11",  &i2c_pad_info10 },
};

static void setup_i2c_busses(void)
{
	int i;
#ifdef CONFIG_DISPLAY_I2C_BUSSES
	int bus = 0;
#endif
	for (i = 0; i < ARRAY_SIZE(d4400_i2c_busses); i++) {
		if (d4400_i2c_busses[i].info) {
#ifdef CONFIG_DISPLAY_I2C_BUSSES
			printf("%s: i2c dev %d\n",
				d4400_i2c_busses[i].name, bus++);
#endif
			setup_i2c(i, CONFIG_SYS_I2C_SPEED, 0x7f,
				d4400_i2c_busses[i].info);
		}
	}
}
#endif

#ifdef CONFIG_OVDD_VSEL
void setup_ovdd_vsel()
{
#if defined(CONFIG_CMD_WEIM_NOR) && defined(CONFIG_QIXIS)
	u8 reg = readb(CONFIG_QIXIS_BASE_ADDR + QIXIS_PWR_CTL2_REG_OFFSET);

	u8 val = reg;
	val >>= QIXIS_GVDDA_REG_OFFSET;
	val &= QIXIS_GVDD_REG_MASK;
	switch (val) {
	case MANUAL_1_8_V_SEL:
		writel(0x00000007, GVDD1_VSEL_REG);
		writel(0x00000007, GVDD8_VSEL_REG);
		writel(0x00000007, GVDD9_VSEL_REG);
		break;
	case MANUAL_2_5_V_SEL:
		writel(0x00000005, GVDD1_VSEL_REG);
		writel(0x00000005, GVDD8_VSEL_REG);
		writel(0x00000005, GVDD9_VSEL_REG);
		break;
	case MANUAL_3_0_V_SEL:
	case MANUAL_3_3_V_SEL:
		writel(0x00000004, GVDD1_VSEL_REG);
		writel(0x00000004, GVDD8_VSEL_REG);
		writel(0x00000004, GVDD9_VSEL_REG);
		break;
	default:
		break;
	}
	val = reg;
	val >>= QIXIS_GVDDB_REG_OFFSET;
	val &= QIXIS_GVDD_REG_MASK;
	switch (val) {
	case MANUAL_1_8_V_SEL:
		writel(0x00000007, GVDD2_VSEL_REG);
		writel(0x00000007, GVDD4_VSEL_REG);
		writel(0x00000007, GVDD7_VSEL_REG);
		break;
	case MANUAL_2_5_V_SEL:
		writel(0x00000005, GVDD2_VSEL_REG);
		writel(0x00000005, GVDD4_VSEL_REG);
		writel(0x00000005, GVDD7_VSEL_REG);
		break;
	case MANUAL_3_0_V_SEL:
	case MANUAL_3_3_V_SEL:
		writel(0x00000004, GVDD2_VSEL_REG);
		writel(0x00000004, GVDD4_VSEL_REG);
		writel(0x00000004, GVDD7_VSEL_REG);
		break;
	default:
		break;
	}
	val = reg;
	val >>= QIXIS_GVDDC_REG_OFFSET;
	val &= QIXIS_GVDD_REG_MASK;
	switch (val) {
	case MANUAL_1_8_V_SEL:
		writel(0x00000007, GVDD3_VSEL_REG);
		writel(0x00000007, GVDD6_VSEL_REG);
		break;
	case MANUAL_2_5_V_SEL:
		writel(0x00000005, GVDD3_VSEL_REG);
		writel(0x00000005, GVDD6_VSEL_REG);
		break;
	case MANUAL_3_0_V_SEL:
	case MANUAL_3_3_V_SEL:
		writel(0x00000004, GVDD3_VSEL_REG);
		writel(0x00000004, GVDD6_VSEL_REG);
		break;
	default:
		break;
	}
#endif

/* CONFIG_JVDD_HW_SEL_DEFAULT - 1.8V */
#ifdef CONFIG_JVDD_HW_SEL_DEFAULT
	writel(0x00000007, FVDD_VSEL_REG);
	writel(0x00000007, JVDD_VSEL_REG);
#else
	writel(0x00000004, FVDD_VSEL_REG);
	writel(0x00000004, JVDD_VSEL_REG);
#endif
	writel(0x00000007, GVDD5_VSEL_REG);
}
#endif

int board_early_init_f(void)
{
#ifdef CONFIG_D4400_UART
	setup_iomux_uart();
#endif
#ifdef CONFIG_TSEC_ENET
	setup_enet();
#endif
#ifdef CONFIG_CMD_WEIM_NOR
	setup_weim_nor();
#ifdef CONFIG_QIXIS
	setup_weim_qixis();
#endif
#endif
#ifdef CONFIG_MXC_SPI
	setup_iomux_spi();
#endif
	return 0;
}

int board_init(void)
{
#if defined(CONFIG_CMD_WEIM_NOR) && defined(CONFIG_QIXIS)
	printf("QIXIS: %02x:%02x - %02x.%02x\n",
		readb(CONFIG_QIXIS_BASE_ADDR + 0),
		readb(CONFIG_QIXIS_BASE_ADDR + 1),
		readb(CONFIG_QIXIS_BASE_ADDR + 2),
		readb(CONFIG_QIXIS_BASE_ADDR + 3));
#endif

#ifdef CONFIG_OVDD_VSEL
	setup_ovdd_vsel();
#endif
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_I2C_MXC
	setup_i2c_busses();
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: D4400-EVB\n");
	return 0;
}
