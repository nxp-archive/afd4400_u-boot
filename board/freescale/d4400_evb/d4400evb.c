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
#include <asm/arch/d4400-regs.h>
#include <asm/arch/d4400-pins.h>
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

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSL_1  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |		\
	PAD_CTL_DSL_1   | PAD_CTL_HYS)

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

#ifdef CONFIG_D4400_UART
iomux_cfg_t uart4_pads[] = {
	D4400_PAD_UART4_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	D4400_PAD_UART4_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
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
	writel(0x01800000, GCR_51_CONFIG_REG);
	/* Memory WDOG enable, External interrupt polarity high */
	writel(0x00000120, WEIM_CONFIGURATION_REG);
	/* CS0 enable, Non Mux, Data port D[0-15], CRE signal Active low */
	writel(0x00110001, WEIM_GENERAL_CONFIGURATION_REG_1);
	/* 2 Cycles of Address hold time  */
	writel(0x00000002, WEIM_GENERAL_CONFIGURATION_REG_2);
	/* Read wait 3 Cycles */
	writel(0x03080000, WEIM_READ_CONFIGURATION_REG_1);
	writel(0x00000000, WEIM_READ_CONFIGURATION_REG_2);
	/* Write wait 3 Cycles, BE assert- 1 Cycle, BE negate - 1 Cycle
	 * WE assert -1 Cycle, WE negate - 1 Cycle
	 */
	writel(0x030c9249, WEIM_WRITE_CONFIGURATION_REG_1);
}
#endif

static void setup_weim_nor(void)
{
	d4400_iomux_setup_multiple_pads(nor_pads, ARRAY_SIZE(nor_pads));
	weim_norflash_cs_setup();
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
	D4400_PAD_TSEC_MDC_GPIO_D09 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	D4400_PAD_TSEC_MDIO_GPIO_D10 | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static void setup_iomux_enet(void)
{
	d4400_iomux_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
}

static void setup_serdes_sgmii_mode(void)
{
	struct serdes_regs *serdes_regs =
		(struct serdes_regs *)(SERDES2_BASE_ADDR);
	int mask;
	int timeout = 1000000;

	/* Soft reset SERDES PLL1*/
	serdes_regs->pll1_rstctl_offs &= ~(SERDES_PLL1_RST_REQ);

	serdes_regs->pll1_rstctl_offs |= SERDES_PLL1_RST_REQ;

	/* RST_DONE should be 1, RST_ERR should be 0 */
	mask = (serdes_regs->pll1_rstctl_offs &
			(SERDES_PLL1_RST_DONE | SERDES_PLL1_RST_ERR));
	while (mask != SERDES_PLL1_RST_DONE && timeout--)
		mask = (serdes_regs->pll1_rstctl_offs &
				(SERDES_PLL1_RST_DONE | SERDES_PLL1_RST_ERR));

	if (0 == timeout)
		printf("SERDES reset failed\n");

}

static void setup_enet(void)
{
	/* Checking BOOT MODE */
	/* If BOOT mode is SGMII setup SERDES */
	if (BOOT_ETH_MODE_SGMII == d4400_get_eth0_mode())
		setup_serdes_sgmii_mode();
	/* If BOOT mode is RGMII setup IOMUX */
	else
		setup_iomux_enet();
}

int board_eth_init(bd_t *bis)
{
	struct fsl_pq_mdio_info mdio_info;
	struct tsec_info_struct tsec_info[2];

	int num = 0;

#ifdef CONFIG_TSEC1
	SET_STD_TSEC_INFO(tsec_info[num], 1);
	num++;
#endif

#ifdef CONFIG_TSEC2
	SET_STD_TSEC_INFO(tsec_info[num], 2);
	num++;
#endif

	if (!num) {
		printf("No TSECs initialized\n");
		return 0;
	}

	mdio_info.regs = (struct tsec_mii_mng *)CONFIG_SYS_MDIO1_BASE_ADDR;
	mdio_info.name = DEFAULT_MII_NAME;

	fsl_pq_mdio_init(bis, &mdio_info);
	if (num != tsec_eth_init(bis, tsec_info, num))
		printf("TSEC : Unable to register initialize TSEC\n");

	return 0;
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
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	return 0;
}

int checkboard(void)
{
	puts("Board: D4400-EVB\n");
	return 0;
}
