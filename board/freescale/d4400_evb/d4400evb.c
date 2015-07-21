/*
 * Copyright (C) 2013-2015 Freescale Semiconductor, Inc.
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
#if defined(CONFIG_OF_LIBFDT)
#include <libfdt.h>
#endif

#include "d4400evb.h"

DECLARE_GLOBAL_DATA_PTR;

#define SILICON_REVISION_1_0		(0x10)
#define SILICON_REVISION_1_1		(0x11)

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

enum FPGA_GVDD_VSEL {
	MANUAL_1_8_V_SEL,
	MANUAL_2_5_V_SEL,
	MANUAL_3_0_V_SEL,
	MANUAL_3_3_V_SEL,
};

#define QIXIS_ID_REG_OFFSET		0x00
#define QIXIS_ID_VALUE		0x25
#define QIXIS_BOARD_REV_REG_OFFSET	0x01
#define QIXIS_BOARD_REV_A	0x11
#define QIXIS_BOARD_REV_B	0x22
#define QIXIS_QIXIS_REV_MAJOR_OFFSET	0x02
#define QIXIS_QIXIS_REV_MINOR_OFFSET	0x03
#define QIXIS_PWR_CTL2_REG_OFFSET	0x21
#define QIXIS_GVDD_REG_MASK	0x3
#define QIXIS_GVDDA_REG_SHIFT	0x6
#define QIXIS_GVDDB_REG_SHIFT	0x4
#define QIXIS_GVDDC_REG_SHIFT	0x2

#define BCSR_GVDDA_SHIFT	0x4
#define BCSR_GVDD5_SHIFT	0x5
#define BCSR_REV_SHIFT		0x6

#define QIXIS_NUM_REG		97
#if defined(CONFIG_CMD_WEIM_NOR) && defined(CONFIG_QIXIS)
	#define fpga_read(offset)	readb(offset)
#elif defined(CONFIG_FSL_D4400_QSPI) && defined(CONFIG_QIXIS)
	u8 fpga_reg[QIXIS_NUM_REG];
	#define fpga_read(offset)	fpga_reg[offset]
#else
	#error Fpga read is not defined!
#endif

#define GCR72			(0x012C013C)
#define GCR75			(0x012C0148)

#define MMDC_MDCTL		(0x01080000)
#define MMDC_MDPDC		(0x01080004)
#define MMDC_MDOTC		(0x01080008)
#define MMDC_MDCFG0		(0x0108000C)
#define MMDC_MDCFG1		(0x01080010)
#define MMDC_MDCFG2		(0x01080014)
#define MMDC_MDMISC		(0x01080018)
#define MMDC_MDSCR		(0x0108001C)
#define MMDC_MDREF		(0x01080020)
#define MMDC_MDRWD		(0x0108002C)
#define MMDC_MDOR		(0x01080030)
#define MMDC_MDASP		(0x01080040)
#define MMDC_MPZQHWCTRL		(0x01080800)
#define MMDC_MPODTCTRL		(0x01080818)
#define MMDC_MPDGCTRL0		(0x0108083C)
#define MMDC_MPDGCTRL1		(0x01080840)
#define MMDC_MPRDDLCTL0		(0x01080848)
#define MMDC_MPWRDLCTL0		(0x01080850)
#define MMDC_MPRDDLHWCTL	(0x01080860)
#define MMDC_MPWRDLHWCTL	(0x01080864)
#define MMDC_MPRDDLHWST0	(0x01080868)
#define MMDC_MPRDDLHWST1	(0x0108086C)
#define MMDC_MPWRDLHWST0	(0x01080870)
#define MMDC_MPWRDLHWST1	(0x01080874)
#define MMDC_MPPDCMPR2		(0x01080890)
#define MMDC_MPMUR		(0x010808B8)
#define OCRAM_ADDR		(0x60000000)

#define ddr_reg32_write(addr, val) { *((volatile u32*)addr) = val; }
#define ddr_reg32_read(addr) ( *((volatile u32*)addr) )

static enum board_type brd_type = BOARD_TYPE_UNKNOWN;
static enum board_rev  brd_rev  = BOARD_REV_UNKNOWN;
static int  board_checked; /* preloaded to 0 */

static const char rdb_rev_letter[4] = {
	'A', 'C', '2', '3'
};

static void get_board_info(void)
{
	if (!board_checked) {
		board_checked = 1;
		brd_type = BOARD_TYPE_UNKNOWN;
		brd_rev  = BOARD_REV_UNKNOWN;
		u8 reg0 = fpga_read(CONFIG_QIXIS_BASE_ADDR +
					QIXIS_ID_REG_OFFSET);
		u8 reg1 = fpga_read(CONFIG_QIXIS_BASE_ADDR +
					QIXIS_BOARD_REV_REG_OFFSET);

		if (reg0 == reg1) {
			brd_type = BOARD_TYPE_D4400RDB;
			brd_rev  = (reg0 >> BCSR_REV_SHIFT) & 3;
		} else if (reg0 == QIXIS_ID_VALUE) {
			brd_type = BOARD_TYPE_D4400EVB;
			switch (reg1) {
			case QIXIS_BOARD_REV_A:
				brd_rev = BOARD_REV_A;
				break;
			case QIXIS_BOARD_REV_B:
				brd_rev = BOARD_REV_B;
				break;
			default:
				brd_rev = BOARD_REV_UNKNOWN;
				break;
			}
		}
	}
}

enum board_type get_board_type(void)
{
	if (!board_checked)
		get_board_info();
	return brd_type;
}

enum board_rev get_board_rev(void)
{
	if (!board_checked)
		get_board_info();
	return brd_rev;
}

// This routine is copied into OCRAM and run there to calibrate DDR
void ddr_calibrate(void)
{
	u32 refresh;
	refresh = ddr_reg32_read(MMDC_MDREF); /* Store current DDR Refresh */

	/* Put DDR into test mode ready for DDR DQS calibration */
	ddr_reg32_write(MMDC_MDSCR, 0x00008000); /* Disable AXI accesses */
	/* wait for Configuration mode */
	while (!(ddr_reg32_read(MMDC_MDSCR) & 0x00004000))
		;

	ddr_reg32_write(MMDC_MDREF, 0x0000C000); /* Disable refresh Cycles */
	ddr_reg32_write(MMDC_MDSCR, 0x00008020); /* Manual Refresh Command */
	ddr_reg32_write(MMDC_MDSCR, 0x04008050); /* Precharge All actv banks */
	ddr_reg32_write(MMDC_MDSCR, 0x00048033); /* MPR command */
	ddr_reg32_write(MMDC_MPPDCMPR2, 0x00000001); /* MPR Configure */

	/* Read DQS Calibration */
	ddr_reg32_write(MMDC_MPDGCTRL0, 0x10000000); /* Start Read DQS Cal */
	/* wait for Read DQS Calibration to complete */
	while ((ddr_reg32_read(MMDC_MPDGCTRL0) & 0x10000000) != 0)
		;

	/* Read Calibration */
	ddr_reg32_write(MMDC_MPRDDLHWCTL, 0x00000010); /* Start Read Cal */
	/* wait for Read Calibration to complete */
	while (ddr_reg32_read(MMDC_MPRDDLHWCTL) != 0x00000000)
		;

	/* Write Calibration */
	ddr_reg32_write(MMDC_MPWRDLHWCTL, 0x00000010); /* Start Write Cal */
	/* wait for Write Calibration to complete */
	while (ddr_reg32_read(MMDC_MPWRDLHWCTL) != 0x00000000)
		;

	/* Restore DDR operational state */
	ddr_reg32_write(MMDC_MDSCR , 0x00008033); /* Exit MPR Mode for DDR */
	ddr_reg32_write(MMDC_MDREF , refresh); /* Setup DDR Refresh */
	ddr_reg32_write(MMDC_MDSCR , 0x00000000); /* DDR ready */
	/* wait for Configuration mode to end */
	while (ddr_reg32_read(MMDC_MDSCR) & 0x00004000)
		;
}


static void dram_cal(void)
{
	u32 *src_ptr;
	u32 *dest_ptr;

	// Copy DDR initialization code into OCRAM
	dest_ptr = (u32*)OCRAM_ADDR;
	src_ptr = (u32*)ddr_calibrate;
	while (src_ptr < ((u32*)dram_cal)) {
		*dest_ptr++ = *src_ptr++;
	}
	// and run it
	((void (*)(void))OCRAM_ADDR)();
}

static int do_ddrcal(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	/* ZQ Calibration - MPZQSWCTRL[ZQ_SW_FOR]
	 * DQS Cal   - MPDGCTRL0[HW_DG_EN]      => MPDGCTRL#[DG_DL_ABS_OFFSET#]
	 * Read Cal  - MPRDDLHWCTL[HW_RD_DL_EN] => MPRDDLCTL[RD_DL_ABS_OFFSET#]
	 * Write Cal - MPWRDLHWCTL0[HW_WR_DL_EN] => MPWRDLCTL[WR_DL_ABS_OFFSET#]
	 */
	u32 m1 = ddr_reg32_read(MMDC_MPDGCTRL0);
	u32 m2 = ddr_reg32_read(MMDC_MPDGCTRL1);
	u32 m3 = ddr_reg32_read(MMDC_MPRDDLCTL0);
	u32 m4 = ddr_reg32_read(MMDC_MPWRDLCTL0);

	dram_cal();

	printf("MPDGCTRL0: 0x%08X => 0x%08X\n", m1, ddr_reg32_read(MMDC_MPDGCTRL0));
	printf("MPDGCTRL1: 0x%08X => 0x%08X\n", m2, ddr_reg32_read(MMDC_MPDGCTRL1));
	printf("MPRDDLCTL: 0x%08X => 0x%08X\n", m3, ddr_reg32_read(MMDC_MPRDDLCTL0));
	printf("MPWRDLCTL: 0x%08X => 0x%08X\n", m4, ddr_reg32_read(MMDC_MPWRDLCTL0));

	return 0;
}

U_BOOT_CMD(
	ddrcal,	1,	1,	do_ddrcal,
	"run DDR calibration",
	""
);

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

static void setup_weim_nor(void)
{
	d4400_iomux_setup_multiple_pads(nor_pads, ARRAY_SIZE(nor_pads));
	weim_norflash_cs_setup();
}
#endif

#if defined(CONFIG_CMD_WEIM_NOR) && defined(CONFIG_QIXIS)
iomux_cfg_t qixis_pads[] = {
	D4400_PAD_FLASH_CS2_B_EIM_CS2_B | MUX_PAD_CTRL(NO_PAD_CTRL)
};

static void setup_weim_qixis(void)
{
	d4400_iomux_setup_multiple_pads(qixis_pads, ARRAY_SIZE(qixis_pads));

	writel(0x000400C1, WEIM_CS2_GENERAL_CONFIGURATION_REG_1);
	writel(0x3F007716, WEIM_CS2_READ_CONFIGURATION_REG_1);
	writel(0x3F026FF6, WEIM_CS2_WRITE_CONFIGURATION_REG_1);
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
		(struct d4400_ccm_reg *)CCM_BASE_ADDR;

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


#ifdef CONFIG_FSL_D4400_QSPI

#define QSPI_PAD_CTRL_OUT   (PAD_CTL_HYS | PAD_CTL_PUE | PAD_CTL_PUS_100K_DOWN | PAD_CTL_DSL_3)
#define QSPI_PAD_CTRL_IO    (PAD_CTL_HYS | PAD_CTL_ODE | PAD_CTL_DSL_3)

iomux_cfg_t qspi_pads[] = {
	D4400_PAD_FLASH_A21_QSPI_CK     | MUX_PAD_CTRL(QSPI_PAD_CTRL_OUT),
	D4400_PAD_FLASH_A22_QSPI_IO_0   | MUX_PAD_CTRL(QSPI_PAD_CTRL_OUT),
	D4400_PAD_FLASH_A23_QSPI_IO_1   | MUX_PAD_CTRL(QSPI_PAD_CTRL_IO),
	D4400_PAD_FLASH_A24_QSPI_IO_2   | MUX_PAD_CTRL(QSPI_PAD_CTRL_IO),
	D4400_PAD_FLASH_A25_QSPI_IO_3   | MUX_PAD_CTRL(QSPI_PAD_CTRL_IO),
	D4400_PAD_FLASH_CS2_B_QSPI_CS_B | MUX_PAD_CTRL(QSPI_PAD_CTRL_OUT)
};

void setup_qspi(void)
{
	/* Setup pins. */
	d4400_iomux_setup_multiple_pads(qspi_pads, ARRAY_SIZE(qspi_pads));

	struct d4400_ccm_reg *ccm_regs =
		(struct d4400_ccm_reg *)CCM_BASE_ADDR;
	volatile struct qspi_regs *qspi_reg =
		(volatile struct qspi_regs *)QSPI_BASE_ADDR;

	/* Configure QSPI clock to default */
	unsigned int ccsr = ccm_regs->ccsr;
	ccsr &= ~D4400_CCM_CCSR_ACC_QSPI_MASK;
	ccm_regs->ccsr = ccsr;

	/* Configure QSPI default clocks to be enabled */
	unsigned int ccgcr1 = ccm_regs->ccgcr1;
	ccgcr1 |= (D4400_CCM_CCGCR1_SFIF_EN_MASK |
		   D4400_CCM_CCGCR1_SFIF_B_EN_MASK |
		   D4400_CCM_CCGCR1_SFPAD_FA_EN_MASK);
	ccm_regs->ccgcr1 = ccgcr1;

	/* Configure mode and clock */
	volatile u32 val32;

	/* Sampling */
	qspi_reg->smpr = 0;

	/* Set latency which is important because the Boot Rom code changes
	 * the latency setting according to its usage when booting from qspi
	 * flash and that setting may be incompatible with the way Uboot uses
	 * qspi. More specifically, Boot Rom uses single i/o AMBA bus qspi
	 * access whereas Uboot driver uses quad i/o IP qspi access.  These
	 * two methods have different latency settings.
	 */
	qspi_reg->lcr = 0x08;	/* Default 0x08 */

	/* RX buf readout: IP bus (RBDR0 to 31 reg) */
	qspi_reg->rbct = (1 << 8);

	/* AMBA control reg (for AHB commands) */
	qspi_reg->acr = 0x803;  /* Default = 0x803 */

	val32 = qspi_reg->mcr;

	val32 &= ~QSPI_MCR_SCLKCFG_MASK;		/* 0x00 = div1 */
	val32 |= (0x0E << QSPI_MCR_SCLKCFG_OFFSET);	/* 0x0E = ~20MHz */
	val32 |= (QSPI_MCR_CLR_RXF_MASK | QSPI_MCR_CLR_TXF_MASK); /* Clr bufs */

	/* Configure flash type */
	val32 &= ~QSPI_MCR_VMID_MASK;

	if (QUERY_QSPI_FLASH_TYPE() == QSPI_FLASH_TYPE_SPANSION)
		val32 |= (2 << 3);	/* Spansion */
	else if (QUERY_QSPI_FLASH_TYPE() == QSPI_FLASH_TYPE_MACRONIX)
		val32 |= (3 << 3);	/* Macronix */
	else if (QUERY_QSPI_FLASH_TYPE() == QSPI_FLASH_TYPE_NUMONYX)
		val32 |= (4 << 3);	/* Numonyx */
	else /* (QUERY_QSPI_FLASH_TYPE() == QSPI_FLASH_TYPE_WINBOND) */
		val32 |= (1 << 3);	/* Winbond */

	/* Drive IO[3:2] high (default) during idle */
	val32 |= (QSPI_MCR_ISD2FA_MASK | QSPI_MCR_ISD3FA_MASK);

	/* Before new settings, set MDIS = 1 to allow change. */
	qspi_reg->mcr |= QSPI_MCR_MDIS_MASK;

	/* Make new config effective */
	qspi_reg->mcr = val32;

	/* Once clock is set in MCR, set MDIS = 0 to disallow clk disable. */
	qspi_reg->mcr &= ~QSPI_MCR_MDIS_MASK;

	/* Turn off all interrupts */
	qspi_reg->rser = 0;

	/* Clear pending irqs */
	qspi_reg->fr = 0xffffffff;

	/* Set 24-bit addressing mode as default.  Qspi driver sets 32-bit
	 * addressing if required.  See drivers/spi/mxc_spi.c.
	 */
	QSPI_SET_ADDR_MODE(QSPI_ADDR_MODE_24BIT);

	/* Note that qspi quad I/O mode is enabled/disabled when the
	 * spi flash device is probed.  See spi_flash.c/spi_flash_probe().
	 */
}

void qspi_test(void)
{
	/* Toggle IO[3:2] signals */

	volatile u32 val32;
	volatile u32 *mcr = (volatile u32 *)QSPI_MCR_REG;

	val32 = readl(QSPI_MCR_REG);
	val32 |= (1 << 14);	/* MDIS = 1 disable mode */
	val32 |= (0x2 << 24);	/* SCLKCFG 0x0E = div8   */
	val32 |= (2 << 3);	/* 2-Spansion            */
	writel(val32, QSPI_MCR_REG);

	while(1) {
		/* In order to modify b[19:16] to set IOFA/B[2:3] signals
		 * hi/low these steps must be followed:
		 *
		 * 1) Set MCR[14] = 1  (MDIS bit)
		 * 2) Set ISD2FA, ISD3FA, ISD2FB, ISD3FB bits, MCR[19:16].
		 * 3) Set MCR[14] = 0 to have the drive values take effect.
		 *
		 * In order to change MCR[19:16] again, you must repeat the
		 * the steps above.
		 *
		 * Note in AFD4400 that only ISD2/3FA signals are implemented
		 * (one qspi module implemented) and thus only MCR[17:16] are
		 * used. MCR[19:18] are don't cares.
		 *
		 */
		/* MDIS = 1 disable mode, allows MCR[17:16] to be modified */
		*mcr |= (1<<14);
		*mcr &= ~((1<<17) | (1<<16)); /* Now clear IOFA bits   */
		*mcr &= ~(1<<14); /* MDIS = 0, MCR[17:16] takes effect */

		val32 &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16));
		val32 &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16));
		val32 &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16));

		/* MDIS = 1 disable mode, allows MCR[17:16] to be modified */
		*mcr |= (1<<14);
		*mcr |= ((1<<17) | (1<<16)); /* Now set IOFA bits      */
		*mcr &= ~(1<<14); /* MDIS = 0, MCR[17:16] takes effect */

		val32 &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16));
		val32 &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16));
		val32 &= ~((1<<19) | (1<<18) | (1<<17) | (1<<16));
	}
}
#endif /* CONFIG_FSL_D4400_QSPI */

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
static void set_ovdd_vsel(u8 val, u32 reg)
{
	switch (val) {
	case MANUAL_1_8_V_SEL:
		writel(0x00000007, reg);
		break;
	case MANUAL_2_5_V_SEL:
		writel(0x00000005, reg);
		break;
	case MANUAL_3_0_V_SEL:
	case MANUAL_3_3_V_SEL:
		writel(0x00000004, reg);
		break;
	default:
		break;
	}
}

static void setup_ovdd_vsel(void)
{
	u8 gvdd1, gvdd2, gvdd3, gvdd4, gvdd5, gvdd6, gvdd7, gvdd8, gvdd9;
	u8 gvdda, gvddb, gvddc;
	u8 val;
	u8 update = 1;
	switch (get_board_type()) {
	case BOARD_TYPE_D4400EVB:
		val = fpga_read(CONFIG_QIXIS_BASE_ADDR + QIXIS_PWR_CTL2_REG_OFFSET);
		gvdda = (val >> QIXIS_GVDDA_REG_SHIFT) & QIXIS_GVDD_REG_MASK;
		gvddb = (val >> QIXIS_GVDDB_REG_SHIFT) & QIXIS_GVDD_REG_MASK;
		gvddc = (val >> QIXIS_GVDDC_REG_SHIFT) & QIXIS_GVDD_REG_MASK;
		gvdd1 = gvdda;
		gvdd2 = gvddb;
		gvdd3 = gvddc;
		gvdd4 = gvddb;
		gvdd5 = MANUAL_3_3_V_SEL;
		gvdd6 = gvddc;
		gvdd7 = gvddb;
		gvdd8 = gvdda;
		gvdd9 = gvdda;
		break;
	case BOARD_TYPE_D4400RDB:
		val = fpga_read(CONFIG_QIXIS_BASE_ADDR);
		gvdda = ((val >> BCSR_GVDDA_SHIFT) & 1) ?
				MANUAL_3_3_V_SEL : MANUAL_1_8_V_SEL;
		gvddb = ((val >> BCSR_GVDD5_SHIFT) & 1) ?
				MANUAL_3_3_V_SEL : MANUAL_1_8_V_SEL;
		gvdd1 = gvdda;
		gvdd2 = MANUAL_3_3_V_SEL;
		gvdd3 = MANUAL_1_8_V_SEL;
		gvdd4 = MANUAL_3_3_V_SEL;
		gvdd5 = gvddb;
		gvdd6 = MANUAL_1_8_V_SEL;
		gvdd7 = MANUAL_3_3_V_SEL;
		gvdd8 = gvdda;
		gvdd9 = gvdda;
		break;
	default:
		update = 0;
		break;
	}
	if (update) {
		set_ovdd_vsel(gvdd1, GVDD1_VSEL_REG);
		set_ovdd_vsel(gvdd2, GVDD2_VSEL_REG);
		set_ovdd_vsel(gvdd3, GVDD3_VSEL_REG);
		set_ovdd_vsel(gvdd4, GVDD4_VSEL_REG);
		set_ovdd_vsel(gvdd5, GVDD5_VSEL_REG);
		set_ovdd_vsel(gvdd6, GVDD6_VSEL_REG);
		set_ovdd_vsel(gvdd7, GVDD7_VSEL_REG);
		set_ovdd_vsel(gvdd8, GVDD8_VSEL_REG);
		set_ovdd_vsel(gvdd9, GVDD9_VSEL_REG);
	}
	/* DEFAULT_JFVDD - 3.3V */
	set_ovdd_vsel(MANUAL_3_3_V_SEL, FVDD_VSEL_REG);
	set_ovdd_vsel(MANUAL_3_3_V_SEL, JVDD_VSEL_REG);
}
#endif

#if defined(CONFIG_FSL_D4400_QSPI) && defined(CONFIG_QIXIS)
void read_fpga_i2c_reg(u8 *buf, int num)
{
	int old_i2c_dev;
	int old_i2c_speed;

	old_i2c_dev = i2c_get_bus_num();
	old_i2c_speed = i2c_get_bus_speed();

	i2c_set_bus_num(CONFIG_QIXIS_FPGA_I2C_BUS_NUM);
	i2c_set_bus_speed(400000);

	i2c_read(CONFIG_QIXIS_FPGA_I2C_ADDR, 0, 1, buf, num);

	i2c_set_bus_num(old_i2c_dev);
	i2c_set_bus_speed(old_i2c_speed);
}
#endif

static void configure_ina220(void)
{
	int old_i2c_dev;
	int old_i2c_speed;
	uchar buf[2];

	old_i2c_dev = i2c_get_bus_num();
	old_i2c_speed = i2c_get_bus_speed();

	i2c_set_bus_num(CONFIG_BRD_MON_I2C_BUS_NUM);
	i2c_set_bus_speed(400000);

	buf[0] = 0x04; /* FS Bus V = 12V, FS Shunt V = 40 mV */
	buf[1] = 0xCF; /* 2 samples per voltage conversion */
	i2c_write(CONFIG_INA220_I2C_ADDR, 0, 1, buf, 2); /* Set configuration */
	buf[0] = 0xA0; /* Shunt resistor is 1 mOhm */
	buf[1] = 0x00; /* current = 1 mA/LSB, pwr = 20 mW/LSB */
	i2c_write(CONFIG_INA220_I2C_ADDR, 5, 1, buf, 2); /* Set calibration */

	i2c_set_bus_num(old_i2c_dev);
	i2c_set_bus_speed(old_i2c_speed);
}

int board_early_init_f(void)
{
#ifdef CONFIG_D4400_UART
	setup_iomux_uart();
#endif
#ifdef CONFIG_TSEC_ENET
	setup_enet();
#endif
#ifdef CONFIG_CMD_WEIM_NOR
	if (QUERY_FLASH_BOOT_MEM_TYPE() == FLASH_BOOT_MEM_TYPE_NOR)
		setup_weim_nor();
#ifdef CONFIG_QIXIS
	if (QUERY_FLASH_BOOT_MEM_TYPE() == FLASH_BOOT_MEM_TYPE_NOR)
		setup_weim_qixis();
#endif
#endif
#ifdef CONFIG_MXC_SPI
	setup_iomux_spi();
#endif

#ifdef CONFIG_FSL_D4400_QSPI
	/* Qspi setup. */
	if (QUERY_FLASH_BOOT_MEM_TYPE() == FLASH_BOOT_MEM_TYPE_QSPI)
		setup_qspi();
#endif
	return 0;
}

int configure_vid(void);
int board_init(void)
{
	enum board_rev rev;
	enum board_type type;
	char rev_letter = '?';

	if (SILICON_REVISION_1_0 == get_silicon_rev()) {
		printf("\n\n\tDFE silicon version 1.0 is no longer supported.\n");
		printf("\tPlease upgrade DFE device.\n\n");
		while (1) {}
	}

#ifdef CONFIG_I2C_MXC
	setup_i2c_busses();
#endif

#if defined(CONFIG_FSL_D4400_QSPI) && defined(CONFIG_QIXIS)
	/* As soon as i2c bus is configured, read all of the
	 * fpga registers for later use.
	 */
	memset(fpga_reg, 0, QIXIS_NUM_REG);
	read_fpga_i2c_reg(fpga_reg, QIXIS_NUM_REG);
#endif

	rev = get_board_rev();
	type = get_board_type();

	if (BOARD_REV_UNKNOWN != rev)
		rev_letter = (type == BOARD_TYPE_D4400RDB) ?
					rdb_rev_letter[rev] : 'A' + rev;

	switch (get_board_type()) {
	case BOARD_TYPE_D4400EVB:
		printf("Board: D4400-EVB, Rev %c\n", rev_letter);
		break;
	case BOARD_TYPE_D4400RDB:
		printf("Board: D4400-RDB, Rev %c\n", rev_letter);
		break;
	default:
		printf("Board: D4400-???\n");
		break;
	}

	{
		if (BOARD_TYPE_D4400EVB == get_board_type()) {
			printf("QIXIS: %02x:%02x - %02x.%02x\n",
				fpga_read(CONFIG_QIXIS_BASE_ADDR +
					QIXIS_ID_REG_OFFSET),
				fpga_read(CONFIG_QIXIS_BASE_ADDR +
					QIXIS_BOARD_REV_REG_OFFSET),
				fpga_read(CONFIG_QIXIS_BASE_ADDR +
					QIXIS_QIXIS_REV_MAJOR_OFFSET),
				fpga_read(CONFIG_QIXIS_BASE_ADDR +
					QIXIS_QIXIS_REV_MINOR_OFFSET));
		}
	}

	/* Setup GCR signal termination registers */
	if (SILICON_REVISION_1_0 == get_silicon_rev()) {
		writel(0x00140000, GCR72);
	} else {
#ifndef CONFIG_D4400_EVB_MDC_BUG_SW_FIX
		writel(0x00100000, GCR72); /* Enable MDC open drain */
#else
		writel(0x00000000, GCR72); /* Disable MDC open drain */
#endif
	}

	/* EVB Rev A boards don't have SYSREF_IN and RGMII_REFCLK resistors */
	if (BOARD_TYPE_D4400EVB == get_board_type() &&
	    BOARD_REV_A == get_board_rev()) {
		writel(0x000FFE80, GCR75);
	} else {
		writel(0x000FF880, GCR75);
	}

#ifdef CONFIG_OVDD_VSEL
/*TODO - only do ovdd sel on R1.0 silicon - waiting for confirmation
	if (SILICON_REVISION_1_0 == get_silicon_rev()) {
*/
	if (QUERY_FLASH_BOOT_MEM_TYPE() == FLASH_BOOT_MEM_TYPE_NOR)
		setup_ovdd_vsel();
#endif

	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_VID
	configure_vid();
#endif

	if (BOARD_TYPE_D4400RDB == get_board_type())
		configure_ina220();

	return 0;
}

int checkboard(void)
{
	/* Board printout moved to board init so it can check the type/rev */
	return 0;
}

#if defined(CONFIG_OF_BOARD_SETUP)
void ft_board_setup(void *blob, bd_t *bd)
{
	ft_cpu_setup(blob, bd);
}
#endif

int D4400_QueryBootFlashTypeNOR(void)
{
	if (QUERY_FLASH_BOOT_MEM_TYPE() == FLASH_BOOT_MEM_TYPE_NOR)
		return 1;
	else
		return 0;
}
